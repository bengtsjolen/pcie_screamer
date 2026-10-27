#!/usr/bin/env python3
# PCIe Screamer Squirrel - LiteX/Migen target
# Based on enjoy-digital/pcie_screamer by Florent Kermarrec
# Adapted for Screamer PCIe Squirrel (XC7A35T-FGG484)
#
# Usage:
#   python3 pcileech_squirrel.py --build
#   python3 pcileech_squirrel.py --build --load         # JTAG
#   python3 pcileech_squirrel.py --build --flash        # SPI flash
#   python3 pcileech_squirrel.py --build --with-analyzer

import os
import argparse

from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.build.generic_platform import *
from litex.soc.cores.clock import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *
from litex.soc.interconnect import stream

from litepcie.phy.s7pciephy import S7PCIEPHY
from litepcie.common import phy_layout

from gateware.ft601 import FT601Sync
from gateware.msi import MSI
from gateware.pcileech_fifo import PCILeechFIFO

from litescope import LiteScopeAnalyzer

from platforms.pcie_squirrel import Platform



class TLP32To64Packer(Module):
    def __init__(self, swap_words=False):
        self.sink   = sink   = stream.Endpoint(phy_layout(32))
        self.source = source = stream.Endpoint(phy_layout(64))

        # Stored first DWORD of a pair.
        first_dat = Signal(32)
        first_be  = Signal(4)

        self.submodules.fsm = fsm = FSM(reset_state="IDLE")

        # Default: nothing happening.
        self.comb += [
            source.valid.eq(0),
            source.last .eq(0),
            source.dat  .eq(0),
            source.be   .eq(0),
            sink.ready  .eq(0),
        ]

        # ---- IDLE: waiting for first DWORD of a pair (or a lone last) ----
        fsm.act("IDLE",
            If(sink.valid & sink.last,
                # Single-DWORD packet: emit immediately as partial 64-bit beat.
                # CRITICAL: only consume when output can accept.
                source.valid.eq(1),
                source.last .eq(1),
                If(swap_words,
                    source.dat.eq(Cat(Constant(0, 32), sink.dat)),
                    source.be .eq(Cat(Constant(0x0, 4), sink.be)),
                ).Else(
                    source.dat.eq(Cat(sink.dat, Constant(0, 32))),
                    source.be .eq(Cat(sink.be,  Constant(0x0, 4))),
                ),
                sink.ready.eq(source.ready),   # ← FIX: was unconditional 1
            ).Elif(sink.valid,
                # First of a pair: latch and move on.  No output yet,
                # so we can always accept.
                sink.ready.eq(1),
                NextValue(first_dat, sink.dat),
                NextValue(first_be,  sink.be),
                NextState("HAVE_FIRST"),
            ).Else(
                # No data — stay idle, ready to accept.
                sink.ready.eq(1),
            )
        )

        # ---- HAVE_FIRST: waiting for second DWORD to form a 64-bit pair ----
        fsm.act("HAVE_FIRST",
            If(sink.valid,
                # Pair ready: emit both DWORDs as one 64-bit beat.
                source.valid.eq(1),
                source.last .eq(sink.last),
                If(swap_words,
                    source.dat.eq(Cat(sink.dat, first_dat)),
                    source.be .eq(Cat(sink.be,  first_be)),
                ).Else(
                    source.dat.eq(Cat(first_dat, sink.dat)),
                    source.be .eq(Cat(first_be,  sink.be)),
                ),
                # CRITICAL: only consume when output can accept.
                sink.ready.eq(source.ready),   # ← FIX: was unconditional 1
                If(source.ready,
                    NextState("IDLE"),
                )
            )
            # If no data: sink.ready stays 0 (default), we wait.
            # (Could set sink.ready=1 here too — doesn't matter since
            #  sink.valid=0 means no handshake either way.)
        )

# CRG ----------------------------------------------------------------------------------------------

class _CRG(Module):
    def __init__(self, platform, sys_clk_freq):
        self.clock_domains.cd_sys = ClockDomain()
        self.clock_domains.cd_usb = ClockDomain()

        # sys clock: 100MHz oscillator -> PLL -> sys
        sys_clk_100 = platform.request("clk100")
        platform.add_period_constraint(sys_clk_100, 1e9/100e6)
        self.submodules.pll = pll = S7PLL(speedgrade=-2)
        pll.register_clkin(sys_clk_100, 100e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq)

        # usb clock: from FT601 chip
        usb_clk100 = platform.request("usb_fifo_clock")
        platform.add_period_constraint(usb_clk100, 1e9/100e6)
        self.comb += self.cd_usb.clk.eq(usb_clk100)
        self.comb += self.cd_usb.rst.eq(0)


# PCIeSquirrel -------------------------------------------------------------------------------------

class PCIeSquirrel(SoCMini):

    def __init__(self, platform, with_analyzer=False, with_loopback=False,
                 mux_out_fifo_depth=2048, tlp_rx_fifo_depth=2048,
                 ft601_write_fifo_depth=8192, ft601_read_fifo_depth=128):
        sys_clk_freq = int(100e6)

        SoCMini.__init__(self, platform, sys_clk_freq,
                         ident="PCIe Screamer Squirrel", ident_version=True)

        # CRG --------------------------------------------------------------------------------------
        self.submodules.crg = _CRG(platform, sys_clk_freq)

        # PCIe PHY ---------------------------------------------------------------------------------
        pcie_phy = self.submodules.pcie_phy = S7PCIEPHY(platform, platform.request("pcie_x1"),
                                                        data_width=64, bar0_size=0x40000
                                                        #, cd="pcie"
                                                        )
        pcie_phy.config["Device_ID"] = "0666"
        pcie_phy.config["Class_Code_Base"] = "02"
        pcie_phy.config["Class_Code_Sub"] = "00"

        # Advertise 8-bit (256) tag support — matches ufrisk's PCIeSquirrel
        # gateware.  Not strictly required for the current MRRS=4096/MPS=256
        # pattern (only 10 tags in flight for a 10-page dump) but harmless and
        # matches the reference design.
        pcie_phy.config["Extended_Tag_Field"]   = "true"
        pcie_phy.config["Extended_Tag_Default"] = "true"

        # NOTE: Cpl_Finite=true was tried — Vivado refuses to set it while
        #   Buf_Opt_BMA=true:
        #     [IP_Flow 19-3374] An attempt to modify the value of disabled
        #     parameter 'Cpl_Finite' from 'false' to 'true' has been ignored.
        # ufrisk's reference design also uses Cpl_Finite=false with
        # Buf_Opt_BMA=true, so this is NOT the difference between their
        # design (works) and ours (drops 9 CplDs on 10-page dump).
        # Leaving Cpl_Finite at the default (false) deliberately.

        # Force GTP placement to X0Y2 (Vivado defaults to X0Y3 for this package)
        platform.toolchain.pre_optimize_commands.append(
            "set_property LOC GTPE2_CHANNEL_X0Y2 [get_cells "
            "{{pcie_s7/inst/inst/gt_top_i/pipe_wrapper_i/pipe_lane[0]"
            ".gt_wrapper_i/gtp_channel.gtpe2_channel_i}}]"
        )

        platform.add_platform_command("set_false_path -from [get_clocks main_s7pciephy_clkout*] -to [get_clocks main_clkout]")
        platform.add_platform_command("set_false_path -from [get_clocks main_clkout] -to [get_clocks main_s7pciephy_clkout*]")
        platform.add_platform_command("set_false_path -from [get_clocks -of_objects [get_pins pcie_s7/inst/inst/pcie_top_i/pcie_7x_i/pcie_block_i/PIPECLK]] -to [get_clocks main_clkout]")

        self.add_csr("pcie_phy")

        # USB FT601 PHY ----------------------------------------------------------------------------
        self.submodules.usb_phy = FT601Sync(
            platform.request("usb_fifo"), dw=32, timeout=256,
            read_fifo_depth=ft601_read_fifo_depth,
            write_fifo_depth=ft601_write_fifo_depth,
        )

        # USB Loopback (debug) ---------------------------------------------------------------------
        if with_loopback:
            self.submodules.usb_loopback_fifo = stream.SyncFIFO(
                [("data", 32)], 2048)
            self.comb += [
                self.usb_phy.source.connect(self.usb_loopback_fifo.sink),
                self.usb_loopback_fifo.source.connect(self.usb_phy.sink),
            ]

        # PCILeech FIFO ----------------------------------------------------------------------------
        else:
            self.submodules.pcileech_fifo = pcileech_fifo = PCILeechFIFO(
                mux_out_fifo_depth=mux_out_fifo_depth,
                tlp_rx_fifo_depth=tlp_rx_fifo_depth,
            )

            # USB PHY <--> PCILeechFIFO
            # usb_phy.source = words from FT601 (host→FPGA)
            # usb_phy.sink   = words to   FT601 (FPGA→host)
            self.comb += [
                self.usb_phy.source.connect(pcileech_fifo.usb_rx),
                pcileech_fifo.usb_tx.connect(self.usb_phy.sink),
            ]

            # PCIe PHY <--> PCILeechFIFO
            #
            # RX: pcie_phy.source (64-bit) goes directly to PCILeechFIFO,
            # which packs it to 128-bit and splits it into four parallel
            # 32-bit mux ports (ufrisk-style RX path).
            # TX: pcileech_fifo.tlp_tx (32-bit) -> TLP32To64Packer -> PHY.
            self.submodules.tlp_tx_conv = tlp_tx_conv = TLP32To64Packer(
                swap_words=False
            )

            self.comb += [
                self.pcie_phy.source.connect(pcileech_fifo.tlp_rx),
                pcileech_fifo.tlp_tx.connect(tlp_tx_conv.sink),
                tlp_tx_conv.source.connect(self.pcie_phy.sink),
            ]

            # PCIe reset driven by CMD register file.
            # rw[200] starts at 1 (core held in reset at startup).
            # Host clears it via CMD write to bring PCIe core online.
            # Hold PCIe core in reset until host clears rw[200] via CMD write.
            # pcie_phy.pcie_rst_n feeds i_sys_rst_n on the IP — active low.

            # FIXME: without this we cannot reset ip ?!?
            self.comb += self.pcie_phy.pcie_rst_n.eq(~pcileech_fifo.pcie_rst_core)
            
            # Wire PCIe PHY status signals for PCIE register space responses
            self.comb += [
                pcileech_fifo.phy_lnk_up   .eq(self.pcie_phy._link_status.fields.status),
                pcileech_fifo.phy_ltssm    .eq(self.pcie_phy._link_status.fields.ltssm),
                pcileech_fifo.phy_lnk_rate .eq(self.pcie_phy._link_status.fields.rate),
                pcileech_fifo.phy_lnk_width.eq(self.pcie_phy._link_status.fields.width),
                pcileech_fifo.phy_id       .eq(self.pcie_phy.id),
                pcileech_fifo.cfg_dcommand .eq(self.pcie_phy.dcommand),
            ]
            self.comb += [
                pcileech_fifo.tx_axis_seen.eq(self.pcie_phy.s_axis_tx_tvalid & self.pcie_phy.s_axis_tx_tready),
                pcileech_fifo.rx_axis_seen.eq(self.pcie_phy.m_axis_rx_tvalid),
                pcileech_fifo.tx_err_drop.eq(self.pcie_phy.tx_err_drop),
            ]
            # Auto-set BME+MemEn once link is up
            # Replace the combinational wr_en with a registered single-cycle pulse
            bme_done = Signal()
            bme_timer = Signal(20)
            bme_pulse = Signal()
            self.sync.pcie += [
                bme_pulse.eq(0),  # default off
                If(~bme_done,
                   If(self.pcie_phy._link_status.fields.status,
                      If(bme_timer < (1<<20)-1,
                         bme_timer.eq(bme_timer + 1)
                         ).Elif(~bme_pulse,  # only pulse once
                                bme_pulse.eq(1),
                                bme_done.eq(1)
                                )
                      )
                   ),
            ]
            self.comb += [
                self.pcie_phy.cfg_mgmt_dwaddr.eq(1),
                self.pcie_phy.cfg_mgmt_di.eq(0x00000006),
                self.pcie_phy.cfg_mgmt_byte_en.eq(0x3),
                self.pcie_phy.cfg_mgmt_wr_en.eq(bme_pulse),
            ]

        # MSI --------------------------------------------------------------------------------------
        self.submodules.msi = MSI()
        self.comb += self.msi.source.connect(self.pcie_phy.msi)
        self.add_csr("msi")

        # Analyzer ---------------------------------------------------------------------------------
        if with_analyzer:
            analyzer_signals = [
                #self.pcie_phy.sink,
                #self.pcie_phy.source,

                self.pcie_phy.s_axis_tx_tvalid, 
                self.pcie_phy.s_axis_tx_tready,
                self.pcie_phy.s_axis_tx_tdata,
                self.pcie_phy.s_axis_tx_tlast,
                self.pcie_phy.m_axis_rx_tvalid,
                self.pcie_phy.m_axis_rx_tdata,
                self.pcie_phy.m_axis_rx_tlast,
                self.pcie_phy._link_status.fields.status,
                self.pcie_phy._link_status.fields.ltssm,
                
            ]
            self.submodules.analyzer = LiteScopeAnalyzer(
                analyzer_signals, 1024, csr_csv="test/analyzer.csv")
            self.add_csr("analyzer")

# Build --------------------------------------------------------------------------------------------

FIFO_PRESETS = {
    "large": {
        "mux_fifo_depth":           2048,
        "tlp_rx_fifo_depth":        2048,
        "ft601_write_fifo_depth":   8192,
        "ft601_read_fifo_depth":    128,
    },
    "medium": {
        "mux_fifo_depth":           1024,
        "tlp_rx_fifo_depth":        2048,
        "ft601_write_fifo_depth":   4096,
        "ft601_read_fifo_depth":    128,
    },
    "mini": {
        "mux_fifo_depth":           1024,
        "tlp_rx_fifo_depth":        2048,
        "ft601_write_fifo_depth":   1024,
        "ft601_read_fifo_depth":    128,
    },
}


def main():
    parser = argparse.ArgumentParser(description="PCIe Screamer Squirrel Gateware")
    parser.add_argument("--with-analyzer", action="store_true", help="Enable LiteScope analyzer")
    parser.add_argument("--with-loopback", action="store_true", help="Enable USB loopback")
    parser.add_argument("--build",         action="store_true", help="Build bitstream")
    parser.add_argument("--load",          action="store_true", help="Load via JTAG")
    parser.add_argument("--flash",         action="store_true", help="Flash to SPI flash")
    parser.add_argument("--fifo-preset", choices=sorted(FIFO_PRESETS), default="large",
                        help="FIFO size preset (default: large)")
    parser.add_argument("--mux-fifo-depth", type=lambda x: int(x, 0), default=None,
                        help="override mux_out_fifo depth in 256-bit frames")
    parser.add_argument("--tlp-rx-fifo-depth", type=lambda x: int(x, 0), default=None,
                        help="override 128-bit TLP RX FIFO depth")
    parser.add_argument("--ft601-write-fifo-depth", type=lambda x: int(x, 0), default=None,
                        help="override FT601 sys->usb write FIFO depth in 32-bit words")
    parser.add_argument("--ft601-read-fifo-depth", type=lambda x: int(x, 0), default=None,
                        help="override FT601 usb->sys read FIFO depth in 32-bit words")
    args = parser.parse_args()

    fifo = dict(FIFO_PRESETS[args.fifo_preset])
    for key in fifo:
        override = getattr(args, key)
        if override is not None:
            fifo[key] = override

    platform = Platform()
    soc      = PCIeSquirrel(platform,
                            with_analyzer=args.with_analyzer,
                            with_loopback=args.with_loopback,
                            mux_out_fifo_depth=fifo["mux_fifo_depth"],
                            tlp_rx_fifo_depth=fifo["tlp_rx_fifo_depth"],
                            ft601_write_fifo_depth=fifo["ft601_write_fifo_depth"],
                            ft601_read_fifo_depth=fifo["ft601_read_fifo_depth"])
    builder  = Builder(soc, csr_csv="test/csr.csv")
    builder.build(run=args.build)

    if args.load:
        prog = platform.create_programmer()
        prog.load_bitstream(os.path.join(builder.gateware_dir, soc.build_name + ".bit"))

    if args.flash:
        prog = platform.create_programmer()
        prog.flash(0, os.path.join(builder.gateware_dir, soc.build_name + ".bin"))

if __name__ == "__main__":
    main()
