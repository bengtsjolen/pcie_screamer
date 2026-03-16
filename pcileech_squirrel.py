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
from litex.soc.interconnect.stream import StrideConverter

from litepcie.phy.s7pciephy import S7PCIEPHY
from litepcie.common import phy_layout

from gateware.ft601 import FT601Sync
from gateware.msi import MSI
from gateware.pcileech_fifo import PCILeechFIFO

from litescope import LiteScopeAnalyzer

from platforms.pcie_squirrel import Platform

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

    def __init__(self, platform, with_analyzer=False, with_loopback=False):
        sys_clk_freq = int(100e6)

        SoCMini.__init__(self, platform, sys_clk_freq,
                         ident="PCIe Screamer Squirrel", ident_version=True)

        # CRG --------------------------------------------------------------------------------------
        self.submodules.crg = _CRG(platform, sys_clk_freq)

        # PCIe PHY ---------------------------------------------------------------------------------
        pcie_phy = self.submodules.pcie_phy = S7PCIEPHY(platform, platform.request("pcie_x1"),
                                             data_width=64, bar0_size=0x40000)
        pcie_phy.config["Device_ID"] = "0666"
        pcie_phy.config["Class_Code_Base"] = "02"
        pcie_phy.config["Class_Code_Sub"] = "00"

        # Force GTP placement to X0Y2 (Vivado defaults to X0Y3 for this package)
        platform.toolchain.pre_optimize_commands.append(
            "set_property LOC GTPE2_CHANNEL_X0Y2 [get_cells "
            "{{pcie_s7/inst/inst/gt_top_i/pipe_wrapper_i/pipe_lane[0]"
            ".gt_wrapper_i/gtp_channel.gtpe2_channel_i}}]"
        )

        platform.add_platform_command("set_false_path -from [get_clocks main_s7pciephy_clkout*] -to [get_clocks main_clkout]")

        self.add_csr("pcie_phy")

        # USB FT601 PHY ----------------------------------------------------------------------------
        self.submodules.usb_phy = FT601Sync(platform.request("usb_fifo"), dw=32, timeout=1024)

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
            self.submodules.pcileech_fifo = pcileech_fifo = PCILeechFIFO()

            # USB PHY <--> PCILeechFIFO
            # usb_phy.source = words from FT601 (host→FPGA)
            # usb_phy.sink   = words to   FT601 (FPGA→host)
            self.comb += [
                self.usb_phy.source.connect(pcileech_fifo.usb_rx),
                pcileech_fifo.usb_tx.connect(self.usb_phy.sink),
            ]

            # PCIe PHY <--> PCILeechFIFO
            #
            # pcie_phy uses 64-bit AXI stream (phy_layout(64)).
            # PCILeechFIFO uses 32-bit words with last signal.
            # StrideConverter handles the width change; the "last" field in
            # phy_layout maps directly to our tlp_rx/tlp_tx last signal.
            #
            # RX: PCIe bus → pcie_phy.source (64-bit) → conv → pcileech_fifo.tlp_rx (32-bit)
            self.submodules.tlp_rx_conv = tlp_rx_conv = StrideConverter(
                phy_layout(64), phy_layout(32), reverse=False)

            # TX: pcileech_fifo.tlp_tx (32-bit) → conv → pcie_phy.sink (64-bit) → PCIe bus
            self.submodules.tlp_tx_conv = tlp_tx_conv = StrideConverter(
                phy_layout(32), phy_layout(64), reverse=False)

            self.comb += [
                # RX path
                self.pcie_phy.source.connect(tlp_rx_conv.sink),
                tlp_rx_conv.source.connect(pcileech_fifo.tlp_rx),
                # TX path
                pcileech_fifo.tlp_tx.connect(tlp_tx_conv.sink),
                tlp_tx_conv.source.connect(self.pcie_phy.sink),
            ]

            # PCIe reset driven by CMD register file.
            # rw[200] starts at 1 (core held in reset at startup).
            # Host clears it via CMD write to bring PCIe core online.
            # Hold PCIe core in reset until host clears rw[200] via CMD write.
            # pcie_phy.pcie_rst_n feeds i_sys_rst_n on the IP — active low.
            self.comb += If(pcileech_fifo.pcie_rst_core,self.pcie_phy.pcie_rst_n.eq(0))
            
            # Wire PCIe PHY status signals for PCIE register space responses
            self.comb += [
                pcileech_fifo.phy_lnk_up   .eq(self.pcie_phy._link_status.fields.status),
                pcileech_fifo.phy_ltssm    .eq(self.pcie_phy._link_status.fields.ltssm),
                pcileech_fifo.phy_lnk_rate .eq(self.pcie_phy._link_status.fields.rate),
                pcileech_fifo.phy_lnk_width.eq(self.pcie_phy._link_status.fields.width),
                pcileech_fifo.phy_id       .eq(self.pcie_phy.id),
            ]

        # MSI --------------------------------------------------------------------------------------
        self.submodules.msi = MSI()
        self.comb += self.msi.source.connect(self.pcie_phy.msi)
        self.add_csr("msi")

        # LEDs -------------------------------------------------------------------------------------
        usb_counter = Signal(32)
        self.sync.usb += usb_counter.eq(usb_counter + 1)
        self.comb += platform.request("user_led", 0).eq(usb_counter[26])

        pcie_counter = Signal(32)
        self.sync.pcie += pcie_counter.eq(pcie_counter + 1)
        self.comb += platform.request("user_led", 1).eq(pcie_counter[26])

        # Analyzer ---------------------------------------------------------------------------------
        if with_analyzer:
            analyzer_signals = [
                self.pcie_phy.sink,
                self.pcie_phy.source,
            ]
            self.submodules.analyzer = LiteScopeAnalyzer(
                analyzer_signals, 1024, csr_csv="test/analyzer.csv")
            self.add_csr("analyzer")

# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="PCIe Screamer Squirrel Gateware")
    parser.add_argument("--with-analyzer", action="store_true", help="Enable LiteScope analyzer")
    parser.add_argument("--with-loopback", action="store_true", help="Enable USB loopback")
    parser.add_argument("--build",         action="store_true", help="Build bitstream")
    parser.add_argument("--load",          action="store_true", help="Load via JTAG")
    parser.add_argument("--flash",         action="store_true", help="Flash to SPI flash")
    args = parser.parse_args()

    platform = Platform()
    soc      = PCIeSquirrel(platform,
                            with_analyzer=args.with_analyzer,
                            with_loopback=args.with_loopback)
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
