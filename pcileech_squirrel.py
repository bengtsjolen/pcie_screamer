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
from migen.genlib.cdc import MultiReg, BusSynchronizer

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

class TLP64To32Unpacker(Module):
    def __init__(self, swap_words=False):
        self.sink   = sink   = stream.Endpoint(phy_layout(64))
        self.source = source = stream.Endpoint(phy_layout(32))
        
        # Buffered 64-bit beat.
        lo_dat   = Signal(32)
        hi_dat   = Signal(32)
        lo_be    = Signal(4)
        hi_be    = Signal(4)
        lo_v     = Signal()
        hi_v     = Signal()
        beat_last = Signal()
        
        self.submodules.fsm = fsm = FSM(reset_state="LOAD")
        
        self.comb += [
            sink.ready.eq(0),
            source.valid.eq(0),
            source.dat.eq(0),
            source.be.eq(0),
            source.last.eq(0),
        ]
        
        # Accept exactly one 64-bit beat.
        fsm.act("LOAD",
                sink.ready.eq(1),
                If(sink.valid & sink.ready,
                   If(swap_words,
                      NextValue(lo_dat, sink.dat[32:64]),
                      NextValue(lo_be,  sink.be[4:8]),
                      NextValue(hi_dat, sink.dat[0:32]),
                      NextValue(hi_be,  sink.be[0:4]),
                      NextValue(lo_v,   sink.be[4:8] != 0),
                      NextValue(hi_v,   sink.be[0:4] != 0),
                      ).Else(
                          NextValue(lo_dat, sink.dat[0:32]),
                          NextValue(lo_be,  sink.be[0:4]),
                          NextValue(hi_dat, sink.dat[32:64]),
                          NextValue(hi_be,  sink.be[4:8]),
                          NextValue(lo_v,   sink.be[0:4] != 0),
                          NextValue(hi_v,   sink.be[4:8] != 0),
                      ),
                   NextValue(beat_last, sink.last),
                   If(swap_words,
                      If(sink.be[4:8] != 0,
                         NextState("EMIT_LO")
                         ).Elif(sink.be[0:4] != 0,
                                NextState("EMIT_HI")
                                )
                      ).Else(
                          If(sink.be[0:4] != 0,
                             NextState("EMIT_LO")
                             ).Elif(sink.be[4:8] != 0,
                                    NextState("EMIT_HI")
                                    )
                      )
                   )
                )
        
        # Emit first 32-bit word.
        fsm.act("EMIT_LO",
                source.valid.eq(1),
                source.dat.eq(lo_dat),
                source.be.eq(lo_be),
                source.last.eq(beat_last & ~hi_v),
                If(source.ready,
                   If(hi_v,
                      NextState("EMIT_HI")
                      ).Else(
                          NextState("LOAD")
                      )
                   )
                )
        
        # Emit second 32-bit word.
        fsm.act("EMIT_HI",
                source.valid.eq(1),
                source.dat.eq(hi_dat),
                source.be.eq(hi_be),
                source.last.eq(beat_last),
                If(source.ready,
                   NextState("LOAD")
                   )
                )

class StrideConverterBEFilter(Module):
    """Strip ghost words (be=0) from a StrideConverter 64→32 output.

    Inserts 1 cycle of latency.  Correctly moves the 'last' flag from
    a ghost word back onto the preceding real word.

    Connect:  StrideConverter.source → BEFilter.sink
              BEFilter.source → downstream (pcileech_fifo.tlp_rx)
    """
    def __init__(self):
        self.sink   = sink   = stream.Endpoint(phy_layout(32))
        self.source = source = stream.Endpoint(phy_layout(32))

        # Pipeline register
        buf_dat  = Signal(32)
        buf_be   = Signal(4)
        buf_last = Signal()

        # Combinational peek at the incoming word
        is_ghost = Signal()
        self.comb += is_ghost.eq(sink.valid & (sink.be == 0))

        self.submodules.fsm = fsm = FSM(reset_state="EMPTY")

        # ---- EMPTY: buffer has no word, not producing output ----
        fsm.act("EMPTY",
            source.valid.eq(0),
            # Accept any real (non-ghost) word into the buffer.
            # Ghost-when-empty shouldn't happen in normal operation,
            # but if it does, just consume and stay empty.
            sink.ready.eq(1),
            If(sink.valid & ~is_ghost,
                NextValue(buf_dat,  sink.dat),
                NextValue(buf_be,   sink.be),
                NextValue(buf_last, sink.last),
                NextState("FULL"),
            )
        )

        # ---- FULL: present the buffered word, peek at the next ----
        fsm.act("FULL",
            source.valid.eq(1),
            source.dat .eq(buf_dat),
            source.be  .eq(buf_be),
            # Merge ghost's 'last' into our output combinationally:
            # if the very next word is a ghost (be=0, last=1), we set
            # last=1 on THIS word and discard the ghost.
            source.last.eq(buf_last | is_ghost),

            If(source.ready,
                # Output word accepted — advance the pipeline.
                sink.ready.eq(1),
                If(is_ghost,
                    # Ghost consumed (its last was merged above).
                    # Nothing real to buffer → go empty.
                    NextState("EMPTY"),
                ).Elif(sink.valid,
                    # Real word: latch into buffer, stay full.
                    NextValue(buf_dat,  sink.dat),
                    NextValue(buf_be,   sink.be),
                    NextValue(buf_last, sink.last),
                    # NextState stays "FULL" implicitly
                ).Else(
                    # No new word available — drain buffer, go empty.
                    NextState("EMPTY"),
                )
            ).Else(
                # Downstream stalled — hold everything.
                sink.ready.eq(0),
            )
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

    def __init__(self, platform, with_analyzer=False, with_loopback=False):
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

        # Advertise 8-bit (256) tag support — matches ufrisk's PCIeSquirrel gateware.
        # Without this, the IP advertises only 5-bit (32) tags in DEVCAP, but the
        # pcileech host-tool's tag allocator runs an 8-bit counter.  Beyond ~32
        # outstanding MRds the host reuses tag IDs that the device interprets as
        # the old in-flight tag, completions get routed to wrong requests, and
        # the transfer deadlocks.  Empirically we work up to 9 × 4 KB pages
        # (~144 MRds) and stall at 10 pages (160 MRds); enabling extended tags
        # lets pcileech use the full 8-bit tag space.
        pcie_phy.config["Extended_Tag_Field"]   = "true"
        pcie_phy.config["Extended_Tag_Default"] = "true"

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
        self.submodules.usb_phy = FT601Sync(platform.request("usb_fifo"), dw=32, timeout=256)

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
            if 1: 
                self.submodules.tlp_rx_conv = tlp_rx_conv = StrideConverter(
                    phy_layout(64), phy_layout(32), reverse=False)
                self.submodules.tlp_rx_filt = tlp_rx_filt = StrideConverterBEFilter()

                
            else:
                self.submodules.tlp_rx_conv = tlp_rx_conv = TLP64To32Unpacker(
                    swap_words=False
                )

            # TX: pcileech_fifo.tlp_tx (32-bit) → conv → pcie_phy.sink (64-bit) → PCIe bus
            if 0:
                self.submodules.tlp_tx_conv = tlp_tx_conv = StrideConverter(
                    phy_layout(32), phy_layout(64), reverse=False)
            else:
                self.submodules.tlp_tx_conv = tlp_tx_conv = TLP32To64Packer(
                    swap_words=False
                )
                
            self.comb += [
                # RX path
                self.pcie_phy.source.connect(tlp_rx_conv.sink),
                tlp_rx_conv.source.connect(tlp_rx_filt.sink),
                tlp_rx_filt.source.connect(pcileech_fifo.tlp_rx),
                
                # TX path
                pcileech_fifo.tlp_tx.connect(tlp_tx_conv.sink),
                tlp_tx_conv.source.connect(self.pcie_phy.sink),
            ]

            if 1:
                txsink_dbg0      = Signal(64)
                txsink_dbg1      = Signal(64)
                txsink_dbg_be0   = Signal(8)
                txsink_dbg_be1   = Signal(8)
                txsink_dbg_last0 = Signal()
                txsink_dbg_last1 = Signal()
                txsink_dbg_seen  = Signal()
                txsink_dbg_armed = Signal(reset=1)
                txsink_dbg_count = Signal(2)
            
                self.sync += [
                    If(ResetSignal(),
                       txsink_dbg0.eq(0),
                       txsink_dbg1.eq(0),
                       txsink_dbg_be0.eq(0),
                       txsink_dbg_be1.eq(0),
                       txsink_dbg_last0.eq(0),
                       txsink_dbg_last1.eq(0),
                       txsink_dbg_seen.eq(0),
                       txsink_dbg_armed.eq(1),
                       txsink_dbg_count.eq(0),
                       ).Elif(txsink_dbg_armed & self.pcie_phy.sink.valid & self.pcie_phy.sink.ready,
                              txsink_dbg_seen.eq(1),
                              Case(txsink_dbg_count, {
                                  0: [
                                      txsink_dbg0.eq(self.pcie_phy.sink.dat),
                                      txsink_dbg_be0.eq(self.pcie_phy.sink.be),
                                      txsink_dbg_last0.eq(self.pcie_phy.sink.last),
                                  ],
                                  1: [
                                    txsink_dbg1.eq(self.pcie_phy.sink.dat),
                                      txsink_dbg_be1.eq(self.pcie_phy.sink.be),
                                      txsink_dbg_last1.eq(self.pcie_phy.sink.last),
                                      txsink_dbg_armed.eq(0),
                                  ],
                              }),
                              If(txsink_dbg_count != 1,
                                 txsink_dbg_count.eq(txsink_dbg_count + 1)
                                 )
                            )
                ]


                self.comb += [
                    pcileech_fifo.txsink_dbg0.eq(txsink_dbg0),
                    pcileech_fifo.txsink_dbg1.eq(txsink_dbg1),
                    pcileech_fifo.txsink_dbg_be0.eq(txsink_dbg_be0),
                    pcileech_fifo.txsink_dbg_be1.eq(txsink_dbg_be1),
                    pcileech_fifo.txsink_dbg_last0.eq(txsink_dbg_last0),
                    pcileech_fifo.txsink_dbg_last1.eq(txsink_dbg_last1),
                    pcileech_fifo.txsink_dbg_flags.eq(Cat(
                        txsink_dbg_seen,    # bit 0
                        txsink_dbg_armed,   # bit 1
                        txsink_dbg_count,   # bits 3:2
                        Constant(0, 12),
                    )),
                ]

            if 1:
                rx_dbg0 = Signal(64)
                rx_dbg1 = Signal(64)
                rx_dbg2 = Signal(64)
                rx_dbg3 = Signal(64)
                rx_dbg4 = Signal(64)
                rx_dbg5 = Signal(64)
                rx_dbg6 = Signal(64)
                rx_dbg7 = Signal(64)

                rx_be0 = Signal(8)
                rx_be1 = Signal(8)
                rx_be2 = Signal(8)
                rx_be3 = Signal(8)
                rx_be4 = Signal(8)
                rx_be5 = Signal(8)
                rx_be6 = Signal(8)
                rx_be7 = Signal(8)

                rx_lasts = Signal(8)
                rx_seen  = Signal()
                rx_armed = Signal(reset=1)
                rx_count = Signal(4)

                self.sync += [
                    If(ResetSignal(),
                       rx_dbg0.eq(0), rx_dbg1.eq(0), rx_dbg2.eq(0), rx_dbg3.eq(0),
                       rx_dbg4.eq(0), rx_dbg5.eq(0), rx_dbg6.eq(0), rx_dbg7.eq(0),
                       rx_be0.eq(0),  rx_be1.eq(0),  rx_be2.eq(0),  rx_be3.eq(0),
                       rx_be4.eq(0),  rx_be5.eq(0),  rx_be6.eq(0),  rx_be7.eq(0),
                       rx_lasts.eq(0),
                       rx_seen.eq(0),
                       rx_armed.eq(1),
                       rx_count.eq(0),
                       ).Elif(rx_armed & self.pcie_phy.source.valid & self.pcie_phy.source.ready,
                              rx_seen.eq(1),
                              Case(rx_count, {
                                  0: [rx_dbg0.eq(self.pcie_phy.source.dat), rx_be0.eq(self.pcie_phy.source.be), rx_lasts[0].eq(self.pcie_phy.source.last)],
                                  1: [rx_dbg1.eq(self.pcie_phy.source.dat), rx_be1.eq(self.pcie_phy.source.be), rx_lasts[1].eq(self.pcie_phy.source.last)],
                                  2: [rx_dbg2.eq(self.pcie_phy.source.dat), rx_be2.eq(self.pcie_phy.source.be), rx_lasts[2].eq(self.pcie_phy.source.last)],
                                  3: [rx_dbg3.eq(self.pcie_phy.source.dat), rx_be3.eq(self.pcie_phy.source.be), rx_lasts[3].eq(self.pcie_phy.source.last)],
                                  4: [rx_dbg4.eq(self.pcie_phy.source.dat), rx_be4.eq(self.pcie_phy.source.be), rx_lasts[4].eq(self.pcie_phy.source.last)],
                                  5: [rx_dbg5.eq(self.pcie_phy.source.dat), rx_be5.eq(self.pcie_phy.source.be), rx_lasts[5].eq(self.pcie_phy.source.last)],
                                  6: [rx_dbg6.eq(self.pcie_phy.source.dat), rx_be6.eq(self.pcie_phy.source.be), rx_lasts[6].eq(self.pcie_phy.source.last)],
                                7: [
                                    rx_dbg7.eq(self.pcie_phy.source.dat),
                                    rx_be7.eq(self.pcie_phy.source.be),
                                    rx_lasts[7].eq(self.pcie_phy.source.last),
                                    rx_armed.eq(0),
                                ],
                              }),
                              If(rx_count != 7,
                                 rx_count.eq(rx_count + 1)
                                 )
                              )
                ]
                
                self.comb += [
                    pcileech_fifo.rxsink_dbg[0].eq(rx_dbg0),
                    pcileech_fifo.rxsink_dbg[1].eq(rx_dbg1),
                    pcileech_fifo.rxsink_dbg[2].eq(rx_dbg2),
                    pcileech_fifo.rxsink_dbg[3].eq(rx_dbg3),
                    pcileech_fifo.rxsink_dbg[4].eq(rx_dbg4),
                    pcileech_fifo.rxsink_dbg[5].eq(rx_dbg5),
                    pcileech_fifo.rxsink_dbg[6].eq(rx_dbg6),
                    pcileech_fifo.rxsink_dbg[7].eq(rx_dbg7),
                    pcileech_fifo.rxsink_be[0].eq(rx_be0),
                    pcileech_fifo.rxsink_be[1].eq(rx_be1),
                    pcileech_fifo.rxsink_be[2].eq(rx_be2),
                    pcileech_fifo.rxsink_be[3].eq(rx_be3),
                    pcileech_fifo.rxsink_be[4].eq(rx_be4),
                    pcileech_fifo.rxsink_be[5].eq(rx_be5),
                    pcileech_fifo.rxsink_be[6].eq(rx_be6),
                    pcileech_fifo.rxsink_be[7].eq(rx_be7),
                    pcileech_fifo.rxsink_lasts.eq(rx_lasts),
                    pcileech_fifo.rxsink_flags.eq(Cat(rx_seen, rx_armed, rx_count, Constant(0, 10))),
                ]



            if 1:
                rx64_seen       = Signal(16)
                rx32_seen       = Signal(16)
                ser_out_seen    = Signal(16)
                usbtx_seen      = Signal(16)
                self.comb += [
                    pcileech_fifo.diag_rx64_seen.eq(rx64_seen),
                    pcileech_fifo.diag_rx32_seen.eq(rx32_seen),
                    pcileech_fifo.diag_ser_out_seen.eq(ser_out_seen),
                    pcileech_fifo.diag_usbtx_seen.eq(usbtx_seen),
                ]
                self.sync += [
                    If(self.pcie_phy.source.valid & self.pcie_phy.source.ready,
                       rx64_seen.eq(rx64_seen + 1)
                       ),
                    If(tlp_rx_conv.source.valid & tlp_rx_conv.source.ready,
                       rx32_seen.eq(rx32_seen + 1)
                       ),
                ]
                
                self.sync += [
                    If(pcileech_fifo.serializer.source.valid & pcileech_fifo.serializer.source.ready,
                       ser_out_seen.eq(ser_out_seen + 1)
                       ),
                    If(self.usb_phy.sink.valid & self.usb_phy.sink.ready,
                       usbtx_seen.eq(usbtx_seen + 1)
                       ),
                ]

                # --- Extra PCIe-IP edge counters ------------------------------
                # Count in the pcie clock domain (where the IP signals live),
                # then CDC-resync the 16-bit counter value into sys.
                #
                # tx_tlp_cnt_pcie : # of TLPs we handed to the IP
                #                   (MRds/MWrs; CfgRd/CfgWr use cfg_mgmt, not s_axis_tx)
                # rx_tlp_cnt_pcie : # of TLPs the IP delivered (CplDs etc.)
                # tx_err_cnt_pcie : # of tx_err_drop pulses
                #
                # TX uses valid & ready & last — AXI-S guarantees this is a
                # single-cycle transfer event per TLP.
                #
                # RX uses valid & ready & last via rx_datapath.sink, because
                # m_axis_rx_tready isn't directly exposed in pcie_phy.  Using
                # just (valid & last) would over-count whenever we backpressure
                # (IP holds valid & last high until tready comes back).
                rx_sink = self.pcie_phy.rx_datapath.sink  # pcie domain

                tx_tlp_cnt_pcie = Signal(16)
                rx_tlp_cnt_pcie = Signal(16)
                tx_err_cnt_pcie = Signal(16)
                self.sync.pcie += [
                    If(self.pcie_phy.s_axis_tx_tvalid
                       & self.pcie_phy.s_axis_tx_tready
                       & self.pcie_phy.s_axis_tx_tlast,
                       tx_tlp_cnt_pcie.eq(tx_tlp_cnt_pcie + 1)
                       ),
                    If(rx_sink.valid & rx_sink.ready & rx_sink.last,
                       rx_tlp_cnt_pcie.eq(rx_tlp_cnt_pcie + 1)
                       ),
                    If(self.pcie_phy.tx_err_drop,
                       tx_err_cnt_pcie.eq(tx_err_cnt_pcie + 1)
                       ),
                ]

                tx_tlp_sync = BusSynchronizer(16, "pcie", "sys")
                rx_tlp_sync = BusSynchronizer(16, "pcie", "sys")
                tx_err_sync = BusSynchronizer(16, "pcie", "sys")
                self.submodules += tx_tlp_sync, rx_tlp_sync, tx_err_sync
                self.comb += [
                    tx_tlp_sync.i.eq(tx_tlp_cnt_pcie),
                    rx_tlp_sync.i.eq(rx_tlp_cnt_pcie),
                    tx_err_sync.i.eq(tx_err_cnt_pcie),
                    pcileech_fifo.diag_tx_tlp_seen    .eq(tx_tlp_sync.o),
                    pcileech_fifo.diag_rx_tlp_seen    .eq(rx_tlp_sync.o),
                    pcileech_fifo.diag_tx_err_drop_cnt.eq(tx_err_sync.o),
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
                pcileech_fifo.diag_tx_axis_seen.eq(self.pcie_phy.s_axis_tx_tvalid & self.pcie_phy.s_axis_tx_tready),
                pcileech_fifo.diag_rx_axis_seen.eq(self.pcie_phy.m_axis_rx_tvalid),
                pcileech_fifo.diag_tx_err_drop.eq(self.pcie_phy.tx_err_drop),
            ]
            if 1:
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

        if 1:
            source_seen_latch = Signal()
            self.sync.sys += If(self.pcie_phy.source.valid, source_seen_latch.eq(1))
            self.comb += pcileech_fifo.phy_source_seen.eq(source_seen_latch)

            raw_rx_seen_pcie = Signal()
            raw_rx_seen_sys  = Signal()
            self.sync.pcie += If(self.pcie_phy.m_axis_rx_tvalid, raw_rx_seen_pcie.eq(1))
            self.specials += MultiReg(raw_rx_seen_pcie, raw_rx_seen_sys)
            self.comb += pcileech_fifo.phy_raw_rx_seen.eq(raw_rx_seen_sys)
            
        # LEDs -------------------------------------------------------------------------------------
        if 0:
            #tx0_latch = Signal()
            #self.sync.pcie += If(pcileech_fifo.tlp_tx.valid & pcileech_fifo.tlp_tx.ready,tx0_latch.eq(1))
            #tx1_latch = Signal()
            #self.sync.pcie += If(tlp_tx_conv.source.valid & tlp_tx_conv.source.ready,tx1_latch.eq(1))
            #tx2_latch = Signal()
            #self.sync.pcie += If(self.pcie_phy.sink.valid & self.pcie_phy.sink.ready,tx2_latch.eq(1))
            
            tx3_latch = Signal()
            self.sync.pcie += If(self.pcie_phy.m_axis_rx_tvalid, tx3_latch.eq(1))
            tx4_latch = Signal()
            self.sync.pcie += If(self.pcie_phy.s_axis_tx_tvalid, tx4_latch.eq(1))
            self.comb += platform.request("user_led", 0).eq(tx3_latch)
            self.comb += platform.request("user_led", 1).eq(tx4_latch)
        elif 0:
            # LED0: tx_err_drop — TLP dropped by PCIe IP (BME not set or flow control)
            tx_err_drop_latch = Signal()
            self.sync.pcie += If(self.pcie_phy.tx_err_drop, tx_err_drop_latch.eq(1))
            self.comb += platform.request("user_led", 0).eq(tx_err_drop_latch)

            # LED1: rx_seen — CplD received from PCIe bus (entirely in pcie domain, no CDC issue)
            rx_seen = Signal()
            self.sync.pcie += If(self.pcie_phy.source.valid, rx_seen.eq(1))
            self.comb += platform.request("user_led", 1).eq(rx_seen)
        elif 0:
            # Sticky LEDs for tx_seen and rx_seen 
            tx_seen = Signal()
            self.sync.pcie += If(self.pcie_phy.sink.valid & self.pcie_phy.sink.ready, tx_seen.eq(1))
            self.comb += platform.request("user_led", 0).eq(tx_seen)

            rx_seen = Signal()
            self.sync.pcie += If(self.pcie_phy.source.valid, rx_seen.eq(1))
            self.comb += platform.request("user_led", 1).eq(rx_seen)
        elif 0:
            # 7/8 LED0 lit at usb clock so verify usb clock present and polarity of driving led
            usb_counter = Signal(32)
            self.sync.usb += usb_counter.eq(usb_counter + 1)
            self.comb += platform.request("user_led", 0).eq(usb_counter[26]|usb_counter[25]|usb_counter[24])
            # Verify presence of pcie clock
            pcie_counter = Signal(32)
            self.sync.pcie += pcie_counter.eq(pcie_counter + 1)
            self.comb += platform.request("user_led", 1).eq(pcie_counter[26])

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
