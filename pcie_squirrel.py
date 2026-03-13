#!/usr/bin/env python3
# PCIe Screamer Squirrel - LiteX/Migen target
# Based on enjoy-digital/pcie_screamer by Florent Kermarrec
# Adapted for Screamer PCIe Squirrel (XC7A35T-FGG484)
#
# Usage:
#   python3 pcie_squirrel.py --build
#   python3 pcie_squirrel.py --build --load         # JTAG
#   python3 pcie_squirrel.py --build --flash        # SPI flash
#   python3 pcie_squirrel.py --build --with-analyzer

import os
import argparse

from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.build.generic_platform import *
from litex.soc.cores.clock import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *
from litex.soc.interconnect import stream
from litex.soc.cores.uart import UARTWishboneBridge
from litex.soc.cores.usb_fifo import phy_description

from litepcie.phy.s7pciephy import S7PCIEPHY

from gateware.usb import USBCore
from gateware.etherbone import Etherbone
from gateware.tlp import TLP
from gateware.msi import MSI
from gateware.ft601 import FT601Sync

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
        # PCIe screamer used pcie reset for ft601 meaning if pcie is bad, ft601 never comes up
        # optionally we could use sys domain for reset or simply just keep the ft601 out of reset.
        #self.specials += AsyncResetSynchronizer(self.cd_usb, ResetSignal("pcie"))
        #self.specials += AsyncResetSynchronizer(self.cd_usb, ResetSignal("sys"))
        self.comb += self.cd_usb.rst.eq(0)
        
        

# PCIeSquirrel -------------------------------------------------------------------------------------

class PCIeSquirrel(SoCMini):
    usb_map = {
        "wishbone": 0,
        "tlp":      1,
    }

    def __init__(self, platform, with_analyzer=False, with_loopback=False):
        sys_clk_freq = int(100e6)

        SoCMini.__init__(self, platform, sys_clk_freq,
                         ident="PCIe Screamer Squirrel", ident_version=True)

        # CRG --------------------------------------------------------------------------------------
        self.submodules.crg = _CRG(platform, sys_clk_freq)

        # Serial Wishbone Bridge -------------------------------------------------------------------
        self.submodules.bridge = UARTWishboneBridge(
            platform.request("serial"), sys_clk_freq, baudrate=3e6)
        self.bus.add_master(master=self.bridge.wishbone)

        # PCIe PHY ---------------------------------------------------------------------------------
        pcie_phy = self.submodules.pcie_phy = S7PCIEPHY(platform, platform.request("pcie_x1"),
                                             data_width=64, bar0_size=0x40000)
        pcie_phy.config["Device_ID"] = "0666"
        pcie_phy.config["Class_Code_Base"] = "02"
        pcie_phy.config["Class_Code_Sub"] = "00"
        
        # vivado places ip at GTPE2_CHANNEL_X0Y3 for some reason so need to force it like this:
        platform.toolchain.pre_optimize_commands.append("set_property LOC GTPE2_CHANNEL_X0Y2 [get_cells {{pcie_s7/inst/inst/gt_top_i/pipe_wrapper_i/pipe_lane[0].gt_wrapper_i/gtp_channel.gtpe2_channel_i}}]")

        self.add_csr("pcie_phy")

        # USB FT601 PHY ----------------------------------------------------------------------------
        self.submodules.usb_phy = FT601Sync(platform.request("usb_fifo"), dw=32, timeout=1024)

        # USB Loopback -----------------------------------------------------------------------------
        if with_loopback:
            self.submodules.usb_loopback_fifo = stream.SyncFIFO(phy_description(32), 2048)
            self.comb += [
                self.usb_phy.source.connect(self.usb_loopback_fifo.sink),
                self.usb_loopback_fifo.source.connect(self.usb_phy.sink),
            ]
        # USB Core ---------------------------------------------------------------------------------
        else:
            self.submodules.usb_core = USBCore(self.usb_phy, sys_clk_freq)

            # USB <--> Wishbone --------------------------------------------------------------------
            self.submodules.etherbone = Etherbone(self.usb_core, self.usb_map["wishbone"])
            self.bus.add_master(master=self.etherbone.master.bus)

            # USB <--> TLP -------------------------------------------------------------------------
            self.submodules.tlp = TLP(self.usb_core, self.usb_map["tlp"])
            self.comb += [
                self.pcie_phy.source.connect(self.tlp.sender.sink),
                self.tlp.receiver.source.connect(self.pcie_phy.sink),
            ]

        # Wishbone --> MSI -------------------------------------------------------------------------
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
    parser.add_argument("--flash",         action="store_true", help="Flash to SPI")
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
