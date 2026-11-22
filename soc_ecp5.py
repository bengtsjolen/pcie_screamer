#!/usr/bin/env python3
"""ECP5 Versa PCIe bring-up SoC: ECP5-PCIe core, status CSRs and UARTBone."""

import argparse
import os
from pathlib import Path

from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.soc.integration.soc_core import SoCMini
from litex.soc.integration.builder import Builder
from litex.soc.interconnect import wishbone

from gateware.ecp5pcie.core import ECP5PCIeCore
from gateware.ecp5pcie.phy import ECP5PCIePHY

from platforms.pcie_versa import Platform, DEVICES

ROOT = Path(__file__).resolve().parent

# Versa user LEDs are lit when driven low.
LED_ACTIVE_LOW = True


class _CRG(Module):
    def __init__(self, platform):
        self.clock_domains.cd_sys = ClockDomain()

        # The core needs a clean 100 MHz sys clock (SCI, optional DCU refclk), so no PLL.
        clk100 = platform.request("clk100")
        rst_n  = platform.request("rst_n")
        self.comb += self.cd_sys.clk.eq(clk100)
        self.specials += AsyncResetSynchronizer(self.cd_sys, ~rst_n)


class SoCECP5PCIe(SoCMini):
    def __init__(self, platform, gen=1, refclk_freq=None, fabric_clk=False, transaction_layer=False,
                 litepcie=False, bar0_size=0x1000, ignore_perst=False, uart_baudrate=115200,
                 analyzer_csv=None):
        sys_clk_freq = int(100e6)
        SoCMini.__init__(self, platform, sys_clk_freq,
            ident         = "pcie_screamer ECP5 PCIe bring-up",
            ident_version = True,
        )

        self.submodules.crg = _CRG(platform)
        self.add_uartbone(uart_name="serial", baudrate=uart_baudrate)

        # Enable the onboard clock synthesizer that drives EXTREF0 (156.25 MHz as shipped,
        # measured with the DCU word-clock meters).
        self.comb += [
            platform.request("refclk_en").eq(1),
            platform.request("refclk_rst_n").eq(1),
        ]

        perst_n = platform.request("pcie_perst_n")
        if litepcie:
            # Full transaction layer: LitePCIe endpoint with a BAR0 wishbone-SRAM MMIO target.
            from litepcie.core import LitePCIeEndpoint
            from litepcie.frontend.wishbone import LitePCIeWishboneMaster
            self.submodules.pcie_phy = phy = ECP5PCIePHY(platform, perst_n,
                gen          = gen,
                refclk_freq  = refclk_freq,
                fabric_clk   = fabric_clk,
                ignore_perst = ignore_perst,
                bar0_size    = bar0_size,
            )
            self.add_csr("pcie_phy")
            core = phy.core
            self.submodules.pcie_endpoint = endpoint = LitePCIeEndpoint(phy,
                max_pending_requests=2, endianness="big")
            self.submodules.pcie_wb = wb = LitePCIeWishboneMaster(endpoint)
            self.submodules.bar0_sram = wishbone.SRAM(bar0_size, bus=wb.bus)
        else:
            self.submodules.pcie_core = core = ECP5PCIeCore(platform, perst_n,
                gen               = gen,
                refclk_freq       = refclk_freq,
                fabric_clk        = fabric_clk,
                transaction_layer = transaction_layer,
                ignore_perst      = ignore_perst,
            )
            self.add_csr("pcie_core")

        # LEDs: 0 sys heartbeat, 1 PERST# released, 2 rx_present, 3 rx_locked, 4 rx_aligned,
        # 5 L0, 6 DL_Active, 7 pcie_rx heartbeat (DCU clock running).
        sys_count = Signal(27)
        rx_count  = Signal(26)
        self.sync += sys_count.eq(sys_count + 1)
        self.sync.pcie_rx += rx_count.eq(rx_count + 1)
        leds = Cat(sys_count[-1], core.perst_n, core.rx_present, core.rx_locked,
                   core.rx_aligned, core.link_up, core.dll_up, rx_count[-1])
        self.comb += platform.request_all("user_led").eq(~leds if LED_ACTIVE_LOW else leds)

        if analyzer_csv is not None:
            from litescope import LiteScopeAnalyzer
            self.submodules.analyzer = LiteScopeAnalyzer([
                    core.rx_symbol, core.rx_valid, core.tx_symbol,
                    core.ltssm_state, core.dll_state,
                    core.tlp_rx_data, core.tlp_rx_valid,
                ],
                depth        = 1024,
                clock_domain = "pcie_rx",
                csr_csv      = str(analyzer_csv),
            )
            self.add_csr("analyzer")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", choices=DEVICES, required=True, help="LFE5UM (Versa) or LFE5UM5G (Versa 5G)")
    parser.add_argument("--gen", type=int, choices=[1, 2], default=1)
    parser.add_argument("--refclk-mhz", type=float,
                        help="EXTREF0 frequency (Gen1: 156.25 default, the Versa's onboard synthesizer, "
                             "or 100; Gen2: 200)")
    parser.add_argument("--fabric-clk", action="store_true",
                        help="Diagnostic: DCU refclk from the 100 MHz board oscillator instead of EXTREF0")
    parser.add_argument("--transaction-layer", action="store_true",
                        help="Diagnostic: include ECP5-PCIe's own config-space responder")
    parser.add_argument("--litepcie", action="store_true",
                        help="Wire a LitePCIe endpoint with a BAR0 wishbone-SRAM MMIO target")
    parser.add_argument("--ignore-perst", action="store_true", help="Treat PERST# as always released")
    parser.add_argument("--uart-baudrate", type=int, default=115200)
    parser.add_argument("--with-analyzer", action="store_true", help="LiteScope on lane symbols")
    parser.add_argument("--seed", type=int, default=1, help="nextpnr placement seed")
    parser.add_argument("--build", action="store_true", help="Build bitstream")
    parser.add_argument("--load",  action="store_true", help="Load bitstream into SRAM over JTAG")
    parser.add_argument("--flash", action="store_true", help="Write bitstream to SPI flash over JTAG")
    args = parser.parse_args()

    if args.fabric_clk:
        refclk_freq = 100e6
    elif args.refclk_mhz:
        refclk_freq = args.refclk_mhz * 1e6
    else:
        refclk_freq = 156.25e6 if args.gen == 1 else 200e6

    if args.gen == 2 and args.device != "LFE5UM5G":
        parser.error("--gen 2 needs --device LFE5UM5G")
    if args.litepcie and args.transaction_layer:
        parser.error("--litepcie replaces the transaction layer; drop --transaction-layer")
    if args.gen == 2 and args.fabric_clk:
        parser.error("--gen 2 needs a 200 MHz refclk on EXTREF0, not --fabric-clk")

    tools_bin = ROOT / "tools" / "bin"
    if tools_bin.is_dir():
        os.environ["PATH"] = f"{tools_bin}{os.pathsep}{os.environ['PATH']}"

    platform = Platform(device=args.device)
    build_name = f"soc_ecp5_{args.device.lower()}_gen{args.gen}"
    if not args.fabric_clk:
        build_name += f"_ref{int(refclk_freq/1e6)}"
    if args.fabric_clk:
        build_name += "_fabclk"
    if args.transaction_layer:
        build_name += "_tl"
    if args.litepcie:
        build_name += "_litepcie"
    output_dir = ROOT / "build" / build_name

    soc = SoCECP5PCIe(platform,
        gen               = args.gen,
        refclk_freq       = refclk_freq,
        fabric_clk        = args.fabric_clk,
        transaction_layer = args.transaction_layer,
        litepcie          = args.litepcie,
        ignore_perst      = args.ignore_perst,
        uart_baudrate     = args.uart_baudrate,
        analyzer_csv      = output_dir / "analyzer.csv" if args.with_analyzer else None,
    )
    builder = Builder(soc, output_dir=str(output_dir), csr_csv=str(output_dir / "csr.csv"))
    builder.build(run=args.build, build_name=build_name, seed=args.seed)

    bitstream = os.path.join(builder.gateware_dir, build_name + ".bit")
    if args.load:
        platform.create_programmer().load_bitstream(bitstream)
    if args.flash:
        platform.create_programmer().flash(0, bitstream, verify=True)


if __name__ == "__main__":
    main()
