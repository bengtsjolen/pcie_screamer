#!/usr/bin/env python3
"""ECP5 Versa PCIe bring-up SoC: ECP5-PCIe core, status CSRs and UARTBone."""

import argparse
import os
from pathlib import Path

from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.soc.integration.soc_core import SoCMini
from litex.soc.integration.builder import Builder

from gateware.ecp5pcie.core import ECP5PCIeCore

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
    def __init__(self, platform, gen=1, fabric_clk=False, transaction_layer=False,
                 ignore_perst=False, uart_baudrate=115200, analyzer_csv=None):
        sys_clk_freq = int(100e6)
        SoCMini.__init__(self, platform, sys_clk_freq,
            ident         = "pcie_screamer ECP5 PCIe bring-up",
            ident_version = True,
        )

        self.submodules.crg = _CRG(platform)
        self.add_uartbone(uart_name="serial", baudrate=uart_baudrate)

        # ispCLOCK: enable the PCIe refclk output.
        self.comb += [
            platform.request("refclk_en").eq(1),
            platform.request("refclk_rst_n").eq(1),
        ]

        self.submodules.pcie_core = core = ECP5PCIeCore(platform, platform.request("pcie_perst_n"),
            gen               = gen,
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
    parser.add_argument("--fabric-clk", action="store_true",
                        help="Diagnostic: DCU refclk from the 100 MHz board oscillator instead of the slot")
    parser.add_argument("--transaction-layer", action="store_true",
                        help="Diagnostic: include ECP5-PCIe's own config-space responder")
    parser.add_argument("--ignore-perst", action="store_true", help="Treat PERST# as always released")
    parser.add_argument("--uart-baudrate", type=int, default=115200)
    parser.add_argument("--with-analyzer", action="store_true", help="LiteScope on lane symbols")
    parser.add_argument("--build", action="store_true", help="Build bitstream")
    parser.add_argument("--load",  action="store_true", help="Load bitstream over JTAG")
    args = parser.parse_args()

    if args.gen == 2 and args.device != "LFE5UM5G":
        parser.error("--gen 2 needs --device LFE5UM5G")
    if args.gen == 2 and args.fabric_clk:
        parser.error("--gen 2 needs a 200 MHz refclk on EXTREF0, not --fabric-clk")

    tools_bin = ROOT / "tools" / "bin"
    if tools_bin.is_dir():
        os.environ["PATH"] = f"{tools_bin}{os.pathsep}{os.environ['PATH']}"

    platform = Platform(device=args.device)
    build_name = f"soc_ecp5_{args.device.lower()}_gen{args.gen}"
    if args.fabric_clk:
        build_name += "_fabclk"
    if args.transaction_layer:
        build_name += "_tl"
    output_dir = ROOT / "build" / build_name

    soc = SoCECP5PCIe(platform,
        gen               = args.gen,
        fabric_clk        = args.fabric_clk,
        transaction_layer = args.transaction_layer,
        ignore_perst      = args.ignore_perst,
        uart_baudrate     = args.uart_baudrate,
        analyzer_csv      = output_dir / "analyzer.csv" if args.with_analyzer else None,
    )
    builder = Builder(soc, output_dir=str(output_dir), csr_csv=str(output_dir / "csr.csv"))
    builder.build(run=args.build, build_name=build_name)

    if args.load:
        prog = platform.create_programmer()
        prog.load_bitstream(os.path.join(builder.gateware_dir, build_name + ".bit"))


if __name__ == "__main__":
    main()
