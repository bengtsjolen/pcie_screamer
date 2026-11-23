#!/usr/bin/env python3
"""ECP5 Versa LitePCIe example: one DMA channel and one MSI vector.

Same clock and pin setup as the Gen1 search image. `SoC.add_pcie` supplies the
CSR window, the DMA reader and writer, and `LitePCIeMSI` on `phy.msi`.
"""

import argparse
import os
from pathlib import Path

from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.soc.integration.soc_core import SoCMini
from litex.soc.integration.builder import Builder

from litepcie.software import generate_litepcie_software

from gateware.ecp5pcie.phy import ECP5PCIePHY

from platforms.pcie_versa import Platform, DEVICES

ROOT = Path(__file__).resolve().parent

# Versa user LEDs are lit when driven low.
LED_ACTIVE_LOW = True

# One BAR covers the CSR window, including the DMA descriptor table.
BAR0_SIZE = 0x10000


class _CRG(Module):
    def __init__(self, platform):
        self.clock_domains.cd_sys = ClockDomain()

        # The core needs a clean 100 MHz sys clock (SCI, optional DCU refclk), so no PLL.
        clk100 = platform.request("clk100")
        rst_n  = platform.request("rst_n")
        self.comb += self.cd_sys.clk.eq(clk100)
        self.specials += AsyncResetSynchronizer(self.cd_sys, ~rst_n)


class SoCECP5Example(SoCMini):
    def __init__(self, platform, refclk_freq, ignore_perst=False, uart_baudrate=115200):
        sys_clk_freq = int(100e6)
        SoCMini.__init__(self, platform, sys_clk_freq,
            ident         = "pcie_screamer ECP5 LitePCIe example",
            ident_version = True,
        )

        self.submodules.crg = _CRG(platform)
        self.add_uartbone(uart_name="serial", baudrate=uart_baudrate)

        # Enable the onboard clock synthesizer that drives EXTREF0 (156.25 MHz as shipped).
        self.comb += [
            platform.request("refclk_en").eq(1),
            platform.request("refclk_rst_n").eq(1),
        ]

        perst_n = platform.request("pcie_perst_n")
        self.submodules.pcie_phy = phy = ECP5PCIePHY(platform, perst_n,
            gen          = 1,
            refclk_freq  = refclk_freq,
            ignore_perst = ignore_perst,
            bar0_size    = BAR0_SIZE,
        )
        self.add_csr("pcie_phy")

        # ndmas=1 forces MSI. max_pending_requests matches the four-deep retry buffer;
        # the adapter still emits one TLP at a time, and read tags stay outstanding
        # until their completions return.
        self.add_pcie(phy=phy, ndmas=1, address_width=64,
            max_pending_requests=4, with_msi=True, msi_type="msi")

        core = phy.core
        sys_count = Signal(27)
        rx_count  = Signal(26)
        self.sync += sys_count.eq(sys_count + 1)
        self.sync.pcie_rx += rx_count.eq(rx_count + 1)
        leds = Cat(sys_count[-1], core.perst_n, core.rx_present, core.rx_locked,
                   core.rx_aligned, core.link_up, core.dll_up, rx_count[-1])
        self.comb += platform.request_all("user_led").eq(~leds if LED_ACTIVE_LOW else leds)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", choices=DEVICES, required=True,
                        help="LFE5UM (Versa). The example is Gen1 on the non-5G part")
    parser.add_argument("--refclk-mhz", type=float, default=156.25,
                        help="EXTREF0 frequency (default 156.25, the Versa's onboard synthesizer)")
    parser.add_argument("--ignore-perst", action="store_true", help="Treat PERST# as always released")
    parser.add_argument("--uart-baudrate", type=int, default=115200)
    parser.add_argument("--seed", type=int, default=1, help="nextpnr placement seed")
    parser.add_argument("--build", action="store_true", help="Build bitstream")
    parser.add_argument("--load",  action="store_true", help="Load bitstream into SRAM over JTAG")
    parser.add_argument("--flash", action="store_true", help="Write bitstream to SPI flash over JTAG")
    args = parser.parse_args()

    if args.device != "LFE5UM":
        parser.error("this example is the non-5G Versa (LFE5UM), Gen1")

    tools_bin = ROOT / "tools" / "bin"
    if tools_bin.is_dir():
        os.environ["PATH"] = f"{tools_bin}{os.pathsep}{os.environ['PATH']}"

    refclk_freq = args.refclk_mhz * 1e6
    platform = Platform(device=args.device)
    build_name = f"soc_ecp5_example_{args.device.lower()}_gen1_ref{int(refclk_freq / 1e6)}"
    output_dir = ROOT / "build" / build_name

    soc = SoCECP5Example(platform,
        refclk_freq   = refclk_freq,
        ignore_perst  = args.ignore_perst,
        uart_baudrate = args.uart_baudrate,
    )
    builder = Builder(soc, output_dir=str(output_dir), csr_csv=str(output_dir / "csr.csv"))
    builder.build(run=args.build, build_name=build_name, seed=args.seed)

    software = output_dir / "litepcie_software"
    generate_litepcie_software(soc, str(software))

    bitstream = os.path.join(builder.gateware_dir, build_name + ".bit")
    if args.load:
        platform.create_programmer().load_bitstream(bitstream)
    if args.flash:
        platform.create_programmer().flash(0, bitstream, verify=True)


if __name__ == "__main__":
    main()
