#!/usr/bin/env python3
"""ECP5 Versa V4: VexRiscv plus ChunkDMA (one MRd per chunk) on ECP5PCIePHY.

On-chip buffers only; the Versa DDR3 is unused. ReadBench stays on the Squirrel
search top: its completion path does not close 100 MHz here.
"""

import argparse
import os
from pathlib import Path

from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.soc.integration.soc_core import SoCCore
from litex.soc.integration.builder import Builder as LiteXBuilder
from litex.soc.interconnect.csr import CSRStatus

from gateware.ecp5pcie.phy import ECP5PCIePHY
from gateware.chunk_dma import ChunkDMA

from platforms.pcie_versa import Platform, DEVICES

ROOT = Path(__file__).resolve().parent

LED_ACTIVE_LOW = True


class Builder(LiteXBuilder):
    def add_software_package(self, name, src_dir=None):
        for index, (package_name, _) in enumerate(self.software_packages):
            if package_name == name:
                if src_dir is not None:
                    self.software_packages[index] = (name, src_dir)
                return
        super().add_software_package(name, src_dir)

    def _check_meson(self):
        pass


class _CRG(Module):
    def __init__(self, platform):
        self.clock_domains.cd_sys = ClockDomain()
        clk100 = platform.request("clk100")
        rst_n  = platform.request("rst_n")
        self.comb += self.cd_sys.clk.eq(clk100)
        self.specials += AsyncResetSynchronizer(self.cd_sys, ~rst_n)


class SoCECP5V4(SoCCore):
    def __init__(self, platform, gen=1, refclk_freq=None, fabric_clk=False,
                 bar0_size=0x1000, ignore_perst=False):
        sys_clk_freq = int(100e6)
        SoCCore.__init__(self, platform, sys_clk_freq,
            cpu_type            = "vexriscv",
            cpu_variant         = "minimal",
            integrated_rom_size = 0x8000,
            integrated_sram_size= 0x2000,
            uart_name           = "serial",
            uart_baudrate       = 115200,
            uart_rx_fifo_rx_we  = True,
            with_timer          = False,
            with_ctrl           = False,
            bus_timeout         = None,
            ident               = "pcie_screamer ECP5 V4",
            ident_version       = True,
        )

        self.submodules.crg = _CRG(platform)
        self.comb += [
            platform.request("refclk_en").eq(1),
            platform.request("refclk_rst_n").eq(1),
        ]

        self.submodules.pcie_phy = pcie_phy = ECP5PCIePHY(platform,
            platform.request("pcie_perst_n"),
            gen          = gen,
            refclk_freq  = refclk_freq,
            fabric_clk   = fabric_clk,
            ignore_perst = ignore_perst,
            bar0_size    = bar0_size,
        )
        self.add_csr("pcie_phy")

        self.pcie_id = CSRStatus(16)
        self.comb += self.pcie_id.status.eq(pcie_phy.id)
        self.add_csr("pcie_id")

        bme_done  = Signal()
        bme_timer = Signal(20)
        bme_pulse = Signal()
        self.sync.pcie += [
            bme_pulse.eq(0),
            If(~bme_done,
                If(pcie_phy._link_status.fields.status,
                    If(bme_timer < (1 << 20) - 1,
                        bme_timer.eq(bme_timer + 1),
                    ).Elif(~bme_pulse,
                        bme_pulse.eq(1),
                        bme_done.eq(1),
                    ),
                ),
            ),
        ]
        self.comb += [
            pcie_phy.cfg_mgmt_dwaddr.eq(1),
            pcie_phy.cfg_mgmt_di.eq(0x00000006),
            pcie_phy.cfg_mgmt_byte_en.eq(0x3),
            pcie_phy.cfg_mgmt_wr_en.eq(bme_pulse),
        ]

        self.submodules.chunk_dma = chunk_dma = ChunkDMA()
        self.add_csr("chunk_dma")
        self.comb += [
            pcie_phy.source.connect(chunk_dma.tlp_rx),
            chunk_dma.tlp_tx.connect(pcie_phy.sink),
            chunk_dma.link_ready.eq(
                pcie_phy._link_status.fields.status
                & pcie_phy._bus_master_enable.status
                & (pcie_phy.id != 0)
            ),
            chunk_dma.requester_id.eq(pcie_phy.id),
        ]

        core = pcie_phy.core
        sys_count = Signal(27)
        rx_count  = Signal(26)
        self.sync += sys_count.eq(sys_count + 1)
        self.sync.pcie_rx += rx_count.eq(rx_count + 1)
        leds = Cat(sys_count[-1], core.perst_n, core.rx_present, core.rx_locked,
                   core.rx_aligned, core.link_up, core.dll_up, rx_count[-1])
        self.comb += platform.request_all("user_led").eq(~leds if LED_ACTIVE_LOW else leds)

    def add_csr_bridge(self, name="csr", origin=None, register=False):
        super().add_csr_bridge(name=name, origin=origin, register=True)

    def check_bios_requirements(self):
        for mem in ["rom", "sram"]:
            if mem not in self.bus.regions:
                raise RuntimeError(f"custom BIOS needs {mem} region")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", choices=DEVICES, required=True)
    parser.add_argument("--gen", type=int, choices=[1, 2], default=1)
    parser.add_argument("--refclk-mhz", type=float)
    parser.add_argument("--fabric-clk", action="store_true")
    parser.add_argument("--ignore-perst", action="store_true")
    parser.add_argument("--seed", type=int, default=10,
                        help="nextpnr seed (10 closed clk100 at 101.09 MHz)")
    parser.add_argument("--build", action="store_true")
    parser.add_argument("--load", action="store_true")
    parser.add_argument("--flash", action="store_true")
    args = parser.parse_args()

    if args.fabric_clk:
        refclk_freq = 100e6
    elif args.refclk_mhz:
        refclk_freq = args.refclk_mhz * 1e6
    else:
        refclk_freq = 156.25e6 if args.gen == 1 else 200e6
    if args.gen == 2 and args.device != "LFE5UM5G":
        parser.error("--gen 2 needs --device LFE5UM5G")

    tools_bin = ROOT / "tools" / "bin"
    if tools_bin.is_dir():
        os.environ["PATH"] = f"{tools_bin}{os.pathsep}{os.environ['PATH']}"

    platform = Platform(device=args.device)
    build_name = f"soc_ecp5_v4_{args.device.lower()}_gen{args.gen}"
    if not args.fabric_clk:
        build_name += f"_ref{int(refclk_freq / 1e6)}"
    output_dir = ROOT / "build" / build_name

    soc = SoCECP5V4(platform,
        gen          = args.gen,
        refclk_freq  = refclk_freq,
        fabric_clk   = args.fabric_clk,
        ignore_perst = args.ignore_perst,
    )
    builder = Builder(soc, output_dir=str(output_dir), csr_csv=str(output_dir / "csr.csv"))
    builder.software_packages = []
    builder.software_libraries = []
    builder.add_software_package("bios", str(ROOT / "software" / "soc_chunk_console"))
    builder.build(run=args.build, build_name=build_name, seed=args.seed)

    bitstream = os.path.join(builder.gateware_dir, build_name + ".bit")
    if args.load:
        platform.create_programmer().load_bitstream(bitstream)
    if args.flash:
        platform.create_programmer().flash(0, bitstream)


if __name__ == "__main__":
    main()
