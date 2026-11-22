#!/usr/bin/env python3
"""ECP5 Versa search image: the Squirrel search console on ECP5PCIePHY.

ChunkDMA and ReadBench share the PHY, selected by bench_select, and the CPU
can read both buffers. Same software/soc_search_console as
soc_squirrel_search.py. On-chip buffers only.
"""

import argparse
import os
from pathlib import Path

from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.soc.integration.soc_core import SoCCore
from litex.soc.integration.soc import SoCRegion
from litex.soc.integration.builder import Builder as LiteXBuilder
from litex.soc.interconnect import wishbone
from litex.soc.interconnect.csr import CSRStatus, CSRStorage
from litex.soc.interconnect.stream import PipeValid

from litepcie.common import phy_layout

from gateware.ecp5pcie.phy import ECP5PCIePHY
from gateware.chunk_dma import ChunkDMA
from gateware.read_bench import ReadBench

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


class WishboneReadReg(Module):
    """One wait state on a slave so its read data is a register, not a BRAM output."""

    def __init__(self, slave, front):
        taken = Signal()
        self.comb += [
            slave.adr.eq(front.adr),
            slave.dat_w.eq(front.dat_w),
            slave.sel.eq(front.sel),
            slave.we.eq(front.we),
            slave.cti.eq(front.cti),
            slave.bte.eq(front.bte),
            slave.cyc.eq(front.cyc),
            slave.stb.eq(front.stb & ~taken & ~front.ack),
            front.err.eq(0),
        ]
        self.sync += [
            If(front.ack,
                front.ack.eq(0),
                taken.eq(0),
            ).Elif(slave.ack,
                front.ack.eq(1),
                front.dat_r.eq(slave.dat_r),
                taken.eq(1),
            ),
        ]


class _CRG(Module):
    def __init__(self, platform):
        self.clock_domains.cd_sys = ClockDomain()
        clk100 = platform.request("clk100")
        rst_n  = platform.request("rst_n")
        self.comb += self.cd_sys.clk.eq(clk100)
        self.specials += AsyncResetSynchronizer(self.cd_sys, ~rst_n)


class SoCECP5Search(SoCCore):
    def __init__(self, platform, gen=1, refclk_freq=None, fabric_clk=False,
                 bar0_size=0x1000, ignore_perst=False):
        sys_clk_freq = int(100e6)
        SoCCore.__init__(self, platform, sys_clk_freq,
            cpu_type            = "vexriscv",
            cpu_variant         = "minimal",
            integrated_rom_size = 0x10000,
            integrated_sram_size= 0x2000,
            uart_name           = "serial",
            uart_baudrate       = 115200,
            uart_rx_fifo_rx_we  = True,
            with_timer          = False,
            with_ctrl           = False,
            bus_timeout         = None,
            ident               = "pcie_screamer ECP5 search",
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
        self.bus.add_slave(
            name   = "dma_buf",
            slave  = chunk_dma.bus,
            region = SoCRegion(origin=0x20000000, size=0x2000, mode="r"),
        )
        self.bench_select = CSRStorage(1)
        self.add_csr("bench_select")
        self.submodules.read_bench = read_bench = ReadBench()
        self.add_csr("read_bench")
        self.bus.add_slave(
            name   = "read_buf",
            slave  = read_bench.bus,
            region = SoCRegion(origin=0x20008000, size=0x4000, mode="r"),
        )
        # A register in front of each engine. Without it the shared TLP beat
        # is decoded by both engines in one cycle and that route misses 100 MHz.
        self.submodules.chunk_rx = chunk_rx = PipeValid(phy_layout(64))
        self.submodules.bench_rx = bench_rx = PipeValid(phy_layout(64))
        self.comb += [
            chunk_rx.sink.valid.eq(pcie_phy.source.valid & ~self.bench_select.storage),
            chunk_rx.sink.dat.eq(pcie_phy.source.dat),
            chunk_rx.sink.be.eq(pcie_phy.source.be),
            chunk_rx.sink.last.eq(pcie_phy.source.last),
            bench_rx.sink.valid.eq(pcie_phy.source.valid & self.bench_select.storage),
            bench_rx.sink.dat.eq(pcie_phy.source.dat),
            bench_rx.sink.be.eq(pcie_phy.source.be),
            bench_rx.sink.last.eq(pcie_phy.source.last),
            pcie_phy.source.ready.eq(Mux(self.bench_select.storage,
                                         bench_rx.sink.ready,
                                         chunk_rx.sink.ready)),
            chunk_rx.source.connect(chunk_dma.tlp_rx),
            bench_rx.source.connect(read_bench.tlp_rx),
            pcie_phy.sink.valid.eq(Mux(self.bench_select.storage,
                                       read_bench.tlp_tx.valid,
                                       chunk_dma.tlp_tx.valid)),
            pcie_phy.sink.dat.eq(Mux(self.bench_select.storage,
                                     read_bench.tlp_tx.dat,
                                     chunk_dma.tlp_tx.dat)),
            pcie_phy.sink.be.eq(Mux(self.bench_select.storage,
                                    read_bench.tlp_tx.be,
                                    chunk_dma.tlp_tx.be)),
            pcie_phy.sink.last.eq(Mux(self.bench_select.storage,
                                      read_bench.tlp_tx.last,
                                      chunk_dma.tlp_tx.last)),
            chunk_dma.tlp_tx.ready.eq(Mux(self.bench_select.storage, 0,
                                          pcie_phy.sink.ready)),
            read_bench.tlp_tx.ready.eq(Mux(self.bench_select.storage,
                                           pcie_phy.sink.ready, 0)),
            chunk_dma.link_ready.eq(
                pcie_phy._link_status.fields.status
                & pcie_phy._bus_master_enable.status
                & (pcie_phy.id != 0)
            ),
            chunk_dma.requester_id.eq(pcie_phy.id),
            read_bench.link_ready.eq(
                pcie_phy._link_status.fields.status
                & pcie_phy._bus_master_enable.status
                & (pcie_phy.id != 0)
            ),
            read_bench.requester_id.eq(pcie_phy.id),
        ]

        core = pcie_phy.core
        sys_count = Signal(27)
        rx_count  = Signal(26)
        self.sync += sys_count.eq(sys_count + 1)
        self.sync.pcie_rx += rx_count.eq(rx_count + 1)
        leds = Cat(sys_count[-1], core.perst_n, core.rx_present, core.rx_locked,
                   core.rx_aligned, core.link_up, core.dll_up, rx_count[-1])
        self.comb += platform.request_all("user_led").eq(~leds if LED_ACTIVE_LOW else leds)

        # BRAM clock-to-out plus the slave data mux does not reach the CPU in 10 ns.
        for name, slave in list(self.bus.slaves.items()):
            front = wishbone.Interface.like(slave)
            self.submodules += WishboneReadReg(slave, front)
            self.bus.slaves[name] = front

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
    parser.add_argument("--seed", type=int, default=1)
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
    build_name = f"soc_ecp5_search_{args.device.lower()}_gen{args.gen}"
    if not args.fabric_clk:
        build_name += f"_ref{int(refclk_freq / 1e6)}"
    output_dir = ROOT / "build" / build_name

    soc = SoCECP5Search(platform,
        gen          = args.gen,
        refclk_freq  = refclk_freq,
        fabric_clk   = args.fabric_clk,
        ignore_perst = args.ignore_perst,
    )
    builder = Builder(soc, output_dir=str(output_dir), csr_csv=str(output_dir / "csr.csv"))
    builder.software_packages = []
    builder.software_libraries = []
    builder.add_software_package("bios", str(ROOT / "software" / "soc_search_console"))
    builder.build(run=args.build, build_name=build_name, seed=args.seed)

    bitstream = os.path.join(builder.gateware_dir, build_name + ".bit")
    if args.load:
        platform.create_programmer().load_bitstream(bitstream)
    if args.flash:
        platform.create_programmer().flash(0, bitstream)


if __name__ == "__main__":
    main()
