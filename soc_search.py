#!/usr/bin/env python3

import os
import argparse

from migen import *

from litex.soc.cores.clock import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import Builder as LiteXBuilder
from litex.soc.interconnect.csr import CSRStatus

from litepcie.phy.s7pciephy import S7PCIEPHY

from gateware.chunk_dma import ChunkDMA

from platforms.pcie_squirrel import Platform


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
    def __init__(self, platform, sys_clk_freq):
        self.clock_domains.cd_sys = ClockDomain()

        sys_clk_100 = platform.request("clk100")
        self.submodules.pll = pll = S7PLL(speedgrade=-2)
        pll.register_clkin(sys_clk_100, 100e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq)


class SoCSearch(SoCCore):
    def __init__(self, platform):
        sys_clk_freq = int(100e6)

        SoCCore.__init__(self, platform, sys_clk_freq,
            cpu_type             = "vexriscv",
            cpu_variant          = "minimal",
            integrated_rom_size  = 0x8000,
            integrated_sram_size = 0x2000,
            uart_name            = "serial",
            uart_baudrate        = 115200,
            uart_rx_fifo_rx_we   = True,
            with_timer           = False,
            with_ctrl            = False,
        )

        self.submodules.crg = _CRG(platform, sys_clk_freq)

        self.submodules.pcie_phy = pcie_phy = S7PCIEPHY(
            platform,
            platform.request("pcie_x1"),
            data_width = 64,
            bar0_size  = 0x40000,
        )
        pcie_phy.config["Device_ID"] = "0666"
        pcie_phy.config["Class_Code_Base"] = "02"
        pcie_phy.config["Class_Code_Sub"] = "00"
        pcie_phy.config["Extended_Tag_Field"] = "true"
        pcie_phy.config["Extended_Tag_Default"] = "true"

        platform.toolchain.pre_optimize_commands.append(
            "set_property LOC GTPE2_CHANNEL_X0Y2 [get_cells "
            "{{pcie_s7/inst/inst/gt_top_i/pipe_wrapper_i/pipe_lane[0]"
            ".gt_wrapper_i/gtp_channel.gtpe2_channel_i}}]"
        )

        platform.add_false_path_constraints_by_name(
            "main_s7pciephy_clkout*", "main_crg_clkout")

        self.add_csr("pcie_phy")

        self.pcie_id = CSRStatus(16)
        self.comb += self.pcie_id.status.eq(pcie_phy.id)
        self.add_csr("pcie_id")

        bme_done = Signal()
        bme_timer = Signal(20)
        bme_pulse = Signal()
        self.sync.pcie += [
            bme_pulse.eq(0),
            If(~bme_done,
               If(self.pcie_phy._link_status.fields.status,
                  If(bme_timer < (1<<20)-1,
                     bme_timer.eq(bme_timer + 1)
                     ).Elif(~bme_pulse,
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

        self.submodules.chunk_dma = chunk_dma = ChunkDMA()
        self.add_csr("chunk_dma")

        self.comb += [
            self.pcie_phy.source.connect(chunk_dma.tlp_rx),
            chunk_dma.tlp_tx.connect(self.pcie_phy.sink),
            chunk_dma.link_ready.eq(
                self.pcie_phy._link_status.fields.status
                & self.pcie_phy._bus_master_enable.status
                & (self.pcie_phy.id != 0)
            ),
            chunk_dma.requester_id.eq(self.pcie_phy.id),
        ]

    def check_bios_requirements(self):
        for mem in ["rom", "sram"]:
            if mem not in self.bus.regions:
                raise RuntimeError(f"custom BIOS needs {mem} region")


def main():
    parser = argparse.ArgumentParser(description="PCIe Screamer Search VexRiscv SoC")
    parser.add_argument("--build", action="store_true", help="Build bitstream")
    parser.add_argument("--load",  action="store_true", help="Load via JTAG")
    parser.add_argument("--flash", action="store_true", help="Flash to SPI flash")
    args = parser.parse_args()

    platform = Platform()
    soc = SoCSearch(platform)
    soc.build_name = "soc_search"

    builder = Builder(soc, output_dir="build/soc_search", csr_csv="test/soc_search_csr.csv")
    builder.software_packages = []
    builder.software_libraries = []

    bios_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "software", "soc_search_console")
    builder.add_software_package("bios", bios_dir)

    builder.build(run=args.build, build_name=soc.build_name)

    if args.load:
        prog = platform.create_programmer()
        prog.load_bitstream(os.path.join(builder.gateware_dir, soc.build_name + ".bit"))

    if args.flash:
        prog = platform.create_programmer()
        prog.flash(0, os.path.join(builder.gateware_dir, soc.build_name + ".bin"))


if __name__ == "__main__":
    main()
