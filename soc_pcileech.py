#!/usr/bin/env python3

import os
import argparse

from migen import *

from litex.soc.cores.clock import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import Builder as LiteXBuilder
from litex.soc.interconnect.csr import CSRStatus

from litepcie.phy.s7pciephy import S7PCIEPHY

from gateware.pcileech_fifo import PCILeechFIFO
from gateware.pcileech_bridge import PCILeechUsbBridge
from gateware.tlp32 import TLP32To64Packer

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


class SoCPCILeech(SoCCore):
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

        link_ready = Signal()
        self.comb += link_ready.eq(
            self.pcie_phy._link_status.fields.status
            & self.pcie_phy._bus_master_enable.status
            & (self.pcie_phy.id != 0)
        )

        self.submodules.pcileech_fifo = pcileech_fifo = PCILeechFIFO(
            mux_out_fifo_depth = 2048,
            tlp_rx_fifo_depth  = 2048,
        )
        self.submodules.pcileech_bridge = pcileech_bridge = PCILeechUsbBridge()
        self.add_csr("pcileech_bridge")

        self.submodules.tlp_tx_conv = tlp_tx_conv = TLP32To64Packer(
            swap_words=False
        )

        self.comb += [
            self.pcie_phy.source.connect(pcileech_fifo.tlp_rx),
            pcileech_fifo.tlp_tx.connect(tlp_tx_conv.sink),
            tlp_tx_conv.source.connect(self.pcie_phy.sink),
            pcileech_bridge.fifo_tx.connect(pcileech_fifo.usb_rx),
            pcileech_fifo.usb_tx.connect(pcileech_bridge.fifo_rx),
            pcileech_bridge.link_ready.eq(link_ready),
            self.pcie_phy.pcie_rst_n.eq(~pcileech_fifo.pcie_rst_core),
            pcileech_fifo.phy_lnk_up.eq(self.pcie_phy._link_status.fields.status),
            pcileech_fifo.phy_ltssm.eq(self.pcie_phy._link_status.fields.ltssm),
            pcileech_fifo.phy_lnk_rate.eq(self.pcie_phy._link_status.fields.rate),
            pcileech_fifo.phy_lnk_width.eq(self.pcie_phy._link_status.fields.width),
            pcileech_fifo.phy_id.eq(self.pcie_phy.id),
            pcileech_fifo.cfg_dcommand.eq(self.pcie_phy.dcommand),
            pcileech_fifo.tx_axis_seen.eq(
                self.pcie_phy.s_axis_tx_tvalid & self.pcie_phy.s_axis_tx_tready),
            pcileech_fifo.rx_axis_seen.eq(self.pcie_phy.m_axis_rx_tvalid),
            pcileech_fifo.tx_err_drop.eq(self.pcie_phy.tx_err_drop),
        ]

    def check_bios_requirements(self):
        for mem in ["rom", "sram"]:
            if mem not in self.bus.regions:
                raise RuntimeError(f"custom BIOS needs {mem} region")


def main():
    parser = argparse.ArgumentParser(description="PCIe Screamer PCILeechFIFO VexRiscv SoC")
    parser.add_argument("--build", action="store_true", help="Build bitstream")
    parser.add_argument("--load",  action="store_true", help="Load via JTAG")
    parser.add_argument("--flash", action="store_true", help="Flash to SPI flash")
    args = parser.parse_args()

    platform = Platform()
    soc = SoCPCILeech(platform)
    soc.build_name = "soc_pcileech"

    builder = Builder(soc, output_dir="build/soc_pcileech",
                      csr_csv="test/soc_pcileech_csr.csv")
    builder.software_packages = []
    builder.software_libraries = []

    bios_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "software", "soc_pcileech_console")
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
