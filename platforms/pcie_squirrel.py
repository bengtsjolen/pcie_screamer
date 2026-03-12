# PCIe Screamer Squirrel platform for LiteX/Migen
# Based on enjoy-digital/pcie_screamer platform
# Adapted for Screamer PCIe Squirrel (XC7A35T-FGG484)

from migen import *
from litex.build.generic_platform import *
from litex.build.xilinx import XilinxPlatform

_io = [
    # 100 MHz system clock
    ("clk100", 0, Pins("H4"), IOStandard("LVCMOS33")),

    # LEDs
    ("user_led", 0, Pins("Y6"),  IOStandard("LVCMOS33")),
    ("user_led", 1, Pins("AB5"), IOStandard("LVCMOS33")),

    # Buttons (active-low)
    ("user_btn", 0, Pins("AB3"), IOStandard("LVCMOS33")),
    ("user_btn", 1, Pins("AA5"), IOStandard("LVCMOS33")),

    # Serial (undocumented but present)
    ("serial", 0,
        Subsignal("tx", Pins("T1")),
        Subsignal("rx", Pins("U1")),
        IOStandard("LVCMOS33"),
    ),

    # PCIe x1
    ("pcie_x1", 0,
        Subsignal("rst_n", Pins("B13"), IOStandard("LVCMOS33")),
        Subsignal("clk_p", Pins("F6")),
        Subsignal("clk_n", Pins("E6")),
        Subsignal("rx_p",  Pins("B10")),
        Subsignal("rx_n",  Pins("A10")),
        Subsignal("tx_p",  Pins("B6")),
        Subsignal("tx_n",  Pins("A6")),
    ),

    # FT601 USB3 FIFO clock (100 MHz from FT601)
    ("usb_fifo_clock", 0, Pins("W19"), IOStandard("LVCMOS33")),

    # FT601 USB3 FIFO interface
    ("usb_fifo", 0,
        Subsignal("rst",   Pins("Y9")),
        Subsignal("data",  Pins(
            "N13 N14 N15 P15 P16 N17 P17 R17",
            "P19 R18 R19 T18 U18 V18 V19 V17",
            "W20 Y19 T21 T20 U21 V20 W22 W21",
            "Y22 Y21 AA21 AB22 AA20 AB21 AA19 AB20")),
        Subsignal("be",    Pins("Y18 AA18 AB18 W17")),
        Subsignal("rxf_n", Pins("AB8")),
        Subsignal("txe_n", Pins("AA8")),
        Subsignal("rd_n",  Pins("AA6")),
        Subsignal("wr_n",  Pins("AB7")),
        Subsignal("oe_n",  Pins("AB6")),
        Subsignal("siwua", Pins("Y8")),
        IOStandard("LVCMOS33"), Misc("SLEW=FAST"),
    ),
]

class Platform(XilinxPlatform):
    default_clk_name   = "clk100"
    default_clk_period = 1e9/100e6

    def __init__(self):
        XilinxPlatform.__init__(self, "xc7a35t-fgg484-2", _io, toolchain="vivado")
        self.toolchain.bitstream_commands = [
            "set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 4 [current_design]",
            "set_property BITSTREAM.CONFIG.CONFIGRATE 40 [current_design]",
            "set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]",
        ]
        self.toolchain.additional_commands = [
            "write_cfgmem -force -format bin -interface spix4 -size 16 "
            "-loadbit \"up 0x0 {build_name}.bit\" -file {build_name}.bin"
        ]
