# Lattice ECP5 Versa platform for the open-source ECP5 PCIe backend.
# Wraps litex-boards' lattice_versa_ecp5, whose default device (LFE5UM5G) is the
# wrong part for the non-5G Versa; the device must be chosen explicitly here.

from litex.build.generic_platform import Pins, IOStandard
from litex.build.openfpgaloader import OpenFPGALoader
from litex_boards.platforms import lattice_versa_ecp5

DEVICES = ("LFE5UM", "LFE5UM5G")

# The DCU uses dedicated SerDes and refclk pins (DCU0 CH0, EXTREF0) that must not be
# requested as fabric IOs, so PERST# from litex-boards' "pcie_x1" is exposed on its own.
_io = [
    ("pcie_perst_n", 0, Pins("A6"), IOStandard("LVCMOS33")),
]


class Platform(lattice_versa_ecp5.Platform):
    def __init__(self, device, toolchain="trellis", **kwargs):
        if device not in DEVICES:
            raise ValueError(f"device must be one of {DEVICES}, got {device!r}")
        lattice_versa_ecp5.Platform.__init__(self, device=device, toolchain=toolchain, **kwargs)
        self.add_extension(_io)

    def create_programmer(self):
        # litex-boards' OpenOCD config hardcodes the LFE5UM5G-45 IDCODE (0x81112043);
        # openFPGALoader detects either part. JTAG is on FT2232H channel A.
        return OpenFPGALoader(cable="ft2232")
