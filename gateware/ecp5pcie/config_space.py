from migen import *


# PCIe config space for a single-function x1 endpoint: Type 0 header, PCI PM cap at 0x40,
# 32-bit one-vector MSI cap at 0x48, PCIe cap v2 at 0x60, nothing in extended config.
# Register file in the core's pcie_rx domain.

PM_CAP   = 0x40
MSI_CAP  = 0x48
PCIE_CAP = 0x60


def _bar_mask(size):
    assert size >= 128 and size & (size - 1) == 0
    return ~(size - 1) & 0xffffffff


class ConfigSpace(Module):
    """Type 0 config space register file.

    `reg` is the dword index (offset >> 2, 10 bits). `rd_data` is valid the cycle after `reg`
    changes. A write with `we` applies `wr_data` to the bytes in `wr_be` that are writable.
    Byte i of `rd_data`/`wr_data` is config space byte offset*4 + i.

    Vendor `0x10ee` is the Xilinx id, the same one the Squirrel uses. Device
    `0x0667` is a second id so a Squirrel (`10ee:0666`) and this endpoint can
    both sit in one host. A rescan reuses an id the host has already seen, so
    this id stays. It is not a Lattice id.
    """
    def __init__(self, vendor_id=0x10ee, device_id=0x0667, revision=0, class_code=0x020000,
                 subsystem_vendor_id=None, subsystem_id=None, bar0_size=4096, gen=1):
        self.reg     = Signal(10)
        self.rd_data = Signal(32)
        self.we      = Signal()
        self.wr_be   = Signal(4)
        self.wr_data = Signal(32)

        # Local write from the SoC (the Squirrel cfg_mgmt port). Same masked update as a
        # host CfgWr. A host write in the same cycle wins; `mgmt_done` stays low then.
        self.mgmt_we   = Signal()
        self.mgmt_reg  = Signal(10)
        self.mgmt_be   = Signal(4)
        self.mgmt_data = Signal(32)
        self.mgmt_done = Signal()

        self.command    = Signal(16)
        self.dev_ctl    = Signal(16)
        self.bar0       = Signal(32)
        self.bus_master = Signal()
        self.mem_enable = Signal()
        self.max_payload_size = Signal(3)   # Device Control encoding: 128 << n bytes
        self.max_read_request = Signal(3)
        # Classic MSI, 32-bit address, one vector. Enable is Message Control bit 0.
        self.msi_enable  = Signal()
        self.msi_address = Signal(32)
        self.msi_data    = Signal(16)

        subsystem_vendor_id = vendor_id if subsystem_vendor_id is None else subsystem_vendor_id
        subsystem_id        = device_id if subsystem_id is None else subsystem_id
        self.vendor_id = vendor_id
        self.device_id = device_id
        self.revision  = revision

        dev_cap  = (1 << 15) | (7 << 9) | (7 << 6) | 1       # role-based errors, no L0s/L1 limit, MPS 256
        link_cap = (1 << 4) | gen                            # x1, max speed
        link_sta = (1 << 4) | gen                            # x1, current speed
        speeds   = 0b0010 if gen == 1 else 0b0110            # Link Cap 2 supported speeds

        # dword index: (reset value, writable bits)
        regs = {
            0x00 >> 2: ((device_id << 16) | vendor_id,                       0),
            0x04 >> 2: ((1 << 20),                                           0x0000_0546),
            0x08 >> 2: ((class_code << 8) | revision,                        0),
            0x0c >> 2: (0,                                                   0x0000_00ff),
            0x10 >> 2: (0,                                                   _bar_mask(bar0_size)),
            0x2c >> 2: ((subsystem_id << 16) | subsystem_vendor_id,          0),
            0x34 >> 2: (PM_CAP,                                              0),
            0x3c >> 2: (0,                                                   0x0000_00ff),
            # PCI Power Management: version 3, D0 only; PMCSR PowerState RW, No_Soft_Reset.
            # Next pointer is the MSI capability.
            (PM_CAP + 0x0) >> 2: ((0x0003 << 16) | (MSI_CAP << 8) | 0x01,    0),
            (PM_CAP + 0x4) >> 2: ((1 << 3),                                  0x0000_0003),
            # MSI: 32-bit address, one vector, no per-vector mask. Message Control enable is RW.
            (MSI_CAP + 0x0) >> 2: ((PCIE_CAP << 8) | 0x05,                   0x0001_0000),
            (MSI_CAP + 0x4) >> 2: (0,                                        0xffff_fffc),
            (MSI_CAP + 0x8) >> 2: (0,                                        0x0000_ffff),
            # PCI Express v2, endpoint.
            (PCIE_CAP + 0x00) >> 2: ((0x0002 << 16) | 0x10,                  0),
            (PCIE_CAP + 0x04) >> 2: (dev_cap,                                0),
            # Device Control: error reporting, RO, MPS, ext tag, no snoop, MRRS 512.
            (PCIE_CAP + 0x08) >> 2: (0x2810,                                 0x0000_79ff),
            (PCIE_CAP + 0x0c) >> 2: (link_cap,                               0),
            # Link Control: ASPM, RCB, common clock, extended synch.
            (PCIE_CAP + 0x10) >> 2: ((link_sta << 16),                       0x0000_00cb),
            (PCIE_CAP + 0x2c) >> 2: (speeds,                                 0),
            # Link Control 2: target link speed.
            (PCIE_CAP + 0x30) >> 2: (gen,                                    0x0000_000f),
        }

        storage = {}
        for idx, (reset, writable) in regs.items():
            if writable:
                storage[idx] = Signal(32, reset=reset, name=f"cfg_{idx*4:03x}")

        byte_mask = Cat(*[Replicate(self.wr_be[i], 8) for i in range(4)])
        mgmt_mask = Cat(*[Replicate(self.mgmt_be[i], 8) for i in range(4)])
        for idx, sig in storage.items():
            writable = regs[idx][1]
            self.sync.pcie_rx += If(self.we & (self.reg == idx),
                sig.eq((sig & ~(byte_mask & writable)) | (self.wr_data & byte_mask & writable))
            ).Elif(self.mgmt_we & (self.mgmt_reg == idx),
                sig.eq((sig & ~(mgmt_mask & writable)) | (self.mgmt_data & mgmt_mask & writable)))
        self.sync.pcie_rx += self.mgmt_done.eq(self.mgmt_we & ~self.we)

        cases = {idx: self.rd_data.eq(storage[idx] if idx in storage else reset)
                 for idx, (reset, _) in regs.items()}
        cases["default"] = self.rd_data.eq(0)
        self.sync.pcie_rx += Case(self.reg, cases)

        self.comb += [
            self.command.eq(storage[0x04 >> 2][:16]),
            self.bar0.eq(storage[0x10 >> 2]),
            self.dev_ctl.eq(storage[(PCIE_CAP + 0x08) >> 2][:16]),
            self.mem_enable.eq(self.command[1]),
            self.bus_master.eq(self.command[2]),
            self.max_payload_size.eq(self.dev_ctl[5:8]),
            self.max_read_request.eq(self.dev_ctl[12:15]),
            self.msi_enable.eq(storage[MSI_CAP >> 2][16]),
            self.msi_address.eq(storage[(MSI_CAP + 4) >> 2]),
            self.msi_data.eq(storage[(MSI_CAP + 8) >> 2][:16]),
        ]
