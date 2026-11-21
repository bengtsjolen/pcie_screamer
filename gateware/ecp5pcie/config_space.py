from migen import *


# PCIe config space for a single-function x1 endpoint: Type 0 header, PCI PM cap at 0x40,
# PCIe cap v2 at 0x60, nothing in extended config. Register file in the core's pcie_rx domain.

PM_CAP   = 0x40
PCIE_CAP = 0x60


def _bar_mask(size):
    assert size >= 128 and size & (size - 1) == 0
    return ~(size - 1) & 0xffffffff


class ConfigSpace(Module):
    """Type 0 config space register file.

    `reg` is the dword index (offset >> 2, 10 bits). `rd_data` is valid the cycle after `reg`
    changes. A write with `we` applies `wr_data` to the bytes in `wr_be` that are writable.
    Byte i of `rd_data`/`wr_data` is config space byte offset*4 + i.
    """
    def __init__(self, vendor_id=0x10ee, device_id=0x0666, revision=0, class_code=0x020000,
                 subsystem_vendor_id=None, subsystem_id=None, bar0_size=4096, gen=1):
        self.reg     = Signal(10)
        self.rd_data = Signal(32)
        self.we      = Signal()
        self.wr_be   = Signal(4)
        self.wr_data = Signal(32)

        self.command    = Signal(16)
        self.dev_ctl    = Signal(16)
        self.bar0       = Signal(32)
        self.bus_master = Signal()
        self.mem_enable = Signal()
        self.max_payload_size = Signal(3)   # Device Control encoding: 128 << n bytes
        self.max_read_request = Signal(3)

        subsystem_vendor_id = vendor_id if subsystem_vendor_id is None else subsystem_vendor_id
        subsystem_id        = device_id if subsystem_id is None else subsystem_id

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
            (PM_CAP + 0x0) >> 2: ((0x0003 << 16) | (PCIE_CAP << 8) | 0x01,   0),
            (PM_CAP + 0x4) >> 2: ((1 << 3),                                  0x0000_0003),
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
        for idx, sig in storage.items():
            writable = regs[idx][1]
            self.sync.pcie_rx += If(self.we & (self.reg == idx),
                sig.eq((sig & ~(byte_mask & writable)) | (self.wr_data & byte_mask & writable)))

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
        ]
