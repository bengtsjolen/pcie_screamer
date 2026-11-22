"""SoC-side config write: the Squirrel cfg_mgmt port sets Command (MSE|BME), and a host
CfgWr in the same cycle wins with the local write retried on the next free cycle."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from migen import *

from gateware.ecp5pcie.config_space import ConfigSpace


class DUT(Module):
    def __init__(self):
        self.submodules.cfg = ClockDomainsRenamer({"pcie_rx": "sys"})(ConfigSpace())
        self.wr_en  = Signal()
        self.dwaddr = Signal(10)
        self.di     = Signal(32)
        self.be     = Signal(4)
        self.done   = Signal()
        hold = Signal()
        self.comb += [
            self.cfg.mgmt_we.eq((self.wr_en | hold) & ~self.cfg.we),
            self.cfg.mgmt_reg.eq(self.dwaddr),
            self.cfg.mgmt_be.eq(self.be),
            self.cfg.mgmt_data.eq(self.di),
            self.done.eq(self.cfg.mgmt_done),
        ]
        self.sync += hold.eq((self.wr_en | hold) & self.cfg.we)


def main():
    dut = DUT()
    result = {}

    def tb():
        # One-cycle write of Command = MSE | BME, the Squirrel pulse.
        yield dut.dwaddr.eq(1)
        yield dut.di.eq(0x00000006)
        yield dut.be.eq(0x3)
        yield dut.wr_en.eq(1)
        yield
        yield dut.wr_en.eq(0)
        yield
        result["command"] = (yield dut.cfg.command)
        result["bme"] = (yield dut.cfg.bus_master)
        result["mse"] = (yield dut.cfg.mem_enable)
        result["done"] = (yield dut.done)

        # Host CfgWr collides: the local write must wait, then land.
        yield dut.di.eq(0x00000000)
        yield dut.cfg.we.eq(1)
        yield dut.cfg.reg.eq(0x3c >> 2)   # interrupt line, not Command
        yield dut.cfg.wr_be.eq(0x1)
        yield dut.cfg.wr_data.eq(0x0000005A)
        yield dut.wr_en.eq(1)
        yield
        yield dut.wr_en.eq(0)
        yield dut.cfg.we.eq(0)
        result["during"] = (yield dut.cfg.command)
        yield
        yield
        result["after"] = (yield dut.cfg.command)

    run_simulation(dut, tb(), clocks={"sys": 10})
    ok = (result["command"] == 0x0006 and result["bme"] == 1
          and result["mse"] == 1 and result["done"] == 1
          and result["during"] == 0x0006 and result["after"] == 0x0000)
    print(f"command=0x{result['command']:04x} bme={result['bme']} mse={result['mse']} "
          f"done={result['done']}")
    print(f"collision held 0x{result['during']:04x}, then 0x{result['after']:04x}")
    print("PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
