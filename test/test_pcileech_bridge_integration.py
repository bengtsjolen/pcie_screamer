#!/usr/bin/env python3
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from migen import *
from migen.sim import run_simulation

from gateware.pcileech_bridge import PCILeechUsbBridge
from gateware.pcileech_fifo import PCILeechFIFO


class Top(Module):
    def __init__(self):
        self.clock_domains.cd_sys = ClockDomain()
        self.submodules.fifo = fifo = PCILeechFIFO(
            mux_out_fifo_depth=16, tlp_rx_fifo_depth=16)
        self.submodules.bridge = bridge = PCILeechUsbBridge()
        self.comb += [
            bridge.fifo_tx.connect(fifo.usb_rx),
            fifo.usb_tx.connect(bridge.fifo_rx),
        ]


def tb(top):
    yield top.bridge.link_ready.eq(1)
    yield top.fifo.tlp_tx.ready.eq(1)
    yield from top.bridge.tx_data.write(0xDEADBEEF)
    yield from top.bridge.tx_data.write(0x77040000)  # TLP magic + last
    for _ in range(50):
        if (yield top.fifo.tlp_tx.valid):
            break
        yield
    else:
        raise AssertionError("TLP did not reach tlp_tx")
    assert (yield top.fifo.tlp_tx.dat) == 0xDEADBEEF
    assert (yield top.fifo.tlp_tx.last) == 1


def main():
    top = Top()
    run_simulation(top, tb(top))
    print("PCILeech bridge + FIFO integration OK")


if __name__ == "__main__":
    main()
