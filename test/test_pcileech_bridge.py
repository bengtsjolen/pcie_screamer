#!/usr/bin/env python3
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from migen import *
from migen.sim import run_simulation

from gateware.pcileech_bridge import PCILeechUsbBridge


def wait_for(dut, signal, value, timeout=100):
    for _ in range(timeout):
        if (yield signal) == value:
            return
        yield
    raise AssertionError(f"signal did not become {value}")


def tb(dut):
    yield dut.link_ready.eq(1)
    yield dut.fifo_tx.ready.eq(0)
    yield dut.fifo_rx.valid.eq(0)
    yield

    assert not (yield dut.fifo_tx.valid)
    assert (yield dut.tx_full.status) == 0
    assert (yield dut.rx_empty.status) == 1
    status = yield from dut.status.read()
    assert status == 0b0101, hex(status)

    # TX: hold the sink stalled, verify stability, then consume.
    yield from dut.tx_data.write(0x11223344)
    yield from wait_for(dut, dut.fifo_tx.valid, 1)
    assert (yield dut.fifo_tx.data) == 0x44332211
    for _ in range(5):
        assert (yield dut.fifo_tx.valid)
        assert (yield dut.fifo_tx.data) == 0x44332211
        yield
    yield dut.fifo_tx.ready.eq(1)
    yield
    yield
    assert not (yield dut.fifo_tx.valid)

    # Second TX word, ordering/consumption.
    yield from dut.tx_data.write(0xDEADBEEF)
    yield from wait_for(dut, dut.fifo_tx.valid, 1)
    assert (yield dut.fifo_tx.data) == 0xEFBEADDE
    yield
    assert not (yield dut.fifo_tx.valid)

    # RX: hold valid until ready, poison data after the handshake.
    yield dut.fifo_rx.data.eq(0x55667788)
    yield dut.fifo_rx.valid.eq(1)
    yield from wait_for(dut, dut.fifo_rx.ready, 1)
    yield
    yield dut.fifo_rx.valid.eq(0)
    yield dut.fifo_rx.data.eq(0xDEADBEEF)
    for _ in range(20):
        if not (yield dut.rx_empty.status):
            break
        yield
    else:
        raise AssertionError("RX FIFO did not fill")
    assert (yield from dut.rx_data.read()) == 0x88776655
    yield
    assert (yield dut.rx_empty.status) == 1

    # Second RX word, ordering/consumption.
    yield dut.fifo_rx.data.eq(0xAABBCCDD)
    yield dut.fifo_rx.valid.eq(1)
    yield from wait_for(dut, dut.fifo_rx.ready, 1)
    yield
    yield dut.fifo_rx.valid.eq(0)
    for _ in range(20):
        if not (yield dut.rx_empty.status):
            break
        yield
    else:
        raise AssertionError("second RX word not captured")
    assert (yield from dut.rx_data.read()) == 0xDDCCBBAA
    yield
    assert (yield dut.rx_empty.status) == 1


def main():
    dut = PCILeechUsbBridge()
    run_simulation(dut, tb(dut))
    print("PCILeech bridge simulation OK")


if __name__ == "__main__":
    main()
