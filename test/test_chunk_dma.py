#!/usr/bin/env python3
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from migen import *
from migen.sim import run_simulation

from gateware.chunk_dma import ChunkDMA


REQ_ID = 0x1600
STATUS_LINK_READY = 1 << 0
STATUS_DONE = 1 << 2
STATUS_ERROR = 1 << 3


def expect_mrd64(dut, addr, length, tag):
    beats = []
    for _ in range(2):
        for _ in range(200):
            if (yield dut.tlp_tx.valid):
                break
            yield
        else:
            raise AssertionError("TLP beat not emitted")
        beats.append(((yield dut.tlp_tx.dat), (yield dut.tlp_tx.last),
                      (yield dut.tlp_tx.be)))
        yield

    length_field = 0 if length == 1024 else length
    dw0 = 0x20000000 | length_field
    dw1 = (REQ_ID << 16) | (tag << 8) | 0x0f
    expected0 = (dw1 << 32) | dw0
    expected1 = ((addr >> 32) << 32) | (addr & 0xfffffffc)

    assert beats[0][0] == expected0, hex(beats[0][0])
    assert beats[0][1] == 0
    assert beats[0][2] == 0xff
    assert beats[1][0] == expected1, hex(beats[1][0])
    assert beats[1][1] == 1
    assert beats[1][2] == 0xff


def tb_request(dut):
    yield dut.link_ready.eq(1)
    yield dut.requester_id.eq(REQ_ID)
    yield dut.tlp_tx.ready.eq(1)
    yield dut.tlp_rx.valid.eq(0)
    yield from dut.addr_lo.write(0x0003f000)
    yield from dut.addr_hi.write(1)
    yield from dut.length.write(1024)
    yield from dut.buffer.write(0)
    yield from dut.start.write(1)
    yield from expect_mrd64(dut, 0x1_0003_f000, 1024, 0)


def main():
    dut = ChunkDMA()
    run_simulation(dut, tb_request(dut))
    print("ChunkDMA request simulation OK")


if __name__ == "__main__":
    main()
