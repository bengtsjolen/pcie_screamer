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
    last_be = 0x0f if length == 1 else 0xff
    dw1 = (REQ_ID << 16) | (tag << 8) | last_be
    expected0 = (dw1 << 32) | dw0
    expected1 = ((addr >> 32) << 32) | (addr & 0xfffffffc)

    assert beats[0][0] == expected0, hex(beats[0][0])
    assert beats[0][1] == 0
    assert beats[0][2] == 0xff
    assert beats[1][0] == expected1, hex(beats[1][0])
    assert beats[1][1] == 1
    assert beats[1][2] == 0xff


def expect_no_tx(dut, cycles=10):
    for _ in range(cycles):
        assert not (yield dut.tlp_tx.valid)
        yield


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
    yield from expect_no_tx(dut)


def tb_request_single(dut):
    yield dut.link_ready.eq(1)
    yield dut.requester_id.eq(REQ_ID)
    yield dut.tlp_tx.ready.eq(1)
    yield dut.tlp_rx.valid.eq(0)
    yield from dut.addr_lo.write(0x0003f002)
    yield from dut.addr_hi.write(1)
    yield from dut.length.write(1)
    yield from dut.buffer.write(1)
    yield from dut.start.write(1)
    yield from expect_mrd64(dut, 0x1_0003_f000, 1, 0)
    yield from expect_no_tx(dut)


def beat(dut, dw0, dw1, be=0xff, last=0):
    yield dut.tlp_rx.dat.eq((dw1 << 32) | dw0)
    yield dut.tlp_rx.be.eq(be)
    yield dut.tlp_rx.last.eq(last)
    yield dut.tlp_rx.valid.eq(1)
    yield
    for _ in range(100):
        if (yield dut.tlp_rx.ready):
            break
        yield
    else:
        raise AssertionError("tlp_rx.ready not asserted")
    yield dut.tlp_rx.valid.eq(0)


def cpl_chunk(dut, data, addr, tag):
    assert 1 <= len(data) <= 1024
    length = len(data) & 0x3ff
    dw0 = (0x4a << 24) | length
    dw1 = (len(data) * 4) & 0xfff
    dw2 = (REQ_ID << 16) | ((tag & 0xff) << 8) | (addr & 0x7f)

    beats = [(dw0, dw1, 0xff, 0)]
    beats.append((dw2, data[0], 0xff, len(data) == 1))
    i = 1
    while i + 1 < len(data):
        last = int(i + 2 == len(data))
        beats.append((data[i], data[i + 1], 0xff, last))
        i += 2
    if i < len(data):
        beats.append((data[i], 0, 0x0f, 1))

    for dw0, dw1, be, last in beats:
        yield from beat(dut, dw0, dw1, be, last)


def wait_status(dut, mask, timeout=4000):
    for _ in range(timeout):
        status = yield from dut.status.read()
        if status & mask:
            return status
        yield
    raise AssertionError("status mask not reached")


def read64(dut, buffer, index):
    yield from dut.rd_buffer.write(buffer)
    yield from dut.rd_index.write(index)
    yield
    yield
    return (yield from dut.rd_data.read())


def tb_single(dut):
    data = [0x10000000 + i for i in range(1024)]

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
    yield from cpl_chunk(dut, data, 0x3f000, 0)

    status = yield from wait_status(dut, STATUS_DONE | STATUS_ERROR)
    assert status & STATUS_DONE
    assert not (status & STATUS_ERROR)
    for i in range(512):
        value = yield from read64(dut, 0, i)
        assert (value & 0xffffffff) == data[2 * i]
        assert (value >> 32) == data[2 * i + 1]


def main():
    dut = ChunkDMA()
    run_simulation(dut, tb_request(dut))

    dut = ChunkDMA()
    run_simulation(dut, tb_single(dut))

    print("ChunkDMA simulation OK")


if __name__ == "__main__":
    main()
