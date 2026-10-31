#!/usr/bin/env python3
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from migen import *
from migen.sim import run_simulation

from gateware.read_bench import ReadBench


REQ_ID = 0x1600
STATUS_LINK_READY = 1 << 0
STATUS_DONE = 1 << 2
STATUS_ERROR = 1 << 3


def tx_beat(dut):
    for _ in range(400):
        if (yield dut.tlp_tx.valid):
            break
        yield
    else:
        raise AssertionError("TLP beat not emitted")
    beat = ((yield dut.tlp_tx.dat), (yield dut.tlp_tx.last),
            (yield dut.tlp_tx.be))
    yield
    return beat


def expect_request(dut, addr, tag, dwords=1024):
    b0 = yield from tx_beat(dut)
    b1 = yield from tx_beat(dut)
    is64 = (addr >> 32) != 0
    dw0 = (0x20000000 if is64 else 0) | (0 if dwords == 1024 else dwords)
    dw1 = (REQ_ID << 16) | (tag << 8) | 0xff
    expected0 = (dw1 << 32) | dw0
    expected1 = ((addr & 0xfffffffc) if not is64 else
                 (((addr & 0xfffffffc) << 32) | (addr >> 32)))
    assert b0[0] == expected0, (hex(b0[0]), hex(expected0))
    assert b0[1] == 0
    assert b0[2] == 0xff
    assert b1[0] == expected1, (hex(b1[0]), hex(expected1))
    assert b1[1] == 1
    assert b1[2] == (0xff if is64 else 0x0f)


def rx_beat(dut, data, last, be=0xff):
    yield dut.tlp_rx.dat.eq(data)
    yield dut.tlp_rx.be.eq(be)
    yield dut.tlp_rx.last.eq(last)
    yield dut.tlp_rx.valid.eq(1)
    for _ in range(200):
        yield
        if (yield dut.tlp_rx.ready):
            break
    else:
        raise AssertionError("tlp_rx.ready not asserted")
    yield dut.tlp_rx.valid.eq(0)


def cpl(dut, tag, length=1024, remaining=1024, data=0xdeadbeef):
    dw0 = (0x4a << 24) | (0 if length == 1024 else length)
    dw1 = (remaining * 4) & 0xfff
    dw2 = (REQ_ID << 16) | ((tag & 0xff) << 8)
    yield from rx_beat(dut, (dw1 << 32) | dw0, 0)
    yield from rx_beat(dut, (data << 32) | dw2, 1)


def cpu_read(dut, adr):
    yield dut.bus.sel.eq(0xf)
    yield dut.bus.adr.eq(adr)
    yield dut.bus.cyc.eq(1)
    yield dut.bus.stb.eq(1)
    for _ in range(20):
        yield
        if (yield dut.bus.ack):
            break
    else:
        raise AssertionError("CPU buffer read did not ack")
    data = yield dut.bus.dat_r
    yield dut.bus.cyc.eq(0)
    yield dut.bus.stb.eq(0)
    yield
    return data


def wait_status(dut, mask, timeout=2000):
    for _ in range(timeout):
        status = yield from dut.status.read()
        if status & mask:
            return status
        yield
    raise AssertionError("status mask not reached")



def cpl_data(dut, tag, data):
    length = len(data)
    dw0 = (0x4a << 24) | (0 if length == 1024 else length)
    dw1 = (length * 4) & 0xfff
    dw2 = (REQ_ID << 16) | ((tag & 0xff) << 8)
    yield from rx_beat(dut, (dw1 << 32) | dw0, 0)
    yield from rx_beat(dut, (data[0] << 32) | dw2, length == 1)
    i = 1
    while i + 1 < length:
        last = int(i + 2 == length)
        yield from rx_beat(dut, (data[i + 1] << 32) | data[i], last)
        i += 2
    if i < length:
        yield from rx_beat(dut, data[i], 1, be=0x0f)


def tb_data(dut):
    data = [0x70000000 + i for i in range(16)]
    yield dut.link_ready.eq(1)
    yield dut.requester_id.eq(REQ_ID)
    yield dut.tlp_tx.ready.eq(1)
    yield dut.tlp_rx.valid.eq(0)
    yield from dut.test_addr.write(8)
    yield from dut.test_data.write(0xaaaaaaaaaaaaaaaa)
    yield from dut.test_we.write(1)
    yield from dut.test_we.write(0)
    yield from dut.auto_release.write(1)
    yield from dut.addr_lo.write(0x3f000)
    yield from dut.addr_hi.write(0)
    yield from dut.length_lo.write(0x40)
    yield from dut.length_hi.write(0)
    yield from dut.start.write(1)
    yield from expect_request(dut, 0x3f000, 0, 16)
    yield from cpl_data(dut, 0, data)
    status = yield from wait_status(dut, STATUS_DONE | STATUS_ERROR)
    assert status & STATUS_DONE
    assert not (status & STATUS_ERROR)
    for _ in range(4):
        yield
    for i in range(16):
        value = yield from cpu_read(dut, i)
        assert value == data[i], (i, hex(value), hex(data[i]))
    assert (yield from cpu_read(dut, 16)) == 0xaaaaaaaa
    assert (yield from cpu_read(dut, 17)) == 0xaaaaaaaa

def tb(dut):
    yield dut.link_ready.eq(1)
    yield dut.requester_id.eq(REQ_ID)
    yield dut.tlp_tx.ready.eq(1)
    yield dut.tlp_rx.valid.eq(0)
    yield from dut.auto_release.write(1)
    yield from dut.addr_lo.write(0x3f000)
    yield from dut.addr_hi.write(0)
    yield from dut.length_lo.write(0x8000)
    yield from dut.length_hi.write(0)
    yield from dut.start.write(1)

    for i in range(4):
        yield from expect_request(dut, 0x3f000 + i * 0x1000, i)

    for i, tag in enumerate((2, 0, 1, 3)):
        if tag == 2:
            yield from cpl(dut, tag, 512, 1024, 0x10000002)
            yield from cpl(dut, tag, 512, 512, 0x11000002)
        else:
            yield from cpl(dut, tag, 1024, 1024, 0x10000000 | tag)
        yield from expect_request(dut, 0x43000 + i * 0x1000, tag)

    for tag in (2, 0, 1, 3):
        if tag == 2:
            yield from cpl(dut, tag, 512, 1024, 0x20000002)
            yield from cpl(dut, tag, 512, 512, 0x22000002)
        else:
            yield from cpl(dut, tag, 1024, 1024, 0x20000000 | tag)

    status = yield from wait_status(dut, STATUS_DONE | STATUS_ERROR)
    assert status & STATUS_DONE
    assert not (status & STATUS_ERROR)
    assert (yield from dut.outstanding.read()) == 0
    for _ in range(4):
        yield
    assert (yield from cpu_read(dut, 0x000)) == 0x20000000
    assert (yield from cpu_read(dut, 0x001)) == 0
    assert (yield from cpu_read(dut, 0x400)) == 0x20000001
    assert (yield from cpu_read(dut, 0x401)) == 0
    value = yield from cpu_read(dut, 0x800)
    assert value == 0x20000002, hex(value)
    assert (yield from cpu_read(dut, 0x801)) == 0
    assert (yield from cpu_read(dut, 0x802)) == 0x22000002
    assert (yield from cpu_read(dut, 0x803)) == 0
    assert (yield from cpu_read(dut, 0xC00)) == 0x20000003
    assert (yield from cpu_read(dut, 0xC01)) == 0


def main():
    dut = ReadBench(timeout_cycles=2000)
    run_simulation(dut, tb(dut))

    dut = ReadBench(timeout_cycles=2000)
    run_simulation(dut, tb_data(dut))
    print("ReadBench simulation OK")


if __name__ == "__main__":
    main()
