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
    last_be = 0x0f if length == 1 else 0xff
    is64 = (addr >> 32) != 0
    dw0 = (0x20000000 if is64 else 0) | length_field
    dw1 = (REQ_ID << 16) | (tag << 8) | last_be
    expected0 = (dw1 << 32) | dw0
    expected1 = (((addr & 0xfffffffc) << 32) | (addr >> 32)
                 if is64 else (addr & 0xfffffffc))

    assert beats[0][0] == expected0, hex(beats[0][0])
    assert beats[0][1] == 0
    assert beats[0][2] == 0xff
    assert beats[1][0] == expected1, hex(beats[1][0])
    assert beats[1][1] == 1
    assert beats[1][2] == (0xff if is64 else 0x0f)


def expect_no_tx(dut, cycles=20):
    for _ in range(cycles):
        assert not (yield dut.tlp_tx.valid)
        yield


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


def tb_cpu_buffer(dut):
    data = [0x60000000 + i for i in range(16)]

    yield dut.link_ready.eq(1)
    yield dut.requester_id.eq(REQ_ID)
    yield dut.tlp_tx.ready.eq(1)
    yield dut.tlp_rx.valid.eq(0)
    yield from dut.addr_lo.write(0x3f000)
    yield from dut.addr_hi.write(0)
    yield from dut.length.write(16)
    yield from dut.buffer.write(0)
    yield from dut.start.write(1)
    yield from expect_mrd64(dut, 0x3f000, 16, 0)
    yield from cpl_chunk(dut, data, 0x3f000, 0)

    status = yield from wait_status(dut, STATUS_DONE | STATUS_ERROR)
    assert status & STATUS_DONE
    assert not (status & STATUS_ERROR)

    for i in range(16):
        value = yield from cpu_read(dut, i)
        assert value == data[i], (i, hex(value), hex(data[i]))


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


def cpl_chunk(dut, data, addr, tag, remaining=None):
    assert 1 <= len(data) <= 1024
    if remaining is None:
        remaining = len(data)
    length = len(data) & 0x3ff
    dw0 = (0x4a << 24) | length
    dw1 = (remaining * 4) & 0xfff
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


def check_buffer(dut, buffer, data):
    assert len(data) % 2 == 0
    for i in range(len(data) // 2):
        value = yield from read64(dut, buffer, i)
        assert (value & 0xffffffff) == data[2 * i]
        assert (value >> 32) == data[2 * i + 1]


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


def tb_split(dut):
    data = [0x20000000 + i for i in range(1024)]
    yield dut.link_ready.eq(1)
    yield dut.requester_id.eq(REQ_ID)
    yield dut.tlp_tx.ready.eq(1)
    yield dut.tlp_rx.valid.eq(0)
    yield from dut.addr_lo.write(0x0003f000)
    yield from dut.addr_hi.write(0)
    yield from dut.length.write(1024)
    yield from dut.buffer.write(0)
    yield from dut.start.write(1)
    yield from expect_mrd64(dut, 0x3f000, 1024, 0)
    yield from cpl_chunk(dut, data[:512], 0x3f000, 0, remaining=1024)
    yield from cpl_chunk(dut, data[512:], 0x3f800, 0, remaining=512)
    status = yield from wait_status(dut, STATUS_DONE | STATUS_ERROR)
    assert status & STATUS_DONE
    assert not (status & STATUS_ERROR)
    for i in range(512):
        value = yield from read64(dut, 0, i)
        assert (value & 0xffffffff) == data[2 * i]
        assert (value >> 32) == data[2 * i + 1]


def tb_split_odd(dut):
    data = [0x28000000 + i for i in range(1024)]
    yield dut.link_ready.eq(1)
    yield dut.requester_id.eq(REQ_ID)
    yield dut.tlp_tx.ready.eq(1)
    yield dut.tlp_rx.valid.eq(0)
    yield from dut.addr_lo.write(0x0003f000)
    yield from dut.addr_hi.write(0)
    yield from dut.length.write(1024)
    yield from dut.buffer.write(0)
    yield from dut.start.write(1)
    yield from expect_mrd64(dut, 0x3f000, 1024, 0)
    yield from cpl_chunk(dut, data[:513], 0x3f000, 0, remaining=1024)
    yield from cpl_chunk(dut, data[513:], 0x3f804, 0, remaining=511)
    status = yield from wait_status(dut, STATUS_DONE | STATUS_ERROR)
    assert status & STATUS_DONE
    assert not (status & STATUS_ERROR)
    yield from check_buffer(dut, 0, data)


def tb_pingpong(dut):
    first = [0x30000000 + i for i in range(1024)]
    second = [0x40000000 + i for i in range(1024)]
    yield dut.link_ready.eq(1)
    yield dut.requester_id.eq(REQ_ID)
    yield dut.tlp_tx.ready.eq(1)
    yield dut.tlp_rx.valid.eq(0)
    yield from dut.addr_lo.write(0x3f000)
    yield from dut.addr_hi.write(0)
    yield from dut.length.write(1024)
    yield from dut.buffer.write(0)
    yield from dut.start.write(1)
    yield from expect_mrd64(dut, 0x3f000, 1024, 0)
    yield from cpl_chunk(dut, first, 0x3f000, 0)
    status = yield from wait_status(dut, STATUS_DONE | STATUS_ERROR)
    assert status & STATUS_DONE
    assert not (status & STATUS_ERROR)

    yield from dut.addr_lo.write(0x40000)
    yield from dut.addr_hi.write(0)
    yield from dut.length.write(1024)
    yield from dut.buffer.write(1)
    yield from dut.start.write(1)
    yield from expect_mrd64(dut, 0x40000, 1024, 1)
    yield from cpl_chunk(dut, second, 0x40000, 1)
    status = yield from wait_status(dut, STATUS_DONE | STATUS_ERROR)
    assert status & STATUS_DONE
    assert not (status & STATUS_ERROR)

    assert (yield from read64(dut, 0, 0)) == ((first[1] << 32) | first[0])
    assert (yield from read64(dut, 1, 0)) == ((second[1] << 32) | second[0])

    value = yield from read64(dut, 0, 256)
    assert value == ((first[513] << 32) | first[512])
    assert value != 0
    value = yield from read64(dut, 1, 256)
    assert value == ((second[513] << 32) | second[512])
    assert value != 0


def tb_partial(dut):
    data = [0x50000000 + i for i in range(16)]
    yield dut.link_ready.eq(1)
    yield dut.requester_id.eq(REQ_ID)
    yield dut.tlp_tx.ready.eq(1)
    yield dut.tlp_rx.valid.eq(0)
    yield from dut.addr_lo.write(0x0003f010)
    yield from dut.addr_hi.write(0)
    yield from dut.length.write(16)
    yield from dut.buffer.write(0)
    yield from dut.start.write(1)
    yield from expect_mrd64(dut, 0x3f010, 16, 0)
    yield from cpl_chunk(dut, data, 0x3f010, 0)
    status = yield from wait_status(dut, STATUS_DONE | STATUS_ERROR)
    assert status & STATUS_DONE
    for i in range(8):
        value = yield from read64(dut, 0, i)
        assert (value & 0xffffffff) == data[2 * i]
        assert (value >> 32) == data[2 * i + 1]


def tb_error(dut):
    yield dut.link_ready.eq(1)
    yield dut.requester_id.eq(REQ_ID)
    yield dut.tlp_tx.ready.eq(1)
    yield dut.tlp_rx.valid.eq(0)
    yield from dut.addr_lo.write(0x3f000)
    yield from dut.addr_hi.write(0)
    yield from dut.length.write(0)
    yield from dut.start.write(1)
    yield from wait_status(dut, STATUS_ERROR)
    assert (yield from dut.error.read()) == 5


def tb_cpl_error(dut):
    yield dut.link_ready.eq(1)
    yield dut.requester_id.eq(REQ_ID)
    yield dut.tlp_tx.ready.eq(1)
    yield dut.tlp_rx.valid.eq(0)
    yield from dut.addr_lo.write(0x3f000)
    yield from dut.addr_hi.write(0)
    yield from dut.length.write(1)
    yield from dut.start.write(1)
    yield from expect_mrd64(dut, 0x3f000, 1, 0)
    yield from beat(dut, (0x4a << 24) | 1, 1 << 13, be=0xff, last=0)
    yield from wait_status(dut, STATUS_ERROR)
    assert (yield from dut.error.read()) == 2


def tb_tag_error(dut):
    yield dut.link_ready.eq(1)
    yield dut.requester_id.eq(REQ_ID)
    yield dut.tlp_tx.ready.eq(1)
    yield dut.tlp_rx.valid.eq(0)
    yield from dut.addr_lo.write(0x3f000)
    yield from dut.addr_hi.write(0)
    yield from dut.length.write(1)
    yield from dut.start.write(1)
    yield from expect_mrd64(dut, 0x3f000, 1, 0)
    yield from cpl_chunk(dut, [0x12345678], 0x3f000, 1)
    yield from wait_status(dut, STATUS_ERROR)
    assert (yield from dut.error.read()) == 9


def start_one(dut, length=1, buffer=0):
    yield dut.link_ready.eq(1)
    yield dut.requester_id.eq(REQ_ID)
    yield dut.tlp_tx.ready.eq(1)
    yield dut.tlp_rx.valid.eq(0)
    yield from dut.addr_lo.write(0x3f000)
    yield from dut.addr_hi.write(0)
    yield from dut.length.write(length)
    yield from dut.buffer.write(buffer)
    yield from dut.start.write(1)


def tb_lower_error(dut):
    yield from start_one(dut)
    yield from expect_mrd64(dut, 0x3f000, 1, 0)
    yield from beat(dut, (0x4a << 24) | 1, 4, be=0xff, last=0)
    yield from beat(dut, (REQ_ID << 16) | 4, 0x11111111,
                    be=0xf0, last=1)
    yield from wait_status(dut, STATUS_ERROR)
    assert (yield from dut.error.read()) == 9


def tb_byte_error(dut):
    yield from start_one(dut)
    yield from expect_mrd64(dut, 0x3f000, 1, 0)
    yield from beat(dut, (0x4a << 24) | 1, 8, be=0xff, last=0)
    yield from wait_status(dut, STATUS_ERROR)
    assert (yield from dut.error.read()) == 7


def tb_requester_error(dut):
    yield from start_one(dut)
    yield from expect_mrd64(dut, 0x3f000, 1, 0)
    yield from beat(dut, (0x4a << 24) | 1, 4, be=0xff, last=0)
    yield from beat(dut, (0x1111 << 16) | 0, 0x11111111,
                    be=0xf0, last=1)
    yield from wait_status(dut, STATUS_ERROR)
    assert (yield from dut.error.read()) == 9


def tb_long_error(dut):
    yield from start_one(dut)
    yield from expect_mrd64(dut, 0x3f000, 1, 0)
    yield from beat(dut, (0x4a << 24) | 2, 4, be=0xff, last=0)
    yield from wait_status(dut, STATUS_ERROR)
    assert (yield from dut.error.read()) == 8


def tb_retry(dut):
    data = [0x60000000 + i for i in range(16)]
    yield from start_one(dut, length=0)
    yield from wait_status(dut, STATUS_ERROR)
    assert (yield from dut.error.read()) == 5
    yield from start_one(dut, length=16)
    yield from expect_mrd64(dut, 0x3f000, 16, 0)
    yield from cpl_chunk(dut, data, 0x3f000, 0)
    status = yield from wait_status(dut, STATUS_DONE | STATUS_ERROR)
    assert status & STATUS_DONE
    assert not (status & STATUS_ERROR)
    yield from check_buffer(dut, 0, data)


def tb_restart_error(dut):
    first = [0x70000000 + i for i in range(16)]
    second = [0x71000000 + i for i in range(16)]
    yield from start_one(dut, length=16)
    yield from expect_mrd64(dut, 0x3f000, 16, 0)
    yield from cpl_chunk(dut, first[:8], 0x3f000, 0, remaining=16)
    yield from beat(dut, (0x4a << 24) | 8, 8 * 4, be=0xff, last=0)
    yield from beat(dut, (REQ_ID << 16) | (1 << 8) | 0x20,
                    first[8], be=0xff, last=0)
    yield from wait_status(dut, STATUS_ERROR)
    assert (yield from dut.error.read()) == 9
    yield from check_buffer(dut, 0, first[:8])

    yield from start_one(dut, length=16)
    yield from expect_mrd64(dut, 0x3f000, 16, 1)
    yield from cpl_chunk(dut, second, 0x3f000, 1)
    status = yield from wait_status(dut, STATUS_DONE | STATUS_ERROR)
    assert status & STATUS_DONE
    assert not (status & STATUS_ERROR)
    yield from check_buffer(dut, 0, second)


def tb_odd_length(dut):
    data = [0x68000000, 0x68000001, 0x68000002]
    yield from start_one(dut, length=3)
    yield from expect_mrd64(dut, 0x3f000, 3, 0)
    yield from cpl_chunk(dut, data, 0x3f000, 0)
    status = yield from wait_status(dut, STATUS_DONE | STATUS_ERROR)
    assert status & STATUS_DONE
    assert not (status & STATUS_ERROR)
    assert (yield from read64(dut, 0, 0)) == \
        ((data[1] << 32) | data[0])
    value = yield from read64(dut, 0, 1)
    assert (value & 0xffffffff) == data[2]
    assert (value >> 32) == 0


def tb_overlength(dut):
    data = [0x72000000 + i for i in range(16)]
    yield from start_one(dut, length=16)
    yield from expect_mrd64(dut, 0x3f000, 16, 0)
    yield from cpl_chunk(dut, data[:8], 0x3f000, 0, remaining=16)
    yield from beat(dut, (0x4a << 24) | 9, 8 * 4, be=0xff, last=0)
    yield from wait_status(dut, STATUS_ERROR)
    assert (yield from dut.error.read()) == 8


def tb_link_loss(dut):
    data = [0x73000000 + i for i in range(16)]
    yield from start_one(dut, length=16)
    yield from expect_mrd64(dut, 0x3f000, 16, 0)
    yield from cpl_chunk(dut, data[:8], 0x3f000, 0, remaining=16)
    yield dut.link_ready.eq(0)
    yield from wait_status(dut, STATUS_ERROR)
    assert (yield from dut.error.read()) == 1


def tb_length_error(dut):
    yield from start_one(dut, length=1025)
    yield from wait_status(dut, STATUS_ERROR)
    assert (yield from dut.error.read()) == 5


def tb_timeout(dut):
    yield dut.link_ready.eq(1)
    yield dut.requester_id.eq(REQ_ID)
    yield dut.tlp_tx.ready.eq(1)
    yield dut.tlp_rx.valid.eq(0)
    yield from dut.addr_lo.write(0x3f000)
    yield from dut.addr_hi.write(0)
    yield from dut.length.write(1)
    yield from dut.start.write(1)
    yield from expect_mrd64(dut, 0x3f000, 1, 0)
    yield from wait_status(dut, STATUS_ERROR, timeout=200)
    assert (yield from dut.error.read()) == 4


def tb_message(dut):
    data = [0x74000000 + i for i in range(16)]
    yield from start_one(dut, length=16)
    yield from expect_mrd64(dut, 0x3f000, 16, 0)
    yield from beat(dut, 0x16000000, 0x11111111, be=0xff, last=0)
    yield from beat(dut, 0x22222222, 0x33333333, be=0xff, last=1)
    yield from cpl_chunk(dut, data, 0x3f000, 0, remaining=16)
    status = yield from wait_status(dut, STATUS_DONE | STATUS_ERROR)
    assert status & STATUS_DONE
    assert not (status & STATUS_ERROR)
    yield from check_buffer(dut, 0, data)


def tb_cpl_ur(dut):
    yield from start_one(dut, length=16)
    yield from expect_mrd64(dut, 0x3f000, 16, 0)
    yield from beat(dut, (0x0a << 24) | 4, 1 << 13, be=0xff, last=0)
    yield from wait_status(dut, STATUS_ERROR)
    assert (yield from dut.error.read()) == 2


def main():
    dut = ChunkDMA()
    run_simulation(dut, tb_request(dut))

    dut = ChunkDMA()
    run_simulation(dut, tb_cpu_buffer(dut))

    dut = ChunkDMA()
    run_simulation(dut, tb_request_single(dut))

    dut = ChunkDMA()
    run_simulation(dut, tb_single(dut))

    dut = ChunkDMA()
    run_simulation(dut, tb_split(dut))
    dut = ChunkDMA()
    run_simulation(dut, tb_split_odd(dut))
    dut = ChunkDMA()
    run_simulation(dut, tb_pingpong(dut))
    dut = ChunkDMA()
    run_simulation(dut, tb_partial(dut))
    dut = ChunkDMA()
    run_simulation(dut, tb_error(dut))
    dut = ChunkDMA()
    run_simulation(dut, tb_cpl_error(dut))
    dut = ChunkDMA()
    run_simulation(dut, tb_tag_error(dut))
    dut = ChunkDMA()
    run_simulation(dut, tb_message(dut))

    dut = ChunkDMA()
    run_simulation(dut, tb_cpl_ur(dut))

    dut = ChunkDMA()
    run_simulation(dut, tb_lower_error(dut))
    dut = ChunkDMA()
    run_simulation(dut, tb_byte_error(dut))
    dut = ChunkDMA()
    run_simulation(dut, tb_requester_error(dut))
    dut = ChunkDMA()
    run_simulation(dut, tb_long_error(dut))
    dut = ChunkDMA()
    run_simulation(dut, tb_retry(dut))
    dut = ChunkDMA()
    run_simulation(dut, tb_restart_error(dut))
    dut = ChunkDMA()
    run_simulation(dut, tb_odd_length(dut))
    dut = ChunkDMA()
    run_simulation(dut, tb_overlength(dut))
    dut = ChunkDMA()
    run_simulation(dut, tb_link_loss(dut))
    dut = ChunkDMA()
    run_simulation(dut, tb_length_error(dut))
    dut = ChunkDMA(timeout_cycles=20)
    run_simulation(dut, tb_timeout(dut))

    print("ChunkDMA simulation OK")


if __name__ == "__main__":
    main()
