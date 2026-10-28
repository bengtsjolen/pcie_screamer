#!/usr/bin/env python3
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from migen import *
from migen.sim import run_simulation

from gateware.tlp_dma import TLPDMA


REQ_ID = 0x1600
STATUS_LINK_READY = 1 << 0
STATUS_DONE       = 1 << 2
STATUS_ERROR      = 1 << 3


def beat(dut, dw0, dw1, be=0xFF, last=0):
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


def cpl(dut, data, addr, tag, status=0, cpl_type=0x4A):
    dw0 = (cpl_type << 24) | (1 if cpl_type == 0x4A else 0)
    dw2 = (REQ_ID << 16) | ((tag & 0xFF) << 8) | (addr & 0x7F)
    yield from beat(dut, dw0, status << 13, last=0)
    if cpl_type == 0x4A:
        yield from beat(dut, dw2, data, be=0xF0, last=1)
    else:
        yield from beat(dut, dw2, 0, be=0x0F, last=1)


def expect_mrd(dut, addr, expected_tag=0, timeout=200):
    beats = []
    for _ in range(2):
        for _ in range(timeout):
            if (yield dut.tlp_tx.valid):
                break
            yield
        else:
            raise AssertionError("TLP beat not emitted")
        beats.append(((yield dut.tlp_tx.dat), (yield dut.tlp_tx.last), (yield dut.tlp_tx.be)))
        yield

    tag = (beats[0][0] >> 40) & 0xFF
    assert tag == expected_tag, f"unexpected tag {tag}, expected {expected_tag}"
    is64 = (addr >> 32) != 0
    dw0 = 0x20000001 if is64 else 0x00000001
    requester_dw1 = (REQ_ID << 16) | (tag << 8) | 0xF
    assert beats[0][0] == ((requester_dw1 << 32) | dw0), hex(beats[0][0])
    assert beats[0][1] == 0
    assert beats[0][2] == 0xFF

    expected_data = (addr & 0xFFFFFFFC) if not is64 else (((addr >> 32) << 32) | (addr & 0xFFFFFFFC))
    assert beats[1][0] == expected_data, hex(beats[1][0])
    assert beats[1][1] == 1
    assert beats[1][2] == (0xFF if is64 else 0x0F)

    return tag


def wait_for_status(dut, mask, timeout=200):
    for _ in range(timeout):
        status = yield from dut.status.read()
        if status & mask:
            return status
        yield
    raise AssertionError(f"status mask 0x{mask:x} not reached")


def read_dword(dut, index):
    yield from dut.rd_index.write(index)
    yield
    yield
    return (yield from dut.rd_data.read())


def tb(dut):
    yield dut.link_ready.eq(1)
    yield dut.requester_id.eq(REQ_ID)
    yield dut.tlp_tx.ready.eq(1)
    yield dut.tlp_rx.valid.eq(0)

    yield from dut.addr_lo.write(0x3F000)
    yield from dut.addr_hi.write(1)
    yield from dut.length.write(2)
    yield from dut.start.write(1)

    # First request: 4 DW header -> 2 x 64-bit beats.
    tag0 = yield from expect_mrd(dut, 0x1_0003_F000)

    # No second request may be issued before the first completion.
    for _ in range(20):
        assert not (yield dut.tlp_tx.valid)
        yield

    yield from cpl(dut, 0x11111111, 0x1_0003_F000, tag0)

    # Second request.
    tag1 = yield from expect_mrd(dut, 0x1_0003_F004, 1)
    yield from cpl(dut, 0x22222222, 0x1_0003_F004, tag1)

    status = yield from wait_for_status(dut, STATUS_DONE | STATUS_ERROR)
    assert status & STATUS_DONE, f"unexpected status 0x{status:x}"
    assert status & STATUS_LINK_READY
    assert not (status & STATUS_ERROR)

    for _ in range(10):
        yield
    status = yield from dut.status.read()
    assert status & STATUS_DONE

    assert (yield from read_dword(dut, 0)) == 0x11111111
    assert (yield from read_dword(dut, 1)) == 0x22222222


def tb_timeout(dut):
    yield dut.link_ready.eq(1)
    yield dut.requester_id.eq(REQ_ID)
    yield dut.tlp_tx.ready.eq(1)
    yield dut.tlp_rx.valid.eq(0)
    yield from dut.addr_lo.write(0x3F000)
    yield from dut.addr_hi.write(0)
    yield from dut.length.write(1)
    yield from dut.start.write(1)
    yield from expect_mrd(dut, 0x3F000)
    status = yield from dut.status.read()
    assert not (status & STATUS_ERROR), "timeout reported before it elapsed"
    yield from wait_for_status(dut, STATUS_ERROR, timeout=200)
    assert (yield from dut.error.read()) == 4


def tb_not_ready(dut):
    yield dut.link_ready.eq(0)
    yield dut.requester_id.eq(REQ_ID)
    yield dut.tlp_tx.ready.eq(1)
    yield dut.tlp_rx.valid.eq(0)
    yield from dut.length.write(1)
    yield from dut.start.write(1)
    for _ in range(20):
        assert not (yield dut.tlp_tx.valid)
        yield
    yield from wait_for_status(dut, STATUS_ERROR, timeout=20)
    assert (yield from dut.error.read()) == 1


def tb_link_drop(dut):
    yield dut.link_ready.eq(1)
    yield dut.requester_id.eq(REQ_ID)
    yield dut.tlp_tx.ready.eq(1)
    yield dut.tlp_rx.valid.eq(0)
    yield from dut.addr_lo.write(0x3F000)
    yield from dut.addr_hi.write(1)
    yield from dut.length.write(2)
    yield from dut.start.write(1)
    yield from expect_mrd(dut, 0x1_0003_F000)

    yield dut.link_ready.eq(0)
    yield from wait_for_status(dut, STATUS_ERROR, timeout=200)
    assert (yield from dut.error.read()) == 1

    for _ in range(20):
        assert not (yield dut.tlp_tx.valid)
        yield


def tb_cpl_error(dut):
    yield dut.link_ready.eq(1)
    yield dut.requester_id.eq(REQ_ID)
    yield dut.tlp_tx.ready.eq(1)
    yield dut.tlp_rx.valid.eq(0)
    yield from dut.addr_lo.write(0x3F000)
    yield from dut.addr_hi.write(0)
    yield from dut.length.write(1)
    yield from dut.start.write(1)
    tag0 = yield from expect_mrd(dut, 0x3F000)
    yield from cpl(dut, 0, 0x3F000, tag0, status=1, cpl_type=0x0A)
    yield from wait_for_status(dut, STATUS_ERROR, timeout=50)
    assert (yield from dut.error.read()) == 2

    yield from dut.addr_lo.write(0x3F004)
    yield from dut.addr_hi.write(0)
    yield from dut.length.write(1)
    yield from dut.start.write(1)
    tag1 = yield from expect_mrd(dut, 0x3F004, 1)
    yield from cpl(dut, 0, 0x3F004, tag1, status=0, cpl_type=0x0A)
    yield from wait_for_status(dut, STATUS_ERROR, timeout=50)
    assert (yield from dut.error.read()) == 3


def tb_invalid_length(dut):
    yield dut.link_ready.eq(1)
    yield dut.requester_id.eq(REQ_ID)
    yield dut.tlp_tx.ready.eq(1)
    yield dut.tlp_rx.valid.eq(0)
    yield from dut.length.write(0)
    yield from dut.start.write(1)
    yield from wait_for_status(dut, STATUS_ERROR, timeout=20)
    assert (yield from dut.error.read()) == 5

    yield from dut.length.write(1025)
    yield from dut.start.write(1)
    yield from wait_for_status(dut, STATUS_ERROR, timeout=20)
    assert (yield from dut.error.read()) == 5


def tb_retry_after_timeout(dut):
    yield dut.link_ready.eq(1)
    yield dut.requester_id.eq(REQ_ID)
    yield dut.tlp_tx.ready.eq(1)
    yield dut.tlp_rx.valid.eq(0)

    yield from dut.addr_lo.write(0x3F000)
    yield from dut.addr_hi.write(0)
    yield from dut.length.write(1)
    yield from dut.start.write(1)
    tag0 = yield from expect_mrd(dut, 0x3F000)
    yield from wait_for_status(dut, STATUS_ERROR, timeout=300)
    assert (yield from dut.error.read()) == 4

    yield from dut.addr_lo.write(0x2000)
    yield from dut.addr_hi.write(0)
    yield from dut.length.write(1)
    yield from dut.start.write(1)
    tag1 = yield from expect_mrd(dut, 0x2000, 1)

    # Stale completion for the old address must be ignored.
    yield from cpl(dut, 0xBAD0BAD0, 0x3F000, tag0)
    yield from cpl(dut, 0, 0x3F000, tag0, status=1, cpl_type=0x0A)
    for _ in range(20):
        status = yield from dut.status.read()
        assert not (status & STATUS_DONE), "stale completion accepted"
        yield

    # Matching completion for the new address must complete.
    yield from cpl(dut, 0x600D600D, 0x2000, tag1)
    status = yield from wait_for_status(dut, STATUS_DONE | STATUS_ERROR)
    assert status & STATUS_DONE
    assert not (status & STATUS_ERROR)
    yield from dut.rd_index.write(0)
    yield
    yield
    assert (yield from dut.rd_data.read()) == 0x600D600D


def tb_unaligned(dut):
    yield dut.link_ready.eq(1)
    yield dut.requester_id.eq(REQ_ID)
    yield dut.tlp_tx.ready.eq(1)
    yield dut.tlp_rx.valid.eq(0)
    yield from dut.addr_lo.write(0x1001)
    yield from dut.addr_hi.write(0)
    yield from dut.length.write(1)
    yield from dut.start.write(1)
    tag0 = yield from expect_mrd(dut, 0x1000)
    yield from cpl(dut, 0xCAFEBABE, 0x1000, tag0)
    status = yield from wait_for_status(dut, STATUS_DONE | STATUS_ERROR)
    assert status & STATUS_DONE
    assert not (status & STATUS_ERROR)
    yield from dut.rd_index.write(0)
    yield
    yield
    assert (yield from dut.rd_data.read()) == 0xCAFEBABE


def tb_link_drop_tag(dut):
    yield dut.link_ready.eq(1)
    yield dut.requester_id.eq(REQ_ID)
    yield dut.tlp_tx.ready.eq(0)
    yield dut.tlp_rx.valid.eq(0)
    yield from dut.addr_lo.write(0x3F000)
    yield from dut.addr_hi.write(0)
    yield from dut.length.write(1)
    yield from dut.start.write(1)
    for _ in range(10):
        yield

    yield dut.link_ready.eq(0)
    yield dut.tlp_tx.ready.eq(1)
    yield from wait_for_status(dut, STATUS_ERROR, timeout=100)
    assert (yield from dut.error.read()) == 1

    yield dut.link_ready.eq(1)
    yield from dut.addr_lo.write(0x2000)
    yield from dut.addr_hi.write(0)
    yield from dut.length.write(1)
    yield from dut.start.write(1)
    tag1 = yield from expect_mrd(dut, 0x2000, 1)
    assert tag1 == 1, f"tag reused after link drop: {tag1}"

    # Stale page-aligned completion from the old request must be dropped.
    yield from cpl(dut, 0xBAD0BAD0, 0x3F000, 0)
    for _ in range(20):
        status = yield from dut.status.read()
        assert not (status & STATUS_DONE), "stale old-tag completion accepted"
        yield

    yield from cpl(dut, 0x600D600D, 0x2000, tag1)
    status = yield from wait_for_status(dut, STATUS_DONE | STATUS_ERROR)
    assert status & STATUS_DONE
    assert not (status & STATUS_ERROR)


def check_verilog():
    from migen.fhdl import verilog
    verilog.convert(TLPDMA())


def main():
    dut = TLPDMA()
    run_simulation(dut, tb(dut))

    dut = TLPDMA(timeout_cycles=20)
    run_simulation(dut, tb_timeout(dut))

    dut = TLPDMA()
    run_simulation(dut, tb_not_ready(dut))

    dut = TLPDMA()
    run_simulation(dut, tb_link_drop(dut))

    dut = TLPDMA()
    run_simulation(dut, tb_link_drop_tag(dut))

    dut = TLPDMA(timeout_cycles=100)
    run_simulation(dut, tb_retry_after_timeout(dut))

    dut = TLPDMA()
    run_simulation(dut, tb_cpl_error(dut))

    dut = TLPDMA()
    run_simulation(dut, tb_invalid_length(dut))

    dut = TLPDMA()
    run_simulation(dut, tb_unaligned(dut))

    check_verilog()

    print("TLPDMA simulation OK")


if __name__ == "__main__":
    main()
