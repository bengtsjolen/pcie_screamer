"""TLPDMA on the ECP5 router's raw TLP port: a host CfgWr enables bus master, TLPDMA issues
one MRd32, the host answers with a CplD, and the dword lands in TLPDMA's buffer. This is the
Squirrel V2 data path minus the DCU."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from migen import *

import migen.fhdl.specials as _specials
_orig_get_port = _specials.Memory.get_port
def _get_port(self, *a, **k):
    port = _orig_get_port(self, *a, **k)
    if getattr(port, "dat_r", None) is None:
        port.dat_r = Signal(self.width)
    return port
_specials.Memory.get_port = _get_port

from gateware.ecp5pcie.config_space import ConfigSpace
from gateware.ecp5pcie.sys_router import TLPSysRouter
from gateware.ecp5pcie.adapter import TLPStreamAdapter
from gateware.tlp_dma import TLPDMA


def bswap(x):
    return int.from_bytes((x & 0xffffffff).to_bytes(4, "big"), "little")


class DUT(Module):
    def __init__(self):
        self.rx_data  = Signal(32)
        self.rx_valid = Signal(4)
        self.rx_ready = Signal()
        self.tx_data  = Signal(32)
        self.tx_valid = Signal(4)
        self.tx_ready = Signal(reset=1)
        self.retry    = Signal(8)

        self.submodules.cfg = cfg = ClockDomainsRenamer({"pcie_rx": "sys"})(ConfigSpace())
        self.submodules.router = router = TLPSysRouter(cfg)
        self.submodules.adapter = adapter = TLPStreamAdapter(
            self.rx_data, self.rx_valid, self.rx_ready,
            self.tx_data, self.tx_valid, self.tx_ready, self.retry)
        self.submodules.dma = dma = TLPDMA(timeout_cycles=400)
        self.comb += [
            adapter.source.connect(router.sink),
            router.source.connect(adapter.sink),
            router.phy_source.connect(dma.tlp_rx),
            dma.tlp_tx.connect(router.phy_sink),
            dma.link_ready.eq(cfg.bus_master & (router.id != 0)),
            dma.requester_id.eq(router.id),
        ]


def cfg_wr(reg, tag, data, be=0xf, bus=1):
    return [0x44000001, (0x0010 << 16) | (tag << 8) | be, (bus << 24) | (reg << 2), bswap(data)]


def main():
    dut = DUT()
    result = {}
    seen = []
    payload = 0x41534556

    def set_beat(dat, valid):
        yield dut.rx_data.eq(dat)
        yield dut.rx_valid.eq(valid)

    def send(dwords):
        for _ in range(400):
            if (yield dut.rx_ready):
                break
            yield
        for d in dwords:
            yield from set_beat(bswap(d), 0xf)
            yield
        yield from set_beat(0, 0)
        for _ in range(4):
            yield

    def host():
        for _ in range(5):
            yield
        yield from send(cfg_wr(0x01, 1, 0x06, be=0x1))
        for _ in range(4000):
            if (yield dut.tx_valid):
                words = []
                while (yield dut.tx_valid):
                    words.append(bswap((yield dut.tx_data)))
                    yield
                seen.append(words)
                ft = words[0] >> 24
                if ft in (0x00, 0x20):
                    adr = words[2] if ft == 0x00 else (words[2] << 32) | words[3]
                    tag = (words[1] >> 8) & 0xff
                    req_id = (words[1] >> 16) & 0xffff
                    cpl = [
                        0x4a000001,
                        (0xac00 << 16) | 4,
                        (req_id << 16) | (tag << 8) | (adr & 0x7f),
                        payload,
                    ]
                    yield from send(cpl)
            else:
                yield

    def control():
        for _ in range(800):
            if (yield dut.dma.link_ready):
                break
            yield
        result["ready"] = (yield dut.dma.link_ready)
        result["id"] = (yield dut.router.id)
        yield from dut.dma.addr_lo.write(0x3F000)
        yield from dut.dma.addr_hi.write(0)
        yield from dut.dma.length.write(1)
        yield from dut.dma.start.write(1)
        for _ in range(1500):
            st = yield from dut.dma.status.read()
            if st & 0x8:          # error
                result["status"] = st
                result["error"] = yield from dut.dma.error.read()
                return
            if st & 0x4:          # done
                result["status"] = st
                break
            yield
        yield
        yield
        yield from dut.dma.rd_index.write(0)
        yield
        yield
        result["data"] = yield from dut.dma.rd_data.read()

    run_simulation(dut,
        {"pcie_rx": [host()], "sys": [control()]},
        clocks={"pcie_rx": 16, "sys": 10})

    print(f"TLPs seen by host ({len(seen)}):")
    for w in seen:
        print("  ", [f"{d:08x}" for d in w])
    data = result.get("data")
    ok = (result.get("ready") == 1 and result.get("id") not in (None, 0)
          and (result.get("status", 0) & 0x4) and not (result.get("status", 0) & 0x8)
          and data == payload)
    print(f"link_ready={result.get('ready')} id=0x{result.get('id', 0):04x} "
          f"status=0x{result.get('status', 0):x} error={result.get('error', 0)} "
          f"data={'-' if data is None else f'0x{data:08x}'}")
    print("PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
