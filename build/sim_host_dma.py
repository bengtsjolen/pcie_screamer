"""Endpoint-initiated DMA: HostDMAScratch -> LitePCIeEndpoint (master port) -> packetizer ->
sys router -> adapter -> core TX. A host model on the core side records MWr payloads and answers
MRd with a CplD. Checks a 32-bit and a 64-bit host address, each written then read back."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from migen import *

# Sim-only shim for write-only FIFO memory ports (see sim_adapter.py).
import migen.fhdl.specials as _specials
_orig_get_port = _specials.Memory.get_port
def _get_port(self, *a, **k):
    port = _orig_get_port(self, *a, **k)
    if getattr(port, "dat_r", None) is None:
        port.dat_r = Signal(self.width)
    return port
_specials.Memory.get_port = _get_port

from litepcie.common import get_bar_mask
from litepcie.core import LitePCIeEndpoint

from gateware.ecp5pcie.config_space import ConfigSpace
from gateware.ecp5pcie.sys_router import TLPSysRouter
from gateware.ecp5pcie.adapter import TLPStreamAdapter
from gateware.ecp5pcie.host_dma import HostDMA


def bswap(x):
    return int.from_bytes((x & 0xffffffff).to_bytes(4, "big"), "little")


class PhyModel:
    data_width = 64
    def __init__(self, router):
        self.sink      = router.phy_sink
        self.source    = router.phy_source
        self.bar0_mask = get_bar_mask(0x1000)
        self.id        = router.id


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
        self.comb += [
            adapter.source.connect(router.sink),
            router.source.connect(adapter.sink),
        ]
        phy = PhyModel(router)
        self.submodules.endpoint = endpoint = LitePCIeEndpoint(phy,
            max_pending_requests=2, address_width=64, endianness="big")
        self.submodules.dma = HostDMA(endpoint, enable=cfg.bus_master)


def cfg_wr(reg, tag, data, be=0xf, bus=1):
    return [0x44000001, (0x0010 << 16) | (tag << 8) | be, (bus << 24) | (reg << 2), bswap(data)]


def main():
    dut = DUT()
    host_mem = {}
    result = {}
    seen = []

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
        """Core-side model: enumerate, then service the endpoint's DMA requests."""
        for _ in range(5):
            yield
        # Enable memory space + bus master (also captures our BDF/id).
        yield from send(cfg_wr(0x01, 1, 0x06, be=0x1))

        for _ in range(4000):
            if (yield dut.tx_valid):
                words = []
                while (yield dut.tx_valid):
                    words.append(bswap((yield dut.tx_data)))
                    yield
                seen.append([f"{w:08x}" for w in words])
                ft = words[0] >> 24
                if ft in (0x40, 0x60):          # MWr32 / MWr64
                    if ft == 0x40:
                        adr, data = words[2], bswap(words[3])
                    else:
                        adr, data = (words[2] << 32) | words[3], bswap(words[4])
                    host_mem[adr] = data
                elif ft in (0x00, 0x20):        # MRd32 / MRd64
                    if ft == 0x00:
                        adr = words[2]
                    else:
                        adr = (words[2] << 32) | words[3]
                    tag = (words[1] >> 8) & 0xff
                    req_id = (yield dut.endpoint.phy.id)
                    val = host_mem.get(adr, 0xA5A5A5A5)
                    cpl = [
                        0x4a000001,
                        (0xac00 << 16) | 4,
                        (req_id << 16) | (tag << 8) | (adr & 0x7f),
                        bswap(val),
                    ]
                    yield from send(cpl)
            else:
                yield

    def control():
        """sys-domain: drive the plain DMA control interface a dword at a time."""
        def op(addr, we, wdata=0):
            yield dut.dma.addr.eq(addr)
            yield dut.dma.wdata.eq(wdata)
            yield dut.dma.we.eq(we)
            yield
            yield dut.dma.start.eq(1)
            yield
            yield dut.dma.start.eq(0)
            done = False
            for _ in range(600):
                if (yield dut.dma.done):
                    done = True
                if not (yield dut.dma.busy) and done:
                    break
                yield
            return (yield dut.dma.rdata)

        # Wait for Bus Master Enable.
        for _ in range(800):
            if (yield dut.cfg.bus_master):
                break
            yield
        result["bme"] = (yield dut.cfg.bus_master)

        for name, addr in (("dma32", 0x1234_5670), ("dma64", 0x1_0000_4000)):
            yield from op(addr, we=1, wdata=0xdeadbeef if name == "dma32" else 0xcafef00d)
            for _ in range(20):
                yield
            rd = yield from op(addr, we=0)
            result[name] = (addr, rd)

    run_simulation(dut,
        {"pcie_rx": [host()], "sys": [control()]},
        clocks={"pcie_rx": 16, "sys": 10})
    print(f"TLPs seen by host ({len(seen)}):")
    for w in seen:
        print("  ", w)

    ok = result.get("bme") == 1
    print(f"bus master enabled: {result.get('bme')}")
    expect = {"dma32": (0x1234_5670, 0xdeadbeef), "dma64": (0x1_0000_4000, 0xcafef00d)}
    print("--- host memory seen by the model ---")
    for a, d in sorted(host_mem.items()):
        print(f"  0x{a:011x} = 0x{d:08x}")
    for name, (addr, want) in expect.items():
        got_addr, rd = result.get(name, (None, None))
        stored = host_mem.get(addr)
        good = stored == want and rd == want
        ok &= good
        print(f"{name}: wrote 0x{want:08x} to 0x{addr:x}; host saw "
              f"{'-' if stored is None else f'0x{stored:08x}'}; read back "
              f"{'-' if rd is None else f'0x{rd:08x}'} {'ok' if good else 'BAD'}")
    print("PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
