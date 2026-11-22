from migen import *
from migen.genlib.cdc import MultiReg

from litex.soc.interconnect import stream
from litex.soc.interconnect.csr import *


CAPTURE_LAYOUT = [("data", 32), ("be", 4), ("dir", 1), ("sop", 1), ("time", 16)]


class TLPCapture(Module, AutoCSR):
    """Records the words on the core's TLP ports (pcie_rx domain) for readout over CSRs.

    Each entry is one word: data (data[7:0] = wire byte 0), per-byte valid (be), direction
    (0 = received, 1 = transmitted), start of TLP and the low 16 bits of a pcie_rx cycle
    counter. Words that find the FIFO full, or both directions active in the same cycle (the
    received word wins), set the sticky overflow bit, which clears while capture is disabled.

    `event` (1 cycle) with `event_data` records an entry with be = 0 once neither port is
    active; an event arriving while one is still pending sets overflow.
    """
    def __init__(self, rx_data, rx_valid, tx_data, tx_valid, active=1, depth=4096,
                 event=0, event_data=0):
        fifo = stream.AsyncFIFO(CAPTURE_LAYOUT, depth, buffered=True)
        fifo = ClockDomainsRenamer({"write": "pcie_rx", "read": "sys"})(fifo)
        self.submodules.fifo = fifo

        self._ctrl  = CSRStorage(fields=[
            CSRField("enable", size=1, reset=1, description="Capture TLP port words"),
        ])
        self._level = CSRStatus(fields=[
            CSRField("valid",    size=1, description="An entry is available in data"),
            CSRField("overflow", size=1, description="Words were dropped"),
        ])
        self._data = CSRStatus(64, description="data[31:0], be[35:32], dir[36], sop[37], time[53:38]")
        self._pop  = CSR()

        enable   = Signal()
        overflow = Signal()
        time     = Signal(16)
        rx_active, tx_active = Signal(), Signal()
        rx_prev,   tx_prev   = Signal(), Signal()
        pending      = Signal()
        pending_data = Signal(32)
        self.specials += MultiReg(self._ctrl.fields.enable, enable, "pcie_rx")
        self.comb += [
            rx_active.eq(rx_valid != 0),
            tx_active.eq(tx_valid != 0),
            fifo.sink.valid.eq(enable & active & (rx_active | tx_active | pending)),
            fifo.sink.time.eq(time),
            If(rx_active,
                fifo.sink.data.eq(rx_data),
                fifo.sink.be.eq(rx_valid),
                fifo.sink.dir.eq(0),
                fifo.sink.sop.eq(~rx_prev),
            ).Elif(tx_active,
                fifo.sink.data.eq(tx_data),
                fifo.sink.be.eq(tx_valid),
                fifo.sink.dir.eq(1),
                fifo.sink.sop.eq(~tx_prev),
            ).Else(
                fifo.sink.data.eq(pending_data),
                fifo.sink.be.eq(0),
                fifo.sink.sop.eq(1),
            ),
        ]
        self.sync.pcie_rx += [
            time.eq(time + 1),
            rx_prev.eq(rx_active),
            tx_prev.eq(tx_active),
            If(~rx_active & ~tx_active, pending.eq(0)),
            If(event,
                pending.eq(1),
                pending_data.eq(event_data),
            ),
            If(~enable,
                overflow.eq(0),
            ).Elif((fifo.sink.valid & ~fifo.sink.ready) | (rx_active & tx_active) |
                   (event & pending & (rx_active | tx_active)),
                overflow.eq(1),
            ),
        ]

        overflow_sys = Signal()
        self.specials += MultiReg(overflow, overflow_sys)
        src = fifo.source
        self.comb += [
            self._level.fields.valid.eq(src.valid),
            self._level.fields.overflow.eq(overflow_sys),
            self._data.status.eq(Cat(src.data, src.be, src.dir, src.sop, src.time)),
            src.ready.eq(self._pop.re),
        ]
