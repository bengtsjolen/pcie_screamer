from migen import *

from litex.soc.interconnect import stream
from litex.soc.interconnect.csr import *


def swap32(value):
    return Cat(value[24:32], value[16:24], value[8:16], value[0:8])


class PCILeechUsbBridge(Module, AutoCSR):
    def __init__(self):
        self.fifo_tx = stream.Endpoint([("data", 32)])
        self.fifo_rx = stream.Endpoint([("data", 32)])
        self.link_ready = Signal()

        self.tx_data = CSRStorage(32)
        self.tx_full = CSRStatus()
        self.rx_data = CSRStatus(32)
        self.rx_empty = CSRStatus()
        self.status = CSRStatus(4)

        self.submodules.tx_fifo = tx_fifo = stream.SyncFIFO([("data", 32)], 256)
        self.submodules.rx_fifo = rx_fifo = stream.SyncFIFO([("data", 32)], 1024)

        self.comb += [
            tx_fifo.sink.valid.eq(self.tx_data.re),
            tx_fifo.sink.data.eq(self.tx_data.storage),
            self.tx_full.status.eq(~tx_fifo.sink.ready),

            self.fifo_tx.valid.eq(tx_fifo.source.valid),
            self.fifo_tx.data.eq(swap32(tx_fifo.source.data)),
            tx_fifo.source.ready.eq(self.fifo_tx.ready),

            rx_fifo.sink.valid.eq(self.fifo_rx.valid),
            rx_fifo.sink.data.eq(swap32(self.fifo_rx.data)),
            self.fifo_rx.ready.eq(rx_fifo.sink.ready),

            self.rx_data.status.eq(rx_fifo.source.data),
            rx_fifo.source.ready.eq(self.rx_data.we),
            self.rx_empty.status.eq(~rx_fifo.source.valid),

            self.status.status.eq(Cat(self.link_ready, self.tx_full.status,
                                      self.rx_empty.status, 0)),
        ]
