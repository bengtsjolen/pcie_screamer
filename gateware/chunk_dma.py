from migen import *

from litex.soc.interconnect import stream
from litex.soc.interconnect.csr import *

from litepcie.common import phy_layout

from gateware.tlp32 import TLP32To64Packer


class ChunkDMA(Module, AutoCSR):
    def __init__(self, timeout_cycles=1_000_000):
        self.addr_lo = CSRStorage(32)
        self.addr_hi = CSRStorage(32)
        self.length = CSRStorage(11)
        self.buffer = CSRStorage(1)
        self.start = CSRStorage(1)
        self.status = CSRStatus(4)
        self.error = CSRStatus(4)
        self.rd_buffer = CSRStorage(1)
        self.rd_index = CSRStorage(9)
        self.rd_data = CSRStatus(64)

        self.link_ready = Signal()
        self.requester_id = Signal(16)
        self.tlp_rx = stream.Endpoint(phy_layout(64))
        self.submodules.packer = packer = ResetInserter()(TLP32To64Packer())
        self.tlp_tx = packer.source
        sink = packer.sink

        mem0 = Memory(64, 512)
        mem1 = Memory(64, 512)
        wp0 = mem0.get_port(write_capable=True)
        wp1 = mem1.get_port(write_capable=True)
        rp0 = mem0.get_port()
        rp1 = mem1.get_port()
        self.specials += mem0, mem1, wp0, wp1, rp0, rp1

        busy = Signal()
        done = Signal()
        error = Signal()
        error_code = Signal(4)
        tag = Signal(8)
        req_tag = Signal(8)
        addr = Signal(64)
        count = Signal(11)
        word = Signal(2)
        timeout = Signal(max=timeout_cycles)

        wr_buf = Signal()
        wr_en = Signal()
        wr_addr = Signal(9)
        wr_data = Signal(64)

        self.comb += [
            self.status.status.eq(Cat(self.link_ready, busy, done, error)),
            self.error.status.eq(error_code),
            rp0.adr.eq(self.rd_index.storage),
            rp1.adr.eq(self.rd_index.storage),
            self.rd_data.status.eq(Mux(self.rd_buffer.storage,
                                       rp1.dat_r, rp0.dat_r)),
            wp0.adr.eq(wr_addr),
            wp1.adr.eq(wr_addr),
            wp0.dat_w.eq(wr_data),
            wp1.dat_w.eq(wr_data),
            wp0.we.eq(wr_en & ~wr_buf),
            wp1.we.eq(wr_en & wr_buf),
            sink.valid.eq(0),
            sink.dat.eq(0),
            sink.be.eq(0),
            sink.last.eq(0),
        ]

        self.submodules.fsm = fsm = FSM(reset_state="IDLE")

        fsm.act("IDLE",
            If(self.start.re & self.start.storage & ~busy,
                NextValue(done, 0),
                NextValue(error, 0),
                NextValue(error_code, 0),
                If(~self.link_ready,
                    NextValue(error, 1),
                    NextValue(error_code, 1),
                    NextState("ERROR"),
                ).Elif((self.length.storage == 0) |
                       (self.length.storage > 1024),
                    NextValue(error, 1),
                    NextValue(error_code, 5),
                    NextState("ERROR"),
                ).Else(
                    NextValue(busy, 1),
                    NextValue(req_tag, tag),
                    NextValue(tag, tag + 1),
                    NextValue(addr, Cat(self.addr_lo.storage,
                                        self.addr_hi.storage) &
                                      0xFFFFFFFFFFFFFFFC),
                    NextValue(count, self.length.storage),
                    NextValue(wr_buf, self.buffer.storage),
                    NextValue(word, 0),
                    NextState("ISSUE"),
                ),
            )
        )

        fsm.act("ISSUE",
            sink.valid.eq(self.link_ready),
            sink.be.eq(0xf),
            Case(word, {
                0: sink.dat.eq(0x20000000 |
                               Mux(count == 1024, 0, count)),
                1: sink.dat.eq((self.requester_id << 16) |
                               (req_tag << 8) |
                               Mux(count == 1, 0x0f, 0xff)),
                2: sink.dat.eq(addr[0:32]),
                3: sink.dat.eq(addr[32:64]),
            }),
            If(word == 3, sink.last.eq(1)),
            If(~self.link_ready,
                packer.reset.eq(1),
                NextValue(busy, 0),
                NextValue(error, 1),
                NextValue(error_code, 1),
                NextState("ERROR"),
            ).Elif(sink.ready,
                If(word == 3,
                    NextValue(timeout, 0),
                    NextState("CPL_H0"),
                ).Else(
                    NextValue(word, word + 1),
                ),
            ),
        )

        # Completion parsing is added in Task 2.
        fsm.act("CPL_H0", NextState("CPL_H0"))

        fsm.act("ERROR",
            packer.reset.eq(1),
            NextValue(busy, 0),
            NextValue(error, 1),
            NextState("IDLE"),
        )
