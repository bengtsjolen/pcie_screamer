from migen import *

from litex.soc.interconnect import stream
from litex.soc.interconnect.csr import *

from litepcie.common import phy_layout

from gateware.tlp32 import TLP32To64Packer


class ReadBench(Module, AutoCSR):
    def __init__(self, timeout_cycles=1_000_000):
        self.addr_lo = CSRStorage(32)
        self.addr_hi = CSRStorage(32)
        self.length_lo = CSRStorage(32)
        self.length_hi = CSRStorage(32)
        self.start = CSRStorage(1)
        self.status = CSRStatus(4)
        self.error = CSRStatus(4)
        self.outstanding = CSRStatus(3)

        self.link_ready = Signal()
        self.requester_id = Signal(16)
        self.tlp_rx = stream.Endpoint(phy_layout(64))
        self.submodules.packer = packer = ResetInserter()(TLP32To64Packer())
        self.tlp_tx = packer.source
        sink = packer.sink

        busy = Signal()
        done = Signal()
        error = Signal()
        error_code = Signal(4)
        tag_busy = Signal(4)
        tag_free = Signal(4)
        tag_sel = Signal(2)
        tag_available = Signal()
        req_addr = Signal(64)
        req_len = Signal(13)
        req_tag = Signal(8)
        next_addr = Signal(64)
        end_addr = Signal(64)
        end = Signal(65)
        remaining = Signal(64)
        chunk_len = Signal(13)
        count = Signal(11)
        is64 = Signal()
        last_word = Signal(2)
        word = Signal(2)
        cpl_tag = Signal(8)
        cpl_len = Signal(11)
        cpl_done = Signal()
        tag = Signal(8)
        tag_busy_sel = Signal()
        tag_mask = Signal(4)
        cpl_mask = Signal(4)
        timeout = Signal(max=timeout_cycles)
        outstanding = Signal(3)

        base = Signal(64)
        length = Signal(64)

        self.comb += [
            base.eq(Cat(self.addr_lo.storage, self.addr_hi.storage)
                    & 0xFFFFFFFFFFFFFFFC),
            length.eq(Cat(self.length_lo.storage, self.length_hi.storage)),
            end.eq(base + length),
            remaining.eq(end_addr - next_addr),
            chunk_len.eq(Mux(remaining > 4096, 4096, remaining[0:13])),
            count.eq(req_len >> 2),
            is64.eq(req_addr[32:64] != 0),
            last_word.eq(Mux(is64, 3, 2)),
            tag.eq(self.tlp_rx.dat[8:16]),
            tag_mask.eq(Array([1, 2, 4, 8])[tag[0:2]]),
            cpl_mask.eq(Array([1, 2, 4, 8])[cpl_tag[0:2]]),
            tag_busy_sel.eq(Array([tag_busy[0], tag_busy[1],
                                   tag_busy[2], tag_busy[3]])[tag[0:2]]),
            tag_free.eq(~tag_busy),
            tag_available.eq(tag_free != 0),
            tag_sel.eq(0),
            If(tag_free[0],
                tag_sel.eq(0),
            ).Elif(tag_free[1],
                tag_sel.eq(1),
            ).Elif(tag_free[2],
                tag_sel.eq(2),
            ).Elif(tag_free[3],
                tag_sel.eq(3),
            ),
            self.status.status.eq(Cat(self.link_ready, busy, done, error)),
            self.error.status.eq(error_code),
            self.outstanding.status.eq(outstanding),
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
                NextValue(outstanding, 0),
                NextValue(tag_busy, 0),
                NextValue(cpl_tag, 0),
                NextValue(next_addr, base),
                NextValue(end_addr, end[0:64]),
                NextValue(timeout, 0),
                If(~self.link_ready,
                    NextValue(error, 1),
                    NextValue(error_code, 1),
                    NextState("ERROR"),
                ).Elif(end[64] | (length == 0),
                    NextValue(error, 1),
                    NextValue(error_code, 5),
                    NextState("ERROR"),
                ).Else(
                    NextValue(busy, 1),
                    NextState("ISSUE"),
                ),
            )
        )

        fsm.act("ISSUE",
            NextValue(timeout, 0),
            If(next_addr < end_addr,
                If(tag_available,
                    NextValue(req_addr, next_addr),
                    NextValue(req_len, chunk_len),
                    NextValue(req_tag, tag_sel),
                    NextValue(tag_busy, tag_busy | (1 << tag_sel)),
                    NextValue(outstanding, outstanding + 1),
                    NextValue(next_addr, next_addr + chunk_len),
                    NextValue(word, 0),
                    NextState("TX"),
                ).Else(
                    NextState("WAIT"),
                ),
            ).Elif(outstanding == 0,
                NextValue(busy, 0),
                NextValue(done, 1),
                NextState("IDLE"),
            ).Else(
                NextState("WAIT"),
            )
        )

        fsm.act("TX",
            sink.valid.eq(self.link_ready),
            sink.be.eq(0xf),
            Case(word, {
                0: sink.dat.eq(Mux(is64, 0x20000000, 0) |
                               Mux(count == 1024, 0, count)),
                1: sink.dat.eq((self.requester_id << 16) |
                               (req_tag << 8) |
                               Mux(count == 1, 0x0f, 0xff)),
                2: sink.dat.eq(Mux(is64, req_addr[32:64],
                                   req_addr[0:32])),
                3: sink.dat.eq(req_addr[0:32]),
            }),
            If(word == last_word,
                sink.last.eq(1),
            ),
            If(~self.link_ready,
                packer.reset.eq(1),
                NextValue(busy, 0),
                NextValue(error, 1),
                NextValue(error_code, 1),
                NextState("ERROR"),
            ).Elif(sink.ready,
                If(word == last_word,
                    NextState("ISSUE"),
                ).Else(
                    NextValue(word, word + 1),
                ),
            ),
        )

        fsm.act("WAIT",
            self.tlp_rx.ready.eq(1),
            If(self.tlp_rx.valid,
                If((self.tlp_rx.dat[24:32] == 0x4a) |
                   (self.tlp_rx.dat[24:32] == 0x0a),
                    If(self.tlp_rx.dat[45:48] != 0,
                        NextValue(busy, 0),
                        NextValue(error, 1),
                        NextValue(error_code, 2),
                        NextState("ERROR"),
                    ).Else(
                        NextValue(cpl_len, Mux(self.tlp_rx.dat[0:10] == 0,
                                              1024, self.tlp_rx.dat[0:10])),
                        NextValue(cpl_done,
                                  self.tlp_rx.dat[32:44] ==
                                  ((Mux(self.tlp_rx.dat[0:10] == 0, 1024,
                                        self.tlp_rx.dat[0:10]) << 2) &
                                   0xfff)),
                        NextState("CPL_H"),
                    )
                ).Else(
                    NextValue(busy, 0),
                    NextValue(error, 1),
                    NextValue(error_code, 3),
                    NextState("ERROR"),
                )
            ).Elif(~self.link_ready,
                NextValue(busy, 0),
                NextValue(error, 1),
                NextValue(error_code, 1),
                NextState("ERROR"),
            ).Elif(timeout == timeout_cycles - 1,
                NextValue(busy, 0),
                NextValue(error, 1),
                NextValue(error_code, 4),
                NextState("ERROR"),
            ).Else(
                NextValue(timeout, timeout + 1),
            ),
        )

        fsm.act("CPL_H",
            self.tlp_rx.ready.eq(1),
            If(self.tlp_rx.valid,
                If(~tag_busy_sel,
                    NextValue(busy, 0),
                    NextValue(error, 1),
                    NextValue(error_code, 3),
                    NextState("ERROR"),
                ).Elif(self.tlp_rx.last,
                    If(cpl_done,
                        NextValue(tag_busy, tag_busy & ~tag_mask),
                        NextValue(outstanding, outstanding - 1),
                        If(next_addr < end_addr,
                            NextState("ISSUE"),
                        ).Elif(outstanding == 1,
                            NextValue(busy, 0),
                            NextValue(done, 1),
                            NextState("IDLE"),
                        ).Else(
                            NextState("WAIT"),
                        ),
                    ).Else(
                        NextState("WAIT"),
                    ),
                ).Else(
                    NextValue(cpl_tag, tag),
                    NextState("CPL"),
                )
            ).Elif(~self.link_ready,
                NextValue(busy, 0),
                NextValue(error, 1),
                NextValue(error_code, 1),
                NextState("ERROR"),
            ).Elif(timeout == timeout_cycles - 1,
                NextValue(busy, 0),
                NextValue(error, 1),
                NextValue(error_code, 4),
                NextState("ERROR"),
            ).Else(
                NextValue(timeout, timeout + 1),
            ),
        )

        fsm.act("CPL",
            self.tlp_rx.ready.eq(1),
            If(self.tlp_rx.valid,
                If(self.tlp_rx.last,
                    If(cpl_done,
                        NextValue(tag_busy, tag_busy & ~cpl_mask),
                        NextValue(outstanding, outstanding - 1),
                        If(next_addr < end_addr,
                            NextState("ISSUE"),
                        ).Elif(outstanding == 1,
                            NextValue(busy, 0),
                            NextValue(done, 1),
                            NextState("IDLE"),
                        ).Else(
                            NextState("WAIT"),
                        ),
                    ).Else(
                        NextState("WAIT"),
                    ),
                )
            ).Elif(~self.link_ready,
                NextValue(busy, 0),
                NextValue(error, 1),
                NextValue(error_code, 1),
                NextState("ERROR"),
            ).Elif(timeout == timeout_cycles - 1,
                NextValue(busy, 0),
                NextValue(error, 1),
                NextValue(error_code, 4),
                NextState("ERROR"),
            ).Else(
                NextValue(timeout, timeout + 1),
            ),
        )

        fsm.act("ERROR",
            packer.reset.eq(1),
            NextValue(busy, 0),
            NextValue(error, 1),
            NextState("IDLE"),
        )
