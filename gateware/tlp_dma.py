from migen import *

from litex.soc.interconnect import stream
from litex.soc.interconnect.csr import *

from litepcie.common import phy_layout

from gateware.tlp32 import TLP32To64Packer


class TLPDMA(Module, AutoCSR):
    def __init__(self, max_dwords=1024, timeout_cycles=1_000_000):
        assert max_dwords == 1024

        self.addr_lo   = CSRStorage(32)
        self.addr_hi   = CSRStorage(32)
        self.length    = CSRStorage(11)
        self.start     = CSRStorage(1)
        self.status    = CSRStatus(4)
        self.error     = CSRStatus(4)
        self.rd_index  = CSRStorage(10)
        self.rd_data   = CSRStatus(32)

        self.link_ready   = Signal()
        self.requester_id = Signal(16)
        self.tlp_rx       = stream.Endpoint(phy_layout(64))

        self.submodules.packer = packer = ResetInserter()(TLP32To64Packer())
        self.tlp_tx = packer.source
        sink = packer.sink

        mem = Memory(32, max_dwords)
        wp = mem.get_port(write_capable=True)
        rp = mem.get_port()
        self.specials += mem, wp, rp

        busy       = Signal()
        done       = Signal()
        error      = Signal()
        error_code = Signal(4)
        cpl_status = Signal(3)
        tag        = Signal(8)
        req_tag    = Signal(8)
        addr       = Signal(64)
        is64       = Signal()
        last_word  = Signal(2)
        count      = Signal(max=max_dwords + 1)
        index      = Signal(max=max_dwords)
        word       = Signal(2)
        rx_beat    = Signal(2)
        timeout    = Signal(max=timeout_cycles)
        rd_reg     = Signal(32)

        self.comb += [
            self.status.status.eq(Cat(self.link_ready, busy, done, error)),
            self.error.status.eq(error_code),
            is64.eq(addr[32:64] != 0),
            last_word.eq(Mux(is64, 3, 2)),
            rp.adr.eq(self.rd_index.storage),
            self.rd_data.status.eq(rd_reg),
            wp.adr.eq(index),
            wp.dat_w.eq(self.tlp_rx.dat[32:64]),
            wp.we.eq(0),
            sink.valid.eq(0),
            sink.dat.eq(0),
            sink.be.eq(0),
            sink.last.eq(0),
        ]

        self.sync += rd_reg.eq(rp.dat_r)

        self.submodules.fsm = fsm = FSM(reset_state="IDLE")

        fsm.act("IDLE",
            If(self.start.re & self.start.storage & ~busy,
                NextValue(done, 0),
                If(~self.link_ready,
                    NextValue(error, 1),
                    NextValue(error_code, 1),
                    NextState("ERROR"),
                ).Elif((self.length.storage == 0) | (self.length.storage > max_dwords),
                    NextValue(error, 1),
                    NextValue(error_code, 5),
                    NextState("ERROR"),
                ).Else(
                    NextValue(error, 0),
                    NextValue(error_code, 0),
                    NextValue(busy, 1),
                    NextValue(req_tag, tag),
                    NextValue(tag, tag + 1),
                    NextValue(addr, Cat(self.addr_lo.storage, self.addr_hi.storage) & 0xFFFFFFFFFFFFFFFC),
                    NextValue(count, self.length.storage),
                    NextValue(index, 0),
                    NextValue(word, 0),
                    NextValue(rx_beat, 0),
                    NextValue(cpl_status, 0),
                    NextState("ISSUE"),
                ),
            )
        )

        fsm.act("ISSUE",
            sink.valid.eq(self.link_ready),
            sink.be.eq(0xF),
            Case(word, {
                0: sink.dat.eq(Mux(is64, 0x20000001, 0x00000001)),
                1: sink.dat.eq((self.requester_id << 16) | (req_tag << 8) | 0xF),
                2: sink.dat.eq(Cat(Constant(0, 2), addr[2:32])),
                3: sink.dat.eq(addr[32:64]),
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
                    NextValue(timeout, 0),
                    NextState("WAIT"),
                ).Else(
                    NextValue(word, word + 1),
                ),
            ),
        )

        fsm.act("WAIT",
            self.tlp_rx.ready.eq(1),
            If(~self.link_ready,
                NextValue(busy, 0),
                NextValue(error, 1),
                NextValue(error_code, Mux(error, error_code, 1)),
                NextState("ERROR"),
            ).Elif(self.tlp_rx.valid,
                If(rx_beat == 0,
                    If(self.tlp_rx.dat[24:32] == 0x4A,
                        If(self.tlp_rx.dat[0:10] == 1,
                            NextValue(cpl_status, self.tlp_rx.dat[45:48]),
                            NextValue(rx_beat, 1),
                            NextValue(timeout, 0),
                        ).Else(
                            NextValue(busy, 0),
                            NextValue(error, 1),
                            NextValue(error_code, 3),
                            NextState("ERROR"),
                        ),
                    ).Elif(self.tlp_rx.dat[24:32] == 0x0A,
                        NextValue(cpl_status, self.tlp_rx.dat[45:48]),
                        NextValue(rx_beat, 2),
                        NextValue(timeout, 0),
                    ).Else(
                        NextValue(busy, 0),
                        NextValue(error, 1),
                        NextValue(error_code, 3),
                        NextState("ERROR"),
                    ),
                ).Elif(rx_beat == 1,
                    If((self.tlp_rx.dat[0:7] != addr[0:7]) |
                       (self.tlp_rx.dat[8:16] != req_tag) |
                       (self.tlp_rx.dat[16:32] != self.requester_id),
                        NextValue(rx_beat, 0),
                        NextValue(cpl_status, 0),
                    ).Elif(cpl_status != 0,
                        NextValue(busy, 0),
                        NextValue(error, 1),
                        NextValue(error_code, 2),
                        NextState("ERROR"),
                    ).Elif((~self.tlp_rx.last) |
                          (self.tlp_rx.be[4:8] != 0xF),
                        NextValue(busy, 0),
                        NextValue(error, 1),
                        NextValue(error_code, 3),
                        NextState("ERROR"),
                    ).Else(
                        wp.we.eq(1),
                        NextValue(rx_beat, 0),
                        NextValue(addr, addr + 4),
                        NextValue(index, index + 1),
                        If(index + 1 == count,
                            NextValue(busy, 0),
                            NextValue(done, 1),
                            NextState("IDLE"),
                        ).Else(
                            NextValue(word, 0),
                            NextValue(req_tag, tag),
                            NextValue(tag, tag + 1),
                            NextState("ISSUE"),
                        ),
                    ),
                ).Else(
                    If((self.tlp_rx.dat[0:7] != addr[0:7]) |
                       (self.tlp_rx.dat[8:16] != req_tag) |
                       (self.tlp_rx.dat[16:32] != self.requester_id),
                        NextValue(rx_beat, 0),
                        NextValue(cpl_status, 0),
                    ).Elif(~self.tlp_rx.last,
                        NextValue(busy, 0),
                        NextValue(error, 1),
                        NextValue(error_code, 3),
                        NextState("ERROR"),
                    ).Else(
                        NextValue(busy, 0),
                        NextValue(error, 1),
                        NextValue(error_code, Mux(cpl_status != 0, 2, 3)),
                        NextState("ERROR"),
                    ),
                ),
            ).Elif(timeout == timeout_cycles - 1,
                NextValue(busy, 0),
                NextValue(error, 1),
                NextValue(error_code, Mux(error, error_code, 4)),
                NextState("ERROR"),
            ).Else(
                NextValue(timeout, timeout + 1),
            ),
        )

        fsm.act("ERROR",
            packer.reset.eq(1),
            NextState("IDLE"),
        )
