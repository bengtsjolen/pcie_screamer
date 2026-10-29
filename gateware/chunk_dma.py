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
        self.debug = CSRStatus(64)

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
        debug = Signal(64)
        tag = Signal(8)
        req_tag = Signal(8)
        addr = Signal(64)
        count = Signal(11)
        is64 = Signal()
        last_word = Signal(2)
        data_count = Signal(11)
        cpl_len = Signal(11)
        cpl_count = Signal(11)
        pending = Signal(32)
        pending_valid = Signal()
        word = Signal(2)
        timeout = Signal(max=timeout_cycles)

        wr_buf = Signal()
        wr_en = Signal()
        wr_addr = Signal(9)
        wr_data = Signal(64)

        self.comb += [
            self.status.status.eq(Cat(self.link_ready, busy, done, error)),
            self.error.status.eq(error_code),
            self.debug.status.eq(debug),
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
            NextValue(wr_en, 0),
            If(self.start.re & self.start.storage & ~busy,
                NextValue(done, 0),
                NextValue(error, 0),
                NextValue(error_code, 0),
                NextValue(data_count, 0),
                NextValue(pending, 0),
                NextValue(pending_valid, 0),
                NextValue(cpl_len, 0),
                NextValue(cpl_count, 0),
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
                    NextValue(is64, self.addr_hi.storage != 0),
                    NextValue(last_word, Mux(self.addr_hi.storage != 0,
                                             3, 2)),
                    NextValue(wr_buf, self.buffer.storage),
                    NextValue(word, 0),
                    NextState("ISSUE"),
                ),
            )
        )

        fsm.act("ISSUE",
            NextValue(wr_en, 0),
            sink.valid.eq(self.link_ready),
            sink.be.eq(0xf),
            Case(word, {
                0: sink.dat.eq(Mux(is64, 0x20000000, 0x00000000) |
                               Mux(count == 1024, 0, count)),
                1: sink.dat.eq((self.requester_id << 16) |
                               (req_tag << 8) |
                               Mux(count == 1, 0x0f, 0xff)),
                2: sink.dat.eq(addr[0:32]),
                3: sink.dat.eq(addr[32:64]),
            }),
            If(word == last_word, sink.last.eq(1)),
            If(~self.link_ready,
                packer.reset.eq(1),
                NextValue(busy, 0),
                NextValue(error, 1),
                NextValue(error_code, 1),
                NextState("ERROR"),
            ).Elif(sink.ready,
                If(word == last_word,
                    NextValue(timeout, 0),
                    NextState("CPL_H0"),
                ).Else(
                    NextValue(word, word + 1),
                ),
            ),
        )

        fsm.act("CPL_H0",
            NextValue(wr_en, 0),
            self.tlp_rx.ready.eq(1),
            If(self.tlp_rx.valid,
                If(self.tlp_rx.dat[24:32] == 0x4a,
                    If(self.tlp_rx.dat[45:48] != 0,
                        NextValue(debug, self.tlp_rx.dat),
                        NextValue(busy, 0),
                        NextValue(error, 1),
                        NextValue(error_code, 2),
                        NextState("ERROR"),
                    ).Elif(data_count >= count,
                        NextValue(debug, self.tlp_rx.dat),
                        NextValue(busy, 0),
                        NextValue(error, 1),
                        NextValue(error_code, 8),
                        NextState("ERROR"),
                    ).Elif(self.tlp_rx.dat[32:44] !=
                           ((count - data_count) << 2)[0:12],
                        NextValue(debug, self.tlp_rx.dat),
                        NextValue(busy, 0),
                        NextValue(error, 1),
                        NextValue(error_code, 7),
                        NextState("ERROR"),
                    ).Elif(Mux(self.tlp_rx.dat[0:10] == 0, 1024,
                               self.tlp_rx.dat[0:10]) >
                           (count - data_count),
                        NextValue(debug, self.tlp_rx.dat),
                        NextValue(busy, 0),
                        NextValue(error, 1),
                        NextValue(error_code, 8),
                        NextState("ERROR"),
                    ).Else(
                        NextValue(cpl_len, Mux(self.tlp_rx.dat[0:10] == 0,
                                               1024, self.tlp_rx.dat[0:10])),
                        NextValue(cpl_count, 0),
                        NextValue(timeout, 0),
                        NextState("CPL_H1"),
                    )
                ).Elif(self.tlp_rx.dat[24:32] == 0x0a,
                    NextValue(debug, self.tlp_rx.dat),
                    NextValue(busy, 0),
                    NextValue(error, 1),
                    NextValue(error_code, Mux(self.tlp_rx.dat[45:48] != 0,
                                              2, 3)),
                    NextState("ERROR"),
                ).Else(
                    If(self.tlp_rx.last,
                        NextState("CPL_H0"),
                    ).Else(
                        NextValue(timeout, 0),
                        NextState("SKIP"),
                    )
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

        fsm.act("SKIP",
            NextValue(wr_en, 0),
            self.tlp_rx.ready.eq(1),
            If(self.tlp_rx.valid & self.tlp_rx.last,
                NextValue(timeout, 0),
                NextState("CPL_H0"),
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

        fsm.act("CPL_H1",
            NextValue(wr_en, 0),
            self.tlp_rx.ready.eq(1),
            If(self.tlp_rx.valid,
                If((self.tlp_rx.dat[0:7] !=
                    (addr + (data_count << 2))[0:7]) |
                   (self.tlp_rx.dat[8:16] != req_tag) |
                   (self.tlp_rx.dat[16:32] != self.requester_id),
                    NextValue(busy, 0),
                    NextValue(debug, self.tlp_rx.dat),
                    NextValue(error, 1),
                    NextValue(error_code, 9),
                    NextState("ERROR"),
                ).Else(
                    NextValue(data_count, data_count + 1),
                    NextValue(cpl_count, 1),
                    If(pending_valid,
                        NextValue(wr_en, 1),
                        NextValue(wr_addr, data_count >> 1),
                        NextValue(wr_data, Cat(pending,
                                               self.tlp_rx.dat[32:64])),
                        NextValue(pending_valid, 0),
                    ).Else(
                        NextValue(pending, self.tlp_rx.dat[32:64]),
                        NextValue(pending_valid, 1),
                    ),
                    If(self.tlp_rx.last,
                        NextState("CPL_END"),
                    ).Else(
                        NextState("CPL_D"),
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

        fsm.act("CPL_D",
            NextValue(wr_en, 0),
            self.tlp_rx.ready.eq(1),
            If(self.tlp_rx.valid,
                If((self.tlp_rx.be[0:4] != 0) & (self.tlp_rx.be[4:8] != 0),
                    If(pending_valid,
                        NextValue(wr_en, 1),
                        NextValue(wr_addr, data_count >> 1),
                        NextValue(wr_data, Cat(pending,
                                               self.tlp_rx.dat[0:32])),
                        NextValue(pending, self.tlp_rx.dat[32:64]),
                        NextValue(pending_valid, 1),
                    ).Else(
                        NextValue(wr_en, 1),
                        NextValue(wr_addr, data_count >> 1),
                        NextValue(wr_data, Cat(self.tlp_rx.dat[0:32],
                                               self.tlp_rx.dat[32:64])),
                        NextValue(pending_valid, 0),
                    ),
                    NextValue(data_count, data_count + 2),
                    NextValue(cpl_count, cpl_count + 2),
                ).Elif(self.tlp_rx.be[0:4] != 0,
                    If(pending_valid,
                        NextValue(wr_en, 1),
                        NextValue(wr_addr, data_count >> 1),
                        NextValue(wr_data, Cat(pending,
                                               self.tlp_rx.dat[0:32])),
                        NextValue(pending_valid, 0),
                    ).Else(
                        NextValue(pending, self.tlp_rx.dat[0:32]),
                        NextValue(pending_valid, 1),
                    ),
                    NextValue(data_count, data_count + 1),
                    NextValue(cpl_count, cpl_count + 1),
                ).Elif(self.tlp_rx.be[4:8] != 0,
                    If(pending_valid,
                        NextValue(wr_en, 1),
                        NextValue(wr_addr, data_count >> 1),
                        NextValue(wr_data, Cat(pending,
                                               self.tlp_rx.dat[32:64])),
                        NextValue(pending_valid, 0),
                    ).Else(
                        NextValue(pending, self.tlp_rx.dat[32:64]),
                        NextValue(pending_valid, 1),
                    ),
                    NextValue(data_count, data_count + 1),
                    NextValue(cpl_count, cpl_count + 1),
                ),
                If(self.tlp_rx.last,
                    NextState("CPL_END"),
                ),
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

        fsm.act("CPL_END",
            NextValue(wr_en, 0),
            If(cpl_count != cpl_len,
                NextValue(debug, Cat(cpl_count, cpl_len, data_count)),
                NextValue(busy, 0),
                NextValue(error, 1),
                NextValue(error_code, 10),
                NextState("ERROR"),
            ).Elif(data_count > count,
                NextValue(debug, Cat(cpl_count, cpl_len, data_count)),
                NextValue(busy, 0),
                NextValue(error, 1),
                NextValue(error_code, 11),
                NextState("ERROR"),
            ).Elif(data_count == count,
                If(pending_valid,
                    NextValue(wr_en, 1),
                    NextValue(wr_addr, data_count >> 1),
                    NextValue(wr_data, Cat(pending, Constant(0, 32))),
                ),
                NextValue(busy, 0),
                NextValue(done, 1),
                NextState("IDLE"),
            ).Else(
                NextValue(timeout, 0),
                NextState("CPL_H0"),
            ),
        )

        fsm.act("ERROR",
            NextValue(wr_en, 0),
            packer.reset.eq(1),
            NextValue(busy, 0),
            NextValue(error, 1),
            NextState("IDLE"),
        )
