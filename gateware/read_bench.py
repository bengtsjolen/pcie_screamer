from migen import *

from litex.soc.interconnect import stream
from litex.soc.interconnect import wishbone
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
        self.tag_ready = CSRStatus(4)
        self.release = CSRStorage(4)
        self.auto_release = CSRStorage(1)
        self.stop = CSRStorage(1)
        self.debug = CSRStatus(64)
        self.mem0 = CSRStatus(64)
        self.first_addr = CSRStatus(11)
        self.first_data = CSRStatus(64)
        self.first_tag = CSRStatus(8)
        self.first_wp_we = CSRStatus(1)
        self.first_test_we = CSRStatus(1)
        self.fifo_seen = CSRStatus(1)
        self.fifo_tag = CSRStatus(2)
        self.fifo_addr = CSRStatus(9)
        self.fifo_data = CSRStatus(64)
        self.fifo_wp_we = CSRStatus(1)
        self.cnt_seen = CSRStatus(1)
        self.cnt_tag = CSRStatus(2)
        self.cnt_count = CSRStatus(11)
        self.cnt_data = CSRStatus(12)
        self.cnt_len = CSRStatus(11)
        self.cnt_new = CSRStatus(12)
        self.test_addr = CSRStorage(9)
        self.test_data = CSRStorage(64)
        self.test_we = CSRStorage(1)

        self.bus = bus = wishbone.Interface(
            data_width=32, address_width=14, addressing="word")

        self.link_ready = Signal()
        self.requester_id = Signal(16)
        self.tlp_rx = stream.Endpoint(phy_layout(64))
        self.submodules.packer = packer = ResetInserter()(TLP32To64Packer())
        self.tlp_tx = packer.source
        sink = packer.sink

        bmem0 = Memory(64, 512)
        bmem1 = Memory(64, 512)
        bmem2 = Memory(64, 512)
        bmem3 = Memory(64, 512)
        wp0 = bmem0.get_port(write_capable=True)
        wp1 = bmem1.get_port(write_capable=True)
        wp2 = bmem2.get_port(write_capable=True)
        wp3 = bmem3.get_port(write_capable=True)
        rp0 = bmem0.get_port()
        rp1 = bmem1.get_port()
        rp2 = bmem2.get_port()
        rp3 = bmem3.get_port()
        self.specials += (bmem0, bmem1, bmem2, bmem3,
                          wp0, wp1, wp2, wp3,
                          rp0, rp1, rp2, rp3)

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
        cpl_data = Signal()
        cpl_status = Signal(3)
        data_tag = Signal(2)
        tag = Signal(8)
        tag_busy_sel = Signal()
        tag_mask = Signal(4)
        cpl_mask = Signal(4)
        timeout = Signal(max=timeout_cycles)
        outstanding = Signal(3)
        tag_ready = Signal(4)
        release_req = Signal(4)
        release_req_valid = Signal()
        stop_req = Signal()
        stop = Signal()
        release_mask = Signal(4)
        stop_mask = Signal(4)
        release_count = Signal(3)
        stop_count = Signal(3)
        release_consumed = Signal()
        stop_consumed = Signal()

        wr0 = Signal(9)
        wr1 = Signal(9)
        wr2 = Signal(9)
        wr3 = Signal(9)
        pending0 = Signal(32)
        pending1 = Signal(32)
        pending2 = Signal(32)
        pending3 = Signal(32)
        pending_valid0 = Signal()
        pending_valid1 = Signal()
        pending_valid2 = Signal()
        pending_valid3 = Signal()
        tag_data0 = Signal(12)
        tag_data1 = Signal(12)
        tag_data2 = Signal(12)
        tag_data3 = Signal(12)
        tag_count0 = Signal(11)
        tag_count1 = Signal(11)
        tag_count2 = Signal(11)
        tag_count3 = Signal(11)
        tag_data_sel = Signal(12)
        tag_count_sel = Signal(11)
        new_count = Signal(12)
        wr_sel = Signal(9)
        pending_sel = Signal(32)
        pending_valid_sel = Signal()
        data_we = Signal()
        dlo_valid = Signal()
        dhi_valid = Signal()
        dlo = Signal(32)
        dhi = Signal(32)
        flush = Signal()
        wr_req = Signal()
        wr_req_addr = Signal(9)
        wr_req_data = Signal(64)
        wr_req_tag = Signal(2)
        rp_dat = Signal(64)
        debug = Signal(64)
        first_addr = Signal(11)
        first_data = Signal(64)
        first_tag = Signal(8)
        first_wp_we = Signal()
        first_test_we = Signal()
        first_seen = Signal()
        fifo_seen = Signal()
        fifo_tag = Signal(2)
        fifo_addr = Signal(9)
        fifo_data = Signal(64)
        fifo_wp_we = Signal()
        cnt_seen = Signal()
        cnt_tag = Signal(2)
        cnt_count = Signal(11)
        cnt_data = Signal(12)
        cnt_len = Signal(11)
        cnt_new = Signal(12)
        cpl_end = Signal()

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
            wr_sel.eq(Array([wr0, wr1, wr2, wr3])[cpl_tag[0:2]]),
            pending_sel.eq(Array([pending0, pending1,
                                  pending2, pending3])[cpl_tag[0:2]]),
            pending_valid_sel.eq(Array([pending_valid0, pending_valid1,
                                        pending_valid2,
                                        pending_valid3])[cpl_tag[0:2]]),
            tag_data_sel.eq(Array([tag_data0, tag_data1,
                                   tag_data2, tag_data3])[cpl_tag[0:2]]),
            tag_count_sel.eq(Array([tag_count0, tag_count1,
                                    tag_count2, tag_count3])[cpl_tag[0:2]]),
            new_count.eq(tag_data_sel + cpl_len),
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
            self.tag_ready.status.eq(tag_ready),
            self.debug.status.eq(debug),
            self.mem0.status.eq(rp0.dat_r),
            self.first_addr.status.eq(first_addr),
            self.first_data.status.eq(first_data),
            self.first_tag.status.eq(first_tag),
            self.first_wp_we.status.eq(first_wp_we),
            self.first_test_we.status.eq(first_test_we),
            self.fifo_seen.status.eq(fifo_seen),
            self.fifo_tag.status.eq(fifo_tag),
            self.fifo_addr.status.eq(fifo_addr),
            self.fifo_data.status.eq(fifo_data),
            self.fifo_wp_we.status.eq(fifo_wp_we),
            self.cnt_seen.status.eq(cnt_seen),
            self.cnt_tag.status.eq(cnt_tag),
            self.cnt_count.status.eq(cnt_count),
            self.cnt_data.status.eq(cnt_data),
            self.cnt_len.status.eq(cnt_len),
            self.cnt_new.status.eq(cnt_new),
            rp0.adr.eq(bus.adr[1:10]),
            rp1.adr.eq(bus.adr[1:10]),
            rp2.adr.eq(bus.adr[1:10]),
            rp3.adr.eq(bus.adr[1:10]),
            rp_dat.eq(Array([rp0.dat_r, rp1.dat_r,
                             rp2.dat_r, rp3.dat_r])[bus.adr[10:12]]),
            release_mask.eq(release_req & tag_ready),
            stop_mask.eq(tag_ready),
            release_count.eq(release_mask[0] + release_mask[1] +
                             release_mask[2] + release_mask[3]),
            stop_count.eq(stop_mask[0] + stop_mask[1] +
                          stop_mask[2] + stop_mask[3]),
            release_consumed.eq(0),
            stop_consumed.eq(0),
            bus.dat_r.eq(Mux(bus.adr[0], rp_dat[32:64],
                             rp_dat[0:32])),
            bus.err.eq(0),
            wp0.adr.eq(Mux(self.test_we.storage,
                           self.test_addr.storage, wr_req_addr)),
            wp1.adr.eq(wr_req_addr),
            wp2.adr.eq(wr_req_addr),
            wp3.adr.eq(wr_req_addr),
            wp0.dat_w.eq(Mux(self.test_we.storage,
                             self.test_data.storage, wr_req_data)),
            wp1.dat_w.eq(wr_req_data),
            wp2.dat_w.eq(wr_req_data),
            wp3.dat_w.eq(wr_req_data),
            wp0.we.eq(self.test_we.storage |
                      (wr_req & (wr_req_tag == 0))),
            wp1.we.eq(wr_req & (wr_req_tag == 1)),
            wp2.we.eq(wr_req & (wr_req_tag == 2)),
            wp3.we.eq(wr_req & (wr_req_tag == 3)),
            data_we.eq(0),
            data_tag.eq(0),
            dlo_valid.eq(0),
            dhi_valid.eq(0),
            dlo.eq(0),
            dhi.eq(0),
            flush.eq(0),
            cpl_end.eq(0),
            sink.valid.eq(0),
            sink.dat.eq(0),
            sink.be.eq(0),
            sink.last.eq(0),
        ]

        wr_idx = [wr0, wr1, wr2, wr3]
        pending = [pending0, pending1, pending2, pending3]
        pending_valid = [pending_valid0, pending_valid1,
                         pending_valid2, pending_valid3]

        def write_dwords(t):
            w = wr_idx[t]
            p = pending[t]
            pv = pending_valid[t]
            return [
                If(dlo_valid & dhi_valid,
                    wr_req.eq(1),
                    wr_req_addr.eq(w),
                    wr_req_tag.eq(t),
                    w.eq(w + 1),
                    If(pv,
                        wr_req_data.eq(Cat(p, dlo)),
                        p.eq(dhi),
                        pv.eq(1),
                    ).Else(
                        wr_req_data.eq(Cat(dlo, dhi)),
                    ),
                ).Elif(dlo_valid,
                    If(pv,
                        wr_req.eq(1),
                        wr_req_addr.eq(w),
                        wr_req_data.eq(Cat(p, dlo)),
                        wr_req_tag.eq(t),
                        w.eq(w + 1),
                        pv.eq(0),
                    ).Else(
                        p.eq(dlo),
                        pv.eq(1),
                    ),
                ).Elif(dhi_valid,
                    If(pv,
                        wr_req.eq(1),
                        wr_req_addr.eq(w),
                        wr_req_data.eq(Cat(p, dhi)),
                        wr_req_tag.eq(t),
                        w.eq(w + 1),
                        pv.eq(0),
                    ).Else(
                        p.eq(dhi),
                        pv.eq(1),
                    ),
                ),
            ]

        self.sync += [
            bus.ack.eq(0),
            If(bus.cyc & bus.stb & ~bus.ack,
                bus.ack.eq(1),
            ),
            If(self.release.re & (self.release.storage != 0),
                release_req.eq(self.release.storage),
                release_req_valid.eq(1),
            ),
            If(self.stop.re & self.stop.storage,
                stop_req.eq(1),
            ),
            If(release_consumed,
                release_req_valid.eq(0),
            ),
            If(stop_consumed,
                stop_req.eq(0),
            ),
            If(self.start.re & self.start.storage & ~busy,
                release_req.eq(0),
                release_req_valid.eq(0),
                stop_req.eq(0),
            ),
            wr_req.eq(0),
            If(data_we,
                Case(data_tag, {
                    t: write_dwords(t) for t in range(4)
                }),
            ).Elif(flush,
                Case(data_tag, {
                    t: [
                        If(pending_valid[t],
                            wr_req.eq(1),
                            wr_req_addr.eq(wr_idx[t]),
                            wr_req_data.eq(Cat(pending[t],
                                               Constant(0, 32))),
                            wr_req_tag.eq(t),
                            wr_idx[t].eq(wr_idx[t] + 1),
                            pending_valid[t].eq(0),
                        ),
                    ] for t in range(4)
                }),
            ),
            If(data_we | flush,
                debug.eq(Cat(data_we, flush, pending_valid_sel, cpl_tag,
                             wr_sel, tag_ready, outstanding)),
            ),
            If(wr_req & ~first_seen,
                first_seen.eq(1),
                first_addr.eq(wr_req_addr),
                first_data.eq(wr_req_data),
                first_tag.eq(wr_req_tag),
                first_wp_we.eq(wp0.we),
                first_test_we.eq(self.test_we.storage),
            ),
            If(~cnt_seen & cpl_end,
                cnt_seen.eq(1),
                cnt_tag.eq(cpl_tag[0:2]),
                cnt_count.eq(tag_count_sel),
                cnt_data.eq(tag_data_sel),
                cnt_len.eq(cpl_len),
                cnt_new.eq(new_count),
            ),
            If(wr_req & ~fifo_seen,
                fifo_seen.eq(1),
                fifo_tag.eq(wr_req_tag),
                fifo_addr.eq(wr_req_addr),
                fifo_data.eq(wr_req_data),
                fifo_wp_we.eq(wp0.we),
            ),
        ]

        self.submodules.fsm = fsm = FSM(reset_state="IDLE")

        fsm.act("IDLE",
            If(self.start.re & self.start.storage & ~busy,
                NextValue(done, 0),
                NextValue(error, 0),
                NextValue(error_code, 0),
                NextValue(outstanding, 0),
                NextValue(first_seen, 0),
                NextValue(fifo_seen, 0),
                NextValue(cnt_seen, 0),
                NextValue(tag_busy, 0),
                NextValue(tag_ready, 0),
                NextValue(stop, 0),
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
            If(stop_req,
                stop_consumed.eq(1),
                NextValue(stop, 1),
                NextValue(tag_busy, tag_busy & ~tag_ready),
                NextValue(tag_ready, 0),
                NextValue(outstanding, outstanding - stop_count),
                If(outstanding == stop_count,
                    NextValue(busy, 0),
                    NextValue(done, 1),
                    NextState("IDLE"),
                ).Else(
                    NextState("WAIT"),
                ),
            ).Elif(release_req_valid,
                release_consumed.eq(1),
                NextValue(tag_busy, tag_busy & ~release_mask),
                NextValue(tag_ready, tag_ready & ~release_mask),
                NextValue(outstanding, outstanding - release_count),
                NextState("ISSUE"),
            ).Elif(next_addr < end_addr,
                If(tag_available,
                    NextValue(req_addr, next_addr),
                    NextValue(req_len, chunk_len),
                    NextValue(req_tag, tag_sel),
                    NextValue(tag_busy, tag_busy | (1 << tag_sel)),
                    NextValue(outstanding, outstanding + 1),
                    NextValue(next_addr, next_addr + chunk_len),
                    NextValue(word, 0),
                    Case(tag_sel, {
                        0: [NextValue(wr0, 0), NextValue(pending_valid0, 0),
                            NextValue(tag_data0, 0),
                            NextValue(tag_count0, chunk_len >> 2)],
                        1: [NextValue(wr1, 0), NextValue(pending_valid1, 0),
                            NextValue(tag_data1, 0),
                            NextValue(tag_count1, chunk_len >> 2)],
                        2: [NextValue(wr2, 0), NextValue(pending_valid2, 0),
                            NextValue(tag_data2, 0),
                            NextValue(tag_count2, chunk_len >> 2)],
                        3: [NextValue(wr3, 0), NextValue(pending_valid3, 0),
                            NextValue(tag_data3, 0),
                            NextValue(tag_count3, chunk_len >> 2)],
                    }),
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
            If(stop_req,
                stop_consumed.eq(1),
                self.tlp_rx.ready.eq(0),
                NextValue(stop, 1),
                NextValue(tag_busy, tag_busy & ~tag_ready),
                NextValue(tag_ready, 0),
                NextValue(outstanding, outstanding - stop_count),
                If(outstanding == stop_count,
                    NextValue(busy, 0),
                    NextValue(done, 1),
                    NextState("IDLE"),
                ).Else(
                    NextState("WAIT"),
                ),
            ).Elif(release_req_valid,
                release_consumed.eq(1),
                self.tlp_rx.ready.eq(0),
                NextValue(tag_busy, tag_busy & ~release_mask),
                NextValue(tag_ready, tag_ready & ~release_mask),
                NextValue(outstanding, outstanding - release_count),
                NextState("ISSUE"),
            ).Elif(self.tlp_rx.valid,
                If((self.tlp_rx.dat[24:32] == 0x4a) |
                   (self.tlp_rx.dat[24:32] == 0x0a),
                    NextValue(cpl_len, Mux(self.tlp_rx.dat[0:10] == 0,
                                          1024, self.tlp_rx.dat[0:10])),
                    NextValue(cpl_data,
                              self.tlp_rx.dat[24:32] == 0x4a),
                    NextValue(cpl_status, self.tlp_rx.dat[45:48]),
                    NextState("CPL_H"),
                ).Else(
                    If(self.tlp_rx.last,
                        NextState("WAIT"),
                    ).Else(
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
            self.tlp_rx.ready.eq(1),
            If(self.tlp_rx.valid & self.tlp_rx.last,
                NextState("WAIT"),
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
                    If(self.tlp_rx.last,
                        NextState("WAIT"),
                    ).Else(
                        NextState("SKIP"),
                    ),
                ).Elif(cpl_status != 0,
                    NextValue(busy, 0),
                    NextValue(error, 1),
                    NextValue(error_code, 2),
                    NextValue(debug, self.tlp_rx.dat),
                    NextState("ERROR"),
                ).Else(
                    NextValue(cpl_tag, tag),
                    If(cpl_data,
                        data_we.eq(1),
                        data_tag.eq(tag[0:2]),
                        dhi_valid.eq(1),
                        dhi.eq(self.tlp_rx.dat[32:64]),
                    ),
                    If(self.tlp_rx.last,
                        NextState("CPL_END"),
                    ).Else(
                        NextState("CPL"),
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

        fsm.act("CPL",
            self.tlp_rx.ready.eq(1),
            If(self.tlp_rx.valid,
                data_we.eq(1),
                data_tag.eq(cpl_tag[0:2]),
                dlo_valid.eq(self.tlp_rx.be[0:4] != 0),
                dhi_valid.eq(self.tlp_rx.be[4:8] != 0),
                dlo.eq(self.tlp_rx.dat[0:32]),
                dhi.eq(self.tlp_rx.dat[32:64]),
                If(self.tlp_rx.last,
                    NextState("CPL_END"),
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

        fsm.act("CPL_END",
            flush.eq(1),
            cpl_end.eq(1),
            data_tag.eq(cpl_tag[0:2]),
            If(new_count < tag_count_sel,
                Case(cpl_tag[0:2], {
                    0: NextValue(tag_data0, new_count),
                    1: NextValue(tag_data1, new_count),
                    2: NextValue(tag_data2, new_count),
                    3: NextValue(tag_data3, new_count),
                }),
                NextState("WAIT"),
            ).Else(
                NextState("COMMIT"),
            ),
        )

        fsm.act("COMMIT",
            If(self.auto_release.storage | stop,
                NextValue(tag_busy, tag_busy & ~cpl_mask),
                NextValue(outstanding, outstanding - 1),
                If((next_addr < end_addr) & ~stop,
                    NextState("ISSUE"),
                ).Elif(outstanding == 1,
                    NextValue(busy, 0),
                    NextValue(done, 1),
                    NextState("IDLE"),
                ).Else(
                    NextState("WAIT"),
                ),
            ).Else(
                NextValue(tag_ready, tag_ready | cpl_mask),
                If(next_addr < end_addr,
                    NextState("ISSUE"),
                ).Else(
                    NextState("WAIT"),
                ),
            ),
        )

        fsm.act("ERROR",
            packer.reset.eq(1),
            NextValue(busy, 0),
            NextValue(error, 1),
            NextState("IDLE"),
        )

