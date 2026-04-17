# This file is Copyright (c) 2016 Florent Kermarrec <florent@enjoy-digital.fr>
# This file is Copyright (c) 2018-2019 Pierre-Olivier Vauboin <po@lambdaconcept.com>
# License: BSD

from migen import *
from migen.fhdl.specials import Tristate

from litex.soc.interconnect import stream
from litex.soc.cores.usb_fifo import phy_description

class FT601Sync(Module):
    def __init__(self, pads, dw=32, timeout=256):
        # NOTE on sizing: migen's AsyncFIFO uses a synchronous-read port
        # (see migen/genlib/fifo.py: rdport = storage.get_port(clock_domain="read"),
        # no async_read=True) so it CAN infer BRAM.  If the design needs
        # a deeper write_fifo to match ufrisk (fifo_32_32_clk1_comtx ≈ 8192
        # entries × 32b ≈ 32 KB = 8 BRAM36), bump the depth directly.  The
        # critical buffers for the 5-page-dump stall live in PCILeechFIFO
        # (tlp_rx_fifo, mux_out_fifo) — those use buffered=True to force
        # BRAM inference.
        read_fifo  = ClockDomainsRenamer({"write": "usb", "read": "sys"})(
            stream.AsyncFIFO(phy_description(dw), 128))
        write_fifo = ClockDomainsRenamer({"write": "sys", "read": "usb"})(
            stream.AsyncFIFO(phy_description(dw), 1024))

        read_buffer = ClockDomainsRenamer("usb")(stream.SyncFIFO(phy_description(dw), 4))
        self.comb += read_buffer.source.connect(read_fifo.sink)

        self.submodules += read_fifo
        self.submodules += read_buffer
        self.submodules += write_fifo

        self.read_buffer = read_buffer

        self.sink = write_fifo.sink
        self.source = read_fifo.source

        self.tdata_w = tdata_w = Signal(dw)
        self.data_r = data_r = Signal(dw)
        self.data_oe = data_oe = Signal()
        self.specials += Tristate(pads.data, tdata_w, data_oe, data_r)

        data_w = Signal(dw)
        _data_w = Signal(dw)
        self.sync.usb += [
            _data_w.eq(data_w)
        ]
        for i in range(dw):
            self.specials += [
                Instance("ODDR",
                         p_DDR_CLK_EDGE="SAME_EDGE",
                         i_C=ClockSignal("usb"), i_CE=1, i_S=0, i_R=0,
                         i_D1=_data_w[i], i_D2=data_w[i], o_Q=tdata_w[i]
                )
            ]

        self.rd_n = rd_n = Signal()
        _rd_n = Signal(reset=1)
        self.wr_n = wr_n = Signal()
        _wr_n = Signal(reset=1)
        self.oe_n = oe_n = Signal()
        _oe_n = Signal(reset=1)
        self.sync.usb += [
            _rd_n.eq(rd_n),
            _wr_n.eq(wr_n),
            _oe_n.eq(oe_n),
        ]
        self.specials += [
            Instance("ODDR",
                     p_DDR_CLK_EDGE="SAME_EDGE",
                     i_C=ClockSignal("usb"), i_CE=1, i_S=0, i_R=0,
                     i_D1=_rd_n, i_D2=rd_n, o_Q=pads.rd_n
            ),
            Instance("ODDR",
                     p_DDR_CLK_EDGE="SAME_EDGE",
                     i_C=ClockSignal("usb"), i_CE=1, i_S=0, i_R=0,
                     i_D1=_wr_n, i_D2=wr_n, o_Q=pads.wr_n
            ),
            Instance("ODDR",
                     p_DDR_CLK_EDGE="SAME_EDGE",
                     i_C=ClockSignal("usb"), i_CE=1, i_S=0, i_R=0,
                     i_D1=_oe_n, i_D2=oe_n, o_Q=pads.oe_n
            )
        ]

        self.comb += [
            pads.rst.eq(~ResetSignal("usb")),
            pads.be.eq(0xf),
            pads.siwua.eq(1),
            data_oe.eq(oe_n),
        ]

        fsm = FSM()
        self.submodules.fsm = ClockDomainsRenamer("usb")(fsm)

        self.tempsendval = tempsendval = Signal(dw)
        self.temptosend = temptosend = Signal()

        self.tempreadval = tempreadval = Signal(dw)
        self.temptoread = temptoread = Signal()

        self.wants_read = wants_read = Signal()
        self.wants_write = wants_write = Signal()
        self.cnt_write = cnt_write = Signal(max=timeout+1)
        self.cnt_read = cnt_read = Signal(max=timeout+1)

        first_write = Signal()

        # Drain-wait counter: when write_fifo empties during WRITE, wait
        # before going IDLE.  This gives the serializer pipeline AND the PCIe
        # root complex time to deliver more data.  Without this, inter-CplD
        # gaps from the root complex cause FT601 to send USB short packets,
        # terminating the transfer prematurely.
        #
        # Under sustained backpressure (large multi-page reads), PCIe flow
        # control credits between tlp_rx_fifo and the root complex introduce
        # gaps that can be tens of µs.  We need drain_wait ≫ any realistic
        # inter-CplD gap, or the transfer splits mid-burst and pcileech waits
        # for completion tags that never arrive.
        #
        # Observed: 9-page dumps work with 1000 cycles; 10-page dumps stall
        # because one gap exceeds 10 µs → short packet → missing CplDs.
        #
        # 10000 cycles = 100 µs @ 100 MHz — spans any realistic root-complex
        # credit refill round-trip, while still small enough to terminate
        # small probe responses (52-byte CFG/CMD reads) quickly at end-of-burst.
        drain_wait_max = 10000
        drain_wait = Signal(max=drain_wait_max + 1)

        # ----------------------------------------------------------------
        # Sync-word filler — mirrors ufrisk's pcileech_ft601.sv lines 87-99.
        #
        # Why this exists: FT601 in 245-Sync mode buffers writes into an
        # internal IN endpoint FIFO and only ships USB packets when the
        # buffer fills to a 1024-byte boundary OR SIWU# is asserted (low).
        # We hold SIWU# high (matching the LiteX reference + ufrisk).  If
        # the FPGA stops writing mid-packet, the tail bytes get stranded
        # inside FT601 — observed as a 14-page MemRd dump pushing 17371 DW
        # to FT601 but the host only receiving 68660 of the 69484 bytes
        # before its 5 s read times out (824 B = ~3 CplDs stuck).
        #
        # Ufrisk's solution: after the data queue stays empty for 15 clk
        # cycles, REINJECT 5 × 0x66665555 sync words.  These drain to the
        # FT601 chip and keep its IN buffer monotonically filling until it
        # naturally crosses a 1024-B USB packet boundary and ships.
        # Because the filler runs forever (whenever queue is idle), no
        # tail can ever stay stranded for long.
        #
        # We do the same: a small counter (sync_idle_max=15) ticks while
        # WRITE has no real data; on overflow we arm `sync_remaining=5`
        # and the WRITE state emits five sync words from a new branch
        # below.  Real data ALWAYS preempts sync filler (the existing
        # `write_fifo.source.valid` Elif branch sits before the new sync
        # branch in the FSM), so filler costs USB bandwidth only when
        # the pipeline is genuinely idle — and even then only briefly,
        # because drain_wait keeps counting through the filler too and
        # eventually returns the FSM to IDLE so RX can be serviced.
        sync_idle_max  = 15
        sync_words_max = 5
        sync_idle      = Signal(max=sync_idle_max + 1)
        sync_remaining = Signal(max=sync_words_max + 1)

        # Idle-side filler timer: counts usb-clk cycles spent in IDLE with
        # no real data.  When it saturates at sync_idle_max, IDLE forces a
        # WRITE re-entry with sync_remaining=5 armed, so filler fires even
        # when wants_write would otherwise be 0 (no real data pending).
        #
        # First attempt had sync_idle accumulation *only* in the WRITE Else
        # branch, which meant: once FT601 raised txe_n even briefly
        # (observed 234 µs of txen_high during a 14-page dump), FSM bounced
        # to IDLE and got stuck there since `wants_write` checks only real
        # data.  Result: diag_ft601_filler_emit = 0 despite 824 B stranded
        # in FT601.  Adding an independent IDLE timer that re-arms the
        # filler mirrors ufrisk's IDLE → TX_WAIT1 transition on
        # `data_queue_count > 0`: his data_cooldown_count continuously
        # refills the queue with sync words, so his IDLE always finds
        # something to send.  We approximate by having IDLE directly kick
        # the FSM back to WRITE for a new filler batch every 15 idle
        # cycles (provided txe_n=0).
        idle_filler_timer = Signal(max=sync_idle_max + 1)

        # ----------------------------------------------------------------
        # Diagnostic counters (exposed via attributes for upstream CDC).
        # Count in the usb domain; upstream resyncs into sys.
        #
        #   diag_filler_emit : # of sync-word writes the filler has driven.
        #   diag_wrn0_accept : # of cycles with wr_n=0 AND txe_n=0 (FT601
        #                      actually sampled a DW).  Sum of real-data
        #                      beats + filler beats (+ temptosend replays).
        #   diag_txen_high   : # of usb-clk cycles observed with txe_n=1
        #                      (chip-full back-pressure from FT601).
        #
        # Subtracting real data sent (= usbtx_seen) from diag_wrn0_accept
        # tells us exactly how many filler+replay beats reached FT601.  If
        # that difference is zero after a dump, the filler never fired.
        # ----------------------------------------------------------------
        self.diag_filler_emit = Signal(16)
        self.diag_wrn0_accept = Signal(16)
        self.diag_txen_high   = Signal(16)
        self.sync.usb += [
            If((sync_remaining != 0) & (wr_n == 0) & (pads.txe_n == 0),
                self.diag_filler_emit.eq(self.diag_filler_emit + 1),
            ),
            If((wr_n == 0) & (pads.txe_n == 0),
                self.diag_wrn0_accept.eq(self.diag_wrn0_accept + 1),
            ),
            If(pads.txe_n,
                self.diag_txen_high.eq(self.diag_txen_high + 1),
            ),
        ]

        self.comb += [
            wants_read.eq(~temptoread & ~pads.rxf_n),
            wants_write.eq((temptosend | write_fifo.source.valid) & (pads.txe_n == 0)),
        ]

        self.fsmstate = Signal(4)
        self.comb += [
            self.fsmstate.eq(Cat(fsm.ongoing("IDLE"),
                                 fsm.ongoing("WRITE"),
                                 fsm.ongoing("RDWAIT"),
                                 fsm.ongoing("READ")))
        ]

        self.sync.usb += [
            If(~fsm.ongoing("READ"),
                If(temptoread,
                    If(read_buffer.sink.ready,
                        temptoread.eq(0)
                    )
                )
            )
        ]
        self.comb += [
            If(~fsm.ongoing("READ"),
                If(temptoread,
                    read_buffer.sink.data.eq(tempreadval),
                    read_buffer.sink.valid.eq(1),
                )
            )
        ]

        fsm.act("IDLE",
            rd_n.eq(1),
            wr_n.eq(1),

            If(wants_write,
                oe_n.eq(1),
                NextValue(cnt_write, 0),
                NextValue(first_write, 1),
                NextValue(drain_wait, 0),
                NextValue(idle_filler_timer, 0),
                NextState("WRITE"),
            ).Elif(wants_read,
                oe_n.eq(0),
                NextValue(idle_filler_timer, 0),
                NextState("RDWAIT")
            ).Elif((idle_filler_timer == sync_idle_max) & (pads.txe_n == 0),
                # No real data, no RX, idle long enough, chip accepts
                # writes: re-enter WRITE with sync_remaining=5 armed so
                # WRITE's Elif(sync_remaining!=0) branch will emit a
                # batch of filler sync words.  Then WRITE's Else branch
                # will count drain_wait up to max and bounce us back
                # here, forming a continuous filler loop like ufrisk's.
                oe_n.eq(1),
                NextValue(cnt_write, 0),
                NextValue(first_write, 1),
                NextValue(drain_wait, 0),
                NextValue(sync_remaining, sync_words_max),
                NextValue(sync_idle, 0),
                NextValue(idle_filler_timer, 0),
                NextState("WRITE"),
            ).Else(
                oe_n.eq(1),
                # Saturate at sync_idle_max — don't wrap, wait for
                # either wants_write/wants_read or txe_n=0 to fire.
                If(idle_filler_timer < sync_idle_max,
                    NextValue(idle_filler_timer, idle_filler_timer + 1),
                ),
            )
        )

        fsm.act("WRITE",
            NextValue(first_write, 0),

            rd_n.eq(1),
            If(pads.txe_n,
                # Host can't accept more: stash the pending word and bounce
                # back to IDLE.  IDLE will decide TX vs RX on re-entry.
                oe_n.eq(1),
                wr_n.eq(1),
                write_fifo.source.ready.eq(0),
                If(write_fifo.source.valid & ~first_write,
                    NextValue(temptosend, 1)
                ),
                NextState("IDLE")
            ).Elif(temptosend,
                oe_n.eq(1),
                data_w.eq(tempsendval),
                wr_n.eq(0),
                NextValue(temptosend, 0)
            ).Elif(write_fifo.source.valid,
                # Active write beat — push a FIFO word to FT601 and reset
                # drain_wait so a transient empty FIFO state doesn't
                # prematurely exit WRITE.  Real data ALSO cancels any
                # pending sync filler: the next sync re-arms after another
                # 15 cycles of idle.
                oe_n.eq(1),
                data_w.eq(write_fifo.source.data),
                write_fifo.source.ready.eq(1),
                NextValue(tempsendval, write_fifo.source.data),
                NextValue(temptosend, 0),
                NextValue(drain_wait, 0),
                NextValue(sync_idle, 0),
                NextValue(sync_remaining, 0),
                wr_n.eq(0),
            ).Elif(sync_remaining != 0,
                # Sync-word filler beat — emit 0x66665555 (five times in a
                # row when armed).  Does NOT reset drain_wait, so the FSM
                # still eventually returns to IDLE for RX servicing even
                # if the pipeline stays idle indefinitely.
                oe_n.eq(1),
                data_w.eq(0x66665555),
                NextValue(tempsendval, 0x66665555),
                NextValue(temptosend, 0),
                NextValue(sync_remaining, sync_remaining - 1),
                NextValue(sync_idle, 0),
                wr_n.eq(0),
            ).Else(
                # write_fifo is empty (source.valid==0 ⇒ FIFO read-side level==0)
                # AND no sync words pending.  Two things happen here:
                #
                #   1. sync_idle counts up; on reaching sync_idle_max we
                #      arm a fresh batch of 5 sync words (next cycle the
                #      sync-emit Elif above will fire).  This is exactly
                #      ufrisk's data_cooldown_count==15 behavior — keeps
                #      FT601's IN buffer monotonically filling so partial
                #      USB packets can't stall.
                #
                #   2. drain_wait counts up regardless of sync activity;
                #      on reaching drain_wait_max we leave for IDLE so
                #      RX (incoming pcileech commands) can be serviced.
                #
                # drain_wait_max = 10000 cycles = 100 µs @ 100 MHz — spans
                # any realistic PCIe root-complex credit-refill round-trip
                # (tens of µs under sustained backpressure from multi-page
                # reads).  Smaller values cause the transfer to split
                # mid-burst when a single gap exceeds the timeout.
                oe_n.eq(1),
                wr_n.eq(1),
                If(sync_idle == sync_idle_max,
                    NextValue(sync_remaining, sync_words_max),
                    NextValue(sync_idle, 0),
                ).Else(
                    NextValue(sync_idle, sync_idle + 1),
                ),
                If(drain_wait < drain_wait_max,
                    NextValue(drain_wait, drain_wait + 1),
                ).Else(
                    NextValue(temptosend, 0),
                    NextValue(sync_idle, 0),
                    NextValue(sync_remaining, 0),
                    NextState("IDLE"),
                )
            )
        )

        fsm.act("RDWAIT",
            rd_n.eq(0),
            oe_n.eq(0),
            wr_n.eq(1),
            NextValue(cnt_read, 0),
            NextState("READ")
        )

        fsm.act("READ",
            If(wants_write,
                NextValue(cnt_read, cnt_read + 1),
            ),

            wr_n.eq(1),
            If(pads.rxf_n,
                oe_n.eq(0),
                rd_n.eq(1),
                NextState("IDLE"),
            ).Elif(cnt_read > timeout,
                NextValue(cnt_write, 0),
                NextValue(first_write, 1),
                NextState("WRITE"),
                oe_n.eq(1),
            ).Else(
                oe_n.eq(0),
                read_buffer.sink.valid.eq(1),
                read_buffer.sink.data.eq(data_r),
                NextValue(tempreadval, data_r),
                If(read_buffer.sink.ready,
                    rd_n.eq(0)
                ).Else(
                    NextValue(temptoread, 1),
                    NextState("IDLE"),
                    rd_n.eq(1)
                )
            )
        )
