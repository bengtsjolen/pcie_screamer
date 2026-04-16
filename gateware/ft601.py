# This file is Copyright (c) 2016 Florent Kermarrec <florent@enjoy-digital.fr>
# This file is Copyright (c) 2018-2019 Pierre-Olivier Vauboin <po@lambdaconcept.com>
# License: BSD

from migen import *
from migen.fhdl.specials import Tristate

from litex.soc.interconnect import stream
from litex.soc.cores.usb_fifo import phy_description

class FT601Sync(Module):
    def __init__(self, pads, dw=32, timeout=256):
        # FIFO depths chosen to match ufrisk's PCIeSquirrel reference:
        #   - write_fifo (sys→usb): ufrisk uses fifo_32_32_clk1_comtx with depth 8192
        #     (32 KB).  A typical pcileech USB read is 0x1E000 ≈ 120 KB; having tens
        #     of KB of buffering here means FT601 can sustain a USB transfer across
        #     transient upstream stalls (mux/CDC gaps, PCIe CplD inter-packet gaps)
        #     without emitting a USB short-packet that terminates the host's read
        #     early.
        #   - read_fifo (usb→sys): RX traffic is low-rate (commands + MRd TLPs).
        #     256 entries (1 KB) is plenty.
        read_fifo = ClockDomainsRenamer({"write": "usb", "read": "sys"})(stream.AsyncFIFO(phy_description(dw), 256))
        write_fifo = ClockDomainsRenamer({"write": "sys", "read": "usb"})(stream.AsyncFIFO(phy_description(dw), 8192))

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
        # gaps from the root complex (1-5 µs) cause FT601 to send USB short
        # packets, terminating the transfer prematurely.
        # 1000 cycles = 10 µs at 100 MHz — covers typical inter-CplD gaps.
        drain_wait = Signal(max=1001)  # 0..1000

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
                NextState("WRITE"),
            ).Elif(wants_read,
                oe_n.eq(0),
                NextState("RDWAIT")
            ).Else(
                oe_n.eq(1),
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
                # prematurely exit WRITE.
                oe_n.eq(1),
                data_w.eq(write_fifo.source.data),
                write_fifo.source.ready.eq(1),
                NextValue(tempsendval, write_fifo.source.data),
                NextValue(temptosend, 0),
                NextValue(drain_wait, 0),
                wr_n.eq(0),
            ).Else(
                # write_fifo is empty (source.valid==0 ⇒ FIFO read-side level==0).
                # Hold in WRITE for up to `drain_wait_max` cycles waiting for
                # more upstream data.  This matches ufrisk's architecture:
                # pcileech_ft601.sv TX_ACTIVE does NOT transition directly to
                # RX; it only leaves via TX_COOLDOWN → IDLE.  Any RX servicing
                # happens only after IDLE is re-entered.
                #
                # drain_wait_max = 1000 cycles ≈ 10 µs @ 100 MHz — covers
                # typical inter-CplD gaps from the PCIe root complex and the
                # sys→usb CDC latency through the AsyncFIFO.
                oe_n.eq(1),
                wr_n.eq(1),
                If(drain_wait < 1000,
                    NextValue(drain_wait, drain_wait + 1),
                ).Else(
                    NextValue(temptosend, 0),
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
