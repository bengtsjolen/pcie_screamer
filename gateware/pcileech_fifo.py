from migen import *
from litex.soc.interconnect import stream
from litex.soc.interconnect.stream import SyncFIFO
from litepcie.common import phy_layout

MAGIC       = 0x77
TYPE_TLP    = 0b00
TYPE_CFG    = 0b01
TYPE_LOOP   = 0b10
TYPE_CMD    = 0b11


class PCILeechMux(Module):
    """Migen port of pcileech_mux.sv.

    Important behavioral points copied from SV:
      - p_req is NOT constant-high. It is a registered one-cycle request based on
        rd_en and port priority.
      - p_ctx is only the 2-bit context. The mux itself appends the low 2-bit
        type tag according to the selected port.
      - port priority is fixed: p0 > p1 > p2 > p3.
      - a frame is emitted after 7 data words, or after idle padding.
    """
    def __init__(self, nports=4):
        assert nports == 4, "pcileech_mux.sv only has 4 ports"

        self.p_din      = [Signal(32, name=f"p{i}_din") for i in range(nports)]
        self.p_ctx      = [Signal(2,  name=f"p{i}_ctx") for i in range(nports)]
        self.p_wr       = [Signal(    name=f"p{i}_wr") for i in range(nports)]
        self.p_has_data = [Signal(    name=f"p{i}_has") for i in range(nports)]
        self.p_req      = [Signal(    name=f"p{i}_req") for i in range(nports)]

        self.dout  = Signal(256)
        self.valid = Signal(reset=0)
        self.rd_en = Signal()

        mux_valid        = Signal(reset=0)
        mux_count        = Signal(3, reset=0)
        mux_data         = Signal(224, reset=0)
        mux_status       = Signal(28, reset=0x0fffffff)
        mux_skip_counter = Signal(4, reset=0)

        p_wr_any = Signal()
        self.comb += p_wr_any.eq(self.p_wr[0] | self.p_wr[1] | self.p_wr[2] | self.p_wr[3])
        mux_wr = Signal()
        self.comb += mux_wr.eq(p_wr_any | (mux_skip_counter > 7))

        self.sync += [
            If(ResetSignal(),
                self.valid.eq(0),
                mux_count.eq(0),
                mux_valid.eq(0),
                mux_skip_counter.eq(0),
                *[self.p_req[i].eq(0) for i in range(4)],
            ).Else(
                # request data, exactly like SV priority chain
                self.p_req[0].eq(self.rd_en & self.p_has_data[0]),
                self.p_req[1].eq(self.rd_en & self.p_has_data[1] & ~self.p_has_data[0]),
                self.p_req[2].eq(self.rd_en & self.p_has_data[2] & ~self.p_has_data[1] & ~self.p_has_data[0]),
                self.p_req[3].eq(self.rd_en & self.p_has_data[3] & ~self.p_has_data[2] & ~self.p_has_data[1] & ~self.p_has_data[0]),

                # count / emit logic
                If(mux_wr & (mux_count < 6),
                    mux_valid.eq(0),
                    mux_count.eq(mux_count + 1),
                ).Elif(mux_wr & (mux_count == 6),
                    mux_valid.eq(1),
                    mux_count.eq(0),
                    mux_skip_counter.eq(0),
                ).Elif(mux_count > 0,
                    mux_valid.eq(0),
                    mux_skip_counter.eq(mux_skip_counter + 1),
                ).Else(
                    mux_valid.eq(0),
                ),

                # output register
                self.dout[223:0].eq(mux_data),
                self.dout[227:224].eq(mux_status[0:4]),
                self.dout[231:228].eq(0xE),
                self.dout[235:232].eq(mux_status[8:12]),
                self.dout[239:236].eq(mux_status[4:8]),
                self.dout[243:240].eq(mux_status[16:20]),
                self.dout[247:244].eq(mux_status[12:16]),
                self.dout[251:248].eq(mux_status[24:28]),
                self.dout[255:252].eq(mux_status[20:24]),
                self.valid.eq(mux_valid),

                # selected source write into accumulators; priority exact like SV
                If(self.p_wr[0],
                    mux_status.eq(Cat(Const(0b00, 2), self.p_ctx[0], mux_status[0:24])),
                    mux_data.eq(Cat(self.p_din[0], mux_data[0:192])),
                ),
                If(self.p_wr[1] & ~self.p_wr[0],
                    mux_status.eq(Cat(Const(0b01, 2), self.p_ctx[1], mux_status[0:24])),
                    mux_data.eq(Cat(self.p_din[1], mux_data[0:192])),
                ),
                If(self.p_wr[2] & ~self.p_wr[1] & ~self.p_wr[0],
                    mux_status.eq(Cat(Const(0b10, 2), self.p_ctx[2], mux_status[0:24])),
                    mux_data.eq(Cat(self.p_din[2], mux_data[0:192])),
                ),
                If(self.p_wr[3] & ~self.p_wr[2] & ~self.p_wr[1] & ~self.p_wr[0],
                    mux_status.eq(Cat(Const(0b11, 2), self.p_ctx[3], mux_status[0:24])),
                    mux_data.eq(Cat(self.p_din[3], mux_data[0:192])),
                ),
                If(~self.p_wr[3] & ~self.p_wr[2] & ~self.p_wr[1] & ~self.p_wr[0] & (mux_skip_counter > 7),
                    mux_status.eq(Cat(Const(0b1111, 4), mux_status[0:24])),
                    mux_data.eq(Cat(Const(0xffffffff, 32), mux_data[0:192])),
                ),
            )
        ]


class MuxSerializer(Module):
    def __init__(self):
        self.sink   = stream.Endpoint([("data", 256)])
        self.source = stream.Endpoint([("data", 32)])

        buf   = Signal(256)
        count = Signal(3)
        rsync = Signal(3)

        self.submodules.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            self.sink.ready.eq(1),
            If(self.sink.valid,
                NextValue(buf, self.sink.data),
                NextValue(count, 7),
                NextValue(rsync, 4),
                NextState("RESYNC"),
            )
        )
        fsm.act("RESYNC",
            self.source.valid.eq(1),
            self.source.data.eq(0x66665555),
            If(self.source.ready,
                If(rsync == 0,
                    NextState("SEND"),
                ).Else(
                    NextValue(rsync, rsync - 1),
                )
            )
        )
        fsm.act("SEND",
            self.source.valid.eq(1),
            self.source.data.eq(buf[224:256]),
            If(self.source.ready,
                NextValue(buf, buf << 32),
                If(count == 0,
                    NextState("IDLE"),
                ).Else(
                    NextValue(count, count - 1),
                )
            )
        )


class PCILeechFIFO(Module):
    def __init__(self):
        self.usb_rx = stream.Endpoint([("data", 32)])
        self.usb_tx = stream.Endpoint([("data", 32)])

        self.tlp_rx = stream.Endpoint(phy_layout(32))
        self.tlp_tx = stream.Endpoint(phy_layout(32))

        self.pcie_rst_core   = Signal(reset=0)
        self.pcie_rst_subsys = Signal(reset=0)

        self.phy_lnk_up    = Signal()
        self.phy_ltssm     = Signal(6)
        self.phy_lnk_rate  = Signal()
        self.phy_lnk_width = Signal(2)
        self.phy_id        = Signal(16)
        self.cfg_dcommand  = Signal(16)
        self.tlp_rx_level  = Signal(16)

        # USB RX unpack / resync
        rx64       = Signal(64)
        rx64_valid = Signal()
        rx_lo      = Signal(32)
        rx_phase   = Signal()
        self.comb += self.usb_rx.ready.eq(1)

        rx_word = Signal(32)
        self.comb += rx_word.eq(Cat(
            self.usb_rx.data[24:32],
            self.usb_rx.data[16:24],
            self.usb_rx.data[8:16],
            self.usb_rx.data[0:8],
        ))

        self.sync += [
            rx64_valid.eq(0),
            If(self.usb_rx.valid,
                If(rx_word == 0x66665555,
                    rx_phase.eq(0),
                ).Elif(rx_phase == 0,
                    rx_lo.eq(rx_word),
                    rx_phase.eq(1),
                ).Else(
                    rx64.eq(Cat(rx_word, rx_lo)),
                    rx64_valid.eq(1),
                    rx_phase.eq(0),
                )
            )
        ]

        magic_ok = Signal()
        pkt_type = Signal(2)
        pkt_last = Signal()
        pkt_data = Signal(32)
        self.comb += [
            magic_ok.eq((rx64[0:8] == MAGIC) | (rx64[24:32] == MAGIC)),
            pkt_type.eq(Mux(rx64[24:32] == MAGIC, rx64[16:18], rx64[8:10])),
            pkt_last.eq(Mux(rx64[24:32] == MAGIC, rx64[18], rx64[10])),
            pkt_data.eq(rx64[32:64]),
        ]

        rx_is_tlp  = Signal()
        rx_is_cfg  = Signal()
        rx_is_loop = Signal()
        rx_is_cmd  = Signal()
        self.comb += [
            rx_is_tlp.eq(rx64_valid & magic_ok & (pkt_type == TYPE_TLP)),
            rx_is_cfg.eq(rx64_valid & magic_ok & (pkt_type == TYPE_CFG)),
            rx_is_loop.eq(rx64_valid & magic_ok & (pkt_type == TYPE_LOOP)),
            rx_is_cmd.eq(rx64_valid & magic_ok & (pkt_type == TYPE_CMD)),
        ]

        # host->PCIe TLP
        self.submodules.tlp_tx_fifo = tlp_tx_fifo = SyncFIFO(phy_layout(32), 256)
        tlp_tx_suppress = Signal()
        self.sync += [
            If(rx_is_tlp & pkt_last,
                tlp_tx_suppress.eq(1),
            ).Elif(rx_is_tlp & ~pkt_last & tlp_tx_suppress,
                tlp_tx_suppress.eq(0),
            )
        ]
        pkt_data_swapped = Signal(32)
        self.comb += pkt_data_swapped.eq(Cat(
            pkt_data[24:32], pkt_data[16:24], pkt_data[8:16], pkt_data[0:8]
        ))
        self.comb += [
            tlp_tx_fifo.sink.valid.eq(rx_is_tlp & ~tlp_tx_suppress & self.phy_lnk_up),
            tlp_tx_fifo.sink.dat.eq(pkt_data_swapped),
            tlp_tx_fifo.sink.be.eq(0xf),
            tlp_tx_fifo.sink.last.eq(pkt_last),
            tlp_tx_fifo.source.connect(self.tlp_tx),
        ]

        # PCIe->host TLP, keep current filter for now
        self.submodules.tlp_rx_fifo = tlp_rx_fifo = SyncFIFO(phy_layout(32), 256)
        tlp_filter_first = Signal(reset=1)
        tlp_filter_pass  = Signal(reset=0)
        tlp_is_cpl = Signal()
        self.comb += tlp_is_cpl.eq(
            (self.tlp_rx.dat[25:32] == 0b0000101) |
            (self.tlp_rx.dat[25:32] == 0b0100101)
        )
        self.sync += [
            If(self.tlp_rx.valid & self.tlp_rx.ready,
                tlp_filter_first.eq(self.tlp_rx.last),
                If(tlp_filter_first,
                    tlp_filter_pass.eq(tlp_is_cpl),
                )
            )
        ]
        tlp_rx_gated_valid = Signal()
        self.comb += [
            tlp_rx_gated_valid.eq(self.tlp_rx.valid & Mux(tlp_filter_first, tlp_is_cpl, tlp_filter_pass)),
            tlp_rx_fifo.sink.valid.eq(tlp_rx_gated_valid),
            tlp_rx_fifo.sink.dat.eq(self.tlp_rx.dat),
            tlp_rx_fifo.sink.be.eq(self.tlp_rx.be),
            tlp_rx_fifo.sink.last.eq(self.tlp_rx.last),
            self.tlp_rx.ready.eq(tlp_rx_fifo.sink.ready),
        ]

        rx_seen_count = Signal(16)
        self.sync += If(self.tlp_rx.valid & self.tlp_rx.ready, rx_seen_count.eq(rx_seen_count + 1))
        self.comb += self.tlp_rx_level.eq(Cat(tlp_rx_fifo.level[0:8], rx_seen_count[0:8]))

        # loopback
        self.submodules.loop_fifo = loop_fifo = SyncFIFO([("data", 32), ("ctx", 2)], 64)
        self.comb += [
            loop_fifo.sink.valid.eq(rx_is_loop),
            loop_fifo.sink.data.eq(rx64[32:64]),
            loop_fifo.sink.ctx.eq(rx64[10:12]),
        ]

        # CFG path
        rw = Signal(240)
        self.submodules.cfg_rx_fifo = cfg_rx_fifo = SyncFIFO([("data", 64)], 64)
        self.submodules.cfg_tx_fifo = cfg_tx_fifo = SyncFIFO([("data", 32)], 64)
        self.comb += [
            cfg_rx_fifo.sink.valid.eq(rx_is_cfg),
            cfg_rx_fifo.sink.data.eq(rx64),
        ]
        cfg_cmd = cfg_rx_fifo.source.data
        cfg_cmd_valid = cfg_rx_fifo.source.valid
        cfg_addr_byte = Signal(16)
        cfg_cmd_read = Signal()
        self.comb += [
            cfg_addr_byte.eq(cfg_cmd[16:32]),
            cfg_cmd_read.eq(cfg_cmd_valid & ~ResetSignal()),
            cfg_rx_fifo.source.ready.eq(cfg_cmd_read & cfg_tx_fifo.sink.ready),
        ]
        cfg_word_index = Signal(8)
        cfg_ro_readback = Signal(16)
        self.comb += [cfg_word_index.eq(cfg_addr_byte[1:9]), cfg_ro_readback.eq(0)]
        self.comb += Case(cfg_word_index, {
            0:  cfg_ro_readback.eq(0x6745),
            4:  cfg_ro_readback.eq(0x1600),
            5:  cfg_ro_readback.eq(Cat(self.phy_ltssm, self.phy_lnk_width, self.phy_lnk_up, Const(0, 7))),
            6:  cfg_ro_readback.eq(Cat(self.phy_lnk_width, self.phy_lnk_up, Const(0, 4), self.phy_lnk_rate, Const(0, 8))),
            11: cfg_ro_readback.eq(rw[176:192]),
            12: cfg_ro_readback.eq(self.cfg_dcommand),
        })
        self.comb += [
            cfg_tx_fifo.sink.valid.eq(cfg_cmd_read),
            cfg_tx_fifo.sink.data.eq(Cat(
                cfg_ro_readback[0:8], cfg_ro_readback[8:16],
                cfg_addr_byte[0:8], cfg_addr_byte[8:16],
            )),
            cfg_tx_fifo.sink.last.eq(1),
        ]

        # CMD path
        self.submodules.cmd_rx_fifo = cmd_rx_fifo = SyncFIFO([("data", 64)], 64)
        self.submodules.cmd_tx_fifo = cmd_tx_fifo = SyncFIFO([("data", 32)], 64)
        self.comb += [
            cmd_rx_fifo.sink.valid.eq(rx_is_cmd),
            cmd_rx_fifo.sink.data.eq(rx64),
        ]
        cmd = cmd_rx_fifo.source.data
        cmd_valid = cmd_rx_fifo.source.valid
        in_addr_byte = Signal(16)
        in_mask = Signal(16)
        in_value = Signal(16)
        f_rw = Signal()
        in_cmd_read = Signal()
        in_cmd_write = Signal()
        self.comb += [
            in_addr_byte.eq(cmd[16:32]),
            in_value.eq(Cat(cmd[56:64], cmd[48:56])),
            in_mask.eq(Cat(cmd[40:48], cmd[32:40])),
            f_rw.eq(cmd[31]),
            in_cmd_read.eq(cmd_valid & cmd[12] & ~cmd[30]),
            in_cmd_write.eq(cmd_valid & cmd[13] & ~cmd[30] & f_rw),
        ]

        # rw reset values from SV initialvalues()
        def rw_reset_stmts():
            return [
                rw[0:16].eq(0xEFCD),
                rw[16].eq(0), rw[17].eq(0), rw[18].eq(1), rw[19].eq(0),
                rw[20].eq(0), rw[21].eq(0), rw[31].eq(0),
                rw[32:64].eq(240 >> 3),
                rw[64:96].eq(0), rw[96:128].eq(0),
                rw[128:144].eq(0x10EE), rw[144:160].eq(0x0007),
                rw[160:176].eq(0x10EE), rw[176:192].eq(0x0666),
                rw[192:200].eq(0x02),
                rw[200].eq(1), rw[201].eq(0), rw[202].eq(1), rw[203].eq(1),
                rw[204].eq(1), rw[205].eq(1), rw[206].eq(0), rw[207].eq(0),
                rw[208:224].eq(0), rw[224:233].eq(0),
            ]

        write_cases = {}
        for word_idx in range(15):
            bit_base = word_idx * 16
            stmts = []
            for b in range(16):
                if bit_base + b < 240:
                    stmts.append(If(in_mask[b], rw[bit_base + b].eq(in_value[b])))
            write_cases[word_idx] = stmts

        self.sync += [
            If(ResetSignal(),
                *rw_reset_stmts(),
            ).Elif(in_cmd_write,
                Case(in_addr_byte[1:8], write_cases)
            )
        ]

        rw_readback = Signal(16)
        ro_readback = Signal(16)
        readback = Signal(16)
        self.comb += [rw_readback.eq(0), ro_readback.eq(0), readback.eq(0)]
        rw_rb_cases = {word_idx: rw_readback.eq(rw[word_idx*16:word_idx*16+16]) for word_idx in range(15)}
        self.comb += Case(in_addr_byte[1:8], rw_rb_cases)
        VERSION_MAJOR = 0x04
        VERSION_MINOR = 0x0e
        DEVICE_ID = 0x04
        self.comb += Case(in_addr_byte[1:8], {
            0: ro_readback.eq(0xab89),
            3: ro_readback.eq(self.tlp_rx_level),
            4: ro_readback.eq(Cat(Const(VERSION_MAJOR, 8), Const(VERSION_MINOR, 8))),
            5: ro_readback.eq(Cat(Const(DEVICE_ID, 8), Const(0, 8))),
        })
        self.comb += If(f_rw, readback.eq(rw_readback)).Else(readback.eq(ro_readback))

        self.comb += [
            cmd_tx_fifo.sink.valid.eq(in_cmd_read),
            cmd_tx_fifo.sink.data.eq(Cat(
                readback[0:8], readback[8:16],
                in_addr_byte[0:8], in_addr_byte[8:16],
            )),
            cmd_tx_fifo.sink.last.eq(1),
            cmd_rx_fifo.source.ready.eq(
                (in_cmd_read & cmd_tx_fifo.sink.ready) |
                in_cmd_write |
                (cmd_valid & ~cmd[12] & ~cmd[13])
            ),
        ]

        self.comb += [
            self.pcie_rst_core.eq(rw[200]),
            self.pcie_rst_subsys.eq(rw[201]),
        ]

        # mux + serializer, port order EXACTLY like SV: p0=TLP, p1=CFG, p2=LOOP, p3=CMD
        self.submodules.mux = mux = PCILeechMux(nports=4)
        self.submodules.serializer = serializer = MuxSerializer()
        self.comb += [
            mux.rd_en.eq(serializer.sink.ready),
            serializer.sink.valid.eq(mux.valid),
            serializer.sink.data.eq(mux.dout),
        ]
        tx_swapped = Signal(32)
        self.comb += tx_swapped.eq(Cat(
            serializer.source.data[24:32],
            serializer.source.data[16:24],
            serializer.source.data[8:16],
            serializer.source.data[0:8],
        ))
        self.comb += [
            self.usb_tx.valid.eq(serializer.source.valid),
            self.usb_tx.data.eq(tx_swapped),
            serializer.source.ready.eq(self.usb_tx.ready),
        ]

        self.comb += [
            # p0: TLP RX
            mux.p_din[0].eq(tlp_rx_fifo.source.dat),
            mux.p_ctx[0].eq(Cat(tlp_rx_fifo.source.last, Const(0, 1))),  # {0,last}
            mux.p_wr[0].eq(tlp_rx_fifo.source.valid),
            mux.p_has_data[0].eq(tlp_rx_fifo.source.valid),
            tlp_rx_fifo.source.ready.eq(mux.p_req[0]),

            # p1: CFG
            mux.p_din[1].eq(cfg_tx_fifo.source.data),
            mux.p_ctx[1].eq(0),
            mux.p_wr[1].eq(cfg_tx_fifo.source.valid),
            mux.p_has_data[1].eq(cfg_tx_fifo.source.valid),
            cfg_tx_fifo.source.ready.eq(mux.p_req[1]),

            # p2: loopback
            mux.p_din[2].eq(loop_fifo.source.data),
            mux.p_ctx[2].eq(loop_fifo.source.ctx),
            mux.p_wr[2].eq(loop_fifo.source.valid),
            mux.p_has_data[2].eq(loop_fifo.source.valid),
            loop_fifo.source.ready.eq(mux.p_req[2]),

            # p3: command
            mux.p_din[3].eq(cmd_tx_fifo.source.data),
            mux.p_ctx[3].eq(0),
            mux.p_wr[3].eq(cmd_tx_fifo.source.valid),
            mux.p_has_data[3].eq(cmd_tx_fifo.source.valid),
            cmd_tx_fifo.source.ready.eq(mux.p_req[3]),
        ]
