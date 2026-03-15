# gateware/pcileech_fifo.py
#
# Migen implementation of pcileech_fifo.sv + pcileech_mux.sv (ufrisk)
#
# Sits between:
#   - LiteX FT601Sync (usb_phy): 32-bit word stream in/out
#   - LiteX S7PCIEPHY (pcie_phy): 64-bit AXI-stream TLP in/out
#
# Implements the pcileech wire protocol:
#   RX from USB: 32-bit words → pack to 64-bit → dispatch on magic+type
#     0x77 + type 00 → TLP TX (to PCIe)
#     0x77 + type 01 → CFG (not yet implemented)
#     0x77 + type 10 → LOOPBACK (echo back to USB)
#     0x77 + type 11 → CMD (register file read/write)
#
#   TX to USB: collect 32-bit words from sources → pack 7 into 256-bit frame
#     → serialize back to 8x32 words → FT601
#     Sources (priority order, matches pcileech_mux.sv):
#       p0: LOOPBACK
#       p1: CMD response
#       p2: PCIe CFG response   (not yet implemented)
#       p3-p6: PCIe TLP RX
#
# CMD register file (rw[239:0]) mirrors pcileech_fifo.sv:
#   Key bits:
#     rw[200] = pcie_rst_core   (1 at reset, host clears to bring PCIe online)
#     rw[201] = pcie_rst_subsys
#
# (c) 2025, BSD licence

from migen import *
from litex.soc.interconnect import stream
from litex.soc.interconnect.stream import SyncFIFO
from litepcie.common import phy_layout
from functools import reduce

# ---------------------------------------------------------------------------
# Wire-protocol constants
# ---------------------------------------------------------------------------
MAGIC       = 0x77
TYPE_TLP    = 0b00
TYPE_CFG    = 0b01
TYPE_LOOP   = 0b10
TYPE_CMD    = 0b11

# ---------------------------------------------------------------------------
# PCILeechMux
#
# Collects 32-bit words from up to 8 input ports and packs them 7-at-a-time
# into 256-bit output frames.  Mirrors pcileech_mux.sv exactly.
#
# Frame layout (SV concatenation, MSB first = bit 255 down):
#   [255:252] ctx_reg[1]   [251:248] ctx_reg[0]
#   [247:244] ctx_reg[3]   [243:240] ctx_reg[2]
#   [239:236] ctx_reg[5]   [235:232] ctx_reg[4]
#   [231:228] 4'hE         [227:224] ctx_reg[6]
#   [223:192] data_reg[0]  [191:160] data_reg[1]
#   [159:128] data_reg[2]  [127: 96] data_reg[3]
#   [ 95: 64] data_reg[4]  [ 63: 32] data_reg[5]
#   [ 31:  0] data_reg[6]
#
# ctx tag per word = {ctx[1:0], type_tag[1:0]}  (4 bits)
# ---------------------------------------------------------------------------
class PCILeechMux(Module):
    def __init__(self, nports=8):
        # Per-port signals — caller connects these
        self.p_din  = [Signal(32, name=f"p{i}_din")  for i in range(nports)]
        self.p_ctx  = [Signal(4,  name=f"p{i}_ctx")  for i in range(nports)]
        self.p_wr   = [Signal(    name=f"p{i}_wr")   for i in range(nports)]
        self.p_req  = [Signal(    name=f"p{i}_req")  for i in range(nports)]

        # 256-bit output frame
        self.dout   = Signal(256)
        self.valid  = Signal()
        self.rd_en  = Signal()   # driven by downstream (serializer IDLE)

        # -------------------------------------------------------------------
        # Internal state — 15 slots (SV uses data_reg[14], indices 0..13)
        # -------------------------------------------------------------------
        DEPTH    = 15
        data_reg = Array([Signal(32, name=f"dr{i}") for i in range(DEPTH)])
        ctx_reg  = Array([Signal(4,  name=f"cr{i}") for i in range(DEPTH)])

        idx_base    = Signal(4, reset=0)
        idle_count  = Signal(22, reset=0)
        en          = Signal()          # always 1 after reset - mux runs freely
        dout_valid  = Signal()
        dout_buf_valid = Signal()
        dout_buf_data  = Signal(256)

        self.sync += en.eq(~ResetSignal())

        # -------------------------------------------------------------------
        # Priority-index chain (combinational)
        # p_idx[i] = slot index where port i will write its word
        # -------------------------------------------------------------------
        p_idx = [Signal(4, name=f"pidx{i}") for i in range(nports + 1)]
        self.comb += p_idx[0].eq(idx_base)
        for i in range(nports):
            self.comb += p_idx[i+1].eq(p_idx[i] + self.p_wr[i])

        # Per-port pending signals — set high when FIFO has data not yet in a mux
        # slot AND mux is free. Suppresses idle padding to let back-to-back
        # responses (e.g. batched 000a/000c reads) accumulate in the same frame.
        self.p_pending = [Signal(name=f"p{i}_pending") for i in range(nports)]
        any_pending = Signal()
        self.comb += any_pending.eq(reduce(lambda a, b: a | b, self.p_pending))

        # Idle port — pads frame with 0xFFFFFFFF when stalled.
        # Threshold of 64 cycles (~430ns@150MHz): long enough for back-to-back
        # USB responses to both arrive, short enough to not delay single responses.
        idle_idx = Signal(4)
        idle_wr  = Signal()
        self.comb += [
            idle_idx.eq(p_idx[nports]),
            # Two-tier idle emission:
            # - With data (idle_idx > 0): emit after 1000 cycles (~6.7µs)
            #   Ensures CMD/CFG responses are delivered promptly.
            # - Without data (idle_idx == 0): emit after 3M cycles (~20ms)
            #   Occasional empty frame keeps FT_ReadPipe from timing out
            #   without flooding the FT601 TX FIFO.
            idle_wr .eq(en & (idle_idx < 7) & (
                ((idle_count > 1000)   & (idle_idx > 0)) |
                ((idle_count > 3000000) & (idle_idx == 0))
            )),
        ]
        idx_max = Signal(4)
        self.comb += idx_max.eq(idle_idx + idle_wr)

        # All ports: req = rd_en (mirrors SV assign p_req_data = rd_en)
        # All ports: req always high - ports write freely, backpressure via FIFOs
        for i in range(nports):
            self.comb += self.p_req[i].eq(1)



        # -------------------------------------------------------------------
        # Output frame assembly — must mirror SV dout_data exactly
        # SV: { ctx[1],ctx[0], ctx[3],ctx[2], ctx[5],ctx[4], 4'hE,ctx[6],
        #        data[0], data[1], data[2], data[3], data[4], data[5], data[6] }
        # Migen Cat() is LSB-first so we reverse the order:
        # -------------------------------------------------------------------
        dout_data = Signal(256)
        self.comb += dout_data.eq(Cat(
            data_reg[6], data_reg[5], data_reg[4],         # bits  95:0
            data_reg[3], data_reg[2], data_reg[1],         # bits 191:96
            data_reg[0],                                    # bits 223:192
            ctx_reg[6],                                     # bits 227:224
            Signal(4, reset=0xE),                          # bits 231:228  (4'hE)
            ctx_reg[4],  ctx_reg[5],                       # bits 239:232
            ctx_reg[2],  ctx_reg[3],                       # bits 247:240
            ctx_reg[0],  ctx_reg[1],                       # bits 255:248
        ))

        # Output hold register — latches completed frame until serializer takes it.
        # frame_valid stays high until rd_en (serializer ready) clears it.
        # Exposed as self.frame_valid so port ready signals can gate on it.
        frame_valid = Signal()
        self.frame_valid = frame_valid
        frame_data  = Signal(256)

        self.comb += [
            self.valid.eq(frame_valid),
            self.dout .eq(frame_data),
        ]

        # frame_ready: serializer consumed the frame this cycle
        frame_consumed = Signal()
        self.comb += frame_consumed.eq(frame_valid & self.rd_en)

        # -------------------------------------------------------------------
        # Sequential logic
        # -------------------------------------------------------------------
        self.sync += [
            If(ResetSignal(),
                idx_base.eq(0),
                idle_count.eq(0),
                frame_valid.eq(0),
            ).Else(
                # Clear frame_valid when serializer consumes it
                If(frame_consumed,
                    frame_valid.eq(0),
                ),

                # Run mux advance logic every cycle (en always high after reset)
                If(en & ~frame_valid,
                    # Advance base index, wrapping after frame emit
                    idx_base.eq(idx_max - Mux(idx_max >= 7, 7, 0)),

                    # Idle counter: increment every cycle no real data arrives,
                    # reset when any port writes data this cycle.
                    If(idle_idx == p_idx[0],   # no port wrote anything
                        idle_count.eq(idle_count + 1),
                    ).Else(
                        idle_count.eq(0),
                    ),

                    # Write data/ctx from active input ports into slots
                    *[If(self.p_wr[i],
                        data_reg[p_idx[i]].eq(self.p_din[i]),
                        ctx_reg [p_idx[i]].eq(self.p_ctx[i]),
                      ) for i in range(nports)],

                    # Idle port fills padding slot
                    If(idle_wr,
                        data_reg[idle_idx].eq(0xFFFFFFFF),
                        ctx_reg [idle_idx].eq(0b1111),
                    ),

                    # Latch completed frame into hold register
                    If(idx_max >= 7,
                        frame_valid.eq(1),
                        frame_data .eq(dout_data),
                    ),
                ),
            )
        ]


# ---------------------------------------------------------------------------
# MuxSerializer
#
# Consumes 256-bit frames from PCILeechMux and outputs them as 8 sequential
# 32-bit words (MSB-first, i.e. word0 = bits[255:224]).
# Drives mux.rd_en high whenever it is idle (ready for next frame).
# ---------------------------------------------------------------------------
class MuxSerializer(Module):
    def __init__(self):
        self.sink   = stream.Endpoint([("data", 256)])  # from PCILeechMux
        self.source = stream.Endpoint([("data", 32)])   # to FT601

        buf   = Signal(256)
        count = Signal(3)   # remaining words to send (0 = done)
        rsync = Signal(3)   # resync words remaining

        self.submodules.fsm = fsm = FSM(reset_state="IDLE")

        fsm.act("IDLE",
            self.sink.ready.eq(1),
            If(self.sink.valid,
                NextValue(buf,   self.sink.data),
                NextValue(count, 7),
                NextValue(rsync, 4),   # will send rsync+1 = 5 resync words
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
            # Word 0 is at MSB — shift left by 32 each cycle
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


# ---------------------------------------------------------------------------
# PCILeechFIFO
#
# Top-level module.  Wire this between usb_phy and pcie_phy in pcie_squirrel.py
#
# Connections expected from caller:
#
#   # USB (from FT601Sync — 32-bit word streams)
#   self.usb_rx  = stream.Endpoint([("data", 32)])   # words from host
#   self.usb_tx  = stream.Endpoint([("data", 32)])   # words to host
#
#   # PCIe TLP (from/to S7PCIEPHY — 64-bit AXI stream)
#   self.tlp_rx  = stream.Endpoint(phy_layout(64))   # TLPs from PCIe bus
#   self.tlp_tx  = stream.Endpoint(phy_layout(64))   # TLPs to PCIe bus
#
#   # Control outputs
#   self.pcie_rst_core   = Signal()   # → pcie_phy reset
#   self.pcie_rst_subsys = Signal()   # → subsystem reset
# ---------------------------------------------------------------------------
class PCILeechFIFO(Module):
    def __init__(self):
        # USB side — 32-bit word streams
        self.usb_rx = stream.Endpoint([("data", 32)])   # words arriving from host
        self.usb_tx = stream.Endpoint([("data", 32)])   # words going to host

        # PCIe TLP side - 32-bit, matches phy_layout(32): dat, be, last
        self.tlp_rx = stream.Endpoint(phy_layout(32))  # from PCIe RX
        self.tlp_tx = stream.Endpoint(phy_layout(32))  # to PCIe TX

        # Control outputs driven by CMD register file
        self.pcie_rst_core   = Signal(reset=0)
        self.pcie_rst_subsys = Signal(reset=0)

        # PCIe PHY status inputs (from pcie_phy._link_status.fields.*)
        # Wire these from the caller for correct PHY register responses.
        self.phy_lnk_up    = Signal()   # user_lnk_up
        self.phy_ltssm     = Signal(6)  # pl_ltssm_state
        self.phy_lnk_rate  = Signal()   # pl_sel_lnk_rate  (0=Gen1, 1=Gen2)
        self.phy_lnk_width = Signal(2)  # pl_sel_lnk_width (0b00=x1)
        self.phy_id        = Signal(16) # PCIe BDF: {bus[7:0], dev[4:0], fn[2:0]}

        # ===================================================================
        # RX PATH: USB → dispatch
        # Step 1: pack two consecutive 32-bit words into one 64-bit frame
        # ===================================================================
        rx64       = Signal(64)
        rx64_valid = Signal()
        rx_lo      = Signal(32)
        rx_phase   = Signal()    # 0 = waiting for low word, 1 = have low word

        # Always consume USB RX — we can't apply backpressure here
        self.comb += self.usb_rx.ready.eq(1)

        # FT601 byte-swaps each 32-bit word on the bus (see pcileech_ft601.sv lines 42-45).
        # FT601Sync passes data through unmodified, so we must reverse the swap here.
        rx_word = Signal(32)
        self.comb += rx_word.eq(Cat(
            self.usb_rx.data[24:32],
            self.usb_rx.data[16:24],
            self.usb_rx.data[ 8:16],
            self.usb_rx.data[ 0: 8],
        ))

        self.sync += [
            rx64_valid.eq(0),
            If(self.usb_rx.valid,
                # Resync pattern — reset alignment
                If(rx_word == 0x66665555,
                    rx_phase.eq(0),
                ).Elif(rx_phase == 0,
                    rx_lo  .eq(rx_word),
                    rx_phase.eq(1),
                ).Else(
                    # SV: com_rx_data64 = (prev << 32) | new  → new word in [31:0], prev in [63:32]
                    rx64  .eq(Cat(rx_word, rx_lo)),
                    rx64_valid.eq(1),
                    rx_phase.eq(0),
                )
            )
        ]

        # Step 2: decode magic + type
        magic_ok  = Signal()
        pkt_type  = Signal(2)
        pkt_last  = Signal()
        pkt_data  = Signal(32)

        self.comb += [
            magic_ok .eq(rx64[0:8]  == MAGIC),
            pkt_type .eq(rx64[8:10]),
            pkt_last .eq(rx64[10]),
            pkt_data .eq(rx64[32:64]),
        ]

        rx_is_tlp  = Signal()
        rx_is_cfg  = Signal()
        rx_is_loop = Signal()
        rx_is_cmd  = Signal()

        self.comb += [
            rx_is_tlp .eq(rx64_valid & magic_ok & (pkt_type == TYPE_TLP)),
            rx_is_cfg .eq(rx64_valid & magic_ok & (pkt_type == TYPE_CFG)),
            rx_is_loop.eq(rx64_valid & magic_ok & (pkt_type == TYPE_LOOP)),
            rx_is_cmd .eq(rx64_valid & magic_ok & (pkt_type == TYPE_CMD)),
        ]

        # ===================================================================
        # TLP TX FIFO: host→PCIe  (256 deep, 32+1 bit)
        # Receives TLP words from USB, outputs to pcie_phy TX
        # ===================================================================
        self.submodules.tlp_tx_fifo = tlp_tx_fifo = SyncFIFO(
            phy_layout(32), 256
        )
        self.comb += [
            tlp_tx_fifo.sink.valid.eq(rx_is_tlp),
            tlp_tx_fifo.sink.dat  .eq(pkt_data),
            tlp_tx_fifo.sink.be   .eq(0xf),
            tlp_tx_fifo.sink.last .eq(pkt_last),
            tlp_tx_fifo.source.connect(self.tlp_tx),
        ]

        # ===================================================================
        # TLP RX FIFO: PCIe→host  (256 deep, 32+1 bit)
        # Receives TLP words from pcie_phy RX, feeds TX mux port 3
        # ===================================================================
        self.submodules.tlp_rx_fifo = tlp_rx_fifo = SyncFIFO(
            phy_layout(32), 256
        )
        self.comb += self.tlp_rx.connect(tlp_rx_fifo.sink)

        # ===================================================================
        # LOOPBACK FIFO: host→host echo  (64 deep, 34 bit)
        # SV: din = {com_dout[11:10], com_dout[63:32]}
        #   = {rx64[11:10], rx64[63:32]}  (ctx bits + upper payload word)
        # p0_ctx = loop_dout[33:32], p0_tag = 2'b10
        # nibble = (ctx<<2)|0b10
        # ===================================================================
        self.submodules.loop_fifo = loop_fifo = SyncFIFO(
            [("data", 32), ("ctx", 2)], 64
        )
        self.comb += [
            loop_fifo.sink.valid.eq(rx_is_loop),
            loop_fifo.sink.data .eq(rx64[32:64]),   # upper word = com_dout[63:32]
            loop_fifo.sink.ctx  .eq(rx64[10:12]),   # bits[11:10] = com_dout[11:10]
        ]

        # ===================================================================
        # rw[] register file — defined early so both CFG and CMD decoders
        # can reference it. Named aliases follow below CMD section.
        # ===================================================================
        rw = Signal(240)

        # ===================================================================
        # CFG register file: TYPE_CFG frames (flag bits 0x01 = FPGA_REG_PCIE)
        # Responds on mux port 1 (nibble 0x1) with zeros for all PCIe reads.
        # This is sufficient for pcileech to complete init and proceed.
        # ===================================================================
        self.submodules.cfg_rx_fifo = cfg_rx_fifo = SyncFIFO(
            [("data", 64)], 64
        )
        self.submodules.cfg_tx_fifo = cfg_tx_fifo = SyncFIFO(
            [("data", 32)], 64
        )
        self.comb += [
            cfg_rx_fifo.sink.valid.eq(rx_is_cfg),
            cfg_rx_fifo.sink.data .eq(rx64),
        ]

        cfg_cmd       = cfg_rx_fifo.source.data
        cfg_cmd_valid = cfg_rx_fifo.source.valid
        cfg_addr_byte = Signal(16)
        cfg_cmd_read  = Signal()
        self.comb += [
            cfg_addr_byte.eq(cfg_cmd[16:32]),
            cfg_cmd_read .eq(cfg_cmd_valid & cfg_cmd[12]),
            cfg_rx_fifo.source.ready.eq(1),
        ]

        # CFG register readback — mirrors pcileech_pcie_cfg_a7.sv mapping.
        # Handles both ro[] (READONLY) and rw[] (READWRITE) reads.
        # All addresses are byte-addressed; we respond with a 16-bit value per read.
        # Byte offset → bits:
        #   0x000A: ro[85:80]  = pl_ltssm_state[5:0]
        #   0x000C: ro[97:96]  = pl_sel_lnk_width, ro[98]=pl_phy_lnk_up, ro[102]=pl_sel_lnk_rate
        #   0x0016: rw[191:176]= pl_directed_link_* + pl_transmit_hot_rst (read back rw value)
        # Word index = byte_offset >> 1
        cfg_word_index  = Signal(8)
        cfg_ro_readback = Signal(16)
        self.comb += cfg_word_index.eq(cfg_addr_byte[1:9])  # byte_addr >> 1

        # phy_id_latched: holds PCIe BDF. INIT=0x0c00; fallback=0x0016 for 16:00.0
        # sentinel) so wDeviceId is non-zero even before enumeration.
        # Latches real BDF once pcie_phy.id becomes valid after link trains.
        phy_id_latched = Signal(16, reset=0x0c00)
        self.sync += If(self.phy_id, phy_id_latched.eq(self.phy_id))
        self.comb += Case(cfg_word_index, {
            5:  cfg_ro_readback.eq(Cat(self.phy_ltssm,    Signal(10))),  # byte 0x0A: ro[85:80]=ltssm
            6:  cfg_ro_readback.eq(Cat(self.phy_lnk_width,              # byte 0x0C: ro[97:96]=width
                                       self.phy_lnk_up,                 #            ro[98]=pl_phy_lnk_up
                                       Signal(3),                        #            ro[101:99]=0
                                       self.phy_lnk_rate,               #            ro[102]=rate
                                       Signal(9))),                      #            ro[111:103]=0
            11: cfg_ro_readback.eq(rw[176:192]),                        # byte 0x16: rw[191:176] pl_directed_*
            4:  cfg_ro_readback.eq(Mux(phy_id_latched, phy_id_latched, 0x1600)),          # byte 0x08: PCIe BDF {bus,dev,fn} — outer swap gives 0x0016 → wDeviceId=0x1600
            "default": cfg_ro_readback.eq(0),
        })

        self.comb += [
            # Response format: [31:16]=addr_byte echo, [15:0]={val[7:0],val[15:8]}
            cfg_tx_fifo.sink.valid.eq(cfg_cmd_read),
            cfg_tx_fifo.sink.data .eq(Cat(
                Cat(cfg_ro_readback[8:16], cfg_ro_readback[0:8]),  # byte-swap
                cfg_addr_byte,
            )),
            cfg_tx_fifo.sink.last .eq(1),
        ]


        # Named aliases for important rw bits
        rw_pcie_rst_core   = rw[200]
        rw_pcie_rst_subsys = rw[201]
        rw_cfgtlp_en       = rw[202]
        rw_cfgtlp_zero     = rw[203]
        rw_cfgtlp_filter   = rw[204]
        rw_bar_en          = rw[205]
        rw_cfgtlp_wren     = rw[206]
        rw_alltlp_filter   = rw[207]

        # CMD RX FIFO — buffers incoming CMD frames (64-bit)
        self.submodules.cmd_rx_fifo = cmd_rx_fifo = SyncFIFO(
            [("data", 64)], 64
        )
        self.comb += [
            cmd_rx_fifo.sink.valid.eq(rx_is_cmd),
            cmd_rx_fifo.sink.data .eq(rx64),
        ]

        # CMD TX FIFO — buffers CMD responses going back to host (32-bit)
        self.submodules.cmd_tx_fifo = cmd_tx_fifo = SyncFIFO(
            [("data", 32)], 64
        )

        # CMD register file decode
        cmd        = cmd_rx_fifo.source.data
        cmd_valid  = cmd_rx_fifo.source.valid

        in_addr_byte = Signal(16)
        in_addr_bit  = Signal(18)   # byte_addr * 8
        in_value     = Signal(16)   # byte-swapped per SV
        in_mask      = Signal(16)   # byte-swapped per SV
        f_rw         = Signal()     # addr bit15 → rw register space
        in_cmd_read  = Signal()
        in_cmd_write = Signal()

        self.comb += [
            in_addr_byte.eq(cmd[16:32]),
            # bit address = byte_address[14:0] << 3
            in_addr_bit .eq(Cat(Signal(3), cmd[16:31])),
            # SV: in_cmd_value = {cmd[48+:8], cmd[56+:8]}  (big-endian 16-bit)
            in_value    .eq(Cat(cmd[56:64], cmd[48:56])),
            # SV: in_cmd_mask  = {cmd[32+:8], cmd[40+:8]}
            in_mask     .eq(Cat(cmd[40:48], cmd[32:40])),
            f_rw        .eq(cmd[31]),          # bit15 of addr byte
            # bit14 of addr = shadow config space — we don't support that yet
            in_cmd_read .eq(cmd_valid & cmd[12] & ~cmd[30]),
            in_cmd_write.eq(cmd_valid & cmd[13] & ~cmd[30] & f_rw),
            # Always consume CMD FIFO
            cmd_rx_fifo.source.ready.eq(1),
        ]

        # Register file reset values (match pcileech_fifo.sv initialvalues task)
        def rw_reset_stmts():
            return [
                rw[  0: 16].eq(0xEFCD),   # MAGIC
                rw[ 16]    .eq(0),         # inactivity timer enable
                rw[ 17]    .eq(0),         # send count enable
                rw[ 18]    .eq(1),         # wait for DRP completion
                rw[ 19]    .eq(0),
                rw[ 20]    .eq(0),         # DRP RD EN
                rw[ 21]    .eq(0),         # DRP WR EN
                rw[ 31]    .eq(0),         # global system reset
                rw[ 32: 64].eq(240 >> 3),  # bytecount of rw[]
                rw[ 64: 96].eq(0),         # inactivity timer ticks
                rw[ 96:128].eq(0),         # send count
                rw[128:144].eq(0x10EE),    # CFG_SUBSYS_VEND_ID
                rw[144:160].eq(0x0007),    # CFG_SUBSYS_ID
                rw[160:176].eq(0x10EE),    # CFG_VEND_ID
                rw[176:192].eq(0x0666),    # CFG_DEV_ID
                rw[192:200].eq(0x02),      # CFG_REV_ID
                rw[200]    .eq(1),         # PCIE CORE RESET (asserted at startup)
                rw[201]    .eq(0),         # PCIE SUBSYSTEM RESET
                rw[202]    .eq(1),         # CFGTLP PROCESSING ENABLE
                rw[203]    .eq(1),         # CFGTLP ZERO DATA
                rw[204]    .eq(1),         # CFGTLP FILTER TLP FROM USER
                rw[205]    .eq(1),         # BAR PIO ENABLE
                rw[206]    .eq(0),         # CFGTLP PCIE WRITE ENABLE
                rw[207]    .eq(0),         # ALL TLP FILTER
                rw[208:224].eq(0),         # DRP di
                rw[224:233].eq(0),         # DRP addr
            ]

        # Apply register writes — 16 bits at a time, masked.
        #
        # Address arithmetic (matches pcileech_fifo.sv):
        #   in_addr_byte = cmd[31:16]  e.g. 0x8019 for PCIe reset write
        #   in_addr_bit  = in_addr_byte[14:0] << 3  = byte_offset * 8
        #   We do 16-bit (2-byte) aligned accesses, so:
        #   word_index   = byte_offset >> 1  = in_addr_byte[1:8] (7 bits, low byte)
        #   bit_base     = word_index * 16
        #
        # Example: PCIe reset at rw[200]
        #   host sends in_addr_byte = 0x8019  (byte offset 0x19 = 25)
        #   word_index = 25 >> 1 = 12  → bit_base = 12*16 = 192
        #   rw[192:208] covers bits 192..207 including rw[200]=pcie_rst_core ✓
        #
        # Case key = in_addr_byte[1:8]  (7-bit word index, ignoring f_rw flag
        # in bit15 and f_shadowcfgspace in bit14 — those are already checked
        # by in_cmd_write guard above)

        write_cases = {}
        for word_idx in range(15):   # 15 x 16-bit words = 240 bits
            bit_base = word_idx * 16
            stmts = []
            for b in range(16):
                if bit_base + b < 240:
                    stmts.append(
                        If(in_mask[b], rw[bit_base + b].eq(in_value[b]))
                    )
            write_cases[word_idx] = stmts

        self.sync += [
            If(ResetSignal(),
                *rw_reset_stmts(),
            ).Elif(in_cmd_write,
                Case(in_addr_byte[1:8],   # word index = byte_offset >> 1
                    {k: v for k, v in write_cases.items()}
                )
            )
        ]

        # Drive control outputs from register file
        self.comb += [
            self.pcie_rst_core  .eq(rw_pcie_rst_core),
            self.pcie_rst_subsys.eq(rw_pcie_rst_subsys),
        ]

        # CMD read response → cmd_tx_fifo
        # Response format (matches pcileech_fifo.sv):
        #   [31:16] = in_addr_byte (echoed address)
        #   [15: 0] = {data[7:0], data[15:8]}  (byte-swapped 16-bit value)
        #
        # Two register spaces selected by f_rw (addr bit15):
        #   f_rw=0 → ro[] read-only registers
        #   f_rw=1 → rw[] read-write registers
        #
        # ro[] layout (matches pcileech_fifo.sv):
        #   ro[ 0: 16] = 0xab89          byte offset 0x00  MAGIC
        #   ro[16: 32] = 0x0000          byte offset 0x02  (reserved)
        #   ro[32: 48] = 0x0000          byte offset 0x04  (reserved)
        #   ro[48: 64] = 0x0000          byte offset 0x06  (reserved)
        #   ro[64: 72] = VERSION_MAJOR   byte offset 0x08
        #   ro[72: 80] = VERSION_MINOR   byte offset 0x09
        #   ro[80: 88] = DEVICE_ID       byte offset 0x0A
        #   ro[88: 96] = 0x00            byte offset 0x0B
        #   (remaining ro[] = 0)

        VERSION_MAJOR = 0x04   # match pcileech-fpga PCIeSquirrel major version
        VERSION_MINOR = 0x0e   # match pcileech-fpga PCIeSquirrel v4.14 minor version
        DEVICE_ID     = 0x04   # PCIeSquirrel device ID

        rw_readback = Signal(16)
        ro_readback = Signal(16)
        readback    = Signal(16)

        # rw[] combinational readback
        rw_rb_cases = {}
        for word_idx in range(15):
            bit_base = word_idx * 16
            rw_rb_cases[word_idx] = rw_readback.eq(rw[bit_base:bit_base+16])
        self.comb += Case(in_addr_byte[1:8], rw_rb_cases)

        # ro[] combinational readback — word_index = byte_offset >> 1
        # byte offset 0x00 → word_index 0 → magic 0xab89
        # byte offset 0x08 → word_index 4 → {VERSION_MINOR, VERSION_MAJOR}
        # byte offset 0x0A → word_index 5 → {0x00, DEVICE_ID}
        self.comb += Case(in_addr_byte[1:8], {
            0: ro_readback.eq(0xab89),
            4: ro_readback.eq(Cat(Signal(8, reset=VERSION_MAJOR),
                                  Signal(8, reset=VERSION_MINOR))),
            5: ro_readback.eq(Cat(Signal(8, reset=DEVICE_ID), Signal(8))),
        })

        # Select ro[] or rw[] based on f_rw flag
        self.comb += If(f_rw,
            readback.eq(rw_readback)
        ).Else(
            readback.eq(ro_readback)
        )

        # CMD TX response format (matches pcileech_fifo.sv lines 361-365):
        #   [31:16] = in_cmd_address_byte  (echoed back)
        #   [15:0]  = {data_in[7:0], data_in[15:8]}  (byte-swapped 16-bit value)
        self.comb += [
            cmd_tx_fifo.sink.valid.eq(in_cmd_read),
            cmd_tx_fifo.sink.data .eq(Cat(
                Cat(readback[8:16], readback[0:8]),  # [15:0]  byte-swapped value
                in_addr_byte,                         # [31:16] echoed address
            )),
            cmd_tx_fifo.sink.last.eq(1),
        ]

        # ===================================================================
        # TX PATH: mux + serialize → USB
        # ===================================================================
        self.submodules.mux        = mux        = PCILeechMux(nports=8)
        self.submodules.serializer = serializer = MuxSerializer()

        # Mux rd_en driven by serializer being idle
        self.comb += mux.rd_en.eq(serializer.sink.ready)

        # Connect mux output to serializer
        self.comb += [
            serializer.sink.valid.eq(mux.valid),
            serializer.sink.data .eq(mux.dout),
        ]

        # Connect serializer output to USB TX with byte-swap.
        # FT601 swaps bytes on TX (pcileech_ft601.sv line 36), so we pre-swap
        # to cancel it out and deliver correct byte order to the host.
        tx_swapped = Signal(32)
        self.comb += tx_swapped.eq(Cat(
            serializer.source.data[24:32],
            serializer.source.data[16:24],
            serializer.source.data[ 8:16],
            serializer.source.data[ 0: 8],
        ))
        self.comb += [
            self.usb_tx.valid.eq(serializer.source.valid),
            self.usb_tx.data .eq(tx_swapped),
            serializer.source.ready.eq(self.usb_tx.ready),
        ]

        # -------------------------------------------------------------------
        # Mux port wiring — matches pcileech_fifo.sv port priority exactly:
        #
        # SV:  p0=loopback(tag=10), p1=CMD(tag=11), p2=CFG(tag=01), p3-p6=TLP(tag=00)
        #
        # nibble = (p_ctx[1:0] << 2) | p_tag[1:0]
        # Host filter: (nibble & 0x03) == (flags & 0x03)
        #   TYPE_LOOP=0x02 → tag=0b10 → matches p0 nibble & 0x03 = 0x2
        #   TYPE_CMD =0x03 → tag=0b11 → matches p1 nibble & 0x03 = 0x3
        #   TYPE_CFG =0x01 → tag=0b01 → matches p2 nibble & 0x03 = 0x1
        #   TYPE_TLP =0x00 → tag=0b00 → matches p3 nibble & 0x03 = 0x0
        # -------------------------------------------------------------------

        # p0: loopback — tag=0b10, ctx from stored loop_fifo.ctx
        self.comb += [
            mux.p_din[0].eq(loop_fifo.source.data),
            mux.p_ctx[0].eq(Cat(Signal(2, reset=0b10), loop_fifo.source.ctx)),
            mux.p_wr [0].eq(loop_fifo.source.valid & mux.p_req[0]),
            loop_fifo.source.ready.eq(mux.p_req[0] & ~mux.frame_valid),
            mux.p_pending[0].eq(0),  # loopback does not suppress idle
        ]

        # p1: CMD response — tag=0b11, ctx=0b00
        self.comb += [
            mux.p_din[1].eq(cmd_tx_fifo.source.data),
            mux.p_ctx[1].eq(0b0011),
            mux.p_wr [1].eq(cmd_tx_fifo.source.valid & mux.p_req[1]),
            cmd_tx_fifo.source.ready.eq(mux.p_req[1] & ~mux.frame_valid),
            mux.p_pending[1].eq(cmd_tx_fifo.source.valid & ~mux.frame_valid),
        ]

        # p2: CFG response — tag=0b01, ctx=0b00
        self.comb += [
            mux.p_din[2].eq(cfg_tx_fifo.source.data),
            mux.p_ctx[2].eq(0b0001),
            mux.p_wr [2].eq(cfg_tx_fifo.source.valid & mux.p_req[2]),
            cfg_tx_fifo.source.ready.eq(mux.p_req[2] & ~mux.frame_valid),
            mux.p_pending[2].eq(cfg_tx_fifo.source.valid & ~mux.frame_valid),
        ]

        # p3: TLP RX (PCIe → host) — tag=0b00, ctx={first,last} from phy
        self.comb += [
            mux.p_din[3].eq(tlp_rx_fifo.source.dat),
            mux.p_ctx[3].eq(Cat(Signal(2, reset=0b00),
                                Cat(tlp_rx_fifo.source.last, Signal()))),
            mux.p_wr [3].eq(tlp_rx_fifo.source.valid & mux.p_req[3]),
            tlp_rx_fifo.source.ready.eq(mux.p_req[3] & ~mux.frame_valid),
            mux.p_pending[3].eq(0),  # TLP RX does not suppress idle - it fills slots opportunistically
        ]

        # p4-p7: stubs
        for i in range(4, 8):
            self.comb += [
                mux.p_din[i].eq(0),
                mux.p_ctx[i].eq(0),
                mux.p_wr [i].eq(0),
                mux.p_pending[i].eq(0),
            ]
