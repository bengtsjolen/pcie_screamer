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
    def __init__(self, nports=8, registered=0):
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
        # Internal state — 15 slots matching SV pcileech_mux.sv
        # SV uses sliding window: slots 0-6 = current frame, 7-13 = overflow
        # idx_base advances EVERY cycle (not just when frame_valid=0)
        # This allows multiple responses to accumulate in the same frame
        # -------------------------------------------------------------------
        DEPTH    = 15
        data_reg = Array([Signal(32, reset=0xFFFFFFFF, name=f"dr{i}") for i in range(DEPTH)])
        ctx_reg  = Array([Signal(4,  reset=0b1111,    name=f"cr{i}") for i in range(DEPTH)])

        idx_base    = Signal(4, reset=0)
        idle_count  = Signal(4, reset=0)   # SV uses 4-bit idle count, threshold=7
        en          = Signal()             # always 1 after reset - mux runs freely
        dout_valid  = Signal()
        dout_buf_valid = Signal()
        dout_buf_data  = Signal(256)

        if registered:
            self.sync += en.eq(self.rd_en & ~ResetSignal())
        else:
            self.comb += en.eq(self.rd_en & ~ResetSignal())
        self.en_out = en  # expose for port ready gating

        # -------------------------------------------------------------------
        # Priority-index chain (combinational)
        # p_idx[i] = slot index where port i will write its word
        # -------------------------------------------------------------------
        p_idx = [Signal(4, name=f"pidx{i}") for i in range(nports + 1)]
        self.comb += p_idx[0].eq(idx_base)
        for i in range(nports):
            self.comb += p_idx[i+1].eq(p_idx[i] + self.p_wr[i])

        # Idle port — pads frame with 0xFFFFFFFF when stalled.
        # Threshold of 64 cycles (~430ns@150MHz): long enough for back-to-back
        # USB responses to both arrive, short enough to not delay single responses.
        idle_idx = Signal(4)
        idle_wr  = Signal()
        self.comb += [
            idle_idx.eq(p_idx[nports]),
            # SV: p8_wr_en = en && (idx_base > 0) && (idle_count > 7) && (idx_base == p8_idx)
            # idle fires when: mux running, some data written, idle long enough, no more data this cycle
            idle_wr .eq(en & (idle_idx > 0) & (idle_count > 7) & (idle_idx == p_idx[nports])),
        ]
        idx_max = Signal(4)
        self.comb += idx_max.eq(idle_idx + idle_wr)

        # SV: p_req_data = rd_en (the global FT601-ready signal)
        for i in range(nports):
            #if registered:
            #    self.comb += self.p_req[i].eq(en)
            #else:
            self.comb += self.p_req[i].eq(self.rd_en)


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
            Constant(0xE, 4),                              # bits 231:228  (4'hE) — must be Const not Signal
            ctx_reg[4],  ctx_reg[5],                       # bits 239:232
            ctx_reg[2],  ctx_reg[3],                       # bits 247:240
            ctx_reg[0],  ctx_reg[1],                       # bits 255:248
        ))

        self.comb += [
            self.valid.eq(self.rd_en & (dout_buf_valid | dout_valid)),
            self.dout.eq(Mux(dout_buf_valid, dout_buf_data, dout_data)),
        ]

        # -------------------------------------------------------------------
        # Sequential logic — mirrors SV pcileech_mux.sv always block exactly
        # Key difference from naive impl: idx_base advances EVERY cycle (not
        # just when ~frame_valid), enabling multiple responses in one frame.
        # -------------------------------------------------------------------
        self.sync += [
            If(ResetSignal(),
                idx_base.eq(0),
                idle_count.eq(0),
                dout_valid.eq(0),
                dout_buf_valid.eq(0),
            ).Else(
                # OUTPUT BUFFER LOGIC (mirrors SV)
                If(en,
                    dout_buf_valid.eq(0),
                ).Elif(dout_valid,
                    dout_buf_data.eq(dout_data),
                    dout_buf_valid.eq(1),
                ),

                # OUTPUT VALID: frame ready when idx_max >= 7
                dout_valid.eq(en & (idx_max >= 7)),

                If(en,
                    # NEXT INDEX BASE: advance every cycle, subtract 7 on frame emit
                    idx_base.eq(idx_max - Mux(idx_max >= 7, 7, 0)),

                    # IDLE COUNT: increment when no new data, reset when data arrives
                    If((idx_base > 0) & (idx_base == p_idx[nports]),
                        idle_count.eq(idle_count + 1),
                    ).Else(
                        idle_count.eq(0),
                    ),

                    # Write data/ctx from all active ports into their slots
                    *[If(self.p_wr[i],
                        data_reg[p_idx[i]].eq(self.p_din[i]),
                        ctx_reg [p_idx[i]].eq(self.p_ctx[i]),
                      ) for i in range(nports)],

                    # Idle port fills its slot when threshold reached
                    If(idle_wr,
                        data_reg[idle_idx].eq(0xFFFFFFFF),
                        ctx_reg [idle_idx].eq(0b1111),
                    ),
                ),

                # OVERFLOW: when frame emits, copy overflow slots to base positions
                # ONLY when downstream actually consumes the frame (rd_en=1).
                # Without this gate, the registered en lag can cause the overflow
                # copy to overwrite a frame that was never consumed.
                # (mirrors SV: if(dout_valid) data_reg[i] <= data_reg[7+i])
                If(dout_valid & self.rd_en,
                    *[If(idx_base > i,
                        data_reg[i].eq(data_reg[7+i]),
                        ctx_reg [i].eq(ctx_reg [7+i]),
                      ) for i in range(7)],
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
        self.sink   = stream.Endpoint([("data", 256)])
        self.source = stream.Endpoint([("data", 32)])
        
        buf   = Signal(256)
        count = Signal(3)
        rsync = Signal(3)
        
        self.submodules.fsm = fsm = FSM(reset_state="IDLE")
        
        fsm.act("IDLE",
                self.sink.ready.eq(1),
                If(self.sink.valid,
                   NextValue(buf,   self.sink.data),
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
                   If(count == 0,
                      # Last word sent — try to grab next frame immediately
                      self.sink.ready.eq(1),
                      If(self.sink.valid,
                         # Back-to-back: load next frame, skip RESYNC
                         NextValue(buf,   self.sink.data),
                         NextValue(count, 7),
                         # Stay in SEND
                         ).Else(
                             NextState("IDLE"),
                         )
                      ).Else(
                          NextValue(buf, buf << 32),
                          NextValue(count, count - 1),
                      )
                   )
                )

class MuxSerializer2(Module):
    def __init__(self):
        self.sink   = stream.Endpoint([("data", 256)])
        self.source = stream.Endpoint([("data", 32)])
        
        buf        = Signal(256)
        count      = Signal(3)
        rsync      = Signal(3)
        
        next_buf   = Signal(256)
        next_valid = Signal()
    
        self.submodules.fsm = fsm = FSM(reset_state="IDLE")
        
        self.comb += [
            self.sink.ready.eq(0),
            self.source.valid.eq(0),
            self.source.data.eq(0),
        ]
        
        fsm.act("IDLE",
                self.sink.ready.eq(1),
                If(self.sink.valid,
                   NextValue(buf, self.sink.data),
                   NextValue(count, 7),
                   NextValue(rsync, 4),
                   NextValue(next_valid, 0),
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
                If(~next_valid,
                   self.sink.ready.eq(1),
                   If(self.sink.valid,
                      NextValue(next_buf, self.sink.data),
                      NextValue(next_valid, 1),
                      )
                   ),
                
                self.source.valid.eq(1),
                self.source.data.eq(buf[224:256]),
                
                If(self.source.ready,
                   If(count == 0,
                      If(next_valid,
                         NextValue(buf, next_buf),
                         NextValue(count, 7),
                         NextValue(next_valid, 0),
                         # stay in SEND, no RESYNC
                         ).Else(
                             NextState("IDLE"),
                         )
                      ).Else(
                          NextValue(buf, buf << 32),
                          NextValue(count, count - 1),
                      )
                   )
                )



        
class MuxWordQueueTX(Module):
    def __init__(self, idle_threshold=64, word_fifo_depth=32, start_wait_max=8):
        self.sink   = stream.Endpoint([("data", 256)])  # from PCILeechMux
        self.source = stream.Endpoint([("data", 32)])   # to USB / FT601
        
        # ------------------------------------------------------------------
        # Real 32-bit word FIFO.
        # Sync words are NOT stored here.
        # ------------------------------------------------------------------
        self.submodules.word_fifo = word_fifo = SyncFIFO([("data", 32)], word_fifo_depth)
        
        # ------------------------------------------------------------------
        # Two full-frame buffers.
        #
        # fb0 = frame currently being expanded into word_fifo
        # fb1 = next full frame already captured from mux
        # ------------------------------------------------------------------
        fb0       = Signal(256)
        fb0_valid = Signal()
        fb0_idx   = Signal(4)   # 0..7
        
        fb1       = Signal(256)
        fb1_valid = Signal()

        # heuristic for differing wat during dump start
        fb0_word0 = Signal(32)
        is_dump_start = Signal()
        active_start_wait_max = Signal(12)
        ctrl_start_wait_max = 8
        dump_start_wait_max = 2048
        
        self.comb += [fb0_word0.eq(fb0[224:256]),
                      is_dump_start.eq(fb0_word0 == 0xf2ffffef),
                      active_start_wait_max.eq(Mux(is_dump_start,dump_start_wait_max,ctrl_start_wait_max))
                      ]

        
        # ------------------------------------------------------------------
        # Burst / sync state.
        #
        # armed_for_sync : after long idle, next burst should start with 5 syncs
        # sync_count     : sync words still to emit for current burst
        # burst_pending  : first data of a new burst has arrived, but output has
        #                  not started yet (waiting for watermark or timeout)
        # burst_started  : actively outputting sync/data for current burst
        # ------------------------------------------------------------------
        armed_for_sync = Signal(reset=1)
        sync_count     = Signal(3)   # 0..5
        burst_pending  = Signal()
        burst_started  = Signal()
        
        # Startup wait counter while pending.
        start_wait = Signal(max=dump_start_wait_max + 1)
        
        # ------------------------------------------------------------------
        # Idle tracking.
        # ------------------------------------------------------------------
        idle_count = Signal(max=idle_threshold + 1)
        
        fifo_empty    = Signal()
        fifo_has_room = Signal()
        self.comb += [
            fifo_empty.eq(word_fifo.level == 0),
            fifo_has_room.eq(word_fifo.level < word_fifo_depth),
        ]
        
        fully_idle = Signal()
        self.comb += fully_idle.eq(
        (sync_count == 0) &
            ~fb0_valid &
            ~fb1_valid &
            fifo_empty &
            ~burst_pending &
            ~burst_started
        )
        
        # ------------------------------------------------------------------
        # Start condition:
        #   - if we already have a second frame buffered, great
        #   - or if one full frame of words is already queued
        #   - or after a short timeout, start anyway so tiny bursts don't stall
        # ------------------------------------------------------------------
        start_now = Signal()
        self.comb += start_now.eq(
            burst_pending & (
                fb1_valid |
                (word_fifo.level >= 8) |
                (start_wait >= active_start_wait_max)
            )
        )
        
        # ------------------------------------------------------------------
        # Output path:
        #   - emit sync words first
        #   - then real FIFO data
        # Only active once burst_started=1.
        # ------------------------------------------------------------------
        sending_sync = Signal()
        sending_data = Signal()
        self.comb += [
            sending_sync.eq(burst_started & (sync_count != 0)),
            sending_data.eq(burst_started & (sync_count == 0) & word_fifo.source.valid),
            
            self.source.valid.eq(sending_sync | sending_data),
            self.source.data.eq(Mux(sending_sync, 0x66665555, word_fifo.source.data)),
            
            word_fifo.source.ready.eq(burst_started & (sync_count == 0) & self.source.ready),
        ]
        
        # ------------------------------------------------------------------
        # Accept a new 256-bit frame whenever fb1 is free.
        # This gives us 2 full-frame buffers of elasticity.
        # ------------------------------------------------------------------
        self.comb += self.sink.ready.eq(~fb1_valid)
        
        # ------------------------------------------------------------------
        # Feed words from fb0 into word_fifo.
        # This can happen while sync is being emitted.
        # ------------------------------------------------------------------
        self.comb += [
            word_fifo.sink.valid.eq(fb0_valid & fifo_has_room),
            word_fifo.sink.data.eq(fb0[224:256]),
        ]
        
        # ------------------------------------------------------------------
        # Sequential logic
        # ------------------------------------------------------------------
        self.sync += [
            # --------------------------------------------------------------
            # Re-arm sync after long true idle.
            # --------------------------------------------------------------
            If(fully_idle,
               If(idle_count < idle_threshold,
                  idle_count.eq(idle_count + 1)
                  ).Else(
                      armed_for_sync.eq(1)
                  )
               ).Else(
                   idle_count.eq(0)
               ),
            
            # --------------------------------------------------------------
            # Capture incoming 256-bit frame.
            #
            # If pipeline was idle before this frame, mark burst_pending.
            # --------------------------------------------------------------
            If(self.sink.valid & self.sink.ready,
               If(~fb0_valid & ~fb1_valid & ~burst_pending & ~burst_started & (sync_count == 0) & fifo_empty,
                  burst_pending.eq(1),
                  start_wait.eq(0)
                  ),
               
               If(~fb0_valid,
                  fb0.eq(self.sink.data),
                  fb0_valid.eq(1),
                  fb0_idx.eq(0)
                  ).Else(
                      fb1.eq(self.sink.data),
                      fb1_valid.eq(1)
                  )
               ),
            
            # --------------------------------------------------------------
            # While pending, wait a little for more buffered data.
            # --------------------------------------------------------------
            If(burst_pending & ~start_now,
               If(start_wait != dump_start_wait_max,
                  start_wait.eq(start_wait + 1)
                  )
               ),
            
            # --------------------------------------------------------------
            # Start the burst.
            # Only here do we arm the 5 sync words.
            # --------------------------------------------------------------
            If(start_now,
               burst_pending.eq(0),
               burst_started.eq(1),
               If(armed_for_sync,
                  sync_count.eq(5),
                  armed_for_sync.eq(0)
                  )
               ),
            
            # --------------------------------------------------------------
            # Drain one sync word.
            # --------------------------------------------------------------
            If((sync_count != 0) & burst_started & self.source.ready,
               sync_count.eq(sync_count - 1)
               ),
            
            # --------------------------------------------------------------
            # Advance fb0 as words are accepted into word_fifo.
            # Promote fb1 immediately when fb0 finishes.
            # --------------------------------------------------------------
            If(word_fifo.sink.valid & word_fifo.sink.ready,
               If(fb0_idx == 7,
                  If(fb1_valid,
                     fb0.eq(fb1),
                     fb0_valid.eq(1),
                     fb0_idx.eq(0),
                     fb1_valid.eq(0)
                     ).Else(
                         fb0_valid.eq(0)
                     )
                  ).Else(
                      fb0.eq(fb0 << 32),
                      fb0_idx.eq(fb0_idx + 1)
                  )
               ),
            
            # --------------------------------------------------------------
            # End burst when everything has drained.
            # --------------------------------------------------------------
            If(burst_started &
               (sync_count == 0) &
               ~fb0_valid &
               ~fb1_valid &
               fifo_empty,
               burst_started.eq(0)
               )
        ]
        
        
        
        
        
        
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
        self.cfg_dcommand  = Signal(16) # PCIe Device Control register (dcommand)

        # Extra diagnostics from pcie_phy / wrappers
        self.diag_tx_conv_seen   = Signal()  # tlp_tx_conv.source.valid & ready
        self.diag_tx_axis_seen   = Signal()  # pcie_phy.s_axis_tx_tvalid & ready
        self.diag_rx_axis_seen   = Signal()  # pcie_phy.m_axis_rx_tvalid
        self.diag_tx_err_drop    = Signal()  # pcie_phy.tx_err_drop

        self.tlp_tx_dbg0 = Signal(32)
        self.tlp_tx_dbg1 = Signal(32)
        self.tlp_tx_dbg2 = Signal(32)
        self.tlp_tx_dbg3 = Signal(32)
        self.tlp_tx_dbg_be0 = Signal(4)
        self.tlp_tx_dbg_be1 = Signal(4)
        self.tlp_tx_dbg_be2 = Signal(4)
        self.tlp_tx_dbg_be3 = Signal(4)
        self.tlp_tx_dbg_last0 = Signal()
        self.tlp_tx_dbg_last1 = Signal()
        self.tlp_tx_dbg_last2 = Signal()
        self.tlp_tx_dbg_last3 = Signal()
        dbg_armed   = Signal(reset=1)
        dbg_count   = Signal(3)
        dbg_seen    = Signal()

        self.tx64_dbg0      = Signal(64)
        self.tx64_dbg1      = Signal(64)
        self.tx64_dbg_flags = Signal(16)  # low bits: seen/armed/count

        self.txsink_dbg0      = Signal(64)
        self.txsink_dbg1      = Signal(64)
        self.txsink_dbg_be0   = Signal(8)
        self.txsink_dbg_be1   = Signal(8)
        self.txsink_dbg_last0 = Signal()
        self.txsink_dbg_last1 = Signal()
        self.txsink_dbg_flags = Signal(16)

        self.rxsink_dbg = Array(Signal(64) for _ in range(8))
        self.rxsink_be  = Array(Signal(8)  for _ in range(8))
        self.rxsink_lasts = Signal(8)
        self.rxsink_flags = Signal(16)
        
        # Flow counters captured in top-level / locally.
        self.diag_rx64_seen      = Signal(16)
        self.diag_rx32_seen      = Signal(16)
        self.diag_ser_out_seen   = Signal(16)
        self.diag_usbtx_seen     = Signal(16)
        
        self.diag_rxfifo_in_seen = Signal(16)
        self.diag_rxfifo_out_seen= Signal(16)
        self.diag_mux_p3_wr_seen = Signal(16)

        rxfifo_in_seen   = Signal(16)
        rxfifo_out_seen  = Signal(16)
        mux_p3_wr_seen   = Signal(16)
        
        
        # Timeout / snapshot outputs
        self.diag_force_pcie_reset = Signal()
        
        # Diagnostic output - read via CMD register 0x0006 (ro, word_index 3):
        # [7:0]=tlp_rx_fifo.level, [13:8]=rx_seen_count[5:0],
        # [14]=phy_source_seen (pcie_phy.source.valid after CDC),
        # [15]=phy_raw_rx_seen (m_axis_rx_tvalid before CDC)
        self.tlp_rx_level    = Signal(16)
        self.phy_source_seen = Signal()  # driven from squirrel.py sys domain
        self.phy_raw_rx_seen = Signal()  # driven from squirrel.py sys domain (CDC from pcie)
        

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

        # Magic check: CMD/CFG/LOOP frames have 0x77 at rx64[7:0] (byte-built frames)
        # TLP frames (DeviceFPGA_TxTlp) have 0x77 at rx64[31:24] (DWORD-written flags)
        magic_ok  = Signal()
        pkt_type  = Signal(2)
        pkt_last  = Signal()
        pkt_data  = Signal(32)

        self.comb += [
            magic_ok .eq((rx64[0:8] == MAGIC) | (rx64[24:32] == MAGIC)),
            # For CMD/CFG/LOOP (magic at [7:0]): type at bits[9:8], last at bit[10]
            # For TLP/LOOP2 (magic at [31:24]): type at bits[17:16], last at bit[18]
            pkt_type .eq(Mux(rx64[24:32] == MAGIC, rx64[16:18], rx64[8:10])),
            pkt_last .eq(Mux(rx64[24:32] == MAGIC, rx64[18], rx64[10])),
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
        # Receives TLP words from USB, outputs to pcie_phy TX.
        # tlp_tx_suppress: after pkt_last=1, suppress further writes until next
        # TLP frame arrives (handles padding DWORDs that pcileech appends).
        # ===================================================================
        self.submodules.tlp_tx_fifo = tlp_tx_fifo = SyncFIFO(
            phy_layout(32), 256
        )
        tlp_tx_suppress = Signal()
        self.sync += [
            If(rx_is_tlp & pkt_last,
                tlp_tx_suppress.eq(1),
            ).Elif(rx_is_tlp & ~pkt_last & tlp_tx_suppress,
                # Next TLP started — the padding DW was suppressed, clear now
                tlp_tx_suppress.eq(0),
            )
        ]
        # pkt_data byte order: rx_word already byteswaps FT601 data to match
        # Xilinx PCIe IP s_axis_tx_tdata byte order (DW0 byte0 at tdata[7:0]).
        self.comb += [
            tlp_tx_fifo.sink.valid.eq(rx_is_tlp & ~(tlp_tx_suppress & pkt_last)),
            tlp_tx_fifo.sink.dat  .eq(pkt_data),
            tlp_tx_fifo.sink.be   .eq(0xf),
            tlp_tx_fifo.sink.last .eq(pkt_last),
            tlp_tx_fifo.source.connect(self.tlp_tx),
        ]




        # TLP Debug
        self.sync += [
            If(ResetSignal(),
               dbg_armed.eq(1),
               dbg_count.eq(0),
               dbg_seen.eq(0),
               ).Elif(dbg_armed & self.tlp_tx.valid & self.tlp_tx.ready,
                      dbg_seen.eq(1),
                      Case(dbg_count, {
                          0: [
                              self.tlp_tx_dbg0.eq(self.tlp_tx.dat),
                              self.tlp_tx_dbg_be0.eq(self.tlp_tx.be),
                              self.tlp_tx_dbg_last0.eq(self.tlp_tx.last),
                          ],
                          1: [
                              self.tlp_tx_dbg1.eq(self.tlp_tx.dat),
                              self.tlp_tx_dbg_be1.eq(self.tlp_tx.be),
                              self.tlp_tx_dbg_last1.eq(self.tlp_tx.last),
                          ],
                          2: [
                              self.tlp_tx_dbg2.eq(self.tlp_tx.dat),
                              self.tlp_tx_dbg_be2.eq(self.tlp_tx.be),
                              self.tlp_tx_dbg_last2.eq(self.tlp_tx.last),
                          ],
                          3: [
                              self.tlp_tx_dbg3.eq(self.tlp_tx.dat),
                              self.tlp_tx_dbg_be3.eq(self.tlp_tx.be),
                              self.tlp_tx_dbg_last3.eq(self.tlp_tx.last),
                              dbg_armed.eq(0),
                          ],
                      }),
                      If(dbg_count != 3,
                         dbg_count.eq(dbg_count + 1)
                         )
                      )
        ]
        








        

        # ===================================================================
        # TLP RX FIFO: PCIe→host  (256 deep, 32+1 bit)
        # Receives TLP words from pcie_phy RX, feeds TX mux port 3.
        # Filter: only Cpl (0b0000101x) and CplD (0b0100101x) pass through.
        # CfgRd/CfgWr and other TLPs from enumeration are discarded.
        # Matches ufrisk pcileech_tlps128_filter cfgtlp_filter=1 behavior.
        # ===================================================================
        self.submodules.tlp_rx_fifo = tlp_rx_fifo = SyncFIFO(
            phy_layout(32), 512
        )
        tlp_filter_bypass = Signal()  # wired to ~rw[202] after rw is defined
        # TLP filter state: track first beat and whether current TLP passes
        tlp_filter_first  = Signal(reset=1)   # next beat is first of TLP
        tlp_filter_pass   = Signal(reset=0)   # current TLP passes filter
        # First DWORD bits[31:25] = {Fmt[2:0], Type[4:3]} — check Cpl/CplD
        tlp_is_cpl = Signal()
        # PCIe IP delivers byte0 (TLP type) at tdata[7:0], so dat[7:0] = byte0.
        # byte0 = {R, Fmt[2:0], Type[4:0]}. Cpl=0x0a (fmt=000,type=01010),
        # CplD=0x4a (fmt=010,type=01010). Check bits[7:1] = {Fmt[2:0],Type[4:3]}:
        self.comb += tlp_is_cpl.eq(
            (self.tlp_rx.dat[1:8] == 0b0000101) |   # Cpl  byte0[7:1]=0b0000101
            (self.tlp_rx.dat[1:8] == 0b0100101)     # CplD byte0[7:1]=0b0100101
        )
        self.sync += [
            If(self.tlp_rx.valid & self.tlp_rx.ready,
                tlp_filter_first.eq(self.tlp_rx.last),
                If(tlp_filter_first,
                    tlp_filter_pass.eq(tlp_is_cpl),
                )
            )
        ]





        
        # Gate TLP RX into FIFO: first beat requires Cpl/CplD, subsequent beats follow filter state
        tlp_rx_gated_valid = Signal()
        self.comb += [
            # rw[202]=cfgtlp_filter_en: when 1 (default), only pass Cpl/CplD
            # when 0, pass all TLPs (useful for debugging — set reset value to 0)
            # tlp_filter_bypass=1 → pass all TLPs; =0 → only Cpl/CplD
            tlp_rx_gated_valid.eq(self.tlp_rx.valid & 
                Mux(tlp_filter_bypass,
                    1,
                    Mux(tlp_filter_first, tlp_is_cpl, tlp_filter_pass))),
            tlp_rx_fifo.sink.valid.eq(tlp_rx_gated_valid),
            tlp_rx_fifo.sink.dat  .eq(self.tlp_rx.dat),
            tlp_rx_fifo.sink.be   .eq(self.tlp_rx.be),
            tlp_rx_fifo.sink.last .eq(self.tlp_rx.last),
            self.tlp_rx.ready     .eq(tlp_rx_fifo.sink.ready),
        ]

        # Diagnostic: expose tlp_rx_fifo level + rx_seen counter via CMD register
        self.tlp_rx_level = Signal(16)
        rx_seen_count = Signal(16)
        self.sync += [
            If(self.tlp_rx.valid & self.tlp_rx.ready,
                rx_seen_count.eq(rx_seen_count + 1),
            )
        ]
        self.comb += self.tlp_rx_level.eq(Cat(
            tlp_rx_fifo.level[0:9],  # [8:0]  fifo fill level (9 bits for depth 512)
            rx_seen_count[0:5],      # [13:9] beats reaching self.tlp_rx
            self.phy_source_seen,    # [14]   pcie_phy.source.valid fired (after CDC)
            self.phy_raw_rx_seen,    # [15]   m_axis_rx_tvalid fired (raw PCIe IP RX)
        ))

        # -------------------------------------------------------------------
        # PCIe timeout diagnostics / recovery
        # -------------------------------------------------------------------
        TIMEOUT_CYCLES = 200_000_000  # 2 seconds at 100 MHz sys clock

        tx_axis_seen_count = Signal(8)
        rx_axis_seen_count = Signal(8)
        tx_err_drop_count  = Signal(8)

        tx_axis_ever     = Signal()
        rx_axis_ever     = Signal()
        tx_err_drop_ever = Signal()

        waiting_for_rx   = Signal()
        stall_counter    = Signal(max=TIMEOUT_CYCLES + 1)
        stall_hit        = Signal()
        reset_holdoff    = Signal(24)  # ~0.16s at 100 MHz

        # Snapshots captured when timeout hits
        snap_tx_axis_seen_count = Signal(8)
        snap_rx_axis_seen_count = Signal(8)
        snap_tx_err_drop_count  = Signal(8)
        snap_flags              = Signal(16)

        self.sync += [
            # Defaults
            self.diag_force_pcie_reset.eq(0),
            
            # Live counters / sticky bits
            If(self.diag_tx_axis_seen,
               tx_axis_seen_count.eq(tx_axis_seen_count + 1),
               tx_axis_ever.eq(1)
               ),
            If(self.diag_rx_axis_seen,
               rx_axis_seen_count.eq(rx_axis_seen_count + 1),
               rx_axis_ever.eq(1)
               ),
            If(self.diag_tx_err_drop,
               tx_err_drop_count.eq(tx_err_drop_count + 1),
               tx_err_drop_ever.eq(1)
               ),

            # Start watchdog on first TX beat, stop on any RX beat
            If(~waiting_for_rx & self.diag_tx_axis_seen,
               waiting_for_rx.eq(1),
               stall_counter.eq(0)
               ).Elif(waiting_for_rx & self.diag_rx_axis_seen,
                      waiting_for_rx.eq(0),
                      stall_counter.eq(0)
                      ).Elif(waiting_for_rx & ~stall_hit,
                             If(stall_counter < TIMEOUT_CYCLES,
                                stall_counter.eq(stall_counter + 1)
                                ).Else(
                                    stall_hit.eq(1),
                                    waiting_for_rx.eq(0),
                                    
                                    # Snapshot current diagnostics
                                    snap_tx_axis_seen_count.eq(tx_axis_seen_count),
                                    snap_rx_axis_seen_count.eq(rx_axis_seen_count),
                                    snap_tx_err_drop_count.eq(tx_err_drop_count),
                                    snap_flags.eq(Cat(
                                        tx_axis_ever,      # bit 0
                                        rx_axis_ever,      # bit 1
                                        tx_err_drop_ever,  # bit 2
                                        Constant(1, 1),    # bit 3 = stall_hit snapshot
                                        Constant(0, 12)
                                    )),
                                    
                                    # Start reset pulse
                                    reset_holdoff.eq((1 << 24) - 1)
                                )
                             ),
        
            # Automatic PCIe reset pulse after timeout
            If(reset_holdoff != 0,
               self.diag_force_pcie_reset.eq(1),
               reset_holdoff.eq(reset_holdoff - 1)
               ),

            # Optional: clear live TX/RX wait state after reset pulse finishes
            If((reset_holdoff == 1)),
            stall_hit.eq(0)
        ]
        




        

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
            cfg_cmd_read .eq(cfg_cmd_valid & ~ResetSignal()),  # gate on reset to prevent spurious boot responses
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

        # CFG register layout mirrors ufrisk pcileech_pcie_cfg_a7.sv ro[] exactly.
        # ro[] is byte-addressed; pcileech reads 16 bits at a time with byteswap:
        #   returned value = {ro[b+7:b], ro[b+15:b+8]} (lo byte first in FIFO)
        # We build cfg_ro_readback[15:0] = ro[b+15:b] directly; the FIFO path
        # bytswaps it correctly in cfg_tx_fifo.sink.data below.
        #
        # ro[85:80]  = pl_ltssm_state[5:0]      (byte 0x000a bits[5:0])
        # ro[87:86]  = pl_rx_pm_state[1:0]       (byte 0x000a bits[7:6])
        # ro[90:88]  = pl_tx_pm_state[2:0]       (byte 0x000b bits[2:0])
        # ro[93:91]  = pl_initial_link_width[2:0] (byte 0x000b bits[5:3])
        # ro[95:94]  = pl_lane_reversal[1:0]     (byte 0x000b bits[7:6])
        # ro[97:96]  = pl_sel_lnk_width[1:0]     (byte 0x000c bits[1:0])
        # ro[98]     = pl_phy_lnk_up              (byte 0x000c bit[2])
        # ro[99]     = pl_link_gen2_cap           (byte 0x000c bit[3])
        # ro[100]    = pl_link_partner_gen2_sup   (byte 0x000c bit[4])
        # ro[101]    = pl_link_upcfg_cap          (byte 0x000c bit[5])
        # ro[102]    = pl_sel_lnk_rate            (byte 0x000c bit[6])
        # ro[103]    = pl_directed_change_done    (byte 0x000c bit[7])

        # Derive pl_initial_link_width (3-bit count) from pl_sel_lnk_width (2-bit encoded)
        # sel=00→1, sel=01→2, sel=10→4
        initial_link_width = Signal(3)
        self.comb += Case(self.phy_lnk_width, {
            0b00: initial_link_width.eq(1),
            0b01: initial_link_width.eq(2),
            0b10: initial_link_width.eq(4),
            "default": initial_link_width.eq(1),
        })

        self.comb += Case(cfg_word_index, {
            0:  cfg_ro_readback.eq(0x6745),         # byte 0x00: wMagicPCIe
            4:  cfg_ro_readback.eq(self.phy_id),    # byte 0x08: real BDF from PCIe IP
            # byte 0x000a: ro[87:80] = {pl_rx_pm_state[1:0], pl_ltssm[5:0]}
            # byte 0x000b: ro[95:88] = {pl_lane_rev[1:0], pl_init_lnk_width[2:0], pl_tx_pm[2:0]}
            # ufrisk observed: 000a1608 → value=0x1608, bytes={0x08,0x16}
            #   byte0(0x000a)=0x16=pl_ltssm=22(L0), byte1(0x000b)=0x08=pl_init_lnk_width=1(x1)
            # word5: ufrisk returns 0x1608: byte0=0x08=lnk_width, byte1=0x16=ltssm
            5:  cfg_ro_readback.eq(Cat(
                    Constant(0, 3),         # ro[90:88] pl_tx_pm_state = 0
                    initial_link_width,     # ro[93:91] pl_initial_link_width (was self.phy_lnk_width[0:2])
                    Constant(0, 2),         # ro[95:94] pl_lane_reversal = 0
                    self.phy_ltssm[0:6],    # ro[85:80] bits[5:0] = ltssm
                    Constant(0, 2),         # ro[87:86] pl_rx_pm_state = 0
                )),
            # byte 0x000c: ro[103:96] = {directed_done,lnk_rate,upcfg,partner_gen2,gen2_cap,lnk_up,lnk_width[1:0]}
            # ufrisk observed: 000c7c00 → value=0x7c00, bytes={0x00,0x7c}
            #   byte0(0x000c)=0x7c=0b01111100: lnk_width=0b00,lnk_up=1,gen2=1,partner=1,upcfg=1,rate=1,done=0
            # word6: ufrisk returns 0x7c00: byte0=0x00, byte1=0x7c=lnk_up+caps
            # readback[7:0]=0, readback[15:8]=lnk_up/rate/caps byte
            6:  cfg_ro_readback.eq(Cat(
                    Constant(0, 8),         # byte 0x000d = 0 (low byte)
                    self.phy_lnk_width[0:2],# ro[97:96] pl_sel_lnk_width
                    self.phy_lnk_up,        # ro[98]    pl_phy_lnk_up
                    Constant(1, 1),         # ro[99]    pl_link_gen2_cap = 1
                    Constant(1, 1),         # ro[100]   pl_link_partner_gen2_supported = 1
                    Constant(1, 1),         # ro[101]   pl_link_upcfg_cap = 1
                    self.phy_lnk_rate,      # ro[102]   pl_sel_lnk_rate
                    Constant(0, 1),         # ro[103]   pl_directed_change_done = 0
            )),
            11: cfg_ro_readback.eq(rw[176:192]),    # byte 0x16: pl_directed_link_*
            12: cfg_ro_readback.eq(self.cfg_dcommand), # byte 0x18: cfg_dcommand
            "default": cfg_ro_readback.eq(0),
        })

        self.comb += [
            # Response format per DeviceFPGA_ConfigRead parser:
            #   dwData[15:0]  = _byteswap_ushort(wAddr | flags_C000)  → address echo
            #   dwData[31:16] = value (byte-swapped per SV convention)
            cfg_tx_fifo.sink.valid.eq(cfg_cmd_read),
            cfg_tx_fifo.sink.data .eq(Cat(
                cfg_ro_readback[0:8], cfg_ro_readback[8:16],  # X[15:0]  = value (lo byte, hi byte)
                cfg_addr_byte[0:8],   cfg_addr_byte[8:16],    # X[31:16] = addr (lo byte, hi byte)
            )),
            cfg_tx_fifo.sink.last .eq(1),
        ]


        # Named aliases for important rw bits
        rw_pcie_rst_core   = rw[200]
        rw_pcie_rst_subsys = rw[201]
        rw_cfgtlp_en       = rw[202]
        # Wire filter bypass: rw[202]=1 means filter ON (normal), 0 means bypass (debug)
        self.comb += tlp_filter_bypass.eq(~rw[202])
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
            in_addr_bit .eq(Cat(Constant(0, 3), cmd[16:31])),
            # SV: in_cmd_value = {cmd[48+:8], cmd[56+:8]}  (big-endian 16-bit)
            in_value    .eq(Cat(cmd[56:64], cmd[48:56])),
            # SV: in_cmd_mask  = {cmd[32+:8], cmd[40+:8]}
            in_mask     .eq(Cat(cmd[40:48], cmd[32:40])),
            f_rw        .eq(cmd[31]),          # bit15 of addr byte
            # bit14 of addr = shadow config space — we don't support that yet
            in_cmd_read .eq(cmd_valid & cmd[12] & ~cmd[30]),
            in_cmd_write.eq(cmd_valid & cmd[13] & ~cmd[30] & f_rw),
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
                rw[200]    .eq(0),         # PCIE CORE RESET off — sys_rst_n=1 so user_lnk_up can assert
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
        # pcie_rst_core driven by rw[200] BUT gated: only assert reset if
        # link is NOT up yet. Once link trains (phy_lnk_up=1), never reset.
        # This allows pcileech to clear rw[200] before link comes up,
        # while preventing reset from breaking an established link.
        self.comb += [
            self.pcie_rst_core  .eq(rw_pcie_rst_core),  # rw[200]=0 at reset → always 0
            self.pcie_rst_subsys.eq(rw_pcie_rst_subsys),
        ]

        # ===================================================================
        # Inactivity timer (matches pcileech_fifo.sv)
        #
        # rw[16]    = enable
        # rw[95:64] = tick count (how many ticks of inactivity before firing)
        #
        # A 64-bit free-running counter (tickcount64) increments every cycle.
        # When enabled and no CMD write has occurred for 'ticks' cycles,
        # a dummy CMD response word is pushed into cmd_tx_fifo.  This flows
        # through the mux → serializer → FT601 → USB, unblocking the host's
        # FT_ReadPipe call so pcileech can check completion status.
        #
        # The host configures the timer during init via CMD write to rw[16]
        # and rw[95:64].  Typical value: ~100k ticks = ~1ms at 100 MHz.
        # ===================================================================
        tickcount64        = Signal(64)
        inactivity_base    = Signal(64)
        inactivity_fire    = Signal()
        timer_enable       = Signal()
        timer_ticks        = Signal(32)

        self.comb += [
            timer_enable.eq(rw[16]),
            timer_ticks .eq(rw[64:96]),
        ]

        # Free-running 64-bit tick counter
        self.sync += tickcount64.eq(tickcount64 + 1)

        # Timer fires when: enabled, no CMD write this cycle, cmd_tx_fifo
        # has space, and enough ticks have elapsed since base was set.
        # Comparison: (inactivity_base + timer_ticks) < tickcount64
        # Use subtraction to avoid 64+32 bit add: (tickcount64 - inactivity_base) > timer_ticks
        inactivity_elapsed = Signal(64)
        self.comb += [
            inactivity_elapsed.eq(tickcount64 - inactivity_base),
            inactivity_fire.eq(
                timer_enable
                & ~in_cmd_write
                #& ~in_cmd_read
                #& ~cmd_rx_fifo.source.valid
                #& ~cfg_rx_fifo.source.valid
                #& ~tlp_tx_fifo.sink.valid
                & cmd_tx_fifo.sink.ready
                & (inactivity_elapsed > timer_ticks)
            ),
        ]

        # Update base: reset on TX activity (data flowing to host) or
        # output buffer full, matching SV pcileech_fifo.sv line 396:
        #   if ( dcom.com_din_wr_en | ~dcom.com_din_ready )
        #       _cmd_timer_inactivity_base <= tickcount64;
        # The SV resets when data is being SENT to host or the output
        # buffer is full.  The old code incorrectly reset on USB RX
        # activity (data FROM host), which meant the timer never fired
        # while the host was sending MRd TLPs even though no data was
        # flowing BACK to the host.
        #
        # tx_com_activity is driven later (after mux/serializer are defined)
        # from:  mux.valid | ~mux_out_fifo.sink.ready

        tx_com_activity = Signal()
        self.sync += [
            If(tx_com_activity,
                inactivity_base.eq(tickcount64),
            ).Elif(inactivity_fire,
                inactivity_base.eq(tickcount64),
                rw[16].eq(0)
            )
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
        DEVICE_ID     = 0x04   # Match ufrisk reference (DEVICE_ID=4 → small tags)

        rw_readback = Signal(16)
        ro_readback = Signal(16)
        readback    = Signal(16)

        # Explicit defaults before Case to prevent stale latch values
        self.comb += [rw_readback.eq(0), ro_readback.eq(0)]

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
            3: ro_readback.eq(self.tlp_rx_level),          # byte 0x06: tlp_rx_fifo fill level (diagnostic)
            4: ro_readback.eq(Cat(Constant(VERSION_MINOR,8),
                                  Constant(VERSION_MAJOR,8))),
            # DEVICE_ID response: need DWORD=000a0400 so pcileech uses small-tag profile
            # Cat puts first arg at LSB: Cat(lo, hi) → {hi, lo} in Verilog
            # We need readback[15:8]=DEVICE_ID so hi=DEVICE_ID → second arg has DEVICE_ID
            # Use named signals to ensure stable Migen elaboration order
            5: ro_readback.eq(Cat(Constant(0, 8), Constant(DEVICE_ID, 8))),  # DWORD=000a0400 → Tag=0x01 profile



            
            # FIXME: extra status readbacks
            6:  ro_readback.eq(Cat(snap_tx_axis_seen_count, snap_rx_axis_seen_count)),  # 0x000c
            7:  ro_readback.eq(Cat(snap_tx_err_drop_count, snap_flags[0:8])),            # 0x000e
            
            #9:  ro_readback.eq(self.tlp_tx_dbg0[0:16]),   # 0x0012 low half dbg0
            #10: ro_readback.eq(self.tlp_tx_dbg0[16:32]),  # 0x0014 high half dbg0
            #11: ro_readback.ebq(self.tlp_tx_dbg1[0:16]),   # 0x0016
            #12: ro_readback.eq(self.tlp_tx_dbg1[16:32]),  # 0x0018
            #13: ro_readback.eq(self.tlp_tx_dbg2[0:16]),   # 0x001a
            #14: ro_readback.eq(self.tlp_tx_dbg2[16:32]),  # 0x001c
            #15: ro_readback.eq(self.tlp_tx_dbg3[0:16]),   # 0x001e
            #16: ro_readback.eq(self.tlp_tx_dbg3[16:32]),  # 0x0020
            #17: ro_readback.eq(Cat(self.tlp_tx_dbg_be0, self.tlp_tx_dbg_be1,
            #                                              self.tlp_tx_dbg_be2, self.tlp_tx_dbg_be3)),           # 0x0022
            #18: ro_readback.eq(Cat(self.tlp_tx_dbg_last0, self.tlp_tx_dbg_last1,
            #                                              self.tlp_tx_dbg_last2, self.tlp_tx_dbg_last3,
            #                                              dbg_seen, dbg_armed, Constant(0, 10))),               # 0x0024

            #9:  ro_readback.eq(self.tx64_dbg0[ 0:16]),   # 0x0012
            #10: ro_readback.eq(self.tx64_dbg0[16:32]),   # 0x0014
            #11: ro_readback.eq(self.tx64_dbg0[32:48]),   # 0x0016
            #12: ro_readback.eq(self.tx64_dbg0[48:64]),   # 0x0018
            #13: ro_readback.eq(self.tx64_dbg1[ 0:16]),   # 0x001a
            #14: ro_readback.eq(self.tx64_dbg1[16:32]),   # 0x001c
            #15: ro_readback.eq(self.tx64_dbg1[32:48]),   # 0x001e
            #16: ro_readback.eq(self.tx64_dbg1[48:64]),   # 0x0020
            #17: ro_readback.eq(self.tx64_dbg_flags),     # 0x0022

            9:  ro_readback.eq(self.txsink_dbg0[ 0:16]),   # 0x0012
            10: ro_readback.eq(self.txsink_dbg0[16:32]),   # 0x0014
            11: ro_readback.eq(self.txsink_dbg0[32:48]),   # 0x0016
            12: ro_readback.eq(self.txsink_dbg0[48:64]),   # 0x0018
            13: ro_readback.eq(self.txsink_dbg1[ 0:16]),   # 0x001a
            14: ro_readback.eq(self.txsink_dbg1[16:32]),   # 0x001c
            15: ro_readback.eq(self.txsink_dbg1[32:48]),   # 0x001e
            16: ro_readback.eq(self.txsink_dbg1[48:64]),   # 0x0020
            17: ro_readback.eq(Cat(self.txsink_dbg_be0, self.txsink_dbg_be1)),   # 0x0022
            18: ro_readback.eq(Cat(self.txsink_dbg_last0,
                                                          self.txsink_dbg_last1,
                                                          self.txsink_dbg_flags[2:16])),                 # 0x0024


            19: ro_readback.eq(self.rxsink_dbg[0][ 0:16]),   # 0x0026
            20: ro_readback.eq(self.rxsink_dbg[0][16:32]),   # 0x0028
            21: ro_readback.eq(self.rxsink_dbg[0][32:48]),   # 0x002a
            22: ro_readback.eq(self.rxsink_dbg[0][48:64]),   # 0x002c
        
            23: ro_readback.eq(self.rxsink_dbg[1][ 0:16]),   # 0x0026
            24: ro_readback.eq(self.rxsink_dbg[1][16:32]),   # 0x0028
            25: ro_readback.eq(self.rxsink_dbg[1][32:48]),   # 0x002a
            26: ro_readback.eq(self.rxsink_dbg[1][48:64]),   # 0x002c

            27: ro_readback.eq(self.rxsink_dbg[2][ 0:16]),   # 0x0026
            28: ro_readback.eq(self.rxsink_dbg[2][16:32]),   # 0x0028
            29: ro_readback.eq(self.rxsink_dbg[2][32:48]),   # 0x002a
            30: ro_readback.eq(self.rxsink_dbg[2][48:64]),   # 0x002c

            31: ro_readback.eq(self.rxsink_dbg[3][ 0:16]),   # 0x0026
            32: ro_readback.eq(self.rxsink_dbg[3][16:32]),   # 0x0028
            33: ro_readback.eq(self.rxsink_dbg[3][32:48]),   # 0x002a
            34: ro_readback.eq(self.rxsink_dbg[3][48:64]),   # 0x002c
            
            35: ro_readback.eq(self.rxsink_dbg[4][ 0:16]),   # 0x0026
            36: ro_readback.eq(self.rxsink_dbg[4][16:32]),   # 0x0028
            37: ro_readback.eq(self.rxsink_dbg[4][32:48]),   # 0x002a
            38: ro_readback.eq(self.rxsink_dbg[4][48:64]),   # 0x002c
        
            39: ro_readback.eq(self.rxsink_dbg[5][ 0:16]),   # 0x0026
            40: ro_readback.eq(self.rxsink_dbg[5][16:32]),   # 0x0028
            41: ro_readback.eq(self.rxsink_dbg[5][32:48]),   # 0x002a
            42: ro_readback.eq(self.rxsink_dbg[5][48:64]),   # 0x002c
        
            43: ro_readback.eq(self.rxsink_dbg[6][ 0:16]),   # 0x0026
            44: ro_readback.eq(self.rxsink_dbg[6][16:32]),   # 0x0028
            45: ro_readback.eq(self.rxsink_dbg[6][32:48]),   # 0x002a
            46: ro_readback.eq(self.rxsink_dbg[5][48:64]),   # 0x002c
        
            47: ro_readback.eq(self.rxsink_dbg[7][ 0:16]),   # 0x0026
            48: ro_readback.eq(self.rxsink_dbg[7][16:32]),   # 0x0028
            49: ro_readback.eq(self.rxsink_dbg[7][32:48]),   # 0x002a
            50: ro_readback.eq(self.rxsink_dbg[7][48:64]),   # 0x002c
    
            51: ro_readback.eq(Cat(self.rxsink_be[0], self.rxsink_be[1])),  # 0x0060
            52: ro_readback.eq(Cat(self.rxsink_be[2], self.rxsink_be[3])),  # 0x0062
            53: ro_readback.eq(Cat(self.rxsink_be[4], self.rxsink_be[5])),  # 0x0064
            54: ro_readback.eq(Cat(self.rxsink_be[6], self.rxsink_be[7])),  # 0x0066
            55: ro_readback.eq(Cat(self.rxsink_lasts, self.rxsink_flags[0:8])),  # 0x0068

            56: ro_readback.eq(self.diag_rx64_seen),       # 0x0070
            57: ro_readback.eq(self.diag_rx32_seen),       # 0x0072
            58: ro_readback.eq(self.diag_rxfifo_in_seen),  # 0x0074
            59: ro_readback.eq(self.diag_rxfifo_out_seen), # 0x0076
            60: ro_readback.eq(self.diag_mux_p3_wr_seen),  # 0x0078
            61: ro_readback.eq(self.diag_ser_out_seen),    # 0x007a
            62: ro_readback.eq(self.diag_usbtx_seen),      # 0x007c
            
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
        # CMD TX response: either a real CMD read response or an inactivity
        # timer keepalive.  CMD reads take priority.
        #
        # Keepalive data: looks like a CMD read of ro[0] (magic=0xab89).
        # pcileech parses it as a normal CMD frame, doesn't match any
        # pending register read, and discards it.  The important thing
        # is that data flows through the mux → FT601 → USB so the host's
        # FT_ReadPipe unblocks.
        cmd_tx_valid = Signal()
        cmd_tx_data  = Signal(32)

        self.comb += [
            If(in_cmd_read,
                cmd_tx_valid.eq(1),
                cmd_tx_data .eq(Cat(
                    readback[0:8], readback[8:16],
                    in_addr_byte[0:8], in_addr_byte[8:16],
                )),
            ).Elif(inactivity_fire,
                cmd_tx_valid.eq(1),
                # Keepalive word: match ufrisk pcileech_fifo.sv inactivity marker
                cmd_tx_data .eq(Cat(
                    Constant(0xDE, 8), Constant(0xCE, 8),  # value = 0xCEDE
                    Constant(0xFF, 8), Constant(0xFF, 8),  # addr  = 0xFFFF
                )),
            ).Else(
                cmd_tx_valid.eq(0),
                cmd_tx_data .eq(0),
            ),
            cmd_tx_fifo.sink.valid.eq(cmd_tx_valid & ~ResetSignal()),
            cmd_tx_fifo.sink.data .eq(cmd_tx_data),
            cmd_tx_fifo.sink.last .eq(1),
        ]

        # ===================================================================
        # TX PATH: mux + serialize → USB
        # ===================================================================
        # if registered == 0 it seems we get 1374 for a single 4k page instead of 13b4 and stalls there too
        self.submodules.mux        = mux        = PCILeechMux(nports=8,registered=0)
        self.submodules.serializer = serializer = MuxSerializer()

        

        # Mux rd_en driven by serializer being idle
        #self.comb += mux.rd_en.eq(serializer.sink.ready)
        
        # Connect mux output to serializer
        if 1:
            # Decouple mux from serializer with a deeper FIFO.
            # The SV reference uses a ~2KB 256→32 async FIFO here.
            # With depth 32, we get 32×32=1KB of buffering which absorbs
            # FT601 bus turnaround stalls without backpressuring the mux.
            self.submodules.mux_out_fifo = mux_out_fifo = SyncFIFO([("data", 256)], 32)

            self.comb += [
                # Mux writes frames into a small FIFO / skid buffer.
                mux_out_fifo.sink.valid.eq(mux.valid),
                mux_out_fifo.sink.data.eq(mux.dout),
                mux.rd_en.eq(mux_out_fifo.sink.ready),

                # Serializer reads whole frames from the FIFO.
                mux_out_fifo.source.connect(serializer.sink),
            ]
        elif 0:
            self.comb += [
                serializer.sink.valid.eq(mux.valid),
                serializer.sink.data .eq(mux.dout),
                mux.rd_en.eq(serializer.sink.ready),
            ]
        else:
            self.submodules.txpipe = txpipe = MuxWordQueueTX(idle_threshold=64, word_fifo_depth = 32, start_wait_max=16)
            
            self.comb += [
                txpipe.sink.valid.eq(mux.valid),
                txpipe.sink.data.eq(mux.dout),
                mux.rd_en.eq(txpipe.sink.ready),
            ]
            serializer=txpipe

        # Connect serializer output to USB TX with byte-swap and sync injection.
        #
        # FT601 short-packet prevention (ft601_bug_workaround equivalent):
        # The SV reference (pcileech_com.sv line 198) injects 0x66665555 sync
        # words whenever the FT601-side FIFO is nearly empty.  Without this,
        # gaps in the serializer output cause the FT601 chip's internal TX
        # buffer to drain → chip sends a USB short packet → USB bulk IN
        # transfer terminates prematurely → host gets partial data.
        #
        # We implement this by tracking whether data recently flowed.  When
        # the serializer goes quiet mid-burst, we inject sync words to keep
        # the write_fifo (and thus the FT601 chip) fed.  The host's RX parser
        # already strips sync words, so these are harmless.
        #
        # FT601 swaps bytes on TX (pcileech_ft601.sv line 36), so we pre-swap
        # to cancel it out and deliver correct byte order to the host.

        tx_out_data  = Signal(32)
        tx_out_valid = Signal()

        # Sync word injection state
        recently_active = Signal(max=2048)
        inject_sync     = Signal()

        self.sync += [
            If(serializer.source.valid & self.usb_tx.ready,
                # Real data flowing — reset the injection counter.
                # 1500 cycles ≈ 15 µs at 100 MHz, long enough to cover
                # FT601 RX mode + bus turnaround + serializer gaps.
                recently_active.eq(1500),
            ).Elif((recently_active > 0) & self.usb_tx.ready,
                recently_active.eq(recently_active - 1),
            )
        ]
        self.comb += inject_sync.eq(~serializer.source.valid & (recently_active > 0))

        # Mux real data with injected sync words
        self.comb += [
            If(serializer.source.valid,
                tx_out_data.eq(serializer.source.data),
                tx_out_valid.eq(1),
            ).Elif(inject_sync,
                tx_out_data.eq(0x66665555),
                tx_out_valid.eq(1),
            ).Else(
                tx_out_data.eq(0),
                tx_out_valid.eq(0),
            ),
            serializer.source.ready.eq(self.usb_tx.ready),
        ]

        # Byte-swap and drive USB TX
        tx_swapped = Signal(32)
        self.comb += tx_swapped.eq(Cat(
            tx_out_data[24:32],
            tx_out_data[16:24],
            tx_out_data[ 8:16],
            tx_out_data[ 0: 8],
        ))
        self.comb += [
            self.usb_tx.valid.eq(tx_out_valid),
            self.usb_tx.data .eq(tx_swapped),
        ]

        # Drive the inactivity timer reset from TX activity
        # (defined earlier as forward-declared signal tx_com_activity)
        # Matches SV: dcom.com_din_wr_en | ~dcom.com_din_ready
        # mux.valid = frame being emitted; serializer output = data reaching FT601
        self.comb += tx_com_activity.eq(
            (serializer.source.valid & serializer.source.ready) |
            mux.valid
        )

        # -------------------------------------------------------------------
        # Mux port wiring — matches pcileech_fifo.sv port priority:
        #
        # SV (pcileech_fifo.sv lines 148-179):
        #   p0=TLP(tag=00)  HIGHEST PRIORITY
        #   p1=CFG(tag=01)
        #   p2=LOOPBACK(tag=10)
        #   p3=CMD(tag=11)  LOWEST PRIORITY
        #
        # In the SV mux, nibble = (p_ctx << 2) | port_number.
        # bits[1:0] of nibble = port number = type tag.
        # In our PCILeechMux, the tag is embedded in p_ctx directly
        # (the mux doesn't add a port-number tag).  So the port number
        # only affects scheduling priority, not the tag encoding.
        #
        # The host parses (nibble & 0x03) to determine the type:
        #   0x00 = TLP, 0x01 = CFG, 0x02 = LOOP, 0x03 = CMD
        # -------------------------------------------------------------------

        # p0: TLP RX (PCIe → host) — HIGHEST PRIORITY (matches SV p0)
        # ctx nibble: bits[1:0]=TYPE_TLP=0b00, bit[2]=last, bit[3]=0
        self.comb += [
            mux.p_din[0].eq(tlp_rx_fifo.source.dat),
            mux.p_ctx[0].eq(Cat(Constant(0b00, 2),           # bits[1:0] = TYPE_TLP tag
                                tlp_rx_fifo.source.last,     # bit[2] = last
                                Constant(0, 1))),             # bit[3] = 0
            mux.p_wr [0].eq(tlp_rx_fifo.source.valid & mux.p_req[0]),
            tlp_rx_fifo.source.ready.eq(mux.p_req[0]),
        ]

        # p1: CFG response — tag=0b01, ctx=0b00
        self.comb += [
            mux.p_din[1].eq(cfg_tx_fifo.source.data),
            mux.p_ctx[1].eq(0b0001),
            mux.p_wr [1].eq(cfg_tx_fifo.source.valid & mux.p_req[1]),
            cfg_tx_fifo.source.ready.eq(mux.p_req[1]),
        ]

        # p2: loopback — tag=0b10, ctx from stored loop_fifo.ctx
        self.comb += [
            mux.p_din[2].eq(loop_fifo.source.data),
            mux.p_ctx[2].eq(Cat(Signal(2, reset=0b10), loop_fifo.source.ctx)),
            mux.p_wr [2].eq(loop_fifo.source.valid & mux.p_req[2]),
            loop_fifo.source.ready.eq(mux.p_req[2]),
        ]

        # p3: CMD response — tag=0b11, ctx=0b00 — LOWEST PRIORITY
        self.comb += [
            mux.p_din[3].eq(cmd_tx_fifo.source.data),
            mux.p_ctx[3].eq(0b0011),
            mux.p_wr [3].eq(cmd_tx_fifo.source.valid & mux.p_req[3]),
            cmd_tx_fifo.source.ready.eq(mux.p_req[3]),
        ]

        # p4-p7: stubs
        for i in range(4, 8):
            self.comb += [
                mux.p_din[i].eq(0),
                mux.p_ctx[i].eq(0),
                mux.p_wr [i].eq(0),
            ]

        if 1:
            self.sync += [
                    If(tlp_rx_fifo.sink.valid & tlp_rx_fifo.sink.ready,rxfifo_in_seen.eq(rxfifo_in_seen + 1)),
                    If(tlp_rx_fifo.source.valid & tlp_rx_fifo.source.ready,rxfifo_out_seen.eq(rxfifo_out_seen + 1)),
                    # breaks things:
                    #If(mux.p_wr[3],mux_p3_wr_seen.eq(mux_p3_wr_seen + 1)),
                ]
            self.comb += [
                self.diag_rxfifo_in_seen.eq(rxfifo_in_seen),
                self.diag_rxfifo_out_seen.eq(rxfifo_out_seen),
                self.diag_mux_p3_wr_seen.eq(mux_p3_wr_seen),
            ]
        


            
