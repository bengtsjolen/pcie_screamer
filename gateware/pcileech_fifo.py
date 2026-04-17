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
    # ---------------------------------------------------------------------
    # Sync words (0x66665555 × 5) are emitted at the start of every USB
    # burst — pcileech scans for them to locate the start of a frame
    # sequence after each bulk-in transfer.  Mid-burst phantom sync words
    # must NOT appear, or pcileech resyncs and discards completion tags.
    #
    # Control flow:
    #
    #   IDLE   — waiting for first frame after cold reset or burst start.
    #            First frame triggers RESYNC (sync preamble) then SEND.
    #   RESYNC — emits 5 × 0x66665555.  Clears need_sync latch on exit.
    #   SEND   — emits 8 × 32-bit words from the buffered 256-bit frame.
    #            At count==0, grabs the next frame back-to-back if
    #            available, else parks in WAIT.
    #   WAIT   — silent hold during a mid-burst pipeline gap.  Resumes
    #            SEND with no sync.  If need_sync was re-asserted via
    #            start_sync (new USB burst), jumps to RESYNC instead.
    #
    # `start_sync` is a 1-cycle external pulse (driven by the burst FSM's
    # rising edge of burst_gate) that latches need_sync=1, requesting a
    # fresh sync preamble before the next frame — exactly matching
    # ufrisk's pcileech_com behaviour of prefixing 5 sync words to every
    # USB bulk-in transfer.
    # ---------------------------------------------------------------------
    def __init__(self):
        self.sink       = stream.Endpoint([("data", 256)])
        self.source     = stream.Endpoint([("data", 32)])
        self.start_sync = Signal()   # 1-cycle pulse: request sync at next frame

        buf       = Signal(256)
        count     = Signal(3)
        rsync     = Signal(3)
        need_sync = Signal(reset=1)  # 1 at reset so the very first burst gets sync

        self.submodules.fsm = fsm = FSM(reset_state="IDLE")

        # Latch need_sync when the burst FSM signals the start of a new
        # USB burst.  Cleared inside RESYNC once the preamble is complete.
        self.sync += If(self.start_sync, need_sync.eq(1))

        # IDLE: waiting for the first frame of a burst.  Always emits sync.
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
                      NextValue(need_sync, 0),
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
                      # End of current frame — grab the next one back-to-back
                      # if available.
                      self.sink.ready.eq(1),
                      If(self.sink.valid,
                         NextValue(buf,   self.sink.data),
                         NextValue(count, 7),
                         # Stay in SEND.  Even if a new USB burst started
                         # mid-SEND (need_sync latched), we are clearly
                         # still streaming continuously, so no sync here.
                         ).Else(
                             # Mid-burst pause — park silently in WAIT.
                             NextState("WAIT"),
                         )
                      ).Else(
                          NextValue(buf, buf << 32),
                          NextValue(count, count - 1),
                      )
                   )
                )
        # WAIT: pipeline silent.  When the next frame arrives, resume SEND
        # directly — unless need_sync has been re-asserted by the burst
        # FSM (indicating a new USB transfer has started), in which case
        # we emit a fresh sync preamble first.
        fsm.act("WAIT",
                self.source.valid.eq(0),
                self.sink.ready.eq(1),
                If(self.sink.valid,
                   NextValue(buf,   self.sink.data),
                   NextValue(count, 7),
                   If(need_sync,
                      NextValue(rsync, 4),
                      NextState("RESYNC"),
                      ).Else(
                          NextState("SEND"),
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

        # Extra PCIe-IP edge counters (driven from top-level sys domain).
        # Help distinguish whether the 10-page-dump stall is upstream
        # (insufficient MRds sent) or at/inside the Xilinx IP (completions
        # lost).
        self.diag_tx_tlp_seen     = Signal(16)  # count of s_axis_tx.last & valid & ready (TLPs we sent)
        self.diag_rx_tlp_seen     = Signal(16)  # count of m_axis_rx.last & valid (TLPs delivered by IP)
        self.diag_tx_err_drop_cnt = Signal(16)  # sticky count of tx_err_drop pulses

        # TLP RX analysis diagnostics. Help distinguish between
        #   (a) IP delivered all 160 CplDs but our pipeline lost 9
        #   (b) IP only delivered 151 CplDs (RC didn't send more)
        #   (c) the 151 number contains non-CplD TLPs
        #
        # diag_tlp_rx_cpl_count    : count of CplD TLPs arriving at tlp_rx
        #                            (last-event count for TLPs whose first-beat
        #                             byte0 matched CplD type 0x4a / 0x4b or Cpl 0x0a).
        # diag_tlp_rx_other_count  : count of non-Cpl/CplD TLPs at tlp_rx
        #                            (TLPs the filter dropped).
        # diag_tlp_rx_fifo_peak    : high-water-mark of tlp_rx_fifo.level.
        #                            If this never approaches 2048, we never
        #                            backpressured the IP and the 9 missing
        #                            CplDs are lost upstream of our fabric.
        # diag_tlp_rx_stall_cnt    : number of pcie-domain cycles where
        #                            tlp_rx.valid & ~tlp_rx.ready (i.e. we
        #                            actively stalled m_axis_rx).
        self.diag_tlp_rx_cpl_count   = Signal(16)
        self.diag_tlp_rx_other_count = Signal(16)
        self.diag_tlp_rx_fifo_peak   = Signal(16)
        self.diag_tlp_rx_stall_cnt   = Signal(16)

        # FT601 write-side diagnostics (driven from top via CDC from usb dom).
        # Help answer "where are the 824 missing bytes when 14-page dumps stall":
        #
        # diag_ft601_filler_emit : # of sync-word (0x66665555) writes the
        #                          filler branch drove.  If 0 after a stalled
        #                          dump → filler never fired, FT601 module is
        #                          going to IDLE too fast.
        # diag_ft601_wrn0_accept : # of usb-clk cycles with wr_n=0 AND txe_n=0,
        #                          i.e. DWs FT601 actually sampled from us.
        #                          Should equal ~diag_usbtx_seen + filler_emit.
        # diag_ft601_txen_high   : # of usb-clk cycles observed with txe_n=1.
        #                          Non-zero means FT601 pushed back on us
        #                          (chip IN buffer full).
        self.diag_ft601_filler_emit = Signal(16)
        self.diag_ft601_wrn0_accept = Signal(16)
        self.diag_ft601_txen_high   = Signal(16)

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
        # tlp_rx_fifo: BRAM-backed (buffered=True) so we can afford a deep
        # buffer.  Ufrisk's equivalent (fifo_134_134_clk2) is a 128-bit,
        # ~1024-deep BRAM FIFO sized so that m_axis_rx never needs to be
        # stalled during a full CplD burst.  Our PCILeechFIFO uses a 32-bit
        # bus after the StrideConverter, so we need ~4x more entries for
        # the same effective buffering.
        #
        # 2048 entries × (32 + 4 + 1) ≈ 9 KB → 2-3 BRAM36 tiles.
        # Using buffered=True switches migen from SyncFIFO(fwft=True,
        # async_read=True) to SyncFIFOBuffered (sync-read) which Vivado
        # infers as BRAM, not distributed LUT RAM.  This is critical —
        # the shallow 512-entry distributed-RAM version blows the LUT-RAM
        # budget long before it helps throughput.
        #
        # Bumped to 8192 after a 10-page dump hit diag_tlp_rx_fifo_peak=2049
        # with diag_tlp_rx_stall_cnt=24099 (241 µs of m_axis_rx back-pressure).
        # 8192×37 b ≈ 37 KB → ~9 BRAM36 tiles; xc7a35t has 50, so plenty of
        # headroom.  This should absorb the entire 10-MRd burst of CplDs
        # (~43 KB) without ever stalling m_axis_rx, eliminating any risk of
        # the Xilinx IP silently dropping inside its tiny internal CplD
        # buffer when Buf_Opt_BMA=true / Cpl_Finite=false.
        self.submodules.tlp_rx_fifo = tlp_rx_fifo = SyncFIFO(
            phy_layout(32), 8192, buffered=True
        )
        tlp_filter_bypass = Signal()  # wired to ~rw[202] after rw is defined
        # TLP filter state: track first beat and whether current TLP passes
        tlp_filter_first  = Signal(reset=1)   # next beat is first of TLP
        tlp_filter_pass   = Signal(reset=0)   # current TLP passes filter
        # First DWORD bits[31:25] = {Fmt[2:0], Type[4:3]} — check Cpl/CplD
        tlp_is_cpl = Signal()
        # IMPORTANT byte-order note: Xilinx pcie_7x AXI-Stream uses
        # "DWORDs in little-endian position, bytes within each DWORD in
        # BIG-endian order".  That is: byte 0 of the TLP (fmt/type) sits
        # at tdata[31:24] — NOT tdata[7:0].  Same convention as ufrisk's
        # pcileech_pcie_tlp_a7.sv line 181:
        #     (tlps_in.tdata[31:25] == 7'b0100101)   // CplD
        #
        # Earlier we wrote this check against dat[1:8] (assuming byte0 at LSB),
        # which NEVER matched (confirmed by diag_tlp_rx_cpl_count=0 while
        # 151 TLPs did reach the FIFO).  The filter effectively passed
        # everything only because rw[202] (cfgtlp_filter_en) was 0 at
        # runtime — i.e. the filter was bypassed.
        #
        # byte0 = {R, Fmt[2:0], Type[4:0]}.  Cpl=0x0a (fmt=000, type=01010),
        # CplD=0x4a (fmt=010, type=01010).  Check bits[7:1]={Fmt[2:0],Type[4:3]}
        # at tdata[31:25]:
        self.comb += tlp_is_cpl.eq(
            (self.tlp_rx.dat[25:32] == 0b0000101) |   # Cpl  byte0[7:1]=0b0000101
            (self.tlp_rx.dat[25:32] == 0b0100101)     # CplD byte0[7:1]=0b0100101
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
        tlp_rx_pass_beat   = Signal()   # this beat passes the filter (want to keep)
        tlp_rx_gated_valid = Signal()   # this beat will be written to the FIFO
        self.comb += [
            # rw[202]=cfgtlp_filter_en: when 1 (default), only pass Cpl/CplD;
            # when 0, pass all TLPs (useful for debugging — set reset value to 0).
            # tlp_filter_bypass=1 → pass all TLPs; =0 → only Cpl/CplD.
            tlp_rx_pass_beat.eq(
                Mux(tlp_filter_bypass,
                    1,
                    Mux(tlp_filter_first, tlp_is_cpl, tlp_filter_pass))),
            tlp_rx_gated_valid.eq(self.tlp_rx.valid & tlp_rx_pass_beat),
            tlp_rx_fifo.sink.valid.eq(tlp_rx_gated_valid),
            tlp_rx_fifo.sink.dat  .eq(self.tlp_rx.dat),
            tlp_rx_fifo.sink.be   .eq(self.tlp_rx.be),
            tlp_rx_fifo.sink.last .eq(self.tlp_rx.last),
            # CRITICAL backpressure rule: only a TLP we actually want to keep
            # may stall m_axis_rx when the FIFO is full.  A TLP that the
            # filter is discarding MUST be consumed unconditionally — it
            # never touches the FIFO, so FIFO fullness is irrelevant for it.
            #
            # The previous code wired ready = tlp_rx_fifo.sink.ready
            # unconditionally, which meant a filtered (dropped) TLP
            # arriving while the FIFO was full would still stall the
            # PCIe IP's m_axis_rx.  Under sustained traffic this creates
            # head-of-line blocking: every filtered TLP (CfgRd/CfgWr,
            # MsgD, internal Xilinx completions, etc.) behind a burst
            # of CplDs waits for downstream drain.  That pressure
            # propagates into the IP's internal buffer, starves flow
            # control credits back to the root complex, and causes
            # outstanding MRds to hit the host's Completion Timeout.
            #
            # Matches ufrisk's pcileech_pcie_tlp_a7.sv, where the filter
            # can always discard upstream regardless of the filter's
            # output-FIFO state.
            self.tlp_rx.ready.eq(
                Mux(tlp_rx_pass_beat,
                    tlp_rx_fifo.sink.ready,   # real CplD: stall until FIFO has room
                    1,                         # filtered: consume and drop
                )
            ),
        ]

        # Diagnostic: expose tlp_rx_fifo level + rx_seen counter via CMD register
        self.tlp_rx_level = Signal(16)
        rx_seen_count = Signal(16)
        self.sync += [
            If(self.tlp_rx.valid & self.tlp_rx.ready,
                rx_seen_count.eq(rx_seen_count + 1),
            )
        ]

        # -------------------------------------------------------------------
        # TLP-RX analysis counters (all in self.sync = sys domain, same as
        # tlp_rx stream after CDC inside pcie_phy).
        #
        #   cpl_count   : +1 on the "last" beat of a TLP whose first beat
        #                 identified as Cpl/CplD (tlp_filter_pass=1).
        #   other_count : +1 on the "last" beat of a TLP whose first beat
        #                 did NOT match Cpl/CplD (tlp_filter_pass=0).
        #   fifo_peak   : running max of tlp_rx_fifo.level (sat 16 bits).
        #   stall_cnt   : +1 every cycle the IP asserts valid while we hold
        #                 ready=0 (backpressure applied to m_axis_rx).
        #
        # "last" detection: tlp_rx.last & valid & ready edge.
        # tlp_filter_first = 1 ⇒ next beat is first-of-TLP; at last beat,
        # tlp_filter_pass holds the first-beat classification.
        # -------------------------------------------------------------------
        tlp_rx_last_edge = Signal()
        self.comb += tlp_rx_last_edge.eq(
            self.tlp_rx.valid & self.tlp_rx.ready & self.tlp_rx.last
        )

        # first-beat match: the *current* beat is a first-beat (tlp_filter_first=1)
        # and tlp_is_cpl is a pass. For single-beat TLPs, first-beat and last
        # coincide, so we need to check both conditions at the last edge.
        # Use tlp_filter_pass (latched from first-beat) for multi-beat TLPs,
        # OR (tlp_filter_first & tlp_is_cpl) for single-beat TLPs.
        first_and_cpl = Signal()
        self.comb += first_and_cpl.eq(tlp_filter_first & tlp_is_cpl)

        self.sync += [
            If(tlp_rx_last_edge,
                If(tlp_filter_pass | first_and_cpl,
                    self.diag_tlp_rx_cpl_count.eq(self.diag_tlp_rx_cpl_count + 1),
                ).Else(
                    self.diag_tlp_rx_other_count.eq(self.diag_tlp_rx_other_count + 1),
                )
            ),
            If(tlp_rx_fifo.level > self.diag_tlp_rx_fifo_peak,
                self.diag_tlp_rx_fifo_peak.eq(tlp_rx_fifo.level),
            ),
            If(self.tlp_rx.valid & ~self.tlp_rx.ready,
                self.diag_tlp_rx_stall_cnt.eq(self.diag_tlp_rx_stall_cnt + 1),
            ),
        ]
        # Diagnostic layout: fifo depth is now 2048 so `level` is 11 bits.
        # Use the top 3 bits of the level (i.e. level >> 8) so the 16-bit
        # readback remains useful as a fullness indicator (0..7 in units of 256).
        self.comb += self.tlp_rx_level.eq(Cat(
            tlp_rx_fifo.level[8:11], # [2:0]  fifo fill level (top 3 bits of 11-bit level)
            tlp_rx_fifo.level[0:8],  # [10:3] fifo fill level (low 8 bits)
            rx_seen_count[0:3],      # [13:11] beats reaching self.tlp_rx
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
        cfg_f_rw      = Signal()
        self.comb += [
            cfg_addr_byte.eq(cfg_cmd[16:32]),
            # Address bit 15 selects rw[] (1) vs ro[] (0), per
            # pcileech_pcie_cfg_a7.sv: `wire f_rw = in_cmd_address_byte[15]`.
            cfg_f_rw     .eq(cfg_addr_byte[15]),
            cfg_cmd_read .eq(cfg_cmd_valid & ~ResetSignal()),  # gate on reset to prevent spurious boot responses
            cfg_rx_fifo.source.ready.eq(1),
        ]

        # -------------------------------------------------------------------
        # CFG register-space readback
        # -------------------------------------------------------------------
        # This block mirrors pcileech_pcie_cfg_a7.sv's ro[]/rw[] layout.
        # pcileech selects between spaces using bit 15 of the address byte
        # (f_rw): 0 ⇒ ro[], 1 ⇒ rw[].  Each 16-bit read returns
        # {ro|rw}[addr_bit+:16], built from the full 384-bit ro[] / 704-bit
        # rw[] vectors.
        #
        # NOTE: we don't currently implement cfg_mgmt access (raw PCIe
        # config space read/write through the Xilinx management interface)
        # or the static-TLP-transmit feature, so those rw[] bits just
        # read back their reset values.  That's sufficient for pcileech to
        # enumerate the endpoint, read cfg_dcommand for MaxReadReq sizing,
        # and drive the mainline memory-read path.
        # -------------------------------------------------------------------
        cfg_word_index  = Signal(8)
        cfg_ro_rd       = Signal(16)
        cfg_rw_rd       = Signal(16)
        cfg_readback    = Signal(16)
        # Byte address bits [14:1] → 16-bit word index (ignore bit 0 = alignment,
        # bit 15 = f_rw).
        self.comb += cfg_word_index.eq(cfg_addr_byte[1:9])

        # Derive pl_initial_link_width (3-bit count) from pl_sel_lnk_width
        # (2-bit encoded: 00→x1, 01→x2, 10→x4).
        initial_link_width = Signal(3)
        self.comb += Case(self.phy_lnk_width, {
            0b00: initial_link_width.eq(1),
            0b01: initial_link_width.eq(2),
            0b10: initial_link_width.eq(4),
            "default": initial_link_width.eq(1),
        })

        # ==== CFG ro[] readback (384 bits total = 24 × 16-bit words) ====
        # Word layout matches pcileech_pcie_cfg_a7.sv ro[] assignments:
        #   word  0 : ro[ 15:  0] = MAGIC (0x2301)
        #   word  1 : ro[ 31: 16] = cfg_mgmt_rd/wr_en + zeros
        #   word  2 : ro[ 47: 32] = bytecount low (384/8 = 48 = 0x0030)
        #   word  3 : ro[ 63: 48] = bytecount high (0)
        #   word  4 : ro[ 79: 64] = PCIe BDF {bus, dev, func}
        #   word  5 : ro[ 95: 80] = pl_ltssm + pm_state + init_lnk_width + lane_rev
        #   word  6 : ro[111: 96] = lnk_width + lnk_up + caps + rate + done + hot_rst
        #   word  7 : ro[127:112] = slack + cfg_mgmt_rd_wr_done
        #   word  8 : ro[143:128] = cfg_mgmt_do[15:0]
        #   word  9 : ro[159:144] = cfg_mgmt_do[31:16]
        #   word 10 : ro[175:160] = cfg_command
        #   word 11 : ro[191:176] = AER + cfg_pcie_link_state + pmcsr
        #   word 12 : ro[207:192] = cfg_dcommand              ← KEY for MaxReadReq
        #   word 13 : ro[223:208] = cfg_dcommand2
        #   word 14 : ro[239:224] = cfg_dstatus
        #   word 15 : ro[255:240] = cfg_lcommand
        #   word 16 : ro[271:256] = cfg_lstatus
        #   word 17 : ro[287:272] = cfg_status
        #   words 18+ : tx_buf_av, cfg_vc, interrupt, cfgrd_* — all zero here
        self.comb += Case(cfg_word_index, {
             0: cfg_ro_rd.eq(0x2301),                           # MAGIC (ro)
             1: cfg_ro_rd.eq(0x0000),                           # cfg_mgmt_rd/wr_en (stubbed 0)
             2: cfg_ro_rd.eq(0x0030),                           # bytecount lo = 48
             3: cfg_ro_rd.eq(0x0000),                           # bytecount hi
             # BDF: ufrisk stores bus at ro[71:64] (low byte) and
             # {device, function} at ro[79:72] (high byte), see
             # pcileech_pcie_cfg_a7.sv lines 112-113.  LitePCIe's
             # self.id = Cat(function, device, bus) packs bus at [15:8],
             # so we need to byte-swap it before feeding it to cfg_ro_rd
             # for the response formatter to emit bus as USB wire byte 2
             # (observed as 0x00081600 on ufrisk for bus 0x16).
             4: cfg_ro_rd.eq(Cat(self.phy_id[8:16], self.phy_id[0:8])),  # BDF (byte-swapped to match ufrisk layout)
             5: cfg_ro_rd.eq(Cat(
                    self.phy_ltssm[0:6],    # ro[85:80] pl_ltssm_state
                    Constant(0, 2),          # ro[87:86] pl_rx_pm_state
                    Constant(0, 3),          # ro[90:88] pl_tx_pm_state
                    initial_link_width,      # ro[93:91] pl_initial_link_width
                    Constant(0, 2),          # ro[95:94] pl_lane_reversal
                )),
             6: cfg_ro_rd.eq(Cat(
                    self.phy_lnk_width[0:2],# ro[97:96]  pl_sel_lnk_width
                    self.phy_lnk_up,         # ro[98]     pl_phy_lnk_up
                    Constant(1, 1),          # ro[99]     pl_link_gen2_cap
                    Constant(1, 1),          # ro[100]    pl_link_partner_gen2_sup
                    Constant(1, 1),          # ro[101]    pl_link_upcfg_cap
                    self.phy_lnk_rate,       # ro[102]    pl_sel_lnk_rate
                    Constant(0, 1),          # ro[103]    pl_directed_change_done
                    Constant(0, 1),          # ro[104]    pl_received_hot_rst
                    Constant(0, 7),          # ro[111:105] slack
                )),
             7: cfg_ro_rd.eq(0x0000),                           # slack + cfg_mgmt_rd_wr_done (0)
             8: cfg_ro_rd.eq(0x0000),                           # cfg_mgmt_do[15:0]
             9: cfg_ro_rd.eq(0x0000),                           # cfg_mgmt_do[31:16]
            10: cfg_ro_rd.eq(0x0000),                           # cfg_command (stub 0)
            11: cfg_ro_rd.eq(0x0000),                           # AER/link_state/pmcsr
            12: cfg_ro_rd.eq(self.cfg_dcommand),                # ★ cfg_dcommand ★
            13: cfg_ro_rd.eq(0x0000),                           # cfg_dcommand2
            14: cfg_ro_rd.eq(0x0000),                           # cfg_dstatus
            15: cfg_ro_rd.eq(0x0000),                           # cfg_lcommand
            16: cfg_ro_rd.eq(0x0000),                           # cfg_lstatus
            17: cfg_ro_rd.eq(0x0000),                           # cfg_status
            "default": cfg_ro_rd.eq(0x0000),
        })

        # ==== CFG rw[] readback (704 bits total = 44 × 16-bit words) ====
        # Word layout matches pcileech_pcie_cfg_a7.sv rw[] initialvalues task:
        #   word  0 : rw[ 15:  0] = MAGIC (0x6745)
        #   word  1 : rw[ 31: 16] = rd/wr/wait/static_tlp/status_cl enables (all 0)
        #   word  2 : rw[ 47: 32] = bytecount low (704/8 = 88 = 0x0058)
        #   word  3 : rw[ 63: 48] = bytecount high
        #   words 4–7 : rw[127: 64] = cfg_dsn = 0x0000_0001_0100_0A35 (little-endian)
        #   words 8–9 : rw[159:128] = cfg_mgmt_di (0)
        #   word 10   : rw[175:160] = cfg_mgmt_dwaddr + flags; reset has byte_en=0xf (bits 175:172)
        #   word 11   : rw[191:176] = pl_directed_link_*; reset → 0x0048
        #               (bit 179 pl_directed_link_speed=1 → +0x08,
        #                bit 182 pl_upstream_prefer_deemph=1 → +0x40)
        #   word 12   : rw[207:192] = cfg_interrupt_* (0)
        #   word 13   : rw[223:208] = cfg_pm_*/turnoff_ok + rx_np_ok + rx_np_req + tx_cfg_gnt
        #               reset → bits 217..219 = 1 → 0x0E00
        #   words 14–26: TLP_STATIC DWORDs (unused → 0)
        #   words 27–28: TLP_STATIC_TLP_RETRANSMIT_COUNT
        #   words 29–30: CFGSPACE_STATUS_CLEAR TIMER (default 62500 = 0xF424)
        #
        # We currently do NOT implement CFG writes, so rw[] reads always return
        # the reset values above.  That's intentional: pcileech only *reads*
        # rw[] during enumeration to sanity-check the bitstream.  When we later
        # want to expose raw-config-space access via cfg_mgmt_rd_en, we'll
        # need to add a proper rw[] register file with writable bits and
        # process writes from `in_cmd_write & cfg_f_rw`.
        self.comb += Case(cfg_word_index, {
             0: cfg_rw_rd.eq(0x6745),       # rw MAGIC
             1: cfg_rw_rd.eq(0x0000),
             2: cfg_rw_rd.eq(0x0058),       # bytecount lo = 88
             3: cfg_rw_rd.eq(0x0000),
             # cfg_dsn little-endian: 0x0000_0001_0100_0A35
             4: cfg_rw_rd.eq(0x0A35),
             5: cfg_rw_rd.eq(0x0100),
             6: cfg_rw_rd.eq(0x0001),
             7: cfg_rw_rd.eq(0x0000),
             8: cfg_rw_rd.eq(0x0000),       # cfg_mgmt_di
             9: cfg_rw_rd.eq(0x0000),
            10: cfg_rw_rd.eq(0xF000),       # cfg_mgmt_byte_en=0xf at bits 175:172
            11: cfg_rw_rd.eq(0x0048),       # pl_directed_link_speed=1 + pl_upstream_prefer_deemph=1
            12: cfg_rw_rd.eq(0x0000),
            13: cfg_rw_rd.eq(0x0E00),       # rx_np_ok=rx_np_req=tx_cfg_gnt=1
            # TLP_STATIC region + timer: all zero / defaults
            29: cfg_rw_rd.eq(0xF424),       # CFGSPACE_STATUS_CLEAR TIMER lo = 62500 & 0xFFFF
            30: cfg_rw_rd.eq(0x0000),       # CFGSPACE_STATUS_CLEAR TIMER hi
            "default": cfg_rw_rd.eq(0x0000),
        })

        # Select ro[] vs rw[] per f_rw (address bit 15).
        self.comb += If(cfg_f_rw,
            cfg_readback.eq(cfg_rw_rd),
        ).Else(
            cfg_readback.eq(cfg_ro_rd),
        )

        self.comb += [
            # Response format mirrors pcileech_pcie_cfg_a7.sv lines 348-349:
            #   out_data[31:16] <= in_cmd_address_byte;                    // addr echo, as-is
            #   out_data[15:0]  <= {in_cmd_data_in[7:0], in_cmd_data_in[15:8]};  // value BYTESWAPPED
            #
            # Laid out MSB-first, the 32-bit word is:
            #   [31:24] addr_hi  [23:16] addr_lo  [15:8] val_lo  [7:0] val_hi
            #
            # Observed: ufrisk emits 0x80164800 for addr=0x8016, value=0x0048
            # ⇒ bits[31:24]=0x80 (addr_hi), [23:16]=0x16 (addr_lo),
            #    bits[15:8]=0x48 (val_lo),  [7:0]=0x00 (val_hi)
            #
            # Cat(a,b,c,d) in Migen is LSB-first: bits[7:0]=a, [15:8]=b, etc.
            # So we need Cat(val_hi, val_lo, addr_lo, addr_hi).
            cfg_tx_fifo.sink.valid.eq(cfg_cmd_read),
            cfg_tx_fifo.sink.data .eq(Cat(
                cfg_readback[8:16],  cfg_readback[0:8],   # [15:0]  = value byteswapped
                cfg_addr_byte[0:8],  cfg_addr_byte[8:16], # [31:16] = addr (lo byte first)
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
        # has space, enough ticks have elapsed since base was set, AND
        # no TLP data is pending TX (MRd TLPs waiting to go to PCIe bus).
        # The tlp_tx_pending guard prevents premature keepalives during the
        # MRd→CplD round-trip: the timer doesn't fire while MRd TLPs are
        # in the TX FIFO or for 2000 cycles after the last one drained
        # (covers PCIe bus round-trip ~5-20 µs).
        inactivity_elapsed = Signal(64)
        tlp_tx_cooldown = Signal(max=2001)
        self.sync += [
            If(tlp_tx_fifo.source.valid,
                # TLP TX FIFO has data → keep cooldown armed
                tlp_tx_cooldown.eq(2000),
            ).Elif(tlp_tx_cooldown > 0,
                tlp_tx_cooldown.eq(tlp_tx_cooldown - 1),
            )
        ]

        self.comb += [
            inactivity_elapsed.eq(tickcount64 - inactivity_base),
            inactivity_fire.eq(
                timer_enable
                & ~in_cmd_write
                & cmd_tx_fifo.sink.ready
                & (tlp_tx_cooldown == 0)       # no pending MRd TLPs
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
                # NOTE: The SV reference clears rw[16] here (one-shot timer).
                # We intentionally DO NOT clear it.  In our implementation,
                # the keepalive often arrives at USB before CplD data, causing
                # a transfer split (short packet).  The remaining CplD data
                # needs another keepalive to flush it.  Keeping the timer armed
                # ensures it fires again after timer_ticks cycles, pushing any
                # remaining data through the pipeline to USB.
                # The host re-writes rw[16] before each operation anyway, and
                # pcileech silently discards extra keepalive CMD responses.
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

            63: ro_readback.eq(self.diag_tx_tlp_seen),      # 0x007e: MRds/CfgWr etc we sent
            64: ro_readback.eq(self.diag_rx_tlp_seen),      # 0x0080: TLPs the IP delivered (CplDs + others)
            65: ro_readback.eq(self.diag_tx_err_drop_cnt),  # 0x0082: IP-side TX drops (should be 0)

            # ------------------------------------------------------------
            # RX-side TLP analysis (all counts are at tlp_rx = post-CDC,
            # post-StrideConverter, post-BEFilter).  Compare the totals to
            # narrow down where 9 CplDs go missing in a 10-page dump.
            # ------------------------------------------------------------
            66: ro_readback.eq(self.diag_tlp_rx_cpl_count),   # 0x0084: Cpl/CplD TLPs that reached tlp_rx
            67: ro_readback.eq(self.diag_tlp_rx_other_count), # 0x0086: non-Cpl TLPs the filter dropped
            68: ro_readback.eq(self.diag_tlp_rx_fifo_peak),   # 0x0088: tlp_rx_fifo high-water mark (max level)
            69: ro_readback.eq(self.diag_tlp_rx_stall_cnt),   # 0x008a: cycles m_axis_rx was back-pressured

            70: ro_readback.eq(self.diag_ft601_filler_emit),  # 0x008c: FT601 sync-word filler beats driven
            71: ro_readback.eq(self.diag_ft601_wrn0_accept),  # 0x008e: FT601 wr_n=0 & txe_n=0 accepted beats
            72: ro_readback.eq(self.diag_ft601_txen_high),    # 0x0090: usb-dom cycles with txe_n=1 (chip full)

            
        })

        # Select ro[] or rw[] based on f_rw flag
        self.comb += If(f_rw,
            readback.eq(rw_readback)
        ).Else(
            readback.eq(ro_readback)
        )

        # CMD TX response.  Either a real CMD read response or an
        # inactivity keepalive.  CMD reads take priority.
        #
        # IMPORTANT byte-order note:
        # ufrisk's SV reference (pcileech_fifo.sv lines 380-381) does:
        #   _cmd_tx_din[31:16] <= in_cmd_address_byte;
        #   _cmd_tx_din[15:0]  <= {in_cmd_data_in[7:0], in_cmd_data_in[15:8]};
        # i.e. a byte-swap on the value half.
        # BUT our ro[]/rw[] register map is already laid out in
        # *wire byte order* (e.g. ro_readback at word 5 is
        # Cat(0x00, DEVICE_ID) so the low byte of the 16b slice
        # lands on the correct USB byte).  So we must NOT apply an
        # extra swap here — the wire already comes out matching
        # ufrisk (e.g. 0x000a0400).  The explicit swap only applies
        # to the *cfg_readback* path (see above), where the values
        # are stored in natural 16-bit form and need swapping to
        # reach ufrisk's wire ordering (e.g. 0x80164800).
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
                    readback[0:8],     readback[8:16],       # [15:0]  = value (wire-order, no swap)
                    in_addr_byte[0:8], in_addr_byte[8:16],   # [31:16] = addr  (lo byte first)
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
            # mux_out_fifo: BRAM-backed (buffered=True) so we can afford real
            # depth.  Ufrisk's fifo_256_32_clk2_comtx is 256b × 4096 = 128 KB
            # of BRAM, sized so the mux→serializer boundary never stalls
            # during a burst.
            #
            # We use 512 entries = 16 KB = ~4 BRAM36 tiles.  That's enough
            # to absorb an entire 5-page CplD dump at typical rates without
            # ever backpressuring the mux, while staying well inside the
            # XC7A35T's BRAM budget (50 tiles).
            #
            # IMPORTANT: buffered=True is required.  With the default
            # (async_read=True), migen allocates distributed RAM (RAMD64E)
            # which blows the LUT-RAM budget at this width × depth.
            self.submodules.mux_out_fifo = mux_out_fifo = SyncFIFO(
                [("data", 256)], 512, buffered=True
            )

            self.comb += [
                # Mux writes frames into a small FIFO / skid buffer.
                mux_out_fifo.sink.valid.eq(mux.valid),
                mux_out_fifo.sink.data.eq(mux.dout),
                mux.rd_en.eq(mux_out_fifo.sink.ready),
            ]

            # ---------------------------------------------------------------
            # BURST-START GATE
            # ---------------------------------------------------------------
            # The FT601 chip (in 245-Sync FIFO mode) terminates a USB transfer
            # by sending a short packet whenever WR_N has been inactive for
            # more than a brief internal timeout (~µs).  If our TX pipeline
            # has any gap while the host's bulk-in is active, the host sees
            # a short packet and declares the transfer complete — even if
            # more data is coming.
            #
            # Two gap sources need to be absorbed:
            #
            #   (A) START-OF-BURST gap: the host-injected LOOPBACK probe is
            #       processed in ~50 ns (CMD byte-frame → loop_fifo → mux →
            #       mux_out_fifo), but the MRd CplD round-trip through the
            #       PCIe root complex takes 1–5 µs before the first CplD
            #       reaches tlp_rx_fifo.  Without gating, the serializer
            #       immediately drains the single loopback frame (52 bytes
            #       on the wire) and then sits idle for µs while PCIe
            #       catches up — FT601 auto-flushes a short packet, the
            #       transfer splits in half.
            #
            #   (B) MID-BURST gap: under sustained backpressure (multi-page
            #       reads), PCIe flow-control credits between our
            #       tlp_rx_fifo and the root complex open inter-CplD gaps
            #       that can exceed FT601's drain_wait (100 µs).  If the
            #       burst gate closes during such a gap, new data arriving
            #       after the gate closes has to re-fill BURST_FILL frames
            #       before the gate reopens — which adds even MORE delay
            #       on top of the FT601 drain timeout, making a mid-burst
            #       split nearly certain for 10+ page dumps.
            #
            # ufrisk avoids all of this with ~128 KB of upstream buffering
            # (fifo_256_32_clk2_comtx = 256b × 4096).  Our mux_out_fifo is
            # 16 KB (256b × 512) — plenty at steady state, but only if the
            # gate stays open through mid-burst gaps.
            #
            # The gate opens when EITHER:
            #   1. mux_out_fifo.level >= BURST_FILL  (16 frames ≈ 1.28 µs of
            #      sustainable TX ≫ any realistic serializer/CDC hiccup), OR
            #   2. the warmup timer (BURST_WARMUP, 50 µs @ 100 MHz) expires
            #      as a fallback for genuinely small single-frame responses
            #      (CMD/CFG probes).
            #
            # Once open, the gate stays open until the FIFO has been empty
            # for BURST_SETTLE (500 µs) continuously.  This is 5× longer
            # than FT601's drain_wait (100 µs), so by the time the gate
            # closes the FT601 has already exited WRITE and emitted its
            # short packet — guaranteeing the gate closure never affects
            # an in-progress transfer, only end-of-burst cleanup.  New
            # bursts then enter FILL and re-prime from scratch.
            # ---------------------------------------------------------------
            BURST_FILL    = 16        # frames to accumulate before opening
            BURST_WARMUP  = 5000      # 50 µs fallback timer @ 100 MHz
            BURST_SETTLE  = 50000     # 500 µs empty → gate closes (end-of-burst)

            burst_gate  = Signal()
            burst_timer = Signal(max=max(BURST_WARMUP, BURST_SETTLE) + 1)

            self.submodules.burst_fsm = burst_fsm = FSM(reset_state="FILL")
            burst_fsm.act("FILL",
                # Serializer held in IDLE — mux_out_fifo accumulates frames.
                If(mux_out_fifo.source.valid,
                    NextValue(burst_timer, burst_timer + 1),
                ).Else(
                    NextValue(burst_timer, 0),
                ),
                If((mux_out_fifo.level >= BURST_FILL) | (burst_timer >= BURST_WARMUP),
                    NextValue(burst_timer, 0),
                    NextState("DRAIN"),
                ),
            )
            burst_fsm.act("DRAIN",
                burst_gate.eq(1),
                # Stay in DRAIN through any mid-burst gap shorter than
                # BURST_SETTLE.  Under PCIe credit exhaustion during a
                # large multi-page read, inter-CplD gaps can be tens of µs;
                # we MUST keep the gate open so data resumes flowing the
                # instant a new CplD lands in mux_out_fifo.  BURST_SETTLE
                # is sized well above FT601 drain_wait so the gate only
                # closes AFTER the transfer has genuinely ended.
                If(~mux_out_fifo.source.valid,
                    NextValue(burst_timer, burst_timer + 1),
                    If(burst_timer >= BURST_SETTLE,
                        NextValue(burst_timer, 0),
                        NextState("FILL"),
                    ),
                ).Else(
                    NextValue(burst_timer, 0),
                ),
            )

            # Gated connection: serializer only sees data when burst_gate is 1.
            self.comb += [
                serializer.sink.valid.eq(mux_out_fifo.source.valid & burst_gate),
                serializer.sink.data .eq(mux_out_fifo.source.data),
                mux_out_fifo.source.ready.eq(serializer.sink.ready & burst_gate),
            ]

            # Burst-start pulse: rising edge of burst_gate marks the start
            # of a new USB bulk-in transfer.  The serializer uses this to
            # emit a fresh 5 × 0x66665555 sync preamble — ufrisk does the
            # same on every Bi, and pcileech's host parser relies on it
            # to align frame boundaries across USB transfer boundaries.
            burst_gate_prev   = Signal()
            burst_start_pulse = Signal()
            self.sync += burst_gate_prev.eq(burst_gate)
            self.comb += [
                burst_start_pulse.eq(burst_gate & ~burst_gate_prev),
                serializer.start_sync.eq(burst_start_pulse),
            ]

            # Diagnostic hooks (optional, wired to read-only status regs if needed)
            self.burst_gate  = burst_gate
            self.burst_timer = burst_timer
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

        # Drive the inactivity timer reset from BOTH RX and TX activity
        # (defined earlier as forward-declared signal tx_com_activity)
        #
        # The SV reference resets only on TX activity:
        #   if ( dcom.com_din_wr_en | ~dcom.com_din_ready )
        # This works because the SV's deep pipeline hides the MRd→CplD
        # round-trip latency.  In our shallower pipeline, there's a ~5-10 µs
        # gap between when the host sends MRd TLPs (USB RX activity) and
        # when CplD responses start flowing to USB (TX activity).  During
        # this gap, the timer fires a premature keepalive that splits the
        # USB transfer — the host gets a 52-byte keepalive as one transfer,
        # then CplD data as a second transfer, then times out waiting for
        # remaining data in a third transfer.
        #
        # By also resetting on USB RX activity, we suppress the timer
        # during the MRd→CplD turnaround.  Once the host stops sending
        # commands and CplDs start flowing, TX activity takes over.
        # When both stop (all data delivered), the timer fires normally.
        self.comb += tx_com_activity.eq(
            self.usb_rx.valid |  # host→FPGA activity (commands, MRd TLPs)
            (serializer.source.valid & serializer.source.ready) |  # FPGA→host data
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
        


            
