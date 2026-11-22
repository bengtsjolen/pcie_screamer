from migen import *
from migen.genlib.cdc import MultiReg, GrayCounter

from litex.soc.interconnect import stream
from litex.soc.interconnect.packet import PacketFIFO

from litepcie.common import phy_layout


def _gray_to_bin(gray):
    """Combinational Gray -> binary (log-depth prefix XOR)."""
    binary, shift = gray, 1
    while shift < len(gray):
        binary = binary ^ (binary >> shift)
        shift *= 2
    return binary


def bswap32(x):
    """Wire byte order <-> dword value. Wire byte 0 is the LSB of `x` (core port
    convention) and the MSB of the result (LitePCIe header convention: fmt/type in
    bits [31:24])."""
    return Cat(x[24:32], x[16:24], x[8:16], x[0:8])


class TLPStreamAdapter(Module):
    """Bridges the ECP5-PCIe core's bare-TLP ports to a LitePCIe 64-bit phy stream.

    Core side (`pcie_rx` domain, 62.5 MHz Gen1): 32-bit `rx`/`tx` beats, 4 bytes per
    cycle with `data[7:0]` the first wire byte, TLPs delimited by gaps in `valid`. The
    core samples `rx_ready` only before a TLP starts, and TX must hold `valid` for the
    whole TLP with no bubbles (the DLL only replays the single oldest TLP).

    LitePCIe side (`sys` domain): `source` feeds the depacketizer, `sink` comes from the
    packetizer, both `phy_layout(64)`. Each 32-bit dword is `bswap32`'d so wire byte 0
    lands in bits [31:24], where the header decoder reads fmt/type.

    RX packs two `pcie_rx` dwords into one 64-bit `sys` beat; an odd final dword sets
    `be=0x0F`. TX unpacks and, via a store-and-forward `PacketFIFO`, only hands the core
    complete, gap-free TLPs. Both crossings use async FIFOs at least one max TLP deep, so
    the core-side TX never underruns once started (`sys` at 100 MHz outruns `pcie_rx`).
    """
    def __init__(self, rx_data, rx_valid, rx_ready, tx_data, tx_valid, tx_ready,
                 retry_occupancy, max_tlp_dwords=72, depth=256, guard=16):
        assert depth & (depth - 1) == 0 and depth >= 2 * max_tlp_dwords
        self.source = stream.Endpoint(phy_layout(64))   # RX: core -> depacketizer
        self.sink   = stream.Endpoint(phy_layout(64))   # TX: packetizer -> core

        # RX: pcie_rx 32-bit dwords -> sys 64-bit beats --------------------------------------------
        rx_fifo = stream.AsyncFIFO([("dat", 32)], depth, buffered=True)
        rx_fifo = ClockDomainsRenamer({"write": "pcie_rx", "read": "sys"})(rx_fifo)
        self.submodules.rx_fifo = rx_fifo

        # Accept a new TLP only when the FIFO can hold a whole max-size one; the core does not
        # check rx_ready again once a TLP has started. There is no level signal across the CDC,
        # so track occupancy on the write side with a Gray-coded read count synced back from sys.
        cw = depth.bit_length() + 1
        rd_gray_sys = ClockDomainsRenamer("sys")(GrayCounter(cw))
        self.submodules += rd_gray_sys
        self.comb += rd_gray_sys.ce.eq(rx_fifo.source.valid & rx_fifo.source.ready)
        rd_gray = Signal(cw)
        self.specials += MultiReg(rd_gray_sys.q, rd_gray, "pcie_rx")
        wr_count = Signal(cw)
        occupancy = Signal(cw)
        self.comb += occupancy.eq(wr_count - _gray_to_bin(rd_gray))  # lags -> over-estimates
        self.sync.pcie_rx += If(rx_fifo.sink.valid & rx_fifo.sink.ready, wr_count.eq(wr_count + 1))
        self.comb += rx_ready.eq(rx_fifo.sink.ready & (occupancy < depth - max_tlp_dwords))

        # Delay one dword so `last` can be marked on the gap after the final dword.
        prev_valid = Signal()
        prev_dat   = Signal(32)
        self.sync.pcie_rx += [
            prev_valid.eq(rx_valid != 0),
            prev_dat.eq(rx_data),
        ]
        self.comb += [
            rx_fifo.sink.valid.eq(prev_valid),
            rx_fifo.sink.dat.eq(bswap32(prev_dat)),
            rx_fifo.sink.last.eq(rx_valid == 0),  # current cycle is a gap -> prev was last
        ]

        # Pack two dwords into a 64-bit beat.
        lane      = Signal()        # 0: next dword fills the low lane
        hold      = Signal(32)      # registered low lane
        hold_first = Signal()
        tlp_first = Signal(reset=1) # next dequeued dword starts a TLP
        src = self.source
        self.comb += [
            If(lane == 0,
                If(rx_fifo.source.valid & rx_fifo.source.last,
                    # Single trailing dword: emit a low-only beat.
                    src.valid.eq(1),
                    src.dat.eq(rx_fifo.source.dat),
                    src.be.eq(0x0F),
                    src.first.eq(tlp_first),
                    src.last.eq(1),
                    rx_fifo.source.ready.eq(src.ready),
                ).Elif(rx_fifo.source.valid,
                    # Buffer the low dword; pop it now so the high lane reads the next one.
                    rx_fifo.source.ready.eq(1),
                ),
            ).Else(
                src.valid.eq(rx_fifo.source.valid),
                src.dat.eq(Cat(hold, rx_fifo.source.dat)),
                src.be.eq(0xFF),
                src.first.eq(hold_first),
                src.last.eq(rx_fifo.source.last),
                rx_fifo.source.ready.eq(src.ready),
            ),
        ]
        self.sync += [
            If(lane == 0,
                If(rx_fifo.source.valid & rx_fifo.source.last,
                    If(src.ready, tlp_first.eq(1)),
                ).Elif(rx_fifo.source.valid,
                    hold.eq(rx_fifo.source.dat),
                    hold_first.eq(tlp_first),
                    tlp_first.eq(0),
                    lane.eq(1),
                ),
            ).Else(
                If(rx_fifo.source.valid & src.ready,
                    lane.eq(0),
                    tlp_first.eq(rx_fifo.source.last),
                ),
            ),
        ]

        # TX: sys 64-bit beats -> pcie_rx 32-bit dwords --------------------------------------------
        # Store-and-forward so the core only ever sees complete, gap-free TLPs.
        pkt_fifo = PacketFIFO(phy_layout(64), payload_depth=max_tlp_dwords, buffered=True)
        self.submodules.pkt_fifo = pkt_fifo
        self.comb += self.sink.connect(pkt_fifo.sink)

        tx_fifo = stream.AsyncFIFO([("dat", 32)], depth, buffered=True)
        tx_fifo = ClockDomainsRenamer({"write": "sys", "read": "pcie_rx"})(tx_fifo)
        self.submodules.tx_fifo = tx_fifo

        # Register the packet FIFO's block-RAM output before the unpacker (timing).
        self.submodules.pkt_buf = pkt_buf = stream.Buffer(phy_layout(64), pipe_valid=True)
        self.comb += pkt_fifo.source.connect(pkt_buf.sink)

        psrc = pkt_buf.source
        tx_hi = Signal()
        hi_valid = Signal()
        self.comb += hi_valid.eq(psrc.be[4:8] != 0)
        self.comb += [
            If(~tx_hi,
                tx_fifo.sink.valid.eq(psrc.valid),
                tx_fifo.sink.dat.eq(bswap32(psrc.dat[0:32])),
                tx_fifo.sink.last.eq(psrc.last & ~hi_valid),
                If(psrc.valid & tx_fifo.sink.ready & ~hi_valid, psrc.ready.eq(1)),
            ).Else(
                tx_fifo.sink.valid.eq(psrc.valid),
                tx_fifo.sink.dat.eq(bswap32(psrc.dat[32:64])),
                tx_fifo.sink.last.eq(psrc.last),
                If(psrc.valid & tx_fifo.sink.ready, psrc.ready.eq(1)),
            ),
        ]
        self.sync += [
            If(~tx_hi,
                If(psrc.valid & tx_fifo.sink.ready & hi_valid, tx_hi.eq(1)),
            ).Else(
                If(psrc.valid & tx_fifo.sink.ready, tx_hi.eq(0)),
            ),
        ]

        # pcie_rx TX FSM: wait until a TLP start is available and the DLL retry buffer is
        # empty, then stream the whole TLP with `valid` held.
        guard_count = Signal(max=guard + 1)
        fsm = ClockDomainsRenamer("pcie_rx")(FSM(reset_state="IDLE"))
        self.submodules.tx_fsm = fsm
        fsm.act("IDLE",
            If(guard_count != 0, NextValue(guard_count, guard_count - 1)),
            If(tx_fifo.source.valid & tx_ready & (retry_occupancy == 0) & (guard_count == 0),
                NextState("SEND"),
            ),
        )
        fsm.act("SEND",
            tx_valid.eq(Replicate(tx_fifo.source.valid, 4)),
            tx_data.eq(tx_fifo.source.dat),
            tx_fifo.source.ready.eq(1),
            If(tx_fifo.source.valid & tx_fifo.source.last,
                NextValue(guard_count, guard),
                NextState("IDLE"),
            ),
        )
