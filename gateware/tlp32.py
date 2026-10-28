# 32-bit TLP stream to 64-bit PCIe AXI-Stream packer.

from migen import *

from litex.soc.interconnect import stream

from litepcie.common import phy_layout


class TLP32To64Packer(Module):
    def __init__(self, swap_words=False):
        self.sink   = sink   = stream.Endpoint(phy_layout(32))
        self.source = source = stream.Endpoint(phy_layout(64))

        # Stored first DWORD of a pair.
        first_dat = Signal(32)
        first_be  = Signal(4)

        self.submodules.fsm = fsm = FSM(reset_state="IDLE")

        # Default: nothing happening.
        self.comb += [
            source.valid.eq(0),
            source.last .eq(0),
            source.dat  .eq(0),
            source.be   .eq(0),
            sink.ready  .eq(0),
        ]

        # ---- IDLE: waiting for first DWORD of a pair (or a lone last) ----
        fsm.act("IDLE",
            If(sink.valid & sink.last,
                # Single-DWORD packet: emit immediately as partial 64-bit beat.
                # CRITICAL: only consume when output can accept.
                source.valid.eq(1),
                source.last .eq(1),
                If(swap_words,
                    source.dat.eq(Cat(Constant(0, 32), sink.dat)),
                    source.be .eq(Cat(Constant(0x0, 4), sink.be)),
                ).Else(
                    source.dat.eq(Cat(sink.dat, Constant(0, 32))),
                    source.be .eq(Cat(sink.be,  Constant(0x0, 4))),
                ),
                sink.ready.eq(source.ready),   # ← FIX: was unconditional 1
            ).Elif(sink.valid,
                # First of a pair: latch and move on.  No output yet,
                # so we can always accept.
                sink.ready.eq(1),
                NextValue(first_dat, sink.dat),
                NextValue(first_be,  sink.be),
                NextState("HAVE_FIRST"),
            ).Else(
                # No data — stay idle, ready to accept.
                sink.ready.eq(1),
            )
        )

        # ---- HAVE_FIRST: waiting for second DWORD to form a 64-bit pair ----
        fsm.act("HAVE_FIRST",
            If(sink.valid,
                # Pair ready: emit both DWORDs as one 64-bit beat.
                source.valid.eq(1),
                source.last .eq(sink.last),
                If(swap_words,
                    source.dat.eq(Cat(sink.dat, first_dat)),
                    source.be .eq(Cat(sink.be,  first_be)),
                ).Else(
                    source.dat.eq(Cat(first_dat, sink.dat)),
                    source.be .eq(Cat(first_be,  sink.be)),
                ),
                # CRITICAL: only consume when output can accept.
                sink.ready.eq(source.ready),   # ← FIX: was unconditional 1
                If(source.ready,
                    NextState("IDLE"),
                )
            )
            # If no data: sink.ready stays 0 (default), we wait.
            # (Could set sink.ready=1 here too — doesn't matter since
            #  sink.valid=0 means no handshake either way.)
        )
