# 64-bit PCIe AXI-Stream to 128-bit TLP packing/filtering.
#
# Mirrors the 64->128 conversion and Cpl/CplD filtering used by
# pcileech_pcie_a7.sv / pcileech_pcie_tlp_a7.sv in the ufrisk reference.

from migen import *

from litex.soc.interconnect import stream

from litepcie.common import phy_layout


TLP128_LAYOUT = [("dat", 128), ("keep", 4)]


class TLP64To128Packer(Module):
    """Pack two 64-bit AXI-Stream beats into one 128-bit TLP beat."""

    def __init__(self):
        self.sink   = sink   = stream.Endpoint(phy_layout(64))
        self.source = source = stream.Endpoint(TLP128_LAYOUT)

        buf    = Signal(128)
        keep   = Signal(4)
        last   = Signal()
        first  = Signal(reset=1)
        length = Signal(3)

        dw1_valid = Signal()
        len_next  = Signal(3)
        self.comb += [
            dw1_valid.eq(sink.be[4:8] != 0),
            len_next.eq(length + 1 + dw1_valid),
        ]

        self.submodules.fsm = fsm = FSM(reset_state="ASSEMBLE")

        self.comb += [
            sink.ready.eq(0),
            source.valid.eq(0),
            source.dat.eq(buf),
            source.keep.eq(keep),
            source.first.eq(first),
            source.last.eq(last),
        ]

        fsm.act("ASSEMBLE",
            sink.ready.eq(1),
            If(sink.valid,
                Case(length, {
                    0: [
                        NextValue(buf[0:32],  sink.dat[0:32]),
                        NextValue(buf[32:64], sink.dat[32:64]),
                        NextValue(keep, Cat(1, dw1_valid, 0, 0)),
                    ],
                    1: [
                        NextValue(buf[32:64],  sink.dat[0:32]),
                        NextValue(buf[64:96],  sink.dat[32:64]),
                        NextValue(keep, Cat(keep[0], 1, dw1_valid, 0)),
                    ],
                    2: [
                        NextValue(buf[64:96],  sink.dat[0:32]),
                        NextValue(buf[96:128], sink.dat[32:64]),
                        NextValue(keep, Cat(keep[0:2], 1, dw1_valid)),
                    ],
                    "default": [
                        # The upper DWORD cannot normally be valid here: a
                        # 1-DWORD beat only occurs at the end of a TLP.
                        NextValue(buf[96:128], sink.dat[0:32]),
                        NextValue(keep, Cat(keep[0:3], 1)),
                    ],
                }),
                NextValue(last, sink.last),
                NextValue(length, len_next),
                If(sink.last | (len_next == 4),
                    NextState("OUTPUT"),
                ),
            ),
        )

        fsm.act("OUTPUT",
            source.valid.eq(1),
            If(source.ready,
                NextValue(first, last),
                NextValue(length, 0),
                NextState("ASSEMBLE"),
            ),
        )


class TLP128Filter(Module):
    """Pass only Cpl/CplD TLPs unless bypassed."""

    def __init__(self):
        self.sink   = sink   = stream.Endpoint(TLP128_LAYOUT)
        self.source = source = stream.Endpoint(TLP128_LAYOUT)
        self.bypass = Signal()

        pass_reg = Signal()
        is_cpl   = Signal()
        pass_now = Signal()

        self.comb += [
            is_cpl.eq(
                (sink.dat[25:32] == 0b0000101) |   # Cpl
                (sink.dat[25:32] == 0b0100101)     # CplD
            ),
            pass_now.eq(Mux(self.bypass, 1,
                Mux(sink.first, is_cpl, pass_reg))),
            sink.ready.eq(Mux(pass_now, source.ready, 1)),
            source.valid.eq(sink.valid & pass_now),
            source.dat.eq(sink.dat),
            source.keep.eq(sink.keep),
            source.first.eq(sink.first),
            source.last.eq(sink.last),
        ]

        self.sync += [
            If(sink.valid & sink.ready,
                If(sink.first,
                    pass_reg.eq(is_cpl),
                )
            )
        ]
