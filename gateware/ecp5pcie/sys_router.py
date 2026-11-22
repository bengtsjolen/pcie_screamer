from functools import reduce
from operator import or_

from migen import *

from litex.soc.interconnect import stream
from litex.soc.interconnect.packet import Dispatcher, Arbiter

from litepcie.common import phy_layout


def bswap32(x):
    return Cat(x[24:32], x[16:24], x[8:16], x[0:8])


# Fmt/Type of non-posted requests the endpoint does not support and LitePCIe does not claim:
# MRdLk32/64, IORd, IOWr, CfgRd1, CfgWr1, MRd64 (no 64-bit BAR). These get an Unsupported Request.
UR_REQUESTS = (0x20, 0x01, 0x21, 0x02, 0x42, 0x05, 0x45)

# Fmt/Type forwarded to the LitePCIe transaction layer: MRd32, MWr32, Cpl, CplD.
LITEPCIE_TLPS = (0x00, 0x40, 0x0a, 0x4a)


class TLPSysRouter(Module):
    """Routes the core's TLP stream (64-bit, `sys`) around a LitePCIe transaction layer.

    RX (`sink`, from the adapter): CfgRd0/CfgWr0 are answered here from `config_space`;
    MRd32/MWr32/Cpl/CplD go to `phy_source` (the LitePCIe depacketizer); other non-posted
    requests get an Unsupported Request completion; everything else is dropped. TX (`source`,
    to the adapter) merges the config completions, UR completions and `phy_sink` (the LitePCIe
    packetizer) at packet boundaries.

    Each lane carries a dword with wire byte 0 in bits [31:24] (the adapter's convention), so
    headers are built directly and read data is `bswap32`'d.
    """
    def __init__(self, config_space, data_width=64):
        self.sink       = stream.Endpoint(phy_layout(data_width))  # RX from adapter
        self.source     = stream.Endpoint(phy_layout(data_width))  # TX to adapter
        self.phy_source = stream.Endpoint(phy_layout(data_width))  # to LitePCIe depacketizer
        self.phy_sink   = stream.Endpoint(phy_layout(data_width))  # from LitePCIe packetizer

        self.cfg_rd  = Signal()
        self.cfg_wr  = Signal()
        self.ur      = Signal()
        self.cpl     = Signal()
        self.dropped = Signal()
        self.id      = Signal(16)  # bus/device/function from the last CfgWr0

        # RX classification ------------------------------------------------------------------------
        fmt_type = Signal(8)
        self.comb += fmt_type.eq(self.sink.dat[24:32])  # DW0 wire byte 0, valid on the first beat
        is_cfg0     = Signal()
        is_litepcie = Signal()
        is_ur       = Signal()
        self.comb += [
            is_cfg0.eq((fmt_type & 0xbf) == 0x04),
            is_litepcie.eq(reduce(or_, [fmt_type == t for t in LITEPCIE_TLPS])),
            is_ur.eq(reduce(or_, [fmt_type == t for t in UR_REQUESTS])),
        ]

        cfg_in  = stream.Endpoint(phy_layout(data_width))
        ur_in   = stream.Endpoint(phy_layout(data_width))
        discard = stream.Endpoint(phy_layout(data_width))
        self.comb += discard.ready.eq(1)
        self.comb += If(discard.valid & discard.first, self.dropped.eq(1))

        # Register valid/payload and ready on both LitePCIe-facing ports. Without this the
        # depacketizer/packetizer handshakes chain combinationally through the router into the
        # adapter's FIFOs, which does not close 100 MHz on the ECP5.
        self.submodules.rx_buf = rx_buf = stream.Buffer(
            phy_layout(data_width), pipe_valid=True, pipe_ready=True)
        self.submodules.tx_buf = tx_buf = stream.Buffer(
            phy_layout(data_width), pipe_valid=True, pipe_ready=True)
        self.comb += [
            rx_buf.source.connect(self.phy_source),
            self.phy_sink.connect(tx_buf.sink),
        ]

        self.submodules.dispatcher = dispatcher = Dispatcher(
            self.sink, [cfg_in, rx_buf.sink, ur_in, discard])
        sel = Signal(2)
        self.comb += [
            If(is_cfg0,         sel.eq(0)
            ).Elif(is_litepcie, sel.eq(1)
            ).Elif(is_ur,       sel.eq(2)
            ).Else(             sel.eq(3)),
            dispatcher.sel.eq(sel),
        ]

        # Config completer -------------------------------------------------------------------------
        cfg_cpl = stream.Endpoint(phy_layout(data_width))
        is_write  = Signal()
        requester = Signal(16)
        tag       = Signal(8)
        tc        = Signal(3)
        attr      = Signal(2)
        wr_be     = Signal(4)
        reg       = Signal(10)
        wr_data   = Signal(32)
        completer = Signal(16)
        self.comb += config_space.reg.eq(reg)

        cfg_fsm = FSM(reset_state="RCV0")
        self.submodules.cfg_fsm = cfg_fsm
        cfg_fsm.act("RCV0",
            cfg_in.ready.eq(1),
            If(cfg_in.valid,
                NextValue(is_write,  cfg_in.dat[30]),  # fmt/type bit 6 (CfgWr0 = 0x44)
                NextValue(tc,        cfg_in.dat[20:23]),
                NextValue(attr,      cfg_in.dat[12:14]),
                NextValue(requester, cfg_in.dat[48:64]),
                NextValue(tag,       cfg_in.dat[40:48]),
                NextValue(wr_be,     cfg_in.dat[32:36]),
                NextState("RCV1"),
            ),
        )
        cfg_fsm.act("RCV1",
            cfg_in.ready.eq(1),
            If(cfg_in.valid,
                NextValue(reg,       cfg_in.dat[2:12]),
                NextValue(completer, cfg_in.dat[16:32]),
                NextValue(wr_data,   cfg_in.dat[32:64]),
                NextState("APPLY"),
            ),
        )
        cfg_fsm.act("APPLY",
            config_space.we.eq(is_write),
            config_space.wr_be.eq(wr_be),
            config_space.wr_data.eq(bswap32(wr_data)),  # lane is wire order; config file is native
            If(is_write,
                self.cfg_wr.eq(1),
                NextValue(self.id, completer),
            ).Else(
                self.cfg_rd.eq(1),
            ),
            NextState("WAIT"),     # rd_data is valid one cycle after reg settles
        )
        cfg_fsm.act("WAIT", NextState("SEND0"))

        has_data = Signal()
        status   = Signal(3)
        self.comb += has_data.eq(~is_write)
        hdr0 = Cat(has_data, C(0, 11), attr, C(0, 6), tc, C(0, 1),
                   Mux(has_data, C(0x4a, 8), C(0x0a, 8)))
        hdr1 = Cat(C(4, 12), C(0, 1), status, completer)
        hdr2 = Cat(C(0, 7), C(0, 1), tag, requester)
        cfg_fsm.act("SEND0",
            cfg_cpl.valid.eq(1),
            cfg_cpl.first.eq(1),
            cfg_cpl.dat.eq(Cat(hdr0, hdr1)),
            cfg_cpl.be.eq(0xFF),
            If(cfg_cpl.ready, NextState("SEND1")),
        )
        cfg_fsm.act("SEND1",
            cfg_cpl.valid.eq(1),
            cfg_cpl.last.eq(1),
            cfg_cpl.dat.eq(Cat(hdr2, bswap32(config_space.rd_data))),
            cfg_cpl.be.eq(Mux(has_data, 0xFF, 0x0F)),
            If(cfg_cpl.ready,
                self.cpl.eq(1),
                NextState("RCV0"),
            ),
        )

        # Unsupported Request completer ------------------------------------------------------------
        ur_cpl = stream.Endpoint(phy_layout(data_width))
        ur_req  = Signal(16)
        ur_tag  = Signal(8)
        ur_tc   = Signal(3)
        ur_attr = Signal(2)
        ur_fsm = FSM(reset_state="RCV")
        self.submodules.ur_fsm = ur_fsm
        ur_fsm.act("RCV",
            ur_in.ready.eq(1),
            If(ur_in.valid & ur_in.first,
                NextValue(ur_tc,   ur_in.dat[20:23]),
                NextValue(ur_attr, ur_in.dat[12:14]),
                NextValue(ur_req,  ur_in.dat[48:64]),
                NextValue(ur_tag,  ur_in.dat[40:48]),
            ),
            If(ur_in.valid & ur_in.last, NextState("SEND0")),
        )
        ur_hdr0 = Cat(C(0, 12), ur_attr, C(0, 6), ur_tc, C(0, 1), C(0x0a, 8))
        ur_hdr1 = Cat(C(4, 12), C(0, 1), C(1, 3), self.id)   # status = Unsupported Request
        ur_hdr2 = Cat(C(0, 7), C(0, 1), ur_tag, ur_req)
        ur_fsm.act("SEND0",
            ur_cpl.valid.eq(1),
            ur_cpl.first.eq(1),
            ur_cpl.dat.eq(Cat(ur_hdr0, ur_hdr1)),
            ur_cpl.be.eq(0xFF),
            If(ur_cpl.ready, NextState("SEND1")),
        )
        ur_fsm.act("SEND1",
            ur_cpl.valid.eq(1),
            ur_cpl.last.eq(1),
            ur_cpl.dat.eq(Cat(ur_hdr2, C(0, 32))),
            ur_cpl.be.eq(0x0F),
            If(ur_cpl.ready,
                self.ur.eq(1),
                NextState("RCV"),
            ),
        )

        # TX merge ---------------------------------------------------------------------------------
        self.submodules.arbiter = Arbiter([cfg_cpl, ur_cpl, tx_buf.source], self.source)
