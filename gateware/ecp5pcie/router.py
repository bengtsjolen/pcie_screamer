from functools import reduce
from operator import or_

from migen import *


def bswap32(x):
    return Cat(x[24:32], x[16:24], x[8:16], x[0:8])


# Fmt/Type bytes of non-posted requests this endpoint doesn't support: MRd32/64, MRdLk32/64,
# IORd, IOWr, CfgRd1, CfgWr1.
UR_REQUESTS = (0x00, 0x20, 0x01, 0x21, 0x02, 0x42, 0x05, 0x45)


class TLPRouter(Module):
    """Answers TLPs from the ECP5-PCIe core's TLP ports in its pcie_rx domain.

    CfgRd0/CfgWr0 go to `config_space` and get a CplD/Cpl, the other non-posted requests in
    `UR_REQUESTS` get an Unsupported Request Cpl, everything else is dropped.

    Core port contract: data[7:0] is wire byte 0, TLPs are delimited by gaps in `valid`. The
    core's RX buffer only samples `rx_ready` before starting a TLP, so `rx_ready` drops at the
    first word and comes back once the TLP is handled. TX waits for `tx_ready` with no TLP in
    the retry buffer (the core only replays the oldest one), then sends the TLP with `valid`
    held; `guard` cycles after a TLP cover the latency before it shows up in `retry_occupancy`.
    """
    def __init__(self, config_space, rx_data, rx_valid, rx_ready, tx_data, tx_valid, tx_ready,
                 retry_occupancy, guard=16):
        self.cfg_rd  = Signal()   # event pulses
        self.cfg_wr  = Signal()
        self.ur      = Signal()
        self.dropped = Signal()
        self.cpl     = Signal()
        self.id      = Signal(16) # bus/device/function from the last CfgWr0

        words = [Signal(32, name=f"rx_word{i}") for i in range(4)]
        count = Signal(3)
        hdr   = [bswap32(w) for w in words]
        fmt_type = hdr[0][24:32]
        is_cfg0  = (fmt_type & 0xbf) == 0x04
        is_write = fmt_type[6]
        is_ur    = reduce(or_, [fmt_type == t for t in UR_REQUESTS])

        requester = Signal(16)
        completer = Signal(16)
        tag       = Signal(8)
        tc        = Signal(3)
        attr      = Signal(2)
        status    = Signal(3)
        has_data  = Signal()

        cpl_hdr = [
            Cat(has_data, C(0, 11), attr, C(0, 6), tc, C(0, 1), Mux(has_data, C(0x4a, 8), C(0x0a, 8))),
            Cat(C(4, 12), C(0, 1), status, completer),
            Cat(C(0, 7), C(0, 1), tag, requester),
        ]
        tx_words = Array([bswap32(h) for h in cpl_hdr] + [config_space.rd_data])
        index    = Signal(2)
        guard_count = Signal(max=guard + 1)

        self.comb += [
            config_space.reg.eq(hdr[2][2:12]),
            config_space.wr_be.eq(hdr[1][0:4]),
            config_space.wr_data.eq(words[3]),
        ]

        fsm = ClockDomainsRenamer("pcie_rx")(FSM(reset_state="IDLE"))
        self.submodules.fsm = fsm

        fsm.act("IDLE",
            rx_ready.eq(rx_valid == 0),
            If(rx_valid != 0,
                NextValue(words[0], rx_data),
                NextValue(count, 1),
                NextState("RECEIVE"),
            )
        )
        fsm.act("RECEIVE",
            If(rx_valid != 0,
                *[If(count == i, NextValue(words[i], rx_data)) for i in range(1, 4)],
                If(count != 7, NextValue(count, count + 1)),
            ).Else(
                NextState("DISPATCH"),
            )
        )
        fsm.act("DISPATCH",
            NextValue(requester, hdr[1][16:32]),
            NextValue(tag,       hdr[1][8:16]),
            NextValue(tc,        hdr[0][20:23]),
            NextValue(attr,      hdr[0][12:14]),
            If(is_cfg0 & (count == Mux(is_write, 4, 3)),
                config_space.we.eq(is_write),
                If(is_write,
                    self.cfg_wr.eq(1),
                    NextValue(self.id, hdr[2][16:32]),
                ).Else(
                    self.cfg_rd.eq(1),
                ),
                NextValue(status,    0),
                NextValue(has_data,  ~is_write),
                NextValue(completer, hdr[2][16:32]),
                NextState("WAIT"),
            ).Elif(is_ur,
                self.ur.eq(1),
                NextValue(status,    1),
                NextValue(has_data,  0),
                NextValue(completer, self.id),
                NextState("WAIT"),
            ).Else(
                self.dropped.eq(1),
                NextState("IDLE"),
            )
        )
        fsm.act("WAIT",
            If(guard_count != 0, NextValue(guard_count, guard_count - 1)),
            If(tx_ready & (retry_occupancy == 0) & (guard_count == 0),
                NextValue(index, 0),
                NextState("SEND"),
            )
        )
        fsm.act("SEND",
            tx_valid.eq(0b1111),
            tx_data.eq(tx_words[index]),
            NextValue(index, index + 1),
            If(index == Mux(has_data, 3, 2),
                self.cpl.eq(1),
                NextValue(guard_count, guard),
                NextState("IDLE"),
            )
        )
