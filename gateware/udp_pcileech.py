"""PCIeLeech word stream over LiteEth UDP, and the TLP split in front of it.

The payload is the FT601 byte pipe. LeechCore RawUDP ``send()`` /
``recvfrom()`` this buffer on UDP port 28474. LiteEth's 8-to-32 converter
stores the first pipe byte in ``data[7:0]``, which is the bus order
``PCILeechFIFO`` already byte-swaps, so the bridge passes each word through.

Replies are datagrams back to the last sender, on the same port. A datagram
goes out when 256 words are queued or the stream has been idle for 64
cycles, so a short command response is not held for a full packet.
"""

from migen import *

from litex.soc.interconnect import stream
from litex.soc.interconnect.packet import Dispatcher

from litepcie.common import phy_layout


UDP_PORT = 28474
MAX_WORDS = 256
IDLE_FLUSH = 64


class UDPWordBridge(Module):
    """LiteEth UDP user port (32-bit data) <-> PCILeechFIFO usb_rx/usb_tx."""

    def __init__(self, port, udp_port=UDP_PORT):
        self.usb_rx = stream.Endpoint([("data", 32)])
        self.usb_tx = stream.Endpoint([("data", 32)])

        peer_ip    = Signal(32)
        peer_port  = Signal(16)
        peer_valid = Signal()
        # LiteEth leaves last_be at 0 on every beat except the last, where it
        # marks the valid bytes. A whole word is still data, so keep it.
        self.comb += [
            self.usb_rx.valid.eq(port.source.valid & (~port.source.last | (port.source.last_be != 0))),
            self.usb_rx.data.eq(port.source.data),
            port.source.ready.eq(1),
        ]
        self.sync += If(port.source.valid,
            peer_ip.eq(port.source.ip_address),
            peer_port.eq(port.source.src_port),
            peer_valid.eq(1),
        )

        tx_fifo = stream.SyncFIFO([("data", 32)], MAX_WORDS)
        self.submodules.tx_fifo = tx_fifo
        send_len = Signal(max=MAX_WORDS + 1)
        sent     = Signal(max=MAX_WORDS + 1)
        idle     = Signal(max=IDLE_FLUSH + 1)

        fsm = FSM(reset_state="COLLECT")
        self.submodules.fsm = fsm
        fsm.act("COLLECT",
            tx_fifo.sink.data.eq(self.usb_tx.data),
            tx_fifo.sink.valid.eq(self.usb_tx.valid & peer_valid),
            self.usb_tx.ready.eq(tx_fifo.sink.ready & peer_valid),
            If(self.usb_tx.valid,
                NextValue(idle, 0),
            ).Elif(idle != IDLE_FLUSH,
                NextValue(idle, idle + 1),
            ),
            If(peer_valid & tx_fifo.source.valid &
               ((tx_fifo.level >= MAX_WORDS) | ((idle == IDLE_FLUSH) & ~self.usb_tx.valid)),
                NextValue(send_len, tx_fifo.level),
                NextValue(sent, 0),
                NextState("SEND"),
            ),
        )
        fsm.act("SEND",
            port.sink.valid.eq(tx_fifo.source.valid),
            port.sink.data.eq(tx_fifo.source.data),
            port.sink.last.eq(sent == (send_len - 1)),
            port.sink.last_be.eq(0xf),
            port.sink.length.eq(send_len << 2),
            port.sink.src_port.eq(udp_port),
            port.sink.dst_port.eq(peer_port),
            port.sink.ip_address.eq(peer_ip),
            tx_fifo.source.ready.eq(port.sink.ready),
            If(port.sink.valid & port.sink.ready,
                NextValue(sent, sent + 1),
                If(port.sink.last,
                    NextValue(idle, 0),
                    NextState("COLLECT"),
                ),
            ),
        )


class InboundTLPSplit(Module):
    """Send completions to PCIeLeech. Answer inbound memory reads with UR.

    `TLPSysRouter` forwards MRd32, MWr32, Cpl and CplD on `phy.source`.
    Completions are the replies to PCIeLeech's own reads. A host Memory
    Read of BAR0 has to be completed here, or the root port waits until
    its completion timeout. Memory writes are posted and are consumed.
    """

    def __init__(self, completer_id):
        self.sink   = stream.Endpoint(phy_layout(64))
        self.cpl    = stream.Endpoint(phy_layout(64))   # to PCILeechFIFO.tlp_rx
        self.source = stream.Endpoint(phy_layout(64))   # UR completions, toward phy.sink

        req = stream.Endpoint(phy_layout(64))
        ft  = Signal(8)
        self.comb += ft.eq(self.sink.dat[24:32])
        is_cpl = Signal()
        self.comb += is_cpl.eq((ft == 0x0a) | (ft == 0x4a))

        self.submodules.disp = Dispatcher(self.sink, [self.cpl, req])
        self.comb += self.disp.sel.eq(~is_cpl)

        is_mwr = Signal()
        tc     = Signal(3)
        attr   = Signal(2)
        tag    = Signal(8)
        req_id = Signal(16)
        mwr_now = Signal()
        self.comb += mwr_now.eq(Mux(req.first, req.dat[30], is_mwr))

        ur_hdr0 = Cat(C(0, 12), attr, C(0, 6), tc, C(0, 1), C(0x0a, 8))
        ur_hdr1 = Cat(C(4, 12), C(0, 1), C(1, 3), completer_id)  # status = UR
        ur_hdr2 = Cat(C(0, 7), C(0, 1), tag, req_id)

        fsm = FSM(reset_state="IDLE")
        self.submodules.fsm = fsm
        fsm.act("IDLE",
            req.ready.eq(1),
            If(req.valid & req.first,
                NextValue(is_mwr, req.dat[30]),
                NextValue(tc,     req.dat[20:23]),
                NextValue(attr,   req.dat[12:14]),
                NextValue(tag,    req.dat[40:48]),
                NextValue(req_id, req.dat[48:64]),
            ),
            If(req.valid & req.last & ~mwr_now,
                NextState("SEND0"),
            ),
        )
        fsm.act("SEND0",
            self.source.valid.eq(1),
            self.source.first.eq(1),
            self.source.dat.eq(Cat(ur_hdr0, ur_hdr1)),
            self.source.be.eq(0xff),
            If(self.source.ready,
                NextState("SEND1"),
            ),
        )
        fsm.act("SEND1",
            self.source.valid.eq(1),
            self.source.last.eq(1),
            self.source.dat.eq(Cat(ur_hdr2, C(0, 32))),
            self.source.be.eq(0x0f),
            If(self.source.ready,
                NextState("IDLE"),
            ),
        )

