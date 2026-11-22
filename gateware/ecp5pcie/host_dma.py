from migen import *

from litex.soc.interconnect.csr import *


class HostDMA(Module):
    """Single-dword host-memory DMA on a LitePCIe crossbar master port (plain-signal core).

    Issues one memory request to an arbitrary host physical address and, for reads, captures
    the completion. 32-bit vs 64-bit addressing (MWr32/MRd32 vs MWr64/MRd64) is chosen by the
    packetizer from the address, so the endpoint must be built with `address_width=64`.

    This is the pcileech-shaped primitive: a controller (here, CSRs driven by the dev host over
    UARTBone) reads or writes host memory a dword at a time. Requests are gated on `enable`
    (wired to Bus Master Enable) so the endpoint never DMAs before the host allows it.

    Control interface (all held stable between `start` and `done`):
        addr   : 64-bit host physical address
        wdata  : write data
        we     : 1 write, 0 read
        start  : one-cycle strobe to launch a request
        enable : gate (request is dropped on the floor while low)
    Status:
        busy   : request in flight
        done   : one-cycle strobe when a request finishes
        rdata  : read completion data (valid after a read's `done`)
        err    : last completion was an error (UR/CA)
    """
    def __init__(self, endpoint, enable=None):
        port = endpoint.crossbar.get_master_port()
        self.port = port  # exposed for simulation probing
        aw = len(port.source.adr)

        self.addr  = Signal(64)
        self.wdata = Signal(32)
        self.we    = Signal()
        self.start = Signal()
        self.enable = Signal(reset=1) if enable is None else enable

        self.rdata = Signal(32)
        self.busy  = Signal()
        self.done  = Signal()
        self.err   = Signal()

        # # #

        we_r = Signal()

        # Request parameters track the control inputs; they are stable between start and done.
        # `valid`/`we` are driven per-state in the FSM.
        self.sync += [
            port.source.channel.eq(port.channel),
            port.source.first.eq(1),
            port.source.last.eq(1),
            port.source.len.eq(1),
            port.source.req_id.eq(endpoint.phy.id),
            port.source.tag.eq(0),
            port.source.adr.eq(self.addr[:aw]),
            port.source.dat[:32].eq(self.wdata),
        ]

        self.submodules.fsm = fsm = FSM(reset_state="IDLE")
        fsm.act("IDLE",
            If(self.start & self.enable,
                NextValue(we_r, self.we),
                NextState("ISSUE"),
            ),
        )
        fsm.act("ISSUE",
            self.busy.eq(1),
            port.source.valid.eq(1),
            port.source.we.eq(we_r),
            If(port.source.ready,
                If(we_r,
                    self.done.eq(1),
                    NextState("IDLE"),
                ).Else(
                    NextState("RECEIVE"),
                ),
            ),
        )
        fsm.act("RECEIVE",
            self.busy.eq(1),
            port.sink.ready.eq(1),
            If(port.sink.valid & port.sink.first,
                NextValue(self.rdata, port.sink.dat[:32]),
                NextValue(self.err, port.sink.err),
                self.done.eq(1),
                NextState("IDLE"),
            ),
        )


class HostDMAScratch(Module, AutoCSR):
    """CSR wrapper around `HostDMA` so the dev host can poke host memory over UARTBone.

    CSR flow:
        addr_lo/addr_hi <- target physical address
        wdata           <- value for a write
        ctrl.we         <- 1 to write, 0 to read
        start           <- strobe (write triggers a request)
        poll status.busy == 0 (status.done latches that a request completed)
        rdata           -> completion data (reads)
    `ctrl.force` issues even when Bus Master Enable is clear.
    """
    def __init__(self, endpoint, enable=None):
        enable = Signal(reset=1) if enable is None else enable

        self._addr_lo = CSRStorage(32, description="Host address bits [31:0]")
        self._addr_hi = CSRStorage(32, description="Host address bits [63:32] (0 -> 32-bit TLP)")
        self._wdata   = CSRStorage(32, description="Write data")
        self._rdata   = CSRStatus(32,  description="Read completion data")
        self._ctrl    = CSRStorage(fields=[
            CSRField("we",    size=1, description="1: write, 0: read"),
            CSRField("force", size=1, description="Issue even if Bus Master Enable is clear"),
        ])
        self._start   = CSR()  # write strobes a request
        self._status  = CSRStatus(fields=[
            CSRField("busy",       size=1),
            CSRField("done",       size=1, description="Sticky: a request has completed"),
            CSRField("err",        size=1, description="Last completion was an error"),
            CSRField("bus_master", size=1, description="Host has set Bus Master Enable"),
        ])

        gated_enable = Signal()
        self.comb += gated_enable.eq(enable | self._ctrl.fields.force)
        self.submodules.dma = dma = HostDMA(endpoint, enable=gated_enable)

        done_sticky = Signal()
        self.sync += If(dma.start, done_sticky.eq(0)).Elif(dma.done, done_sticky.eq(1))
        self.comb += [
            dma.addr.eq(Cat(self._addr_lo.storage, self._addr_hi.storage)),
            dma.wdata.eq(self._wdata.storage),
            dma.we.eq(self._ctrl.fields.we),
            dma.start.eq(self._start.re),   # CSR.re is the *write* strobe
            self._rdata.status.eq(dma.rdata),
            self._status.fields.busy.eq(dma.busy),
            self._status.fields.done.eq(done_sticky),
            self._status.fields.err.eq(dma.err),
            self._status.fields.bus_master.eq(enable),
        ]
