import subprocess
import sys
from pathlib import Path

from migen import *
from migen.genlib.cdc import MultiReg, GrayCounter

from litex.soc.interconnect import stream
from litex.soc.interconnect.csr import *
from litex.soc.cores.freqmeter import FreqMeter

from .capture import TLPCapture
from .config_space import ConfigSpace
from .router import TLPRouter

ROOT        = Path(__file__).resolve().parents[2]
VERILOG_DIR = Path(__file__).resolve().parent / "verilog"

TRACE_LAYOUT = [("time", 32), ("ltssm", 8), ("dll", 8)]


def default_refclk_freq(gen):
    return 100e6 if gen == 1 else 200e6


def core_name(gen=1, refclk_freq=None, fabric_clk=False, transaction_layer=False):
    refclk_freq = refclk_freq or default_refclk_freq(gen)
    return (f"ecp5_pcie_core_gen{gen}"
            + ("" if refclk_freq == default_refclk_freq(gen) else f"_ref{int(refclk_freq/1e6)}")
            + ("_fabclk" if fabric_clk else "")
            + ("_tl" if transaction_layer else ""))


def core_verilog(gen=1, refclk_freq=None, fabric_clk=False, transaction_layer=False):
    refclk_freq = refclk_freq or default_refclk_freq(gen)
    path = VERILOG_DIR / f"{core_name(gen, refclk_freq, fabric_clk, transaction_layer)}.v"
    if not path.exists():
        cmd = [sys.executable, str(ROOT / "tools" / "gen_ecp5_pcie_core.py"), "--gen", str(gen),
               "--refclk-mhz", str(refclk_freq/1e6)]
        if fabric_clk:
            cmd.append("--fabric-clk")
        if transaction_layer:
            cmd.append("--transaction-layer")
        subprocess.run(cmd, check=True)
    return path


class _GrayEventCounter(Module):
    """Counts single-cycle events in `cd` and presents the count in sys."""
    def __init__(self, event, cd, width=32):
        self.value = Signal(width)

        counter = ClockDomainsRenamer(cd)(GrayCounter(width))
        self.submodules += counter
        self.comb += counter.ce.eq(event)
        gray = Signal(width)
        self.specials += MultiReg(counter.q, gray)
        # Log-depth prefix XOR; GrayDecoder's serial chain misses 100 MHz at 32 bits.
        binary, shift = gray, 1
        while shift < width:
            binary = binary ^ (binary >> shift)
            shift *= 2
        self.sync += self.value.eq(binary)


class ECP5PCIeCore(Module, AutoCSR):
    """ECP5-PCIe DCU/PHY/LTSSM/DLL black box with bring-up CSRs.

    The generated core runs in `pcie_rx` (DCU recovered clock, 62.5 MHz at Gen1, 125 MHz at
    Gen2). `pcie_tx` is frequency-locked to it. `sys` must be 100 MHz: the core uses it for
    SCI access and, with `fabric_clk`, as DCU reference clock. Otherwise the DCU reference is
    EXTREF0 at `refclk_freq` (Gen1: 100 or 156.25 MHz; Gen2: 200 MHz).
    """
    def __init__(self, platform, perst_n, gen=1, refclk_freq=None, fabric_clk=False,
                 transaction_layer=False, external_tl=False, ignore_perst=False,
                 trace_depth=64, config=None):
        self.gen = gen
        name = core_name(gen, refclk_freq, fabric_clk, transaction_layer)
        platform.add_source(str(core_verilog(gen, refclk_freq, fabric_clk, transaction_layer)))

        self.clock_domains.cd_pcie_rx = ClockDomain(reset_less=True)
        self.clock_domains.cd_pcie_tx = ClockDomain(reset_less=True)
        # The word clocks are internal nets, which LiteX's FREQUENCY PORT does not reach. After
        # flattening, nextpnr names them after their drivers inside the generated core.
        word_clk_mhz = 62.5 if gen == 1 else 125.0
        clocks = {
            "serdes.pcie_clk":        word_clk_mhz,      # rx: 4 symbols per cycle
            "aligner_tx_clk":         word_clk_mhz,      # tx: 4 symbols per cycle
            "serdes.rxf_clk":         2*word_clk_mhz,    # DCU RX PCLK, 2 symbols per cycle
            "serdes.tx_fifo.txf_clk": 2*word_clk_mhz,    # DCU TX PCLK, 2 symbols per cycle
        }
        for net, mhz in clocks.items():
            platform.add_platform_command(f'FREQUENCY NET "{name}.{net}" {mhz} MHz;')

        # Core ports, pcie_rx domain.
        self.perst_n     = Signal()
        self.ltssm_state = Signal(8)
        self.link_up     = Signal()
        self.dll_state   = Signal(8)
        self.dll_up      = Signal()
        self.rx_present  = Signal()
        self.rx_locked   = Signal()
        self.rx_aligned  = Signal()
        self.tx_locked   = Signal()
        self.reset_done  = Signal()
        self.det_status  = Signal()
        self.rx_lol_err  = Signal()
        self.rx_sym_err  = Signal()
        self.rx_symbol   = Signal(36)
        self.rx_valid    = Signal(4)
        self.tx_symbol   = Signal(36)
        self.fc = {
            "ph":   Signal(8), "pd":   Signal(12),
            "nph":  Signal(8), "npd":  Signal(12),
            "cplh": Signal(8), "cpld": Signal(12),
        }
        self.retry_buffer_occupation   = Signal(8)
        self.receive_buffer_occupation = Signal(8)
        self.tx_seq_num   = Signal(12)
        self.rx_seq_num   = Signal(12)
        self.ack_nak_valid    = Signal()
        self.ack_nak_ack      = Signal()
        self.ack_nak_id       = Signal(12)
        self.replay_started   = Signal()
        self.replay_abandoned = Signal()
        self.retry_debug      = Signal(96)
        self.tlp_rx_data  = Signal(32)
        self.tlp_rx_valid = Signal(4)
        self.tlp_rx_ready = Signal()
        self.tlp_tx_data  = Signal(32)
        self.tlp_tx_valid = Signal(4)
        self.tlp_tx_ready = Signal()

        self.comb += self.perst_n.eq(1 if ignore_perst else perst_n)

        self.specials += Instance(name,
            i_clk         = ClockSignal("sys"),
            i_rst         = ResetSignal("sys"),
            i_perst_n     = self.perst_n,
            o_rx_clk_out  = self.cd_pcie_rx.clk,
            o_tx_clk_out  = self.cd_pcie_tx.clk,
            o_ltssm_state = self.ltssm_state,
            o_link_up     = self.link_up,
            o_dll_state   = self.dll_state,
            o_dll_up      = self.dll_up,
            o_rx_present  = self.rx_present,
            o_rx_locked   = self.rx_locked,
            o_rx_aligned  = self.rx_aligned,
            o_tx_locked   = self.tx_locked,
            o_reset_done  = self.reset_done,
            o_det_status  = self.det_status,
            o_rx_lol_err  = self.rx_lol_err,
            o_rx_sym_err  = self.rx_sym_err,
            o_rx_symbol   = self.rx_symbol,
            o_rx_valid    = self.rx_valid,
            o_tx_symbol   = self.tx_symbol,
            **{f"o_fc_{k}": v for k, v in self.fc.items()},
            o_retry_buffer_occupation   = self.retry_buffer_occupation,
            o_receive_buffer_occupation = self.receive_buffer_occupation,
            o_tx_seq_num   = self.tx_seq_num,
            o_rx_seq_num   = self.rx_seq_num,
            o_ack_nak_valid    = self.ack_nak_valid,
            o_ack_nak_ack      = self.ack_nak_ack,
            o_ack_nak_id       = self.ack_nak_id,
            o_replay_started   = self.replay_started,
            o_replay_abandoned = self.replay_abandoned,
            o_retry_debug      = self.retry_debug,
            o_tlp_rx_data  = self.tlp_rx_data,
            o_tlp_rx_valid = self.tlp_rx_valid,
            i_tlp_rx_ready = self.tlp_rx_ready,
            i_tlp_tx_data  = self.tlp_tx_data,
            i_tlp_tx_valid = self.tlp_tx_valid,
            o_tlp_tx_ready = self.tlp_tx_ready,
        )

        # Capture events: [31] Ack/Nak received, [30] it was an Ack, [29] replay started,
        # [28] replay abandoned, [23:12] oldest unacknowledged sequence number, [11:0] Ack/Nak ID.
        self.submodules.capture = TLPCapture(self.tlp_rx_data, self.tlp_rx_valid,
            self.tlp_tx_data, self.tlp_tx_valid, active=self.dll_up,
            event=self.ack_nak_valid | self.replay_started | self.replay_abandoned,
            event_data=Cat(self.ack_nak_id, self.retry_debug[12:24], C(0, 4), self.replay_abandoned,
                           self.replay_started, self.ack_nak_ack & self.ack_nak_valid, self.ack_nak_valid))

        if transaction_layer:
            # The core answers config requests itself.
            self.comb += [
                self.tlp_rx_ready.eq(1),
                self.tlp_tx_valid.eq(0),
            ]
        elif external_tl:
            # The TLP ports are exposed; routing and config space live in `sys` (ECP5PCIePHY).
            pass
        else:
            # Config space and routing state reset with the link (Hot Reset, link loss).
            config_space = ConfigSpace(gen=gen, **(config or {}))
            router = TLPRouter(config_space,
                self.tlp_rx_data, self.tlp_rx_valid, self.tlp_rx_ready,
                self.tlp_tx_data, self.tlp_tx_valid, self.tlp_tx_ready,
                self.retry_buffer_occupation)
            config_space = ResetInserter(["pcie_rx"])(config_space)
            router       = ResetInserter(["pcie_rx"])(router)
            self.submodules.config_space = config_space
            self.submodules.router       = router
            self.comb += [
                config_space.reset_pcie_rx.eq(~self.dll_up),
                router.reset_pcie_rx.eq(~self.dll_up),
            ]
            for name, event in [("cfg_rd", router.cfg_rd), ("cfg_wr", router.cfg_wr), ("ur", router.ur),
                                ("dropped", router.dropped), ("cpl", router.cpl)]:
                counter = _GrayEventCounter(event, "pcie_rx")
                setattr(self.submodules, f"tl_{name}_counter", counter)
                csr = CSRStatus(32, name=f"tl_{name}", description=f"Router {name} events")
                setattr(self, f"_tl_{name}", csr)
                self.comb += csr.status.eq(counter.value)
            self._cfg = CSRStatus(fields=[
                CSRField("command", size=16, description="Command register"),
                CSRField("id",      size=16, description="Bus/device/function from the last CfgWr0"),
            ])
            self._bar0 = CSRStatus(32, description="BAR0 as programmed by the host")
            self.specials += [
                MultiReg(config_space.command, self._cfg.fields.command),
                MultiReg(router.id,            self._cfg.fields.id),
                MultiReg(config_space.bar0,    self._bar0.status),
            ]

        # Status CSRs -----------------------------------------------------------------------------
        self._status = CSRStatus(fields=[
            CSRField("link_up",    size=1, description="LTSSM in L0"),
            CSRField("dll_up",     size=1, description="DL_Active"),
            CSRField("rx_present", size=1, description="DCU: no loss of signal"),
            CSRField("rx_locked",  size=1, description="DCU: CDR locked"),
            CSRField("rx_aligned", size=1, description="DCU: word alignment"),
            CSRField("tx_locked",  size=1, description="DCU: TX PLL locked"),
            CSRField("reset_done", size=1, description="DCU reset sequence done"),
            CSRField("det_status", size=1, description="Receiver detected"),
            CSRField("perst_n",    size=1, description="PERST# (1 = released)"),
        ])
        self._state = CSRStatus(fields=[
            CSRField("ltssm", size=8, description="ECP5-PCIe ltssm.State"),
            CSRField("dll",   size=8, description="ECP5-PCIe dll.State"),
        ])
        self._fc_p   = CSRStatus(fields=[CSRField("hdr", size=8), CSRField("data", size=12)],
            description="Posted credits advertised by the link partner")
        self._fc_np  = CSRStatus(fields=[CSRField("hdr", size=8), CSRField("data", size=12)],
            description="Non-posted credits advertised by the link partner")
        self._fc_cpl = CSRStatus(fields=[CSRField("hdr", size=8), CSRField("data", size=12)],
            description="Completion credits advertised by the link partner (0 = infinite)")
        self._dll = CSRStatus(fields=[
            CSRField("tx_seq",  size=12),
            CSRField("rx_seq",  size=12),
            CSRField("retry_occupancy",   size=4),
            CSRField("receive_occupancy", size=4),
        ])
        self._rx_lol_errors = CSRStatus(32, description="Cycles without CDR lock while training or in L0")
        self._rx_sym_errors = CSRStatus(32, description="Cycles with an 8b10b error while training or in L0")
        self._rx_tlps       = CSRStatus(32, description="TLPs delivered by the DLL")
        self._retry_debug   = CSRStatus(96, description="ackd_seq[11:0], FIFO r_data[23:12], "
            "r_level[26:24], r_rdy[27], replay state[29:28], slot valid[33:30], slot IDs[81:34]")
        self.specials += MultiReg(self.retry_debug, self._retry_debug.status)

        # A CSRStatus with fields drives .status from its field signals, so drive the fields.
        def sync_fields(csr, sources):
            for name, src in sources.items():
                self.specials += MultiReg(src, getattr(csr.fields, name))

        sync_fields(self._status, {
            "link_up":    self.link_up,    "dll_up":     self.dll_up,
            "rx_present": self.rx_present, "rx_locked":  self.rx_locked,
            "rx_aligned": self.rx_aligned, "tx_locked":  self.tx_locked,
            "reset_done": self.reset_done, "det_status": self.det_status,
            "perst_n":    self.perst_n,
        })
        sync_fields(self._state,  {"ltssm": self.ltssm_state, "dll": self.dll_state})
        sync_fields(self._fc_p,   {"hdr": self.fc["ph"],   "data": self.fc["pd"]})
        sync_fields(self._fc_np,  {"hdr": self.fc["nph"],  "data": self.fc["npd"]})
        sync_fields(self._fc_cpl, {"hdr": self.fc["cplh"], "data": self.fc["cpld"]})
        sync_fields(self._dll, {
            "tx_seq": self.tx_seq_num, "rx_seq": self.rx_seq_num,
            "retry_occupancy":   self.retry_buffer_occupation[:4],
            "receive_occupancy": self.receive_buffer_occupation[:4],
        })

        tlp_rx_valid_d = Signal()
        self.sync.pcie_rx += tlp_rx_valid_d.eq(self.tlp_rx_valid != 0)
        tlp_rx_end = tlp_rx_valid_d & (self.tlp_rx_valid == 0)

        self.submodules.lol_errors = lol_errors = _GrayEventCounter(self.rx_lol_err, "pcie_rx")
        self.submodules.sym_errors = sym_errors = _GrayEventCounter(self.rx_sym_err, "pcie_rx")
        self.submodules.rx_tlps    = rx_tlps    = _GrayEventCounter(tlp_rx_end,      "pcie_rx")
        self.comb += [
            self._rx_lol_errors.status.eq(lol_errors.value),
            self._rx_sym_errors.status.eq(sym_errors.value),
            self._rx_tlps.status.eq(rx_tlps.value),
        ]

        # DCU word clocks, in Hz.
        self.submodules.rx_freq = FreqMeter(period=int(100e6), clk=self.cd_pcie_rx.clk)
        self.submodules.tx_freq = FreqMeter(period=int(100e6), clk=self.cd_pcie_tx.clk)

        # LTSSM / DLL transition trace -------------------------------------------------------------
        # Each entry is the new state and the pcie_rx cycle count since PERST# release.
        perst_rx = Signal()
        time     = Signal(32)
        prev     = Signal(16)
        cur      = Cat(self.ltssm_state, self.dll_state)
        self.specials += MultiReg(~self.perst_n, perst_rx, "pcie_rx")

        trace = stream.AsyncFIFO(TRACE_LAYOUT, trace_depth)
        trace = ClockDomainsRenamer({"write": "pcie_rx", "read": "sys"})(trace)
        self.submodules.trace = trace

        overflow_rx = Signal()
        self.sync.pcie_rx += [
            prev.eq(cur),
            If(perst_rx,
                time.eq(0),
            ).Elif(~time[-1],
                time.eq(time + 1),
            ),
            If(trace.sink.valid & ~trace.sink.ready, overflow_rx.eq(1)),
        ]
        self.comb += [
            trace.sink.valid.eq(cur != prev),
            trace.sink.time.eq(time),
            trace.sink.ltssm.eq(self.ltssm_state),
            trace.sink.dll.eq(self.dll_state),
        ]

        self._trace_level = CSRStatus(fields=[
            CSRField("valid",    size=1, description="An entry is available in trace_data"),
            CSRField("overflow", size=1, description="Entries were dropped (sticky until PERST#)"),
        ])
        self._trace_data = CSRStatus(48, description="time[31:0], ltssm[39:32], dll[47:40]")
        self._trace_pop  = CSR()
        overflow_sys = Signal()
        self.specials += MultiReg(overflow_rx, overflow_sys)
        self.comb += [
            self._trace_level.fields.valid.eq(trace.source.valid),
            self._trace_level.fields.overflow.eq(overflow_sys),
            self._trace_data.status.eq(Cat(trace.source.time, trace.source.ltssm, trace.source.dll)),
            trace.source.ready.eq(self._trace_pop.re),
        ]
        self.sync.pcie_rx += If(perst_rx, overflow_rx.eq(0))
