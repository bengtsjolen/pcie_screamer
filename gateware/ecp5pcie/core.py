import subprocess
import sys
from pathlib import Path

from migen import *
from migen.genlib.cdc import MultiReg, GrayCounter, GrayDecoder

from litex.soc.interconnect import stream
from litex.soc.interconnect.csr import *
from litex.soc.cores.freqmeter import FreqMeter

ROOT        = Path(__file__).resolve().parents[2]
VERILOG_DIR = Path(__file__).resolve().parent / "verilog"

TRACE_LAYOUT = [("time", 32), ("ltssm", 8), ("dll", 8)]


def core_name(gen=1, fabric_clk=False, transaction_layer=False):
    return (f"ecp5_pcie_core_gen{gen}"
            + ("_fabclk" if fabric_clk else "")
            + ("_tl" if transaction_layer else ""))


def core_verilog(gen=1, fabric_clk=False, transaction_layer=False):
    path = VERILOG_DIR / f"{core_name(gen, fabric_clk, transaction_layer)}.v"
    if not path.exists():
        cmd = [sys.executable, str(ROOT / "tools" / "gen_ecp5_pcie_core.py"), "--gen", str(gen)]
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
        decoder = GrayDecoder(width)
        self.submodules += counter, decoder
        self.comb += counter.ce.eq(event)
        self.specials += MultiReg(counter.q, decoder.i)
        self.comb += self.value.eq(decoder.o)


class ECP5PCIeCore(Module, AutoCSR):
    """ECP5-PCIe DCU/PHY/LTSSM/DLL black box with bring-up CSRs.

    The generated core runs in `pcie_rx` (DCU recovered clock, 62.5 MHz at Gen1, 125 MHz at
    Gen2). `pcie_tx` is frequency-locked to it. `sys` must be 100 MHz: the core uses it for
    SCI access and, with `fabric_clk`, as DCU reference clock.
    """
    def __init__(self, platform, perst_n, gen=1, fabric_clk=False, transaction_layer=False,
                 ignore_perst=False, trace_depth=64):
        self.gen = gen
        name = core_name(gen, fabric_clk, transaction_layer)
        platform.add_source(str(core_verilog(gen, fabric_clk, transaction_layer)))

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
            o_tlp_rx_data  = self.tlp_rx_data,
            o_tlp_rx_valid = self.tlp_rx_valid,
            i_tlp_rx_ready = self.tlp_rx_ready,
            i_tlp_tx_data  = self.tlp_tx_data,
            i_tlp_tx_valid = self.tlp_tx_valid,
            o_tlp_tx_ready = self.tlp_tx_ready,
        )

        # Until the TLP layers exist, drain received TLPs and send none.
        self.comb += [
            self.tlp_rx_ready.eq(1),
            self.tlp_tx_valid.eq(0),
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

        status_rx = Cat(self.link_up, self.dll_up, self.rx_present, self.rx_locked,
                        self.rx_aligned, self.tx_locked, self.reset_done, self.det_status)
        status_sys = Signal(len(status_rx))
        perst_n_sys = Signal()
        self.specials += [
            MultiReg(status_rx, status_sys),
            MultiReg(self.perst_n, perst_n_sys),
            MultiReg(Cat(self.ltssm_state, self.dll_state), self._state.status),
            MultiReg(Cat(self.fc["ph"],   self.fc["pd"]),   self._fc_p.status),
            MultiReg(Cat(self.fc["nph"],  self.fc["npd"]),  self._fc_np.status),
            MultiReg(Cat(self.fc["cplh"], self.fc["cpld"]), self._fc_cpl.status),
            MultiReg(Cat(self.tx_seq_num, self.rx_seq_num,
                         self.retry_buffer_occupation[:4], self.receive_buffer_occupation[:4]),
                     self._dll.status),
        ]
        self.comb += self._status.status.eq(Cat(status_sys, perst_n_sys))

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
