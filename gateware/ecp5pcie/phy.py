from migen import *
from migen.genlib.cdc import MultiReg

from litex.soc.interconnect.csr import *
from litex.soc.interconnect import stream

from litepcie.common import get_bar_mask, phy_layout

from .core import ECP5PCIeCore
from .adapter import TLPStreamAdapter
from .config_space import ConfigSpace
from .sys_router import TLPSysRouter


class ECP5PCIePHY(Module, AutoCSR):
    """LitePCIe PHY front-end for the ECP5-PCIe soft core.

    Presents the interface `LitePCIeEndpoint` and `SoC.add_pcie` expect: `sink`, `source`,
    `data_width`, `endianness`, `msi`, `bar0_size`/`bar0_mask`, `id`, `max_request_size`,
    `max_payload_size`, and the Squirrel SoC contract on the same ports: `_link_status`,
    `_bus_master_enable`, `cfg_mgmt_*` and `cd_pcie`. The DCU/PHY/LTSSM/DLL run in the core's
    `pcie_rx` domain. `TLPStreamAdapter` crosses the 32-bit TLP ports into 64-bit `sys` beats;
    `TLPSysRouter` answers config space from `ConfigSpace` and forwards memory requests and
    completions. `msi` is consumed in `sys` (same clock as `cd_pcie`), so it needs no crossing.

    `source` carries TLPs toward the SoC (completions in, depacketizer for LitePCIe), `sink`
    carries TLPs from the SoC (requests out, packetizer for LitePCIe). Both use the S7 dword
    order `TLPDMA` already speaks: `dat[31:0]` is DW0 and bits `[31:24]` are the first byte
    on the wire. `cd_pcie` tracks `sys`, because that is the clock the TLP streams use.
    """
    data_width = 64
    endianness = "big"

    def __init__(self, platform, perst_n, gen=1, refclk_freq=None, fabric_clk=False,
                 bar0_size=4096, config=None, **core_kwargs):
        config = dict(config or {})
        config["bar0_size"] = bar0_size
        config["gen"]       = gen

        self.bar0_size = bar0_size
        self.bar0_mask = get_bar_mask(bar0_size)
        self.id               = Signal(16, reset_less=True)
        self.max_request_size = Signal(16, reset_less=True)
        self.max_payload_size = Signal(16, reset_less=True)
        self.external_hard_ip = False

        # Soft core (DCU/PHY/LTSSM/DLL) with the raw TLP ports exposed.
        self.submodules.core = core = ECP5PCIeCore(platform, perst_n, gen=gen,
            refclk_freq=refclk_freq, fabric_clk=fabric_clk, external_tl=True, **core_kwargs)

        # Config space and router in `sys`. Reset routing/config state when the link drops.
        config_space = ClockDomainsRenamer({"pcie_rx": "sys"})(ConfigSpace(**config))
        router       = TLPSysRouter(config_space)
        self.vendor_id = config_space.vendor_id
        self.device_id = config_space.device_id
        self.revision  = config_space.revision

        # Adapter: core 32-bit `pcie_rx` TLP ports <-> 64-bit `sys` streams.
        self.submodules.adapter = adapter = TLPStreamAdapter(
            core.tlp_rx_data, core.tlp_rx_valid, core.tlp_rx_ready,
            core.tlp_tx_data, core.tlp_tx_valid, core.tlp_tx_ready,
            core.retry_buffer_occupation)
        # Register the RX stream between the adapter's packing FSM and the router's classification/
        # config-capture logic; that combinational cone is the sys critical path otherwise.
        rx_pipe = stream.Buffer(phy_layout(64), pipe_valid=True, pipe_ready=True)
        self.submodules.rx_pipe = rx_pipe
        self.comb += [
            adapter.source.connect(rx_pipe.sink),     # core RX -> pipe
            rx_pipe.source.connect(router.sink),      # pipe -> router
            router.source.connect(adapter.sink),      # router -> core TX
        ]

        # LitePCIe endpoint interface and identity (captured before the reset wrapper).
        self.sink   = router.phy_sink     # TLPs from the packetizer -> core
        self.source = router.phy_source   # TLPs to the depacketizer <- core
        self.msi    = router.msi          # LitePCIeMSI -> one MWr32
        self.comb += [
            self.id.eq(router.id),
            self.max_payload_size.eq(128 << config_space.max_payload_size),
            self.max_request_size.eq(128 << config_space.max_read_request),
        ]
        self.link_up    = core.link_up
        self.dll_up     = core.dll_up
        self.bus_master = config_space.bus_master
        self.mem_enable = config_space.mem_enable
        self.bar0       = config_space.bar0
        self.command    = config_space.command

        # Squirrel cfg_mgmt: one masked config write, retried if a host CfgWr lands the same cycle.
        self.cfg_mgmt_dwaddr     = Signal(10)
        self.cfg_mgmt_di         = Signal(32)
        self.cfg_mgmt_byte_en    = Signal(4)
        self.cfg_mgmt_wr_en      = Signal()
        self.cfg_mgmt_rd_wr_done = Signal()
        mgmt_hold = Signal()
        self.comb += [
            config_space.mgmt_we.eq((self.cfg_mgmt_wr_en | mgmt_hold) & ~config_space.we),
            config_space.mgmt_reg.eq(self.cfg_mgmt_dwaddr),
            config_space.mgmt_be.eq(self.cfg_mgmt_byte_en),
            config_space.mgmt_data.eq(self.cfg_mgmt_di),
            self.cfg_mgmt_rd_wr_done.eq(config_space.mgmt_done),
        ]
        self.sync += mgmt_hold.eq((self.cfg_mgmt_wr_en | mgmt_hold) & config_space.we)

        # The SoC's BME timer uses `sync.pcie`. The TLP streams are in `sys`, so this domain
        # follows `sys` rather than the 62.5 MHz DCU word clock.
        self.clock_domains.cd_pcie = ClockDomain()
        self.comb += [
            self.cd_pcie.clk.eq(ClockSignal("sys")),
            self.cd_pcie.rst.eq(ResetSignal("sys")),
        ]

        # Event pulses and CSR fields (read before wrapping so the inner signals stay reachable).
        events = {n: getattr(router, n) for n in ["cfg_rd", "cfg_wr", "ur", "cpl", "dropped"]}
        router_id   = router.id
        cfg_command    = config_space.command
        cfg_bar0       = config_space.bar0
        cfg_msi_enable = config_space.msi_enable

        dll_up_sys = Signal()
        ltssm_sys  = Signal(8)
        self.specials += [
            MultiReg(core.dll_up, dll_up_sys),
            MultiReg(core.ltssm_state, ltssm_sys),
        ]
        config_space = ResetInserter()(config_space)
        router       = ResetInserter()(router)
        self.submodules.config_space = config_space
        self.submodules.router       = router
        self.comb += [
            config_space.reset.eq(~dll_up_sys),
            router.reset.eq(~dll_up_sys),
        ]

        # Router event counters (sys) -> CSR. 16-bit wrapping: they are debug aids, and wider
        # counters only add to the CSR read mux on the `sys` critical path.
        for name, pulse in events.items():
            counter = Signal(16)
            self.sync += If(pulse, counter.eq(counter + 1))
            csr = CSRStatus(16, name=f"tl_{name}", description=f"Router {name} events (wraps)")
            setattr(self, f"_tl_{name}", csr)
            self.comb += csr.status.eq(counter)

        self._cfg = CSRStatus(fields=[
            CSRField("command", size=16, description="Command register"),
            CSRField("id",      size=16, description="Bus/device/function from the last CfgWr0"),
        ])
        self._bar0 = CSRStatus(32, description="BAR0 as programmed by the host")
        # Same field layout as S7PCIEPHY, so the Squirrel SoC's link_ready expression applies.
        self._link_status = CSRStatus(fields=[
            CSRField("status", size=1, description="DLL active"),
            CSRField("rate",   size=1, description="0 = 2.5 GT/s, 1 = 5.0 GT/s"),
            CSRField("width",  size=2, description="0 = x1"),
            CSRField("ltssm",  size=6, description="ECP5-PCIe LTSSM state index"),
        ])
        self._bus_master_enable = CSRStatus(description="1 when Command[BME] is set")
        self._msi_enable        = CSRStatus(description="1 when the MSI capability enable bit is set")
        self._msix_enable       = CSRStatus(description="MSI-X enable (stuck at 0)")
        self._max_request_size  = CSRStatus(16, description="Max read request size (bytes)")
        self._max_payload_size  = CSRStatus(16, description="Max payload size (bytes)")
        self.comb += [
            self._cfg.fields.command.eq(cfg_command),
            self._cfg.fields.id.eq(router_id),
            self._bar0.status.eq(cfg_bar0),
            self._link_status.fields.status.eq(dll_up_sys),
            self._link_status.fields.rate.eq(gen == 2),
            self._link_status.fields.width.eq(0),
            self._link_status.fields.ltssm.eq(ltssm_sys),
            self._bus_master_enable.status.eq(self.bus_master),
            self._msi_enable.status.eq(cfg_msi_enable),
            self._max_request_size.status.eq(self.max_request_size),
            self._max_payload_size.status.eq(self.max_payload_size),
        ]
