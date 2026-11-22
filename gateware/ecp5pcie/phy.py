from migen import *
from migen.genlib.cdc import MultiReg

from litex.soc.interconnect.csr import *

from litepcie.common import get_bar_mask

from .core import ECP5PCIeCore
from .adapter import TLPStreamAdapter
from .config_space import ConfigSpace
from .sys_router import TLPSysRouter


class ECP5PCIePHY(Module, AutoCSR):
    """LitePCIe PHY front-end for the ECP5-PCIe soft core.

    Presents the interface `LitePCIeEndpoint` expects - `sink`, `source`, `data_width`,
    `bar0_size`/`bar0_mask`, `id`, `max_request_size`, `max_payload_size` - while the DCU/PHY/
    LTSSM/DLL run in the core's `pcie_rx` domain. `TLPStreamAdapter` crosses the 32-bit TLP
    ports into 64-bit `sys` beats; `TLPSysRouter` answers config space from `ConfigSpace` and
    forwards memory requests and completions to/from LitePCIe.

    `source` carries TLPs *into* LitePCIe (depacketizer), `sink` TLPs *out* of it (packetizer).
    """
    data_width = 64

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

        # Adapter: core 32-bit `pcie_rx` TLP ports <-> 64-bit `sys` streams.
        self.submodules.adapter = adapter = TLPStreamAdapter(
            core.tlp_rx_data, core.tlp_rx_valid, core.tlp_rx_ready,
            core.tlp_tx_data, core.tlp_tx_valid, core.tlp_tx_ready,
            core.retry_buffer_occupation)
        self.comb += [
            adapter.source.connect(router.sink),      # core RX -> router
            router.source.connect(adapter.sink),      # router -> core TX
        ]

        # LitePCIe endpoint interface and identity (captured before the reset wrapper).
        self.sink   = router.phy_sink     # TLPs from the packetizer -> core
        self.source = router.phy_source   # TLPs to the depacketizer <- core
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

        # Event pulses and CSR fields (read before wrapping so the inner signals stay reachable).
        events = {n: getattr(router, n) for n in ["cfg_rd", "cfg_wr", "ur", "cpl", "dropped"]}
        router_id   = router.id
        cfg_command = config_space.command
        cfg_bar0    = config_space.bar0

        dll_up_sys = Signal()
        self.specials += MultiReg(core.dll_up, dll_up_sys)
        config_space = ResetInserter()(config_space)
        router       = ResetInserter()(router)
        self.submodules.config_space = config_space
        self.submodules.router       = router
        self.comb += [
            config_space.reset.eq(~dll_up_sys),
            router.reset.eq(~dll_up_sys),
        ]

        # Router event counters (sys) -> CSR.
        for name, pulse in events.items():
            counter = Signal(32)
            self.sync += If(pulse, counter.eq(counter + 1))
            csr = CSRStatus(32, name=f"tl_{name}", description=f"Router {name} events")
            setattr(self, f"_tl_{name}", csr)
            self.comb += csr.status.eq(counter)

        self._cfg = CSRStatus(fields=[
            CSRField("command", size=16, description="Command register"),
            CSRField("id",      size=16, description="Bus/device/function from the last CfgWr0"),
        ])
        self._bar0 = CSRStatus(32, description="BAR0 as programmed by the host")
        self.comb += [
            self._cfg.fields.command.eq(cfg_command),
            self._cfg.fields.id.eq(router_id),
            self._bar0.status.eq(cfg_bar0),
        ]
