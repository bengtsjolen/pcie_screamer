#!/usr/bin/env python3
"""ECP5 Versa: PCIeLeech FIFO carried over UDP on gigabit Ethernet.

The PCIe endpoint is the same Gen1 `ECP5PCIePHY` as the LitePCIe example.
`PCILeechFIFO` is the Squirrel protocol engine. Its 32-bit word stream,
which `FT601Sync` would carry, is a UDP payload on RGMII port 0 instead.

The board answers ARP and ICMP, so ``ping 192.168.1.50`` is the link check.
PCIeLeech words are the FT601 pipe on UDP port 28474, the port
``pcileech -device RAWUDP://ip=192.168.1.50`` opens.
``tools/pcileech_udp.py`` sends one loopback word and can check a core
read plus the inactivity marker that ends ``FT_ReadPipe``.

``rw[200]`` does not hold the ECP5 core in reset. The core comes up with
slot PERST#, the way the other Versa images do. The Squirrel image holds
the hard IP in reset until the host clears that bit.
"""

import argparse
import os
from pathlib import Path

from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer

from litex.gen import LiteXModule

from litex.soc.integration.soc_core import SoCMini
from litex.soc.integration.builder import Builder
from litex.soc.cores.clock import ECP5PLL
from litex.soc.interconnect import stream
from litex.soc.interconnect.packet import PacketFIFO
from litex.soc.interconnect.csr import CSRStatus, CSRStorage, CSRField

from liteeth.common import eth_udp_user_description, eth_phy_description
from liteeth.phy.ecp5rgmii import LiteEthPHYRGMII
from liteeth.mac.gap import LiteEthMACGap
from liteeth.core import LiteEthUDPIPCore

from gateware.ecp5pcie.phy import ECP5PCIePHY
from gateware.pcileech_fifo import PCILeechFIFO
from gateware.tlp32 import TLP32To64Packer
from gateware.udp_pcileech import UDP_PORT, UDPWordBridge, InboundTLPSplit

from platforms.pcie_versa import Platform, DEVICES

ROOT = Path(__file__).resolve().parent

LED_ACTIVE_LOW = True

MAC_ADDRESS = 0x10e2d5000001
IP_ADDRESS  = "192.168.1.50"
# ARP, ICMP and UDP run here. At gigabit the RGMII TX datapath consumes one
# octet every eth_tx (125 MHz) cycle for the whole frame, and the TX CDC in
# front of it is a plain async FIFO, not a store-and-forward buffer. If the
# MAC user side is slower than 125 MHz it cannot keep that FIFO full, so it
# underruns mid-frame: RX_CTL drops inside the frame, the frame is truncated,
# and the appended FCS no longer matches, so the far end drops it as a CRC
# error.
#
# Running the whole stack at 125 MHz to match the drain does NOT close timing
# on this -8 part: the LiteEth IP/ICMP depacketizer cloud tops out ~75 MHz
# (crg_clkout post-route) next to the PCIe core. So the stack stays at a slow,
# timing-closing clock and the underrun is fixed instead by buffering a full
# frame in the eth_tx domain before it reaches the PHY (store-and-forward).
ETH_CLK_FREQ = int(50e6)


class _CRG(Module):
    def __init__(self, platform):
        self.clock_domains.cd_sys = ClockDomain()

        # The PCIe core wants this 100 MHz clock directly, with no PLL.
        clk100 = platform.request("clk100")
        rst_n  = platform.request("rst_n")
        self.comb += self.cd_sys.clk.eq(clk100)
        self.specials += AsyncResetSynchronizer(self.cd_sys, ~rst_n)

        # Ethernet MAC clock only. The PCIe core keeps the raw 100 MHz pin.
        self.clock_domains.cd_eth = ClockDomain()
        self.submodules.pll = pll = ECP5PLL()
        pll.register_clkin(clk100, 100e6)
        pll.create_clkout(self.cd_eth, ETH_CLK_FREQ)


class _EthTxSkew(LiteXModule):
    """One-tap steps of the RGMII TX clock DELAYF."""

    def __init__(self, crg):
        self._control = CSRStorage(fields=[
            CSRField("direction", size=1, description="1 increases the TX clock delay, 0 decreases it"),
            CSRField("load_n",    size=1, reset=1, description="0 holds the delay at its reset value"),
            CSRField("step",      size=1, description="Write 1 to move one tap"),
        ])
        self._status = CSRStatus(fields=[
            CSRField("cflag", size=1, description="Delay is at its minimum or maximum"),
            CSRField("count", size=8, description="Steps issued since the delay was reloaded"),
        ])

        move  = Signal()
        count = Signal(8)
        cflag = Signal(2)
        self.sync += [
            crg.tx_delay_direction.eq(self._control.fields.direction),
            crg.tx_delay_load_n.eq(self._control.fields.load_n),
            crg.tx_delay_move.eq(move),
            cflag.eq(Cat(crg.tx_delay_cflag, cflag[0])),
            If(~self._control.fields.load_n,
                move.eq(0),
                count.eq(0),
            ).Elif(self._control.re & self._control.fields.step,
                move.eq(1),
                count.eq(count + 1),
            ).Else(
                move.eq(0),
            ),
        ]
        self.comb += [
            self._status.fields.cflag.eq(cflag[1]),
            self._status.fields.count.eq(count),
        ]


class _TXStoreForwardPHY(Module):
    """RGMII PHY wrapper that buffers each formed TX frame in the eth_tx
    (125 MHz) domain before it reaches the wire.

    At gigabit the RGMII TX datapath drains one octet per eth_tx cycle for the
    whole frame, but the MAC/IP/UDP stack runs at 50 MHz (it won't close timing
    at 125 MHz on this part). The MAC's own TX CDC is a plain async FIFO, so the
    slow producer cannot keep it full and it underruns mid-frame: RX_CTL drops
    inside the frame, the frame is truncated, and the appended FCS no longer
    matches, so the far end drops it as a CRC error.

    The MAC still computes the correct FCS value even when the stream has
    bubbles (its CRC clock-enable is gated on ``valid``), so only the byte
    *timing* is wrong. A store-and-forward ``PacketFIFO`` here accumulates the
    complete, already-formed frame (preamble + data + FCS) and replays it to
    the PHY gaplessly. A gap stage after it restores the inter-frame gap that
    the FIFO strips. RX passes straight through, untouched.

    The FIFO is ``buffered`` so its memory is a synchronous-read block RAM with
    a registered output, not distributed LUT RAM with a huge async read mux: at
    this depth the async mux would land directly on the RGMII TX DDR output and
    wreck the 125 MHz eth_tx timing.

    The MAC-facing ``ready`` is tied high rather than taken from the FIFO. The
    RGMII PHY always held ``ready`` high, so the MAC TX datapath's internal
    cdc -> CRC -> gap ready chain terminated at a constant and was never a real
    timing path. Routing the FIFO's ``ready`` back into it instead lights that
    whole chain up as the eth_tx critical path. Tying it high keeps the PHY's
    constant-ready boundary; it is safe because the FIFO is far deeper than a
    max frame, only ever takes one byte per cycle, and drains at line rate, so
    it never fills.
    """

    def __init__(self, phy, depth=2048):
        self.dw     = phy.dw
        self.sink   = stream.Endpoint(eth_phy_description(phy.dw))  # MAC TX -> here
        self.source = phy.source                                   # PHY RX -> MAC

        self.submodules.tx_fifo = ClockDomainsRenamer("eth_tx")(
            PacketFIFO(eth_phy_description(phy.dw),
                       payload_depth=depth, param_depth=4, buffered=True))
        self.submodules.tx_gap = ClockDomainsRenamer("eth_tx")(LiteEthMACGap(phy.dw))
        self.comb += [
            self.sink.connect(self.tx_fifo.sink, omit={"ready"}),
            self.sink.ready.eq(1),
            self.tx_fifo.source.connect(self.tx_gap.sink),
            self.tx_gap.source.connect(phy.sink),
        ]


class SoCECP5UDP(SoCMini):
    def __init__(self, platform, refclk_freq, ignore_perst=False, uart_baudrate=115200,
                 eth_phy=0, ip_address=IP_ADDRESS):
        sys_clk_freq = int(100e6)
        SoCMini.__init__(self, platform, sys_clk_freq,
            ident         = "pcie_screamer ECP5 PCIeLeech UDP",
            ident_version = True,
        )

        self.submodules.crg = _CRG(platform)
        platform.add_period_constraint(self.crg.cd_eth.clk, 1e9/ETH_CLK_FREQ)
        self.add_uartbone(uart_name="serial", baudrate=uart_baudrate)

        self.comb += [
            platform.request("refclk_en").eq(1),
            platform.request("refclk_rst_n").eq(1),
        ]

        perst_n = platform.request("pcie_perst_n")
        self.submodules.pcie_phy = phy = ECP5PCIePHY(platform, perst_n,
            gen          = 1,
            refclk_freq  = refclk_freq,
            ignore_perst = ignore_perst,
            bar0_size    = 0x10000,
        )
        self.add_csr("pcie_phy")

        # RGMII. TX is clocked from the PHY's RX clock, as on the LiteX Versa target.
        # The TX clock delay is a DELAYF so it can be stepped over UARTBone.
        self.submodules.ethphy = LiteEthPHYRGMII(
            clock_pads       = platform.request("eth_clocks", eth_phy),
            pads             = platform.request("eth", eth_phy),
            tx_delay         = 0e-9,
            rx_delay         = 0e-9,
            dynamic_tx_delay = True,
        )
        self.submodules.eth_tx_skew = _EthTxSkew(self.ethphy.crg)
        # Store-and-forward the formed TX frame in eth_tx so the 50 MHz stack
        # cannot underrun the 125 MHz gigabit TX datapath (see the class doc).
        self.submodules.eth_phy_sf = eth_phy_sf = _TXStoreForwardPHY(self.ethphy)
        # The MAC user side is renamed onto cd_eth after the UDP port exists,
        # so the port's own CDC lands on that clock too. A second crossing
        # brings the 32-bit port back to the PCIe/sys side.
        ethcore = LiteEthUDPIPCore(
            eth_phy_sf, MAC_ADDRESS, ip_address, ETH_CLK_FREQ,
            with_icmp=True, rx_cdc_depth=2048, tx_cdc_depth=64)
        udp = ethcore.udp.crossbar.get_port(UDP_PORT, dw=32)
        self.submodules.ethcore = ClockDomainsRenamer("eth")(ethcore)
        udp_layout = eth_udp_user_description(32)
        self.submodules.udp_tx_cdc = stream.ClockDomainCrossing(
            udp_layout, cd_from="sys", cd_to="eth", depth=16, buffered=True)
        self.submodules.udp_rx_cdc = stream.ClockDomainCrossing(
            udp_layout, cd_from="eth", cd_to="sys", depth=16, buffered=True)
        self.comb += [
            self.udp_tx_cdc.source.connect(udp.sink),
            udp.source.connect(self.udp_rx_cdc.sink),
        ]
        udp_sys = type("UDPPort", (), {})()
        udp_sys.sink   = self.udp_tx_cdc.sink
        udp_sys.source = self.udp_rx_cdc.source
        self.submodules.udp_bridge = bridge = UDPWordBridge(udp_sys, UDP_PORT)

        self.submodules.pcileech_fifo = fifo = PCILeechFIFO(
            inactivity_one_shot=True,
            inactivity_ticks=0x000186a0,  # 1 ms at 100 MHz, RawUDP's read end
        )
        # PCILeechFIFO ties usb_rx.ready high itself, so only valid/data are driven here.
        self.comb += [
            fifo.usb_rx.valid.eq(bridge.usb_rx.valid),
            fifo.usb_rx.data.eq(bridge.usb_rx.data),
            bridge.usb_rx.ready.eq(1),
            fifo.usb_tx.connect(bridge.usb_tx),
            fifo.phy_lnk_up.eq(phy._link_status.fields.status),
            fifo.phy_ltssm.eq(phy._link_status.fields.ltssm),
            fifo.phy_lnk_rate.eq(phy._link_status.fields.rate),
            fifo.phy_lnk_width.eq(phy._link_status.fields.width),
            fifo.phy_id.eq(phy.id),
            fifo.cfg_dcommand.eq(phy.config_space.dev_ctl),
            fifo.tx_axis_seen.eq(phy.sink.valid & phy.sink.ready),
            fifo.rx_axis_seen.eq(phy.source.valid),
        ]

        self.submodules.inbound = inbound = InboundTLPSplit(phy.id)
        self.submodules.tlp_tx_conv = packer = TLP32To64Packer(swap_words=False)
        self.comb += [
            phy.source.connect(inbound.sink),
            inbound.cpl.connect(fifo.tlp_rx),
            fifo.tlp_tx.connect(packer.sink),
        ]
        # UR completions and PCIeLeech transmits share the path to the wire.
        from litex.soc.interconnect.packet import Arbiter
        self.submodules.tx_arb = Arbiter([inbound.source, packer.source], phy.sink)

        core = phy.core
        sys_count = Signal(27)
        rx_count  = Signal(26)
        self.sync += sys_count.eq(sys_count + 1)
        self.sync.pcie_rx += rx_count.eq(rx_count + 1)
        leds = Cat(sys_count[-1], core.perst_n, core.rx_present, core.rx_locked,
                   core.rx_aligned, core.link_up, core.dll_up, rx_count[-1])
        self.comb += platform.request_all("user_led").eq(~leds if LED_ACTIVE_LOW else leds)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", choices=DEVICES, required=True,
                        help="LFE5UM (Versa). Gen1 on the non-5G part")
    parser.add_argument("--refclk-mhz", type=float, default=156.25)
    parser.add_argument("--ignore-perst", action="store_true")
    parser.add_argument("--uart-baudrate", type=int, default=115200)
    parser.add_argument("--eth-phy", type=int, default=0, choices=(0, 1),
                        help="Versa RGMII port (default 0, the primary jack)")
    parser.add_argument("--ip-address", default=IP_ADDRESS)
    parser.add_argument("--seed", type=int, default=24,
                        help="nextpnr seed (24 places the SerDes on this family of netlists)")
    parser.add_argument("--build", action="store_true")
    parser.add_argument("--load", action="store_true", help="Load bitstream into SRAM over JTAG")
    parser.add_argument("--flash", action="store_true")
    args = parser.parse_args()

    if args.device != "LFE5UM":
        parser.error("this image is the non-5G Versa (LFE5UM), Gen1")

    tools_bin = ROOT / "tools" / "bin"
    if tools_bin.is_dir():
        os.environ["PATH"] = f"{tools_bin}{os.pathsep}{os.environ['PATH']}"

    refclk_freq = args.refclk_mhz * 1e6
    platform = Platform(device=args.device)
    build_name = f"soc_ecp5_udp_{args.device.lower()}_gen1_ref{int(refclk_freq / 1e6)}"
    output_dir = ROOT / "build" / build_name

    soc = SoCECP5UDP(platform,
        refclk_freq   = refclk_freq,
        ignore_perst  = args.ignore_perst,
        uart_baudrate = args.uart_baudrate,
        eth_phy       = args.eth_phy,
        ip_address    = args.ip_address,
    )
    builder = Builder(soc, output_dir=str(output_dir), csr_csv=str(output_dir / "csr.csv"))
    builder.build(run=args.build, build_name=build_name, seed=args.seed)

    bitstream = os.path.join(builder.gateware_dir, build_name + ".bit")
    if args.load:
        platform.create_programmer().load_bitstream(bitstream)
    if args.flash:
        platform.create_programmer().flash(0, bitstream, verify=True)


if __name__ == "__main__":
    main()
