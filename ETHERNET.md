# PCIeLeech over UDP: Ethernet bring-up (`soc_ecp5_udp.py`)

The goal of `soc_ecp5_udp.py` is to carry the PCIeLeech FIFO word stream over
UDP on the Versa gigabit jack instead of an FT601 shield. Before any of that
matters, the board has to be reachable: answer ARP, answer ICMP, so that
`ping 192.168.1.50` works. This document records how that link was brought up,
what did **not** work, and the fix that made frames leave the board intact.

- RGMII port 0, static `192.168.1.50`, MAC `10:e2:d5:00:00:01`.
- PCIeLeech words are the FT601 pipe on UDP port `28474` (`tools/pcileech_udp.py`, LeechCore RawUDP).

## Symptom

The board received ARP requests and generated an ARP reply. The directly
attached PHY/transceiver accepted the reply and put it on the wire, but the
switch (and host NIC) dropped it. The working theory was a **bad FCS / CRC** on
the transmitted frame, which is exactly what the switch reports and silently
drops. ICMP had the same problem. So nothing ever came back and `ping` failed.

## Root cause: gigabit TX underrun

At gigabit, RGMII TX runs in the `eth_tx` clock domain off the PHY's recovered
125 MHz RX clock, and the LiteEth RGMII PHY is gigabit-only (DDR). The TX
datapath therefore **consumes one octet every `eth_tx` (125 MHz) cycle for the
entire frame**, continuously.

The MAC/IP/UDP stack, however, ran at 50 MHz (`ETH_CLK_FREQ`), because the PCIe
core already owns the 100 MHz `sys`/`clk100` and the stack would not meet timing
at 100 MHz. The only thing bridging the slow stack to the fast PHY datapath is
the MAC's internal TX clock-domain-crossing FIFO — and that is a **plain async
FIFO, not a store-and-forward buffer**: it starts draining as soon as it holds a
single word, it does not wait for a whole frame.

Consequences:

- The 50 MHz producer cannot supply one octet per 8 ns. After the ~8-cycle
  preamble prefill, the CDC underruns within a handful of bytes.
- When the CDC underruns, `valid` drops mid-frame. The RGMII TX simply mirrors
  `valid` onto `TX_CTL`, so `TX_CTL`/`RX_DV` **deasserts in the middle of the
  frame**. The frame is truncated / gapped on the wire.
- The MAC still computes the *correct* FCS value even with bubbles (the CRC
  engine's clock-enable is gated on `valid`), so only the byte *timing* is
  wrong. But a truncated frame with a mid-frame `RX_CTL` gap no longer matches
  its appended FCS, so the far end drops it as a CRC/alignment error.

The tell was already in the original config: `rx_cdc_depth=2048` but
`tx_cdc_depth=64`. RX (125 MHz producer -> 50 MHz consumer) was correctly
over-buffered; TX is the mirror case (fast consumer, slow producer) and **no
finite streaming FIFO fixes a sustained rate deficit** once draining has begun.

## What did not work: run the whole stack at line rate

The obvious fix is to clock the MAC/IP/UDP stack at 125 MHz so it can keep the
TX datapath fed. On this part (ECP5 `LFE5UM-45`, speed grade -8, already full of
the PCIe SerDes core) it does not close timing — not even close.

Post-route `nextpnr` result with the stack on a 125 MHz PLL clock:

| Clock | Required | Achieved | Result |
|---|---|---|---|
| `eth_clocks0_rx` (gigabit PHY datapath) | 125 MHz | 122 MHz | ok (baseline) |
| `crg_clkout` (MAC/IP/UDP stack) | **125 MHz** | **74.6 MHz** | **FAIL** |

The critical path is structural, not a stray net: a ~13 ns combinational cloud
spanning `ip_rx` IPv4 checksum -> ICMP echo FIFO -> ICMP/IP/MAC depacketizer
FSMs. The LiteEth IP/ICMP stack simply tops out around 75 MHz here, a ~1.7x gap
from 125 MHz, and no seed/pipelining tweak bridges that.

So the stack must stay slow. Since 50 MHz underruns and 125 MHz won't build,
clock choice alone cannot solve it.

## The fix: store-and-forward `PacketFIFO` in `eth_tx`

Keep the stack at 50 MHz and **decouple it from the line rate with a
store-and-forward buffer in the fast (`eth_tx`, 125 MHz) domain**, right before
the RGMII PHY. Because the MAC already computes the correct FCS value even when
the stream has bubbles, we only need to fix the *timing* of the bytes on the
wire. A `PacketFIFO` buffers each complete, already-formed frame (preamble +
data + FCS) and replays it to the PHY **gaplessly**, so `RX_CTL` stays asserted
for the whole frame and the FCS matches again.

This is implemented as a SoC-level PHY shim, `_TXStoreForwardPHY`, that wraps
the real RGMII PHY (no LiteEth edits):

```
MAC TX (50 MHz stack -> eth_tx) --> [ PacketFIFO (store-and-forward) ] --> [ Gap ] --> RGMII PHY
RGMII PHY RX --------------------------------------------------------------------------> MAC RX  (unchanged)
```

- A `PacketFIFO` only presents a packet once its `last` has been enqueued, so a
  full frame is buffered before it starts draining at 125 MHz — exactly the
  store-and-forward semantics the MAC's async CDC lacks.
- A `Gap` stage after it restores the inter-frame gap that the FIFO strips
  (the FIFO only stores `valid` beats, so queued frames would otherwise go out
  back-to-back).
- RX passes straight through, untouched.

### Getting it to close timing

Dropping a `PacketFIFO` in naively *did* fix the underrun but regressed the
gigabit `eth_tx` domain below the 125 MHz line rate. It took a few iterations,
each removing a path the buffer introduced into the `eth_tx` domain:

| Attempt | `eth_clocks0_rx` (needs ~125) | stack (`crg_clkout`, @50) |
|---|---|---|
| Stack @ 125 MHz (no FIFO) | 122 ok, but stack **74.6 FAIL** | 74.6 FAIL |
| Naive `PacketFIFO` | 90.7 — `ready` chain | 78 ok |
| `+ buffered=True` (block RAM) | 99.0 — async-RAM mux removed | 83 ok |
| `+ skid Buffer` (valid+ready) | 116.0 | 72 ok |
| **Tie MAC-facing `ready` = 1** | **123.5** | **71 ok** |

The lessons, in order:

1. **`buffered=True`.** A deep (2048) non-buffered FIFO infers *distributed LUT
   RAM with an async read mux*, and that huge mux landed directly on the RGMII
   TX DDR output. `buffered=True` makes it a synchronous-read **block RAM** with
   a registered output.
2. **The `ready` chain.** The RGMII PHY always held `ready` high, so the MAC's
   internal `cdc -> CRC -> preamble -> gap` combinational `ready` chain
   terminated at a *constant* and was never a real timing path. Any buffer that
   drives that chain from a real register lights the whole chain up as the
   `eth_tx` critical path. Registering `ready` (a skid buffer) helped but did
   not fully erase it.
3. **Tie the MAC-facing `ready` high.** The clean fix is to preserve the PHY's
   constant-ready boundary: tie the MAC-facing `ready` to `1` and let the FIFO
   snoop the stream. This is safe because the FIFO is far deeper than a max
   frame, only ever takes one byte per cycle, and drains at line rate, so it
   never fills. The gigabit domain returns to **123.5 MHz** — its original
   baseline — with the native LiteEth preamble/CDC path as the limiter, and the
   shim entirely off the critical path.

## Result

`ping 192.168.1.50` succeeds, and the ARP entry resolves and goes `REACHABLE`
(the host/switch now accept the board's ARP reply, i.e. a valid FCS):

```
5 packets transmitted, 5 received, 0% packet loss
rtt min/avg/max/mdev = 0.238/0.269/0.362/0.047 ms

192.168.1.50 dev enp0s31f6 lladdr 10:e2:d5:00:00:01 REACHABLE
```

## Caveats

- `eth_clocks0_rx` reports **123.5 MHz vs a 125 MHz constraint** — a ~1% miss,
  the same as the original working design. It is absorbed by `--timing-allow-fail`,
  the DDR I/O margin, and the tunable TX-clock skew (`eth_tx_skew`, a run-time
  `DELAYF` on the RGMII TX clock over UARTBone). If the link is ever flaky, step
  that skew.
- `clk100` (PCIe/`sys`) reports **~61 MHz, FAIL at 100 MHz**. This is
  **pre-existing** (it failed in every build, and is noted in the source), is
  unrelated to the Ethernet work, and PCIe runs on it under `--timing-allow-fail`.
- The bitstream is loaded into **SRAM** (volatile); it is lost on power cycle.
  Use `--flash` to persist it.

## Build / load

```sh
./venv/bin/python soc_ecp5_udp.py --device LFE5UM --build --seed 24
openFPGALoader -c ft2232 \
  build/soc_ecp5_udp_lfe5um_gen1_ref156/gateware/soc_ecp5_udp_lfe5um_gen1_ref156.bit
ping 192.168.1.50
```
