# Running a LitePCIe example on the ECP5

Date: 2026-11-23

This is what is still missing before a stock LitePCIe example device can
run on the ECP5 Versa. [DEPENDENCIES.md](DEPENDENCIES.md) maps where the
PHY sits. [VARIANTS.md](VARIANTS.md) and [PERFORMANCE.md](PERFORMANCE.md)
record the images that have actually linked.

## What an example is

The files in `litex/litepcie/examples/` and the PCIe targets in
`litex/litex-boards` are not bitstreams that can be retargeted. Each one
builds a vendor PHY (`S7PCIEPHY`, `USPCIEPHY`, `USPPCIEPHY`, `GW5APCIEPHY`,
`C5PCIEPHY`, or `LFCPNXPCIEPHY`) and then calls `SoC.add_pcie` in
`litex/soc/integration/soc.py`. That call is the device:

- `LitePCIeEndpoint` on `phy.sink` / `phy.source`, with `endianness` taken
  from `phy.endianness`.
- A BAR0 wishbone master (`LitePCIeWishboneMaster`) onto the SoC bus, so
  the host can read and write CSRs.
- `LitePCIeDMA` channels, reader and writer, with a descriptor table and
  optional loopback.
- MSI. `add_pcie` asserts `with_msi` whenever `ndmas > 0`, and it connects
  `msi.source` to `phy.msi`.

`litepcie/gen.py` refuses any other PHY. Its comment lists 7-series,
UltraScale, Cyclone V, GW5AT, and CertusPro-NX. An ECP5 example is a new
Versa target that calls `add_pcie` with `ECP5PCIePHY`. It is not
`examples/acorn.yml` with the device name changed. The Acorn YAML asks
for a 128-bit x4 PHY, 8 outstanding requests, and 16 MSI vectors.

## What already lines up

`ECP5PCIePHY` presents the stream ports `LitePCIeEndpoint` uses:
`sink` and `source` in `phy_layout(64)`, `data_width = 64`, `bar0_size`,
`bar0_mask`, `id`, `max_request_size`, `max_payload_size`, and `cd_pcie`
following `sys`. Dword order matches `S7PCIEPHY`, whose `endianness` is
`"big"`. `soc_ecp5.py --litepcie` passes that endianness by hand.

`TLPSysRouter` (`gateware/ecp5pcie/sys_router.py`) already sorts the TLP
types a normal example uses:

| Fmt/Type | Treatment |
|---|---|
| `MRd32` `0x00`, `MWr32` `0x40`, `Cpl` `0x0a`, `CplD` `0x4a` | Forwarded to the LitePCIe endpoint |
| `CfgRd0`, `CfgWr0` | Answered from `ConfigSpace` |
| `MRd64`, IO read/write, locked reads, type-1 config | Unsupported Request completion |
| `MWr64`, messages, anything else | Dropped |

`soc_ecp5.py --litepcie` is the closest image that elaborates today. It
builds the endpoint with `address_width=64`, a private BAR0 SRAM, and
`HostDMAScratch` (one dword to or from a host physical address, gated on
Bus Master Enable). It does not call `add_pcie`, has no `LitePCIeDMA`
table, and has no MSI. The BAR0 write-then-read of that image passed in
simulation. The Versa images that have been flashed are the search path,
which never instantiates `LitePCIeEndpoint`. Those measured
endpoint-originated 32-bit reads only.

## Before `add_pcie` will elaborate

Two attributes on the PHY.

`phy.endianness` has to be `"big"`, as a class attribute the way
`S7PCIEPHY` defines it. `add_pcie` reads `phy.endianness` and does not
take it as an argument.

`phy.msi` has to be a `stream.Endpoint(msi_layout())`. That layout is a
single `dat` field of 8 bits. On the 7-series PHY this stream is wired
to the hard IP: `valid` to `cfg_interrupt`, `ready` from
`cfg_interrupt_rdy`, `dat` to `cfg_interrupt_di`. The hard IP owns the
MSI capability and turns that strobe into a one-dword memory write using
the address and data the host programmed. `_msi_enable` and
`_msix_enable` on that PHY are live status from the same block. On
`ECP5PCIePHY` both CSRs are tied to 0.

## MSI

This is the piece that keeps the example driver from running, separate
from the memory TLP types.

`ConfigSpace` (`gateware/ecp5pcie/config_space.py`) has a PCI power
capability at `0x40` and a PCIe capability at `0x60`. It has no MSI
capability and no MSI-X capability, so the host has nowhere to set the
MSI enable bit or to write the message address and data.

Classic MSI, which is what `add_pcie` connects to `phy.msi`, needs:

- An MSI capability in config space. One vector is enough for a first
  target. The 7-series example IP is configured as 64-bit incapable and
  one vector, with a comment that multi-vector is unfinished there too.
- `_msi_enable` reflecting that capability's enable bit, so software and
  the DMA block can see that the host turned MSI on.
- A transmitter that, on `phy.msi.valid`, emits one posted `MWr32` of
  one dword to the programmed message address, with the programmed
  message data, and then sets `phy.msi.ready`. The transmit path of
  `TLPSysRouter` already merges `phy_sink` without looking at the format,
  so this write can enter through the same port as any other LitePCIe
  request. It has to wait until Bus Master Enable is set.

MSI-X is the other `msi_type` `add_pcie` accepts. `LitePCIeMSIX` does not
use `phy.msi`. It sends the same one-dword memory write through a
crossbar master port, which this router already forwards. It still needs
an MSI-X capability whose table and PBA live in a BAR the host can write.
That is more config space than a first example needs. One MSI vector
unblocks `ndmas=1`.

INTx is unused by these examples. The 7-series PHY sets legacy interrupt
generation off. Message TLPs that arrive from the host are dropped by
the router. Nothing has to be added for them.

## Memory transfers the example actually issues

Host access to BAR0 is `MRd32` and `MWr32`, because the BAR we advertise
is 32-bit. Both formats are forwarded. A host write of BAR0 has been
simulated and has not been done on the Versa. A host read of BAR0 through
`LitePCIeWishboneMaster` has not been done on the Versa either. Config
reads and writes have, which is how the board enumerates.

`LitePCIeDMA` with `address_width=64` emits `MRd32` or `MWr32` when the
host address fits in 32 bits, and `MRd64` or `MWr64` otherwise. Those
requests leave through `phy.sink`, which the router does not filter, so
a DMA descriptor aimed above 4 GiB can go out on the wire. The completion
comes back as `Cpl` or `CplD` and is forwarded. Inbound `MRd64` is
answered with Unsupported Request, and inbound `MWr64` is dropped. That
matters only if a future BAR is 64-bit. The example BAR is not.

The DMA reader depends on a completion with a non-zero status freeing
its tag. `litex/litepcie` commit `33defbe` does that in
`litepcie/tlp/depacketizer.py` by treating `status != 0` as the end of
the completion. That commit is local to this checkout. An example built
against upstream LitePCIe stalls the reader on the first Unsupported
Request. `HostDMAScratch` has its own timeout for the same case and
pulses `crossbar.tlp_ctrl_rst`. `LitePCIeDMA` does not.

IO, locked transactions, and type-1 config are out of the examples.
Leaving them as Unsupported Request is the right behavior for this
endpoint.

## Sizes to turn down

`add_pcie` defaults to `max_pending_requests=8` and the Acorn YAML asks
for the same. The ECP5 DLL retry buffer holds 4 TLPs
(`TLPBuffer` in the transmit path of `dll_tlp.py`, `max_tlps` from a
2-bit replay count). The receive buffer also holds 4 and drops a TLP
that does not fit. A Versa example should pass `max_pending_requests=4`
until that buffer is larger. The search image was measured with 4 reads
in flight, on `ReadBench` rather than on `LitePCIeDMA`.

The DLL advertises 2 non-posted header credits and 2 posted header
credits at init (`dll.py`, `NPH` and `PH`), plus credits released as
TLPs are retired. Enumeration works with that. A host that pipelines
many BAR0 reads will stall until a credit comes back. The example's
wishbone master issues one request at a time, so this does not block it.

Data width stays 64. The endpoint and the DMA are parameterized by
`phy.data_width`. The 128-bit width in the example YAML is the Xilinx
user-clock datapath, not a requirement of the DMA.

`ConfigSpace` advertises a maximum payload of 256 bytes and resets
Device Control with a maximum read request of 512. `max_payload_size`
and `max_request_size` on the PHY follow Device Control, and LitePCIe
splits DMA on those values. One such TLP fits in the 512-byte buffer
slot. The example does not need a larger payload.

BAR0 in `soc_ecp5.py` defaults to 4 KiB. An example that maps the CSR
bus plus a DMA descriptor window through that BAR needs a larger BAR, or
a second BAR. Config space currently implements only BAR0.

## A minimal Versa example

One new target, not a change to `gen.py`. It would:

1. Build `ECP5PCIePHY` for Gen1, 156.25 MHz reference, the same arguments
   as `soc_ecp5_search.py`.
2. Set `endianness = "big"` and provide `phy.msi`.
3. Add a one-vector MSI capability and the memory-write transmitter
   described above.
4. Call `self.add_pcie(phy=self.pcie_phy, ndmas=1, address_width=64,
   max_pending_requests=4)` so the host gets the CSR window, one DMA
   reader, one DMA writer, and one MSI.
5. Keep the local depacketizer fix.

The first hardware checks, after enumeration, are a host write and read
of a CSR through BAR0, a DMA write of one buffer below 4 GiB, a DMA read
of one buffer, and one MSI delivered for each. A DMA aimed above 4 GiB
is the `MWr64` / `MRd64` check. The 1 GiB bandwidth number in
[PERFORMANCE.md](PERFORMANCE.md) does not transfer: that path is
`ReadBench` with four outstanding 4 KiB reads, and `LitePCIeDMA` splits
on the negotiated maximum read request and posts an interrupt per
descriptor.

## Left out of that example

Gen2 and the `LFE5UM5G` part. The speed-change state in the LTSSM does
not leave `Recovery.Speed`, and this Versa is the non-5G silicon.
[DEPENDENCIES.md](DEPENDENCIES.md) covers that.

PTM. `add_pcie` can pass `with_ptm=True`. The default examples do not.
There is no PTM capability here.

A root port. The core is an endpoint, and `TLPSysRouter` answers type-0
config only.

More than one lane, a 64-bit BAR, MSI-X, and INTx. None of these are
required to run the one-DMA example above.
