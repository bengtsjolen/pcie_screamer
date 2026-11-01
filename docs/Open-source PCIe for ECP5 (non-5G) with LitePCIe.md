# Open-source PCIe for ECP5 (non-5G) with LitePCIe

Nov 1, 2026 · @Bengt

## Summary

Start from ECP5-PCIe, not Yumewatari. ECP5-PCIe went well past its 2020 GSoC report before stalling in May 2023: it has an LTSSM, a DLL with InitFC/UpdateFC, LCRC, sequence numbers, a replay buffer, ACK/NAK and a config-space responder.

What's missing for LitePCIe is a PHY wrapper that exposes LitePCIe's TLP-stream contract, plus a proper Type 0 config space, MSI and TX credit gating. No Migen translation is needed to get a first link.

## What LitePCIe expects from a PHY

LitePCIe is agnostic below the TLP layer. Every existing PHY (`s7pciephy.py`, `gw5apciephy.py`, `lfcpnxpciephy.py`) exposes the same surface, and an `ECP5PCIEPHY` must match it.

| Member | Meaning |
| --- | --- |
| `sink` / `source` | `stream.Endpoint(phy_layout(dw))`: `dat` + `be` with `first`/`last`. Bare TLPs (header + payload), no STP/seq/LCRC/END. |
| `data_width` | 64 is natural for Gen1 x1 |
| `id` | 16-bit completer ID (bus/dev/fn), captured from config writes |
| `max_payload_size`, `max_request_size` | Decoded from Device Control; Gowin and CertusPro PHYs hard-wire 128 |
| `bar0_mask` | From `get_bar_mask(bar0_size)`, used by the depacketizer for address masking |
| `msi` | `stream.Endpoint(msi_layout())`; the PHY turns it into an interrupt |
| `endianness`, `qword_aligned` | Class attributes for dword ordering and 3DW-header packing |
| `_link_status` etc. | CSRs; optional, but read by the software side |

The Xilinx S7 hard block gives LitePCIe three things for free that must be supplied here: it answers all config TLPs, generates MSI from `cfg_interrupt`, and gates TX on flow-control credits. LitePCIe's depacketizer routes only MRd32/MWr32/Cpl/CplD, plus CfgRd0/CfgWr0 when `with_configuration` is set (a root-port feature); everything else is dropped.

## What exists, layer by layer

ECP5-PCIe covers every layer up to config completions; Yumewatari stops at a partial LTSSM.

| Layer | [Yumewatari](https://github.com/whitequark/Yumewatari) (Migen) | [ECP5-PCIe](https://github.com/ECP5-PCIe/ECP5-PCIe) (Amaranth, last commit May 2023) |
| --- | --- | --- |
| DCU wrapper | `CH0_PROTOCOL="PCIE"`, CTC bypassed, 1:2 gear | PCIe mode with CTC enabled (hard SKP compensation), 8b10b in the PCS, x1/x2/x4 fabric gearing, SCI access, `speed_5GTps=False` path |
| Rx detect | FSM from TN1261 | Same, reworked |
| PHY RX/TX, scrambler | Yes | Yes |
| LTSSM | Partial | Detect, Polling, Config, L0, Recovery, Gen2 speed change (`support_5GTps`) |
| DLLP, CRC16 | No | Yes |
| DLL control | No | DL\_Inactive to InitFC1 to InitFC2 to Active; UpdateFC parsing; Ack/Nak receive and schedule |
| TLP framing | No | STP/seq/LCRC/END/EDB, nullify, replay buffer, replay timer, REPLAY\_NUM |
| Config space | No | Plain 4 KiB RAM with VID/DID and a PCIe capability; completions work |
| Memory TLPs | No | Header decode only, not handled |

Yumewatari's only advantage is that it is already Migen, and its CTC-bypass DCU configuration is the weaker one.

## Gaps between `ecp5_pcie.PCIePhy` and a LitePCIe PHY

Seven pieces of work separate ECP5-PCIe from a LitePCIe-compatible PHY; config space and DLL robustness are the largest.

1. **Stream adapter.** The DLL's TLP side is a byte-symbol stream (`StreamInterface(9, ratio)`, 4 symbols/cycle in the `rx` domain). LitePCIe wants 64-bit `dat`/`be` with `first`/`last` in `sys`. About 200–300 lines, very testable in sim.
   - Byte-to-dword packing with the right endianness; `dll_tlp.py` has several `# TODO: Endianness correct?` comments.
   - `be` on the last beat, computed from the header Length field.
   - An async FIFO in each direction.
2. **Config space, rewritten.** The current one is a RAM where every byte is writable. The Migen rewrite is the piece most worth owning.
   - BAR0 with size-mask semantics (low bits read 0 after the host writes all 1s), so `bar0_size` works.
   - Command register: Memory Space Enable and Bus Master Enable. LitePCIe DMA must not master before BME is set.
   - Read-only masking for ID/class/capability fields; RW1C on status registers.
   - MSI capability (address/data/enable) chained in the capability list. It does not exist today.
   - Device Control decode into `max_payload_size`/`max_request_size`.
   - Bus/device number capture from CfgWr0 into `phy.id`.
   - UR completions for CfgRd1/CfgWr1 and for MRd outside BAR0, or the host sees completion timeouts.
3. **TX/RX demux and mux.** RX: config TLPs to the config block, Msg TLPs dropped, the rest to LitePCIe. TX: arbitrate config completions, the MSI MemWr (built from the MSI cap address/data when `phy.msi` fires) and LitePCIe's packetizer output.
4. **TX credit gating.** `dll.credits_rx` is parsed from UpdateFC, but `dll_tlp.py` never checks it before sending, and `dll.py` has `sending_tlp = Signal() # TODO: Connect this`. Add consumed/limit counters for P/NP/Cpl headers and data and stall the packetizer when out of credit. Hosts usually advertise plenty, but DMA writes will eventually violate it.
5. **DLL robustness.**
   - The replay buffer holds `2**len(replay_num)` = 4 TLPs, which throttles DMA bursts hard. It is also wrongly coupled to REPLAY\_NUM.
   - Replay timeout is `462 # TODO: is 462 right` and counts `rx` cycles, not symbol times, so with multi-symbol gearing it is off by the gearing ratio. Check it and the AckNak latency timer against PCIe 1.1 Table 3-4 (MPS 128, x1).
   - Advertised RX credits are tiny (PH=2, PD=0x20, NPH=2) and need sizing against a real RX FIFO for MPS 128/256.
   - The ACK path pops the unacked FIFO without checking the sequence number (its own TODO); fine until a NAK arrives mid-burst.
6. **Non-5G configuration.** Construct with `speed_5GTps=False` and `support_5GTps=False` on the LTSSM, or it advertises 5 GT/s in TS1/TS2 and the host attempts a speed change through Recovery. Check `D_TX_MAX_RATE "2.5"` and the CMU/PLL values for a 100 MHz refclk: the DCU is the same block in Trellis for LFE5UM and LFE5UM5G, but the defaults came from 5G sample code.
7. **Reset plumbing.** PERST# resets the LTSSM and DCU. Link-down or hot reset resets the DLL (already there via `ResetInserter(~dll.up)`). DLL-down must then propagate into LitePCIe's reset, including flushing DMA.

## Migen translation: not at first

Instantiate ECP5-PCIe as generated Verilog first; translate to Migen layer by layer once it enumerates.

ECP5-PCIe needs Amaranth 0.4 or older. It uses `Record`, old-style `Memory` and `amaranth.hdl.ast.Part`, and its `setup.py` leaves `amaranth` unpinned, so a current install breaks.

**Quickest route:** pin Amaranth 0.4 in a venv and emit Verilog of `PCIePhy`, DCU instance included. In LiteX, `platform.add_source()` it and `Instance()` it inside `ECP5PCIEPHY`, exporting `rx_clk`/`tx_clk` and the TLP symbol streams as ports. Yosys handles the mixed sources and nextpnr sees one design.

**Later translation** covers about 3.5 kLoC: `ltssm.py` 796, `ecp5_serdes.py` 684, `dll_tlp.py` 529, `serdes.py` 394, `phy_rx`/`phy_tx` 373, `dll.py` 331, `crc.py` 239, `dllp.py` 172, and part of `memory.py`. The mapping is mechanical:

- `m.d.rx +=` to `self.sync.rx +=`
- `with m.If` to `If().Elif()`
- `m.FSM` to `FSM()` with `NextState`/`NextValue`
- `FFSynchronizer` to `MultiReg`
- `Record` to Migen `Record`

The traps are deep `with m.If` nesting becoming nested `If(...)` expressions, `Value.shift_left`/`Part`, and differing Memory port transparency defaults. Port ECP5-PCIe's sim tests alongside each layer so it is checked against the original.

## Versa (LFE5UM-45F) specifics

The board support already exists in litex-boards; the refclk path is the main hardware trap.

- `lattice_versa_ecp5.py` accepts `device="LFE5UM"` and defines `pcie_x1` (DCU0 CH0: rx Y5/Y6, tx W4/W5, refclk Y11/Y12, `perst` A6) plus `refclk_en`/`refclk_rst_n` for the onboard ispCLOCK.
- The slot refclk reaches the DCU through the ispCLOCK, which only selects it when PRSNT is grounded. The GSoC work found a host x16 slot that didn't ground the PRSNT pins and needed the J4 header jumpered. Check this before debugging the LTSSM.
- Bandwidth: Gen1 x1 is about 200 MB/s after 8b10b and before TLP overhead. A 64-bit LitePCIe datapath at sys of 50 MHz or more has margin. The DCU `rx`/`tx` domains run at 62.5 MHz with 4-symbol gearing, which closes comfortably on a -8.

## Suggested order and effort

Steps 1–4 are a few weeks for someone fluent in both HDLs, mostly config space and bring-up; step 5 is where DLL corner cases eat time.

1. **Sim first.** ECP5-PCIe's `virtual_serdes.py` and `PCIeVirtualTLPGenerator` (`upstream=False`) give a root model against the endpoint. Get the stream adapter and new config space passing there.
2. **Hardware: L0 and DL\_Active** on the non-5G Versa via the Verilog-instance route, with LTSSM and DLL state on CSRs over UARTBone.
3. **Enumerate.** `lspci -vv` shows BAR0 and the MSI cap. Use a VID/DID the `litepcie` kernel driver already matches, or add yours to its table.
4. **LitePCIe MMIO** through the BAR0 CSR bridge; `litepcie_util info` should work.
5. **DMA.** Needs credit gating and the larger replay buffer (gaps 4 and 5); expect replay/ACK bugs to surface here.
6. **Translate to Migen** if still wanted.

## Sources

- [litepcie (enjoy-digital)](https://github.com/enjoy-digital/litepcie)
- [ECP5-PCIe](https://github.com/ECP5-PCIe/ECP5-PCIe) and its [GSoC development report](https://ecp5-pcie.github.io/gsoc/2020/09/23/development-report.html)
- [Yumewatari (whitequark)](https://github.com/whitequark/Yumewatari)
- [litex-boards](https://github.com/litex-hub/litex-boards)
