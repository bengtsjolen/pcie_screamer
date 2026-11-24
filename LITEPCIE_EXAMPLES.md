# LitePCIe on the ECP5 Versa

Date: 2026-11-23

`soc_ecp5_litepcie.py` is a stock LitePCIe device on the non-5G Versa
(LFE5UM-45F): one BAR0 CSR window, one DMA reader, one DMA writer, and
one 32-bit MSI vector. It calls `SoC.add_pcie`. It is not a retarget of
`litepcie/gen.py`. The 196.7 MiB/s figure in
[PERFORMANCE.md](PERFORMANCE.md) is `ReadBench` on `soc_ecp5_search.py`,
a different engine.

## What runs

Measured on Rechner2 on 2026-11-23. Root port `0000:ac:00.0`, endpoint
`0000:ad:00.0`. The bitstream is seed 24, loaded into SRAM.

* The endpoint enumerates as `10ee:0667` revision 0. BAR0 is 64 KiB.
  Link status is 2.5 GT/s, x1. Vendor `10ee` is the Xilinx id, the same
  one the Squirrel uses for device `0666`. Device `0667` is a second id
  so both cards can be in one host. A rescan only brings back an id the
  host has already seen, so this id stays. It is not a Lattice id.
  The LitePCIe driver does not list it. After `insmod`, the build's
  printed line binds the id the bitstream actually contains:

  ```sh
  echo 10ee 0667 | sudo tee /sys/bus/pci/drivers/litepcie/new_id
  ```
* The MSI capability is at `0x48`: one vector, 32-bit, not maskable.
  After the driver binds, `lspci` shows `MSI: Enable+`.
* BAR0 CSRs work. `litepcie_util info` prints the identifier
  (`pcie_screamer ECP5 LitePCIe` plus the build time; the 2026-11-23
  image printed `pcie_screamer ECP5 LitePCIe example`).
  `litepcie_util scratch_test` writes and reads `0x12345678` and
  `0xdeadbeef`.
* On-chip DMA loopback works. `sudo ./litepcie_util -t 2 dma_test`
  keeps both counts moving, holds the difference at 128, and reports
  0 data errors. The speed column stays at **0.38 Gbps**. That is the
  reader, about 47.5 MB/s of the 8 KiB buffers. The writer sends those
  same buffers back to the host, so the link carries both directions.
  128 is half of the 256-buffer ring; the driver keeps the reader that
  far ahead of the writer.

Post-route, the SerDes clocks meet their constraints. `clk100` does not:
this image's fmax is 74.96 MHz against the 100 MHz system clock. The
loopback above passed on that image. Seeds 1 and 12 do not place the
DCU for this netlist. Seed 24 does.

The load is SRAM. A PCIe rescan keeps the image, because PERST# is fabric
pin A6 and is not PROGRAMN. A Versa power cycle drops it. SPI flash has
not been programmed.

## What this image does not do

* Close timing at 100 MHz.
* Gate transmits on flow-control credits. The adapter also waits until
  the DLL retry buffer is empty, so one TLP is unacknowledged at a time.
  Four read tags may still be outstanding, because a header-only memory
  read is acknowledged before its completion returns.
  `max_pending_requests=4` is that tag limit.
* Exercise an address above 4 GiB. The DMA is built with
  `address_width=64` and emits a 32-bit request when the host address
  fits in 32 bits. `MRd64` / `MWr64` were not run. An inbound `MRd64`
  gets Unsupported Request. An inbound `MWr64` is dropped.
* External loopback (`litepcie_util -e`). That flag turns the on-chip
  loopback off. It was not re-run after the MSI fix.
* Gen2, the LFE5UM5G part, MSI-X, INTx, more than one lane, a 64-bit
  BAR, PTM, or a root port.

A one-dword Memory Write must use Last DW Byte Enable 0 and First DW
Byte Enable `0xf`. The MSI header does that (`0x0f`). `0xff` is a
malformed TLP: the DMA engines still advance, and the host never sees
the interrupt, so `dma_test` freezes with `TX_BUFFERS` at 128.

## Build the bitstream

From the repo root, with the Yosys/nextpnr/ecppack binaries on `PATH`
or under `tools/bin`, and the LiteX checkout importable (this tree uses
`./venv`):

```sh
./venv/bin/python soc_ecp5_litepcie.py --device LFE5UM --build --seed 24
```

The build prints the PCIe id from the config space it elaborated, and
the `new_id` line for that id. The soc script
copies `litex/litepcie/litepcie/software` to
`build/soc_ecp5_litepcie_lfe5um_gen1_ref156/litepcie_software` and writes
`csr.h`, `soc.h`, and `mem.h` there.
`litex/litepcie` commit `33defbe` is also local: a completion with
`status != 0` ends the request, so the DMA reader frees the tag instead
of stalling.

`--seed 24` is required for this netlist. The build passes
`--timing-allow-fail`, so the bitstream is still written when `clk100`
misses timing.

## Load

JTAG is FT2232 channel A. This programs SRAM and does not write SPI
flash:

```sh
openFPGALoader --cable ft2232 --bitstream \
  build/soc_ecp5_litepcie_lfe5um_gen1_ref156/gateware/soc_ecp5_litepcie_lfe5um_gen1_ref156.bit
```

`soc_ecp5_litepcie.py --load` calls the same programmer. UART is FT2232
channel B, 115200 baud
(`/dev/serial/by-id/usb-Lattice_Lattice_ECP5_Versa_Board-if01-port0`
on the machine that has the USB cable).

## Host driver and tests

Build the module on the machine that will load it. A `.ko` built against
another kernel will not insert. The software tree can be the one the
bitstream build just wrote; the module itself is compiled on the host.

Bus numbers below are Rechner2. Use the endpoint `lspci` shows.

```sh
echo 1 | sudo tee /sys/bus/pci/devices/0000:ad:00.0/remove
echo 1 | sudo tee /sys/bus/pci/devices/0000:ac:00.0/rescan
sudo setpci -s ad:00.0 COMMAND=0x06
sudo lspci -s ad:00.0 -vvv | grep -E 'Control:|Region 0|MSI:|LnkSta:'
```

`Control` should show `Mem+` and `BusMaster+`. `Region 0` is a 64 KiB
memory BAR. Before the driver binds, MSI is `Enable-`. Link status
should be `2.5GT/s` and `x1`.

```sh
cd build/soc_ecp5_litepcie_lfe5um_gen1_ref156/litepcie_software/kernel
make
sudo rmmod litepcie 2>/dev/null || true
sudo insmod litepcie.ko
echo 10ee 0667 | sudo tee /sys/bus/pci/drivers/litepcie/new_id
cd ../user
make
sudo ./litepcie_util info
sudo ./litepcie_util scratch_test
sudo ./litepcie_util -t 2 dma_test
```

`insmod` loads the driver. `new_id` exists only after that, and writing
it probes a device already enumerated with this id. Use the vendor and
device the build printed if they differ from `10ee 0667`. `dmesg`
should show the identifier string,
`1 MSI IRQs allocated.`, and `/dev/litepcie0`. `kernel/init.sh` does
the same insert and then `chmod 666` on `/dev/litepcie0`, but only when
the module was not already loaded. After a later rescan the node often
comes back root-only, which is why the util is run with `sudo`.
`sudo chmod 666 /dev/litepcie0` makes it usable without `sudo`.

`info` reads the identifier ROM through BAR0. `scratch_test` writes the
scratch CSR. `dma_test` is the on-chip loopback. Both buffer columns
should keep climbing, the difference should stay at 128, and errors
should stay at 0.

## Device

`add_pcie` in `litex/soc/integration/soc.py` builds `LitePCIeEndpoint`
(`endianness` from `phy.endianness`, which is `"big"`), a BAR0 wishbone
master, `LitePCIeDMA`, and `LitePCIeMSI` wired to `phy.msi`.
`litepcie/gen.py` still refuses an ECP5 PHY. This example is the
LitePCIe image: endpoint, BAR0, one DMA channel, and MSI.

Config space is a Type 0 header. The capability chain is PCI PM at
`0x40`, MSI at `0x48`, PCIe capability at `0x60`. The MSI transmitter
in `TLPSysRouter` sends one posted `MWr32` when `phy.msi.valid` is set
and Bus Master Enable is set, then pulses `ready`. Message address and
data come from the capability. The requester id is the completer id
captured from the last `CfgWr0`.

`TLPSysRouter` forwards inbound `MRd32`, `MWr32`, `Cpl`, and `CplD`.
`CfgRd0` and `CfgWr0` are answered locally. Other unsupported
non-posted requests get Unsupported Request.
