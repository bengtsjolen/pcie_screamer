#!/usr/bin/env python3
"""Drive the ECP5 endpoint's host-memory DMA engine over UARTBone (the `--litepcie` build).

The FPGA issues real PCIe memory reads/writes to arbitrary host physical addresses - the
pcileech-shaped primitive. Bus Master Enable must be set by the host (it is once the OS has
enumerated the device); use --force to override the gate for bring-up.

Run the bridge first, e.g.:
    venv/bin/litex_server --uart --uart-port /dev/serial/by-id/usb-Lattice_..._if01-port0 \
        --uart-baudrate 115200

Examples:
    tools/ecp5_dma.py --csr-csv build/.../csr.csv read  0x00000000e6600000
    tools/ecp5_dma.py --csr-csv build/.../csr.csv write 0x00000000e6600000 0xdeadbeef
    tools/ecp5_dma.py --csr-csv build/.../csr.csv dump  0x00000000e6600000 16
"""

import argparse
import sys
import time

from litex import RemoteClient


def _wait_idle(bus, timeout=1.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        st = bus.regs.pcie_dma_status.read()
        if not (st & 0x1):          # busy
            return st
    raise SystemExit("DMA engine stayed busy (no completion?); check link/BME")


def _prep(bus, addr, force):
    bus.regs.pcie_dma_addr_lo.write(addr & 0xffffffff)
    bus.regs.pcie_dma_addr_hi.write((addr >> 32) & 0xffffffff)


def dma_write(bus, addr, value, force=False):
    _prep(bus, addr, force)
    bus.regs.pcie_dma_wdata.write(value & 0xffffffff)
    bus.regs.pcie_dma_ctrl.write(0x1 | (0x2 if force else 0x0))   # we=1
    bus.regs.pcie_dma_start.write(1)
    st = _wait_idle(bus)
    return st


def dma_read(bus, addr, force=False):
    _prep(bus, addr, force)
    bus.regs.pcie_dma_ctrl.write(0x0 | (0x2 if force else 0x0))   # we=0
    bus.regs.pcie_dma_start.write(1)
    st = _wait_idle(bus)
    return bus.regs.pcie_dma_rdata.read(), st


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("command", choices=["read", "write", "dump", "status"])
    p.add_argument("address", nargs="?", default="0", help="Host physical address (hex ok)")
    p.add_argument("value", nargs="?", help="Write data / dword count for dump (hex ok)")
    p.add_argument("--csr-csv", required=True)
    p.add_argument("--host", default="localhost")
    p.add_argument("--port", type=int, default=1234)
    p.add_argument("--force", action="store_true", help="Issue even if Bus Master Enable is clear")
    args = p.parse_args()

    addr = int(args.address, 0)
    bus = RemoteClient(host=args.host, port=args.port, csr_csv=args.csr_csv)
    bus.open()
    try:
        bme = (bus.regs.pcie_dma_status.read() >> 3) & 1
        if args.command == "status":
            st = bus.regs.pcie_dma_status.read()
            print(f"busy={st & 1} done={(st >> 1) & 1} err={(st >> 2) & 1} bus_master={bme}")
            return
        if not bme and not args.force:
            print("warning: Bus Master Enable is clear; the request will be dropped. Use --force.",
                  file=sys.stderr)
        if args.command == "write":
            value = int(args.value, 0)
            st = dma_write(bus, addr, value, args.force)
            print(f"write 0x{value:08x} -> 0x{addr:016x}  (err={(st >> 2) & 1})")
        elif args.command == "read":
            data, st = dma_read(bus, addr, args.force)
            print(f"0x{addr:016x}: 0x{data:08x}  (err={(st >> 2) & 1})")
        elif args.command == "dump":
            count = int(args.value, 0) if args.value else 16
            for i in range(count):
                a = addr + 4 * i
                data, st = dma_read(bus, a, args.force)
                flag = " ERR" if (st >> 2) & 1 else ""
                print(f"0x{a:016x}: 0x{data:08x}{flag}")
    finally:
        bus.close()


if __name__ == "__main__":
    main()
