#!/usr/bin/env python3
"""Read ECP5 PCIe bring-up CSRs over UARTBone.

The UARTBone image this was written against has been removed.
`soc_ecp5_litepcie.py` is the LitePCIe image, and its host path is the
kernel driver rather than this console.

Start the bridge first, e.g.:
    venv/bin/litex_server --uart --uart-port /dev/ttyUSB1 --uart-baudrate 115200
"""

import argparse
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "third_party" / "ECP5-PCIe" / "Gateware"))

from litex import RemoteClient

from ecp5_pcie.ltssm import State as LTSSMState
from ecp5_pcie.dll import State as DLLState


def _state_names(enum):
    # ltssm.State aliases substates to their parent value; keep the most specific name.
    names = {}
    for name, member in enum.__members__.items():
        if len(name) > len(names.get(member.value, "")):
            names[member.value] = name
    return names


LTSSM_NAMES = _state_names(LTSSMState)
DLL_NAMES   = _state_names(DLLState)

STATUS_BITS = ["link_up", "dll_up", "rx_present", "rx_locked", "rx_aligned",
               "tx_locked", "reset_done", "det_status", "perst_n"]

WORD_CLK_HZ = {1: 62.5e6, 2: 125e6}


def ltssm_name(v): return LTSSM_NAMES.get(v, f"?{v}")
def dll_name(v):   return DLL_NAMES.get(v, f"?{v}")


# CSR name prefixes: the bare core (`pcie_core_*`), or the `--litepcie` build where the core
# sits inside ECP5PCIePHY (`pcie_phy_phy_core_*`) and the router/config CSRs belong to the PHY.
CSR_PREFIXES = ("pcie_core_", "pcie_phy_phy_core_", "pcie_phy_phy_")


def csr(bus, reg):
    for prefix in CSR_PREFIXES:
        found = getattr(bus.regs, prefix + reg, None)
        if found is not None:
            return found
    return None


def read(bus, reg):
    found = csr(bus, reg)
    if found is None:
        raise SystemExit(f"CSR '{reg}' not found; does --csr-csv match the loaded bitstream?")
    return found.read()


def field(value, offset, size):
    return (value >> offset) & ((1 << size) - 1)


def show_status(bus):
    status = read(bus, "status")
    state  = read(bus, "state")
    dll    = read(bus, "dll")
    flags  = " ".join(f"{name}={field(status, i, 1)}" for i, name in enumerate(STATUS_BITS))
    print(f"status    : {flags}")
    print(f"ltssm     : {ltssm_name(field(state, 0, 8))}")
    print(f"dll       : {dll_name(field(state, 8, 8))}")
    for name in ("p", "np", "cpl"):
        fc = read(bus, f"fc_{name}")
        print(f"fc_{name:<6} : hdr={field(fc, 0, 8)} data={field(fc, 8, 12)}")
    print(f"seq       : tx={field(dll, 0, 12)} rx={field(dll, 12, 12)} "
          f"retry_occ={field(dll, 24, 4)} recv_occ={field(dll, 28, 4)}")
    print(f"errors    : lol={read(bus, 'rx_lol_errors')} sym={read(bus, 'rx_sym_errors')}")
    print(f"rx_tlps   : {read(bus, 'rx_tlps')}")
    print(f"clocks    : rx={read(bus, 'rx_freq_value')/1e6:.3f} MHz "
          f"tx={read(bus, 'tx_freq_value')/1e6:.3f} MHz")
    if csr(bus, "tl_cpl") is not None:
        counts = " ".join(f"{n}={read(bus, 'tl_' + n)}" for n in ("cfg_rd", "cfg_wr", "ur", "dropped", "cpl"))
        cfg = read(bus, "cfg")
        print(f"router    : {counts}")
        print(f"config    : command=0x{field(cfg, 0, 16):04x} id={field(cfg, 24, 8):02x}:"
              f"{field(cfg, 19, 5):02x}.{field(cfg, 16, 3)} bar0=0x{read(bus, 'bar0'):08x}")
    if csr(bus, "retry_debug") is not None:
        print(f"retry     : {describe_retry(read(bus, 'retry_debug'))}")


REPLAY_STATES = ["Idle", "Replay", "Replay-Start", "Replay-End"]


def describe_retry(dbg):
    slots = " ".join(f"{field(dbg, 34 + 12 * i, 12)}" if field(dbg, 30 + i, 1) else "-" for i in range(4))
    return (f"ackd={field(dbg, 0, 12)} fifo_level={field(dbg, 24, 3)} fifo_head="
            f"{field(dbg, 12, 12) if field(dbg, 27, 1) else '-'} replay={REPLAY_STATES[field(dbg, 28, 2)]} "
            f"slots=[{slots}]")


def describe_event(data):
    parts = []
    if data >> 31 & 1:
        parts.append(f"{'Ack' if data >> 30 & 1 else 'Nak'} {data & 0xfff}")
    if data >> 29 & 1:
        parts.append(f"replay of {data >> 12 & 0xfff}")
    if data >> 28 & 1:
        parts.append(f"replay of {data >> 12 & 0xfff} abandoned")
    return ", ".join(parts)


TLP_TYPES = {
    0x00: "MRd32", 0x20: "MRd64", 0x01: "MRdLk32", 0x21: "MRdLk64", 0x40: "MWr32", 0x60: "MWr64",
    0x02: "IORd", 0x42: "IOWr", 0x04: "CfgRd0", 0x44: "CfgWr0", 0x05: "CfgRd1", 0x45: "CfgWr1",
    0x0a: "Cpl", 0x4a: "CplD", 0x0b: "CplLk", 0x4b: "CplDLk",
}


def describe_tlp(data):
    """One-line summary of a TLP given its wire bytes."""
    if len(data) < 12:
        return "short"
    dw = [int.from_bytes(data[i:i + 4], "big") for i in range(0, len(data) - len(data) % 4, 4)]
    fmt_type = dw[0] >> 24
    name = TLP_TYPES.get(fmt_type, "Msg" if fmt_type & 0x38 == 0x30 else f"?{fmt_type:02x}")
    length = dw[0] & 0x3ff
    if name.startswith("Cfg"):
        reg = dw[2] & 0xffc
        payload = f" data=0x{int.from_bytes(data[12:16], 'little'):08x}" if fmt_type & 0x40 and len(data) >= 16 else ""
        return (f"{name} req={dw[1] >> 16:04x} tag={(dw[1] >> 8) & 0xff:02x} be={dw[1] & 0xf:x} "
                f"bdf={dw[2] >> 24:02x}:{(dw[2] >> 19) & 0x1f:02x}.{(dw[2] >> 16) & 7} reg=0x{reg:03x}{payload}")
    if name.startswith("Cpl"):
        status = ["SC", "UR", "CRS", "CA"][(dw[1] >> 13) & 3] if (dw[1] >> 13) & 7 < 4 else "?"
        payload = f" data=0x{int.from_bytes(data[12:16], 'little'):08x}" if fmt_type & 0x40 and len(data) >= 16 else ""
        return (f"{name} cpl={dw[1] >> 16:04x} {status} bc={dw[1] & 0xfff} req={dw[2] >> 16:04x} "
                f"tag={(dw[2] >> 8) & 0xff:02x}{payload}")
    return f"{name} len={length} " + " ".join(f"{d:08x}" for d in dw[1:4])


def drain_capture(bus, gen, limit=None):
    """Print captured TLPs from the TLP port capture FIFO."""
    clk = WORD_CLK_HZ[gen]
    if field(read(bus, "capture_level"), 1, 1):
        print("capture   : overflow, words were dropped")
    tlp, n = None, 0

    def flush():
        if tlp is not None:
            direction, t, data = tlp
            print(f"{t/clk*1e6:9.2f} us {direction} {describe_tlp(data):<70} {data.hex(' ')}")

    while (limit is None or n < limit) and field(read(bus, "capture_level"), 0, 1):
        entry = read(bus, "capture_data")
        csr(bus, "capture_pop").write(1)
        n += 1
        if field(entry, 32, 4) == 0:
            flush()
            tlp = None
            print(f"{field(entry, 38, 16)/clk*1e6:9.2f} us DL {describe_event(field(entry, 0, 32))}")
            continue
        word = field(entry, 0, 32).to_bytes(4, "little")[:bin(field(entry, 32, 4)).count("1")]
        direction = "TX" if field(entry, 36, 1) else "RX"
        if field(entry, 37, 1) or tlp is None or tlp[0] != direction:
            flush()
            tlp = (direction, field(entry, 38, 16), bytearray())
        tlp[2].extend(word)
    flush()
    return n


def drain_trace(bus, gen, limit=None):
    """Print queued trace entries, at most `limit` (the LTSSM can refill the FIFO as fast
    as UARTBone drains it)."""
    clk = WORD_CLK_HZ[gen]
    level = read(bus, "trace_level")
    if field(level, 1, 1):
        print("trace     : overflow, entries were dropped")
    n = 0
    while (limit is None or n < limit) and field(read(bus, "trace_level"), 0, 1):
        entry = read(bus, "trace_data")
        csr(bus, "trace_pop").write(1)
        t = field(entry, 0, 32)
        print(f"{t/clk*1e3:12.3f} ms  {ltssm_name(field(entry, 32, 8)):<32} {dll_name(field(entry, 40, 8))}")
        n += 1
    return n


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("command", choices=["status", "trace", "watch", "tlp"], nargs="?", default="status")
    parser.add_argument("--csr-csv", required=True, help="build/<build_name>/csr.csv")
    parser.add_argument("--gen", type=int, choices=[1, 2], default=1, help="For trace timestamps")
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=1234)
    parser.add_argument("--interval", type=float, default=0.5)
    args = parser.parse_args()

    bus = RemoteClient(host=args.host, port=args.port, csr_csv=args.csr_csv)
    bus.open()
    try:
        if args.command == "status":
            show_status(bus)
        elif args.command == "trace":
            drain_trace(bus, args.gen, limit=64)
        elif args.command == "tlp":
            drain_capture(bus, args.gen)
        else:
            while True:
                drain_trace(bus, args.gen, limit=64)
                time.sleep(args.interval)
    except KeyboardInterrupt:
        pass
    finally:
        bus.close()


if __name__ == "__main__":
    main()
