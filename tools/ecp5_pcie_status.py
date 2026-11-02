#!/usr/bin/env python3
"""Read the ECP5 PCIe bring-up CSRs (soc_ecp5.py) over UARTBone.

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


def read(bus, reg):
    return getattr(bus.regs, f"pcie_core_{reg}").read()


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


def drain_trace(bus, gen):
    clk = WORD_CLK_HZ[gen]
    level = read(bus, "trace_level")
    if field(level, 1, 1):
        print("trace     : overflow, entries were dropped")
    n = 0
    while field(read(bus, "trace_level"), 0, 1):
        entry = read(bus, "trace_data")
        bus.regs.pcie_core_trace_pop.write(1)
        t = field(entry, 0, 32)
        print(f"{t/clk*1e3:12.3f} ms  {ltssm_name(field(entry, 32, 8)):<32} {dll_name(field(entry, 40, 8))}")
        n += 1
    return n


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("command", choices=["status", "trace", "watch"], nargs="?", default="status")
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
            drain_trace(bus, args.gen)
        else:
            while True:
                drain_trace(bus, args.gen)
                time.sleep(args.interval)
    except KeyboardInterrupt:
        pass
    finally:
        bus.close()


if __name__ == "__main__":
    main()
