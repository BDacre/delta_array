"""Discover the chip-derived id(s) of connected delta board(s).

Firmware now derives each board's id from its SAMD21 serial number, so ids are
large and not human-chosen. This utility scans serial ports (or a single --port),
sends a broadcast whoami, and prints the id each board reports. Copy a printed id
into delta_control.constants.BOARD_REGISTRY under a friendly label so the rest of
the host code can address that board by name.

Sends only a whoami (no motion), so probing every port is safe.
"""

import argparse

from serial import Serial

from delta_control.constants import BOARD_LABELS, DEFAULT_BAUD, DISCOVERY_TIMEOUT_S
from delta_control.delta_array_env import discover_board_id, find_board_ports
from delta_control.transport import ProtoTransport

DEFAULT_PORT = None  # None scans all /dev/ttyACM* ports


def _report(port, board_id):
    label = BOARD_LABELS.get(board_id)
    known = f" (registered as {label!r})" if label else " (not in BOARD_REGISTRY)"
    print(f"{port} -> board id {board_id}{known}")


def run(port=None) -> None:
    if port is None:
        print("scanning /dev/ttyACM* for delta boards...")
        found = find_board_ports()
        if not found:
            print("no delta board answered whoami on any port "
                  "(check power / USB / that the board is flashed)")
            raise SystemExit(1)
        for p, board_id in found:
            _report(p, board_id)
        return

    print(f"opening {port} for board discovery")
    ser = Serial(port, DEFAULT_BAUD, timeout=DISCOVERY_TIMEOUT_S)
    try:
        board_id = discover_board_id(ProtoTransport(ser))
    finally:
        ser.close()
    if board_id is None:
        print(f"no board answered whoami on {port} (check connection / port / baud)")
        raise SystemExit(1)
    _report(port, board_id)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=DEFAULT_PORT,
                        help="serial port of the delta board; omit to scan all /dev/ttyACM*")
    args = parser.parse_args()
    run(args.port)


if __name__ == "__main__":
    main()
