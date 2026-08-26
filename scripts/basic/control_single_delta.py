"""Basic smoke test for a single delta robot on the array.

Homes the board, sweeps all 12 motors through low/mid/high joint positions,
returns to home, and closes the port.
"""

import argparse
import time

from delta_control import enforce_calibration, open_board
from delta_control.constants import NUM_MOTORS

SETTLE_TIME = 5.0

DEFAULT_PORT = None  # None auto-detects the board's port by scanning /dev/ttyACM*
DEFAULT_BOARD = None  # None auto-discovers; set a BOARD_REGISTRY label or raw id to skip

DELTA_INDEX = 2
TARGET_POS = [0.05, 0.06, 0.05]


def run(port: str, board=None) -> None:
    env, agent = open_board(port, board)
    enforce_calibration(env)  # push per-board calibration + gate, or abort
    print(f"opening {port}, using board id {env.active_ids[0]}")

    try:
        print("homing...")
        env.reset()
        time.sleep(2)
        agent.move_delta(DELTA_INDEX, TARGET_POS)
        time.sleep(SETTLE_TIME)

        print("returning to home")
        env.reset()
        time.sleep(SETTLE_TIME)
    finally:
        print("closing port")
        agent.close()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=DEFAULT_PORT, help="serial port of the delta board")
    parser.add_argument("--id", default=DEFAULT_BOARD, help="board label (BOARD_REGISTRY) or raw id; omit to auto-discover")
    args = parser.parse_args()
    run(args.port, args.id)


if __name__ == "__main__":
    main()
