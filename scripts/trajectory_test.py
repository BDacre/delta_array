"""Basic test for a single delta robot on the array following a trajectory.

Homes the board, sends a single batched joint-space sine trajectory (one
TrajectoryCommand on the wire; firmware advances row-by-row as the PID
settles each one), returns to home, and closes the port.
"""

import argparse
import time

import numpy as np

from delta_control import open_board
from delta_control.constants import MAX_TRAJECTORY_ROWS, NUM_MOTORS

SETTLE_TIME = 2.0
NUM_ROWS = MAX_TRAJECTORY_ROWS

# Per-row wall-clock budget while the trajectory executes on-board. The
# command ACK fires on acceptance, not completion, so the host must sleep
# long enough for all rows to settle before issuing the next command.
# Stays well under the firmware's 5 s per-row MOVE_TIMEOUT_MS.
PER_ROW_BUDGET_S = 1.0

DEFAULT_PORT = None  # None auto-detects the board's port by scanning /dev/ttyACM*
DEFAULT_BOARD = None  # None auto-discovers; set a BOARD_REGISTRY label or raw id to skip


def sine_trajectory(num_rows: int) -> np.ndarray:
    t = np.arange(num_rows)
    z = 0.04 * np.sin(2 * np.pi * t / num_rows) + 0.05
    return np.tile(z[:, None], (1, NUM_MOTORS))


def run(port: str, board=None) -> None:
    env, agent = open_board(port, board)
    print(f"opening {port}, using board id {env.active_ids[0]}")

    try:
        print("homing...")
        env.reset()
        time.sleep(SETTLE_TIME)

        trajectory = sine_trajectory(NUM_ROWS)
        print(f"sending {NUM_ROWS}-row sine trajectory")
        agent.move_joint_position(trajectory)

        wait_s = PER_ROW_BUDGET_S * NUM_ROWS
        print(f"waiting {wait_s:.1f}s for trajectory to complete")
        time.sleep(wait_s)

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
