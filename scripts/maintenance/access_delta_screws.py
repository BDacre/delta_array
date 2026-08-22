"""Basic smoke test for a single delta robot on the array.

Homes the board, sweeps all 12 motors through low/mid/high joint positions,
returns to home, and closes the port.
"""

import argparse
import time

from delta_control import open_board
from delta_control.constants import NUM_MOTORS

DELTA_ORDER = [0,1,2,3]
TARGET_POS = [0.09, 0.09, 0.09]
DEFAULT_PORT = None  # None auto-detects the board's port by scanning /dev/ttyACM*
# None -> auto-discover whatever single board is connected. open_board raises
# DiscoveryError if more than one board is present, so this is safe for the common
# single-board case. Pass a concrete label (e.g. "board1") via --id to address one
# specific board when several are connected.
DEFAULT_BOARD = None


def run(port: str, board=None) -> None:
    env, agent = open_board(port, board)
    print(f"opening {port}, using board id {env.active_ids[0]}")


    try:
        print("homing...")
        env.reset()
        time.sleep(1)

        print("Moving low")
        low_pos = [0.01] * NUM_MOTORS
        agent.move_joint_position(low_pos)
        time.sleep(1)

        for delta_index in DELTA_ORDER:
            print(f"moving delta {delta_index} to {TARGET_POS}")

            agent.move_delta(delta_index, TARGET_POS)
            #agent.move_joint_position([TARGET_POS] * NUM_MOTORS)
            input(f"delta {delta_index} in position. Press Enter to continue to the next delta...")

            agent.move_joint_position(low_pos)
            time.sleep(1)
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
