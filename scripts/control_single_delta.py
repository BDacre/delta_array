"""Basic smoke test for a single delta robot on the array.

Homes the board, sweeps all 12 motors through low/mid/high joint positions,
returns to home, and closes the port.
"""

import argparse
import time

from delta_control import DeltaArrayEnv
from delta_control.constants import NUM_MOTORS

SETTLE_TIME = 30.0

DEFAULT_PORT = "/dev/ttyACM0"
DEFAULT_ROBOT_ID = 9

DELTA_INDEX = 1
TARGET_POS = [0.09, 0.05, 0.05]


def run(port: str, robot_id: int) -> None:
    print(f"opening {port} for robot id {robot_id}")
    env = DeltaArrayEnv(port, active_ids=(robot_id,))
    agent = env.agents[robot_id]

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
    parser.add_argument("--id", type=int, default=DEFAULT_ROBOT_ID, help="active robot id (1-16)")
    args = parser.parse_args()
    run(args.port, args.id)


if __name__ == "__main__":
    main()
