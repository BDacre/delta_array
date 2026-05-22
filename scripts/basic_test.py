"""Basic smoke test for a single delta robot on the array.

Homes the board, sweeps all 12 motors through low/mid/high joint positions,
returns to home, and closes the port.
"""

import argparse
import time

from delta_control import DeltaArrayEnv
from delta_control.constants import NUM_MOTORS

SETTLE_S = 2.0
SWEEP_POSITIONS = (0.01, 0.05, 0.09)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyACM0", help="serial port of the delta board")
    parser.add_argument("--id", type=int, default=9, help="active robot id (1-16)")
    args = parser.parse_args()

    print(f"opening {args.port} for robot id {args.id}")
    env = DeltaArrayEnv(args.port, active_ids=(args.id,))
    agent = env.agents[args.id]

    try:
        print("homing...")
        env.reset()
        time.sleep(SETTLE_S)

        for value in SWEEP_POSITIONS:
            print(f"moving all motors to {value}")
            agent.move_joint_position([value] * NUM_MOTORS)
            time.sleep(SETTLE_S)

        print("returning to home")
        env.reset()
        time.sleep(SETTLE_S)
    finally:
        print("closing port")
        agent.close()


if __name__ == "__main__":
    main()
