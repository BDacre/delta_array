"""Sequentially exercise each of the 12 motors to verify motor ordering.

Homes the board, then drives motor 0..11 one at a time: the target motor is
raised to a high joint position while every other motor is held low, so exactly
one motor moves per step. Pause between steps (Enter, or --dwell for hands-free)
to note which physical motor moved, then confirm the order matches on the next
microcontroller.

Motor i belongs to delta i // 3 (leg i % 3), so the expected sweep is
delta 0: motors 0,1,2 -> delta 1: motors 3,4,5 -> ... -> delta 3: motors 9,10,11.
"""

import argparse
import time

from delta_control import open_board
from delta_control.constants import NUM_MOTORS, MOTORS_PER_DELTA

BASE_POS = 0.02   # low position every idle motor is held at
RAISED_POS = 0.09  # position the motor under test is driven to
SETTLE_TIME = 1.0  # seconds to let a move finish before prompting


def run(port, board=None, dwell=None, base=BASE_POS, raised=RAISED_POS) -> None:
    env, agent = open_board(port, board)
    print(f"opening {port}, using board id {env.active_ids[0]}")

    try:
        print("homing...")
        env.reset()
        time.sleep(SETTLE_TIME)

        print(f"lowering all motors to {base}")
        agent.move_joint_position([base] * NUM_MOTORS)
        time.sleep(SETTLE_TIME)

        for motor in range(NUM_MOTORS):
            delta = motor // MOTORS_PER_DELTA
            leg = motor % MOTORS_PER_DELTA
            print(f"motor {motor:2d}  (delta {delta}, leg {leg})  -> {raised}")

            pos = [base] * NUM_MOTORS
            pos[motor] = raised
            agent.move_joint_position(pos)
            time.sleep(SETTLE_TIME)

            if dwell is None:
                input("    press Enter for the next motor...")
            else:
                time.sleep(dwell)

        print("returning to home")
        env.reset()
        time.sleep(SETTLE_TIME)
    finally:
        print("closing port")
        agent.close()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=None, help="serial port of the delta board; omit to auto-detect")
    parser.add_argument("--id", default=None, help="board label (BOARD_REGISTRY) or raw id; omit to auto-discover")
    parser.add_argument("--dwell", type=float, default=None,
                        help="seconds to pause between motors instead of waiting for Enter")
    parser.add_argument("--base", type=float, default=BASE_POS, help="idle position for the untested motors")
    parser.add_argument("--raised", type=float, default=RAISED_POS, help="position the tested motor moves to")
    args = parser.parse_args()
    run(args.port, args.id, args.dwell, args.base, args.raised)


if __name__ == "__main__":
    main()
