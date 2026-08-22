"""Send every delta on every board to one end-effector location.

Opens all connected boards (DeltaArrayEnv), converts the target end-effector
point (x, y, z, metres) to joint positions via inverse kinematics, and commands
all 4 deltas on each board to that point — 16 deltas total across 4 boards.

The firmware drives each delta to the point autonomously after accepting the
command, so the boards move (roughly) concurrently.
"""

import argparse

from delta_control import DeltaArrayEnv
from delta_control.constants import HOME_POSITION, NUM_MOTORS
from delta_control.delta_array_env import delta  # shared PrismaticDelta (IK)

MOTORS_PER_DELTA = 3
DELTAS_PER_BOARD = NUM_MOTORS // MOTORS_PER_DELTA  # 4


def run(target, motor_space) -> None:
    if motor_space:
        # Motor space: --z is a joint position, applied to all 3 motors of every
        # delta, so each delta travels straight up/down (x/y are ignored).
        per_delta = [target[2]] * MOTORS_PER_DELTA
        what = f"joint z {target[2]} m"
    else:
        # IK: one EE point -> the 3 joint positions for a single delta.
        per_delta = list(delta.ik(target))
        assert len(per_delta) == MOTORS_PER_DELTA, per_delta
        what = f"target EE {tuple(target)} m"

    # Repeat across all 4 deltas on a board to get its 12-motor command.
    board_joints = per_delta * DELTAS_PER_BOARD
    assert len(board_joints) == NUM_MOTORS, board_joints

    env = DeltaArrayEnv()  # every connected board, keyed by chip id
    n = len(env.active_ids)
    print(f"{what} -> joints/delta {[round(j, 4) for j in per_delta]}")
    print(f"commanding {n} board(s) x {DELTAS_PER_BOARD} deltas = {n * DELTAS_PER_BOARD} deltas")
    try:
        for bid in env.active_ids:
            env.agents[bid].move_joint_position(board_joints)
            print(f"  board {bid} -> sent")
    finally:
        env.close()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--x", type=float, default=HOME_POSITION[0], help="EE x (m)")
    parser.add_argument("--y", type=float, default=HOME_POSITION[1], help="EE y (m)")
    parser.add_argument("--z", type=float, default=HOME_POSITION[2], help="EE z (m)")
    parser.add_argument(
        "--motor_space",
        action="store_true",
        help="treat --z as a joint position applied to all motors (x/y ignored)",
    )
    args = parser.parse_args()
    run((args.x, args.y, args.z), args.motor_space)


if __name__ == "__main__":
    main()
