"""Move ONE delta on ONE board to a commanded pose.

Opens a single board, homes it, drives the chosen delta (0-3) to a target, holds
it there, then homes again and releases the port. The other three deltas on the
board stay at home throughout.

The target is given either way round:
  --joints A B C   the three prismatic actuator heights directly (metres)
  --xyz X Y Z      an end-effector tip position (metres), run through IK

Pass --id (a BOARD_REGISTRY label) or --port whenever more than one board is
plugged in — a single board can be auto-discovered, a full array cannot.

Examples::

    # delta 2 up to 60 mm on its middle actuator, others at 50 mm
    python scripts/basic/control_single_delta.py --id board3 --delta 2 --joints 0.05 0.06 0.05

    # delta 0 to 5 mm off-centre in +x at home height
    python scripts/basic/control_single_delta.py --id board3 --delta 0 --xyz 0.005 0 0.1315
"""

import argparse
import time

import numpy as np

from delta_control import enforce_calibration, find_board_ports, open_board
from delta_control.boards import BOARD_LABELS
from delta_control.constants import (
    DELTAS_PER_BOARD,
    HOME_POSITION,
    MAX_JOINT_POS,
    MIN_JOINT_POS,
)
from delta_control.delta_array_env import delta as kinematics

DEFAULT_PORT = None  # None auto-detects the board's port by scanning /dev/ttyACM*
DEFAULT_BOARD = None  # None auto-discovers; set a BOARD_REGISTRY label or raw id to skip
DEFAULT_DELTA = 0
DEFAULT_JOINTS = [0.05, 0.05, 0.05]  # home is 0.05175 on all three
DEFAULT_SETTLE = 5.0


def resolve_target(joints=None, xyz=None):
    """Return the three actuator heights for a --joints or --xyz request.

    Raises ValueError for an xyz outside the reachable workspace (IK returns
    NaN) or for heights outside the actuators' travel. move_joint_position
    silently clips out-of-range values, so checking here is what turns a typo
    into an error message instead of a pose that quietly isn't the one asked for.
    """
    if (joints is None) == (xyz is None):
        raise ValueError("give exactly one of joints / xyz")

    if xyz is not None:
        heights = np.asarray(kinematics.ik(xyz), dtype=float)
        if np.any(np.isnan(heights)):
            raise ValueError(f"{list(xyz)} is outside the delta's reachable workspace")
    else:
        heights = np.asarray(joints, dtype=float)

    out = [round(float(h), 6) for h in heights]
    bad = [h for h in out if not MIN_JOINT_POS <= h <= MAX_JOINT_POS]
    if bad:
        raise ValueError(
            f"joint heights {bad} outside travel [{MIN_JOINT_POS}, {MAX_JOINT_POS}] m "
            f"(full target {out})"
        )
    return out


def run(port=DEFAULT_PORT, board=DEFAULT_BOARD, delta_index=DEFAULT_DELTA,
        joints=None, xyz=None, settle=DEFAULT_SETTLE) -> None:
    if not 0 <= delta_index < DELTAS_PER_BOARD:
        raise ValueError(f"delta index must be 0..{DELTAS_PER_BOARD - 1}, got {delta_index}")
    target = resolve_target(joints, xyz)

    if port is None and board is None:
        # open_board would open every board on the bus in turn and only then
        # complain. Probing with whoami first is quick and says which to pick.
        found = find_board_ports()
        if len(found) != 1:
            listing = "\n".join(
                f"  {p}  id {b}  {BOARD_LABELS.get(b, 'unregistered')}" for p, b in found)
            raise SystemExit(
                f"{len(found)} boards connected; pass --id <label> or --port to choose one:\n"
                + (listing or "  (none — check the USB connections)"))
        port = found[0][0]

    env, agent = open_board(port, board)
    enforce_calibration(env)  # push per-board calibration + gate, or abort
    try:
        board_id = env.active_ids[0]
        label = BOARD_LABELS.get(board_id, "unregistered")
        print(f"board {board_id} ({label}) on {env.port_for(board_id)}")

        print(f"homing all {DELTAS_PER_BOARD} deltas to {HOME_POSITION}...")
        env.reset()
        time.sleep(2)

        tip = np.ravel(kinematics.fk(target))
        print(f"delta {delta_index} -> joints {target} (tip {[round(v, 5) for v in tip]})")
        agent.move_delta(delta_index, target)
        time.sleep(settle)
        print(f"  reached {[round(v, 5) for v in agent.get_joint_positions()[delta_index * 3:delta_index * 3 + 3]]}")

        print("returning to home")
        env.reset()
        time.sleep(settle)
    finally:
        print("closing port")
        env.close()


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default=DEFAULT_PORT, help="serial port of the delta board")
    parser.add_argument("--id", default=DEFAULT_BOARD, help="board label (BOARD_REGISTRY) or raw id; omit to auto-discover")
    parser.add_argument("--delta", type=int, default=DEFAULT_DELTA,
                        help=f"which delta on the board, 0..{DELTAS_PER_BOARD - 1}")
    target = parser.add_mutually_exclusive_group()
    target.add_argument("--joints", type=float, nargs=3, metavar=("A", "B", "C"),
                        help=f"actuator heights in m, each within [{MIN_JOINT_POS}, {MAX_JOINT_POS}]")
    target.add_argument("--xyz", type=float, nargs=3, metavar=("X", "Y", "Z"),
                        help="end-effector tip position in m, converted with IK")
    parser.add_argument("--settle", type=float, default=DEFAULT_SETTLE,
                        help="seconds to hold each pose (s)")
    args = parser.parse_args()

    joints, xyz = args.joints, args.xyz
    if joints is None and xyz is None:
        joints = DEFAULT_JOINTS
    run(args.port, args.id, args.delta, joints, xyz, args.settle)


if __name__ == "__main__":
    main()
