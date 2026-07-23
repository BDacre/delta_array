"""Access the screws on every delta of several boards, one board at a time.

Multi-board variant of access_delta_screws.py. Loops over the boards listed in
BOARD_REGISTRY; for each connected board it drops all motors low, then raises
each of its 4 deltas to the screw-access height in turn, pausing for you to work
on the screws before moving to the next delta / board.

Opens every connected Feather at once (DeltaArrayEnv) and addresses each board by
its BOARD_REGISTRY chip id, so which /dev/ttyACM* each landed on doesn't matter.
Boards in the registry that aren't currently connected are skipped with a note.
"""

import time

from delta_control import DeltaArrayEnv
from delta_control.constants import BOARD_REGISTRY, NUM_MOTORS

DELTA_ORDER = [0, 1, 2, 3]          # the 4 sub-deltas on each board
TARGET_POS = [0.09, 0.09, 0.09]     # screw-access height (raised)
LOW_POS = [0.01] * NUM_MOTORS       # all motors low / out of the way


def access_board(label, agent): 
    print(f"\n=== board {label!r}  (id {agent.robot_id}) ===")
    print("  moving all motors low...")
    agent.move_joint_position(LOW_POS)
    time.sleep(1)

    for delta_index in DELTA_ORDER:
        print(f"  raising delta {delta_index} to {TARGET_POS}")
        agent.move_delta(delta_index, TARGET_POS)
        input(f"  board {label} delta {delta_index} in position. "
              f"Press Enter to lower it and continue...")
        agent.move_joint_position(LOW_POS)
        time.sleep(1)


def run() -> None:
    env = DeltaArrayEnv()  # open every connected board, agents keyed by chip id
    print(f"connected board ids: {env.active_ids}")
    try:
        for label in sorted(BOARD_REGISTRY):
            board_id = BOARD_REGISTRY[label]
            agent = env.agents.get(board_id)
            if agent is None:
                print(f"skipping board {label!r} (id {board_id}) — not connected")
                continue
            access_board(label, agent)
    finally:
        print("\nclosing ports")
        env.close()


def main() -> None:
    run()


if __name__ == "__main__":
    main()
