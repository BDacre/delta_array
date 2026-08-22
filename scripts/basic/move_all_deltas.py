"""Drive every delta in the array to a random z-height.

Each delta gets a random joint value, applied to all 3 of its motors, so the
delta travels straight up/down in z (no x/y tilt).

Boards are addressed in BOARD_REGISTRY label order (board0, board1, ...); a
registry board that isn't connected is skipped.

"""

import argparse
import random

from delta_control import DeltaArrayEnv
from delta_control.boards import BOARD_REGISTRY, board_order
from delta_control.constants import NUM_MOTORS

MOTORS_PER_DELTA = 3
DELTAS_PER_BOARD = NUM_MOTORS // MOTORS_PER_DELTA  # 4


def run(low=0.01, high=0.03, seed=None) -> None:
    rng = random.Random(seed)
    labels = board_order()
    n_deltas = len(labels) * DELTAS_PER_BOARD
    # One random height per delta.
    heights = [round(rng.uniform(low, high), 4) for _ in range(n_deltas)]
    print(f"{n_deltas} random delta heights in [{low}, {high}] (seed={seed})")

    env = DeltaArrayEnv()  # every connected board, keyed by chip id
    try:
        for i, label in enumerate(labels):
            bid = BOARD_REGISTRY[label]
            board_heights = heights[i * DELTAS_PER_BOARD:(i + 1) * DELTAS_PER_BOARD]
            # Each delta's height repeated across its 3 motors -> 12-motor command.
            chunk = [h for h in board_heights for _ in range(MOTORS_PER_DELTA)]
            agent = env.agents.get(bid)
            if agent is None:
                print(f"  {label} (id {bid}) -> NOT CONNECTED, skipped")
                continue
            agent.move_joint_position(chunk)
            print(f"  {label} (id {bid}) -> delta heights {board_heights}")
    finally:
        env.close()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--low", type=float, default=0.01, help="min delta height (m)")
    parser.add_argument("--high", type=float, default=0.03, help="max delta height (m)")
    parser.add_argument("--seed", type=int, default=None, help="RNG seed for reproducibility")
    args = parser.parse_args()
    run(args.low, args.high, args.seed)


if __name__ == "__main__":
    main()
