
import math
import time

from delta_control import enforce_calibration, open_board

SETTLE_TIME = 1.0

DEFAULT_PORT = None  # None auto-detects the board's port by scanning /dev/ttyACM*
DEFAULT_BOARD = None  # None auto-discovers; set a BOARD_REGISTRY label or raw id to skip

DELTA_INDEX = 0
MAX_ACTUATOR_DIFF = 0.005
ACTUATOR_STEP = 0.001


def _inclusive_range(start: float, stop: float, step: float) -> list[float]:
    """Float range inclusive of stop, rounded to avoid accumulation error."""
    n = round((stop - start) / step) + 1
    return [round(start + i * step, 8) for i in range(n) if start + i * step <= stop + 1e-9]


def run(port: str, board=None) -> None:
    env, agent = open_board(port, board)
    enforce_calibration(env)  # push per-board calibration + gate, or abort
    print(f"opening {port}, using board id {env.active_ids[0]}")

    try:
        print("homing...")
        env.reset()
        time.sleep(1)

        min_pos = agent.min_joint_pos
        max_pos = agent.max_joint_pos

        print(f"motor range: [{min_pos:.4f}, {max_pos:.4f}]")
        print(f"window size: {MAX_ACTUATOR_DIFF}, step: {ACTUATOR_STEP}")

        # Move delta to min position so all motors start equal (safe baseline)
        print(f"moving delta {DELTA_INDEX} to min position...")
        agent.move_delta(DELTA_INDEX, [min_pos] * 3)
        time.sleep(SETTLE_TIME)

        # Stages: slide the window [win_start, win_start + MAX_ACTUATOR_DIFF] upward by
        # ACTUATOR_STEP each stage until the window's upper edge reaches max_pos.
        # All motor combinations within a window differ by at most MAX_ACTUATOR_DIFF.
        span = max_pos - min_pos
        if span <= MAX_ACTUATOR_DIFF:
            num_stages = 1
        else:
            num_stages = math.ceil((span - MAX_ACTUATOR_DIFF) / ACTUATOR_STEP) + 1

        pts_per_window = len(_inclusive_range(min_pos, min_pos + MAX_ACTUATOR_DIFF, ACTUATOR_STEP))
        total_pts = num_stages * pts_per_window ** 3
        print(f"stages: {num_stages}, combos per stage: {pts_per_window**3}, total: {total_pts}")
        print(f"estimated time: {total_pts * SETTLE_TIME / 60:.1f} min")

        visited = 0
        for stage in range(num_stages):
            win_start = min(min_pos + stage * ACTUATOR_STEP, max_pos - MAX_ACTUATOR_DIFF)
            win_end = min(win_start + MAX_ACTUATOR_DIFF, max_pos)
            win_pts = _inclusive_range(win_start, win_end, ACTUATOR_STEP)

            print(f"\nstage {stage + 1}/{num_stages}: "
                  f"window [{win_start:.4f}, {win_end:.4f}], "
                  f"{len(win_pts)**3} combos")

            # Snap all motors to window start — all equal, always safe
            agent.move_delta(DELTA_INDEX, [win_start] * 3)
            time.sleep(SETTLE_TIME)

            for m0 in win_pts:
                for m1 in win_pts:
                    for m2 in win_pts:
                        agent.move_delta(DELTA_INDEX, [m0, m1, m2])
                        time.sleep(SETTLE_TIME)
                        visited += 1

        print(f"\nsweep complete — {visited} points visited")

        # Return to safe home position
        agent.move_delta(DELTA_INDEX, [min_pos] * 3)
        time.sleep(SETTLE_TIME)

    finally:
        print("closing port")
        agent.close()


if __name__ == "__main__":
    run(DEFAULT_PORT, DEFAULT_BOARD)
