"""Position-target staircase for one motor: measure deadband and backlash.

Sweeps a single joint's *target* up then back down in small increments, holding
the other 11 motors at a fixed baseline. After each step it waits for the joint
to settle and records commanded vs. measured position. Plotting measured against
commanded reveals:

  * deadband   -- flat regions where a commanded change produces no motion
                  (the firmware ignores errors under position_threshold = 0.8 mm,
                  and static friction widens this further),
  * hysteresis -- the gap between the up-sweep and down-sweep curves, i.e.
                  mechanical backlash + stiction.

The firmware runs a closed-loop PID and RELEASEs the motor once all joints are
within the deadband (or after MOVE_TIMEOUT_MS), so each recorded point is the
settled, motor-released position -- exactly what you want for a static map.

Example:
    python characterize_staircase.py --motor 0 --start 0.03 --stop 0.07 --step 0.0005
"""

import argparse
import csv
import os
import time
from datetime import datetime

from delta_control import open_board
from delta_control.constants import (
    MAX_JOINT_POS,
    MIN_JOINT_POS,
    NUM_MOTORS,
)

DEFAULT_PORT = None  # None auto-detects the board's port by scanning /dev/ttyACM*
DEFAULT_BOARD = None  # None auto-discovers; set a BOARD_REGISTRY label or raw id to skip

# Outputs are collected under <repo>/generated_files/<test>/ so runs don't litter
# the working directory. Anchored to the repo (not cwd) so it lands in the same
# place regardless of where the script is launched from.
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_OUTDIR = os.path.join(REPO_ROOT, "generated_files")
TEST_NAME = "staircase"


def _mm(x):
    """Format a meters value as a filename-safe millimeter token (0.0005 -> '0p5mm')."""
    return f"{x * 1e3:g}".replace(".", "p") + "mm"


def build_output_path(outdir, motor, start, stop, step):
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    name = (f"{TEST_NAME}_motor{motor:02d}_"
            f"{_mm(start)}-{_mm(stop)}_step{_mm(step)}_{stamp}.csv")
    return os.path.join(outdir, TEST_NAME, name)

# One ADC count is ~0.06 mm (ADC_TO_POSITION = 6e-5). Treat the joint as settled
# once the reading holds within ~1.5 counts across several polls; that tolerates
# +/-1 count of ADC jitter without waiting forever.
SETTLE_EPS = 9e-5
SETTLE_STABLE_SAMPLES = 6
SETTLE_POLL_DT = 0.02
SETTLE_TIMEOUT = 6.0  # firmware aborts an unreachable move at MOVE_TIMEOUT_MS = 5 s


def wait_until_settled(agent, motor, timeout=SETTLE_TIMEOUT):
    """Poll one joint until its reading stops changing, or until timeout.

    Returns the last measured position (meters) for `motor`.
    """
    last = None
    stable = 0
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < timeout:
        pos = agent.get_joint_positions()[motor]
        if last is not None and abs(pos - last) < SETTLE_EPS:
            stable += 1
            if stable >= SETTLE_STABLE_SAMPLES:
                return pos
        else:
            stable = 0
        last = pos
        time.sleep(SETTLE_POLL_DT)
    return last


def frange(start, stop, step):
    """Inclusive-ish float range: start, start+step, ... up to (and incl) stop."""
    n = int(round((stop - start) / step))
    return [start + i * step for i in range(n + 1)]


def plot_staircase(rows, motor):
    """Show measured-vs-commanded (deadband + hysteresis) and the sweep sequence."""
    import matplotlib.pyplot as plt

    up = [(t, m) for d, t, m, _ in rows if d == "up"]
    down = [(t, m) for d, t, m, _ in rows if d == "down"]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5))

    # Left: measured vs commanded. Gap between up/down = backlash+stiction;
    # flats = deadband. y=x is perfect tracking.
    cmd_all = [t * 1e3 for _, t, _, _ in rows]
    lo, hi = min(cmd_all), max(cmd_all)
    ax1.plot([lo, hi], [lo, hi], color="0.6", ls="--", lw=1.0, label="ideal (y=x)")
    if up:
        ax1.plot([t * 1e3 for t, _ in up], [m * 1e3 for _, m in up],
                 color="C0", marker="o", ms=4, label="up sweep")
    if down:
        ax1.plot([t * 1e3 for t, _ in down], [m * 1e3 for _, m in down],
                 color="C3", marker="s", ms=4, label="down sweep")
    ax1.set_xlabel("commanded target (mm)")
    ax1.set_ylabel("measured position (mm)")
    ax1.set_title(f"motor {motor}: measured vs commanded")
    ax1.legend(loc="best")
    ax1.grid(True, alpha=0.3)
    ax1.set_aspect("equal", adjustable="datalim")

    # Right: target and measured as sequences over the sweep.
    idx = list(range(len(rows)))
    ax2.plot(idx, [t * 1e3 for _, t, _, _ in rows], color="C1", marker=".",
             label="commanded target")
    ax2.plot(idx, [m * 1e3 for _, _, m, _ in rows], color="C0", marker=".",
             label="measured position")
    ax2.set_xlabel("sweep point index")
    ax2.set_ylabel("position (mm)")
    ax2.set_title(f"motor {motor}: target & position over sweep")
    ax2.legend(loc="best")
    ax2.grid(True, alpha=0.3)

    fig.tight_layout()
    plt.show()


def run(port, board, motor, start, stop, step, baseline, out, outdir, show_plot=True):
    out_path = out or build_output_path(outdir, motor, start, stop, step)
    lo = MIN_JOINT_POS
    hi = MAX_JOINT_POS
    for name, val in (("start", start), ("stop", stop), ("baseline", baseline)):
        if not (lo <= val <= hi):
            raise SystemExit(
                f"{name}={val} outside joint range [{lo}, {hi}] "
                f"(values get clipped by the firmware; pick something inside)"
            )
    if not (0 <= motor < NUM_MOTORS):
        raise SystemExit(f"--motor must be 0..{NUM_MOTORS - 1}, got {motor}")

    up = frange(start, stop, step)
    down = list(reversed(up))[1:]  # skip the repeated top point
    sweep = [("up", t) for t in up] + [("down", t) for t in down]

    env, agent = open_board(port, board)
    print(f"opening {port}, using board id {env.active_ids[0]}")
    print(f"motor {motor}: staircase {start} -> {stop} -> {start} step {step} m, "
          f"others held at {baseline} m")

    rows = []
    try:
        # Seat every joint at the baseline before starting the sweep.
        base_vec = [baseline] * NUM_MOTORS
        base_vec[motor] = start
        agent.move_joint_position(base_vec)
        wait_until_settled(agent, motor)

        for direction, target in sweep:
            vec = [baseline] * NUM_MOTORS
            vec[motor] = target
            agent.move_joint_position(vec)
            measured = wait_until_settled(agent, motor)
            err = measured - target
            rows.append((direction, target, measured, err))
            print(f"  {direction:4s} cmd={target:.5f}  meas={measured:.5f}  "
                  f"err={err * 1e3:+.3f} mm")
    finally:
        print("returning to baseline, closing port")
        try:
            agent.move_joint_position([baseline] * NUM_MOTORS)
            time.sleep(0.5)
        finally:
            agent.close()

    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, "w", newline="") as f:
        w = csv.writer(f)
        # Header comment captures the run parameters so the CSV is self-describing.
        w.writerow([f"# {TEST_NAME} motor={motor} start={start} stop={stop} "
                    f"step={step} baseline={baseline} board={env.active_ids[0]}"])
        w.writerow(["direction", "commanded_m", "measured_m", "error_m"])
        w.writerows(rows)
    print(f"wrote {len(rows)} points to {out_path}")

    if show_plot:
        try:
            plot_staircase(rows, motor)
        except Exception as e:  # headless / no display / backend issue
            print(f"skipping plot ({e}); data is in {out_path}")


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default=DEFAULT_PORT, help="serial port of the delta board")
    p.add_argument("--id", default=DEFAULT_BOARD,
                   help="board label (BOARD_REGISTRY) or raw id; omit to auto-discover")
    p.add_argument("--motor", type=int, required=True, help=f"motor index 0..{NUM_MOTORS - 1}")
    p.add_argument("--start", type=float, default=0.03, help="sweep start (m)")
    p.add_argument("--stop", type=float, default=0.07, help="sweep end (m)")
    p.add_argument("--step", type=float, default=0.0005, help="increment (m); try < deadband to see it")
    p.add_argument("--baseline", type=float, default=0.05, help="hold position for the other 11 motors (m)")
    p.add_argument("--outdir", default=DEFAULT_OUTDIR,
                   help="base directory for generated files (a per-test subdir is added)")
    p.add_argument("--out", default=None,
                   help="explicit output CSV path; overrides the auto-generated descriptive name")
    p.add_argument("--no-plot", dest="plot", action="store_false",
                   help="skip the interactive plot (e.g. headless runs)")
    args = p.parse_args()
    run(args.port, args.id, args.motor, args.start, args.stop, args.step,
        args.baseline, args.out, args.outdir, show_plot=args.plot)


if __name__ == "__main__":
    main()
