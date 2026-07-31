"""Position-target staircase: measure deadband and backlash per motor.

For each motor, sweeps its *target* up then back down in small increments while
holding the other 11 motors at a fixed baseline. After each step it waits for the
joint to settle and records commanded vs. measured position. Plotting measured
against commanded reveals:

  * deadband   -- flat regions where a commanded change produces no motion
                  (the firmware ignores errors under the per-motor deadband,
                  default 0.8 mm, and static friction widens this further),
  * hysteresis -- the gap between the up-sweep and down-sweep curves, i.e.
                  mechanical backlash + stiction.

The firmware runs a closed-loop PID and RELEASEs the motor once all joints are
within the deadband (or after MOVE_TIMEOUT_MS), so each recorded point is the
settled, motor-released position -- exactly what you want for a static map.

By default it sweeps all 12 motors, one at a time; pass --motor to do just one.

Example:
    python characterize_staircase.py                       # all motors
    python characterize_staircase.py --motor 0 --start 0.03 --stop 0.07 --step 0.0005
"""

import argparse
import csv
import math
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


def build_output_path(outdir, motors, start, stop, step):
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    tag = f"motor{motors[0]:02d}" if len(motors) == 1 else f"{len(motors)}motors"
    name = (f"{TEST_NAME}_{tag}_"
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


def sweep_motor(agent, motor, sweep, baseline):
    """Run the up/down target sweep on one motor; return a list of result rows.

    Each row is (motor, direction, commanded_m, measured_m, error_m). The other
    joints are held at `baseline` on every step.
    """
    # Seat every joint at baseline, with this motor at the sweep's first target.
    base_vec = [baseline] * NUM_MOTORS
    base_vec[motor] = sweep[0][1]
    agent.move_joint_position(base_vec)
    wait_until_settled(agent, motor)

    rows = []
    for direction, target in sweep:
        vec = [baseline] * NUM_MOTORS
        vec[motor] = target
        agent.move_joint_position(vec)
        measured = wait_until_settled(agent, motor)
        err = measured - target
        rows.append((motor, direction, target, measured, err))
        print(f"    {direction:4s} cmd={target:.5f}  meas={measured:.5f}  "
              f"err={err * 1e3:+.3f} mm")
    return rows


def _plot_measured_vs_commanded(ax, mrows, motor, legend=True):
    """Draw one motor's measured-vs-commanded (deadband + hysteresis) on `ax`."""
    up = [(t, m) for _, d, t, m, _ in mrows if d == "up"]
    down = [(t, m) for _, d, t, m, _ in mrows if d == "down"]
    cmd_all = [t * 1e3 for _, _, t, _, _ in mrows]
    lo, hi = min(cmd_all), max(cmd_all)
    ax.plot([lo, hi], [lo, hi], color="0.6", ls="--", lw=1.0, label="ideal (y=x)")
    if up:
        ax.plot([t * 1e3 for t, _ in up], [m * 1e3 for _, m in up],
                color="C0", marker="o", ms=3, label="up sweep")
    if down:
        ax.plot([t * 1e3 for t, _ in down], [m * 1e3 for _, m in down],
                color="C3", marker="s", ms=3, label="down sweep")
    ax.set_title(f"motor {motor}")
    ax.grid(True, alpha=0.3)
    if legend:
        ax.legend(loc="best", fontsize=8)


def plot_staircase(rows, motors):
    """Show measured-vs-commanded (deadband + hysteresis) per motor.

    One motor -> the detailed two-panel view (map + sweep sequence).
    Many motors -> a grid of measured-vs-commanded maps.
    """
    import matplotlib.pyplot as plt

    if len(motors) == 1:
        motor = motors[0]
        mrows = [r for r in rows if r[0] == motor]
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5))
        _plot_measured_vs_commanded(ax1, mrows, motor)
        ax1.set_xlabel("commanded target (mm)")
        ax1.set_ylabel("measured position (mm)")
        ax1.set_aspect("equal", adjustable="datalim")

        idx = list(range(len(mrows)))
        ax2.plot(idx, [t * 1e3 for _, _, t, _, _ in mrows], color="C1",
                 marker=".", label="commanded target")
        ax2.plot(idx, [m * 1e3 for _, _, _, m, _ in mrows], color="C0",
                 marker=".", label="measured position")
        ax2.set_xlabel("sweep point index")
        ax2.set_ylabel("position (mm)")
        ax2.set_title(f"motor {motor}: target & position over sweep")
        ax2.legend(loc="best")
        ax2.grid(True, alpha=0.3)
        fig.tight_layout()
        plt.show()
        return

    ncols = 4
    nrows = math.ceil(len(motors) / ncols)
    fig, axes = plt.subplots(nrows, ncols, figsize=(4 * ncols, 3.2 * nrows),
                             squeeze=False)
    for k, motor in enumerate(motors):
        ax = axes[k // ncols][k % ncols]
        mrows = [r for r in rows if r[0] == motor]
        if mrows:
            _plot_measured_vs_commanded(ax, mrows, motor, legend=(k == 0))
        if k % ncols == 0:
            ax.set_ylabel("measured (mm)")
        if k // ncols == nrows - 1:
            ax.set_xlabel("commanded (mm)")
    # Blank any unused cells in the last row.
    for k in range(len(motors), nrows * ncols):
        axes[k // ncols][k % ncols].axis("off")
    fig.suptitle("staircase: measured vs commanded (flats = deadband, "
                 "up/down gap = backlash)")
    fig.tight_layout()
    plt.show()


def run(port, board, motors, start, stop, step, baseline, out, outdir,
        show_plot=True):
    out_path = out or build_output_path(outdir, motors, start, stop, step)
    lo = MIN_JOINT_POS
    hi = MAX_JOINT_POS
    for name, val in (("start", start), ("stop", stop), ("baseline", baseline)):
        if not (lo <= val <= hi):
            raise SystemExit(
                f"{name}={val} outside joint range [{lo}, {hi}] "
                f"(values get clipped by the firmware; pick something inside)"
            )

    up = frange(start, stop, step)
    down = list(reversed(up))[1:]  # skip the repeated top point
    sweep = [("up", t) for t in up] + [("down", t) for t in down]

    env, agent = open_board(port, board)
    print(f"opening {port}, using board id {env.active_ids[0]}")
    print(f"staircase {start} -> {stop} -> {start} step {step} m on motors "
          f"{motors}, others held at {baseline} m")

    rows = []
    try:
        for motor in motors:
            print(f"  motor {motor}:")
            rows.extend(sweep_motor(agent, motor, sweep, baseline))
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
        w.writerow([f"# {TEST_NAME} motors={motors} start={start} stop={stop} "
                    f"step={step} baseline={baseline} board={env.active_ids[0]}"])
        w.writerow(["motor", "direction", "commanded_m", "measured_m", "error_m"])
        w.writerows(rows)
    print(f"wrote {len(rows)} points to {out_path}")

    if show_plot:
        try:
            plot_staircase(rows, motors)
        except Exception as e:  # headless / no display / backend issue
            print(f"skipping plot ({e}); data is in {out_path}")


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default=DEFAULT_PORT, help="serial port of the delta board")
    p.add_argument("--id", default=DEFAULT_BOARD,
                   help="board label (BOARD_REGISTRY) or raw id; omit to auto-discover")
    p.add_argument("--motor", type=int, default=None,
                   help=f"single motor 0..{NUM_MOTORS - 1}; omit to sweep all")
    p.add_argument("--start", type=float, default=0.03, help="sweep start (m)")
    p.add_argument("--stop", type=float, default=0.07, help="sweep end (m)")
    p.add_argument("--step", type=float, default=0.001,
                   help="increment (m); default 1mm is just above the 0.8mm deadband "
                        "so steps actually move. Drop below the deadband to observe it.")
    p.add_argument("--baseline", type=float, default=0.05, help="hold position for the other motors (m)")
    p.add_argument("--outdir", default=DEFAULT_OUTDIR,
                   help="base directory for generated files (a per-test subdir is added)")
    p.add_argument("--out", default=None,
                   help="explicit output CSV path; overrides the auto-generated descriptive name")
    p.add_argument("--no-plot", dest="plot", action="store_false",
                   help="skip the interactive plot (e.g. headless runs)")
    args = p.parse_args()

    if args.motor is not None:
        if not (0 <= args.motor < NUM_MOTORS):
            raise SystemExit(f"--motor must be 0..{NUM_MOTORS - 1}, got {args.motor}")
        motors = [args.motor]
    else:
        motors = list(range(NUM_MOTORS))

    run(args.port, args.id, motors, args.start, args.stop, args.step,
        args.baseline, args.out, args.outdir, show_plot=args.plot)


if __name__ == "__main__":
    main()
