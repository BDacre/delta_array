"""Closed-loop step response for one motor.

Seats one joint at a start position, then commands a step to a target and polls
joint positions as fast as the serial link allows, host-timestamping every
sample. The result shows the closed-loop PID+motor response: rise time,
overshoot, and any stick-slip staircase on the way to the target.

This is a *closed-loop* measurement (PID in the loop), sampled by round-trip
polling over serial at 57600 baud -- expect ~100-200 Hz and some host-side
timing jitter, not a clean bench trace. It is enough to see gross dynamics and
stick-slip. For open-loop / breakaway-PWM characterization you need a firmware
command that drives raw PWM (not available yet). Note also that the firmware
RELEASEs the motor once settled (or at MOVE_TIMEOUT_MS = 5 s), so capture the
window you care about within `--duration`.

Example:
    python characterize_step.py --motor 0 --from 0.045 --to 0.055 --duration 2.0
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

SETTLE_TIME = 2.0  # seconds to let the start position seat before stepping

# Outputs are collected under <repo>/generated_files/<test>/ so runs don't litter
# the working directory. Anchored to the repo (not cwd) so it lands in the same
# place regardless of where the script is launched from.
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
DEFAULT_OUTDIR = os.path.join(REPO_ROOT, "generated_files")
TEST_NAME = "step_response"


def _mm(x):
    """Format a meters value as a filename-safe millimeter token (0.055 -> '55mm')."""
    return f"{x * 1e3:g}".replace(".", "p") + "mm"


def build_output_path(outdir, motor, start, target):
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    name = (f"{TEST_NAME}_motor{motor:02d}_"
            f"{_mm(start)}-{_mm(target)}_{stamp}.csv")
    return os.path.join(outdir, TEST_NAME, name)


def summarize(samples, motor, start, target):
    """Print simple step-response metrics for the target motor.

    samples: list of (t_s, [12 positions]).
    """
    if not samples:
        return
    t = [s[0] for s in samples]
    y = [s[1][motor] for s in samples]
    final = y[-1]
    span = target - start
    print(f"  start={start:.5f}  target={target:.5f}  final={final:.5f} m  "
          f"steady-state error={ (target - final) * 1e3:+.3f} mm")

    if abs(span) > 1e-6:
        # Rise time: first crossing of 90% of the commanded span.
        thresh = start + 0.9 * span
        crossed = None
        for ti, yi in zip(t, y):
            if (span > 0 and yi >= thresh) or (span < 0 and yi <= thresh):
                crossed = ti
                break
        print(f"  rise time to 90%: "
              f"{'%.3f s' % crossed if crossed is not None else 'not reached'}")

        # Overshoot relative to the commanded span, past the target.
        if span > 0:
            peak = max(y)
            overshoot = (peak - target) / span
        else:
            peak = min(y)
            overshoot = (target - peak) / (-span)
        print(f"  overshoot: {max(overshoot, 0.0) * 100:.1f}% "
              f"(peak {peak:.5f} m)")

    dt = [t[i + 1] - t[i] for i in range(len(t) - 1)]
    if dt:
        rate = (len(t) - 1) / (t[-1] - t[0]) if t[-1] > t[0] else 0.0
        print(f"  {len(samples)} samples, mean rate {rate:.0f} Hz "
              f"(dt {min(dt) * 1e3:.1f}-{max(dt) * 1e3:.1f} ms)")


def plot_step(samples, motor, start, target):
    """Show the measured trajectory of the stepped motor vs its target."""
    import matplotlib.pyplot as plt

    t = [s[0] for s in samples]
    y = [s[1][motor] * 1e3 for s in samples]  # mm

    fig, ax = plt.subplots(figsize=(9, 5))
    # Other joints (should stay flat at baseline) as faint context.
    for j in range(NUM_MOTORS):
        if j == motor:
            continue
        ax.plot(t, [s[1][j] * 1e3 for s in samples], color="0.85", lw=0.8, zorder=1)
    ax.plot(t, y, color="C0", lw=1.5, marker=".", ms=3, label=f"motor {motor} position", zorder=3)
    ax.axhline(target * 1e3, color="C3", ls="--", lw=1.2, label="target", zorder=2)
    ax.axhline(start * 1e3, color="0.5", ls=":", lw=1.0, label="start", zorder=2)
    ax.set_xlabel("time since step command (s)")
    ax.set_ylabel("joint position (mm)")
    ax.set_title(f"Closed-loop step response  motor {motor}: "
                 f"{start * 1e3:g} -> {target * 1e3:g} mm")
    ax.legend(loc="best")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    plt.show()


def run(port, board, motor, start, target, baseline, duration, out, outdir, show_plot=True):
    out_path = out or build_output_path(outdir, motor, start, target)
    lo, hi = MIN_JOINT_POS, MAX_JOINT_POS
    for name, val in (("from", start), ("to", target), ("baseline", baseline)):
        if not (lo <= val <= hi):
            raise SystemExit(
                f"{name}={val} outside joint range [{lo}, {hi}] "
                f"(the firmware would clip it; pick something inside)"
            )
    if not (0 <= motor < NUM_MOTORS):
        raise SystemExit(f"--motor must be 0..{NUM_MOTORS - 1}, got {motor}")

    env, agent = open_board(port, board)
    print(f"opening {port}, using board id {env.active_ids[0]}")
    print(f"motor {motor}: step {start} -> {target} m over {duration} s, "
          f"others held at {baseline} m")

    samples = []  # (t_s, [12 positions])
    try:
        # Seat the start pose and let it settle before the step.
        start_vec = [baseline] * NUM_MOTORS
        start_vec[motor] = start
        agent.move_joint_position(start_vec)
        time.sleep(SETTLE_TIME)

        # Fire the step, then poll as fast as possible for `duration`.
        step_vec = [baseline] * NUM_MOTORS
        step_vec[motor] = target
        t0 = time.perf_counter()
        agent.move_joint_position(step_vec)  # t=0 is the moment the command goes out
        while True:
            positions = agent.get_joint_positions()
            t = time.perf_counter() - t0
            samples.append((t, list(positions)))
            if t >= duration:
                break
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
        w.writerow([f"# {TEST_NAME} motor={motor} from={start} to={target} "
                    f"baseline={baseline} duration={duration} board={env.active_ids[0]}"])
        w.writerow(["t_s"] + [f"j{i}" for i in range(NUM_MOTORS)])
        for t, pos in samples:
            w.writerow([f"{t:.5f}"] + [f"{p:.6f}" for p in pos])
    print(f"wrote {len(samples)} samples to {out_path}")
    summarize(samples, motor, start, target)

    if show_plot:
        try:
            plot_step(samples, motor, start, target)
        except Exception as e:  # backend / viewer issue -- data is already saved
            print(f"skipping plot ({e}); data is in {out_path}")


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default=DEFAULT_PORT, help="serial port of the delta board")
    p.add_argument("--id", default=DEFAULT_BOARD,
                   help="board label (BOARD_REGISTRY) or raw id; omit to auto-discover")
    p.add_argument("--motor", type=int, required=True, help=f"motor index 0..{NUM_MOTORS - 1}")
    p.add_argument("--from", dest="start", type=float, default=0.045, help="start position (m)")
    p.add_argument("--to", dest="target", type=float, default=0.055, help="step target (m)")
    p.add_argument("--baseline", type=float, default=0.045, help="hold position for the other 11 motors (m)")
    p.add_argument("--duration", type=float, default=2.0, help="capture window after the step (s)")
    p.add_argument("--outdir", default=DEFAULT_OUTDIR,
                   help="base directory for generated files (a per-test subdir is added)")
    p.add_argument("--out", default=None,
                   help="explicit output CSV path; overrides the auto-generated descriptive name")
    p.add_argument("--no-plot", dest="plot", action="store_false",
                   help="skip the interactive plot (e.g. headless runs)")
    args = p.parse_args()
    run(args.port, args.id, args.motor, args.start, args.target, args.baseline,
        args.duration, args.out, args.outdir, show_plot=args.plot)


if __name__ == "__main__":
    main()
