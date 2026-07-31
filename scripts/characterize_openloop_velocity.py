"""Open-loop velocity vs PWM for each motor, each direction.

Bypasses the PID (via the firmware SetPwmCommand) to measure how fast a joint
moves at a given PWM. Two uses:
  * the slope (velocity per PWM) characterizes the motor+drivetrain gain, and
  * the x-intercept of velocity-vs-PWM is a second estimate of the deadzone /
    static-friction breakaway (complements characterize_breakaway.py).

Method, per motor / direction / PWM level:
  1. Seat the joint at mid-travel (PID move) so there is room to slew.
  2. Drive the motor open-loop at the fixed PWM for --window-ms while sampling
     position as fast as the serial link allows (firmware auto-releases at the
     end of the window).
  3. Velocity = least-squares slope of position vs time, over the samples after
     an initial --skip-s (to drop the acceleration transient).

Safety: the firmware auto-releases at the window end; sampling also stops early
if the joint nears [MIN+margin, MAX-margin]. Window/PWM defaults keep worst-case
travel (~7 mm at full speed) well inside the range from a mid-travel seat.

Example:
    python characterize_openloop_velocity.py --motor 0
    python characterize_openloop_velocity.py --motor 0 --pwm-levels 80,140,200,255
"""

import argparse
import csv
import os
import time
from datetime import datetime

import numpy as np

from delta_control import open_board
from delta_control.constants import (
    MAX_JOINT_POS,
    MIN_JOINT_POS,
    NUM_MOTORS,
)

DEFAULT_PORT = None  # None auto-detects the board's port by scanning /dev/ttyACM*
DEFAULT_BOARD = None  # None auto-discovers; set a BOARD_REGISTRY label or raw id to skip

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_OUTDIR = os.path.join(REPO_ROOT, "generated_files")
TEST_NAME = "openloop_velocity"

SEAT_POSITION = 0.05     # m, mid-travel seat before each pulse
SEAT_SETTLE_S = 1.5      # s, let the PID seat the joint
BOUND_MARGIN = 0.004     # m, keep this far inside [MIN, MAX]
DEFAULT_PWM_LEVELS = [80, 120, 160, 200, 255]


def build_output_path(outdir, motors):
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    tag = f"motor{motors[0]:02d}" if len(motors) == 1 else f"{len(motors)}motors"
    return os.path.join(outdir, TEST_NAME, f"{TEST_NAME}_{tag}_{stamp}.csv")


def seat(agent, motor, baseline, position):
    vec = [baseline] * NUM_MOTORS
    vec[motor] = position
    agent.move_joint_position(vec)
    time.sleep(SEAT_SETTLE_S)


def measure_velocity(agent, motor, signed_pwm, window_ms, skip_s):
    """Drive one motor at a fixed PWM for the window; return (velocity, samples).

    velocity is the least-squares slope (m/s) of position vs time over samples
    after skip_s. samples is a list of (t_s, pos_m). Sampling stops early if the
    joint nears a travel bound.
    """
    lo = MIN_JOINT_POS + BOUND_MARGIN
    hi = MAX_JOINT_POS - BOUND_MARGIN
    samples = []
    t0 = time.perf_counter()
    agent.set_motor_pwm(motor, signed_pwm, duration_ms=window_ms)  # t=0 = command out
    window_s = window_ms / 1000.0
    while True:
        pos = agent.get_joint_positions()[motor]
        t = time.perf_counter() - t0
        samples.append((t, pos))
        if t >= window_s or not (lo <= pos <= hi):
            break
    # Ensure the motor is released even if we broke out early.
    agent.stop()

    fit = [(t, p) for t, p in samples if t >= skip_s]
    if len(fit) >= 2:
        ts = np.array([t for t, _ in fit])
        ps = np.array([p for _, p in fit])
        slope = float(np.polyfit(ts, ps, 1)[0])
    else:
        slope = float("nan")
    return slope, samples


def run(port, board, motors, pwm_levels, window_ms, skip_s, baseline,
        out, outdir, show_plot=True):
    out_path = out or build_output_path(outdir, motors)

    env, agent = open_board(port, board)
    print(f"opening {port}, using board id {env.active_ids[0]}")
    print(f"open-loop velocity: motors {motors}, pwm levels {pwm_levels}, "
          f"window {window_ms} ms")

    rows = []  # (motor, signed_pwm, velocity_m_s, n_samples, travel_m)
    try:
        for motor in motors:
            print(f"  motor {motor}:")
            for sign in (+1, -1):
                for pwm in pwm_levels:
                    seat(agent, motor, baseline, SEAT_POSITION)
                    signed = sign * pwm
                    vel, samples = measure_velocity(agent, motor, signed,
                                                    window_ms, skip_s)
                    travel = samples[-1][1] - samples[0][1] if samples else 0.0
                    rows.append((motor, signed, vel, len(samples), travel))
                    print(f"    pwm={signed:+4d}  vel={vel * 1e3:+7.2f} mm/s  "
                          f"(travel {travel * 1e3:+.2f} mm, {len(samples)} samples)")
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
        w.writerow([f"# {TEST_NAME} motors={motors} pwm_levels={pwm_levels} "
                    f"window_ms={window_ms} skip_s={skip_s} baseline={baseline} "
                    f"board={env.active_ids[0]}"])
        w.writerow(["motor", "signed_pwm", "velocity_m_s", "n_samples", "travel_m"])
        w.writerows(rows)
    print(f"wrote {len(rows)} measurements to {out_path}")

    if show_plot:
        try:
            plot_velocity(rows, motors)
        except Exception as e:
            print(f"skipping plot ({e}); data is in {out_path}")


def plot_velocity(rows, motors):
    """Velocity (mm/s) vs signed PWM, one line per motor."""
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(9, 5))
    for motor in motors:
        pts = sorted((r[1], r[2] * 1e3) for r in rows if r[0] == motor)
        if pts:
            ax.plot([p for p, _ in pts], [v for _, v in pts],
                    marker="o", ms=4, label=f"motor {motor}")
    ax.axhline(0, color="0.6", lw=0.8)
    ax.axvline(0, color="0.6", lw=0.8)
    ax.set_xlabel("commanded PWM (signed: + = BACKWARD, - = FORWARD)")
    ax.set_ylabel("open-loop velocity (mm/s)")
    ax.set_title("open-loop velocity vs PWM "
                 "(x-intercept ~ deadzone / breakaway)")
    ax.legend(loc="best")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    plt.show()


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default=DEFAULT_PORT, help="serial port of the delta board")
    p.add_argument("--id", default=DEFAULT_BOARD,
                   help="board label (BOARD_REGISTRY) or raw id; omit to auto-discover")
    p.add_argument("--motor", type=int, default=None,
                   help=f"single motor 0..{NUM_MOTORS - 1}; omit to sweep all")
    p.add_argument("--pwm-levels", default=None,
                   help="comma-separated |PWM| levels (default 80,120,160,200,255)")
    p.add_argument("--window-ms", type=int, default=250,
                   help="open-loop drive window per measurement (ms)")
    p.add_argument("--skip-s", type=float, default=0.04,
                   help="seconds of initial transient to skip before fitting velocity")
    p.add_argument("--baseline", type=float, default=0.05,
                   help="hold position for the other 11 motors (m)")
    p.add_argument("--outdir", default=DEFAULT_OUTDIR,
                   help="base directory for generated files (a per-test subdir is added)")
    p.add_argument("--out", default=None, help="explicit output CSV path")
    p.add_argument("--no-plot", dest="plot", action="store_false",
                   help="skip the interactive plot")
    args = p.parse_args()

    if args.motor is not None:
        if not (0 <= args.motor < NUM_MOTORS):
            raise SystemExit(f"--motor must be 0..{NUM_MOTORS - 1}, got {args.motor}")
        motors = [args.motor]
    else:
        motors = list(range(NUM_MOTORS))

    if args.pwm_levels:
        pwm_levels = [int(x) for x in args.pwm_levels.split(",") if x.strip()]
    else:
        pwm_levels = DEFAULT_PWM_LEVELS

    run(args.port, args.id, motors, pwm_levels, args.window_ms, args.skip_s,
        args.baseline, args.out, args.outdir, show_plot=args.plot)


if __name__ == "__main__":
    main()
