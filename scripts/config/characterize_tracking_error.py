"""End-effector tracking error along a horizontal (tangential) path.

Motivation: open-loop manipulation that needs horizontal travel (in the xy
plane, tangential to the vertical piston axes) relies on the three prismatic
actuators differencing their heights. If those actuators under/overshoot, the
tip lands off its commanded xy point. This script drives one delta through a
horizontal path and records, at each waypoint, where the tip was *commanded* vs
where it actually ended up, so the horizontal tracking error can be quantified.

There is no external tip tracker on the array, so "actual position" is
reconstructed from the only feedback the boards expose: the per-motor encoder
heights (SetTelemetry / pose_resp). At each waypoint we:

  1. IK the target tip point -> 3 actuator heights, command them (the other 3
     deltas on the board are held at home so the board stays put).
  2. Let the PID settle, then read the measured actuator heights.
  3. FK the *measured* heights back to a tip point -> the actual tip position.
  4. Error = actual - target, split into a horizontal magnitude sqrt(ex^2+ey^2)
     (the quantity of interest) and a z component.

This isolates real actuator tracking error from kinematic modelling error only
insofar as the same IK/FK model is used both ways -- a perfectly tracked joint
gives zero error here regardless of model accuracy, so what it measures is
genuinely the servo/open-loop tracking, expressed in tip-space millimetres.

Targets are validated against the joint limits before commanding: an unreachable
point would be silently clipped by move_joint_position and masquerade as
tracking error, so the requested amplitude is auto-shrunk until every waypoint is
reachable.

Examples:
    python characterize_tracking_error.py                     # board 0, delta 0, cross pattern
    python characterize_tracking_error.py --pattern circle --amp 0.008
    python characterize_tracking_error.py --id board2 --delta 1 --pattern x
    python characterize_tracking_error.py --no-calibration    # measure on firmware defaults
"""

import argparse
import csv
import os
import time
from datetime import datetime

import numpy as np

from delta_control import load_calibration, apply_calibration, open_board
from delta_control.constants import (
    HOME_POSITION,
    MAX_JOINT_POS,
    MIN_JOINT_POS,
    MOTORS_PER_DELTA,
    NUM_MOTORS,
)
from delta_control.delta_array_env import delta  # shared PrismaticDelta (IK/FK)

DEFAULT_PORT = None   # None auto-detects the board's port by scanning /dev/ttyACM*
DEFAULT_BOARD = None  # None auto-discovers; set a BOARD_REGISTRY label or raw id to skip

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
DEFAULT_OUTDIR = os.path.join(REPO_ROOT, "generated_files")
TEST_NAME = "tracking_error"

# Keep commanded joints this far inside [MIN, MAX] so a settled joint never sits
# on a hard limit (where it can't correct further and the error is an artefact).
BOUND_MARGIN = 0.003     # m
DEFAULT_AMP = 0.007      # m, horizontal half-amplitude of the sweep
DEFAULT_N = 15           # waypoints per axis / around the pattern
DEFAULT_SETTLE_S = 0.8   # s, PID settle before sampling
DEFAULT_SAMPLES = 5      # telemetry reads averaged per waypoint
MIN_AMP = 0.001          # m, give up shrinking below this


def home_joints():
    """The 3 actuator heights for a single delta at the home tip pose."""
    return np.asarray(delta.ik(HOME_POSITION), dtype=float)


def target_reachable(target):
    """True if the tip point maps to actuator heights inside the safe band."""
    h = delta.ik(target)
    if np.any(np.isnan(h)):
        return False
    lo = MIN_JOINT_POS + BOUND_MARGIN
    hi = MAX_JOINT_POS - BOUND_MARGIN
    return bool(np.all(h >= lo) and np.all(h <= hi))


def gen_waypoints(pattern, amp, z, center, n):
    """Build the ordered list of tip target points (x, y, z) for a pattern.

    All patterns are centred on `center` at a fixed height `z`, so every point
    differs from its neighbour only in the horizontal plane -- the travel is
    purely tangential to the piston axes, which is the regime under test.
    `n` is the number of waypoints per axis / side / around the pattern.
    """
    cx, cy = center
    if pattern == "x":
        return [(cx + x, cy, z) for x in np.linspace(-amp, amp, n)]
    if pattern == "y":
        return [(cx, cy + y, z) for y in np.linspace(-amp, amp, n)]
    if pattern == "cross":
        pts = [(cx + x, cy, z) for x in np.linspace(-amp, amp, n)]
        pts.append((cx, cy, z))  # recentre before the orthogonal sweep
        pts += [(cx, cy + y, z) for y in np.linspace(-amp, amp, n)]
        return pts
    if pattern == "square":
        # Trace the perimeter: BL -> BR -> TR -> TL -> BL, n points per side.
        corners = [(-amp, -amp), (amp, -amp), (amp, amp), (-amp, amp), (-amp, -amp)]
        pts = []
        for (x0, y0), (x1, y1) in zip(corners, corners[1:]):
            for t in np.linspace(0, 1, n, endpoint=False):
                pts.append((cx + x0 + (x1 - x0) * t, cy + y0 + (y1 - y0) * t, z))
        pts.append((cx - amp, cy - amp, z))
        return pts
    if pattern == "circle":
        th = np.linspace(0, 2 * np.pi, n, endpoint=True)
        return [(cx + amp * np.cos(t), cy + amp * np.sin(t), z) for t in th]
    raise ValueError(f"unknown pattern {pattern!r}")


def fit_amplitude(pattern, amp, z, center, n):
    """Shrink amp until every waypoint is reachable; return (amp, waypoints)."""
    while amp >= MIN_AMP:
        pts = gen_waypoints(pattern, amp, z, center, n)
        if all(target_reachable(p) for p in pts):
            return amp, pts
        amp *= 0.9
    raise SystemExit(
        f"pattern {pattern!r} not reachable even at {MIN_AMP * 1e3:.1f} mm at "
        f"z={z:.4f}; try a z nearer home ({HOME_POSITION[2]:.4f})"
    )


def measure_waypoint(agent, delta_index, target, settle_s, samples):
    """Command one tip target on `delta_index`, settle, and measure the result.

    Returns a dict with the target, the measured (settled, averaged) actuator
    heights, the FK-reconstructed actual tip, the errors, and the firmware's own
    PID error / applied PWM for the three motors.
    """
    s = delta_index * MOTORS_PER_DELTA
    target_j = np.asarray(delta.ik(target), dtype=float)

    # Hold the other three deltas at home; only the tested delta tracks the path.
    vec = np.tile(home_joints(), NUM_MOTORS // MOTORS_PER_DELTA)
    vec[s:s + MOTORS_PER_DELTA] = target_j
    agent.move_joint_position(vec.tolist())
    time.sleep(settle_s)

    pos_acc = np.zeros(MOTORS_PER_DELTA)
    err_acc = np.zeros(MOTORS_PER_DELTA)
    pwm_acc = np.zeros(MOTORS_PER_DELTA)
    got = 0
    for _ in range(samples):
        tel = agent.get_telemetry()
        if tel is None:
            continue
        pos_acc += np.asarray(tel["position"][s:s + MOTORS_PER_DELTA])
        err_acc += np.asarray(tel["error"][s:s + MOTORS_PER_DELTA])
        pwm_acc += np.asarray(tel["pwm"][s:s + MOTORS_PER_DELTA])
        got += 1
    if got == 0:
        # Telemetry unsupported/failing -> fall back to a plain pose read.
        measured_j = np.asarray(agent.get_joint_positions()[s:s + MOTORS_PER_DELTA])
        pid_err = np.full(MOTORS_PER_DELTA, np.nan)
        pwm = np.full(MOTORS_PER_DELTA, np.nan)
    else:
        measured_j = pos_acc / got
        pid_err = err_acc / got
        pwm = pwm_acc / got

    measured_ee = np.asarray(delta.fk(measured_j), dtype=float).reshape(3)
    target_ee = np.asarray(target, dtype=float)
    ee_err = measured_ee - target_ee
    return {
        "target_ee": target_ee,
        "measured_ee": measured_ee,
        "target_j": target_j,
        "measured_j": measured_j,
        "joint_err": measured_j - target_j,
        "pid_err": pid_err,
        "pwm": pwm,
        "ee_err": ee_err,
        "horiz_err": float(np.hypot(ee_err[0], ee_err[1])),
    }


def build_output_path(outdir, board_id, delta_index, pattern):
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    tag = f"board{board_id}_delta{delta_index}_{pattern}"
    return os.path.join(outdir, TEST_NAME, f"{TEST_NAME}_{tag}_{stamp}.csv")


def run(port, board, delta_index, pattern, amp, z, center, n, settle_s, samples,
        calibrate, outdir, out, show_plot=True):
    amp, waypoints = fit_amplitude(pattern, amp, z, center, n)

    env, agent = open_board(port, board)
    board_id = env.active_ids[0]
    print(f"opening {port}, using board id {board_id}, delta {delta_index}")
    print(f"pattern={pattern} amp={amp * 1e3:.2f} mm z={z:.4f} m "
          f"waypoints={len(waypoints)} settle={settle_s}s samples={samples}")

    rows = []
    results = []
    try:
        if calibrate:
            calib = load_calibration(board_id)
            if calib:
                n = apply_calibration(agent, calib)
                print(f"applied calibration to {n} motor(s)")
            else:
                print(f"no calibration file for board {board_id}; running on firmware defaults")
        else:
            print("skipping calibration (--no-calibration)")

        print("homing...")
        env.reset()
        time.sleep(1.5)

        # Seat at the first waypoint with extra settle so the path starts clean.
        measure_waypoint(agent, delta_index, waypoints[0], max(settle_s, 1.0), samples)

        for i, wp in enumerate(waypoints):
            r = measure_waypoint(agent, delta_index, wp, settle_s, samples)
            results.append(r)
            te, me, je = r["target_ee"], r["measured_ee"], r["joint_err"]
            rows.append([
                i,
                *[f"{v:.6f}" for v in te],
                *[f"{v:.6f}" for v in me],
                *[f"{v:.6f}" for v in r["target_j"]],
                *[f"{v:.6f}" for v in r["measured_j"]],
                *[f"{v:.6f}" for v in je],
                *[f"{v:.6f}" for v in r["pid_err"]],
                *[f"{v:.1f}" for v in r["pwm"]],
                f"{r['ee_err'][0]:.6f}", f"{r['ee_err'][1]:.6f}", f"{r['ee_err'][2]:.6f}",
                f"{r['horiz_err']:.6f}",
            ])
            print(f"  wp {i:2d}  tgt=({te[0] * 1e3:+6.2f},{te[1] * 1e3:+6.2f}) mm  "
                  f"horiz_err={r['horiz_err'] * 1e3:5.2f} mm  "
                  f"z_err={r['ee_err'][2] * 1e3:+5.2f} mm")
    finally:
        print("returning home, closing port")
        try:
            env.reset()
            time.sleep(0.5)
        finally:
            agent.close()

    horiz = np.array([r["horiz_err"] for r in results])
    zerr = np.array([abs(r["ee_err"][2]) for r in results])
    print(f"\nhorizontal error: mean={horiz.mean() * 1e3:.2f} mm  "
          f"max={horiz.max() * 1e3:.2f} mm  (n={len(horiz)})")
    print(f"z error:          mean={zerr.mean() * 1e3:.2f} mm  max={zerr.max() * 1e3:.2f} mm")

    out_path = out or build_output_path(outdir, board_id, delta_index, pattern)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow([f"# {TEST_NAME} board={board_id} delta={delta_index} "
                    f"pattern={pattern} amp_m={amp} z_m={z} center={center} "
                    f"settle_s={settle_s} samples={samples} calibrated={calibrate}"])
        w.writerow([
            "wp",
            "tgt_x", "tgt_y", "tgt_z",
            "meas_x", "meas_y", "meas_z",
            "tgt_j0", "tgt_j1", "tgt_j2",
            "meas_j0", "meas_j1", "meas_j2",
            "jerr0", "jerr1", "jerr2",
            "pid_err0", "pid_err1", "pid_err2",
            "pwm0", "pwm1", "pwm2",
            "ee_err_x", "ee_err_y", "ee_err_z",
            "horiz_err",
        ])
        w.writerows(rows)
    print(f"wrote {len(rows)} waypoints to {out_path}")

    if show_plot:
        try:
            plot_tracking(results, pattern, board_id, delta_index, out_path)
        except Exception as e:
            print(f"skipping plot ({e}); data is in {out_path}")


def plot_tracking(results, pattern, board_id, delta_index, csv_path):
    """Target vs measured tip path, plus horizontal/joint error per waypoint."""
    import matplotlib.pyplot as plt

    tgt = np.array([r["target_ee"] for r in results])
    meas = np.array([r["measured_ee"] for r in results])
    horiz = np.array([r["horiz_err"] for r in results]) * 1e3
    ee_err = np.array([r["ee_err"] for r in results]) * 1e3
    jerr = np.array([r["joint_err"] for r in results]) * 1e3

    fig, axes = plt.subplots(2, 2, figsize=(13, 9))
    fig.suptitle(f"tip tracking error  board {board_id} delta {delta_index}  "
                 f"pattern={pattern}")

    # (0,0) XY plane: commanded vs actual path, with error whiskers.
    ax = axes[0, 0]
    ax.plot(tgt[:, 0] * 1e3, tgt[:, 1] * 1e3, "-o", ms=4, color="tab:blue",
            label="target", zorder=2)
    ax.plot(meas[:, 0] * 1e3, meas[:, 1] * 1e3, "-o", ms=4, color="tab:red",
            label="measured (FK)", zorder=3)
    for t, m in zip(tgt, meas):
        ax.plot([t[0] * 1e3, m[0] * 1e3], [t[1] * 1e3, m[1] * 1e3],
                color="0.6", lw=0.8, zorder=1)
    ax.set_xlabel("x (mm)")
    ax.set_ylabel("y (mm)")
    ax.set_title("horizontal plane: commanded vs actual")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")

    # (0,1) horizontal error magnitude per waypoint.
    ax = axes[0, 1]
    ax.plot(horiz, "-o", ms=4, color="tab:purple")
    ax.axhline(horiz.mean(), color="0.5", ls="--", lw=1,
               label=f"mean {horiz.mean():.2f} mm")
    ax.set_xlabel("waypoint")
    ax.set_ylabel("horizontal error |xy| (mm)")
    ax.set_title("horizontal tracking error")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")

    # (1,0) per-axis EE error.
    ax = axes[1, 0]
    ax.plot(ee_err[:, 0], "-o", ms=3, label="x err")
    ax.plot(ee_err[:, 1], "-o", ms=3, label="y err")
    ax.plot(ee_err[:, 2], "-o", ms=3, label="z err")
    ax.axhline(0, color="0.6", lw=0.8)
    ax.set_xlabel("waypoint")
    ax.set_ylabel("tip error (mm)")
    ax.set_title("per-axis tip error (measured - target)")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")

    # (1,1) per-motor joint tracking error.
    ax = axes[1, 1]
    for m in range(MOTORS_PER_DELTA):
        ax.plot(jerr[:, m], "-o", ms=3, label=f"motor {delta_index * 3 + m}")
    ax.axhline(0, color="0.6", lw=0.8)
    ax.set_xlabel("waypoint")
    ax.set_ylabel("joint error (mm)")
    ax.set_title("per-actuator tracking error (measured - target)")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")

    fig.tight_layout(rect=(0, 0, 1, 0.97))
    png_path = os.path.splitext(csv_path)[0] + ".png"
    fig.savefig(png_path, dpi=120)
    print(f"saved plot to {png_path}")
    plt.show()


def main():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default=DEFAULT_PORT, help="serial port of the delta board")
    p.add_argument("--id", default=DEFAULT_BOARD,
                   help="board label (BOARD_REGISTRY) or raw id; omit to auto-discover")
    p.add_argument("--delta", type=int, default=0,
                   help="which delta on the board to test, 0..3 (default 0)")
    p.add_argument("--pattern", default="cross",
                   choices=["cross", "x", "y", "square", "circle"],
                   help="horizontal travel pattern (default cross)")
    p.add_argument("--amp", type=float, default=DEFAULT_AMP,
                   help="horizontal half-amplitude / radius in m (auto-shrunk if unreachable)")
    p.add_argument("--z", type=float, default=HOME_POSITION[2],
                   help="tip height held during the sweep (m); default = home z")
    p.add_argument("--cx", type=float, default=0.0, help="pattern centre x (m)")
    p.add_argument("--cy", type=float, default=0.0, help="pattern centre y (m)")
    p.add_argument("--n", type=int, default=DEFAULT_N,
                   help="waypoints per axis / around the pattern")
    p.add_argument("--settle", type=float, default=DEFAULT_SETTLE_S,
                   help="PID settle time before sampling each waypoint (s)")
    p.add_argument("--samples", type=int, default=DEFAULT_SAMPLES,
                   help="telemetry reads averaged per waypoint")
    p.add_argument("--no-calibration", dest="calibrate", action="store_false",
                   help="do not push the board's calibration first (measure on defaults)")
    p.add_argument("--outdir", default=DEFAULT_OUTDIR,
                   help="base directory for generated files (a per-test subdir is added)")
    p.add_argument("--out", default=None, help="explicit output CSV path")
    p.add_argument("--no-plot", dest="plot", action="store_false",
                   help="skip the interactive plot")
    args = p.parse_args()

    if not (0 <= args.delta <= 3):
        raise SystemExit(f"--delta must be 0..3, got {args.delta}")
    if args.n < 2:
        raise SystemExit(f"--n must be >= 2, got {args.n}")

    run(args.port, args.id, args.delta, args.pattern, args.amp, args.z,
        (args.cx, args.cy), args.n, args.settle, args.samples, args.calibrate,
        args.outdir, args.out, show_plot=args.plot)


if __name__ == "__main__":
    main()
