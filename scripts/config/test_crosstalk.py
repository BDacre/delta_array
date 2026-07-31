"""Measure inter-motor crosstalk: move one motor, watch all the others.

Sweeping a single motor (like sweep_deadband/characterize_staircase) commands the
aggressor to its target and every other motor to `baseline`, which the firmware
PID-holds. If the other joints visibly move anyway, something couples them --
mechanical drag through shared structure, a transient the hold can't reject, or a
bad position reading. If that coupling is large it would corrupt an all-motors-
at-once deadband sweep, so verify it here first.

This drives the aggressor through the same up/down staircase but records ALL 12
positions, sampling continuously through each step so it catches transient
excursions (a victim dragged then pulled back looks settled at the endpoint but
moved a lot in between). For each victim it reports:

  * peak |dev|  -- worst deviation from its at-rest position at ANY sample during
                   the sweep (the "visible movement").
  * final |dev| -- deviation once the aggressor settled (does it stay displaced?).

Compare against the ADC noise floor (~1-2 counts, 0.06-0.12 mm at
ADC_TO_POSITION = 6e-5): deviations under that are just measurement jitter;
anything well above is real crosstalk.

    python test_crosstalk.py --name 4                 # aggressor = motor 0
    python test_crosstalk.py --name 4 --motor 10      # the stiff one
"""

import argparse
import csv
import os
import time
from datetime import datetime

from delta_control import (
    apply_calibration,
    calibration_path,
    load_calibration,
    load_calibration_file,
    open_board,
)
from delta_control.constants import MAX_JOINT_POS, MIN_JOINT_POS, NUM_MOTORS

from characterize_staircase import (
    frange,
    SETTLE_EPS,
    SETTLE_STABLE_SAMPLES,
    SETTLE_POLL_DT,
    SETTLE_TIMEOUT,
)

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
DEFAULT_OUTDIR = os.path.join(REPO_ROOT, "generated_files")
TEST_NAME = "crosstalk"

# ADC noise floor: 1-2 counts. Deviations below this are jitter, not crosstalk.
NOISE_FLOOR_M = 1.2e-4


def load_bias_calib(board_id, name, file):
    """Load bias calibration (explicit file > --name > by chip id), or None."""
    if file:
        return load_calibration_file(file)
    if name is not None:
        path = calibration_path(name)
        if not os.path.exists(path):
            raise SystemExit(f"no calibration file {path} (for --name {name})")
        return load_calibration_file(path)
    return load_calibration(board_id)


def settle_all(agent, ref_motor, rest, timeout=SETTLE_TIMEOUT):
    """Drive is already commanded; poll every joint until `ref_motor` settles.

    Samples all 12 positions each poll. Returns (final_positions, peak_dev) where
    peak_dev[j] is the largest |pos[j] - rest[j]| seen at any sample during the
    step -- i.e. the transient excursion, not just the endpoint.
    """
    peak_dev = [0.0] * NUM_MOTORS
    last = None
    stable = 0
    pos = agent.get_joint_positions()
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < timeout:
        pos = agent.get_joint_positions()
        for j in range(NUM_MOTORS):
            d = abs(pos[j] - rest[j])
            if d > peak_dev[j]:
                peak_dev[j] = d
        if last is not None and abs(pos[ref_motor] - last) < SETTLE_EPS:
            stable += 1
            if stable >= SETTLE_STABLE_SAMPLES:
                break
        else:
            stable = 0
        last = pos[ref_motor]
        time.sleep(SETTLE_POLL_DT)
    return pos, peak_dev


def run(port, board, motor, start, stop, step, baseline, deadband_mm,
        brake, name, file, outdir, show_plot=True):
    for label, val in (("start", start), ("stop", stop), ("baseline", baseline)):
        if not (MIN_JOINT_POS <= val <= MAX_JOINT_POS):
            raise SystemExit(f"{label}={val} outside joint range "
                             f"[{MIN_JOINT_POS}, {MAX_JOINT_POS}]")

    up = frange(start, stop, step)
    sweep = [("up", t) for t in up] + [("down", t) for t in list(reversed(up))[1:]]

    env, agent = open_board(port, board)
    board_id = env.active_ids[0]
    print(f"board {board_id}, aggressor = motor {motor}, "
          f"watching all {NUM_MOTORS} joints")

    rows = []  # (agg, agg_target, victim, rest, final, final_dev, peak_dev)
    peak_overall = [0.0] * NUM_MOTORS
    final_overall = [0.0] * NUM_MOTORS
    try:
        # Reproduce the sweep's board state: bias (if available) + brake mode +
        # the chosen deadband, so crosstalk is measured under real conditions.
        calib = load_bias_calib(board_id, name, file)
        if calib is not None:
            n = apply_calibration(agent, calib)
            print(f"applied bias to {n} motor(s)")
        agent.set_config(deadband=deadband_mm / 1e3, brake_at_setpoint=brake)
        print(f"deadband = {deadband_mm:g} mm, brake_at_setpoint = {brake}")

        # Rest reference: seat EVERY joint at baseline and let it settle; victim
        # deviations are measured against this.
        agent.move_joint_position([baseline] * NUM_MOTORS)
        rest, _ = settle_all(agent, motor, [baseline] * NUM_MOTORS)
        print("rest positions (mm): " +
              " ".join(f"{p * 1e3:.2f}" for p in rest))

        for direction, target in sweep:
            vec = [baseline] * NUM_MOTORS
            vec[motor] = target
            agent.move_joint_position(vec)
            final, peak_dev = settle_all(agent, motor, rest)
            for j in range(NUM_MOTORS):
                rows.append((motor, target, j, rest[j], final[j],
                             final[j] - rest[j], peak_dev[j]))
                if j != motor:
                    peak_overall[j] = max(peak_overall[j], peak_dev[j])
                    final_overall[j] = max(final_overall[j],
                                           abs(final[j] - rest[j]))
            victims = [peak_dev[j] * 1e3 for j in range(NUM_MOTORS) if j != motor]
            print(f"    {direction:4s} agg->{target:.4f}  "
                  f"worst victim peak = {max(victims):.2f} mm")
    finally:
        print("\nreturning to baseline, closing port")
        try:
            agent.move_joint_position([baseline] * NUM_MOTORS)
            time.sleep(0.5)
        finally:
            agent.close()

    out_path = os.path.join(outdir, TEST_NAME,
                            f"{TEST_NAME}_agg{motor:02d}_"
                            f"{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow([f"# {TEST_NAME} aggressor={motor} baseline={baseline} "
                    f"deadband_mm={deadband_mm} brake={brake} board={board_id}"])
        w.writerow(["aggressor", "agg_target_m", "victim", "rest_m",
                    "final_m", "final_dev_m", "peak_dev_m"])
        w.writerows(rows)
    print(f"wrote {len(rows)} rows to {out_path}")

    # Verdict table: worst-case victim deviation vs the noise floor.
    print(f"\naggressor motor {motor} swept {start * 1e3:g}->{stop * 1e3:g} mm "
          f"({(stop - start) * 1e3:g} mm travel). Victim deviation:")
    hdr = f"{'victim':>6}  {'peak |dev| mm':>14}  {'final |dev| mm':>15}  {'verdict':>10}"
    print(hdr)
    print("-" * len(hdr))
    for j in range(NUM_MOTORS):
        if j == motor:
            continue
        peak_mm = peak_overall[j] * 1e3
        final_mm = final_overall[j] * 1e3
        verdict = "CROSSTALK" if peak_overall[j] > 2 * NOISE_FLOOR_M else (
            "borderline" if peak_overall[j] > NOISE_FLOOR_M else "noise")
        print(f"{j:>6}  {peak_mm:>14.2f}  {final_mm:>15.2f}  {verdict:>10}")
    worst = max((peak_overall[j] for j in range(NUM_MOTORS) if j != motor),
                default=0.0)
    print(f"\nnoise floor ~{NOISE_FLOOR_M * 1e3:.2f} mm; worst victim peak "
          f"{worst * 1e3:.2f} mm ({worst / NOISE_FLOOR_M:.1f}x floor)")
    if worst > 2 * NOISE_FLOOR_M:
        print("=> real crosstalk: an all-motors parallel sweep is NOT safe as-is")
    else:
        print("=> within noise: parallel sweep should be fine")

    if show_plot:
        try:
            _plot(rows, motor, start, stop)
        except Exception as e:
            print(f"skipping plot ({e})")


def _plot(rows, motor, start, stop):
    """Bar chart of peak victim deviation per motor + noise-floor line."""
    import matplotlib.pyplot as plt

    peak = {}
    for agg, _t, j, _r, _f, _fd, pd in rows:
        if j != motor:
            peak[j] = max(peak.get(j, 0.0), pd)
    js = sorted(peak)
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.bar([str(j) for j in js], [peak[j] * 1e3 for j in js], color="C3")
    ax.axhline(NOISE_FLOOR_M * 1e3, color="0.5", ls="--",
               label=f"noise floor ~{NOISE_FLOOR_M * 1e3:.2f} mm")
    ax.set_xlabel("victim motor")
    ax.set_ylabel("peak deviation (mm)")
    ax.set_title(f"crosstalk: motor {motor} swept {start * 1e3:g}->{stop * 1e3:g} mm")
    ax.legend()
    ax.grid(True, axis="y", alpha=0.3)
    fig.tight_layout()
    plt.show()


def main():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default=None, help="serial port of the delta board")
    p.add_argument("--id", default=None,
                   help="board label (BOARD_REGISTRY) or raw id; omit to auto-discover")
    p.add_argument("--motor", type=int, default=0,
                   help=f"aggressor motor to move, 0..{NUM_MOTORS - 1} (default 0)")
    p.add_argument("--start", type=float, default=0.04, help="sweep start (m)")
    p.add_argument("--stop", type=float, default=0.06, help="sweep end (m)")
    p.add_argument("--step", type=float, default=0.002, help="increment (m)")
    p.add_argument("--baseline", type=float, default=0.05,
                   help="hold position for the victim motors (m)")
    p.add_argument("--deadband", type=float, default=0.8,
                   help="deadband to set on all motors, in mm (default 0.8)")
    p.add_argument("--brake", default=True, action=argparse.BooleanOptionalAction,
                   help="brake at setpoint (default on, matches the sweep)")
    p.add_argument("--name", default=None,
                   help="calibration name for bias (config/calibration/board_<name>.json)")
    p.add_argument("--file", default=None, help="explicit bias calibration JSON path")
    p.add_argument("--outdir", default=DEFAULT_OUTDIR,
                   help="base directory for generated files")
    p.add_argument("--no-plot", dest="plot", action="store_false",
                   help="skip the bar chart")
    args = p.parse_args()

    if not (0 <= args.motor < NUM_MOTORS):
        raise SystemExit(f"--motor must be 0..{NUM_MOTORS - 1}, got {args.motor}")

    run(args.port, args.id, args.motor, args.start, args.stop, args.step,
        args.baseline, args.deadband, args.brake, args.name, args.file,
        args.outdir, show_plot=args.plot)


if __name__ == "__main__":
    main()
