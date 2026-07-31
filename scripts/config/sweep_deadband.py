"""Find the smallest usable settle deadband, on the brake+bias config.

Now that braking kills the post-release overshoot, the leftover ~0.5 mm tracking
error is mostly the deadband floor: the firmware stops correcting once |err| is
under deadband[i], so a big deadband leaves the joint parked short of target.
Shrinking it should tighten settling -- until it gets close to the ADC noise
floor (~1-2 counts, 0.06-0.12 mm at ADC_TO_POSITION = 6e-5 m/count), where the
joint starts chasing quantisation noise and the error stops improving (or the
worst-case error climbs again as a small limit cycle returns).

This sweeps a series of deadband values -- all on the winning brake+bias config
-- running a short up/down staircase at each and reporting per-motor tracking
metrics so you can pick each motor's knee: the smallest deadband before its
numbers stop improving. It re-applies the board's bias calibration and turns on
braking first, so the run is self-contained regardless of prior board state.

By default all 12 motors sweep in lockstep (same target every step = pure
translation, no differential load), so one run tunes the whole board ~12x faster
than one-at-a-time. This is valid because settling is measured at rest (no
current draw to couple motors) and each joint has independent feedback; only
breakaway characterization, which cares about transient current, needs isolation.
Pass --motor to isolate a single joint (e.g. the stiff motor 10).

Workflow:
    python sweep_deadband.py --name 4                 # all 12, one pass
    python sweep_deadband.py --name 4 --motor 10      # isolate the stiff outlier
then lock each chosen value into config/calibration/board_<name>.json (deadband
per motor, in meters) and re-push with apply_calibration.py --name 4 --brake.
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

# Reuse the staircase primitives so this measures settling exactly like the
# characterization runs we compare against.
from characterize_staircase import (
    frange,
    sweep_motor,
    wait_until_settled,
    SETTLE_EPS,
    SETTLE_STABLE_SAMPLES,
    SETTLE_POLL_DT,
    SETTLE_TIMEOUT,
)
from analyze_staircase import motor_metrics

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
DEFAULT_OUTDIR = os.path.join(REPO_ROOT, "generated_files")
TEST_NAME = "deadband_sweep"

# Default deadband ladder (mm), coarse->fine. 0.8 mm is today's firmware default;
# the floor is set by ADC noise (~0.06-0.12 mm), so there's no point below ~0.1.
DEFAULT_DEADBANDS_MM = [0.8, 0.5, 0.3, 0.2, 0.15, 0.1]


def build_output_path(outdir, motor, db_mm):
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    tag = f"{db_mm:g}".replace(".", "p")
    who = "allmotors" if motor is None else f"motor{motor:02d}"
    name = f"{TEST_NAME}_{who}_db{tag}mm_{stamp}.csv"
    return os.path.join(outdir, TEST_NAME, name)


def write_rows(path, rows, motor, db_mm, board_id):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        who = "all" if motor is None else motor
        w.writerow([f"# {TEST_NAME} motor={who} deadband_mm={db_mm} "
                    f"board={board_id} brake=on"])
        w.writerow(["motor", "direction", "commanded_m", "measured_m", "error_m"])
        w.writerows(rows)


def load_bias_calib(board_id, name, file):
    """Load the board's bias calibration (explicit file > --name > by chip id)."""
    if file:
        return load_calibration_file(file)
    if name is not None:
        path = calibration_path(name)
        if not os.path.exists(path):
            raise SystemExit(f"no calibration file {path} (for --name {name})")
        return load_calibration_file(path)
    calib = load_calibration(board_id)
    if calib is None:
        raise SystemExit(
            f"no calibration for board {board_id}; pass --name or --file "
            f"(the sweep re-applies bias so it must know the bias values)"
        )
    return calib


def wait_until_all_settled(agent, timeout=SETTLE_TIMEOUT):
    """Poll every joint until all readings stop changing, or until timeout.

    The all-motors analogue of characterize_staircase.wait_until_settled: gates
    each step on the slowest joint. Returns the last 12-vector of positions (m).
    """
    last = None
    stable = 0
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < timeout:
        pos = agent.get_joint_positions()
        if last is not None and all(
                abs(p - q) < SETTLE_EPS for p, q in zip(pos, last)):
            stable += 1
            if stable >= SETTLE_STABLE_SAMPLES:
                return pos
        else:
            stable = 0
        last = pos
        time.sleep(SETTLE_POLL_DT)
    return last


def sweep_all_motors(agent, sweep):
    """Run the up/down staircase on all 12 motors in lockstep; return result rows.

    Every motor is commanded to the same target each step (pure translation, no
    differential load) and all 12 settled positions are recorded per step, so one
    pass characterizes the whole board. Rows are (motor, direction, commanded_m,
    measured_m, error_m), matching sweep_motor's schema.
    """
    agent.move_joint_position([sweep[0][1]] * NUM_MOTORS)
    wait_until_all_settled(agent)

    rows = []
    for direction, target in sweep:
        agent.move_joint_position([target] * NUM_MOTORS)
        measured = wait_until_all_settled(agent)
        for i in range(NUM_MOTORS):
            rows.append((i, direction, target, measured[i], measured[i] - target))
        errs_mm = [(measured[i] - target) * 1e3 for i in range(NUM_MOTORS)]
        print(f"    {direction:4s} cmd={target:.5f}  "
              f"err[min..max]={min(errs_mm):+.2f}..{max(errs_mm):+.2f} mm")
    return rows


def recommend(results):
    """Heuristic knee: the smallest deadband still within 15%/25% of the best
    mean/max error seen. Below the noise floor those metrics stop improving (or
    max climbs as a limit cycle returns), so the smallest 'still-good' value is
    the sweet spot. The table is the real output; this is just a pointer."""
    best_mean = min(m["mean_abs"] for _, m in results)
    best_max = min(m["max_abs"] for _, m in results)
    good = [db for db, m in results
            if m["mean_abs"] <= 1.15 * best_mean and m["max_abs"] <= 1.25 * best_max]
    return min(good) if good else None


def report(runs, motors):
    """Print per-motor error-vs-deadband and a suggested per-motor knee.

    runs is a list of (db_mm, rows); rows carry every swept motor's points. For
    each motor we build its (deadband -> metrics) series, print it, and run the
    recommend() heuristic to suggest that motor's deadband.
    """
    picks = {}
    for motor in motors:
        series = [(db, motor_metrics([r for r in rows if r[0] == motor]))
                  for db, rows in runs]
        print(f"\nmotor {motor}  (brake+bias, varying deadband):")
        hdr = (f"{'deadband mm':>12}  {'mean|err| mm':>13}  "
               f"{'max|err| mm':>12}  {'hyst mm':>9}")
        print(hdr)
        print("-" * len(hdr))
        for db_mm, m in series:
            print(f"{db_mm:>12g}  {m['mean_abs']:>13.3f}  {m['max_abs']:>12.3f}  "
                  f"{m['hyst']:>9.3f}")
        picks[motor] = recommend(series)

    print("\nsuggested per-motor deadband (mm) "
          "-- smallest before settling stops improving; verify against tables:")
    for motor in motors:
        pick = picks[motor]
        print(f"  motor {motor:2d}: {pick:g} mm  ->  \"deadband\": {pick / 1e3:g}"
              if pick is not None else f"  motor {motor:2d}: (no clear knee)")
    print("\nlock these into config/calibration/board_<name>.json (per-motor "
          "\"deadband\", in meters), then re-run apply_calibration.py --name <n> --brake")


def run(port, board, motor, deadbands_mm, start, stop, step, baseline,
        name, file, outdir):
    for label, val in (("start", start), ("stop", stop), ("baseline", baseline)):
        if not (MIN_JOINT_POS <= val <= MAX_JOINT_POS):
            raise SystemExit(f"{label}={val} outside joint range "
                             f"[{MIN_JOINT_POS}, {MAX_JOINT_POS}]")

    up = frange(start, stop, step)
    sweep = [("up", t) for t in up] + [("down", t) for t in list(reversed(up))[1:]]

    env, agent = open_board(port, board)
    board_id = env.active_ids[0]
    who = "all motors (parallel)" if motor is None else f"motor {motor}"
    print(f"board {board_id}, sweeping {who}, deadbands {deadbands_mm} mm")

    runs = []
    try:
        # Put the board on the winning config: bias from calibration + braking on,
        # so the only thing changing between runs is the deadband.
        calib = load_bias_calib(board_id, name, file)
        n = apply_calibration(agent, calib)
        agent.set_config(brake_at_setpoint=True)
        print(f"applied bias to {n} motor(s), brake_at_setpoint=True")

        for db_mm in deadbands_mm:
            agent.set_config(deadband=db_mm / 1e3)  # all motors
            print(f"\n  deadband = {db_mm:g} mm:")
            if motor is None:
                rows = sweep_all_motors(agent, sweep)
            else:
                rows = sweep_motor(agent, motor, sweep, baseline)
            runs.append((db_mm, rows))
            write_rows(build_output_path(outdir, motor, db_mm), rows, motor,
                       db_mm, board_id)
    finally:
        print("\nreturning to baseline, closing port")
        try:
            agent.move_joint_position([baseline] * NUM_MOTORS)
            time.sleep(0.5)
        finally:
            agent.close()

    motors = list(range(NUM_MOTORS)) if motor is None else [motor]
    report(runs, motors)


def main():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default=None, help="serial port of the delta board")
    p.add_argument("--id", default=None,
                   help="board label (BOARD_REGISTRY) or raw id; omit to auto-discover")
    p.add_argument("--motor", type=int, default=None,
                   help=f"single motor to isolate, 0..{NUM_MOTORS - 1}; omit to "
                        f"sweep all 12 in lockstep (~12x faster, one run tunes all)")
    p.add_argument("--deadbands", type=float, nargs="+", default=DEFAULT_DEADBANDS_MM,
                   metavar="MM", help="deadband values to test, in mm "
                        f"(default {DEFAULT_DEADBANDS_MM})")
    p.add_argument("--start", type=float, default=0.04, help="sweep start (m)")
    p.add_argument("--stop", type=float, default=0.06, help="sweep end (m)")
    p.add_argument("--step", type=float, default=0.001, help="increment (m)")
    p.add_argument("--baseline", type=float, default=0.05,
                   help="hold position for the other motors (m)")
    p.add_argument("--name", default=None,
                   help="calibration name for the bias values (config/calibration/"
                        "board_<name>.json); the sweep re-applies bias first")
    p.add_argument("--file", default=None,
                   help="explicit bias calibration JSON path (overrides --name)")
    p.add_argument("--outdir", default=DEFAULT_OUTDIR,
                   help="base directory for generated files")
    args = p.parse_args()

    if args.motor is not None and not (0 <= args.motor < NUM_MOTORS):
        raise SystemExit(f"--motor must be 0..{NUM_MOTORS - 1}, got {args.motor}")

    run(args.port, args.id, args.motor, args.deadbands, args.start, args.stop,
        args.step, args.baseline, args.name, args.file, args.outdir)


if __name__ == "__main__":
    main()
