"""Static-friction breakaway PWM for each motor, each direction (open-loop).

Bypasses the PID (via the firmware SetPwmCommand) to find the smallest PWM that
actually starts the joint moving -- the static-friction breakaway point. This is
the number the closed-loop PID can't reveal, and it's what a feedforward bias
should be seeded from.

Method, per motor and per direction sign:
  1. Seat the joint near mid-travel (PID move), so there's room to move.
  2. Ramp PWM from --pwm-start upward in --pwm-step increments. At each level,
     pulse the motor open-loop for --pulse-ms (the firmware auto-releases after),
     then measure how far the joint moved.
  3. The first PWM whose pulse moves the joint more than --motion-threshold is
     the breakaway PWM for that direction. Stop ramping that direction.

Safety: every pulse is bounded in time by the firmware auto-release; the sweep
aborts a direction if the joint leaves [MIN+margin, MAX-margin]; magnitude is
clamped to 255 on both host and firmware.

Sign convention: pwm > 0 == firmware BACKWARD, pwm < 0 == FORWARD. Which physical
direction that is per motor is recorded (delta sign), not assumed.

Example:
    python characterize_breakaway.py --motor 0
    python characterize_breakaway.py            # all 12 motors
"""

import argparse
import csv
import os
import statistics
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

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
DEFAULT_OUTDIR = os.path.join(REPO_ROOT, "generated_files")
TEST_NAME = "breakaway"

# Defaults chosen to be gentle on the hardware.
SEAT_POSITION = 0.05      # m, mid-travel seat before each direction
SEAT_SETTLE_S = 1.5       # s, let the PID seat the joint
BOUND_MARGIN = 0.004      # m, keep this far inside [MIN, MAX]
POST_PULSE_S = 0.08       # s, extra wait after a pulse auto-releases before reading


def build_output_path(outdir, motors):
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    tag = f"motor{motors[0]:02d}" if len(motors) == 1 else f"{len(motors)}motors"
    return os.path.join(outdir, TEST_NAME, f"{TEST_NAME}_{tag}_{stamp}.csv")


def seat(agent, motor, baseline, position):
    """Drive all motors to baseline with `motor` at `position`, let it settle."""
    vec = [baseline] * NUM_MOTORS
    vec[motor] = position
    agent.move_joint_position(vec)
    time.sleep(SEAT_SETTLE_S)


def ramp_direction(agent, motor, sign, trial, pwm_start, pwm_max, pwm_step,
                   pulse_ms, motion_threshold, baseline, rows):
    """Ramp |PWM| upward in one direction until the joint breaks free.

    Returns the breakaway PWM (signed) or None if none found within pwm_max.
    """
    seat(agent, motor, baseline, SEAT_POSITION)
    lo = MIN_JOINT_POS + BOUND_MARGIN
    hi = MAX_JOINT_POS - BOUND_MARGIN

    pwm = pwm_start
    while pwm <= pwm_max:
        before = agent.get_joint_positions()[motor]
        if not (lo <= before <= hi):
            print(f"    motor {motor} sign {sign:+d}: at bound ({before:.4f} m), "
                  f"re-seating")
            seat(agent, motor, baseline, SEAT_POSITION)
            before = agent.get_joint_positions()[motor]
        agent.set_motor_pwm(motor, sign * pwm, duration_ms=pulse_ms)
        time.sleep(pulse_ms / 1000.0 + POST_PULSE_S)
        after = agent.get_joint_positions()[motor]
        delta = after - before
        moved = abs(delta) >= motion_threshold
        rows.append((motor, trial, sign, pwm, before, after, delta, int(moved)))
        print(f"    motor {motor} sign {sign:+d} trial {trial}  pwm={pwm:3d}  "
              f"delta={delta * 1e3:+.3f} mm  {'MOVED' if moved else ''}")
        if moved:
            return sign * pwm
        pwm += pwm_step
    return None


def _fmt_stats(vals):
    """Summarize a list of breakaway |PWM| values across trials."""
    found = [abs(v) for v in vals if v is not None]
    n_miss = len(vals) - len(found)
    if not found:
        return "  -- (no motion in any trial)"
    lo, hi = min(found), max(found)
    mean = sum(found) / len(found)
    spread = hi - lo
    sd = statistics.pstdev(found) if len(found) > 1 else 0.0
    miss = f", {n_miss} no-move" if n_miss else ""
    return (f"mean {mean:5.1f}  range {lo}-{hi} (spread {spread})  "
            f"sd {sd:.1f}  values {found}{miss}")


def print_json_seed(breakaway, motors, board_id, bias_margin, deadband=0):
    """Print a board_<name>.json-ready block seeding bias from the breakaway means.

    Rule: bias = max(0, round(mean_breakaway - bias_margin)), routed by direction
    to match the firmware sign convention -- +dir (BACKWARD) mean -> bias_back,
    -dir (FORWARD) mean -> bias_fwd. `bias_margin` seeds the feedforward just BELOW
    the measured static breakaway so bias alone doesn't sit at the friction
    threshold (which risks creep/lurch); the PID's KP*err term supplies the last
    bit to break free. The margin is a hand-picked seed, not a derived constant --
    it's within the sweep's own resolution (step 5, sd ~2-4) -- so the deadband
    sweep + staircase A/B are what actually confirm it. Motors with no detected
    breakaway get bias 0 (no feedforward). deadband is a placeholder: 0 would stop
    the joint from ever settling, so set a real value (e.g. 0.0008) or run
    sweep_deadband.py before applying this.
    """
    def seed(m, sign):
        vals = [abs(v) for v in breakaway[(m, sign)] if v is not None]
        if not vals:
            return 0
        return max(0, round(sum(vals) / len(vals) - bias_margin))

    print(f"\nboard JSON seed (bias = mean - {bias_margin}; deadband placeholder "
          f"= {deadband} -- set a real deadband before applying):")
    if len(motors) != NUM_MOTORS:
        print(f"  NOTE: only motors {motors} measured; a full calibration needs "
              f"all {NUM_MOTORS}.")
    print("{")
    print(f'    "board_id": {board_id},')
    print(f'    "note": "bias seeded from characterize_breakaway (mean - '
          f'{bias_margin}); deadband is a placeholder",')
    print('    "motors": [')
    for i, m in enumerate(motors):
        # +dir (BACKWARD) -> bias_back ; -dir (FORWARD) -> bias_fwd
        back, fwd = seed(m, +1), seed(m, -1)
        comma = "," if i < len(motors) - 1 else ""
        print(f'        {{"deadband": {deadband}, "bias_back": {back}, '
              f'"bias_fwd": {fwd}}}{comma}')
    print("    ]")
    print("}")


def run(port, board, motors, trials, pwm_start, pwm_max, pwm_step, pulse_ms,
        motion_threshold, baseline, bias_margin, out, outdir, show_plot=True):
    out_path = out or build_output_path(outdir, motors)

    env, agent = open_board(port, board)
    print(f"opening {port}, using board id {env.active_ids[0]}")
    print(f"breakaway sweep: motors {motors}, {trials} trial(s) each, "
          f"pwm {pwm_start}..{pwm_max} step {pwm_step}, pulse {pulse_ms} ms, "
          f"motion threshold {motion_threshold * 1e3:.2f} mm")

    rows = []
    # (motor, sign) -> list of signed breakaway pwm per trial (None = no motion)
    breakaway = {(m, s): [] for m in motors for s in (+1, -1)}
    try:
        for motor in motors:
            print(f"  motor {motor}:")
            for sign in (+1, -1):
                for trial in range(trials):
                    bp = ramp_direction(agent, motor, sign, trial, pwm_start,
                                        pwm_max, pwm_step, pulse_ms,
                                        motion_threshold, baseline, rows)
                    breakaway[(motor, sign)].append(bp)
                    if bp is None:
                        print(f"    motor {motor} sign {sign:+d} trial {trial}: "
                              f"no motion up to pwm={pwm_max}")
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
        w.writerow([f"# {TEST_NAME} motors={motors} trials={trials} "
                    f"pwm={pwm_start}..{pwm_max}:{pwm_step} "
                    f"pulse_ms={pulse_ms} motion_threshold_m={motion_threshold} "
                    f"baseline={baseline} board={env.active_ids[0]}"])
        w.writerow(["motor", "trial", "sign", "pwm", "pos_before_m",
                    "pos_after_m", "delta_m", "moved"])
        w.writerows(rows)
    print(f"wrote {len(rows)} pulses to {out_path}")

    print(f"\nBreakaway PWM summary ({trials} trial(s) per direction):")
    for motor in motors:
        print(f"  motor {motor:2d} +dir: {_fmt_stats(breakaway[(motor, +1)])}")
        print(f"  motor {motor:2d} -dir: {_fmt_stats(breakaway[(motor, -1)])}")

    print_json_seed(breakaway, motors, env.active_ids[0], bias_margin)

    if show_plot:
        try:
            plot_breakaway(rows, breakaway, motors, trials)
        except Exception as e:
            print(f"skipping plot ({e}); data is in {out_path}")


def plot_breakaway(rows, breakaway, motors, trials):
    """Single motor: displacement vs PWM (+ per-trial breakaway strip when
    trials>1). Many motors: mean breakaway bars with min-max whiskers."""
    import matplotlib.pyplot as plt

    def found(m, s):
        return [abs(v) for v in breakaway[(m, s)] if v is not None]

    # rows: (motor, trial, sign, pwm, before, after, delta, moved)
    if len(motors) == 1:
        motor = motors[0]
        if trials > 1:
            fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5))
        else:
            fig, ax1 = plt.subplots(figsize=(9, 5))
            ax2 = None

        for sign, color, lbl in ((+1, "C0", "+dir (BACKWARD)"),
                                 (-1, "C3", "-dir (FORWARD)")):
            pts = [(r[3], abs(r[6]) * 1e3) for r in rows if r[2] == sign]
            if pts:
                ax1.plot([p for p, _ in pts], [d for _, d in pts], color=color,
                         marker="o", ms=3, ls="", alpha=0.6, label=lbl)
        ax1.axhline(0, color="0.7", lw=0.8)
        ax1.set_xlabel("commanded |PWM|")
        ax1.set_ylabel("displacement per pulse (mm)")
        ax1.set_title(f"motor {motor}: open-loop displacement vs PWM")
        ax1.legend(loc="best")
        ax1.grid(True, alpha=0.3)

        if ax2 is not None:
            for i, (sign, color, lbl) in enumerate(((+1, "C0", "+dir"),
                                                    (-1, "C3", "-dir"))):
                vals = found(motor, sign)
                if vals:
                    ax2.plot([i] * len(vals), vals, color=color, marker="o",
                             ms=6, ls="", alpha=0.7, label=lbl)
                    mean = sum(vals) / len(vals)
                    ax2.plot([i - 0.18, i + 0.18], [mean, mean], color=color, lw=2)
            ax2.set_xticks([0, 1])
            ax2.set_xticklabels(["+dir", "-dir"])
            ax2.set_xlim(-0.5, 1.5)
            ax2.set_ylabel("breakaway PWM per trial")
            ax2.set_title(f"motor {motor}: breakaway across {trials} trials "
                          f"(bar = mean)")
            ax2.grid(True, axis="y", alpha=0.3)
    else:
        fig, ax = plt.subplots(figsize=(11, 5))
        x = list(range(len(motors)))
        for off, sign, color, lbl in ((-0.2, +1, "C0", "+dir"),
                                      (0.2, -1, "C3", "-dir")):
            means, err_lo, err_hi = [], [], []
            for m in motors:
                vals = found(m, sign)
                if vals:
                    mean = sum(vals) / len(vals)
                    means.append(mean)
                    err_lo.append(mean - min(vals))
                    err_hi.append(max(vals) - mean)
                else:
                    means.append(0); err_lo.append(0); err_hi.append(0)
            ax.bar([i + off for i in x], means, width=0.4, color=color, label=lbl,
                   yerr=[err_lo, err_hi], capsize=3)
        ax.set_xticks(x)
        ax.set_xticklabels([str(m) for m in motors])
        ax.set_xlabel("motor")
        ax.set_ylabel("breakaway PWM (bar = mean, whisker = min-max)")
        ax.set_title(f"breakaway PWM per motor and direction ({trials} trials)")
        ax.legend(loc="best")
        ax.grid(True, axis="y", alpha=0.3)

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
    p.add_argument("--trials", type=int, default=1,
                   help="repeat each motor/direction this many times to gauge consistency")
    p.add_argument("--pwm-start", type=int, default=20, help="starting |PWM|")
    p.add_argument("--pwm-max", type=int, default=200, help="max |PWM| to try")
    p.add_argument("--pwm-step", type=int, default=5, help="|PWM| increment")
    p.add_argument("--pulse-ms", type=int, default=150, help="open-loop pulse duration (ms)")
    p.add_argument("--motion-threshold", type=float, default=1.2e-4,
                   help="displacement (m) counted as motion (default 0.12mm ~2 ADC counts)")
    p.add_argument("--bias-margin", type=int, default=3,
                   help="seed bias = mean_breakaway - this, so feedforward sits just "
                        "below the static-friction threshold (PID's KP*err tops it "
                        "off). Hand-picked seed, refined later by the deadband/"
                        "staircase A/B; within the sweep's own resolution. Default 3.")
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

    if args.trials < 1:
        raise SystemExit(f"--trials must be >= 1, got {args.trials}")

    run(args.port, args.id, motors, args.trials, args.pwm_start, args.pwm_max,
        args.pwm_step, args.pulse_ms, args.motion_threshold, args.baseline,
        args.bias_margin, args.out, args.outdir, show_plot=args.plot)


if __name__ == "__main__":
    main()
