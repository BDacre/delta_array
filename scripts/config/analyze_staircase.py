"""Compare two staircase runs (e.g. before vs after calibration) per motor.

Ingests two staircase CSVs produced by ``characterize_staircase.py`` and reports,
per motor, how well the joint tracked its commanded target and how much the
before/after run improved. The intended use is A/B-ing a calibration: run the
staircase once on defaults, apply the per-motor bias/deadband, run it again, then

    python analyze_staircase.py BEFORE.csv AFTER.csv

With no paths it auto-picks the two most recent 12-motor staircase CSVs under
``generated_files/staircase/`` (older = before, newer = after).

Per-motor metrics (all in mm):
  * mean|err|  -- average absolute tracking error over every settled point; the
                  headline number. Lower is better; the bias fix should shrink it.
  * max|err|   -- worst single point (catches limit-cycle overshoot spikes).
  * rms err    -- RMS tracking error (penalises the big misses more than mean).
  * hyst       -- mean up-sweep vs down-sweep gap at matching targets = backlash
                  + stiction loop width. Feedforward bias should tighten this.

A summary row aggregates across all motors so you get one before/after number.
"""

import argparse
import csv
import glob
import os
from collections import defaultdict

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
STAIRCASE_DIR = os.path.join(REPO_ROOT, "generated_files", "staircase")


def load_rows(path):
    """Parse a staircase CSV into rows (motor, direction, commanded, measured, error).

    Skips the leading ``#`` parameter comment and the header. Positions in meters.
    """
    rows = []
    with open(path, newline="") as f:
        for rec in csv.reader(f):
            if not rec or rec[0].startswith("#") or rec[0] == "motor":
                continue
            motor, direction, cmd, meas, err = rec
            rows.append((int(motor), direction, float(cmd), float(meas), float(err)))
    if not rows:
        raise SystemExit(f"{path}: no data rows")
    return rows


def _rms(vals):
    return (sum(v * v for v in vals) / len(vals)) ** 0.5


def hysteresis_mm(mrows):
    """Mean |up - down| measured gap at commanded targets present in both sweeps."""
    up = {round(t, 6): m for _, d, t, m, _ in mrows if d == "up"}
    down = {round(t, 6): m for _, d, t, m, _ in mrows if d == "down"}
    gaps = [abs(up[t] - down[t]) for t in up.keys() & down.keys()]
    return (sum(gaps) / len(gaps) * 1e3) if gaps else float("nan")


def motor_metrics(mrows):
    """Compute the per-motor tracking metrics (in mm) for one motor's rows."""
    errs = [abs(e) for *_, e in mrows]
    return {
        "n": len(mrows),
        "mean_abs": sum(errs) / len(errs) * 1e3,
        "max_abs": max(errs) * 1e3,
        "rms": _rms([e for *_, e in mrows]) * 1e3,
        "hyst": hysteresis_mm(mrows),
    }


def by_motor(rows):
    """Group rows by motor index -> {motor: [rows]}, ordered by motor."""
    d = defaultdict(list)
    for r in rows:
        d[r[0]].append(r)
    return dict(sorted(d.items()))


def _aggregate(rows, per_motor):
    """Fleet-wide metrics: error stats pool every point, but hysteresis is the
    mean of the per-motor values (pooling would collide the shared targets)."""
    m = motor_metrics(rows)
    hysts = [v["hyst"] for v in per_motor.values() if v["hyst"] == v["hyst"]]
    m["hyst"] = (sum(hysts) / len(hysts)) if hysts else float("nan")
    return m


def analyze(before_rows, after_rows):
    """Return {motor: {"before": metrics, "after": metrics}} plus an "all" key."""
    b, a = by_motor(before_rows), by_motor(after_rows)
    out = {}
    per_before, per_after = {}, {}
    for motor in sorted(set(b) | set(a)):
        entry = {}
        if motor in b:
            entry["before"] = per_before[motor] = motor_metrics(b[motor])
        if motor in a:
            entry["after"] = per_after[motor] = motor_metrics(a[motor])
        out[motor] = entry
    out["all"] = {
        "before": _aggregate(before_rows, per_before),
        "after": _aggregate(after_rows, per_after),
    }
    return out


def _fmt_delta(before, after):
    """'1.23 -> 0.45 (-63%)' style before/after cell for a single metric."""
    if before == 0 or before != before:  # zero or NaN baseline
        return f"{before:6.3f} -> {after:6.3f}"
    pct = (after - before) / before * 100.0
    return f"{before:6.3f} -> {after:6.3f} ({pct:+5.0f}%)"


def print_report(report, before_path, after_path):
    print(f"before: {os.path.basename(before_path)}")
    print(f"after : {os.path.basename(after_path)}")
    print()
    hdr = f"{'motor':>5}  {'mean|err| mm':>22}  {'max|err| mm':>22}  {'hyst mm':>22}"
    print(hdr)
    print("-" * len(hdr))
    for motor, entry in report.items():
        if "before" not in entry or "after" not in entry:
            label = motor if motor == "all" else f"{motor}"
            print(f"{label:>5}  (present in only one run; skipped)")
            continue
        b, a = entry["before"], entry["after"]
        label = "ALL" if motor == "all" else str(motor)
        if motor == "all":
            print("-" * len(hdr))
        print(f"{label:>5}  {_fmt_delta(b['mean_abs'], a['mean_abs']):>22}  "
              f"{_fmt_delta(b['max_abs'], a['max_abs']):>22}  "
              f"{_fmt_delta(b['hyst'], a['hyst']):>22}")


def plot_overlay(before_rows, after_rows, motors):
    """Grid of measured-vs-commanded, before (grey) overlaid with after (colour)."""
    import math

    import matplotlib.pyplot as plt

    b, a = by_motor(before_rows), by_motor(after_rows)
    ncols = 4
    nrows = math.ceil(len(motors) / ncols)
    fig, axes = plt.subplots(nrows, ncols, figsize=(4 * ncols, 3.2 * nrows),
                             squeeze=False)

    def _curve(ax, mrows, up_color, down_color, alpha, label_prefix):
        for d, marker, col in (("up", "o", up_color), ("down", "s", down_color)):
            pts = [(t, m) for _, dd, t, m, _ in mrows if dd == d]
            if pts:
                ax.plot([t * 1e3 for t, _ in pts], [m * 1e3 for _, m in pts],
                        color=col, marker=marker, ms=3, alpha=alpha,
                        label=f"{label_prefix} {d}")

    for k, motor in enumerate(motors):
        ax = axes[k // ncols][k % ncols]
        cmds = [t * 1e3 for _, _, t, _, _ in (b.get(motor, []) + a.get(motor, []))]
        if cmds:
            lo, hi = min(cmds), max(cmds)
            ax.plot([lo, hi], [lo, hi], color="0.7", ls="--", lw=1.0)
        if motor in b:
            _curve(ax, b[motor], "0.6", "0.75", 0.7, "before")
        if motor in a:
            _curve(ax, a[motor], "C0", "C3", 1.0, "after")
        ax.set_title(f"motor {motor}")
        ax.grid(True, alpha=0.3)
        if k == 0:
            ax.legend(loc="best", fontsize=7)
        if k % ncols == 0:
            ax.set_ylabel("measured (mm)")
        if k // ncols == nrows - 1:
            ax.set_xlabel("commanded (mm)")
    for k in range(len(motors), nrows * ncols):
        axes[k // ncols][k % ncols].axis("off")
    fig.suptitle("staircase before (grey) vs after (colour): tighter to y=x is better")
    fig.tight_layout()
    plt.show()


def _recent_12motor_csvs(n=2):
    """The n most-recently-modified 12-motor staircase CSVs, oldest first."""
    files = glob.glob(os.path.join(STAIRCASE_DIR, "staircase_12motors_*.csv"))
    files.sort(key=os.path.getmtime)
    return files[-n:]


def main():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("before", nargs="?", help="baseline staircase CSV (pre-calibration)")
    p.add_argument("after", nargs="?", help="staircase CSV to compare (post-calibration)")
    p.add_argument("--no-plot", dest="plot", action="store_false",
                   help="skip the overlay plot (e.g. headless runs)")
    args = p.parse_args()

    if args.before and args.after:
        before_path, after_path = args.before, args.after
    elif not args.before and not args.after:
        picks = _recent_12motor_csvs(2)
        if len(picks) < 2:
            raise SystemExit(
                f"need 2 staircase CSVs in {STAIRCASE_DIR} to auto-pick; "
                f"found {len(picks)}. Pass BEFORE and AFTER explicitly."
            )
        before_path, after_path = picks
        print(f"auto-picked two most recent 12-motor runs (older=before):")
    else:
        raise SystemExit("pass either both BEFORE and AFTER, or neither (auto-pick)")

    before_rows = load_rows(before_path)
    after_rows = load_rows(after_path)
    report = analyze(before_rows, after_rows)
    print_report(report, before_path, after_path)

    if args.plot:
        motors = sorted(m for m in report if isinstance(m, int))
        try:
            plot_overlay(before_rows, after_rows, motors)
        except Exception as e:  # headless / no display / backend issue
            print(f"\nskipping plot ({e})")


if __name__ == "__main__":
    main()
