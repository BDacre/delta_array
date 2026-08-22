"""Provision the whole array at startup: push each board's per-motor calibration
and PROVE, by reading it back, that every board is running the right config.

Why this exists: firmware config is RAM-only, so every flash or power-cycle
silently reverts a board to compiled defaults (brake off, 0.8 mm deadband, zero
bias). apply_calibration/read_config act on one board; this does the whole array
in one shot and, crucially, *verifies* each board via ConfigRequest after writing
-- so a board that didn't take its calibration can't slip through unnoticed.

Run it once at the start of every session (or after any reflash/power-cycle).

Modes:
  (default)  apply each connected board's calibration, then read back and verify.
  --check    verify only -- read back and diff, write nothing. A fast, side-effect
             -free gate to confirm the array is already correctly provisioned.

Each board's calibration is resolved by its chip id (config/calibration keyed by
board_id, same as apply_calibration's auto mode). A connected board with no
calibration file, a readback mismatch, or (with --expect) an expected board that
didn't connect all make the tool FAIL with a non-zero exit -- so it drops into a
startup script or pre-run check.

Sends only config writes + a ConfigRequest; no motion.

Examples:
    python provision_array.py                       # apply + verify all connected boards
    python provision_array.py --check               # verify only, no writes
    python provision_array.py --expect board0,board1,board2,board3
"""

import argparse
import sys

from delta_control import DeltaArrayEnv, DiscoveryError
from delta_control.boards import BOARD_LABELS, BOARD_REGISTRY, check_calibration_ids

MAX_SHOWN = 3  # per-board mismatch lines to print inline before eliding


def _label(board_id):
    lbl = BOARD_LABELS.get(board_id)
    return f"{board_id} ({lbl})" if lbl else f"{board_id} (unregistered)"


def _resolve_expected(token):
    """Map a --expect token (BOARD_REGISTRY label or raw id) to a chip id."""
    if token in BOARD_REGISTRY:
        return BOARD_REGISTRY[token]
    try:
        return int(token)
    except ValueError:
        raise SystemExit(
            f"--expect: unknown board {token!r}; use a raw id or a label "
            f"({sorted(BOARD_REGISTRY)})")


def _detail(result, apply):
    """Render a BoardCalibResult as a one-line human status."""
    if result.error:
        return result.error
    if result.mismatches:
        shown = "; ".join(result.mismatches[:MAX_SHOWN])
        more = (f"; (+{len(result.mismatches) - MAX_SHOWN} more)"
                if len(result.mismatches) > MAX_SHOWN else "")
        return f"{len(result.mismatches)} mismatch(es): {shown}{more}"
    verb = "verified" if apply else "already correct"
    return f"{verb} (brake={result.brake_at_setpoint})"


def run(ports, expect, apply):
    # If the registry and the calibration files disagree about which id is which
    # board, provisioning would push one board's numbers onto another. Cheap to
    # check (no hardware) and worth doing before opening a single port.
    problems = check_calibration_ids()
    if problems:
        for line in problems:
            print(f"  [FAIL] registry: {line}")
        print("\nSUMMARY: BOARD_REGISTRY disagrees with config/calibration "
              "-- fix before provisioning")
        return 1

    try:
        env = DeltaArrayEnv(ports=ports)
    except DiscoveryError as e:
        print(f"FAIL: {e}")
        return 1

    try:
        action = "provisioning" if apply else "checking"
        print(f"{action} {len(env.active_ids)} connected board(s)...\n")
        # All orchestration lives in the library; the CLI only formats + gates.
        report = env.provision() if apply else env.check_calibration()
    finally:
        env.close()

    for bid, result in report.items():
        print(f"  [{'OK  ' if result.ok else 'FAIL'}] "
              f"board {_label(bid)}: {_detail(result, apply)}")

    # Expected boards that never connected are as dangerous as a bad config.
    missing = []
    if expect:
        present = set(report)
        missing = [tok for tok in expect if _resolve_expected(tok) not in present]
        for tok in missing:
            print(f"  [FAIL] expected board {tok} did not connect")

    failed = [b for b, r in report.items() if not r.ok]
    total = len(report)
    passed = total - len(failed)
    print()
    if not failed and not missing:
        print(f"SUMMARY: {passed}/{total} boards OK -- array is running its calibration")
        return 0
    parts = [f"{passed}/{total} boards OK"]
    if failed:
        parts.append(f"FAILED: {[_label(b) for b in failed]}")
    if missing:
        parts.append(f"MISSING: {missing}")
    print("SUMMARY: " + " -- ".join(parts))
    if not apply and failed:
        print("  run 'python provision_array.py' (no --check) to push calibration")
    return 1


def main():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--check", dest="apply", action="store_false",
                   help="verify only: read back and diff, write nothing")
    p.add_argument("--expect", default=None,
                   help="comma-separated boards (labels or ids) that MUST be present; "
                        "any that don't connect fail the run")
    p.add_argument("--ports", default=None,
                   help="comma-separated serial ports to open; omit to scan all")
    args = p.parse_args()

    expect = [t.strip() for t in args.expect.split(",") if t.strip()] if args.expect else None
    ports = [t.strip() for t in args.ports.split(",") if t.strip()] if args.ports else None

    sys.exit(run(ports, expect, args.apply))


if __name__ == "__main__":
    main()
