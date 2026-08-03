"""Read back and verify the tuning a delta board is actually running.

SetConfigCommand is write-only, so a board that boots on compiled defaults and a
board that has had a calibration pushed look identical over the wire -- until
now. This connects to a board, asks it for its LIVE config (ConfigRequest), and
prints the per-motor deadband / bias and the board-global brake_at_setpoint.

With a calibration file in scope it also DIFFS the live config against the JSON
and flags every mismatch, so you can confirm a board is running what you think it
is. This matters because the firmware config is RAM-only: a power-cycle or reboot
silently reverts a board to its compiled defaults (brake off, 0.8 mm deadband,
zero bias) until apply_calibration is re-run.

Which calibration file is used mirrors apply_calibration.py:
  --file PATH   diff against exactly that file
  --name NAME   diff against config/calibration/board_<NAME>.json
  (neither)     auto: config/calibration/board_<chip_id>.json for the board found
  --no-diff     just print the live config, don't load / compare any file

Sends only a ConfigRequest (no motion), so it is safe to run any time.

Example:
    python read_config.py --id board0
    python read_config.py --name 3
    python read_config.py --no-diff
"""

import argparse
import os

from delta_control import (
    calibration_path,
    diff_calibration,
    load_calibration,
    load_calibration_file,
    open_board,
)
from delta_control.constants import NUM_MOTORS

DEFAULT_PORT = None   # None auto-detects the board's port
DEFAULT_BOARD = None  # None auto-discovers the single connected board


def resolve_calib(board_id, file, name):
    """Return (calib_dict, source_str) for the diff, or (None, reason) if none."""
    if file:
        return load_calibration_file(file), file
    if name is not None:
        src = calibration_path(name)
        if not os.path.exists(src):
            return None, f"no calibration file {src} (for --name {name})"
        return load_calibration_file(src), src
    src = calibration_path(board_id)
    calib = load_calibration(board_id)
    if calib is None:
        return None, f"no calibration for board {board_id} (expected {src})"
    return calib, src


def print_live(cfg):
    """Print the board's live config as a per-motor table."""
    print(f"  brake_at_setpoint = {cfg['brake_at_setpoint']}")
    print(f"  {'motor':>5}  {'deadband(mm)':>12}  {'bias_fwd':>8}  {'bias_back':>9}")
    for i in range(NUM_MOTORS):
        print(f"  {i:>5}  {cfg['deadband'][i] * 1e3:>12.4f}  "
              f"{cfg['bias_fwd'][i]:>8}  {cfg['bias_back'][i]:>9}")


def run(port, board, file, name, do_diff):
    env, agent = open_board(port, board)
    board_id = env.active_ids[0]
    print(f"connected to board {board_id}")
    try:
        cfg = agent.get_config()
    finally:
        agent.close()

    if cfg is None:
        raise SystemExit(
            "board returned no config_resp -- its firmware predates ConfigRequest. "
            "Reflash with the current firmware to enable config readback.")

    print("live config on board:")
    print_live(cfg)

    if not do_diff:
        return

    calib, src = resolve_calib(board_id, file, name)
    if calib is None:
        print(f"\nno diff: {src}")
        return

    print(f"\ndiff vs {src}:")
    cfg_id = calib.get("board_id")
    if cfg_id is not None and cfg_id != board_id:
        print(f"  WARNING: calibration board_id {cfg_id} != connected board {board_id}")
    mismatches = diff_calibration(cfg, calib)
    if not mismatches:
        print("  MATCH -- board is running this calibration")
    else:
        print(f"  {len(mismatches)} MISMATCH(es):")
        for msg in mismatches:
            print(f"    {msg}")
        print("  (run apply_calibration.py to push the calibration to the board)")
        raise SystemExit(1)


def main():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default=DEFAULT_PORT, help="serial port of the board")
    p.add_argument("--id", default=DEFAULT_BOARD,
                   help="board label (BOARD_REGISTRY) or raw id; omit to auto-discover")
    p.add_argument("--name", default=None,
                   help="diff against config/calibration/board_<name>.json (e.g. --name 3)")
    p.add_argument("--file", default=None,
                   help="explicit calibration JSON path to diff against (overrides --name)")
    p.add_argument("--no-diff", dest="diff", action="store_false",
                   help="just print the live config; do not load or compare a file")
    args = p.parse_args()
    run(args.port, args.id, args.file, args.name, args.diff)


if __name__ == "__main__":
    main()
