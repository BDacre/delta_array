"""Push a board's per-motor calibration (deadband + bias) to the firmware.

The firmware ships with safe defaults (bias 0, 0.8 mm deadband); this loads the
calibration JSON for the connected board and applies it over serial via
SetConfigCommand. Run it after flashing / connecting, before commanding moves.

Which calibration file is used, in priority order:
  --file PATH   apply exactly that file
  --name NAME   apply config/calibration/board_<NAME>.json (a human-readable name,
                e.g. --name 4 -> board_4.json), independent of the chip id
  (neither)     auto: config/calibration/board_<chip_id>.json for the board found

Run it after flashing / connecting, before commanding moves.

Example:
    python apply_calibration.py --name 4
    python apply_calibration.py --file ../config/calibration/board_4.json
    python apply_calibration.py                 # auto by chip id
"""

import argparse
import os

from delta_control import (
    apply_calibration,
    calibration_path,
    load_calibration,
    load_calibration_file,
    open_board,
)

DEFAULT_PORT = None   # None auto-detects the board's port
DEFAULT_BOARD = None  # None auto-discovers the single connected board


def run(port, board, file, name, brake=None):
    env, agent = open_board(port, board)
    board_id = env.active_ids[0]
    print(f"connected to board {board_id}")
    try:
        if file:
            # Explicit path wins.
            calib = load_calibration_file(file)
            src = file
        elif name is not None:
            # Human-readable name: load config/calibration/board_<name>.json,
            # independent of the board's (large) chip id.
            src = calibration_path(name)
            if not os.path.exists(src):
                raise SystemExit(f"no calibration file {src} (for --name {name})")
            calib = load_calibration_file(src)
        else:
            # Auto: look up by the connected board's chip id.
            src = calibration_path(board_id)
            calib = load_calibration(board_id)
            if calib is None:
                raise SystemExit(
                    f"no calibration for board {board_id} (expected {src}); "
                    f"pass --name or --file"
                )

        print(f"applying {src}")
        cfg_id = calib.get("board_id")
        if cfg_id is not None:
            print(f"  calibration board_id: {cfg_id}")

        n = apply_calibration(agent, calib)
        print(f"applied calibration to {n} motor(s):")
        for i, m in enumerate(calib["motors"]):
            if m:
                print(f"  motor {i:2d}: {m}")

        # Optional board-global A/B: brake vs coast at setpoint. --brake / --no-brake
        # sets it explicitly; omit to leave the firmware's current mode untouched.
        if brake is None:
            brake = calib.get("brake_at_setpoint")  # optional JSON default
        if brake is not None:
            agent.set_config(brake_at_setpoint=bool(brake))
            print(f"brake_at_setpoint = {bool(brake)}")
    finally:
        agent.close()


def main():
    p = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default=DEFAULT_PORT, help="serial port of the board")
    p.add_argument("--id", default=DEFAULT_BOARD,
                   help="board label (BOARD_REGISTRY) or raw id; omit to auto-discover")
    p.add_argument("--name", default=None,
                   help="human-readable calibration name; loads "
                        "config/calibration/board_<name>.json (e.g. --name 4)")
    p.add_argument("--file", default=None,
                   help="explicit calibration JSON path (overrides --name)")
    p.add_argument("--brake", default=None, action=argparse.BooleanOptionalAction,
                   help="brake at setpoint (A/B): --brake to short-brake, "
                        "--no-brake to coast; omit to leave the current mode")
    args = p.parse_args()
    run(args.port, args.id, args.file, args.name, brake=args.brake)


if __name__ == "__main__":
    main()
