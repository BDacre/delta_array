"""Per-board motor calibration (deadband + static feedforward bias).

Calibration lives host-side as one JSON file per physical board, keyed by the
board's chip-derived id, under ``config/calibration/``. A single firmware image
runs on every board and ships with safe defaults (bias 0, historical deadband);
the host pushes a board's calibration over serial with SetConfigCommand
(``DeltaArrayAgent.set_config``) after connecting, so each board is tuned without
reflashing and the calibration stays versioned and diffable in git.

File format (``config/calibration/board_<id>.json``)::

    {
        "board_id": 855203507,
        "note": "seeded from characterize_breakaway 2026-07-31",
        "motors": [
            {"deadband": 0.0008, "bias_fwd": 29, "bias_back": 20},
            ...   # exactly 12 entries, motor 0..11
        ]
    }

Each motor entry may set any subset of ``deadband`` / ``bias_fwd`` /
``bias_back``; omitted keys leave that motor's current firmware value unchanged.
An empty ``{}`` entry skips the motor entirely.

Typical use::

    env, agent = open_board()
    calib = load_calibration(env.active_ids[0])
    if calib:
        apply_calibration(agent, calib)
"""

import json
import os

from .constants import NUM_MOTORS

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
DEFAULT_CALIBRATION_DIR = os.path.join(REPO_ROOT, "config", "calibration")

_MOTOR_FIELDS = ("deadband", "bias_fwd", "bias_back")


def calibration_path(board_id, calib_dir=None):
    """Path to the JSON file for a board id (whether or not it exists)."""
    calib_dir = calib_dir or DEFAULT_CALIBRATION_DIR
    return os.path.join(calib_dir, f"board_{board_id}.json")


def load_calibration(board_id, calib_dir=None):
    """Load and validate the calibration for a board id.

    Returns the parsed dict, or None if no file exists for that board (an
    uncalibrated board is a normal, non-error case — it just runs on defaults).
    Raises ValueError if a file exists but is malformed.
    """
    path = calibration_path(board_id, calib_dir)
    if not os.path.exists(path):
        return None
    calib = load_calibration_file(path)
    cfg_id = calib.get("board_id")
    if cfg_id is not None and cfg_id != board_id:
        raise ValueError(
            f"{path}: board_id {cfg_id} does not match requested board {board_id}"
        )
    return calib


def load_calibration_file(path):
    """Load and schema-validate a calibration JSON at an explicit path.

    Unlike load_calibration (which resolves by board id), this accepts any path
    and does not require the filename to encode the id — use it for hand-named
    files. Raises ValueError if the file is malformed.
    """
    with open(path) as f:
        calib = json.load(f)
    _validate(calib, path)
    return calib


def _validate(calib, path):
    motors = calib.get("motors")
    if not isinstance(motors, list) or len(motors) != NUM_MOTORS:
        raise ValueError(
            f"{path}: 'motors' must be a list of {NUM_MOTORS} entries, "
            f"got {type(motors).__name__} of len "
            f"{len(motors) if isinstance(motors, list) else 'n/a'}"
        )
    for i, m in enumerate(motors):
        if not isinstance(m, dict):
            raise ValueError(f"{path}: motor {i} entry must be an object")
        unknown = set(m) - set(_MOTOR_FIELDS)
        if unknown:
            raise ValueError(
                f"{path}: motor {i} has unknown field(s) {sorted(unknown)}; "
                f"allowed: {list(_MOTOR_FIELDS)}"
            )


def apply_calibration(agent, calib):
    """Push a loaded calibration to a board via per-motor set_config calls.

    Sends one SetConfigCommand per motor that has at least one field, so only the
    specified fields are overwritten. Returns the number of motors configured.
    Each call is ACK-or-raise (CommandError) so a dropped frame is not silent.
    """
    motors = calib["motors"]
    if len(motors) != NUM_MOTORS:
        raise ValueError(
            f"calibration has {len(motors)} motor entries, expected {NUM_MOTORS}"
        )
    configured = 0
    for i, m in enumerate(motors):
        fields = {k: m[k] for k in _MOTOR_FIELDS if k in m}
        if not fields:
            continue
        agent.set_config(i, **fields)
        configured += 1
    return configured
