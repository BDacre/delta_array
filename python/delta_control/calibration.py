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
        "brake_at_setpoint": true,
        "motors": [
            {"deadband": 0.0008, "bias_fwd": 29, "bias_back": 20},
            ...   # exactly 12 entries, motor 0..11
        ]
    }

Each motor entry may set any subset of ``deadband`` / ``bias_fwd`` /
``bias_back``; omitted keys leave that motor's current firmware value unchanged.
An empty ``{}`` entry skips the motor entirely.

``brake_at_setpoint`` (optional, top-level, board-global) selects the settle
behaviour: ``true`` short-brakes a joint once it reaches its deadband (kills
post-release coast/overshoot), ``false`` coasts. Omitted -> the board keeps its
current mode (firmware default is coast). Applied by ``apply_calibration``.

Typical use::

    env, agent = open_board()
    calib = load_calibration(env.active_ids[0])
    if calib:
        apply_calibration(agent, calib)
"""

import glob
import json
import os
from dataclasses import dataclass, field

from .constants import NUM_MOTORS

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
DEFAULT_CALIBRATION_DIR = os.path.join(REPO_ROOT, "config", "calibration")

_MOTOR_FIELDS = ("deadband", "bias_fwd", "bias_back")

# Firmware stores deadband as float32, so an exact == against the JSON's float
# would spuriously fail; allow this much rounding slack (1 µm << any real value).
DEADBAND_TOL = 1e-6


@dataclass
class BoardCalibResult:
    """Outcome of provisioning/verifying one board (see DeltaArrayEnv.provision).

    A structured result rather than a printed line, so library callers can act on
    it (assert, log, retry) instead of scraping stdout. `ok` is the single
    pass/fail: True only when no error occurred and the readback matched.
    """
    board_id: int
    applied: bool                 # did we write the calibration (vs. verify-only)
    mismatches: list = field(default_factory=list)  # readback vs JSON diff lines
    brake_at_setpoint: bool | None = None  # live brake mode read back (None if unread)
    error: str | None = None      # non-diff failure (no file, old firmware, id clash)

    @property
    def ok(self) -> bool:
        return self.error is None and not self.mismatches


def calibration_path(board_id, calib_dir=None):
    """Path to the JSON file for a board id (whether or not it exists)."""
    calib_dir = calib_dir or DEFAULT_CALIBRATION_DIR
    return os.path.join(calib_dir, f"board_{board_id}.json")


def load_calibration(board_id, calib_dir=None):
    """Load and validate the calibration for a board id.

    Resolves the file two ways, so it works regardless of how the file is named:
      1. Exact filename match ``board_<chip id>.json`` (calibration_path).
      2. Fallback: scan the directory for any ``board_*.json`` whose ``board_id``
         field equals this board -- files are usually named by a human label
         (board_3.json), not the large chip id, so the exact match rarely hits.

    Returns the parsed dict, or None if no file matches (an uncalibrated board is
    a normal, non-error case — it just runs on defaults). Raises ValueError if the
    exact-name file's board_id contradicts the request, or if more than one file
    claims this board_id.
    """
    calib_dir = calib_dir or DEFAULT_CALIBRATION_DIR
    path = calibration_path(board_id, calib_dir)
    if os.path.exists(path):
        calib = load_calibration_file(path)
        cfg_id = calib.get("board_id")
        if cfg_id is not None and cfg_id != board_id:
            raise ValueError(
                f"{path}: board_id {cfg_id} does not match requested board {board_id}"
            )
        return calib
    return _find_by_board_id(board_id, calib_dir)


def _find_by_board_id(board_id, calib_dir):
    """Scan calib_dir for a board_*.json whose ``board_id`` field matches.

    Filename-agnostic lookup. Files that don't parse/validate can't be the match
    (their id is unreadable), so they're skipped rather than aborting the scan.
    Returns the parsed dict, None if nothing matches, or raises if two files claim
    the same board_id (an ambiguity the caller must resolve).
    """
    matches = []
    for p in sorted(glob.glob(os.path.join(calib_dir, "board_*.json"))):
        try:
            calib = load_calibration_file(p)
        except (ValueError, OSError):
            continue
        if calib.get("board_id") == board_id:
            matches.append((p, calib))
    if not matches:
        return None
    if len(matches) > 1:
        raise ValueError(
            f"multiple calibration files claim board_id {board_id}: "
            f"{[p for p, _ in matches]}"
        )
    return matches[0][1]


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
    brake = calib.get("brake_at_setpoint")
    if brake is not None and not isinstance(brake, bool):
        raise ValueError(
            f"{path}: 'brake_at_setpoint' must be true or false, got {brake!r}"
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
    """Push a loaded calibration to a board via set_config calls.

    Sends one SetConfigCommand per motor that has at least one field, so only the
    specified fields are overwritten. If the calibration carries the optional
    board-global ``brake_at_setpoint`` flag, it is applied too (after the
    per-motor config) so a single JSON fully provisions the board -- every loader
    that goes through this function honours the flag, not just the CLI.
    Returns the number of motors configured (the brake flag is board-global and
    not counted). Each call is ACK-or-raise (CommandError) so a dropped frame is
    not silent.
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
    brake = calib.get("brake_at_setpoint")
    if brake is not None:
        agent.set_config(brake_at_setpoint=bool(brake))
    return configured


def diff_calibration(live, calib, *, deadband_tol=DEADBAND_TOL):
    """Compare a board's live config to a calibration; return mismatch strings.

    ``live`` is a dict as returned by ``DeltaArrayAgent.get_config`` (12-lists for
    deadband/bias_fwd/bias_back plus a brake_at_setpoint bool); ``calib`` is a
    loaded calibration dict. Returns a list of human-readable mismatch lines --
    empty means the board is running this calibration.

    Only fields the JSON actually specifies are checked: an omitted per-motor
    field means "leave the firmware value alone", so it can never be a mismatch.
    brake_at_setpoint is compared only when the JSON sets it (else the board keeps
    whatever mode it had). Deadband is compared with ``deadband_tol`` slack for
    float32 rounding. This is the single source of truth for both read_config.py
    (one board) and provision_array.py (the whole array).
    """
    mismatches = []
    motors = calib["motors"]
    for i in range(NUM_MOTORS):
        m = motors[i] if i < len(motors) else {}
        if "deadband" in m and abs(m["deadband"] - live["deadband"][i]) > deadband_tol:
            mismatches.append(
                f"motor {i:2d} deadband: JSON {m['deadband']:.4f} m "
                f"!= board {live['deadband'][i]:.4f} m")
        for key in ("bias_fwd", "bias_back"):
            if key in m and m[key] != live[key][i]:
                mismatches.append(
                    f"motor {i:2d} {key}: JSON {m[key]} != board {live[key][i]}")
    want_brake = calib.get("brake_at_setpoint")
    if want_brake is not None and bool(want_brake) != live["brake_at_setpoint"]:
        mismatches.append(
            f"brake_at_setpoint: JSON {bool(want_brake)} "
            f"!= board {live['brake_at_setpoint']}")
    return mismatches


class CalibrationError(RuntimeError):
    """One or more connected boards are not running their per-motor calibration."""


def _describe_board(result):
    """One (or more) human-readable lines for a not-ok BoardCalibResult."""
    from .boards import BOARD_LABELS

    label = BOARD_LABELS.get(result.board_id)
    name = f"{result.board_id} ({label})" if label else str(result.board_id)
    if result.error:  # no file, id clash, or firmware too old to report config
        return f"  board {name}: {result.error}"
    lines = [f"  board {name}: {len(result.mismatches)} config mismatch(es)"]
    lines += [f"      {m}" for m in result.mismatches]
    return "\n".join(lines)


def enforce_calibration(env, *, calib_dir=None, quiet=False):
    """Provision every connected board and verify it, or raise CalibrationError.

    The gate every script that MOVES the array should run immediately after
    opening the env and before commanding any motion. Firmware config is
    RAM-only, so a flash or power-cycle silently reverts a board to compiled
    defaults (brake off, 0.8 mm deadband, zero bias) -- a different deadband and
    bias, which distorts an open-loop trajectory without ever erroring.

    Config-only -- no motion. Applies each board's calibration JSON
    (``env.provision``) then reads every board back write-free
    (``env.check_calibration``) as a hard gate. Returns the provision report
    (``{board_id: BoardCalibResult}``) on success; raises ``CalibrationError`` --
    after closing ``env`` to release the serial ports -- if any board is not
    running its calibration.

    This is the same call path as the provision_array CLI, so a script and the
    CLI cannot disagree about what "calibrated" means. Do NOT use it in the
    tools that measure or set calibration (characterize_*, sweep_deadband,
    apply_calibration) or in raw-PWM diagnostics: those deliberately run a board
    off its calibration, and gating them would be circular.

    Usage, right after opening the env::

        env = DeltaArrayEnv()
        enforce_calibration(env)   # provision + gate, or raise before any motion
    """
    report = env.provision(calib_dir=calib_dir)         # apply each board's JSON + verify
    gate = env.check_calibration(calib_dir=calib_dir)   # read-only confirmation, no writes

    bad = [r for r in gate.values() if not r.ok]
    if bad:
        env.close()  # release the ports; we abort before commanding any motion
        detail = "\n".join(_describe_board(r) for r in bad)
        raise CalibrationError(
            f"{len(bad)} of {len(gate)} board(s) are not running their "
            f"calibration (fix the board_<id>.json under config/calibration/, "
            f"then rerun):\n{detail}"
        )

    if not quiet:
        print(f"  calibration verified on {len(report)} board(s): "
              f"{', '.join(str(b) for b in sorted(report))}")
    return report
