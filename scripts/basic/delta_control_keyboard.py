"""Keyboard control for one delta board (a group of 4 deltas).

Each key steps all 3 motors of a single delta up or down by a fixed amount.
The top letter row (q w e r) raises deltas 0-3; the row below (a s d f) lowers
them, so each delta is one vertical key pair. Joint targets are clamped to the
firmware travel limits by the agent, so held keys simply saturate.

Only works in a real terminal (needs a TTY for raw keystroke input).
"""

import argparse
import select
import sys
import termios
import time
import tty

from delta_control import enforce_calibration, open_board
from delta_control.constants import MAX_JOINT_POS, MIN_JOINT_POS

# Single-byte letter keys only (arrow keys are multi-byte escape sequences and
# proved unreliable). Top row q/w/e/r raises deltas 0/1/2/3; the row directly
# below, a/s/d/f, lowers the same deltas.
RAISE_KEYS = {"q": 0, "w": 1, "e": 2, "r": 3}
LOWER_KEYS = {"a": 0, "s": 1, "d": 2, "f": 3}

CONTROLS = """\
Controls
--------
    q w e r      : raise (step up) deltas 0 1 2 3
    a s d f      : lower (step down) deltas 0 1 2 3
    +  /  -      : increase / decrease the step size
    h            : return the whole board to home
    x  or  ESC   : quit (returns home and closes the port)
"""

DEFAULT_PORT = None  # None auto-detects the board's port by scanning /dev/ttyACM*
DEFAULT_BOARD = None  # None auto-discovers; set a BOARD_REGISTRY label or raw id to skip

NUM_DELTAS = 4
MOTORS_PER_DELTA = 3

DEFAULT_STEP = 0.005
MIN_STEP = 0.0005
MAX_STEP = 0.02
STEP_ADJUST = 0.0005


def read_key():
    """Block for a single keypress, decoding arrow-key escape sequences.

    Returns one of the arrow names ("up"/"down"/"left"/"right"), "esc", or the
    literal character pressed.
    """
    ch = sys.stdin.read(1)
    if ch != "\x1b":
        return ch
    # Arrow keys send a multi-byte escape sequence (ESC '[' 'A'..'D'). The
    # trailing bytes follow the ESC almost instantly, so wait a short window
    # for them; if nothing follows it was a genuine ESC keypress. 1 ms was too
    # tight and raced — the '[' and letter often arrived late, so arrow keys
    # read as a bare ESC (quit) and their leftover bytes leaked as stray keys.
    if not select.select([sys.stdin], [], [], 0.05)[0]:
        return "esc"
    if sys.stdin.read(1) != "[":
        return "esc"
    if not select.select([sys.stdin], [], [], 0.05)[0]:
        return "esc"
    return {"A": "up", "B": "down", "C": "right", "D": "left"}.get(
        sys.stdin.read(1), "esc"
    )


def print_status(agent, step):
    """Overwrite the status line with each delta's mean motor height."""
    pos = agent.current_joint_positions
    heights = " ".join(
        f"d{d}:{sum(pos[d * MOTORS_PER_DELTA:(d + 1) * MOTORS_PER_DELTA]) / MOTORS_PER_DELTA:.4f}"
        for d in range(NUM_DELTAS)
    )
    sys.stdout.write(f"\r{heights} | step {step:.4f}   ")
    sys.stdout.flush()


def step_delta(agent, delta_index, delta_step):
    """Move all 3 motors of one delta by delta_step (clamped by the agent)."""
    start = delta_index * MOTORS_PER_DELTA
    current = agent.current_joint_positions[start:start + MOTORS_PER_DELTA]
    target = [
        min(MAX_JOINT_POS, max(MIN_JOINT_POS, m + delta_step)) for m in current
    ]
    agent.move_delta(delta_index, target)


def control_loop(agent, step):
    print("\r\nKeyboard delta control. Press 'q' or ESC to quit.\r")
    print(CONTROLS.replace("\n", "\r\n"))
    print_status(agent, step)

    while True:
        key = read_key()
        print(f"DEBUG \r\nkey pressed: {key}   ")
        if key in ("x", "esc"):
            break
        elif key in RAISE_KEYS:
            step_delta(agent, RAISE_KEYS[key], step)
        elif key in LOWER_KEYS:
            step_delta(agent, LOWER_KEYS[key], -step)
        elif key == "+":
            step = min(MAX_STEP, step + STEP_ADJUST)
            print(f"step size increased to {step:.4f}")
        elif key == "-":
            step = max(MIN_STEP, step - STEP_ADJUST)
            print(f"step size decreased to {step:.4f}")
        elif key == "h":
            agent.reset()
        else:
            continue

        print_status(agent, step)


def run(port, board, step):
    env, agent = open_board(port, board)
    enforce_calibration(env)  # push per-board calibration + gate, or abort
    print(f"opening {port}, using board id {env.active_ids[0]}")

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        print("homing...")
        env.reset()
        time.sleep(1)

        tty.setcbreak(fd)
        control_loop(agent, step)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        print("\r\nreturning to home")
        env.reset()
        time.sleep(1)
        print("closing port")
        agent.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=DEFAULT_PORT, help="serial port of the delta board")
    parser.add_argument("--id", default=DEFAULT_BOARD, help="board label (BOARD_REGISTRY) or raw id; omit to auto-discover")
    parser.add_argument(
        "--step", type=float, default=DEFAULT_STEP, help="step size per keypress (meters)"
    )
    args = parser.parse_args()
    if not sys.stdin.isatty():
        parser.error("stdin is not a TTY; run this in an interactive terminal")
    run(args.port, args.id, args.step)


if __name__ == "__main__":
    main()
