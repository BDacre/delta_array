"""Smoke test for the open-loop PWM control + telemetry (firmware Phase 0).

Exercises the SetPwmCommand / TelemetryResponse path added for motor
characterization, and checks the firmware safety behaviors:

  1. Telemetry read returns 12-element position/error/pwm.
  2. Driving one motor open-loop moves it and only it; telemetry echoes the
     applied PWM while it's driving.
  3. The motor auto-releases after duration_ms (telemetry pwm -> 0).
  4. An out-of-range motor_index is rejected (ACK_VALIDATION_FAIL -> CommandError).
  5. An over-range PWM is clamped to 255 by the firmware.
  6. stop() releases everything.

Tests 4 and 5 bypass the host-side asserts in DeltaArrayAgent (which would catch
these first) by building the frame directly, so we actually exercise the FIRMWARE
validation/clamp.

Example:
    python test_pwm_control.py --motor 0
"""

import argparse
import time

from delta_control import open_board
from delta_control.constants import NUM_MOTORS
from delta_control.delta_array_agent import CommandError

DEFAULT_PORT = None  # None auto-detects the board's port by scanning /dev/ttyACM*
DEFAULT_BOARD = None  # None auto-discovers; set a BOARD_REGISTRY label or raw id to skip

SEAT_POSITION = 0.05      # m, mid-travel seat so there is room to move either way
SEAT_SETTLE_S = 1.5
MOTION_THRESHOLD = 1.2e-4  # m (~0.12 mm) counts as "moved"


class Checker:
    """Tiny pass/fail tracker so the script reports a summary and exit code."""

    def __init__(self):
        self.passed = 0
        self.failed = 0

    def check(self, name, ok, detail=""):
        mark = "PASS" if ok else "FAIL"
        print(f"  [{mark}] {name}" + (f" — {detail}" if detail else ""))
        if ok:
            self.passed += 1
        else:
            self.failed += 1
        return ok


def _raw_set_pwm(agent, motor_index, pwm, duration_ms):
    """Send a SetPwmCommand directly, bypassing DeltaArrayAgent's host asserts,
    to exercise the firmware's own validation/clamping."""
    msg = agent._envelope()
    msg.joint.set_pwm.motor_index = motor_index
    msg.joint.set_pwm.pwm = pwm
    msg.joint.set_pwm.duration_ms = duration_ms
    agent._send_command(msg)


def seat(agent, motor, baseline, position):
    vec = [baseline] * NUM_MOTORS
    vec[motor] = position
    agent.move_joint_position(vec)
    time.sleep(SEAT_SETTLE_S)


def run(port, board, motor, pwm, duration_ms, baseline):
    env, agent = open_board(port, board)
    print(f"opening {port}, using board id {env.active_ids[0]}")
    print(f"PWM control smoke test on motor {motor} "
          f"(pwm={pwm}, duration={duration_ms} ms)\n")
    c = Checker()

    try:
        # --- 1. Telemetry sanity ---
        print("1. Telemetry read")
        t = agent.get_telemetry()
        ok = (t is not None
              and all(len(t[k]) == NUM_MOTORS for k in ("position", "error", "pwm")))
        c.check("telemetry returns 12x position/error/pwm", ok,
                "" if ok else f"got {t!r}")

        # --- 2. Drive one motor open-loop, observe motion + PWM echo ---
        print("\n2. Open-loop drive")
        seat(agent, motor, baseline, SEAT_POSITION)
        before = agent.get_joint_positions()[motor]
        others_before = [p for i, p in enumerate(agent.get_joint_positions())
                         if i != motor]
        agent.set_motor_pwm(motor, pwm, duration_ms=duration_ms)
        # Poll telemetry while it should still be driving.
        max_pwm_seen = 0
        deadline = time.perf_counter() + (duration_ms / 1000.0) * 0.6
        while time.perf_counter() < deadline:
            tl = agent.get_telemetry()
            if tl:
                max_pwm_seen = max(max_pwm_seen, abs(tl["pwm"][motor]))
            time.sleep(0.02)
        c.check("telemetry echoes applied PWM while driving", max_pwm_seen > 0,
                f"max |pwm| seen = {max_pwm_seen}")
        # Wait out the rest of the window + margin, then check motion.
        time.sleep(duration_ms / 1000.0 + 0.15)
        after = agent.get_joint_positions()[motor]
        moved = abs(after - before) >= MOTION_THRESHOLD
        c.check("driven motor moved", moved,
                f"delta = {(after - before) * 1e3:+.2f} mm")
        others_after = [p for i, p in enumerate(agent.get_joint_positions())
                        if i != motor]
        others_still = all(abs(a - b) < MOTION_THRESHOLD * 3
                           for a, b in zip(others_before, others_after))
        c.check("other motors did not move", others_still)

        # --- 3. Auto-release ---
        print("\n3. Auto-release")
        tl = agent.get_telemetry()
        released = tl is not None and tl["pwm"][motor] == 0
        c.check("motor released after duration (pwm -> 0)", released,
                "" if released else f"pwm[{motor}] = {tl['pwm'][motor] if tl else None}")

        # --- 4. Firmware rejects out-of-range motor index ---
        print("\n4. Index validation (firmware)")
        rejected = False
        try:
            _raw_set_pwm(agent, NUM_MOTORS, 60, 200)  # index 12 is invalid
        except CommandError:
            rejected = True
        c.check("out-of-range motor_index rejected", rejected)

        # --- 5. Firmware clamps over-range PWM to 255 ---
        print("\n5. PWM clamp (firmware)")
        seat(agent, motor, baseline, SEAT_POSITION)
        _raw_set_pwm(agent, motor, 1000, 300)  # magnitude 1000 -> expect clamp 255
        time.sleep(0.05)
        tl = agent.get_telemetry()
        clamped = tl is not None and abs(tl["pwm"][motor]) == 255
        c.check("over-range PWM clamped to 255", clamped,
                "" if clamped else f"pwm[{motor}] = {tl['pwm'][motor] if tl else None}")
        agent.stop()

        # --- 6. Stop releases ---
        print("\n6. Stop")
        agent.stop()
        tl = agent.get_telemetry()
        all_released = tl is not None and all(v == 0 for v in tl["pwm"])
        c.check("all motors released after stop()", all_released)

    finally:
        print("\nreturning to baseline, closing port")
        try:
            agent.stop()
            agent.move_joint_position([baseline] * NUM_MOTORS)
            time.sleep(0.5)
        finally:
            agent.close()

    print(f"\n=== {c.passed} passed, {c.failed} failed ===")
    raise SystemExit(1 if c.failed else 0)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default=DEFAULT_PORT, help="serial port of the delta board")
    p.add_argument("--id", default=DEFAULT_BOARD,
                   help="board label (BOARD_REGISTRY) or raw id; omit to auto-discover")
    p.add_argument("--motor", type=int, default=0, help=f"motor index 0..{NUM_MOTORS - 1}")
    p.add_argument("--pwm", type=int, default=120,
                   help="signed PWM to drive during the test (-255..255)")
    p.add_argument("--duration-ms", type=int, default=400,
                   help="open-loop drive window (ms)")
    p.add_argument("--baseline", type=float, default=0.05,
                   help="hold position for the other 11 motors (m)")
    args = p.parse_args()
    if not (0 <= args.motor < NUM_MOTORS):
        raise SystemExit(f"--motor must be 0..{NUM_MOTORS - 1}, got {args.motor}")
    if not (-255 <= args.pwm <= 255):
        raise SystemExit(f"--pwm must be -255..255, got {args.pwm}")
    run(args.port, args.id, args.motor, args.pwm, args.duration_ms, args.baseline)


if __name__ == "__main__":
    main()
