"""Diagnose motors that have stopped moving on one board.

Built around the two ways a healthy-looking motor lies about itself, both of
which depend on WHAT you commanded rather than on the motor:

  * gravity does the work. A joint with no drive still reaches a LOW target by
    falling, so "everything moved to the start position together" proves nothing.
    Only a commanded LIFT tests the drive path, so every test here runs both
    directions and reports them separately.
  * every motor got the same number. A command that sends all 12 joints one value
    (move_all_to_location, an all-low seat) cannot expose crossed or mislabelled
    wiring -- a swapped pair converges together and looks perfect. Only a
    differential command (per-delta, one motor at a time) shows it. --map pulses
    each motor and reports which sensor channel actually answers.

A motor that works under one command shape and not the other is almost never a
dead motor; it is which motor the command reached.

Phase 1 (always runs, read-only, no motion):
  * board id / BOARD_REGISTRY label
  * live per-motor config (deadband + bias) diffed against the calibration JSON.
    Firmware config is RAM-only, so a board that rebooted is back on compiled
    defaults (0.8 mm deadband, zero bias) without saying so.
  * telemetry at rest: position, jitter, error and last PWM per motor, flagging
    channels that read zero, sit at the ADC's full-scale rail, show no noise, or
    are still being driven.

Phase 2 (--move, drives motors), per motor, each direction separately:
  * closed-loop probe: step the commanded position up, then down, watching
    telemetry through each move. PWM that stays 0 means the board thinks the
    joint is already there (sensor/config); PWM without travel means the drive
    path or a jam.
  * open-loop ramp: rising PWM pulses with the PID bypassed (SetPwmCommand), one
    ramp per direction, recording the breakaway PWM and which way the joint went.
  * with --ask: a strong pulse per direction and a prompt for what you SAW --
    the only check not taken through the sensor.

A direction that fails while the other works points at one half-bridge. On the
MotorShield the direction pins are PCA9685 outputs (FORWARD drives IN1, BACKWARD
drives IN2; firmware pwm<0 = FORWARD, pwm>0 = BACKWARD), so the script names the
exact pin for the dead direction.

Hardware sharing mirrors the firmware's motors[] / adcs[] / channels[] tables:
motors 0,1,10,11 on the shield at 0x62; 2,3,8,9 on 0x60; 4,5,6,7 on 0x61. Within
a shield M1/M2 share one TB6612 and M3/M4 the other, so 2 and 3 are one chip and
8 and 9 the other half of that same shield.

Safety: Phase 1 sends only status frames. In Phase 2 every open-loop pulse is
time-bounded by the firmware's auto-release, and SetPwmCommand releases all other
motors first -- they coast, so a raised leg will drop.

Example:
    python diagnose_dead_motor.py --motors 2,3            # read-only triage
    python diagnose_dead_motor.py --motors 2,3 --move     # + drive tests
"""

import argparse
import time

from delta_control import (
    diff_calibration,
    load_calibration,
    open_board,
)
from delta_control.boards import BOARD_LABELS
from delta_control.constants import MAX_JOINT_POS, MIN_JOINT_POS, NUM_MOTORS

DEFAULT_PORT = None   # None auto-detects the board's port by scanning /dev/ttyACM*
DEFAULT_BOARD = None  # None auto-discovers; set a BOARD_REGISTRY label or raw id to skip

# --- firmware mirror: microcontroller/delta_array_12_motor/src -------------
# motors[], adcs[], channels[] from variables_and_parameters.cpp plus the I2C
# addresses from variables_and_parameters.h. If the firmware wiring changes,
# these tables have to change with it.
SHIELD_ADDR = {"MC0": 0x62, "MC1": 0x60, "MC2": 0x61}
ADC_ADDR = {"ADC0": 0x48, "ADC1": 0x49, "ADC2": 0x4A}

MOTOR_SHIELD = ["MC0", "MC0", "MC1", "MC1", "MC2", "MC2",
                "MC2", "MC2", "MC1", "MC1", "MC0", "MC0"]
MOTOR_SHIELD_CH = [1, 2, 1, 2, 1, 2, 3, 4, 3, 4, 3, 4]
MOTOR_ADC = ["ADC2", "ADC2", "ADC1", "ADC1", "ADC0", "ADC0",
             "ADC0", "ADC0", "ADC1", "ADC1", "ADC2", "ADC2"]
MOTOR_ADC_CH = [0, 1, 0, 1, 0, 1, 2, 3, 2, 3, 2, 3]

# PCA9685 output per shield port, from Adafruit_MotorShield::getMotor():
#   M1 -> pwm 8, in2 9,  in1 10      M3 -> pwm 2, in2 3, in1 4
#   M2 -> pwm 13, in2 12, in1 11     M4 -> pwm 7, in2 6, in1 5
# run(FORWARD) drives IN1 high, run(BACKWARD) drives IN2 high, so a direction
# that fails on its own implicates one IN pin (or that half of the TB6612).
PCA_PINS = {1: {"pwm": 8, "in2": 9, "in1": 10},
            2: {"pwm": 13, "in2": 12, "in1": 11},
            3: {"pwm": 2, "in2": 3, "in1": 4},
            4: {"pwm": 7, "in2": 6, "in1": 5}}

# ADC_TO_POSITION in the firmware: metres of travel per ADS1015 count.
ADC_COUNT_M = 6e-5
# ADS1015 single-ended is 12-bit, so 2047 counts is full scale. A position at or
# above this isn't a real joint position -- the input is pinned high (open wiper,
# short to Vdd) rather than reporting travel.
ADC_FULL_SCALE_M = 2047 * ADC_COUNT_M

ZERO_POS_M = 1.5 * ADC_COUNT_M   # below this the ADC is reading nothing
RAIL_MARGIN_M = 2 * ADC_COUNT_M

MOTION_THRESHOLD = 1.2e-4  # m (~0.12 mm, 2 ADC counts) counts as "moved"

REST_SAMPLES = 8
REST_INTERVAL_S = 0.1

SEAT_POSITION = 0.05   # m, mid-travel so a probe has room in both directions
SEAT_SETTLE_S = 1.5
STEP_M = 0.015         # m, closed-loop probe step (well past any deadband)
PROBE_SECONDS = 2.5    # how long to watch telemetry during one step
PROBE_HZ = 12.0
POST_PULSE_S = 0.08    # settle after a pulse auto-releases, before reading
BOUND_MARGIN = 0.004   # m, keep probes this far inside the travel limits

# Firmware sign convention: pwm > 0 -> BACKWARD (IN2), pwm < 0 -> FORWARD (IN1).
PWM_SIGNS = ((+1, "BACKWARD", "in2"), (-1, "FORWARD", "in1"))

V_OK = "OK"
V_ONE_DIRECTION = "ONE_DIRECTION_DEAD"
V_NO_DRIVE_PATH = "NO_DRIVE_PATH"
V_NO_DRIVE_COMMANDED = "NO_DRIVE_COMMANDED"
V_SENSOR_DEAD = "SENSOR_DEAD"
V_SENSOR_RAIL = "SENSOR_AT_RAIL"
V_SENSOR_FROZEN = "SENSOR_FROZEN"
V_WEAK = "WEAK_OR_LOAD_LIMITED"
V_UNKNOWN = "INCONCLUSIVE"


# --- hardware map helpers -------------------------------------------------

def driver_chip(motor):
    """Which TB6612 on the shield drives this motor: M1/M2 -> 'A', M3/M4 -> 'B'."""
    return "A" if MOTOR_SHIELD_CH[motor] <= 2 else "B"


def pins(motor):
    return PCA_PINS[MOTOR_SHIELD_CH[motor]]


def describe(motor):
    sh = MOTOR_SHIELD[motor]
    adc = MOTOR_ADC[motor]
    return (f"{sh}({SHIELD_ADDR[sh]:#04x}) M{MOTOR_SHIELD_CH[motor]} chip{driver_chip(motor)}"
            f"  |  {adc}({ADC_ADDR[adc]:#04x}) ch{MOTOR_ADC_CH[motor]}")


def describe_pins(motor):
    p = pins(motor)
    sh = MOTOR_SHIELD[motor]
    return (f"{sh}({SHIELD_ADDR[sh]:#04x}) M{MOTOR_SHIELD_CH[motor]} PCA9685 pins: "
            f"pwm {p['pwm']}, in1 {p['in1']} (FORWARD), in2 {p['in2']} (BACKWARD)")


def chip_partners(motor):
    """Other motors driven by the same TB6612 (same shield, same M1/M2 or M3/M4 half)."""
    return [m for m in range(NUM_MOTORS)
            if m != motor
            and MOTOR_SHIELD[m] == MOTOR_SHIELD[motor]
            and driver_chip(m) == driver_chip(motor)]


def shield_mates(motor):
    """Motors on the same shield but the OTHER TB6612 -- the control for a dead chip."""
    return [m for m in range(NUM_MOTORS)
            if MOTOR_SHIELD[m] == MOTOR_SHIELD[motor]
            and driver_chip(m) != driver_chip(motor)]


def control_motor(suspects):
    """A motor on a different shield entirely, to prove the test itself works."""
    bad_shields = {MOTOR_SHIELD[m] for m in suspects}
    for m in range(NUM_MOTORS):
        if MOTOR_SHIELD[m] not in bad_shields:
            return m
    return None


def build_test_set(suspects, include_siblings):
    """Suspects first, then same-chip partners, shield mates, and one control."""
    order = list(suspects)
    if include_siblings:
        for m in suspects:
            for extra in chip_partners(m) + shield_mates(m):
                if extra not in order:
                    order.append(extra)
        ctl = control_motor(suspects)
        if ctl is not None and ctl not in order:
            order.append(ctl)
    return order


def shared_hardware(motors):
    """What a failing set has in common -- the part worth suspecting."""
    if len(motors) < 2:
        return []
    lines = []
    chips = {(MOTOR_SHIELD[m], driver_chip(m)) for m in motors}
    shields = {MOTOR_SHIELD[m] for m in motors}
    adcs = {MOTOR_ADC[m] for m in motors}

    if len(chips) == 1:
        sh, chip = next(iter(chips))
        mates = shield_mates(motors[0])
        ports = "/".join(f"M{MOTOR_SHIELD_CH[m]}" for m in sorted(motors))
        lines.append(
            f"all of {motors} are driven by ONE TB6612: {sh}({SHIELD_ADDR[sh]:#04x}) "
            f"chip{chip} ({ports}). Motors {mates} are the other chip on the same "
            f"shield -- if those still lift, the shield's PCA9685 and motor supply "
            f"are fine and chip{chip} or its harness is the suspect."
        )
    elif len(shields) == 1:
        sh = next(iter(shields))
        lines.append(
            f"all of {motors} are on shield {sh}({SHIELD_ADDR[sh]:#04x}) but split "
            f"across both TB6612 halves -- that points at the shield (PCA9685, motor "
            f"supply, ground) rather than one driver chip."
        )
    else:
        lines.append(
            f"motors {motors} span shields {sorted(shields)} -- no single driver "
            f"explains all of them; treat them as separate faults."
        )

    if len(adcs) == 1:
        adc = next(iter(adcs))
        chans = sorted(MOTOR_ADC_CH[m] for m in motors)
        lines.append(
            f"they also share feedback ADC {adc}({ADC_ADDR[adc]:#04x}), channels "
            f"{chans}. That chip is answering if the board answers at all: "
            f"readADC_SingleEnded() spins in an unbounded while(!conversionComplete()) "
            f"loop, so an unreachable ADS1015 would wedge the firmware and kill all 12 "
            f"motors plus serial. A per-channel pot/wiring fault is still possible."
        )
    return lines


def shared_direction(dead_dirs):
    """If every failing motor lost the SAME direction, name the pins they share."""
    motors = sorted(dead_dirs)
    if len(motors) < 2:
        return []
    common = set.intersection(*(set(dead_dirs[m]) for m in motors))
    lines = []
    for sign in common:
        _, run_name, pin_key = next(s for s in PWM_SIGNS if s[0] == sign)
        pin_list = ", ".join(f"motor {m} -> pin {pins(m)[pin_key]}" for m in motors)
        lines.append(
            f"every one of {motors} is dead in the SAME direction "
            f"(pwm {sign:+d} = {run_name}, driven by {pin_key.upper()}): {pin_list}. "
            f"One shared cause fits: those PCA9685 outputs, or the matching half of "
            f"the TB6612 on that shield."
        )
    return lines


# --- phase 1: read-only ---------------------------------------------------

def sample_at_rest(agent, samples, interval):
    """Collect telemetry with nothing commanded. Returns per-motor stats."""
    frames = []
    for _ in range(samples):
        t = agent.get_telemetry()
        if t is not None:
            frames.append(t)
        time.sleep(interval)
    if not frames:
        return None
    stats = []
    for i in range(NUM_MOTORS):
        pos = [f["position"][i] for f in frames]
        pwm = [f["pwm"][i] for f in frames]
        stats.append({
            "pos_mean": sum(pos) / len(pos),
            "jitter": max(pos) - min(pos),
            "pwm_max_abs": max(abs(p) for p in pwm),
            "error": frames[-1]["error"][i],
        })
    return stats


def rest_flags(s):
    """Read-only red flags for one motor's at-rest telemetry."""
    flags = []
    if s["pos_mean"] < ZERO_POS_M:
        flags.append((V_SENSOR_DEAD, "position reads 0 counts (open/disconnected wiper)"))
    elif s["pos_mean"] >= ADC_FULL_SCALE_M - RAIL_MARGIN_M:
        flags.append((V_SENSOR_RAIL,
                      f"position {s['pos_mean'] * 1e3:.1f} mm is at the ADC's full-scale "
                      f"rail ({ADC_FULL_SCALE_M * 1e3:.1f} mm), not a real joint position"))
    elif s["jitter"] == 0.0:
        flags.append((V_SENSOR_FROZEN, "position identical across every sample (no ADC "
                                       "noise at all -- the reading may be frozen)"))
    if not (MIN_JOINT_POS <= s["pos_mean"] <= MAX_JOINT_POS):
        flags.append((V_UNKNOWN,
                      f"position {s['pos_mean'] * 1e3:.1f} mm is outside the travel "
                      f"limits [{MIN_JOINT_POS * 1e3:.1f}, {MAX_JOINT_POS * 1e3:.1f}] mm "
                      f"(sitting on a hard stop, or miscalibrated)"))
    if s["pwm_max_abs"] > 0:
        flags.append((V_UNKNOWN, f"still being driven at rest (|pwm| up to "
                                 f"{s['pwm_max_abs']}) -- stuck in a move or re-arming"))
    return flags


def phase1(agent, board_id, suspects):
    """Config + at-rest telemetry. Returns (stats, flags_by_motor)."""
    label = BOARD_LABELS.get(board_id)
    print(f"\nboard {board_id}" + (f" (registered as {label!r})" if label else
                                   " (not in BOARD_REGISTRY)"))

    print("\n-- config readback --")
    cfg = agent.get_config()
    if cfg is None:
        print("  board returned no config_resp (firmware predates ConfigRequest)")
    else:
        print(f"  brake_at_setpoint = {cfg['brake_at_setpoint']}")
        for m in suspects:
            print(f"  motor {m:2d}  deadband {cfg['deadband'][m] * 1e3:.4f} mm  "
                  f"bias_fwd {cfg['bias_fwd'][m]:3d}  bias_back {cfg['bias_back'][m]:3d}")
        try:
            calib = load_calibration(board_id)
        except ValueError as exc:
            calib = None
            print(f"  calibration lookup failed: {exc}")
        if calib is None:
            print("  no calibration file for this board (running firmware defaults)")
        else:
            mismatches = diff_calibration(cfg, calib)
            if mismatches:
                print(f"  {len(mismatches)} mismatch(es) vs the calibration JSON "
                      f"(a reboot reverts config to compiled defaults):")
                for line in mismatches:
                    print(f"    {line}")
            else:
                print("  live config matches the calibration JSON")

    print(f"\n-- telemetry at rest ({REST_SAMPLES} samples) --")
    stats = sample_at_rest(agent, REST_SAMPLES, REST_INTERVAL_S)
    if stats is None:
        raise SystemExit("board returned no telemetry_resp -- cannot diagnose")

    print(f"  {'motor':>5}  {'pos(mm)':>8}  {'jitter(mm)':>10}  {'err(mm)':>8}  "
          f"{'|pwm|':>5}  hardware")
    flags = {}
    for i in range(NUM_MOTORS):
        s = stats[i]
        flags[i] = rest_flags(s)
        mark = "  <-- suspect" if i in suspects else ""
        print(f"  {i:>5}  {s['pos_mean'] * 1e3:>8.2f}  {s['jitter'] * 1e3:>10.3f}  "
              f"{s['error'] * 1e3:>8.2f}  {s['pwm_max_abs']:>5}  {describe(i)}{mark}")
    for i in range(NUM_MOTORS):
        for _, text in flags[i]:
            print(f"  ! motor {i:2d}: {text}")
    for m in suspects:
        print(f"  motor {m:2d}: {describe_pins(m)}")
    return stats, flags


# --- phase 2: motion, one direction at a time -----------------------------

def hold_vector(agent):
    """Current positions, clipped into the travel limits, as a hold target."""
    return [min(max(p, MIN_JOINT_POS), MAX_JOINT_POS)
            for p in agent.get_joint_positions()]


def seat(agent, motor, position):
    """PID this joint toward `position`, holding the others where they are."""
    vec = hold_vector(agent)
    vec[motor] = position
    agent.move_joint_position(vec)
    time.sleep(SEAT_SETTLE_S)


def closed_loop_probe(agent, motor, step, direction):
    """Step the commanded position by `direction * step` and watch telemetry.

    Returns (max_abs_pwm, net_travel_m, target_m). Direction +1 raises the
    commanded joint position, -1 lowers it -- the lift is what the screw-access
    move does and the only half that actually tests the drive.
    """
    vec = hold_vector(agent)
    start = vec[motor]
    lo, hi = MIN_JOINT_POS + BOUND_MARGIN, MAX_JOINT_POS - BOUND_MARGIN
    target = min(max(start + direction * step, lo), hi)
    if abs(target - start) < MOTION_THRESHOLD:
        # No room this way (joint already at that end): seat mid and retry once.
        seat(agent, motor, SEAT_POSITION)
        vec = hold_vector(agent)
        start = vec[motor]
        target = min(max(start + direction * step, lo), hi)
    vec[motor] = target
    agent.move_joint_position(vec)

    max_pwm, seen = 0, [start]
    deadline = time.time() + PROBE_SECONDS
    while time.time() < deadline:
        t = agent.get_telemetry()
        if t is not None:
            max_pwm = max(max_pwm, abs(t["pwm"][motor]))
            seen.append(t["position"][motor])
        time.sleep(1.0 / PROBE_HZ)
    return max_pwm, seen[-1] - seen[0], target


def openloop_ramp(agent, motor, sign, args):
    """Rising open-loop pulses in one direction. Returns (breakaway_pwm, net_m)."""
    seat(agent, motor, SEAT_POSITION)
    lo, hi = MIN_JOINT_POS + BOUND_MARGIN, MAX_JOINT_POS - BOUND_MARGIN
    pwm = args.pwm_start
    while pwm <= args.pwm_max:
        before = agent.get_joint_positions()[motor]
        if not (lo <= before <= hi):
            seat(agent, motor, SEAT_POSITION)
            before = agent.get_joint_positions()[motor]
        agent.set_motor_pwm(motor, sign * pwm, duration_ms=args.pulse_ms)
        time.sleep(args.pulse_ms / 1000.0 + POST_PULSE_S)
        after = agent.get_joint_positions()[motor]
        if abs(after - before) >= MOTION_THRESHOLD:
            return sign * pwm, after - before
        pwm += args.pwm_step
    return None, 0.0


def map_motor_to_sensor(agent, motors, args):
    """Pulse each motor open-loop and report which POSITION channel answers.

    A motor that only misbehaves when its target differs from its neighbours' --
    fine when every joint is commanded to the same value, wrong when commanded
    per-delta -- is the signature of a transposition, not a failure. Any move that
    sends all 12 motors the same number hides a swap completely; only a
    differential command exposes it.

    Driving index i and watching index j move proves the drive and feedback for
    those two indices are crossed. If index i moves its own sensor but the wrong
    physical leg, the pair is internally consistent and only mislabelled -- that
    one needs eyes, via identify_motor_order.py.
    """
    print("\n-- motor -> sensor map --")
    observed = {}
    for motor in motors:
        seat(agent, motor, SEAT_POSITION)
        movers = []
        for sign, run_name, _ in PWM_SIGNS:
            before = agent.get_joint_positions()
            agent.set_motor_pwm(motor, sign * args.pwm_max, duration_ms=args.pulse_ms * 2)
            time.sleep(args.pulse_ms * 2 / 1000.0 + POST_PULSE_S)
            after = agent.get_joint_positions()
            movers = [(i, after[i] - before[i]) for i in range(NUM_MOTORS)
                      if abs(after[i] - before[i]) >= MOTION_THRESHOLD]
            if movers:
                break
            print(f"  motor {motor:2d}: no channel responded to {run_name}")
        observed[motor] = [i for i, _ in movers]
        if not movers:
            print(f"  motor {motor:2d}: NOTHING responded in either direction "
                  f"(drive path, or the joint is against a stop)")
        else:
            detail = ", ".join(f"channel {i} {d * 1e3:+.2f} mm" for i, d in movers)
            verdict = ("consistent" if observed[motor] == [motor]
                       else "CROSSED -- expected only channel "
                            f"{motor}")
            print(f"  motor {motor:2d}: {detail}  [{verdict}]")
    return observed


def report_map(observed):
    """Name the permutation if the motor->sensor map isn't the identity."""
    crossed = {m: chans for m, chans in observed.items()
               if chans and chans != [m]}
    if not crossed:
        print("\nmotor -> sensor map is the identity for every motor tested: drive and "
              "feedback are not crossed. If a leg still moves on the wrong delta, the "
              "harness-to-leg routing is mislabelled -- confirm with identify_motor_order.py.")
        return
    print("\nmotor -> sensor map is NOT the identity:")
    for m, chans in crossed.items():
        print(f"  driving motor {m} moves sensor channel(s) {chans}, not {m}")
    pairs = [(a, b) for a in crossed for b in crossed
             if a < b and crossed[a] == [b] and crossed[b] == [a]]
    for a, b in pairs:
        print(f"  motors {a} and {b} are a clean TRANSPOSITION -- their harnesses are "
              f"swapped ({MOTOR_SHIELD[a]} M{MOTOR_SHIELD_CH[a]} <-> "
              f"M{MOTOR_SHIELD_CH[b]}, adjacent terminals). Every command that sends "
              f"both the same value hides this; only per-delta targets expose it.")


def eyeball_test(agent, motor, sign, run_name, args):
    """Drive hard one way and ask what was SEEN.

    Every other check reads position through the sensor, so a dead sensor and a
    dead actuator look identical to them. The eye is the only instrument here
    that doesn't go through the ADC.
    """
    print(f"    watch motor {motor} -- driving {run_name} at |pwm|={args.pwm_max}")
    agent.set_motor_pwm(motor, sign * args.pwm_max, duration_ms=args.pulse_ms * 3)
    time.sleep(args.pulse_ms * 3 / 1000.0 + POST_PULSE_S)
    answer = input("    did the leg physically move (or the motor audibly try)? [y/N] ")
    return answer.strip().lower().startswith("y")


def phase2(agent, motors, args, rest_flags_by_motor):
    """Per-motor, per-direction drive tests. Returns {motor: result dict}."""
    print("\n-- drive tests (each direction separately) --")
    results = {}
    for motor in motors:
        print(f"  motor {motor:2d}  ({describe(motor)})")
        seat(agent, motor, SEAT_POSITION)

        closed = {}
        for direction, name in ((+1, "raise"), (-1, "lower")):
            max_pwm, net, target = closed_loop_probe(agent, motor, args.step, direction)
            closed[direction] = (max_pwm, net)
            print(f"    closed loop {name:6s}: target {target * 1e3:5.1f} mm  "
                  f"|pwm| up to {max_pwm:3d}  moved {net * 1e3:+6.2f} mm")

        openloop, seen_moving = {}, {}
        for sign, run_name, pin_key in PWM_SIGNS:
            bp, net = openloop_ramp(agent, motor, sign, args)
            openloop[sign] = (bp, net)
            if bp is None:
                print(f"    open loop  {run_name:8s} (pin {pins(motor)[pin_key]:2d}): "
                      f"no motion up to |pwm|={args.pwm_max}")
                if args.ask:
                    seen_moving[sign] = eyeball_test(agent, motor, sign, run_name, args)
            else:
                print(f"    open loop  {run_name:8s} (pin {pins(motor)[pin_key]:2d}): "
                      f"broke free at pwm={bp:+d}, moved {net * 1e3:+.2f} mm")

        sensor_suspect = any(code in (V_SENSOR_DEAD, V_SENSOR_RAIL, V_SENSOR_FROZEN)
                             for code, _ in rest_flags_by_motor[motor])
        results[motor] = classify(motor, closed, openloop, seen_moving, sensor_suspect)
        print(f"    -> {results[motor]['verdict']}: {results[motor]['detail']}")
    return results


def classify(motor, closed, openloop, seen_moving, sensor_suspect):
    """Turn one motor's per-direction measurements into a verdict."""
    dead_dirs = [s for s in openloop if openloop[s][0] is None]
    live_dirs = [s for s in openloop if openloop[s][0] is not None]
    no_pwm = [d for d in closed if closed[d][0] == 0]
    no_travel = [d for d in closed if abs(closed[d][1]) < MOTION_THRESHOLD]

    # The eye overrides the sensor: motion seen but never measured is a feedback
    # fault, however the automated numbers came out.
    if any(v is True for v in seen_moving.values()):
        return {"verdict": V_SENSOR_DEAD, "dead_dirs": [],
                "detail": "it physically moves but the sensor never reports it -- the "
                          "feedback path: pot, wiper wiring, or that ADC channel. The "
                          "PID is blind, so it stops short or drives into a stop."}

    if not live_dirs:
        detail = ("no motion in EITHER direction, open-loop, with the PID bypassed -- "
                  "the drive path: TB6612 output, motor harness/connector, the motor "
                  "itself, or a mechanical jam. Note this is consistent with the joint "
                  "still reaching a LOW target during a group move: that is gravity, "
                  "not drive.")
        if any(v is False for v in seen_moving.values()):
            detail += " Confirmed by eye: nothing moved."
        return {"verdict": V_NO_DRIVE_PATH, "dead_dirs": list(dead_dirs), "detail": detail}

    if dead_dirs:
        sign = dead_dirs[0]
        _, run_name, pin_key = next(s for s in PWM_SIGNS if s[0] == sign)
        good = live_dirs[0]
        return {"verdict": V_ONE_DIRECTION, "dead_dirs": list(dead_dirs),
                "detail": f"drives fine at pwm {good:+d} but nothing at pwm {sign:+d} "
                          f"({run_name}, IN pin {pins(motor)[pin_key]} on "
                          f"{MOTOR_SHIELD[motor]}). One half-bridge or one PCA9685 "
                          f"output is dead. Gravity covers for the working direction, "
                          f"which is why it looks fine going down and dead going up."}

    if no_travel and not no_pwm:
        return {"verdict": V_WEAK, "dead_dirs": [],
                "detail": f"moves open-loop but the closed-loop step {no_travel} didn't "
                          f"budge it while PWM was being applied -- torque marginal under "
                          f"the assembled load, or the bias/deadband for this motor is "
                          f"wrong. Compare its breakaway PWM against its siblings."}

    if no_pwm:
        detail = (f"the board commanded no PWM at all for direction(s) {no_pwm}: it "
                  f"believes the joint is already at target. Cause is upstream of the "
                  f"driver -- the reported position, or an oversized deadband.")
        if sensor_suspect:
            detail += " Its at-rest telemetry is already flagged, which fits."
        return {"verdict": V_NO_DRIVE_COMMANDED, "dead_dirs": [], "detail": detail}

    return {"verdict": V_OK, "dead_dirs": [],
            "detail": "moves under PID and open-loop in both directions. If the fault "
                      "only shows with the platform assembled, it is load-dependent: "
                      "re-run under load, or compare breakaway PWM against siblings."}


# --- reporting ------------------------------------------------------------

def summarize(suspects, rest_flags_by_motor, drive_results):
    print("\n== summary ==")
    for m in suspects:
        print(f"\nmotor {m:2d}  {describe(m)}")
        if rest_flags_by_motor[m]:
            for _, text in rest_flags_by_motor[m]:
                print(f"  at rest: {text}")
        else:
            print("  at rest: nothing anomalous")
        if drive_results and m in drive_results:
            print(f"  verdict: {drive_results[m]['verdict']} -- "
                  f"{drive_results[m]['detail']}")

    if not drive_results:
        print("\nshared hardware of the motors under test:")
        for line in shared_hardware(sorted(suspects)):
            print(f"  * {line}")
        print("  (read-only pass -- add --move to drive the motors and get a verdict)")
        return

    working = sorted(m for m, r in drive_results.items() if r["verdict"] == V_OK)
    broken = sorted(m for m, r in drive_results.items() if r["verdict"] != V_OK)
    if working:
        print(f"\nstill working: {working}")
    if broken:
        print(f"not working: {broken}")
        for line in shared_hardware(broken):
            print(f"  * {line}")
        dead_dirs = {m: drive_results[m]["dead_dirs"] for m in broken
                     if drive_results[m]["dead_dirs"]}
        for line in shared_direction(dead_dirs):
            print(f"  * {line}")


def parse_motors(text):
    motors = []
    for part in text.split(","):
        part = part.strip()
        if not part:
            continue
        m = int(part)
        if not (0 <= m < NUM_MOTORS):
            raise SystemExit(f"--motors entries must be 0..{NUM_MOTORS - 1}, got {m}")
        if m not in motors:
            motors.append(m)
    if not motors:
        raise SystemExit("--motors needs at least one motor index")
    return motors


def run(args):
    suspects = parse_motors(args.motors)
    env, agent = open_board(args.port, args.id)
    board_id = env.active_ids[0]
    try:
        _, flags = phase1(agent, board_id, suspects)
        drive_results = None
        if args.map:
            report_map(map_motor_to_sensor(agent, build_test_set(suspects, args.siblings),
                                           args))
        if args.move:
            test_set = build_test_set(suspects, args.siblings)
            extra = [m for m in test_set if m not in suspects]
            print(f"\ntesting motors {test_set}"
                  + (f" (suspects {suspects} plus {extra} as controls)" if extra else ""))
            drive_results = phase2(agent, test_set, args, flags)
        summarize(suspects, flags, drive_results)
    finally:
        if args.move:
            try:
                agent.stop()
            except Exception:
                pass
        print("\nclosing port")
        agent.close()


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default=DEFAULT_PORT, help="serial port of the delta board")
    p.add_argument("--id", default=DEFAULT_BOARD,
                   help="board label (BOARD_REGISTRY) or raw id; omit to auto-discover")
    p.add_argument("--motors", default="2,3",
                   help="comma-separated motor indices to diagnose (default 2,3)")
    p.add_argument("--move", action="store_true",
                   help="run the drive tests (phase 2). Without this the script only reads.")
    p.add_argument("--map", action="store_true",
                   help="pulse each motor and report which sensor channel answers -- "
                        "catches crossed drive/feedback wiring, which only shows up when "
                        "motors are commanded to DIFFERENT targets")
    p.add_argument("--no-siblings", dest="siblings", action="store_false",
                   help="test only the named motors, not their same-shield controls")
    p.add_argument("--ask", dest="ask", action="store_true", default=True,
                   help="prompt for what you saw when a direction senses no motion "
                        "(default on)")
    p.add_argument("--no-ask", dest="ask", action="store_false",
                   help="hands-free: skip the eyeball test that separates a dead "
                        "actuator from a dead sensor")
    p.add_argument("--step", type=float, default=STEP_M,
                   help="closed-loop probe step in m (default 0.015)")
    p.add_argument("--pwm-start", type=int, default=20, help="starting |PWM| for the ramp")
    p.add_argument("--pwm-max", type=int, default=200, help="max |PWM| to try")
    p.add_argument("--pwm-step", type=int, default=20, help="|PWM| increment")
    p.add_argument("--pulse-ms", type=int, default=150,
                   help="open-loop pulse duration (ms)")
    run(p.parse_args())


if __name__ == "__main__":
    main()
