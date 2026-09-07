"""Measure what parallel board I/O actually buys, on the real array.

Every board sits on its own /dev/ttyACM* at 57600 baud, so a whole-array
operation costs one board's round trip times the board count when it is done
sequentially, and roughly ONE round trip when the ports are driven
concurrently. This script is the number that justifies
``DeltaArrayEnv(parallel=True)``, and the regression guard on it: it times one
board alone, then the whole array both ways.

    python tests/bench_serial.py --read           # no motion at all
    python tests/bench_serial.py --write --noop   # motors arm, setpoint unchanged
    python tests/bench_serial.py --all --noop     # both

Read mode sends only status requests, so it is safe at any time. Write mode is
gated behind --noop, the only write this bench will do: it reads each board's
MEASURED position via telemetry and commands exactly that back, so the motors
arm and the PID runs but the setpoint does not change and nothing travels. A
board that does not answer the telemetry read is EXCLUDED rather than commanded
-- get_joint_positions would have handed back a stale default for it, and
writing that is not a no-op, it is a move.

Two things about the method, both learned by getting them wrong:

* The sequential and threaded rounds are INTERLEAVED, not run as two phases.
  A board answers a write more slowly when it was commanded milliseconds rather
  than hundreds of milliseconds ago, and the threaded phase revisits each board
  every ~35 ms where the sequential one takes ~290 ms to come back around. Run
  as two blocks, the threaded phase eats that penalty alone and reads ~2x
  slower than it is; interleaving puts both modes in the same board state.
* --settle inserts an idle gap before every measured round for the same reason.
  Without it the benchmark measures how fast the firmware recovers from being
  hammered, not how fast the transport goes.
"""

import argparse
import functools
import statistics
import sys
import time

from delta_control import DeltaArrayEnv
from delta_control.boards import BOARD_LABELS

DEFAULT_SETTLE_S = 0.1


def _ms(seconds):
    return seconds * 1000.0


def time_per_call(fn, rounds, settle):
    """Wall time of `rounds` calls to fn(), in ms each."""
    times = []
    for _ in range(rounds):
        time.sleep(settle)
        t0 = time.perf_counter()
        fn()
        times.append(_ms(time.perf_counter() - t0))
    return times


def time_batches(env, fn, ids, rounds, settle):
    """Interleaved sequential/threaded whole-array rounds -> (seq_ms, par_ms, failures).

    Per-board failures are counted, not raised: fan_out captures them, and an
    unplugged board should show up as a number in the report rather than as a
    crash that loses the other fifteen measurements.
    """
    seq, par, failures = [], [], 0
    for _ in range(rounds):
        for bucket, parallel in ((seq, False), (par, True)):
            time.sleep(settle)
            t0 = time.perf_counter()
            results = env.fan_out(fn, ids, parallel=parallel)
            bucket.append(_ms(time.perf_counter() - t0))
            failures += sum(isinstance(v, Exception) for v in results.values())
    return seq, par, failures


def summarise(label, times):
    print(f"  {label:<32} {statistics.median(times):7.1f} ms median   "
          f"[{min(times):.0f}, {max(times):.0f}]")


def bench_operation(env, name, fn, rounds, settle, ids=None):
    """One operation: each board alone, then the whole array both ways."""
    ids = list(env.active_ids if ids is None else ids)
    n = len(ids)
    print(f"\n{name}  ({n} boards, {rounds} rounds, {settle * 1000:.0f} ms settle)")

    per_board = []
    for board_id in ids:
        times = time_per_call(functools.partial(fn, env.agents[board_id]), rounds, settle)
        per_board.append(statistics.median(times))
        print(f"    board {str(BOARD_LABELS.get(board_id, board_id)):<9} "
              f"{statistics.median(times):7.1f} ms")

    single = statistics.median(per_board)
    seq, par, failures = time_batches(env, fn, ids, rounds, settle)
    seq_med, par_med = statistics.median(seq), statistics.median(par)

    print()
    summarise("one board", [single])
    summarise(f"x{n} sequential", seq)
    summarise(f"x{n} threaded", par)
    print(f"  {'speedup':<32} {seq_med / par_med:7.1f}x   "
          f"({seq_med:.0f} ms -> {par_med:.0f} ms, {1000 / par_med:.1f} Hz)")
    print(f"  {'threaded vs one board alone':<32} {par_med / single:7.2f}x")
    if failures:
        print(f"  per-board failures: {failures}")
    return {"single": single, "sequential": seq_med, "threaded": par_med}


def noop_targets(env):
    """{board_id: measured joint vector} for boards safe to command back to themselves.

    Telemetry, not get_joint_positions: a silent board makes the latter return
    the last COMMANDED vector -- or the [0.05]*12 constructor default on a board
    never commanded this session -- and writing that back would be a real move.
    A board that fails to report is left out of the write benchmark.
    """
    targets, skipped = {}, []
    for board_id, telemetry in env.get_telemetry_all(parallel=False).items():
        if isinstance(telemetry, Exception) or telemetry is None:
            skipped.append(board_id)
            continue
        targets[board_id] = list(telemetry["position"])
    if skipped:
        print(f"  WARNING: no telemetry from {skipped} -- excluded from the write "
              f"benchmark rather than commanded blind")
    return targets


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--read", action="store_true", help="benchmark the read path (no motion)")
    parser.add_argument("--write", action="store_true", help="benchmark the write path; needs --noop")
    parser.add_argument("--all", action="store_true", help="both read and write")
    parser.add_argument("--noop", action="store_true",
                        help="write each board its own measured position (setpoint unchanged)")
    parser.add_argument("--rounds", type=int, default=10, help="samples per measurement")
    parser.add_argument("--settle", type=float, default=DEFAULT_SETTLE_S,
                        help="idle seconds before each measured round (see module docstring)")
    args = parser.parse_args()

    do_write = args.write or args.all
    do_read = args.read or args.all or not do_write
    if do_write and not args.noop:
        parser.error("--write requires --noop: this bench only ever commands a board "
                     "its own measured position, never an invented setpoint")

    env = DeltaArrayEnv()
    try:
        print(f"{len(env.active_ids)} boards: {list(env.active_ids)}")
        if do_read:
            bench_operation(env, "whoami (tiny -> tiny)",
                            lambda a: a.whoami(), args.rounds, args.settle)
            bench_operation(env, "get_joint_positions (12 floats back)",
                            lambda a: a.get_joint_positions(), args.rounds, args.settle)
            bench_operation(env, "get_telemetry (position + error + pwm)",
                            lambda a: a.get_telemetry(), args.rounds, args.settle)
        if do_write:
            print("\nreading current positions for the no-op write...")
            targets = noop_targets(env)
            if not targets:
                print("no board reported a position; skipping the write benchmark")
                return 1
            bench_operation(env, "move_joint_position (no-op: current position)",
                            lambda a: a.move_joint_position(targets[a.robot_id]),
                            args.rounds, args.settle, ids=list(targets))
    finally:
        env.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
