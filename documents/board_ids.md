# Board IDs — how they work

## Summary

Every board runs the **same firmware image**. Instead of a hard-coded id, each
board derives a unique id at boot from its **SAMD21 128-bit factory serial
number** (read from NVM at `0x0080A00C / 0x0080A040 / 0x0080A044 / 0x0080A048`).
The 16 serial bytes are hashed with **FNV-1a** into a 32-bit value, masked to 31
bits (`& 0x7FFFFFFF`, always a positive `int32`) and forced non-zero. This id is
stamped on every reply and is what the host uses to address the board.

- `computeChipId()` / `my_id` in `microcontroller/delta_array_12_motor/src/main.cpp`.
- Deterministic: same chip → same id, every boot. Verified stable and
  port-independent (see below).

## Backdoor / broadcast id

`BROADCAST_ID = 0`. A board also accepts frames addressed to id `0`, so it can be
controlled **before its id is known**. Replies always carry the board's real
`my_id`, never `0`. On the current one-board-per-port setup this is safe; on a
shared multi-board bus a broadcast would make all boards answer at once.

## whoami

`StatusFrame.id_req` → the board replies `id_resp` with its `my_id`. This is how
the host learns an id it doesn't yet know.

## Host discovery (Python)

Topology: **one board per USB serial port**. Flow:

1. **Filter ports by USB VID:PID first.** Delta boards are Adafruit Feather M0s
   (`239A:800B`). `list_board_ports()` reads VID:PID from the OS *without opening
   the port*, so unrelated CDC devices (e.g. an Arduino Uno, `2341:0043`) are
   never opened and never slow discovery.
2. **Broadcast whoami** on each Feather port to learn its id (`discover_board_id`).
3. **`DeltaArrayEnv`** opens one serial connection per board and keys agents by
   id: `env.agents[id]`, `env.agent_for("label")`, `env.port_for(id)`,
   `env.active_ids`. One env manages many ports/boards.
4. **`BOARD_REGISTRY`** (in `constants.py`) maps friendly labels → ids, so code
   can address boards by name instead of the raw number. Populate it with
   `scripts/identify_board.py`.

The port number (`ttyACM0` vs `ttyACM1`) is irrelevant — addressing is by id,
mapped to whatever port that id is found on this session.

### Robustness note

`read_frame()` bails after `MAX_HUNT_BYTES` (4096) discarded bytes, so a chatty
non-protocol port fails cleanly instead of hanging forever hunting for the start
marker.

## Verified

- `whoami id` is stable across repeated reads, power state, and port
  re-enumeration (Uno plugged/unplugged, ports swapping) — always the same value
  for a given chip.
- `whoami id == FNV-1a(USB serial number)` exactly. The USB `serial_number`
  reported by the OS is the same SAMD21 serial the firmware hashes.
- Diagnostic: `scripts/id_stability_test.py` (keys on USB serial, samples whoami
  N times, compares to the expected hash).

## Future avenue — compute id from USB serial, skip the port open

Because `whoami id == FNV-1a(USB serial_number)` and the USB serial is available
from the OS *without opening the port*, the host could map **port → id purely
from USB metadata** — no serial open, no board reset, instant discovery. Would
replace the per-port whoami probe with a pure hash of `port.serial_number`.

Not adopted yet — validate first that the match holds:
- across a **reflash** of the same board, and
- on **multiple physical boards** (rule out coincidence / byte-order quirks).

Keep whoami as the authoritative fallback; treat the serial-derived id as a fast
path guarded by that check.
