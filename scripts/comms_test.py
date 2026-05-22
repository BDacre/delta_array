"""Bare comms test for the delta board.

Verifies the host <-> firmware byte path without touching motors. On a healthy
system the script prints PASS in ~2 seconds. Each step explains what it does
and what a healthy result looks like, so a failure narrows the problem to one
stage.
"""

import time

from serial import Serial

from delta_control import delta_array_pb2
from delta_control.constants import DEFAULT_BAUD, FRAME_END, FRAME_START

PORT = "/dev/ttyACM0"
ROBOT_ID = 9
BOOT_WAIT = 1.5
RX_TIMEOUT = 2.0


def main() -> None:

    ser = Serial(PORT, DEFAULT_BAUD, timeout=RX_TIMEOUT)
    time.sleep(BOOT_WAIT)

    # ---- step 1: drain banner -----------------------------------------------
    # If the board just booted, the firmware printed "READY id=9\r\n" at the
    # end of setup(). If the board was already running before we opened the
    # port (USB CDC does not reset the SAMD21), there will be nothing to
    # drain. Either outcome is fine; the real comms check is step 5.
    print("[1/3] draining boot banner  (expect: 'READY id=9' on fresh boot, else nothing)")
    pending = ser.read(ser.in_waiting)
    print(f"      drained from serial buffer: {pending!r}" if pending else "      no banner msg (board was probably already running)")

    # ---- step 2: send a ping ------------------------------------------------
    # Build the smallest possible proto: just id=ROBOT_ID and request_joint_pose=True.
    # Wrap it in the firmware's frame markers (0xA6 ... 0xA7). On the wire
    # this is 6 bytes total:
    #   a6           FRAME_START
    #   08 09        field 1 (id), varint(ROBOT_ID)
    #   20 01        field 4 (request_joint_pose), varint(true)
    #   a7           FRAME_END
    msg = delta_array_pb2.DeltaMessage()
    msg.id = ROBOT_ID
    msg.request_joint_pose = True
    frame = FRAME_START + msg.SerializeToString() + FRAME_END
    print(f"[2/3] TX [{len(frame)} B]: {frame.hex()}")
    if len(frame) != 6:
        print(f"      ERROR: expected 6 B, got {len(frame)} B")
        ser.close()
        return
    else:
        print("      frame looks good, sending...")
    ser.write(frame)

    # ---- step 3: read reply -------------------------------------------------
    # The firmware's request_joint_pose branch in decodeNanopbData builds a
    # DeltaMessage with id=MY_ID and joint_pos populated from the current ADC
    # reads, encodes it with nanopb, and writes it wrapped in 0xA6 ... 0xA7.
    # We read until we see the FRAME_END byte (or RX_TIMEOUT fires), then
    # locate the FRAME_START and decode whatever sits between them as a proto.
    print(f"[3/3] reading reply  (expect: proto frame with id={ROBOT_ID} within {RX_TIMEOUT}s)")
    raw = ser.read_until(FRAME_END)
    ser.close()
    print(f"      RX [{len(raw)} B]: {raw.hex()}")

    start = raw.rfind(FRAME_START)
    if start < 0 or not raw.endswith(FRAME_END):
        print(f"FAIL — no valid frame found in reply")
        return
    payload = raw[start + 1 : -1]

    reply = delta_array_pb2.DeltaMessage()
    try:
        reply.ParseFromString(payload)
    except Exception as exc:
        print(f"FAIL — proto decode error: {exc}")
        return
    print(f"      decoded: id={reply.id}, joint_pos={list(reply.joint_pos)}")

    # ---- verdict ------------------------------------------------------------
    if reply.id == ROBOT_ID:
        print(f"PASS — host<->firmware comms verified end to end (id={reply.id})")
    else:
        print(f"FAIL — expected id={ROBOT_ID}, got id={reply.id}")
        print("       id=0            -> proto decoded but id field absent (firmware response builder broken?)")
        print("       different id    -> wrong board responded (multi-board bus? MY_ID mismatch?)")
        print("       empty/garbage   -> firmware not transmitting (wrong Serial port? stuck in setup()?)")


if __name__ == "__main__":
    main()
