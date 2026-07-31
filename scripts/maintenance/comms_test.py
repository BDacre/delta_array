"""Bare comms test for the delta board.

Verifies the host <-> firmware byte path without touching motors. On a healthy
system the script prints PASS in ~2 seconds. Each step explains what it does
and what a healthy result looks like, so a failure narrows the problem to one
stage.
"""

import time

from serial import Serial

from delta_control import delta_array_pb2
from delta_control.constants import BROADCAST_ID, DEFAULT_BAUD, FRAME_END, FRAME_START
from delta_control.transport import ProtoTransport, crc16_ccitt

PORT = "/dev/ttyACM0"
# Address the ping to the backdoor/broadcast id: firmware now derives each
# board's id from its chip serial, so we don't know it up front. The board still
# stamps its real (non-zero) id on the reply, which we check below.
TX_ID = BROADCAST_ID
BOOT_WAIT = 1.5
RX_TIMEOUT = 2.0


def main() -> None:

    ser = Serial(PORT, DEFAULT_BAUD, timeout=RX_TIMEOUT)
    time.sleep(BOOT_WAIT)

    # ---- step 1: drain banner -----------------------------------------------
    # Drain anything already buffered on the port (stray bytes, a boot banner if
    # the firmware ever adds one). USB CDC does not reset the SAMD21, so a board
    # already running before we opened the port may have nothing to drain.
    # Either outcome is fine; the real comms check is step 3.
    print("Draining serial in case there is anything in buffer")
    pending = ser.read(ser.in_waiting)
    print(f"      drained from serial buffer: {pending!r}" if pending else "      nothing to drain")

    # ---- step 2: send a ping ------------------------------------------------
    # Build the smallest possible proto payload: id=TX_ID (broadcast=0) with a
    # StatusFrame.pose_req. Because proto3 omits scalar fields set to their
    # default, id=0 is NOT emitted, so the payload is 4 bytes:
    #   12 02        field 2 (status), length-delimited, len=2
    #     0a 00        nested: field 1 (pose_req), length-delimited, len=0
    # The firmware decodes the absent id as 0, which matches BROADCAST_ID.
    #
    # Wire frame is [START][LEN_LO LEN_HI][payload][CRC_LO CRC_HI][END]. We build
    # it by hand here to exercise the exact bytes on the wire; ProtoTransport.send()
    # does the same thing programmatically.
    msg = delta_array_pb2.DeltaMessage()
    msg.id = TX_ID
    msg.status.pose_req.SetInParent()
    payload = msg.SerializeToString()
    n = len(payload)
    crc = crc16_ccitt(payload)
    frame = (
        FRAME_START
        + bytes((n & 0xFF, (n >> 8) & 0xFF))
        + payload
        + bytes((crc & 0xFF, (crc >> 8) & 0xFF))
        + FRAME_END
    )
    EXPECTED_BYTES = 1 + 2 + n + 2 + 1
    print(f"[1/2] TX [{len(frame)} B]: {frame.hex()}")
    if len(frame) != EXPECTED_BYTES:
        print(f"      ERROR: expected {EXPECTED_BYTES} B, got {len(frame)} B")
        ser.close()
        return
    else:
        print("      frame looks good, sending...")
    ser.write(frame)

    # ---- step 3: read reply -------------------------------------------------
    # The firmware's handleStatus(pose_req) branch builds a DeltaMessage with
    # id=MY_ID and status.pose_resp.joint_pos populated from the current ADC
    # reads, encodes it with nanopb, and writes it wrapped in the same framing
    # we used above. Use ProtoTransport to parse it back: hunts for START,
    # reads by length, verifies CRC, returns the payload bytes (or None on any
    # framing/CRC failure).
    print(f"[2/2] reading reply  (expect: proto frame with a non-zero board id within {RX_TIMEOUT}s)")
    transport = ProtoTransport(ser)
    payload = transport.read_frame()
    ser.close()

    if payload is None:
        print("FAIL — no valid framed reply (timeout, bad length, CRC mismatch, or missing end byte)")
        return
    print(f"      RX payload [{len(payload)} B]: {payload.hex()}")

    reply = delta_array_pb2.DeltaMessage()
    try:
        reply.ParseFromString(payload)
    except Exception as exc:
        print(f"FAIL — proto decode error: {exc}")
        return

    has_pose_resp = reply.HasField("status") and reply.status.HasField("pose_resp")
    joint_pos = list(reply.status.pose_resp.joint_pos) if has_pose_resp else []
    print(f"      decoded: id={reply.id}, joint_pos={joint_pos}")

    # ---- verdict ------------------------------------------------------------
    # The board stamps its real chip-derived id (always non-zero) on the reply,
    # so a healthy response has a non-zero id and a pose_resp.
    if reply.id != 0 and has_pose_resp:
        print(f"PASS — host<->firmware comms verified end to end (board id={reply.id})")
    else:
        print(f"FAIL — expected a non-zero id with status.pose_resp, got id={reply.id}, pose_resp={has_pose_resp}")
        print("       id=0            -> board did not stamp its id (firmware response builder broken?)")
        print("       no pose_resp    -> firmware took wrong handler branch")
        print("       empty/garbage   -> firmware not transmitting (wrong Serial port? stuck in setup()?)")


if __name__ == "__main__":
    main()
