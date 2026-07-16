from .constants import FRAME_END, FRAME_START, MAX_HUNT_BYTES


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


class ProtoTransport:
    # Wire format: [START][LEN_LO][LEN_HI][payload...][CRC_LO][CRC_HI][END]
    # LEN is uint16 little-endian (payload length only). CRC is CRC-16/CCITT-FALSE
    # over the payload bytes, little-endian. Encoded protobuf can legitimately
    # contain the start/end byte values, so we read by length and validate with
    # CRC; the trailing END byte is a final sanity gate, not a delimiter.

    def __init__(self, ser):
        self.ser = ser

    def send(self, payload: bytes) -> None:
        n = len(payload)
        crc = crc16_ccitt(payload)
        frame = (
            FRAME_START
            + bytes((n & 0xFF, (n >> 8) & 0xFF))
            + payload
            + bytes((crc & 0xFF, (crc >> 8) & 0xFF))
            + FRAME_END
        )
        self.ser.write(frame)

    def read_frame(self) -> bytes | None:
        # Hunt for the start byte, then read by length and verify CRC. Each
        # underlying read() returns short on serial timeout, which we treat
        # as a dropped frame and report as None.
        #
        # Bound the hunt: a wrong/chatty port (e.g. another CDC device streaming
        # bytes that are never our start marker) would otherwise loop forever,
        # because read() keeps returning data and never times out. After
        # MAX_HUNT_BYTES discarded non-start bytes we give up and report None so
        # the caller fails cleanly instead of hanging silently.
        discarded = 0
        while True:
            b = self.ser.read(1)
            if not b:
                return None
            if b == FRAME_START:
                break
            discarded += 1
            if discarded > MAX_HUNT_BYTES:
                return None

        header = self.ser.read(2)
        if len(header) != 2:
            return None
        n = header[0] | (header[1] << 8)
        if n == 0:
            return None

        payload = self.ser.read(n)
        if len(payload) != n:
            return None

        trailer = self.ser.read(3)
        if len(trailer) != 3:
            return None
        crc_rx = trailer[0] | (trailer[1] << 8)
        if trailer[2:3] != FRAME_END:
            return None
        if crc16_ccitt(payload) != crc_rx:
            return None
        return payload

    def readline(self) -> bytes:
        return self.ser.readline()

    def close(self) -> None:
        self.ser.close()
