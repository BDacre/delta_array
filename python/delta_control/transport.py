from .constants import FRAME_END, FRAME_START


class ProtoTransport:
    def __init__(self, ser):
        self.ser = ser

    def send(self, payload: bytes) -> None:
        self.ser.write(FRAME_START + payload + FRAME_END)

    def read_frame(self) -> bytes | None:
        raw = self.ser.read_until(FRAME_END)
        start = raw.rfind(FRAME_START)
        if start < 0 or not raw.endswith(FRAME_END):
            return None
        return raw[start + 1 : -1]

    def readline(self) -> bytes:
        return self.ser.readline()

    def close(self) -> None:
        self.ser.close()
