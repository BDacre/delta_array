from .constants import FRAME_START, FRAME_END


class ProtoTransport:
    def __init__(self, ser):
        self.ser = ser

    def send(self, payload: bytes) -> None:
        self.ser.write(FRAME_START + payload + FRAME_END)

    def readline(self) -> bytes:
        return self.ser.readline()

    def close(self) -> None:
        self.ser.close()
