import numpy as np

from . import delta_array_pb2
from .constants import (
    MAX_JOINT_POS,
    MAX_TRAJECTORY_ROWS,
    MIN_JOINT_POS,
    NUM_MOTORS,
)


class CommandError(RuntimeError):
    """Raised when a command frame is rejected by the firmware or no ACK arrives.

    The firmware sends a CommandAck for every JointFrame command. If the ACK
    reports anything other than ACK_OK, or if no ACK arrives within the
    transport's read timeout, the host raises this exception rather than
    silently continuing — silent command loss was the failure mode this
    protocol change was designed to eliminate.
    """


class DeltaArrayAgent:
    def __init__(self, transport, robot_id):
        self.transport = transport
        self.robot_id = robot_id
        self.done_moving = False
        self.current_joint_positions = [0.05] * NUM_MOTORS

    def _envelope(self):
        return delta_array_pb2.DeltaMessage(id=self.robot_id)

    def _send(self, msg):
        self.transport.send(msg.SerializeToString())
        payload = self.transport.read_frame()
        if payload is None:
            return None
        reply = delta_array_pb2.DeltaMessage()
        try:
            reply.ParseFromString(payload)
        except Exception:
            return None
        if reply.id != self.robot_id:
            return None
        return reply

    def _send_command(self, msg):
        # JointFrame commands always get an acceptance ACK. Raise if missing
        # or non-OK so callers don't proceed with the next motion assuming
        # this one was queued when it wasn't.
        reply = self._send(msg)
        if reply is None:
            raise CommandError(
                f"no ACK from board {self.robot_id} (timeout / framing / CRC fail)"
            )
        if not reply.HasField("ack"):
            raise CommandError(
                f"board {self.robot_id} returned {reply.WhichOneof('payload')!r}, "
                f"expected ack"
            )
        status = reply.ack.status
        if status != delta_array_pb2.ACK_OK:
            name = delta_array_pb2.AckStatus.Name(status)
            raise CommandError(f"board {self.robot_id} rejected command: {name}")

    def move_joint_position(self, desired_joint_positions):
        pos = np.atleast_2d(np.asarray(desired_joint_positions, dtype=float))
        assert pos.ndim == 2 and pos.shape[1] == NUM_MOTORS, (
            f"expected shape (12,) or (N, 12), got {pos.shape}"
        )
        assert 1 <= pos.shape[0] <= MAX_TRAJECTORY_ROWS, (
            f"trajectory length {pos.shape[0]} outside [1, {MAX_TRAJECTORY_ROWS}]"
        )
        pos = np.clip(pos, MIN_JOINT_POS, MAX_JOINT_POS)
        flat = pos.reshape(-1).tolist()
        msg = self._envelope()
        if pos.shape[0] == 1:
            msg.joint.move.joint_pos.extend(flat)
        else:
            msg.joint.traj.joint_pos.extend(flat)
        self._send_command(msg)

    def get_joint_positions(self):
        msg = self._envelope()
        msg.status.pose_req.SetInParent()
        reply = self._send(msg)
        if (
            reply is not None
            and reply.HasField("status")
            and reply.status.HasField("pose_resp")
        ):
            self.current_joint_positions = list(reply.status.pose_resp.joint_pos)
        return self.current_joint_positions

    def reset(self):
        msg = self._envelope()
        msg.joint.reset.SetInParent()
        self._send_command(msg)

    def stop(self):
        msg = self._envelope()
        msg.joint.stop.SetInParent()
        self._send_command(msg)

    def close(self):
        self.transport.close()
