import numpy as np

from . import delta_array_pb2
from .constants import (
    MAX_JOINT_POS,
    MAX_TRAJECTORY_ROWS,
    MIN_JOINT_POS,
    NUM_MOTORS,
)


class DeltaArrayAgent:
    def __init__(self, transport, robot_id):
        self.transport = transport
        self.robot_id = robot_id
        self.done_moving = False
        self.current_joint_positions = [0.05] * NUM_MOTORS

    def _new_message(self):
        msg = delta_array_pb2.DeltaMessage()
        msg.id = self.robot_id
        msg.request_joint_pose = False
        msg.request_done_state = False
        msg.reset = False
        return msg

    def _send_proto(self, msg, ret_expected=False):
        self.transport.send(msg.SerializeToString())
        if not ret_expected:
            return None
        line = str(self.transport.readline()).strip().split(" ")
        if msg.id == int(line[0].split(":")[-1]):
            return [float(x) for x in line[1:-1]]
        print("ERROR, incorrect robot ID requested.")
        return [0.05] * NUM_MOTORS

    def move_joint_position(self, desired_joint_positions):
        pos = np.atleast_2d(np.asarray(desired_joint_positions, dtype=float))
        assert pos.ndim == 2 and pos.shape[1] == NUM_MOTORS, (
            f"expected shape (12,) or (N, 12), got {pos.shape}"
        )
        assert 1 <= pos.shape[0] <= MAX_TRAJECTORY_ROWS, (
            f"trajectory length {pos.shape[0]} outside [1, {MAX_TRAJECTORY_ROWS}]"
        )
        pos = np.clip(pos, MIN_JOINT_POS, MAX_JOINT_POS)
        msg = self._new_message()
        msg.joint_pos.extend(pos.reshape(-1).tolist())
        self._send_proto(msg)

    def get_joint_positions(self):
        msg = self._new_message()
        msg.request_done_state = True
        msg.joint_pos.extend([0.5] * NUM_MOTORS)
        result = self._send_proto(msg, ret_expected=True)
        if result is not None:
            self.current_joint_positions = result
        return self.current_joint_positions

    def reset(self):
        msg = self._new_message()
        msg.reset = True
        self._send_proto(msg)

    def stop(self):
        msg = self._new_message()
        msg.reset = True
        self._send_proto(msg)

    def close(self):
        self.transport.close()
