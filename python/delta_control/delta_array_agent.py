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
        self.min_joint_pos = MIN_JOINT_POS
        self.max_joint_pos = MAX_JOINT_POS
        self.num_motors = NUM_MOTORS
        self.max_trajectory_rows = MAX_TRAJECTORY_ROWS

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
        self.current_joint_positions = pos[-1].tolist()

    def move_delta(self, delta_index, motor_positions):
        assert 0 <= delta_index <= 3, f"delta_index must be 0-3, got {delta_index}"
        assert len(motor_positions) == 3, f"expected 3 motor positions, got {len(motor_positions)}"
        pos = list(self.current_joint_positions)
        start = delta_index * 3
        pos[start:start + 3] = motor_positions
        self.move_joint_position(pos)

    def whoami(self):
        # Ask this (already-addressed) board to report its chip-derived id, e.g.
        # to confirm/log which board answers on this port. For discovery when the
        # id is unknown, use delta_array_env.discover_board_id (broadcast) instead
        # — _send here filters replies to self.robot_id.
        msg = self._envelope()
        msg.status.id_req.SetInParent()
        reply = self._send(msg)
        if (
            reply is not None
            and reply.HasField("status")
            and reply.status.HasField("id_resp")
        ):
            return reply.status.id_resp.id
        return None

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

    def set_motor_pwm(self, motor_index, pwm, duration_ms=0):
        # Diagnostics: drive one motor open-loop at a fixed PWM, bypassing the
        # PID. The firmware releases all other motors first and auto-releases
        # this one after duration_ms (0 -> firmware default, clamped to the move
        # timeout) so a lost host can't leave a motor driven. Sign of pwm selects
        # direction; magnitude is clamped to 255 on the firmware too.
        assert 0 <= motor_index < NUM_MOTORS, (
            f"motor_index must be 0..{NUM_MOTORS - 1}, got {motor_index}"
        )
        assert -255 <= pwm <= 255, f"pwm must be -255..255, got {pwm}"
        msg = self._envelope()
        msg.joint.set_pwm.motor_index = int(motor_index)
        msg.joint.set_pwm.pwm = int(pwm)
        msg.joint.set_pwm.duration_ms = int(duration_ms)
        self._send_command(msg)

    def set_config(self, motor_index=None, *, deadband=None,
                   bias_fwd=None, bias_back=None, brake_at_setpoint=None):
        # Runtime per-motor tuning. Overwrites the board's compiled defaults for
        # one motor (motor_index) or all of them (motor_index=None) without
        # reflashing. Only the fields passed are applied on the firmware (proto3
        # optional / has_*); omitted fields keep their current value. Used to push
        # a board's calibration (see calibration.apply_calibration) at startup and
        # for live tuning. Follows the ACK-or-raise contract of _send_command.
        #
        # brake_at_setpoint is board-global (it ignores motor_index): when True a
        # settled joint short-brakes instead of coasting, killing the post-release
        # overshoot. Send it once (no motor_index needed) to flip the A/B mode.
        if motor_index is not None:
            assert 0 <= motor_index < NUM_MOTORS, (
                f"motor_index must be 0..{NUM_MOTORS - 1}, got {motor_index}"
            )
        if deadband is not None:
            assert deadband >= 0, f"deadband must be >= 0, got {deadband}"
        if bias_fwd is not None:
            assert 0 <= bias_fwd <= 255, f"bias_fwd must be 0..255, got {bias_fwd}"
        if bias_back is not None:
            assert 0 <= bias_back <= 255, f"bias_back must be 0..255, got {bias_back}"
        if (deadband is None and bias_fwd is None and bias_back is None
                and brake_at_setpoint is None):
            raise ValueError("set_config needs at least one of "
                             "deadband/bias_fwd/bias_back/brake_at_setpoint")
        msg = self._envelope()
        cfg = msg.joint.set_config
        if motor_index is not None:
            cfg.motor_index = int(motor_index)
        if deadband is not None:
            cfg.deadband = float(deadband)
        if bias_fwd is not None:
            cfg.bias_fwd = int(bias_fwd)
        if bias_back is not None:
            cfg.bias_back = int(bias_back)
        if brake_at_setpoint is not None:
            cfg.brake_at_setpoint = bool(brake_at_setpoint)
        self._send_command(msg)

    def get_telemetry(self):
        # Per-motor diagnostics: position (m), PID error (m), and last applied
        # signed PWM (-255..255, 0 = released). error/pwm reflect the most recent
        # control step, so they are most meaningful during an active move.
        # Returns a dict of 12-lists, or None if no valid reply arrived.
        msg = self._envelope()
        msg.status.telemetry_req.SetInParent()
        reply = self._send(msg)
        if (
            reply is not None
            and reply.HasField("status")
            and reply.status.HasField("telemetry_resp")
        ):
            t = reply.status.telemetry_resp
            return {
                "position": list(t.position),
                "error": list(t.error),
                "pwm": list(t.pwm),
            }
        return None

    def get_config(self):
        # Read back the board's LIVE tuning: per-motor deadband (m) and static
        # feedforward bias (bias_fwd/bias_back, PWM counts), plus the board-global
        # brake_at_setpoint mode. set_config is otherwise write-only, so this is
        # the only way to verify what a board is actually running. Config is
        # RAM-only on the firmware -- a board reverts to its compiled defaults
        # after a power-cycle/reboot until apply_calibration is re-run, and this
        # readback is what surfaces that drift. Returns a dict of the four fields
        # (three 12-lists + a bool), or None if no valid reply arrived.
        msg = self._envelope()
        msg.status.config_req.SetInParent()
        reply = self._send(msg)
        if (
            reply is not None
            and reply.HasField("status")
            and reply.status.HasField("config_resp")
        ):
            c = reply.status.config_resp
            return {
                "deadband": list(c.deadband),
                "bias_fwd": list(c.bias_fwd),
                "bias_back": list(c.bias_back),
                "brake_at_setpoint": bool(c.brake_at_setpoint),
            }
        return None

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
