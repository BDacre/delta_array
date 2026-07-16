import glob
import time
from math import pi

import numpy as np
from serial import Serial

from . import delta_array_pb2
from .delta_array_agent import DeltaArrayAgent
from .constants import (
    ACK_TIMEOUT_S,
    BOARD_PORT_GLOB,
    BOARD_REGISTRY,
    BROADCAST_ID,
    DEFAULT_BAUD,
    DISCOVERY_TIMEOUT_S,
    HOME_POSITION,
    LEG_LENGTH,
    SIDE_LENGTH_BASE,
    SIDE_LENGTH_PLATFORM,
    NUM_MOTORS,
    MIN_JOINT_POS,
    MAX_JOINT_POS,

)
from .get_coords import RoboCoords
from .prismatic_delta import PrismaticDelta
from .transport import ProtoTransport

delta = PrismaticDelta(SIDE_LENGTH_PLATFORM, SIDE_LENGTH_BASE, LEG_LENGTH)
rc = RoboCoords()


class DiscoveryError(RuntimeError):
    """Raised when no board answers a broadcast whoami on a port.

    Firmware derives each board's id from its chip serial, so the host no longer
    knows the id up front. It discovers it by sending an id_req to BROADCAST_ID
    and reading back the board's real id. If nothing answers (nothing connected,
    wrong port, framing/CRC failure), we raise rather than guess an id.
    """


def discover_board_id(transport, *, attempts=3):
    """Ask the board on this transport for its chip-derived id.

    Sends a whoami (status.id_req) addressed to BROADCAST_ID so the board answers
    even though we don't yet know its real id, and returns the id from its
    id_resp. Bypasses the per-agent id filter (which needs the id we're trying to
    learn). Returns None if no valid id_resp arrives within `attempts` tries.

    Assumes a single board on the link — on a shared multi-board bus, a broadcast
    request makes every board answer at once and the replies collide.
    """
    req = delta_array_pb2.DeltaMessage(id=BROADCAST_ID)
    req.status.id_req.SetInParent()
    encoded = req.SerializeToString()
    for _ in range(attempts):
        transport.send(encoded)
        payload = transport.read_frame()
        if payload is None:
            continue
        reply = delta_array_pb2.DeltaMessage()
        try:
            reply.ParseFromString(payload)
        except Exception:
            continue
        if reply.HasField("status") and reply.status.HasField("id_resp"):
            return reply.status.id_resp.id
    return None


class DeltaArrayEnv:
    def __init__(self, port, *, active_ids=None, baud=DEFAULT_BAUD):
        # timeout bounds how long read_frame() waits before giving up; sized to
        # tolerate a worst-case in-flight move on the firmware before its ACK.
        self.ser = Serial(port, baud, timeout=ACK_TIMEOUT_S)
        self.transport = ProtoTransport(self.ser)
        # active_ids=None (default) auto-discovers the single connected board's
        # chip-derived id via broadcast whoami. Pass explicit ids to skip
        # discovery (e.g. a known id from BOARD_REGISTRY, or a shared-bus setup).
        if active_ids is None:
            discovered = discover_board_id(self.transport)
            if discovered is None:
                self.close()
                raise DiscoveryError(
                    f"no board answered whoami on {port} "
                    f"(check connection / port / baud)"
                )
            active_ids = (discovered,)
        self.active_ids = tuple(active_ids)
        self.agents = {i: DeltaArrayAgent(self.transport, i) for i in self.active_ids}
        self.done_states = np.array([0] * 12)
        self.lowz = 0.08
        self.highz = 0.11

    @property
    def agent(self):
        # Convenience for the common single-board case: the one discovered/active
        # agent. Raises if the env holds more than one board.
        if len(self.active_ids) != 1:
            raise ValueError(
                f"env has {len(self.active_ids)} boards; index self.agents[id] instead"
            )
        return self.agents[self.active_ids[0]]

    def reset(self):
        pt = np.array(delta.ik(HOME_POSITION))
        jts = np.tile(pt, 4).tolist()

        for i in self.active_ids:
            self.agents[i].move_joint_position(jts)

    def close(self):
        # Release the serial port; safe to call even if the transport is
        # already closed or was never fully opened.
        transport = getattr(self, "transport", None)
        if transport is not None:
            transport.close()

    # def move_over_trajectory(self, traj="vertical"):
    #     if traj == "circle":
    #         thetas = np.linspace(0, 2 * np.pi, 10)
    #         r = 0.01
    #         for theta in thetas:
    #             ee_pts = [r * np.cos(theta), r * np.sin(theta), 0.10]
    #             pts = np.array(Delta.IK(ee_pts))
    #             jts = []
    #             for _ in range(0, 4):
    #                 for j in range(3):
    #                     jts.append(pts[j])

    #             for i in self.active_ids:
    #                 self.agents[i].move_joint_position(jts)

    #     elif traj == "vertical":
    #         pts1 = [0.05, 0.05, 0.05]
    #         pts2 = [0.02, 0.02, 0.02]
    #         jts = []
    #         for i in range(0, 4):
    #             if i == 3:
    #                 jts.extend(pts1)
    #             else:
    #                 jts.extend(pts2)

    #         for i in self.active_ids:
    #             self.agents[i].move_joint_position(jts)

    #         for i in self.active_ids:
    #             self.agents[i].get_joint_positions()

    # def move_delta_array(
    #     self,
    #     point=(0.162378, 0.13125),
    #     pattern="converge",
    #     angle=pi / 4,
    #     wall=(),
    #     zmax=None,
    #     zmin=None,
    # ):
    #     if pattern == "up":
    #         vecs = RC.get_dist_vec((0.162378, 1e16))
    #     elif pattern == "down":
    #         vecs = RC.get_dist_vec((0.162378, 1e16))
    #         vecs = RC.rotate(vecs, np.pi)
    #         vecs = RC.normalize_vec(vecs)
    #     elif pattern == "left":
    #         vecs = RC.get_dist_vec((0.162378, 1e16))
    #         vecs = RC.rotate(vecs, -np.pi / 2)
    #         vecs = RC.normalize_vec(vecs)
    #     elif pattern == "right":
    #         vecs = RC.get_dist_vec((1e16, 0.13125))
    #     elif pattern == "converge":
    #         vecs = RC.get_dist_vec(point)
    #     elif pattern == "rotate":
    #         vecs = RC.get_rot_vec(point, angle)
    #     else:
    #         raise ValueError(f"Unknown pattern: {pattern!r}")

    #     RC.set_pattern(vecs, zmax, zmin)
    #     if len(wall) > 0:
    #         RC.set_wall(wall)
    #     a = 0
    #     while True:
    #         for i in self.active_ids:
    #             pts = RC.get_pattern(i, a)
    #             self.agents[i].move_joint_position(pts)
    #         a = 1 - a
    #         time.sleep(1.5)

    # def grip_and_move_objects(self):
    #     RC.grip_pattern()
    #     a = 0
    #     while True:
    #         for i in self.active_ids:
    #             pts = RC.get_pattern(i, a)
    #             self.agents[i].move_joint_position(pts)
    #         a = 1 - a
    #         time.sleep(0.5)

    # def glamorous(self, wall, w_ht):
    #     RC.set_wall(wall, w_ht)
    #     a = 0
    #     while True:
    #         for i in self.active_ids:
    #             pts = RC.get_pattern(i, a)
    #             self.agents[i].move_joint_position(pts)
    #         a = 1 - a
    #         time.sleep(4)


def find_board_ports(pattern=BOARD_PORT_GLOB, *, baud=DEFAULT_BAUD):
    """Scan serial ports and return [(port, board_id), ...] for those that answer
    a broadcast whoami.

    Lets the host locate the board's port automatically even when ttyACM
    enumeration order changes or another CDC device shares the /dev/ttyACM*
    namespace. Only sends a whoami (no motion), so probing every port is safe.
    Uses a short read timeout, and read_frame() bails on a chatty non-protocol
    port, so a dead/wrong port is skipped quickly.
    """
    found = []
    for port in sorted(glob.glob(pattern)):
        try:
            ser = Serial(port, baud, timeout=DISCOVERY_TIMEOUT_S)
        except Exception:
            continue
        try:
            board_id = discover_board_id(ProtoTransport(ser), attempts=2)
        except Exception:
            board_id = None
        finally:
            ser.close()
        if board_id is not None:
            found.append((port, board_id))
    return found


def _resolve_board_id(id_or_label):
    """Map an id-or-label to a concrete board id (int), or None to auto-discover."""
    if id_or_label is None:
        return None
    if isinstance(id_or_label, str) and not id_or_label.lstrip("-").isdigit():
        try:
            return BOARD_REGISTRY[id_or_label]
        except KeyError:
            raise KeyError(
                f"unknown board label {id_or_label!r}; "
                f"known labels: {sorted(BOARD_REGISTRY)}"
            )
    return int(id_or_label)


def open_board(port=None, id_or_label=None, *, baud=DEFAULT_BAUD):
    """Open a single-board env and return (env, agent).

    port selects the serial port:
      * None -> auto-detect by scanning BOARD_PORT_GLOB for a board that answers
        a whoami. If id_or_label names a specific board, the matching port is
        chosen; otherwise exactly one board must be present.
      * a path (e.g. "/dev/ttyACM1") -> use that port directly.

    id_or_label selects which board:
      * None -> auto-discover the connected board's chip-derived id (default).
      * int (or numeric str) -> address that raw id, skipping discovery.
      * str -> look the label up in BOARD_REGISTRY.
    """
    board_id = _resolve_board_id(id_or_label)

    if port is None:
        found = find_board_ports(baud=baud)
        if not found:
            raise DiscoveryError(
                f"no delta board answered whoami on any {BOARD_PORT_GLOB} port "
                f"(check power / USB / that the board is flashed)"
            )
        if board_id is not None:
            matches = [p for p, bid in found if bid == board_id]
            if not matches:
                raise DiscoveryError(
                    f"board id {board_id} not found; discovered: {found}"
                )
            port = matches[0]
        elif len(found) == 1:
            port, board_id = found[0]
        else:
            raise DiscoveryError(
                f"multiple boards found: {found}; pass an explicit port or id"
            )

    if board_id is None:
        env = DeltaArrayEnv(port, baud=baud)
    else:
        env = DeltaArrayEnv(port, active_ids=(board_id,), baud=baud)
    return env, env.agent
