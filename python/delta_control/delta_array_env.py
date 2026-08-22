import glob
import time
import warnings
from math import pi

import numpy as np
from serial import Serial
from serial.tools import list_ports

from . import delta_array_pb2
from .delta_array_agent import DeltaArrayAgent
from .boards import BOARD_LABELS, BOARD_REGISTRY
from .constants import (
    ACK_TIMEOUT_S,
    BOARD_PORT_GLOB,
    BOARD_USB_IDS,
    BROADCAST_ID,
    DEFAULT_BAUD,
    DISCOVERY_TIMEOUT_S,
    HOME_POSITION,
    LEG_LENGTH,
    BASE_TRIANGLE_SIDE_LEN,
    PLATFORM_TRIANGLE_SIDE_LEN,
    EE_Z_OFFSET,
    NUM_MOTORS,
    MIN_JOINT_POS,
    MAX_JOINT_POS,

)
from .get_coords import RoboCoords
from .prismatic_delta import PrismaticDelta
from .transport import ProtoTransport
from .calibration import (
    BoardCalibResult,
    apply_calibration,
    calibration_path,
    diff_calibration,
    load_calibration,
)

delta = PrismaticDelta(PLATFORM_TRIANGLE_SIDE_LEN, BASE_TRIANGLE_SIDE_LEN, LEG_LENGTH, EE_Z_OFFSET)
rc = RoboCoords()


def list_board_ports():
    """Serial ports that look like delta boards, by USB VID:PID.

    Delta boards are Adafruit Feather M0s (BOARD_USB_IDS). Filtering on the USB
    identity means unrelated CDC devices (e.g. an Arduino Uno on another
    /dev/ttyACM*) are skipped WITHOUT being opened — no wasted seconds probing a
    chatty non-board port. Falls back to BOARD_PORT_GLOB only if no VID:PID
    metadata is available (unusual on Linux USB CDC).
    """
    matched = sorted(
        p.device for p in list_ports.comports() if (p.vid, p.pid) in BOARD_USB_IDS
    )
    if matched:
        return matched
    return sorted(glob.glob(BOARD_PORT_GLOB))


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
    """Controls one or more delta boards, one board per serial port.

    Each board lives on its own /dev/ttyACM* port. The env opens a serial
    connection per board, learns each board's chip-derived id via a broadcast
    whoami, and keys agents by that id. `agents[id]` (or `agent_for(label)` via
    BOARD_REGISTRY) addresses one board; iterate `active_ids` to command them all.

    ports:
      * None (default) -> scan BOARD_PORT_GLOB and open every board found.
      * a single port path, or an iterable of port paths -> open just those.
    ids:
      * None (default) -> keep every board discovered.
      * an iterable of ids and/or BOARD_REGISTRY labels -> keep only those; a
        requested id that never answers raises DiscoveryError.
    verify_calibration:
      * False (default) -> connect only; do not touch board config. Keeps
        low-level uses (diagnostics, characterization, --no-calibration tests)
        unaffected.
      * True -> after discovery, read each board's live config back and warn
        (does NOT write) if any board is not running its calibration JSON, so a
        board silently on firmware defaults is caught at connect. Call
        env.provision() to fix.
    """

    def __init__(self, ports=None, *, ids=None, baud=DEFAULT_BAUD,
                 verify_calibration=False):
        wanted = _resolve_id_filter(ids)

        if ports is None:
            candidates = list_board_ports()
        elif isinstance(ports, str):
            candidates = [ports]
        else:
            candidates = list(ports)

        # board_id -> (port, ProtoTransport). One serial connection per board.
        self._conns = {}
        self.agents = {}
        try:
            for port in candidates:
                try:
                    # Open with the short discovery timeout so a wrong/dead port
                    # is skipped quickly; bumped to ACK_TIMEOUT_S once identified.
                    ser = Serial(port, baud, timeout=DISCOVERY_TIMEOUT_S)
                except Exception:
                    continue  # port busy or vanished — skip it
                transport = ProtoTransport(ser)
                board_id = discover_board_id(transport)
                if board_id is None:
                    ser.close()
                    continue  # no delta board answered on this port
                if wanted is not None and board_id not in wanted:
                    ser.close()
                    continue
                if board_id in self.agents:
                    ser.close()
                    continue  # id already claimed (same board seen on two ports)
                # Reads must now tolerate a worst-case in-flight move before the ACK.
                ser.timeout = ACK_TIMEOUT_S
                self._conns[board_id] = (port, transport)
                self.agents[board_id] = DeltaArrayAgent(transport, board_id)
        except BaseException:
            self.close()
            raise

        where = candidates or "any connected Feather M0 port"
        if wanted is not None:
            missing = wanted - set(self.agents)
            if missing:
                self.close()
                raise DiscoveryError(f"boards {sorted(missing)} not found on {where}")
        if not self.agents:
            self.close()
            raise DiscoveryError(f"no delta boards found on {where}")

        self.active_ids = tuple(sorted(self.agents))
        self.done_states = np.array([0] * 12)
        self.lowz = 0.08
        self.highz = 0.11

        if verify_calibration:
            self._warn_on_calibration_mismatch()

    @property
    def agent(self):
        # Convenience for the single-board case: the one active agent. Raises if
        # the env holds more than one board — index agents[id] / agent_for() then.
        if len(self.active_ids) != 1:
            raise ValueError(
                f"env holds {len(self.active_ids)} boards {self.active_ids}; "
                f"index agents[id] or use agent_for(id_or_label) instead"
            )
        return self.agents[self.active_ids[0]]

    def agent_for(self, id_or_label):
        # Look up an agent by raw id or by BOARD_REGISTRY label.
        return self.agents[_resolve_board_id(id_or_label)]

    def port_for(self, id_or_label):
        # The serial port a given board id / label was found on.
        return self._conns[_resolve_board_id(id_or_label)][0]

    def reset(self):
        pt = np.array(delta.ik(HOME_POSITION))
        jts = np.tile(pt, 4).tolist()

        for i in self.active_ids:
            self.agents[i].move_joint_position(jts)

    def _provision_board(self, board_id, *, apply, calib_dir):
        """Apply (optional) then read back and diff one board's calibration."""
        calib = load_calibration(board_id, calib_dir)
        if calib is None:
            return BoardCalibResult(
                board_id, applied=False,
                error=f"no calibration file (expected {calibration_path(board_id, calib_dir)})")
        cfg_id = calib.get("board_id")
        if cfg_id is not None and cfg_id != board_id:
            return BoardCalibResult(
                board_id, applied=False,
                error=f"calibration board_id {cfg_id} != connected board {board_id}")

        agent = self.agents[board_id]
        if apply:
            apply_calibration(agent, calib)
        live = agent.get_config()
        if live is None:
            return BoardCalibResult(
                board_id, applied=apply,
                error="board returned no config_resp (firmware predates ConfigRequest; reflash)")
        return BoardCalibResult(
            board_id, applied=apply,
            mismatches=diff_calibration(live, calib),
            brake_at_setpoint=live["brake_at_setpoint"])

    def provision(self, *, calib_dir=None):
        """Push each connected board's calibration (by chip id) and verify it.

        For every active board: load its calibration JSON, apply it over serial,
        then read the live config back (ConfigRequest) and diff it against the
        JSON. Config-only -- no motion. Returns ``{board_id: BoardCalibResult}``;
        a board with no calibration file, a readback mismatch, or old firmware is
        reported via ``result.ok is False`` rather than raising, so one bad board
        never aborts the rest. This is the same call path the provision_array CLI
        uses, so a library consumer runs identical logic:

            env = DeltaArrayEnv()
            report = env.provision()
            assert all(r.ok for r in report.values()), \\
                [r for r in report.values() if not r.ok]
        """
        return {b: self._provision_board(b, apply=True, calib_dir=calib_dir)
                for b in self.active_ids}

    def check_calibration(self, *, calib_dir=None):
        """Read back and diff every connected board WITHOUT writing anything.

        A fast, side-effect-free gate: confirm the array is already running its
        calibration before trusting a session. Returns ``{board_id:
        BoardCalibResult}``; ``result.ok is False`` on any mismatch (e.g. a board
        that reverted to firmware defaults after a power-cycle).
        """
        return {b: self._provision_board(b, apply=False, calib_dir=calib_dir)
                for b in self.active_ids}

    def _warn_on_calibration_mismatch(self):
        """Read back every board and warn (no writes) if any isn't calibrated.

        Backs the DeltaArrayEnv(verify_calibration=True) opt-in. Emits a single
        warning summarising the offending boards; call env.provision() to fix.
        """
        bad = [r for r in self.check_calibration().values() if not r.ok]
        if not bad:
            return
        lines = []
        for r in bad:
            label = BOARD_LABELS.get(r.board_id)
            name = f"{r.board_id} ({label})" if label else str(r.board_id)
            reason = r.error or f"{len(r.mismatches)} config mismatch(es)"
            lines.append(f"  board {name}: {reason}")
        warnings.warn(
            f"{len(bad)} board(s) not running their calibration "
            f"(call env.provision() to fix):\n" + "\n".join(lines),
            stacklevel=3,
        )

    def close(self):
        # Release every serial port. Safe to call repeatedly, or on a partially
        # constructed env (e.g. from an error mid-__init__).
        for _port, transport in getattr(self, "_conns", {}).values():
            try:
                transport.close()
            except Exception:
                pass
        self._conns = {}

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


def find_board_ports(*, baud=DEFAULT_BAUD):
    """Scan Feather M0 ports and return [(port, board_id), ...] for those that
    answer a broadcast whoami.

    Only considers ports matching a delta board's USB VID:PID (see
    list_board_ports), so unrelated CDC devices are never opened. Sends only a
    whoami (no motion), so probing is safe. Uses a short read timeout.
    """
    found = []
    for port in list_board_ports():
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


def _resolve_id_filter(ids):
    """Map an iterable of ids/labels to a set of concrete ids, or None for all."""
    if ids is None:
        return None
    return {_resolve_board_id(x) for x in ids}


def open_board(port=None, id_or_label=None, *, baud=DEFAULT_BAUD,
               verify_calibration=False):
    """Open a single board and return (env, agent).

    Thin single-board convenience over DeltaArrayEnv.
      * port None -> scan BOARD_PORT_GLOB; a path (or list) -> open just that.
      * id_or_label None -> the one board found; an id or BOARD_REGISTRY label ->
        that specific board.
      * verify_calibration True -> warn (no writes) if the board isn't running
        its calibration JSON (see DeltaArrayEnv).
    Raises DiscoveryError unless the selection resolves to exactly one board.
    """
    ids = None if id_or_label is None else [id_or_label]
    env = DeltaArrayEnv(ports=port, ids=ids, baud=baud,
                        verify_calibration=verify_calibration)
    if len(env.active_ids) != 1:
        found = env.active_ids
        env.close()
        raise DiscoveryError(
            f"expected exactly one board, found {found}; "
            f"pass a port or id_or_label to disambiguate"
        )
    return env, env.agent
