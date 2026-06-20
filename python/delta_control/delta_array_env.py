import time
from math import pi

import numpy as np
from serial import Serial

from .delta_array_agent import DeltaArrayAgent
from .constants import (
    ACK_TIMEOUT_S,
    DEFAULT_ACTIVE_AGENT_IDS,
    DEFAULT_BAUD,
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


class DeltaArrayEnv:
    def __init__(self, port, *, active_ids=DEFAULT_ACTIVE_AGENT_IDS, baud=DEFAULT_BAUD):
        # timeout bounds how long read_frame() waits before giving up; sized to
        # tolerate a worst-case in-flight move on the firmware before its ACK.
        self.ser = Serial(port, baud, timeout=ACK_TIMEOUT_S)
        self.transport = ProtoTransport(self.ser)
        self.active_ids = tuple(active_ids)
        self.agents = {i: DeltaArrayAgent(self.transport, i) for i in self.active_ids}
        self.done_states = np.array([0] * 12)
        self.lowz = 0.08
        self.highz = 0.11

    def reset(self):
        pt = np.array(delta.ik(HOME_POSITION))
        jts = np.tile(pt, 4).tolist()

        for i in self.active_ids:
            self.agents[i].move_joint_position(jts)

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
