import time
from math import pi

import numpy as np
from serial import Serial

from .agent import DeltaArrayAgent
from .constants import (
    DEFAULT_ACTIVE_AGENT_IDS,
    DEFAULT_BAUD,
    LEG_LENGTH,
    SIDE_LENGTH_BASE,
    SIDE_LENGTH_PLATFORM,
)
from .get_coords import RoboCoords
from .Prismatic_Delta import Prismatic_Delta
from .transport import ProtoTransport

Delta = Prismatic_Delta(SIDE_LENGTH_PLATFORM, SIDE_LENGTH_BASE, LEG_LENGTH)
RC = RoboCoords()


class DeltaArrayEnv:
    def __init__(self, port, *, active_ids=DEFAULT_ACTIVE_AGENT_IDS, baud=DEFAULT_BAUD):
        self.ser = Serial(port, baud)
        self.transport = ProtoTransport(self.ser)
        self.active_ids = tuple(active_ids)
        self.agents = {i: DeltaArrayAgent(self.transport, i) for i in self.active_ids}
        self.done_states = np.array([0] * 12)
        self.lowz = 8
        self.highz = 11

    def reset(self):
        jts = []
        for _ in range(0, 12):
            pt = Delta.IK((0, 0, 8))
            pt = np.array(pt) * 0.01
            jts.extend(pt)

        for i in self.active_ids:
            self.agents[i].move_joint_position(jts)

    def move_over_trajectory(self, traj="vertical"):
        if traj == "circle":
            thetas = np.linspace(0, 2 * np.pi, 10)
            r = 1
            for theta in thetas:
                ee_pts = [r * np.cos(theta), r * np.sin(theta), 10.0]
                pts = Delta.IK(ee_pts)
                pts = np.array(pts) * 0.01
                jts = []
                for _ in range(0, 4):
                    for j in range(3):
                        jts.append(pts[j])

                for i in self.active_ids:
                    self.agents[i].move_joint_position(jts)

        elif traj == "vertical":
            pts1 = [0.05, 0.05, 0.05]
            pts2 = [0.02, 0.02, 0.02]
            jts = []
            for i in range(0, 4):
                if i == 3:
                    jts.extend(pts1)
                else:
                    jts.extend(pts2)

            for i in self.active_ids:
                self.agents[i].move_joint_position(jts)

            for i in self.active_ids:
                self.agents[i].get_joint_positions()

    def move_delta_array(
        self,
        point=(162.378, 131.25),
        pattern="converge",
        angle=pi / 4,
        wall=(),
        zmax=None,
        zmin=None,
    ):
        if pattern == "up":
            vecs = RC.get_dist_vec((162.378, 9999999999999999))
        elif pattern == "down":
            vecs = RC.get_dist_vec((162.378, 9999999999999999))
            vecs = RC.rotate(vecs, np.pi)
            vecs = RC.normalize_vec(vecs)
        elif pattern == "left":
            vecs = RC.get_dist_vec((162.378, 9999999999999999))
            vecs = RC.rotate(vecs, -np.pi / 2)
            vecs = RC.normalize_vec(vecs)
        elif pattern == "right":
            vecs = RC.get_dist_vec((9999999999999999, 131.25))
        elif pattern == "converge":
            vecs = RC.get_dist_vec(point)
        elif pattern == "rotate":
            vecs = RC.get_rot_vec(point, angle)
        else:
            raise ValueError(f"Unknown pattern: {pattern!r}")

        RC.set_pattern(vecs, zmax, zmin)
        if len(wall) > 0:
            RC.set_wall(wall)
        a = 0
        while True:
            for i in self.active_ids:
                pts = RC.get_pattern(i, a)
                self.agents[i].move_joint_position(pts)
            a = 1 - a
            time.sleep(1.5)

    def grip_and_move_objects(self):
        RC.grip_pattern()
        a = 0
        while True:
            for i in self.active_ids:
                pts = RC.get_pattern(i, a)
                self.agents[i].move_joint_position(pts)
            a = 1 - a
            time.sleep(0.5)

    def glamorous(self, wall, w_ht):
        RC.set_wall(wall, w_ht)
        a = 0
        while True:
            for i in self.active_ids:
                pts = RC.get_pattern(i, a)
                self.agents[i].move_joint_position(pts)
            a = 1 - a
            time.sleep(4)
