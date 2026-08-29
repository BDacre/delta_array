#!/usr/bin/env python

import math
import numpy as np

PI = math.pi

class PrismaticDelta:

    VERTEX_ANGLES = (-math.pi / 6, 7 * math.pi / 6, math.pi / 2)

    def __init__(self, platform_side_length, base_side_length, lower_leg_length, ee_z_offset=0.0):

        self.platform_side_length = platform_side_length
        self.base_side_length = base_side_length
        self.lower_leg_length = lower_leg_length

        # Vertical (+z) offset from the platform-triangle plane the leg geometry
        # solves in, up to the end-effector reference point IK accepts / FK returns.
        self.ee_z_offset = ee_z_offset

        self.side_length_platform = platform_side_length
        self.side_length_base = base_side_length
        self.leg_length = lower_leg_length

        # geometry of the equilateral triangles (center -> vertex / side midpoint)
        self.platform_circumradius = math.sqrt(3) * platform_side_length / 3
        self.base_circumradius = math.sqrt(3) * base_side_length / 3
        self.platform_inradius = math.sqrt(3) * platform_side_length / 6

        # Base vertices are the (fixed) prismatic actuator axes, one per vertex.
        self.base_vertex_1, self.base_vertex_2, self.base_vertex_3 = self._triangle_vertices(self.base_circumradius)

    @classmethod
    def _triangle_vertices(cls, circumradius):
        """The three equilateral-triangle vertices (relative to its center) as (1, 3) row vectors."""
        return [
            np.array([[circumradius * math.cos(angle), circumradius * math.sin(angle), 0.0]])
            for angle in cls.VERTEX_ANGLES
        ]

    def get_base_vertices(self, offset, rotation):
        """Base vertices (actuator axes) each rotated then translated by offset."""
        base_vertices = (self.base_vertex_1, self.base_vertex_2, self.base_vertex_3)
        return [np.transpose(rotation @ np.transpose(vertex)) + offset for vertex in base_vertices]

    def get_carriage_joints(self, heights, offset, rotation):
        [transformed_base_vertex_1, transformed_base_vertex_2, transformed_base_vertex_3] = self.get_base_vertices(offset, rotation)
        carriage_joint_1 = transformed_base_vertex_1 + np.array([[0, 0, heights[0]]])
        carriage_joint_2 = transformed_base_vertex_2 + np.array([[0, 0, heights[1]]])
        carriage_joint_3 = transformed_base_vertex_3 + np.array([[0, 0, heights[2]]])
        return [carriage_joint_1, carriage_joint_2, carriage_joint_3]

    def get_platform_vertices(self, platform_center, offset, rotation):
        """Platform vertex positions for a given center, each rotated then translated by offset."""
        local_vertices = [platform_center + vertex_offset
                          for vertex_offset in self._triangle_vertices(self.platform_circumradius)]
        return [np.transpose(rotation @ np.transpose(vertex)) + offset for vertex in local_vertices]

    def validate_pts(self, pts, carriage_joint_lim, par_joint_lim):
        """Filter trajectory points to those that satisfy the joint limits.

        Returns [valid_heights, valid_pts]: for every reachable point in pts
        (via IK, i.e. with every carriage joint below the platform) whose
        parallel and carriage joint angles stay within the given limits.
        """
        pts = np.asarray(pts, dtype=float)
        bases = np.concatenate((self.base_vertex_1, self.base_vertex_2, self.base_vertex_3))
        normalized_bases = bases / np.linalg.norm(bases, axis=1, keepdims=True)

        valid_pts = []
        valid_heights = []
        for pt in pts:
            # rotz(0) == identity, so no rotation is applied to the platform.
            platform_vertices = np.concatenate(
                self.get_platform_vertices(pt, np.zeros((1, 3)), np.identity(3))
            )

            # Project the platform vertices onto the floor (z = 0) and measure
            # the parallel (base universal joint) angle off each actuator axis.
            floor_pt = np.concatenate((platform_vertices[:, 0:2], np.zeros((3, 1))), axis=1)
            leg_dirs = floor_pt - bases
            normalized_leg_dirs = leg_dirs / np.linalg.norm(leg_dirs, axis=1, keepdims=True)
            sin_parallel = np.linalg.norm(np.cross(normalized_bases, normalized_leg_dirs), axis=1)
            parallel_joint_angles = np.arcsin(np.clip(sin_parallel, -1.0, 1.0))
            if np.any(parallel_joint_angles > par_joint_lim):
                continue

            heights = self.ik(pt)
            if np.any(np.isnan(heights)) or np.any(heights < 0):
                continue

            # heights are in the platform-center frame, so measure the carriage
            # angle from the platform-center z (pt is the offset EE reference).
            platform_center_z = pt[2] - self.ee_z_offset
            carriage_joint_angles = math.pi / 2 - np.arcsin(
                np.clip((platform_center_z - heights) / self.lower_leg_length, -1.0, 1.0))
            if np.any(carriage_joint_angles > carriage_joint_lim):
                continue

            valid_heights.append(heights)
            valid_pts.append(pt)

        valid_heights = np.array(valid_heights) if valid_heights else np.empty((0, 3))
        valid_pts = np.array(valid_pts) if valid_pts else np.empty((0, 3))
        return [valid_heights, valid_pts]

    def is_valid(self, pt, max_height):
        """True if pt is reachable with every actuator height in [0, max_height]."""
        heights = self.ik(pt)
        if np.any(np.isnan(heights)):  # unreachable (xy offset exceeds leg length)
            return False
        return bool(np.all(heights >= 0) and np.all(heights <= max_height))

    def ik(self, position):
        """End-effector position (x, y, z) -> the three prismatic actuator heights.

        Returns a (3,) array of heights. Returns an array of NaNs if the position
        is outside the reachable workspace (the required xy offset exceeds the leg
        length for some actuator). NaN is used rather than a magic sentinel so an
        unreachable request can never be mistaken for a valid command downstream.
        """
        # Incoming position is the end-effector reference point; shift down to the
        # platform-center frame the leg geometry solves in.
        position = np.asarray(position, dtype=float).reshape(3).copy()
        position[2] -= self.ee_z_offset
        base_vertices = (self.base_vertex_1, self.base_vertex_2, self.base_vertex_3)
        platform_vertex_offsets = self._triangle_vertices(self.platform_circumradius)
        squared_leg_length = self.lower_leg_length ** 2

        heights = np.empty(3)
        for i, (vertex_offset, base_vertex) in enumerate(zip(platform_vertex_offsets, base_vertices)):
            platform_vertex = position + vertex_offset[0]
            # squared distance from the platform vertex to the (vertical) actuator axis
            squared_xy_distance = np.sum((platform_vertex[0:2] - base_vertex[0][0:2]) ** 2)
            if squared_xy_distance > squared_leg_length:
                return np.full(3, np.nan)
            heights[i] = position[2] - math.sqrt(squared_leg_length - squared_xy_distance)
        return heights

    def ik_traj(self, trajectory):
        heights_trajectory = np.zeros((len(trajectory), 3))
        for i in range(len(trajectory)):
            heights_trajectory[i] = self.ik(trajectory[i])

        return heights_trajectory

    def bound_workspace(self, max_height):
        height_sample = np.arange(0, max_height, 0.3)
        pts = np.zeros((height_sample.shape[0]**3, 3))
        index = -1
        for height_1 in height_sample:
            for height_2 in height_sample:
                for height_3 in height_sample:
                    test_pt = self.fk([height_1, height_2, height_3])
                    if(np.sum(np.isnan(test_pt)) == 0):
                        index = index + 1
                        pts[index,:] = test_pt
                        # print('sum of NaN loop')
        pts = pts[0:index+1,:]
        # ipdb.set_trace()
        return pts

    def bound_constrained_workspace(self, z_vals, max_height):
        height_sample = np.arange(0,max_height,0.1)
        if z_vals[1] < 0:
            height_1_samples = height_sample
        else:
            height_1_samples = z_vals[1]
        if z_vals[2] < 0:
            height_2_samples = height_sample
        else:
            height_2_samples = z_vals[2]
        if z_vals[3] < 0:
            height_3_samples = height_sample
        else:
            height_3_samples = z_vals[3]
        pts = np.zeros((height_sample.shape[0]**3,3))
        index = -1
        for height_1 in height_1_samples:
            for height_2 in height_2_samples:
                for height_3 in height_3_samples:
                    test_pt = self.fk(np.array([height_1, height_2, height_3]))
                    if(np.sum(test_pt) == 0):
                        index = index + 1
                        pts[index,:] = test_pt

        pts = pts[0:index,:]

        return pts


    def fk(self, heights):

        sphere_center_1 = self.base_vertex_1 + np.array([0,0,heights[0]])
        sphere_center_2 = self.base_vertex_2 + np.array([0,0,heights[1]])
        sphere_center_3 = self.base_vertex_3 + np.array([0,0,heights[2]])

        # Subtract each platform vertex offset so the three-sphere intersection
        # solves for the platform CENTER rather than a vertex.
        platform_vertex_1, platform_vertex_2, platform_vertex_3 = self._triangle_vertices(self.platform_circumradius)
        shifted_sphere_center_1 = sphere_center_1 - platform_vertex_1
        shifted_sphere_center_2 = sphere_center_2 - platform_vertex_2
        shifted_sphere_center_3 = sphere_center_3 - platform_vertex_3

        leg_length = self.lower_leg_length
        position = self.interx(shifted_sphere_center_1, shifted_sphere_center_2, shifted_sphere_center_3, leg_length, leg_length, leg_length, 1)
        position = np.transpose(position[0:3])
        # Shift from the platform-center frame up to the end-effector reference point.
        position[0, 2] += self.ee_z_offset

        return position

    def fk_traj(self, heights):
        # FK on trajectory
        traj = np.zeros((heights.shape[0],3))
        for i in np.arange(heights.shape[0]):
            traj[i,:] = self.fk(heights[i,:])

        return traj

    def find_workspace_shape(self, max_height):
        """Approximate the reachable workspace as [low_z, high_z, max_rad_z, max_rad].

        Scans the central axis for the reachable z-range, then for each reachable
        z grows the radius outward to find the largest reachable radius and the z
        at which it occurs.
        """
        step = .1
        z_samples = np.arange(0, max_height, step) + self.lower_leg_length
        low_z = max_height + self.lower_leg_length
        high_z = 0.0
        max_rad = 0.0
        max_rad_z = 0.0
        for z in z_samples:
            if not self.is_valid(np.array([0.0, 0.0, z]), max_height):
                continue
            low_z = min(low_z, z)
            high_z = max(high_z, z)

            radius = 0.0
            while self.is_valid(np.array([0.0, radius + step, z]), max_height):
                radius += step
            if radius > max_rad:
                max_rad = radius
                max_rad_z = z

        return np.array([low_z, high_z, max_rad_z, max_rad])

    def find_workspace_slice(self, z, step, max_height):
        """Return the reachable workspace points at a fixed z.

        Walks outward from the center along evenly spaced directions, collecting
        every reachable point until the direction leaves the workspace. step is
        both the angular spacing (rad) between directions and the outward spatial
        increment along each.
        """
        directions = np.arange(0, 2 * PI, step)
        slice_pts = []
        for direction_angle in directions:
            direction = np.array([math.cos(direction_angle), math.sin(direction_angle), 0.0])
            pt = np.array([0.0, 0.0, z])
            while self.is_valid(pt, max_height):
                slice_pts.append(pt.copy())
                pt = pt + step * direction
        return np.array(slice_pts) if slice_pts else np.empty((0, 3))

    def find_workspace_edge(self, z, step, max_height):
        """Return points on the workspace border at a fixed z.

        For each direction, walks outward until the next step leaves the
        workspace and records the last reachable point. step is both the angular
        spacing (rad) between directions and the outward spatial increment.
        """
        directions = np.arange(0, 2 * PI, step)
        border_pts = []
        for direction_angle in directions:
            direction = np.array([math.cos(direction_angle), math.sin(direction_angle), 0.0])
            pt = np.array([0.0, 0.0, z])
            if not self.is_valid(pt, max_height):
                continue
            while self.is_valid(pt + step * direction, max_height):
                pt = pt + step * direction
            border_pts.append(pt.copy())
        return np.array(border_pts) if border_pts else np.empty((0, 3))

    def interx(self, sphere_center_1=None, sphere_center_2=None, sphere_center_3=None, radius_1=None, radius_2=None, radius_3=None, use_positive_root=None):
        # Trilateration: intersect three spheres and return one of the two solutions.
        # Math below uses short names (x1, y1, z1, r1, ...) to match the algebraic derivation.
        if sphere_center_1 is None or sphere_center_2 is None or sphere_center_3 is None or radius_1 is None or radius_2 is None or radius_3 is None or use_positive_root is None:
            use_positive_root = 1 # default value
        # import ipdb
        # ipdb.set_trace()
        x1 = sphere_center_1[0,0]
        y1 = sphere_center_1[0,1]
        z1 = sphere_center_1[0,2]
        x2 = sphere_center_2[0,0]
        y2 = sphere_center_2[0,1]
        z2 = sphere_center_2[0,2]
        x3 = sphere_center_3[0,0]
        y3 = sphere_center_3[0,1]
        z3 = sphere_center_3[0,2]
        r1 = radius_1
        r2 = radius_2
        r3 = radius_3
        # convert in coord sys at [x1 y1 z1] oriented same as global
        # x2 = 1
        # y2 = 1
        # z2 = 0.2
        # x3 = 2
        # y3 = 1
        # z3 = 1
        # r1 = 2.5
        # r2 = 2.6
        # r3 = 2.7 # TEST VALUES

        x2 = x2 - x1
        y2 = y2 - y1
        z2 = z2 - z1
        x3 = x3 - x1
        y3 = y3 - y1
        z3 = z3 - z1

        a=(16.0*y2**2*z3*y3**2*z2*x3*r1**2*x2-4*y2**3*z3*y3*z2*x3*r1**2*x2+4*y2**3*z3*y3*z2*x3*x2*r3**2-4*y2*y3**3*z2*x2*z3*r1**2*x3+4*y2*y3**3*z2*x2*z3*r2**2*x3+16*z2*x3**2*x2**2*r1**2*y3*y2*z3-4*z2*x3**3*x2*r1**2*y3*y2*z3+4*z2*x3**3*x2*y2*z3*r2**2*y3-4*x2**3*z3*x3*y2*r1**2*z2*y3+4*x2**3*z3*x3*y2*r3**2*z2*y3-4*y2*z3*z2**3*y3*x3*r1**2*x2+4*y2*z3*z2**3*y3*x3*x2*r3**2+8*y2*z3**2*z2**2*y3*x2*r1**2*x3-4*y2*z3**2*z2**2*y3*x2*r2**2*x3+4*x2**2*y3**2*z2*y2**2*z3*x3**2-4*x2**4*z3**2*x3**2*y3*y2-2*z2**2*x3**4*x2**2*y2**2+2*z2**2*x3**3*y2**2*r1**2*x2-2*z2**2*x3**3*y2**2*x2*r3**2+2*z2**2*x3**5*x2*y2*y3+2*x2**5*z3**2*x3*y3*y2+2*z2**3*y3**2*y2**2*z3*x3**2+2*z2**3*y3**2*x2**2*z3*x3**2+2*z2**3*y3**2*x2**2*z3*r1**2-2*z2**3*y3**2*x2**2*z3*r3**2+2*x2**2*y3**4*z2*y2**2*z3-4*x2**2*y3**2*z2**2*x3**2*y2**2+2*x2**4*y3**2*z2*z3*x3**2-2*x2**2*y3**3*z2**2*y2*x3**2+2*x2**2*y3**3*z2**2*y2*z3**2+2*x2**3*y3**2*z2**2*x3*z3**2+2*x2**2*y3**2*z2*y2**2*z3**3+2*x2**2*y3**3*z2**2*y2*r1**2+2*x2**2*y3**2*z2**2*x3**2*r2**2+2*x2**4*y3**2*z2*z3*r1**2-2*x2**4*y3**2*z2*z3*r3**2-2*x2**2*y3**3*z2**2*y2*r3**2+2*x2**3*y3**2*z2**2*x3*r1**2-2*x2**3*y3**2*z2**2*x3*r3**2+2*y2**4*z3*x3**2*y3**2*z2+2*y2**2*z3*x3**4*z2*x2**2-4*y2**2*z3**2*x3**2*x2**2*y3**2+2*y2**3*z3**2*x3**2*z2**2*y3+2*y2**2*z3**2*x3**3*x2*z2**2-2*y2**3*z3**2*x3**2*x2**2*y3+2*y2**3*z3**2*x3**2*r1**2*y3+2*y2**2*z3*x3**4*z2*r1**2-2*y2**2*z3*x3**4*z2*r2**2+2*y2**2*z3**2*x3**2*x2**2*r3**2-2*y2**3*z3**2*x3**2*r2**2*y3+2*y2**2*z3**2*x3**3*x2*r1**2-2*y2**2*z3**2*x3**3*x2*r2**2-2*y2**2*z3**2*y3**2*x2**3*x3-4*y2**4*z3**2*y3**2*x2*x3+2*y2**2*z3**2*y3**2*x2**2*r3**2+4*y2**3*z3**2*y3*x2**3*x3+2*y2**5*z3**2*y3*x2*x3+4*y2*y3**3*z2**2*x3**3*x2+2*y2*y3**5*z2**2*x3*x2-2*y2**2*y3**2*z2**2*x3**3*x2-4*y2**2*y3**4*z2**2*x3*x2+2*y2**2*y3**2*z2**2*x3**2*r2**2-4*z2**2*x3**4*x2**2*y2*y3+2*z2*x3**2*x2**2*y2**2*z3**3+2*z2*x3**2*y2**4*z3*r1**2+2*z2**2*x3**2*y2**3*r1**2*y3-2*z2*x3**2*y2**4*z3*r3**2-2*z2**2*x3**2*y2**3*r3**2*y3-z2**4*y3**2*x3**2*x2**2-z2**4*y3**2*x3**2*y2**2+2*z2**3*y3**4*x2**2*z3+2*z2**3*y3**2*x2**2*z3**3+2*x2**2*y3**5*z2**2*y2-2*x2**2*y3**4*z2**2*y2**2-2*x2**4*y3**2*z2**2*x3**2+2*x2**3*y3**2*z2**2*x3**3+2*x2**4*y3**4*z2*z3+2*x2**3*y3**4*z2**2*x3+2*x2**2*y3**4*z2**2*r2**2-2*y2**4*z3**2*x3**2*y3**2+2*y2**5*z3**2*x3**2*y3+2*y2**4*z3*x3**4*z2+2*y2**2*z3**2*x3**3*x2**3-2*y2**2*z3**2*x3**4*x2**2+2*y2**4*z3**2*x3**3*x2+2*y2**2*z3*x3**4*z2**3-y2**2*z3**4*x3**2*x2**2+2*y2**4*z3**2*x3**2*r3**2-2*y2**2*z3**2*y3**4*x2**2+2*y2**3*z3**2*y3**3*x2**2-y2**2*z3**4*y3**2*x2**2-y2**4*z3**2*y3**2*x2**2+2*y2**3*y3**3*z2**2*x3**2-y2**2*y3**4*z2**2*x3**2-2*y2**4*y3**2*z2**2*x3**2-z2**4*y3**4*x2**2-2*x2**4*y3**4*z2**2-2*y2**4*z3**2*x3**4-y2**4*z3**4*x3**2-2*z2**2*x3**4*y2**4-z2**4*x3**4*y2**2-2*x2**4*z3**2*y3**4-x2**4*z3**4*y3**2-z3**2*y2**6*x3**2-x3**6*z2**2*y2**2-z3**2*x2**6*y3**2-2*y2**4*x3**4*y3**2+2*y2**4*x3**4*r3**2+2*y2**5*x3**4*y3-y2**4*x3**2*r1**4-y2**4*x3**2*y3**4+2*y2**5*x3**2*y3**3-y2**4*x3**2*r3**4-y2**6*x3**2*y3**2-y2**2*x3**4*r1**4-y2**2*x3**4*x2**4+2*y2**2*x3**5*x2**3-y2**2*x3**4*r2**4-y2**2*x3**6*x2**2-2*y2**4*x3**4*x2**2+2*y2**4*x3**4*r2**2+2*y2**4*x3**5*x2+2*x2**4*y3**5*y2-2*x2**4*y3**4*y2**2+2*x2**4*y3**4*r2**2-x2**2*y3**4*r1**4-x2**2*y3**6*y2**2+2*x2**2*y3**5*y2**3-x2**2*y3**4*y2**4-x2**2*y3**4*r2**4-x2**4*y3**2*r1**4-x2**6*y3**2*x3**2+2*x2**5*y3**2*x3**3-x2**4*y3**2*x3**4-x2**4*y3**2*r3**4+2*x2**5*y3**4*x3-2*x2**4*y3**4*x3**2+2*x2**4*y3**4*r3**2-y3**6*z2**2*x2**2-y2**4*x3**6-y2**6*x3**4-x2**6*y3**4-x2**4*y3**6-2*z2*x3**2*r1**2*y2**2*z3*r3**2-2*z2*y3**2*r2**2*x2**2*z3*r1**2+2*z2*y3**2*r2**2*x2**2*z3*r3**2+2*x2**4*y3**2*z2*z3**3+2*y2*r1**4*z2**2*y3*x3*x2+2*x2**2*y3**2*z2*y2**2*z3*r1**2-8*x2**2*y3**3*z2*r1**2*y2*z3-2*x2**2*y3**2*z2*y2**2*z3*r3**2-8*x2**3*y3**2*z2*z3*r1**2*x3+2*y2**2*z3*x3**2*r1**2*y3**2*z2-8*y2**3*z3*x3**2*r1**2*z2*y3-2*y2**2*z3*x3**2*z2*y3**2*r2**2-8*y2**2*z3*x3**3*z2*r1**2*x2-4*y2**2*z3**2*y3**2*x2*z2**2*x3-4*y2**2*z3**2*y3**2*x2*r1**2*x3+4*y2**2*z3**2*y3**2*x2*r2**2*x3-4*y2**3*z3*y3*z2*x3**3*x2-4*y2**3*z3*y3**3*z2*x3*x2-4*y2**3*z3**3*y3*z2*x3*x2+4*y2**3*z3**2*y3*x2*z2**2*x3-4*y2**3*z3**2*y3*x2*r2**2*x3-4*y2*y3**3*z2*x2**3*z3*x3+4*y2*y3**3*z2**2*x3*x2*z3**2-4*y2*y3**3*z2**3*x2*z3*x3-4*y2*y3**3*z2**2*x3*x2*r3**2-4*y2**2*y3**2*z2**2*x3*r1**2*x2+4*y2**2*y3**2*z2**2*x3*x2*r3**2-4*z2**2*x3**2*x2**2*y2*z3**2*y3+2*z2*x3**2*x2**2*y2**2*z3*r1**2-4*z2**2*x3**2*x2**2*y2*r1**2*y3-2*z2*x3**2*x2**2*y2**2*z3*r3**2+4*z2**2*x3**2*x2**2*y2*r3**2*y3-4*z2**3*x3**3*x2*y2*z3*y3+4*z2**2*x3**3*x2*y2*z3**2*y3-4*z2*x3**3*x2**3*y3*y2*z3-4*z2**2*x3**3*x2*y2*r3**2*y3+4*x2**3*z3**2*x3*y2*z2**2*y3-4*x2**3*z3**3*x3*y2*z2*y3-4*x2**3*z3**2*x3*y2*r2**2*y3+2*x2**2*z3*x3**2*r1**2*y3**2*z2-4*x2**2*z3**2*x3**2*r1**2*y3*y2-2*x2**2*z3*x3**2*z2*y3**2*r2**2+4*x2**2*z3**2*x3**2*y2*r2**2*y3-4*y2*z3**3*z2**3*y3*x3*x2+2*y2*z3**2*z2**4*y3*x2*x3+2*y2*z3**4*z2**2*y3*x3*x2-2*r1**2*y3**2*z2*x2**2*z3*r3**2-2*y2**2*z3*r1**2*z2*x3**2*r2**2+2*r1**4*y3*y2*z3**2*x2*x3+2*z2**2*x3**5*y2**2*x2+2*z2**2*x3**4*y2**3*y3+2*z2*x3**2*y2**4*z3**3+2*z2**2*x3**4*y2**2*r2**2-z2**2*x3**4*x2**2*y3**2+2*x2**5*z3**2*x3*y3**2-x2**4*z3**2*x3**2*y2**2-2*x2**4*z3**2*x3**2*y3**2+2*x2**4*z3**2*y3**3*y2+2*x2**4*z3**2*y3**2*r3**2-2*y2**2*x3**4*z2**2*y3**2-2*z2**2*x3**2*x2**2*y3**4-2*x2**2*z3**2*y2**4*x3**2-2*x2**4*y3**2*y2**2*z3**2+2*y2**2*z3**3*z2**3*x3**2-z3**2*y2**2*r1**4*x3**2-z3**2*y2**2*z2**4*x3**2-z3**2*y2**2*r2**4*x3**2-2*z3**2*y2**4*x3**2*z2**2+2*z3**2*y2**4*x3**2*r2**2-2*x3**4*z2**2*y2**2*z3**2+2*x3**4*z2**2*y2**2*r3**2-x3**2*z2**2*y2**2*r1**4-x3**2*z2**2*y2**2*z3**4-x3**2*z2**2*y2**2*r3**4-2*z3**2*x2**4*y3**2*z2**2+2*z3**2*x2**4*y3**2*r2**2-z3**2*x2**2*r1**4*y3**2-z3**2*x2**2*z2**4*y3**2-z3**2*x2**2*r2**4*y3**2+2*y2**3*x3**4*r1**2*y3-2*y2**3*x3**4*x2**2*y3-2*y2**3*x3**4*r2**2*y3+2*y2**4*x3**3*r1**2*x2-2*y2**4*x3**3*x2*y3**2-2*z2**2*x3**2*x2**2*y3**2*z3**2+2*z2**2*x3**2*x2**2*y3**2*r3**2-2*x2**2*z3**2*y2**2*x3**2*z2**2+2*x2**2*z3**2*y2**2*x3**2*r2**2+2*x2**2*y3**2*y2**2*z3**2*r2**2+2*y2**2*z3**3*z2*x3**2*r1**2-2*y2**2*z3**3*z2*x3**2*r2**2+2*z2**3*x3**2*y2**2*z3*r1**2-2*z2**3*x3**2*y2**2*z3*r3**2+2*x2**2*z3**3*r1**2*y3**2*z2-2*x2**2*z3**3*z2*y3**2*r2**2+2*r1**4*y3**2*z2*x2**2*z3+2*y2**2*z3*r1**4*z2*x3**2+2*x2**2*z3*y3**4*r1**2*z2+2*x2**2*z3**2*y3**3*r1**2*y2-2*x2**2*z3*y3**4*z2*r2**2-2*x2**2*z3**2*y3**3*y2*r2**2+2*x2**3*z3**2*y3**2*r1**2*x3-2*x2**3*z3**2*y3**2*r2**2*x3-2*y2**2*z3**2*z2**2*y3**2*x2**2-2*y2**2*x3**2*z2**2*y3**2*z3**2+2*y2**2*x3**2*z2**2*y3**2*r3**2-4*y2*z3**2*z2**2*y3*x3*x2*r3**2-4*y2*z3**3*z2*y3*x2*r1**2*x3+4*y2*z3**3*z2*y3*x2*r2**2*x3-4*r1**4*y3*y2*z3*z2*x3*x2-4*r1**2*y3*y2*z3**2*x2*r2**2*x3-4*y2*r1**2*z2**2*y3*x3*x2*r3**2+4*r1**2*y3*y2*z3*z2*x3*x2*r3**2+4*y2*r1**2*z2*y3*x2*z3*r2**2*x3+2*z2*x3**2*r2**2*y2**2*z3*r3**2+2*y2*z3**2*r2**4*y3*x2*x3+2*y2*r3**4*z2**2*y3*x3*x2+4*y2**2*x3*x2*y3**2*r1**2*r3**2+4*y2**2*x3*x2*y3**2*r1**2*r2**2-4*y2**2*x3*x2*y3**2*r3**2*r2**2+4*y2*x3**2*x2**2*y3*r1**2*r2**2+4*y2*x3**2*x2**2*y3*r1**2*r3**2-4*y2*x3**2*x2**2*y3*r2**2*r3**2-4*y2**3*x3*x2*y3*z3**2*r3**2-4*y2*x3*x2*y3**3*r1**2*r2**2-4*y2**3*x3*x2*y3*r1**2*r3**2-4*y2*x3*x2*y3**3*z2**2*r2**2-4*y2*x3*x2**3*y3*r1**2*r3**2-4*y2*x3**3*x2*y3*r1**2*r2**2-4*y2*x3**3*x2*y3*z2**2*r2**2-4*y2*x3*x2**3*y3*z3**2*r3**2-4*z3**2*y2**2*r1**2*x3**2*z2**2+2*z3**2*y2**2*r1**2*x3**2*r2**2+2*z3**2*y2**2*z2**2*x3**2*r2**2+2*x3**2*z2**2*y2**2*z3**2*r3**2+2*x3**2*z2**2*y2**2*r1**2*r3**2-4*z3**2*x2**2*r1**2*y3**2*z2**2+2*z3**2*x2**2*r1**2*y3**2*r2**2+2*z3**2*x2**2*z2**2*y3**2*r2**2-2*y2**3*x3**2*r1**2*y3*r3**2-2*y2**3*x3**2*x2**2*y3*r1**2+2*y2**3*x3**2*x2**2*y3*r3**2-2*y2**3*x3**2*r1**2*r2**2*y3+2*y2**3*x3**2*r3**2*r2**2*y3-2*y2**2*x3**3*r1**2*x2*r2**2-2*y2**2*x3**3*r1**2*x2*y3**2-2*y2**2*x3**3*r1**2*x2*r3**2+2*y2**2*x3**3*r2**2*x2*y3**2+2*y2**2*x3**3*r2**2*x2*r3**2+2*y2**2*x3**2*r1**2*y3**2*r2**2+4*y2**2*x3**2*x2**2*y3**2*r2**2+2*y2**2*x3**2*r1**2*x2**2*r3**2+4*y2**2*x3**2*x2**2*y3**2*r3**2-8*y2**3*x3**3*r1**2*x2*y3-2*x2**2*y3**3*r1**2*y2*x3**2-2*x2**2*y3**3*r1**2*y2*r3**2-2*x2**2*y3**3*y2*r1**2*r2**2+2*x2**2*y3**3*y2*x3**2*r2**2+2*x2**2*y3**3*y2*r3**2*r2**2-2*x2**3*y3**2*r1**2*y2**2*x3-2*x2**3*y3**2*r1**2*r2**2*x3-2*x2**3*y3**2*r1**2*x3*r3**2+2*x2**3*y3**2*y2**2*x3*r3**2+2*x2**3*y3**2*r2**2*x3*r3**2+2*x2**2*y3**2*y2**2*r1**2*r3**2-4*y2*z3*r2**2*y3*z2*x3*x2*r3**2-4*x2**4*y3**2*r1**2*x3**2+2*x2**4*y3**2*r1**2*r3**2+2*x2**3*y3**2*r1**2*x3**3+2*x2**4*y3**2*x3**2*r2**2-2*x2**5*y3**2*x3*r3**2-2*x2**3*y3**2*r2**2*x3**3+2*x2**4*y3**2*x3**2*r3**2-y3**2*z2**2*r1**4*x2**2-y3**2*z2**2*x2**2*z3**4-y3**2*z2**2*x2**2*r3**4-2*y3**4*z2**2*x2**2*z3**2+2*y3**4*z2**2*x2**2*r3**2+4*y2**3*x3*x2**3*y3**3+4*y2**3*x3**3*x2*y3**3+2*y2*x3*x2**5*y3**3+2*y2**3*x3**5*x2*y3+2*y2**3*x3*x2*y3**5-4*y2**4*x3*x2*y3**4+2*y2**5*x3*x2*y3**3+2*y2*x3**3*x2**5*y3-4*y2*x3**4*x2**4*y3+2*y2**5*x3**3*x2*y3+2*y2*x3**5*x2**3*y3+2*y2*x3*x2**3*y3**5+4*y2**3*x3**3*x2**3*y3+4*y2*x3**3*x2**3*y3**3-2*y2**4*x3**3*x2*r3**2-2*y2**5*x3**2*r3**2*y3+2*y2**4*x3**2*y3**2*r2**2+2*y2**3*x3**2*r1**2*y3**3+2*y2**3*x3**2*r1**4*y3-4*y2**4*x3**2*r1**2*y3**2-3*y2**4*x3**2*x2**2*y3**2+2*y2**4*x3**2*r1**2*r3**2+2*y2**5*x3**2*r1**2*y3+2*y2**4*x3**2*y3**2*r3**2-2*y2**3*x3**2*y3**3*r2**2-y2**2*x3**2*r1**4*y3**2-3*y2**2*x3**2*x2**4*y3**2-y2**2*x3**2*r2**4*y3**2-y2**2*x3**2*r1**4*x2**2-3*y2**2*x3**2*x2**2*y3**4-y2**2*x3**2*x2**2*r3**4+2*y2**2*x3**3*r1**4*x2+2*y2**2*x3**3*r1**2*x2**3-4*y2**2*x3**4*r1**2*x2**2+2*y2**2*x3**4*r1**2*r2**2+2*y2**2*x3**5*r1**2*x2+2*y2**2*x3**4*x2**2*r2**2-2*y2**2*x3**3*x2**3*r3**2-2*y2**2*x3**5*r2**2*x2-3*y2**2*x3**4*x2**2*y3**2+2*y2**2*x3**4*x2**2*r3**2+2*x2**4*y3**3*y2*r1**2-2*x2**4*y3**3*y2*x3**2-2*x2**4*y3**3*y2*r3**2+2*x2**3*y3**4*r1**2*x3-2*x2**3*y3**4*y2**2*x3-2*x2**3*y3**4*r2**2*x3-2*x2**2*y3**3*y2**3*r3**2+2*x2**2*y3**4*y2**2*r2**2+2*x2**2*y3**5*r1**2*y2+2*x2**2*y3**3*r1**4*y2-4*x2**2*y3**4*r1**2*y2**2+2*x2**2*y3**4*r1**2*r2**2+2*x2**2*y3**3*y2**3*r1**2+2*x2**2*y3**4*y2**2*r3**2-2*x2**2*y3**5*y2*r2**2-x2**2*y3**2*y2**2*r1**4-x2**2*y3**2*y2**2*r3**4-x2**2*y3**2*r1**4*x3**2-x2**2*y3**2*r2**4*x3**2+2*x2**3*y3**2*r1**4*x3+2*x2**5*y3**2*r1**2*x3+2*x2**2*y3**2*r1**2*x3**2*r2**2-8*x2**3*y3**3*r1**2*y2*x3+2*y3**2*z2**2*r1**2*x2**2*r3**2+2*y3**2*z2**2*x2**2*z3**2*r3**2+4*y2**4*x3*x2*y3**2*r3**2+4*y2**3*x3*x2*y3**3*z2**2-4*y2**3*x3*x2*y3**3*r2**2-4*y2**2*x3*x2*y3**4*r1**2-4*y2**2*x3*x2*y3**2*r1**4+8*y2**3*x3*x2*y3**3*r1**2+4*y2*x3*x2**3*y3**3*z2**2-4*y2*x3*x2**3*y3**3*r2**2-4*y2**4*x3*x2*y3**2*r1**2+4*y2**3*x3**3*x2*y3*z3**2-4*y2**3*x3**3*x2*y3*r3**2+4*y2**3*x3*x2*y3**3*z3**2-4*y2**3*x3*x2*y3**3*r3**2+4*y2**2*x3*x2*y3**4*r2**2+2*y2*x3*x2*y3**3*r1**4+2*y2**3*x3*x2*y3*r1**4+2*y2**3*x3*x2*y3*z3**4+2*y2**3*x3*x2*y3*r3**4+2*y2*x3*x2*y3**3*z2**4+2*y2*x3*x2*y3**3*r2**4+2*y2*x3*x2**3*y3*r1**4+2*y2*x3**3*x2*y3*r1**4+2*y2*x3**3*x2*y3*z2**4+2*y2*x3**3*x2*y3*r2**4+2*y2*x3*x2**3*y3*z3**4+2*y2*x3*x2**3*y3*r3**4-4*y2*x3**2*x2**2*y3*r1**4-4*y2*x3**2*x2**4*y3*r1**2+8*y2*x3**3*x2**3*y3*r1**2-4*y2*x3**4*x2**2*y3*r1**2+4*y2*x3**3*x2**3*y3*z2**2-4*y2*x3**3*x2**3*y3*r2**2+4*y2*x3**2*x2**4*y3*r3**2+4*y2**3*x3**3*x2*y3*z2**2-4*y2**3*x3**3*x2*y3*r2**2+4*y2*x3**4*x2**2*y3*r2**2+4*y2*x3**3*x2**3*y3*z3**2-4*y2*x3**3*x2**3*y3*r3**2+4*y2*x3*x2**3*y3**3*z3**2-4*y2*x3*x2**3*y3**3*r3**2+16*y2**2*x3**2*x2**2*y3**2*r1**2)
        b=(-z2**3*y3**2-x2**2*y3**2*z2-y2**2*z3*x3**2-y2**2*z3*y3**2+y2**3*z3*y3+y2*y3**3*z2-y2**2*y3**2*z2-z2*x3**2*x2**2-z2*x3**2*y2**2+z2*x3**3*x2+x2**3*z3*x3-x2**2*z3*x3**2-x2**2*z3*y3**2+y2*z3*z2**2*y3+y2*x3**2*z2*y3+y2*z3**2*z2*y3+z2*x3*x2*y3**2+z2*x3*x2*z3**2+x2*z3*y2**2*x3+x2*z3*z2**2*x3+x2**2*y3*y2*z3-y2**2*z3**3-z2**3*x3**2-x2**2*z3**3-r1**2*y3**2*z2-y2**2*z3*r1**2+r1**2*y3*y2*z3+y2*r1**2*z2*y3+z2*y3**2*r2**2-z2*x3**2*r1**2+z2*x3**2*r2**2-x2**2*z3*r1**2+x2**2*z3*r3**2+y2**2*z3*r3**2-y2*z3*r2**2*y3-y2*r3**2*z2*y3+z2*x3*r1**2*x2-z2*x3*x2*r3**2+x2*z3*r1**2*x3-x2*z3*r2**2*x3)
        c=(-2*y2*z3*z2*y3-2*z2*x3*x2*z3+z3**2*y2**2+x3**2*z2**2+z3**2*x2**2+y2**2*x3**2+x2**2*y3**2+y3**2*z2**2-2*y2*x3*x2*y3)
        if a<0 or c==0:
            result = np.array([np.nan,np.nan,np.nan]).reshape((3,1))
            return result # coz c is the denominator and a is under a root
        #     error('Error in interx.m at z');
        z_negative_root = -0.5 * (b - a ** (0.5)) / c
        z_positive_root = -0.5 * (b + a ** (0.5)) / c
        if z_negative_root > z_positive_root:
            if(use_positive_root):
                z = z_negative_root
            else:
                z = z_positive_root
        else:
            if(use_positive_root):
                z = z_positive_root
            else:
                z = z_negative_root
        a=(2*z*z2*x3-2*x2*z*z3+r1**2*x2-r1**2*x3-x2**2*x3-y2**2*x3-z2**2*x3+r2**2*x3+x2*x3**2+x2*y3**2+x2*z3**2-x2*r3**2)
        b=(-2*y2*x3+2*x2*y3)
        if b==0:
            result = np.array([np.nan,np.nan,np.nan]).reshape((3,1))
            return result # coz b is the denominator in the expression
        y=a/b
        if x2 == 0:
            result = np.array([np.nan,np.nan,np.nan]).reshape((3,1))
            return result
        x = 0.5*(r1**2+x2**2-2*y*y2+y2**2-2*z*z2+z2**2-r2**2)/x2
        # convert result back to global
        result = np.array([[x1],[y1],[z1],[1]]) + np.array([[x],[y],[z],[0]])

        return result
