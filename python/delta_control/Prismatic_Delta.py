#!/usr/bin/env python

import time
import serial
import math
import numpy as np
import ipdb

PI = math.pi

class Prismatic_Delta:
    def __init__(self, platform_side_length, base_side_length, lower_leg_length):
        # Naming key (legacy short form -> descriptive):
        #   s_p, s_b -> platform_side_length, base_side_length
        #   u_p, u_b -> platform_circumradius, base_circumradius (center -> vertex)
        #   w_p      -> platform_inradius (center -> side midpoint)
        #   l        -> lower_leg_length (legs attached to platform)

        self.platform_side_length = platform_side_length
        self.base_side_length = base_side_length

        self.lower_leg_length = lower_leg_length

        # geometry of equilateral triangle
        self.platform_circumradius = math.sqrt(3) * platform_side_length / 3
        self.base_circumradius = math.sqrt(3) * base_side_length / 3

        self.platform_inradius = math.sqrt(3) * platform_side_length / 6
        self.base_circumradius = math.sqrt(3) * base_side_length / 3

        base_circumradius = math.sqrt(3) * base_side_length / 3
        self.base_vertex_1 = np.array([[0, base_circumradius, 0]])
        angle_vertex_2 = -math.pi / 6
        self.base_vertex_2 = np.array([[base_circumradius * math.cos(angle_vertex_2), base_circumradius * math.sin(angle_vertex_2), 0]])
        angle_vertex_3 = 7 * math.pi / 6
        self.base_vertex_3 = np.array([[base_circumradius * math.cos(angle_vertex_3), base_circumradius * math.sin(angle_vertex_3), 0]])

    def get_base_verteces(self, offset, rotation):
        transformed_base_vertex_1 = np.transpose((rotation * np.transpose(self.base_vertex_1))) + offset
        transformed_base_vertex_2 = np.transpose((rotation * np.transpose(self.base_vertex_1))) + offset
        transformed_base_vertex_3 = np.transpose((rotation * np.transpose(self.base_vertex_1))) + offset
        return [transformed_base_vertex_1, transformed_base_vertex_2, transformed_base_vertex_3]

    def get_knee_joints(self, heights, offset, rotation):
        [transformed_base_vertex_1, transformed_base_vertex_2, transformed_base_vertex_3] = self.get_base_verteces(offset, rotation)
        knee_joint_1 = transformed_base_vertex_1 + np.array([[0, 0, heights[0]]])
        knee_joint_2 = transformed_base_vertex_2 + np.array([[0, 0, heights[1]]])
        knee_joint_3 = transformed_base_vertex_3 + np.array([[0, 0, heights[2]]])
        return [knee_joint_1, knee_joint_2, knee_joint_3]

    def get_platform_verteces(self, platform_center, offset, rotation):
        platform_circumradius = self.platform_circumradius
        local_corner_1 = platform_center + np.array([[0, platform_circumradius, 0]])
        angle_corner_2 = -math.pi / 6
        local_corner_2 = platform_center + np.array([[platform_circumradius * math.cos(angle_corner_2), platform_circumradius * math.sin(angle_corner_2), 0]])
        angle_corner_3 = 7 * math.pi / 6
        local_corner_3 = platform_center + np.array([[platform_circumradius * math.cos(angle_corner_3), platform_circumradius * math.sin(angle_corner_3), 0]])

        platform_vertex_1 = np.tranpose(rotation * np.transpose(local_corner_1)) + offset
        platform_vertex_2 = np.tranpose(rotation * np.transpose(local_corner_1)) + offset
        platform_vertex_3 = np.tranpose(rotation * np.transpose(local_corner_1)) + offset

        return [platform_vertex_1, platform_vertex_2, platform_vertex_3]

    def validate_pts(self, pts, knee_joint_lim, par_joint_lim):
        # validate that joint limits are satisfied for each point in
        # trajectory and return corrected heights that are valid

        # A corrected point is one that is acheived with every knee
        # joint below the base platform, which is not a constraint for
        # FK and workspace sampling.  This is simply done by calling IK

        valid_pts = np.zeros((len(pts), 3))
        valid_heights = np.zeros((len(pts), 3))
        valid_count = 0
        base_vertex_1 = self.base_vertex_1
        base_vertex_2 = self.base_vertex_2
        base_vertex_3 = self.base_vertex_3
        bases = np.concatenate((base_vertex_1, base_vertex_2, base_vertex_3))

        for row in range(len(pts)):
            pt = pts[row]
            [platform_vertex_1, platform_vertex_2, platform_vertex_3] = self.get_platform_verteces(pt, np.array([[0, 0, 0]]), np.identity(
                3))  # was rotz(0), but this is same as identity
            platform_vertices = np.concatenate((platform_vertex_1, platform_vertex_2, platform_vertex_3))

            floor_pt = np.concatenate((platform_vertices[:, 0:2], np.zeros((3, 1))), axis=1)
            normalized_bases = bases / np.linalg.norm(bases, axis=1,
                                                 keepdims=True)  # https://stackoverflow.com/questions/37914795/normalising-rows-in-numpy-matrix
            normalized_floor_pt = (floor_pt - bases) / np.linalg.norm((floor_pt - bases), axis=1, keepdims=True)
            parallel_joint_angles = math.asin(
                math.sqrt(sum(np.power(np.cross(normalized_bases, normalized_floor_pt), 2))))  # sum of each row
            if sum(parallel_joint_angles > par_joint_lim) == 0:
                heights = self.IK(pt)
                if sum(sum(heights < 0)) > 0:
                    continue
                knee_joint_angles = math.pi / 2 - math.asin(
                    (np.ones((3, 1)) * pt[2] - np.transpose(heights)) / self.lower_leg_length)  # FIXME
                if sum(knee_joint_angles > knee_joint_lim) == 0:
                    valid_heights[valid_count] = heights
                    valid_pts[valid_count] = pt
                    valid_count = valid_count + 1

        valid_pts = valid_pts[0:valid_count + 1]
        valid_heights = valid_heights[0:valid_count + 1]

        return [valid_heights, valid_pts]

    def is_valid(self, pt, max_height):
        min_height = 0
        heights = self.IK(pt)

        # not imaginary and not above max height
        is_valid = sum(heights < min_height) + sum(heights > max_height) == 0 and sum(heights == abs(heights)) == 3
        return is_valid

    def IK(self, position):
        # find corners of ee platform
        platform_circumradius = self.platform_circumradius
        platform_corner_1 = position + np.array([[0, platform_circumradius, 0]])
        angle_corner_2 = -math.pi / 6
        platform_corner_2 = position + np.array([[platform_circumradius * math.cos(angle_corner_2), platform_circumradius * math.sin(angle_corner_2), 0]])
        angle_corner_3 = 7 * math.pi / 6
        platform_corner_3 = position + np.array([[platform_circumradius * math.cos(angle_corner_3), platform_circumradius * math.sin(angle_corner_3), 0]])

        # squared dist to prismatic actuator axis
        squared_xy_distance_1 = np.sum(np.power(platform_corner_1[0][0:2] - self.base_vertex_1[0][0:2], 2))
        squared_xy_distance_2 = np.sum(np.power(platform_corner_2[0][0:2] - self.base_vertex_2[0][0:2], 2))
        squared_xy_distance_3 = np.sum(np.power(platform_corner_3[0][0:2] - self.base_vertex_3[0][0:2], 2))

        squared_leg_length = self.lower_leg_length ** 2
        if squared_xy_distance_1 > squared_leg_length or squared_xy_distance_2 > squared_leg_length or squared_xy_distance_3 > squared_leg_length:
            return [-42., -42., -42.]
        height_1 = position[2] - math.sqrt(squared_leg_length - squared_xy_distance_1)
        height_2 = position[2] - math.sqrt(squared_leg_length - squared_xy_distance_2)
        height_3 = position[2] - math.sqrt(squared_leg_length - squared_xy_distance_3)

        heights = [height_1, height_2, height_3]
        return heights

    def IK_Traj(self, trajectory):
        heights_trajectory = np.zeros((len(trajectory), 3))
        for i in range(len(trajectory)):
            heights_trajectory[i] = self.IK(trajectory[i])

        return heights_trajectory

    def bound_workspace(self, max_height):
        height_sample = np.arange(0, max_height, 0.3)
        pts = np.zeros((height_sample.shape[0]**3, 3))
        index = -1
        for height_1 in height_sample:
            for height_2 in height_sample:
                for height_3 in height_sample:
                    test_pt = self.FK([height_1, height_2, height_3])
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
                    test_pt = self.FK(np.array([height_1, height_2, height_3]))
                    if(np.sum(test_pt) == 0):
                        index = index + 1
                        pts[index,:] = test_pt

        pts = pts[0:index,:]

        return pts


    def FK(self, heights):

        sphere_center_1 = self.base_vertex_1 + np.array([0,0,heights[0]])
        sphere_center_2 = self.base_vertex_2 + np.array([0,0,heights[1]])
        sphere_center_3 = self.base_vertex_3 + np.array([0,0,heights[2]])

        # shift sphere centers by platform offset
        platform_circumradius = self.platform_circumradius
        shifted_sphere_center_1 = sphere_center_1 - np.array([0, platform_circumradius, 0])
        angle_corner_2 = -PI/6
        shifted_sphere_center_2 = sphere_center_2 - np.array([platform_circumradius * math.cos(angle_corner_2), platform_circumradius * math.sin(angle_corner_2), 0])
        angle_corner_3 = 7*PI/6
        shifted_sphere_center_3 = sphere_center_3 - np.array([platform_circumradius * math.cos(angle_corner_3), platform_circumradius * math.sin(angle_corner_3), 0])

        leg_length = self.lower_leg_length
        position = self.interx(shifted_sphere_center_1, shifted_sphere_center_2, shifted_sphere_center_3, leg_length, leg_length, leg_length, 1)
        position = np.transpose(position[0:3])

        return position

    def FK_Traj(self, heights):
        # FK on trajectory
        traj = np.zeros((heights.shape[0],3))
        for i in np.arange(heights.shape[0]):
            traj[i,:] = self.FK(heights[i,:])

        return traj

    def find_workspace_shape(self, max_height):
        step = .1
        z_samples = np.arange(0, max_height, step) + self.lower_leg_length
        max_rad = 0.0
        max_rad_z = 0.0
        low_z = max_height + self.lower_leg_length
        high_z = 0.0
        for z in z_samples:
            if self.is_valid(np.array([0,0,z]),max_height):
                if low_z > z:
                    low_z = z
                if high_z < z:
                    high_z = z

        x = 0.0
        y = max_rad
        pt = np.array([x,y,z])
        if self.is_valid(pt,max_height):
            while True:
                y = y + step
                pt = np.array([x,y,z])
                if self.is_valid(pt,max_height):
                    max_rad_z = z
                    max_rad = y
                else:
                    break

        return np.array([low_z,high_z,max_rad_z,max_rad])

    def find_workspace_slice(self, z, step, max_height):

        # returns workspace slice for a fixed z
        directions = np.arange(0, 2*PI, step)
        border_pts = np.zeros(math.ceil((self.lower_leg_length * 2.0 / step) ** 2, 3))
        index = 0
        for direction_angle in directions:
            pt = np.array([0, 0, z])
        while True:
            if self.is_valid(pt, max_height):
                index = index + 1
                slice_pts[index,:] = pt
            else:
                break
            pt = pt + step * [math.cos(direction_angle), math.sin(direction_angle), 0]

        slice_pts = slice_pts[0:index,:]

        return slice_pts

    def find_workspace_edge(self, z, step, max_height):
        # returns points on the border of the workspace for a fixed value of z
        # step is the rotation in radians to the next point
        directions = np.arange(0, 2*PI,step)
        border_pts = zeros(size(directions, 2), 3)
        index = 0
        for direction_angle in directions:
            pt = np.array([0, 0, z])
        while True:
            next_pt = pt + step * np.array([math.cos(direction_angle), math.sin(direction_angle), 0])
            if not self.is_valid(next_pt, max_height):
                index = index + 1
                border_pts[index,:] = pt
                break
            pt = next_pt
        return border_pts

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
