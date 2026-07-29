import numpy as np
import matplotlib.pyplot as plt

from .constants import (
    EE_OFFSET_MAGNITUDE,
    LEG_LENGTH,
    ROBOT_PITCH_X,
    ROBOT_PITCH_X_HALF,
    ROBOT_PITCH_Y,
    BASE_TRIANGLE_SIDE_LEN,
    PLATFORM_TRIANGLE_SIDE_LEN,
    EE_Z_OFFSET,
)
from .prismatic_delta import PrismaticDelta


class RoboCoords:
    def __init__(self):
        self.delta = PrismaticDelta(PLATFORM_TRIANGLE_SIDE_LEN, BASE_TRIANGLE_SIDE_LEN, LEG_LENGTH, EE_Z_OFFSET)
        self.robot_positions = np.zeros((8, 8, 2))
        self.delta_array = np.zeros((8, 8))
        self.rot_30 = np.pi / 6
        # Default workspace z bounds (meters).
        self.zmax = 0.08
        self.zmin = 0.06

        for i in range(8):
            for j in range(8):
                if j % 2 == 0:
                    self.robot_positions[i, j] = (i * ROBOT_PITCH_X, j * ROBOT_PITCH_Y)
                else:
                    self.robot_positions[i, j] = (ROBOT_PITCH_X_HALF + i * ROBOT_PITCH_X, j * ROBOT_PITCH_Y)

        self.robo_dict_inv = {
            **dict.fromkeys([(0,0),(0,1),(1,0),(1,1)], 16),
            **dict.fromkeys([(2,0),(3,0),(2,1),(3,1)], 15),
            **dict.fromkeys([(4,0),(5,0),(4,1),(5,1)], 14),
            **dict.fromkeys([(6,0),(7,0),(6,1),(7,1)], 13),
            **dict.fromkeys([(0,2),(1,2),(0,3),(1,3)], 12),
            **dict.fromkeys([(2,2),(2,3),(3,2),(3,3)], 11),
            **dict.fromkeys([(2,4),(2,5),(3,4),(3,5)], 10),
            **dict.fromkeys([(2,6),(2,7),(3,6),(3,7)], 9),
            **dict.fromkeys([(4,0),(4,1),(5,0),(5,1)], 8),
            **dict.fromkeys([(4,2),(4,3),(5,2),(5,3)], 7),
            **dict.fromkeys([(4,4),(4,5),(5,4),(5,5)], 6),
            **dict.fromkeys([(4,6),(4,7),(5,6),(5,7)], 5),
            **dict.fromkeys([(6,0),(6,1),(7,0),(7,1)], 4),
            **dict.fromkeys([(6,2),(6,3),(7,2),(7,3)], 3),
            **dict.fromkeys([(6,4),(6,5),(7,4),(7,5)], 2),
            **dict.fromkeys([(6,6),(6,7),(7,6),(7,7)], 1),
        }
        self.robo_dict = {
            16: [(0,0),(0,1),(1,0),(1,1)],
            15: [(0,2),(0,3),(1,2),(1,3)],
            14: [(0,4),(0,5),(1,4),(1,5)],
            13: [(0,6),(0,7),(1,6),(1,7)],
            12: [(2,0),(2,1),(3,0),(3,1)],
            11: [(2,2),(2,3),(3,2),(3,3)],
            10: [(2,4),(2,5),(3,4),(3,5)],
            9:  [(2,6),(2,7),(3,6),(3,7)],
            8:  [(4,0),(4,1),(5,0),(5,1)],
            7:  [(4,2),(4,3),(5,2),(5,3)],
            6:  [(4,4),(4,5),(5,4),(5,5)],
            5:  [(4,6),(4,7),(5,6),(5,7)],
            4:  [(6,0),(6,1),(7,0),(7,1)],
            3:  [(6,2),(6,3),(7,2),(7,3)],
            2:  [(6,4),(6,5),(7,4),(7,5)],
            1:  [(6,6),(6,7),(7,6),(7,7)]
        }

        self.even_pattern = np.zeros((8, 8, 3))
        self.odd_pattern = np.zeros((8, 8, 3))

    def rotate(self, vector, angle, plot=False):
        rot_matrix = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
        vector = vector @ rot_matrix
        if plot:
            plt.quiver(self.robot_positions[:,:,0].flatten(), self.robot_positions[:,:,1].flatten(), vector[:,:,0].flatten(), vector[:,:,1].flatten())
            plt.show()
        return vector

    def normalize_vec(self, vector):
        # Produce per-cell EE xy offsets of magnitude EE_OFFSET_MAGNITUDE (m).
        for i in range(vector.shape[0]):
            for j in range(vector.shape[1]):
                vector[i, j] = vector[i, j] / np.linalg.norm(vector[i, j]) * EE_OFFSET_MAGNITUDE
        return vector

    def get_dist_vec(self, point, norm=True, plot=False):
        focus_pt = np.ones((8, 8, 2)) * np.array((point[0], point[1]))
        dist_vec = focus_pt - self.robot_positions
        if norm:
            dist_vec = self.normalize_vec(dist_vec)
        if plot:
            plt.quiver(self.robot_positions[:,:,0].flatten(), self.robot_positions[:,:,1].flatten(), dist_vec[:,:,0].flatten(), dist_vec[:,:,1].flatten())
            plt.show()
        return dist_vec

    def get_rot_vec(self, point, angle, norm=True, plot=False):
        rot_matrix = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
        focus_pt = np.ones((8, 8, 2)) * np.array((point[0], point[1]))
        dist_vec = focus_pt - self.robot_positions
        if norm:
            dist_vec = self.normalize_vec(dist_vec)
        rot_vecs = dist_vec @ rot_matrix
        if plot:
            plt.quiver(self.robot_positions[:,:,0].flatten(), self.robot_positions[:,:,1].flatten(), rot_vecs[:,:,0].flatten(), rot_vecs[:,:,1].flatten())
            plt.show()
        return rot_vecs

    def set_pattern(self, vecs, zmax=None, zmin=None):
        if zmax is not None and zmin is not None:
            self.zmax, self.zmin = zmax, zmin
        for i in self.robo_dict.keys():
            idx = self.robo_dict[i]
            self.even_pattern[idx[0]] = np.array(self.delta.ik([*vecs[idx[0]], self.zmax]))
            self.even_pattern[idx[1]] = np.array(self.delta.ik([*vecs[idx[1]], self.zmax]))
            self.even_pattern[idx[2]] = np.array(self.delta.ik([*vecs[idx[2]] * -1, self.zmin]))
            self.even_pattern[idx[3]] = np.array(self.delta.ik([*vecs[idx[3]] * -1, self.zmin]))

            self.odd_pattern[idx[0]] = np.array(self.delta.ik([*vecs[idx[0]] * -1, self.zmin]))
            self.odd_pattern[idx[1]] = np.array(self.delta.ik([*vecs[idx[1]] * -1, self.zmin]))
            self.odd_pattern[idx[2]] = np.array(self.delta.ik([*vecs[idx[2]], self.zmax]))
            self.odd_pattern[idx[3]] = np.array(self.delta.ik([*vecs[idx[3]], self.zmax]))

    def get_pattern(self, id, a):
        idx = self.robo_dict[id]
        if a == 0:
            return [*self.even_pattern[idx[0]], *self.even_pattern[idx[1]], *self.even_pattern[idx[2]], *self.even_pattern[idx[3]]]
        else:
            return [*self.odd_pattern[idx[0]], *self.odd_pattern[idx[1]], *self.odd_pattern[idx[2]], *self.odd_pattern[idx[3]]]

    def set_wall(self, wall_array, wall_ht=0.135):
        for (i, j) in wall_array:
            self.even_pattern[(i, j)] = np.array(self.delta.ik([0, 0, wall_ht]))
            self.odd_pattern[(i, j)] = np.array(self.delta.ik([0, 0, wall_ht]))

    def set_pattern_for_gripper(self, idx, val, partial=0):
        if partial == 0:
            self.even_pattern[idx[0]] = np.array(self.delta.ik([*val]))
            self.even_pattern[idx[1]] = np.array(self.delta.ik([*val]))
            self.even_pattern[idx[2]] = np.array(self.delta.ik([*val]))
            self.even_pattern[idx[3]] = np.array(self.delta.ik([*val]))

            self.odd_pattern[idx[0]] = np.array(self.delta.ik([*val]))
            self.odd_pattern[idx[1]] = np.array(self.delta.ik([*val]))
            self.odd_pattern[idx[2]] = np.array(self.delta.ik([*val]))
            self.odd_pattern[idx[3]] = np.array(self.delta.ik([*val]))
        elif partial == 1:
            self.even_pattern[idx[1]] = np.array(self.delta.ik([*val]))
            self.even_pattern[idx[2]] = np.array(self.delta.ik([*val]))

            self.odd_pattern[idx[1]] = np.array(self.delta.ik([*val]))
            self.odd_pattern[idx[2]] = np.array(self.delta.ik([*val]))
            self.set_wall([idx[3], idx[0]], wall_ht=0.11)
        elif partial == 2:
            self.even_pattern[idx[0]] = np.array(self.delta.ik([*val]))
            self.even_pattern[idx[3]] = np.array(self.delta.ik([*val]))

            self.odd_pattern[idx[0]] = np.array(self.delta.ik([*val]))
            self.odd_pattern[idx[3]] = np.array(self.delta.ik([*val]))
            self.set_wall([idx[2], idx[1]], wall_ht=0.11)
        return

    def grip_pattern(self, zmax=0.095, zmin=0.06):
        self.zmax = zmax
        self.zmin = zmin
        # All offsets in meters.
        xval = 0.005
        yval = 0.0
        grip_idxs_l = []
        grip_idxs_r = []
        for i in self.robo_dict.keys():
            idx = self.robo_dict[i]
            if i in [1, 2, 3, 4]:
                grip_idxs_l.append(idx)
                self.set_pattern_for_gripper(idx, [xval * 3, yval * 3, self.zmax], partial=1)
            elif i in [9, 10, 11, 12]:
                grip_idxs_r.append(idx)
                self.set_pattern_for_gripper(idx, [-xval * 3, -yval * 3, self.zmax], partial=2)
            elif i in [13, 14, 15, 16]:
                self.set_pattern_for_gripper(idx, [-xval * 3, -yval * 3, self.zmax])
            else:
                self.set_pattern_for_gripper(idx, [0, 0, self.zmin])

        yval = 0.001
        val1_0 = [-xval * 3, yval, self.zmax]
        val1_1 = [xval, -yval, self.zmax]
        val2_0 = [-xval, yval, self.zmax]
        val2_1 = [xval * 3, -yval, self.zmax]
        for idx in grip_idxs_l:
            self.even_pattern[idx[0]] = np.array(self.delta.ik([*val1_0]))
            self.even_pattern[idx[3]] = np.array(self.delta.ik([*val1_1]))

            self.odd_pattern[idx[0]] = np.array(self.delta.ik([*val1_1]))
            self.odd_pattern[idx[3]] = np.array(self.delta.ik([*val1_0]))

        for idx in grip_idxs_r:
            self.even_pattern[idx[1]] = np.array(self.delta.ik([*val2_0]))
            self.even_pattern[idx[2]] = np.array(self.delta.ik([*val2_1]))

            self.odd_pattern[idx[1]] = np.array(self.delta.ik([*val2_1]))
            self.odd_pattern[idx[2]] = np.array(self.delta.ik([*val2_0]))


if __name__ == "__main__":
    RC = RoboCoords()
    vec = RC.get_dist_vec((0.162378, 0.13125), norm=True, plot=True)
    RC.rotate(vec, np.pi, plot=True)
