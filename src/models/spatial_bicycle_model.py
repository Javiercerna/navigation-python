import matplotlib.pyplot as plt
import numpy as np
from paths.path import Path

import math


class SpatialBicycleModel(object):
    def get_state(self, vehicle_pose, path_array):
        vehicle_position = vehicle_pose[0:2]
        path_xy = np.array([waypoint[0:2] for waypoint in path_array])

        ind_closest_point = self._find_ind_of_closest_point_on_path(
            vehicle_position, path_xy)

        path_point = path_xy[ind_closest_point]
        path_point_next = path_xy[(ind_closest_point + 1) % len(path_xy)]
        path_angle = self._calculate_path_angle(path_point, path_point_next)

        e_y = self._calculate_e_y(
            path_point, path_point_next, vehicle_position)
        e_psi = self._normalize_angle(vehicle_pose[2] - path_angle)

        return np.array([e_y, e_psi])

    def get_linearized_matrices(self, vehicle_pose, path_array):
        vehicle_position = vehicle_pose[0:2]
        path_xy = np.array([waypoint[0:2] for waypoint in path_array])
        path_curvatures = np.array([waypoint[2] for waypoint in path_array])

        ind_closest_point = self._find_ind_of_closest_point_on_path(
            vehicle_position, path_xy)

        path_curvature = path_curvatures[ind_closest_point]

        path_point = path_xy[ind_closest_point]
        path_point_next = path_xy[(ind_closest_point + 1) % len(path_xy)]
        ds = np.linalg.norm(path_point_next - path_point)

        A = np.array([[1, ds],
                      [(-path_curvature ** 2) * ds, 1]])

        B = np.array([[0], [ds]])

        return A, B, path_curvature

    def _find_ind_of_closest_point_on_path(self, vehicle_position, path_xy):
        vehicle_position = np.tile(vehicle_position, reps=(len(path_xy), 1))

        distances = path_xy - vehicle_position
        distances_to_path = np.linalg.norm(
            distances.astype(np.float64), axis=1)

        return np.argmin(distances_to_path)

    def _calculate_path_angle(self, path_point, path_point_next):
        dx = path_point_next[0] - path_point[0]
        dy = path_point_next[1] - path_point[1]

        return np.arctan2(dy, dx)

    def _calculate_e_y(self, path_point, path_point_next, vehicle_position):
        dx = vehicle_position[0] - path_point[0]
        dy = vehicle_position[1] - path_point[1]
        path_angle = self._calculate_path_angle(path_point, path_point_next)

        return -np.sin(path_angle) * dx + np.cos(path_angle)*dy

    def _normalize_angle(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi
