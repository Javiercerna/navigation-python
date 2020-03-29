import numpy as np

import math


class SpatialBicycleModel(object):
    def __init__(self, path, vehicle_pose):
        self.update(path, vehicle_pose)

    def update(self, path, vehicle_pose):
        self.path_xy = path[:, 0:2]
        self.path_curvatures = path[:, 2]
        self.vehicle_pose = vehicle_pose

        self.path_index = self._calculate_closest_point_on_path_index()

    def calculate_spatial_state(self):
        e_y = self._calculate_e_y()
        e_psi = self._calculate_e_psi()

        return np.array([e_y, e_psi])

    def calculate_predicted_poses(self, spatial_state, prediction_horizon):
        predicted_poses = []

        for horizon_step in range(prediction_horizon):
            predicted_pose = self._calculate_pose(spatial_state, horizon_step)
            predicted_poses.append(predicted_pose)

        return np.array(predicted_poses)

    def _calculate_pose(self, spatial_state, horizon_step=0):
        path_index = (self.path_index + horizon_step) % len(self.path_xy)

        path_point = self.path_xy[path_index]
        path_point_next = self.path_xy[(path_index + 1) % len(self.path_xy)]

        path_angle = self._calculate_path_angle(path_point, path_point_next)

        e_y, e_psi = spatial_state

        x = path_point[0] - e_y * np.sin(path_angle)
        y = path_point[1] + e_y * np.cos(path_angle)
        angle = self._normalize_angle(e_psi + path_angle)

        return np.array([x, y, angle])

    def calculate_linearized_matrices(self, horizon_step=0):
        path_index = (self.path_index + horizon_step) % len(self.path_xy)

        path_point = self.path_xy[path_index]
        path_point_next = self.path_xy[(path_index + 1) % len(self.path_xy)]

        ds = np.linalg.norm(path_point_next - path_point)
        path_curvature = self.path_curvatures[path_index]

        A_linearized = np.array([[1, ds],
                                 [(-path_curvature ** 2) * ds, 1]])

        B_linearized = np.array([[0], [ds]])

        return A_linearized, B_linearized

    def get_path_curvature(self, horizon_step=0):
        path_index = (self.path_index + horizon_step) % len(self.path_xy)

        return self.path_curvatures[path_index]

    def _calculate_closest_point_on_path_index(self):
        vehicle_xy = self.vehicle_pose[0:2]
        distances_to_path = np.linalg.norm(self.path_xy - vehicle_xy, axis=1)

        return np.argmin(distances_to_path)

    def _calculate_path_angle(self, path_point, path_point_next):
        dx = path_point_next[0] - path_point[0]
        dy = path_point_next[1] - path_point[1]

        return np.arctan2(dy, dx)

    def _calculate_e_y(self):
        vehicle_xy = self.vehicle_pose[0:2]

        path_point = self.path_xy[self.path_index]
        path_point_next = self.path_xy[
            (self.path_index + 1) % len(self.path_xy)
        ]

        dx = vehicle_xy[0] - path_point[0]
        dy = vehicle_xy[1] - path_point[1]

        path_angle = self._calculate_path_angle(path_point, path_point_next)

        return -np.sin(path_angle) * dx + np.cos(path_angle) * dy

    def _calculate_e_psi(self):
        path_point = self.path_xy[self.path_index]
        path_point_next = self.path_xy[
            (self.path_index + 1) % len(self.path_xy)
        ]

        path_angle = self._calculate_path_angle(path_point, path_point_next)

        return self._normalize_angle(self.vehicle_pose[2] - path_angle)

    def _normalize_angle(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi
