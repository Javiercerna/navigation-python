import matplotlib.pyplot as plt
import numpy as np
from paths.path import Path

import math


class SpatialBicycleModel(object):
    def get_state(self, vehicle_pose, path):
        vehicle_position = vehicle_pose[0:2]

        ind_closest_point = self._find_ind_of_closest_point_on_path(
            vehicle_position, path)

        path_point = path[ind_closest_point]
        path_point_next = path[(ind_closest_point + 1) % len(path)]
        path_angle = self._calculate_path_angle(path_point, path_point_next)

        e_y = self._calculate_e_y(
            path_point, path_point_next, vehicle_position)
        e_psi = self._normalize_angle(vehicle_pose[2] - path_angle)

        return np.array([e_y, e_psi])

    def get_linearized_matrices(self, vehicle_pose, path, path_curvatures):
        vehicle_position = vehicle_pose[0:2]

        ind_closest_point = self._find_ind_of_closest_point_on_path(
            vehicle_position, path)

        path_curvature = path_curvatures[ind_closest_point]

        path_point = path[ind_closest_point]
        path_point_next = path[(ind_closest_point + 1) % len(path)]
        ds = np.linalg.norm(path_point_next - path_point)

        A = np.array([[1, ds],
                      [(-path_curvature ** 2) * ds, 1]])

        B = np.array([[0], [ds]])

        return A, B, path_curvature

    def _find_ind_of_closest_point_on_path(self, vehicle_position, path):
        vehicle_position = np.tile(vehicle_position, reps=(len(path), 1))

        distances = path - vehicle_position
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


if __name__ == '__main__':
    waypoints_x = [x for x in range(0, 5)]
    waypoints_y = [x**2 for x in range(0, 5)]

    print(waypoints_x, waypoints_y)

    path = Path(waypoints_x=waypoints_x, waypoints_y=waypoints_y)

    path_curvatures = path.path_curvature

    vehicle_pose = np.array([3, 4, math.atan(4) + 2*math.pi])

    v = 1
    dt = 0.1
    spatial_bicycle_model = SpatialBicycleModel(ds=v*dt)

    state = spatial_bicycle_model.get_state(vehicle_pose, path.as_array())

    results = 'Lateral deviation(e_y): {: .3f} m \n' + \
        'Angle deviation(e_psi): {: 3f} rad'

    print(results.format(state[0], state[1]))

    print(spatial_bicycle_model.get_linearized_matrices(
        vehicle_pose, path.as_array(), path_curvatures))

    plt.plot(path.as_array()[:, 0], path.as_array()[:, 1])
    plt.plot(vehicle_pose[0], vehicle_pose[1], 'go')
    plt.show()
