import matplotlib.pyplot as plt
import numpy as np

import math


class SpatialBicycleModel(object):
    def __init__(self):
        pass

    def get_state(self, vehicle_pose, path):
        vehicle_position = vehicle_pose[0:2]

        ind_closest_point = self._find_ind_of_closest_point_on_path(
            vehicle_position, path)

        path_point = path[ind_closest_point]
        path_angle = self._calculate_path_angle(path, ind_closest_point)

        e_y = np.linalg.norm(path_point - vehicle_position)
        e_phi = self._normalize_angle(vehicle_pose[2] - path_angle)

        return np.array([e_y, e_phi])

    def _find_ind_of_closest_point_on_path(self, vehicle_position, path):
        vehicle_position = np.tile(vehicle_position, reps=(len(path), 1))

        distances_to_path = np.linalg.norm(path - vehicle_position, axis=1)
        print(distances_to_path)

        return np.argmin(distances_to_path)

    def _calculate_path_angle(self, path, ind_closest_point):
        dx_dt = np.gradient(path[:, 0])
        dy_dt = np.gradient(path[:, 1])

        return np.arctan2(dy_dt[ind_closest_point], dx_dt[ind_closest_point])

    def _normalize_angle(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi


if __name__ == '__main__':
    path = np.array([[x, x**2] for x in range(0, 5)])
    vehicle_pose = np.array([3, 4, math.atan(4) + 2*math.pi])

    spatial_bicycle_model = SpatialBicycleModel()

    state = spatial_bicycle_model.get_state(vehicle_pose, path)

    results = 'Lateral deviation(e_y): {: .3f} m \n' + \
        'Angle deviation(e_psi): {: 3f} rad'

    print(results.format(state[0], state[1]))

    plt.plot(path[:, 0], path[:, 1])
    plt.plot(vehicle_pose[0], vehicle_pose[1], 'go')
    plt.show()
