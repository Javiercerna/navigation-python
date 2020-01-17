import numpy as np
import matplotlib.pyplot as plt

import json


class Path(object):
    def __init__(self, waypoints_x, waypoints_y):
        assert len(waypoints_x) == len(
            waypoints_y), 'waypoints must have same len x and y'

        assert len(waypoints_x) > 1, 'there must be more than 1 waypoint'

        self.waypoints_x = waypoints_x
        self.waypoints_y = waypoints_y

        self._construct_path()
        self.path_curvature = self._compute_path_curvature()

    def as_array(self):
        path_array = np.zeros((len(self.path_x), 2))

        for ind in range(len(self.path_x)):
            path_array[ind] = np.array([self.path_x[ind], self.path_y[ind]])

        return path_array

    def _construct_path(self):
        self.path_x = []
        self.path_y = []

        for ind in range(-1, len(self.waypoints_x) - 1):
            self.path_x += np.linspace(self.waypoints_x[ind],
                                       self.waypoints_x[ind + 1]).tolist()
            self.path_y += np.linspace(self.waypoints_y[ind],
                                       self.waypoints_y[ind + 1]).tolist()

    def _compute_path_curvature(self):
        dx_dt = np.gradient(np.array(self.path_x))
        dy_dt = np.gradient(np.array(self.path_y))

        d2x_dt2 = np.gradient(dx_dt)
        d2y_dt2 = np.gradient(dy_dt)

        curvature = d2x_dt2 * dy_dt - dx_dt * d2y_dt2
        curvature /= (dx_dt * dx_dt + dy_dt * dy_dt) ** 1.5

        return curvature


if __name__ == '__main__':
    with open('simulated_waypoints.json') as f:
        waypoints = json.load(f)
        waypoints_x = waypoints['waypoints_x']
        waypoints_y = waypoints['waypoints_y']

    path = Path(waypoints_x=waypoints_x, waypoints_y=waypoints_y)

    plt.plot(path.path_x, path.path_y)
    plt.show()
