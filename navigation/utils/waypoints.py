import numpy as np


def calculate_ds(waypoints_x, waypoints_y):
    dx_dt = np.gradient(waypoints_x)
    dy_dt = np.gradient(waypoints_y)

    return np.around(np.sqrt(dx_dt * dx_dt + dy_dt * dy_dt), 2)
