from controllers.pure_pursuit import PurePursuit
from controllers.mpc import MPC
from models.spatial_bicycle_model import SpatialBicycleModel
from paths.path import Path
import matplotlib.pyplot as plt
import numpy as np
import pytest
from scipy import sparse

import math


@pytest.mark.parametrize('vehicle, result', [
    ({'x': 0, 'y': 0, 'theta': math.pi/2}, {'steering_angle': 0})
])
def test_pure_pursuit(path_array, vehicle, result, plot):
    vehicle_pose = np.array([vehicle['x'], vehicle['y'], vehicle['theta']])
    pure_pursuit = PurePursuit(wheelbase=1, lookahead_distance=1)

    steering_angle = pure_pursuit.compute_steering_angle(
        vehicle_pose, path_array)

    if plot:
        plt.plot(path_array[:, 0], path_array[:, 1])
        plt.plot(vehicle_pose[0], vehicle_pose[1], 'go')

        plt.axhline(0, color='black', alpha=0.1)
        plt.axvline(0, color='black', alpha=0.1)
        plt.show()

    assert abs(steering_angle - result['steering_angle']) <= 1e-15


@pytest.mark.parametrize('vehicle, result', [
    ({'x': 0, 'y': 0, 'theta': math.pi/2}, {'steering_angle': 0})
])
def test_mpc(waypoints, vehicle, result, plot):
    vehicle_pose = np.array([vehicle['x'], vehicle['y'], vehicle['theta']])

    waypoints_x, waypoints_y = waypoints
    path = Path(waypoints_x, waypoints_y)

    spatial_bicycle_model = SpatialBicycleModel()

    Q = sparse.diags([1., 1.])
    Qn = Q
    R = 0.1*sparse.eye(1)
    prediction_horizon = 2
    kappa_tilde_min = -100
    kappa_tilde_max = 100
    wheelbase = 1

    mpc = MPC(Q, R, Qn, prediction_horizon,
              kappa_tilde_min, kappa_tilde_max, wheelbase)

    steering_angle = mpc.compute_steering_angle(
        spatial_bicycle_model, vehicle_pose, path)

    assert steering_angle == result['steering_angle']
