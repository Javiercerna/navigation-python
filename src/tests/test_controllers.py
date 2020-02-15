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
    path_curvatures = path.path_curvature

    spatial_bicycle_model = SpatialBicycleModel(ds=0.1)

    Q = sparse.diags([1., 1.])
    Qn = Q
    R = 0.1*sparse.eye(1)
    prediction_horizon = 2
    steering_angle_min = -math.pi/2
    steering_angle_max = math.pi/2

    mpc = MPC(Q, R, Qn, prediction_horizon,
              steering_angle_min, steering_angle_max)

    A, B, reference_curvature = spatial_bicycle_model.get_linearized_matrices(
        vehicle_pose, path.as_array(), path_curvatures)

    state = spatial_bicycle_model.get_state(vehicle_pose, path.as_array())

    print(state)
    mpc_result = mpc.compute_steering_angle(A, B, state)

    _, nu = B.shape
    k_tilde = mpc_result.x[-prediction_horizon*nu:-(prediction_horizon-1)*nu]
    curvature = k_tilde + reference_curvature

    wheelbase = 1
    steering_angle = float(np.arctan2(curvature * wheelbase, 1))

    assert steering_angle == result['steering_angle']
