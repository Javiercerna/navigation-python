from models.spatial_bicycle_model import SpatialBicycleModel
import matplotlib.pyplot as plt
import numpy as np
import pytest

import math


@pytest.fixture
def path_array():
    waypoints_x = [0 for _ in range(0, 5)]
    waypoints_y = [x for x in range(0, 5)]
    path_array = np.array([np.array([waypoints_x[ind],
                                     waypoints_y[ind]])
                           for ind in range(len(waypoints_x))])

    return path_array


@pytest.mark.parametrize('vehicle, result', [
    ({'x': 0, 'y': 0, 'theta': math.pi/2}, {'e_y': 0, 'e_psi': 0}),
    ({'x': 1, 'y': 0, 'theta': math.pi/2}, {'e_y': 1, 'e_psi': 0}),
    ({'x': -1, 'y': 0, 'theta': math.pi/2}, {'e_y': -1, 'e_psi': 0}),
    ({'x': 100, 'y': 0, 'theta': 0}, {'e_y': 100, 'e_psi': -math.pi/2}),
    ({'x': -100, 'y': 0, 'theta': math.pi}, {'e_y': -100, 'e_psi': math.pi/2})
])
def test_vehicle_in_path(path_array, vehicle, result, plot):
    vehicle_pose = np.array([vehicle['x'], vehicle['y'], vehicle['theta']])

    spatial_bicycle_model = SpatialBicycleModel(ds=0.1)

    state = spatial_bicycle_model.get_state(vehicle_pose, path_array)

    assert state[0] == result['e_y']
    assert state[1] == result['e_psi']

    if plot:
        plt.plot(path_array[:, 0], path_array[:, 1])
        plt.plot(vehicle_pose[0], vehicle_pose[1], 'go')

        plt.axhline(0, color='black', alpha=0.1)
        plt.axvline(0, color='black', alpha=0.1)
        plt.show()
