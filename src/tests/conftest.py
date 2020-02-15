import numpy as np
import pytest


@pytest.fixture
def plot(request):
    return request.config.getoption("-v")


@pytest.fixture
def waypoints():
    waypoints_x = [0 for _ in range(0, 5)]
    waypoints_y = [x for x in range(0, 5)]
    return waypoints_x, waypoints_y


@pytest.fixture
def path_array(waypoints):
    waypoints_x, waypoints_y = waypoints
    path_array = np.array([np.array([waypoints_x[ind],
                                     waypoints_y[ind]])
                           for ind in range(len(waypoints_x))])

    return path_array
