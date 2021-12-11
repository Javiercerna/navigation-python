from collections import namedtuple

import numpy as np
import yaml


Waypoint = namedtuple('Waypoint', ['x', 'y'])
State = namedtuple('State', ['x', 'y', 'theta'])


def load_parameters(filename: str) -> dict:
    with open(filename, 'r') as f:
        return yaml.load(f, Loader=yaml.SafeLoader)


def load_reference(reference_folder: str) -> np.ndarray:
    reference = {}

    for reference_name in ['x', 'y']:
        with open(f'{reference_folder}/{reference_name}.txt', 'r') as f:
            reference[reference_name] = [float(line.rstrip('\n')) for line in f]

    return np.array([[x, y] for x, y in zip(reference['x'], reference['y'])])


def load_waypoints(waypoints_filename: str) -> list[Waypoint]:
    with open(waypoints_filename, 'r') as f:
        return [
            Waypoint(*[float(value) for value in line.rstrip('\n').split(',')])
            for line in f
        ]

def calculate_ds(waypoints_x, waypoints_y):
    dx_dt = np.gradient(waypoints_x)
    dy_dt = np.gradient(waypoints_y)

    return np.around(np.sqrt(dx_dt * dx_dt + dy_dt * dy_dt), 2)
