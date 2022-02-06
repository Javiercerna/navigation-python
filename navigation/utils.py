from collections import namedtuple

import numpy as np
import os
import yaml


Waypoint = namedtuple('Waypoint', ['x', 'y'])
State = namedtuple('State', ['x', 'y', 'theta'])

WAYPOINTS_FOLDER = 'data/waypoints'


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
    with open(f'{WAYPOINTS_FOLDER}/{waypoints_filename}', 'r') as f:
        return [
            Waypoint(*[float(value) for value in line.rstrip('\n').split(',')])
            for line in f
        ]


def get_project_folder() -> str:
    return os.path.dirname(os.path.dirname(os.path.realpath(__file__)))


def get_waypoint_filenames() -> list[str]:
    return os.listdir(os.path.join(get_project_folder(), WAYPOINTS_FOLDER))


def calculate_ds(waypoints_x, waypoints_y):
    dx_dt = np.gradient(waypoints_x)
    dy_dt = np.gradient(waypoints_y)

    return np.around(np.sqrt(dx_dt * dx_dt + dy_dt * dy_dt), 2)
