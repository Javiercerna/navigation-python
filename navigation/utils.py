from collections import namedtuple

import numpy as np
import os
import yaml


Waypoint = namedtuple('Waypoint', ['x', 'y'])
State = namedtuple('State', ['x', 'y', 'theta'])

REFERENCES_FOLDER = 'data/references'
WAYPOINTS_FOLDER = 'data/waypoints'


def load_parameters(filename: str) -> dict:
    with open(filename, 'r') as f:
        return yaml.load(f, Loader=yaml.SafeLoader)


def load_reference(reference_filename: str) -> np.ndarray:
    return np.loadtxt(os.path.join(REFERENCES_FOLDER, reference_filename))


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


def get_reference_filenames() -> list[str]:
    return os.listdir(os.path.join(get_project_folder(), REFERENCES_FOLDER))


def calculate_ds(waypoints_x, waypoints_y):
    dx_dt = np.gradient(waypoints_x)
    dy_dt = np.gradient(waypoints_y)

    return np.around(np.sqrt(dx_dt * dx_dt + dy_dt * dy_dt), 2)
