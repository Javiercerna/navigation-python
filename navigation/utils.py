import yaml

import numpy as np


def load_parameters(filename: str) -> dict:
    with open(filename, 'r') as f:
        return yaml.load(f, Loader=yaml.SafeLoader)


def load_reference(reference_folder: str) -> np.ndarray:
    reference = {}

    for reference_name in ['x', 'y']:
        with open(f'{reference_folder}/{reference_name}.txt', 'r') as f:
            reference[reference_name] = [float(line.rstrip('\n')) for line in f]

    return np.array([[x, y] for x, y in zip(reference['x'], reference['y'])])
