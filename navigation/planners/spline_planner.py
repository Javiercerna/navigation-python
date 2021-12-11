import numpy as np
from scipy import interpolate

from navigation.planners.base import Planner
from navigation.utils import Waypoint, calculate_ds


class SplinePlanner(Planner):
    def __init__(self, interpolation_distance: float) -> None:
        self.interpolation_distance = interpolation_distance

    def calculate_reference(self, waypoints: list[Waypoint]) -> np.ndarray:
        waypoints_x = [waypoint.x for waypoint in waypoints]
        waypoints_y = [waypoint.y for waypoint in waypoints]

        ds = calculate_ds(waypoints_x, waypoints_y)
        accumulated_ds = np.cumsum(ds) - ds[0]
        interpolation_ds = np.arange(0, accumulated_ds[-1], self.interpolation_distance)

        interpolated_x = interpolate.interp1d(accumulated_ds, waypoints_x)(interpolation_ds)
        interpolated_y = interpolate.interp1d(accumulated_ds, waypoints_y)(interpolation_ds)

        return np.array([[x, y] for x, y in zip(interpolated_x, interpolated_y)])
