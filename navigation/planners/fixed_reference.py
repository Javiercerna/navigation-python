import numpy as np

from navigation.planners.base import Planner
from navigation.utils import Waypoint


class FixedReferencePlanner(Planner):
    def __init__(self, fixed_reference: np.ndarray) -> None:
        self.fixed_reference = fixed_reference

    def calculate_reference(self, waypoints: list[Waypoint]) -> np.ndarray:
        return self.fixed_reference
