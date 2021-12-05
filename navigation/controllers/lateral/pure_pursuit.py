import math
from typing import Union

import numpy as np

from navigation.controllers.base import LateralController
from navigation.models import KinematicBicycleModel
from navigation.vehicle import State


class PurePursuit(LateralController):
    def __init__(self, lookahead_distance: float) -> None:
        self.lookahead_distance = lookahead_distance

    def calculate_steering_angle(
            self,
            vehicle_state: State,
            reference: np.ndarray,
            wheelbase: float,
        ) -> Union[float, None]:
        vehicle_xy = np.array([vehicle_state.x, vehicle_state.y])
        reference_xy = reference[:, 0:2]

        goal_point = self._find_goal_point(vehicle_xy, reference_xy)
        if goal_point is None:
            return None

        curvature = self._calculate_curvature(vehicle_state, goal_point)
        return KinematicBicycleModel.curvature_to_steering_angle(curvature, wheelbase)

    def _find_goal_point(
            self,
            vehicle_xy: np.ndarray,
            reference_xy: np.ndarray,
        ) -> Union[np.ndarray, None]:
        distances_to_path = np.linalg.norm(reference_xy - vehicle_xy, axis=1)
        start_ind = np.argmin(distances_to_path)

        start_point = reference_xy[start_ind]
        for ind in range(start_ind, len(reference_xy)):
            distance = np.linalg.norm(reference_xy[ind] - start_point)
            if distance >= self.lookahead_distance:
                return reference_xy[ind]

        return None

    def _calculate_curvature(self, vehicle_state: State, goal_point: np.ndarray) -> float:
        x, y, theta = vehicle_state
        theta += math.pi / 2
        goal_x, goal_y = goal_point

        new_goal_x = (goal_x - x) * math.cos(theta) + (goal_y - y) * math.sin(theta)
        return 2 * new_goal_x / (self.lookahead_distance ** 2)
