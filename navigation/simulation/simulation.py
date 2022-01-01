from typing import Union

import numpy as np

from navigation.controllers.base import Controller
from navigation.planners.base import Planner
from navigation.utils import State, Waypoint
from navigation.vehicle import Vehicle


class Simulation:
    def __init__(
            self,
            vehicle: Vehicle,
            controller: Controller,
            waypoints: list[Waypoint],
            planner: Union[Planner, None],
            options: dict,
            fixed_reference: Union[np.ndarray, None] = None,
        ) -> None:
        self.vehicle = vehicle
        self.controller = controller
        self.waypoints = waypoints
        self.planner = planner
        self.options = options
        self.fixed_reference = fixed_reference

        self.reference: Union[np.ndarray, None] = None
        self.vehicle_trajectory: list[State] = []

    def update(self) -> None:
        if self.planner is None and self.fixed_reference is not None:
            self.reference = self.fixed_reference
        else:
            self.reference = self.planner.calculate_reference(self.waypoints)

        if self.reference is None:
            return

        linear_velocity = self.controller.calculate_linear_velocity()
        steering_angle = self.controller.calculate_steering_angle(
            self.vehicle.state,
            self.reference,
            self.vehicle.dimensions['wheelbase'],
        )

        if None in [linear_velocity, steering_angle]:
            return

        self.vehicle.update_state(linear_velocity, steering_angle, self.options['dt'])

        self.vehicle_trajectory.append(self.vehicle.state)
