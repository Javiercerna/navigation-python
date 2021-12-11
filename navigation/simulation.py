from typing import Union

import matplotlib.pyplot as plt
import numpy as np

from navigation.controllers.base import Controller
from navigation.planners.base import Planner
from navigation.utils import Waypoint
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

        self._vehicle_trajectory = []
        self._figure = plt.figure(1)

    def update(self) -> None:
        self._vehicle_trajectory.append(self.vehicle.state)

    def show(self, reference: np.ndarray) -> None:
        plt.clf()

        self.show_waypoints()
        self.show_reference(reference)
        self.show_vehicle_trajectory()

        plt.pause(0.01)

    def show_waypoints(self) -> None:
        if not self.waypoints:
            return

        plt.plot(
            [waypoint.x for waypoint in self.waypoints],
            [waypoint.y for waypoint in self.waypoints],
            'rx',
        )

    def show_reference(self, reference: np.ndarray) -> None:
        plt.plot(
            [coordinate[0] for coordinate in reference],
            [coordinate[1] for coordinate in reference],
        )

    def show_vehicle_trajectory(self) -> None:
        plt.plot(
            [state.x for state in self._vehicle_trajectory],
            [state.y for state in self._vehicle_trajectory],
        )

    def run(self) -> None:
        while True:
            try:
                if self.planner is None and self.fixed_reference is not None:
                    reference = self.fixed_reference
                else:
                    reference = self.planner.calculate_reference(self.waypoints)

                if reference is None:
                    continue

                linear_velocity = self.controller.calculate_linear_velocity()
                steering_angle = self.controller.calculate_steering_angle(
                    self.vehicle.state,
                    reference,
                    self.vehicle.dimensions['wheelbase'],
                )

                if None in [linear_velocity, steering_angle]:
                    continue

                self.vehicle.update_state(
                    linear_velocity,
                    steering_angle,
                    self.options['dt'],
                )

                self.update()
                self.show(reference)
            except KeyboardInterrupt:
                break
