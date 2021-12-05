import matplotlib.pyplot as plt
import numpy as np

from navigation.controllers.base import Controller
from navigation.vehicle import Vehicle


class Simulation:
    def __init__(
            self,
            vehicle: Vehicle,
            controller: Controller,
            reference: np.ndarray,
            options: dict
        ) -> None:
        self.vehicle = vehicle
        self.controller = controller
        self.reference = reference
        self.options = options

        self._vehicle_trajectory = []

    def update(self) -> None:
        self._vehicle_trajectory.append(self.vehicle.state)

    def show(self) -> None:
        plt.figure(1)
        plt.clf()

        self.show_reference()
        self.show_vehicle_trajectory()

        plt.pause(0.01)

    def show_reference(self) -> None:
        plt.plot(
            [coordinate[0] for coordinate in self.reference],
            [coordinate[1] for coordinate in self.reference],
        )

    def show_vehicle_trajectory(self) -> None:
        plt.plot(
            [state.x for state in self._vehicle_trajectory],
            [state.y for state in self._vehicle_trajectory],
        )

    def run(self) -> None:
        while True:
            try:
                linear_velocity = self.controller.calculate_linear_velocity()
                steering_angle = self.controller.calculate_steering_angle(
                    self.vehicle.state,
                    self.reference,
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
                self.show()
            except KeyboardInterrupt:
                break
