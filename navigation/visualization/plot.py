import matplotlib.pyplot as plt

from navigation.simulation.simulation import Simulation


class Plot:
    def __init__(self, simulation: Simulation) -> None:
        self.simulation = simulation

    def show(self) -> None:
        plt.clf()

        self.show_waypoints()
        self.show_reference()
        self.show_vehicle_trajectory()

        plt.pause(0.01)

    def show_waypoints(self) -> None:
        if not self.simulation.waypoints:
            return

        plt.plot(
            [waypoint.x for waypoint in self.simulation.waypoints],
            [waypoint.y for waypoint in self.simulation.waypoints],
            'rx',
        )

    def show_reference(self) -> None:
        if len(self.simulation.reference) == 0:
            return

        plt.plot(self.simulation.reference[:, 0], self.simulation.reference[:, 1])

    def show_vehicle_trajectory(self) -> None:
        if not self.simulation.vehicle_trajectory:
            return

        plt.plot(
            [state.x for state in self.simulation.vehicle_trajectory],
            [state.y for state in self.simulation.vehicle_trajectory],
        )
