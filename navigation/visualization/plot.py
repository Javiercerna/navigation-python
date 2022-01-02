import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from tkinter import ttk

from navigation.simulation.simulation import Simulation


class Plot:
    def __init__(self) -> None:
        self.figure = plt.figure()
        self.simulation = None

    def attach_simulation(self, simulation: Simulation) -> None:
        self.simulation = simulation

    def show(self, skip_render: bool = False) -> None:
        plt.clf()

        self.show_waypoints()
        self.show_reference()
        self.show_vehicle_trajectory()

        if skip_render:
            return

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


class CanvasPlot(Plot):
    def __init__(self, frame: ttk.Frame, row: int, column: int) -> None:
        super().__init__()
        self.canvas = FigureCanvasTkAgg(self.figure, frame)
        self.canvas.get_tk_widget().grid(row=row, column=column)

    def show(self) -> None:
        super().show(skip_render=True)
        self.canvas.draw()
