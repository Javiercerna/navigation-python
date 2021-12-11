from navigation.controllers.base import DecoupledController
from navigation.controllers.lateral.pure_pursuit import PurePursuit
from navigation.models import KinematicBicycleModel
from navigation.planners.spline_planner import SplinePlanner
from navigation.simulation import Simulation
from navigation.utils import (
    load_parameters,
    load_reference,
    load_waypoints,
    State,
)
from navigation.vehicle import Vehicle


if __name__ == '__main__':
    parameters = load_parameters('parameters.yml')

    vehicle = Vehicle(
        initial_state=State(**parameters['initial_state']),
        model=KinematicBicycleModel,
        input_limits=parameters['input_limits'],
        dimensions=parameters['vehicle_dimensions'],
    )

    controller = DecoupledController(
        lateral_controller=PurePursuit(**parameters['pure_pursuit']),
        longitudinal_controller=None,
        fixed_linear_velocity=parameters['fixed_linear_velocity'],
    )

    simulation = Simulation(
        vehicle=vehicle,
        controller=controller,
        waypoints=load_waypoints(parameters['waypoints_filename']),
        planner=SplinePlanner(**parameters['spline_planner']),
        options=parameters['simulation_options'],
        fixed_reference=load_reference(parameters['reference_folder']),
    )
    simulation.run()
