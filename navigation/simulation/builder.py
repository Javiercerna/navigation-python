from navigation.controllers.base import DecoupledController
from navigation.controllers.lateral.pure_pursuit import PurePursuit
from navigation.controllers.longitudinal.fixed_velocity import FixedLinearVelocityController
from navigation.models import KinematicBicycleModel
from navigation.planners.fixed_reference import FixedReferencePlanner
from navigation.planners.spline_planner import SplinePlanner
from navigation.simulation.simulation import Simulation
from navigation.utils import load_parameters, load_reference, load_waypoints, State
from navigation.vehicle import Vehicle


def create_simulation(simulation_options: dict):
    parameters = load_parameters('parameters.yml')

    vehicle = Vehicle(
        initial_state=State(**parameters['initial_state']),
        model=KinematicBicycleModel,
        input_limits=parameters['input_limits'],
        dimensions=parameters['vehicle_dimensions'],
    )

    planner = create_planner(simulation_options['Planner'], parameters)
    controller = create_decoupled_controller(
        simulation_options['Lateral Controller'],
        simulation_options['Longitudinal Controller'],
        parameters,
    )

    return Simulation(
        vehicle=vehicle,
        controller=controller,
        planner=planner,
        waypoints=load_waypoints(parameters['waypoints_filename']),
        options=parameters['simulation_options'],
    )


def create_planner(planner_name: str, parameters: dict):
    if planner_name == 'FixedReferencePlanner':
        return FixedReferencePlanner(load_reference(parameters['reference_folder']))
    elif planner_name == 'SplinePlanner':
        return SplinePlanner(**parameters['spline_planner'])


def create_decoupled_controller(
        lateral_controller_name: str,
        longitudinal_controller_name: str,
        parameters: dict,
    ) -> None:
    if lateral_controller_name == 'PurePursuit':
        lateral_controller = PurePursuit(**parameters['pure_pursuit'])

    if longitudinal_controller_name == 'FixedLinearVelocityController':
        longitudinal_controller = FixedLinearVelocityController(
            parameters['fixed_linear_velocity']
        )

    return DecoupledController(lateral_controller, longitudinal_controller)
