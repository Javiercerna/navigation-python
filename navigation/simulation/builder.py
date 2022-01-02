from navigation.controllers.base import DecoupledController
from navigation.controllers.lateral.pure_pursuit import PurePursuit
from navigation.controllers.longitudinal.fixed_velocity import FixedLinearVelocityController
from navigation.models import KinematicBicycleModel
from navigation.planners.spline_planner import SplinePlanner
from navigation.simulation.simulation import Simulation
from navigation.utils import load_parameters, load_waypoints, State
from navigation.vehicle import Vehicle


def create_simulation():
    parameters = load_parameters('parameters.yml')

    vehicle = Vehicle(
        initial_state=State(**parameters['initial_state']),
        model=KinematicBicycleModel,
        input_limits=parameters['input_limits'],
        dimensions=parameters['vehicle_dimensions'],
    )

    planner = SplinePlanner(**parameters['spline_planner'])

    controller = DecoupledController(
        lateral_controller=PurePursuit(**parameters['pure_pursuit']),
        longitudinal_controller=FixedLinearVelocityController(parameters['fixed_linear_velocity']),
    )

    return Simulation(
        vehicle=vehicle,
        controller=controller,
        planner=planner,
        waypoints=load_waypoints(parameters['waypoints_filename']),
        options=parameters['simulation_options'],
    )
