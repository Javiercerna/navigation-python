import re

from navigation.controllers import (
    DecoupledController,
    FixedLinearVelocityController,
    PurePursuit,
)
from navigation.models import KinematicBicycleModel
from navigation.planners import (
    FixedReferencePlanner,
    SplinePlanner,
)
from navigation.simulation.options import (
    LATERAL_CONTROLLER,
    LONGITUDINAL_CONTROLLER,
    PLANNER,
    WAYPOINTS,
)
from navigation.simulation.simulation import Simulation
from navigation.utils import (
    load_parameters,
    load_reference,
    load_waypoints,
    State,
)
from navigation.vehicle import Vehicle


def create_simulation(simulation_options: dict):
    parameters = load_parameters('parameters.yml')

    vehicle = Vehicle(
        initial_state=State(**parameters['initial_state']),
        model=KinematicBicycleModel,
        input_limits=parameters['input_limits'],
        dimensions=parameters['vehicle_dimensions'],
    )

    waypoints = (
        load_waypoints(simulation_options[WAYPOINTS])
        if WAYPOINTS in simulation_options
        else []
    )
    planner = create_planner(simulation_options[PLANNER], parameters)
    controller = create_decoupled_controller(
        simulation_options[LATERAL_CONTROLLER],
        simulation_options[LONGITUDINAL_CONTROLLER],
        parameters,
    )

    return Simulation(
        vehicle=vehicle,
        controller=controller,
        planner=planner,
        waypoints=waypoints,
        options=parameters['simulation_options'],
    )


def create_planner(planner_name: str, parameters: dict):
    if 'FixedReferencePlanner' in planner_name:
        reference_filename = re.search('\((.*)\)', planner_name).group(1)
        return FixedReferencePlanner(load_reference(reference_filename))
    elif planner_name == 'SplinePlanner':
        return SplinePlanner(**parameters['spline_planner'])


def create_decoupled_controller(
        lateral_controller_name: str,
        longitudinal_controller_name: str,
        parameters: dict,
    ) -> None:
    if lateral_controller_name == 'PurePursuit':
        lateral_controller = PurePursuit(**parameters['pure_pursuit'])

    if 'FixedLinearVelocityController' in longitudinal_controller_name:
        linear_velocity = float(
            re.search('\((.*) m/s\)', longitudinal_controller_name).group(1)
        )
        longitudinal_controller = FixedLinearVelocityController(linear_velocity)

    return DecoupledController(lateral_controller, longitudinal_controller)
