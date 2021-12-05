from navigation.controllers.base import DecoupledController
from navigation.controllers.lateral.pure_pursuit import PurePursuit
from navigation.models import KinematicBicycleModel
from navigation.simulation import Simulation
from navigation.utils import load_parameters, load_reference
from navigation.vehicle import State, Vehicle


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
        reference=load_reference(parameters['reference_folder']),
        options=parameters['simulation_options'],
    )
    simulation.run()
