from navigation.simulation.builder import create_simulation
from navigation.simulation.options import (
    LATERAL_CONTROLLER,
    LONGITUDINAL_CONTROLLER,
    PLANNER,
)
from navigation.visualization.plot import Plot


if __name__ == '__main__':
    simulation_options = {
        PLANNER: 'FixedReferencePlanner (reference2.txt)',
        LATERAL_CONTROLLER: 'PurePursuit',
        LONGITUDINAL_CONTROLLER: 'FixedLinearVelocityController',
    }

    simulation = create_simulation(simulation_options)
    plot = Plot()
    plot.attach_simulation(simulation)

    while True:
        simulation.update()
        plot.show()
