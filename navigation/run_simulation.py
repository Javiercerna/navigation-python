from navigation.simulation import (
    create_simulation,
    LATERAL_CONTROLLER,
    LONGITUDINAL_CONTROLLER,
    PLANNER,
)
from navigation.visualization import Plot


if __name__ == '__main__':
    simulation_options = {
        PLANNER: 'FixedReferencePlanner (reference2.txt)',
        LATERAL_CONTROLLER: 'PurePursuit',
        LONGITUDINAL_CONTROLLER: 'FixedLinearVelocityController (5 m/s)',
    }

    simulation = create_simulation(simulation_options)
    plot = Plot()
    plot.attach_simulation(simulation)

    while True:
        simulation.update()
        plot.show()
