from navigation.simulation.builder import create_simulation
from navigation.visualization.plot import Plot


if __name__ == '__main__':
    simulation = create_simulation()
    plot = Plot(simulation)

    while True:
        simulation.update()
        plot.show()
