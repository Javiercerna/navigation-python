import math

import matplotlib.pyplot as plt
import numpy as np

from maps.occupancy_grid import OccupancyGrid
from models.vehicle import Vehicle
from planners.a_star import AStar
from planners.hybrid_a_star import HybridAStar


width = 15
height = 15

map = OccupancyGrid(width, height, resolution=1)

# Example map representation
map.grid_data[:, :] = OccupancyGrid.EMPTY
map.grid_data[5:, 0] = OccupancyGrid.FULL
map.grid_data[5, 4:8] = OccupancyGrid.FULL
map.grid_data[2:6, 8] = OccupancyGrid.FULL
map.grid_data[2, 2] = OccupancyGrid.FULL

start_node = (0, 0, round(math.pi / 2, 3))
goal_node = (10.5, 10.5, round(math.pi / 2, 3))

wheelbase = 1
dt = 0.01
max_velocity = 5
max_steering_angle = math.pi / 10

vehicle = Vehicle(
    wheelbase=wheelbase, initial_state=np.array(start_node),
    dt=dt, max_velocity=max_velocity, max_steering_angle=max_steering_angle
)

a_star = HybridAStar(start_node, goal_node, map, vehicle)

path = a_star.compute_path()

plt.plot([wp[0] for wp in path], [wp[1] for wp in path])
plt.plot(start_node[0], start_node[1], 'bx')
plt.plot(goal_node[0], goal_node[1], 'gx')

if len(path) == 0:
    print('A* algorithm didn\'t find a path...')

map.show()
