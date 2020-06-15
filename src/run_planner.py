import matplotlib.pyplot as plt

from maps.occupancy_grid import OccupancyGrid
from planners.a_star import AStar

width = 15
height = 15

grid = OccupancyGrid(width, height)

grid.grid_data[:, :] = 100

grid.grid_data[5:, 0] = 0
grid.grid_data[5, 4:8] = 0
grid.grid_data[2:6, 8] = 0
grid.grid_data[2, 2] = 0

start_node = (0, 0)
goal_node = (4, 9)

a_star = AStar(start_node, goal_node, map=grid.grid_data)

path = a_star.compute_path()

plt.plot([wp[0] for wp in path], [wp[1] for wp in path])
plt.plot(start_node[0], start_node[1], 'bx')
plt.plot(goal_node[0], goal_node[1], 'gx')

grid.show()
