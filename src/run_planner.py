import matplotlib.pyplot as plt

from maps.occupancy_grid import OccupancyGrid
from planners.a_star import AStar

width = 15
height = 15

map = OccupancyGrid(width, height, resolution=10)

# Example map representation
map.grid_data[:, :] = OccupancyGrid.EMPTY
map.grid_data[5:, 0] = OccupancyGrid.FULL
map.grid_data[5, 4:8] = OccupancyGrid.FULL
map.grid_data[2:6, 8] = OccupancyGrid.FULL
map.grid_data[2, 2] = OccupancyGrid.FULL

start_node = (0, 0)
goal_node = (105, 105)

a_star = AStar(start_node, goal_node, map=map)

path = a_star.compute_path()

plt.plot([wp[0] for wp in path], [wp[1] for wp in path])
plt.plot(start_node[0], start_node[1], 'bx')
plt.plot(goal_node[0], goal_node[1], 'gx')

if len(path) == 0:
    print('A* algorithm didn\'t find a path...')

map.show()
