import math
from collections import deque
from copy import deepcopy

import numpy as np

from .a_star import AStar


class HybridAStar(AStar):
    def __init__(self, start_node, goal_node, map, vehicle):
        super(HybridAStar, self).__init__(start_node, goal_node, map)

        self.vehicle = vehicle

    def _compute_node_neighbors(self, node):
        neighbors = []

        max_steering_angle = self.vehicle.MAX_STEERING_ANGLE

        velocity = 1

        for steering_angle in [-max_steering_angle, 0, max_steering_angle]:
            self.vehicle.state = np.array(node)

            for _ in range(int(1 / self.vehicle.dt)):
                self.vehicle.send_commands(velocity, steering_angle)

            neighbor_x, neighbor_y, neighbor_orientation = self.vehicle.state

            neighbor_x = round(neighbor_x, 3)
            neighbor_y = round(neighbor_y, 3)
            neighbor_orientation = round(neighbor_orientation, 3)

            closest_node = self.map.get_closest_cell((neighbor_x, neighbor_y))

            if not self.map.is_node_within_map(closest_node):
                continue

            grid_row = closest_node[0] // self.map.resolution
            grid_col = closest_node[1] // self.map.resolution

            if self.map.grid_data[grid_row, grid_col] != self.map.EMPTY:
                continue

            neighbors.append((neighbor_x, neighbor_y, neighbor_orientation))

        return neighbors
