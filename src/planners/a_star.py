from collections import deque

import numpy as np


class AStar(object):
    COST_BETWEEN_NODES = 1
    DISTANCE_BETWEEN_NODES = 1

    def __init__(self, start_node, goal_node, map=None):
        self.start_node = start_node
        self.goal_node = goal_node
        self.map = map

        self.path_cost = {start_node: 0}
        self.total_cost = {}

        self._update_node_total_cost(start_node)

        self.nodes_to_explore = {
            start_node: self.total_cost[start_node]
        }

    def compute_path(self):
        came_from = {}

        while len(self.nodes_to_explore) != 0:
            best_node = self._find_node_with_min_cost()

            if best_node == self.goal_node:
                return self._reconstruct_path(came_from)

            self.nodes_to_explore.pop(best_node)

            neighbors = self._compute_node_neighbors(best_node)

            for neighbor in neighbors:
                if neighbor not in self.path_cost:
                    self.path_cost[neighbor] = 1e10

                tentative_path_cost = self._calculate_path_cost(best_node)

                if tentative_path_cost < self.path_cost[neighbor]:
                    came_from[neighbor] = best_node
                    self.path_cost[neighbor] = tentative_path_cost
                    self._update_node_total_cost(neighbor)
                    if neighbor not in self.nodes_to_explore:
                        self.nodes_to_explore[neighbor] = self.total_cost[neighbor]

        return []

    def _find_node_with_min_cost(self):
        return min(self.nodes_to_explore, key=self.nodes_to_explore.get)

    def _reconstruct_path(self, came_from):
        current_node = self.goal_node
        path = deque([current_node])

        while current_node in came_from:
            current_node = came_from[current_node]
            path.appendleft(current_node)

        return path

    def _compute_node_neighbors(self, node):
        # If no map is given, assume 8-connectivity
        if self.map is None:
            return [
                (node[0] - self.DISTANCE_BETWEEN_NODES, node[1] - self.DISTANCE_BETWEEN_NODES),
                (node[0] - self.DISTANCE_BETWEEN_NODES, node[1]),
                (node[0] - self.DISTANCE_BETWEEN_NODES, node[1] + self.DISTANCE_BETWEEN_NODES),
                (node[0], node[1] - self.DISTANCE_BETWEEN_NODES),
                (node[0], node[1] + self.DISTANCE_BETWEEN_NODES),
                (node[0] + self.DISTANCE_BETWEEN_NODES, node[1] - self.DISTANCE_BETWEEN_NODES),
                (node[0] + self.DISTANCE_BETWEEN_NODES, node[1]),
                (node[0] + self.DISTANCE_BETWEEN_NODES, node[1] + self.DISTANCE_BETWEEN_NODES)
            ]

        neighbors = []

        for row in [-1, 0, 1]:
            for col in [-1, 0, 1]:
                if row == 0 and col == 0:
                    continue

                if node[0] + row < 0 or node[0] + row >= self.map.shape[0]:
                    continue

                if node[1] + col < 0 or node[1] + col >= self.map.shape[1]:
                    continue

                if self.map[node[0] + row, node[1] + col] == 0:
                    continue

                neighbors.append((node[0] + row, node[1] + col))

        return neighbors

    def _update_node_total_cost(self, node):
        cost_from_path = self.path_cost[node]
        cost_from_heuristic = self._heuristic(node)

        self.total_cost[node] = cost_from_path + cost_from_heuristic

    def _calculate_path_cost(self, previous_node):
        return self.path_cost[previous_node] + self.COST_BETWEEN_NODES

    def _heuristic(self, node):
        return np.linalg.norm(np.array(self.goal_node) - np.array(node))


if __name__ == '__main__':
    print('Running A* algorithm...')

    start_node = (0, 0)
    goal_node = (0, 10)

    a_star = AStar(start_node, goal_node)

    path = a_star.compute_path()

    print(path)
