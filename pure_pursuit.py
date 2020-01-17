import numpy as np

import math


class PurePursuit(object):
    def __init__(self, wheelbase, lookahead_distance):
        self.wheelbase = wheelbase
        self.lookahead_distance = lookahead_distance

    def compute_steering_angle(self, vehicle_pose, path):
        vehicle_position = vehicle_pose[0:2]
        closest_path_ind = self._find_closest_point_on_path(
            vehicle_position, path)

        goal_point = self._find_goal_point(closest_path_ind, path)

        if goal_point is None:
            return None

        curvature = self._calculate_desired_curvature(
            vehicle_pose, goal_point)

        return self._convert_curvature_to_steering_angle(curvature)

    def _find_closest_point_on_path(self, vehicle_position, path):
        min_distance = 1e20
        min_distance_ind = -1

        for ind in range(len(path)):
            distance = np.linalg.norm(path[ind] - vehicle_position)

            if distance < min_distance:
                min_distance = distance
                min_distance_ind = ind

        return min_distance_ind

    def _find_goal_point(self, closest_path_ind, path):
        start_point = path[closest_path_ind]

        for ind in range(closest_path_ind+1, len(path)):
            distance = np.linalg.norm(path[ind] - start_point)
            if distance >= self.lookahead_distance:
                return path[ind]

        for ind in range(len(path)):
            distance = np.linalg.norm(path[ind] - start_point)
            if distance >= self.lookahead_distance:
                return path[ind]

        print('No goal point found')
        return None

    def _calculate_desired_curvature(self, vehicle_pose, goal_point):
        goal_point = self._convert_to_vehicle_coordinates(
            vehicle_pose, goal_point)

        return 2 * goal_point[0] / (self.lookahead_distance ** 2)

    def _convert_to_vehicle_coordinates(self, vehicle_pose, goal_point):
        vehicle_x, vehicle_y, vehicle_heading = vehicle_pose
        vehicle_heading += math.pi/2
        goal_x, goal_y = goal_point

        new_goal_x = (goal_x - vehicle_x) * np.cos(vehicle_heading) + \
            (goal_y - vehicle_y) * np.sin(vehicle_heading)
        new_goal_y = -(goal_x - vehicle_x) * np.sin(vehicle_heading) + \
            (goal_y - vehicle_y) * np.cos(vehicle_heading)

        return np.array([new_goal_x, new_goal_y])

    def _convert_curvature_to_steering_angle(self, curvature):
        return np.arctan2(curvature * self.wheelbase, 1)
