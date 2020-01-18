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
        vehicle_position = np.tile(vehicle_position, reps=(len(path), 1))

        distances_to_path = np.linalg.norm(path - vehicle_position, axis=1)

        return np.argmin(distances_to_path)

    def _find_goal_point(self, closest_path_ind, path):
        start_point = path[closest_path_ind]

        num_points_visited = 0
        ind = closest_path_ind + 1
        while num_points_visited < len(path):
            distance = np.linalg.norm(path[ind] - start_point)
            if distance >= self.lookahead_distance:
                return path[ind]

            ind = (ind + 1) % len(path)
            num_points_visited += 1

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
