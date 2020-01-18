import numpy as np
import matplotlib.pyplot as plt

from path import Path
from pure_pursuit import PurePursuit
from vehicle import Vehicle

import json
import math

simulation_time = 80
dt = 0.1
simulation_steps = int(simulation_time / dt)

lookahead_distance = 10
velocity = 5

with open('simulated_waypoints.json') as f:
    waypoints = json.load(f)
    waypoints_x = waypoints['waypoints_x']
    waypoints_y = waypoints['waypoints_y']

path = Path(waypoints_x=waypoints_x, waypoints_y=waypoints_y)

wheelbase = 1
max_velocity = 10
max_steering_angle = math.pi/4

initial_state = np.array([0, 0, math.pi/2])

vehicle = Vehicle(
    wheelbase=wheelbase, initial_state=initial_state, dt=dt,
    max_velocity=max_velocity, max_steering_angle=max_steering_angle)

vehicle_x = np.zeros(simulation_steps)
vehicle_y = np.zeros(simulation_steps)
steering_angles = np.zeros(simulation_steps)

pure_pursuit = PurePursuit(
    wheelbase=wheelbase, lookahead_distance=lookahead_distance)

for k in range(simulation_steps):
    steering_angle = pure_pursuit.compute_steering_angle(
        vehicle.state, path.as_array())

    if steering_angle is not None:
        vehicle.send_commands(velocity=velocity, steering_angle=steering_angle)

    vehicle_x[k] = vehicle.state[0]
    vehicle_y[k] = vehicle.state[1]
    steering_angles[k] = steering_angle

    plt.clf()
    plt.plot(vehicle_x[k], vehicle_y[k], '-ko')
    plt.plot(vehicle_x[0:k], vehicle_y[0:k], 'b')
    plt.plot(path.path_x, path.path_y, 'g')

    plt.pause(0.01)

plt.figure()
plt.plot(steering_angles)

plt.show()
