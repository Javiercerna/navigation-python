import numpy as np
import matplotlib.pyplot as plt
from scipy import sparse

from paths.path import Path
from controllers.mpc import MPC
from models.spatial_bicycle_model import SpatialBicycleModel
from models.vehicle import Vehicle

import json
import math

simulation_time = 80
dt = 0.1
simulation_steps = int(simulation_time / dt)

Q = sparse.diags([1, 1])
Qn = Q
R = 0.1*sparse.eye(1)
prediction_horizon = 10
velocity = 5

with open('./src/paths/simulated_waypoints.json') as f:
    waypoints = json.load(f)
    waypoints_x = waypoints['waypoints_x']
    waypoints_y = waypoints['waypoints_y']

path = Path(waypoints_x=waypoints_x, waypoints_y=waypoints_y)

wheelbase = 1
max_velocity = 10
max_steering_angle = math.pi

initial_state = np.array([0, 0, math.pi/2])

vehicle = Vehicle(
    wheelbase=wheelbase, initial_state=initial_state, dt=dt,
    max_velocity=max_velocity, max_steering_angle=max_steering_angle)

vehicle_x = np.zeros(simulation_steps)
vehicle_y = np.zeros(simulation_steps)
steering_angles = np.zeros(simulation_steps)

spatial_bicycle_model = SpatialBicycleModel()

mpc = MPC(Q=Q, R=R, Qn=Qn, prediction_horizon=prediction_horizon,
          kappa_tilde_min=-np.inf, kappa_tilde_max=np.inf, wheelbase=wheelbase)
mpc.setup_optimization_problem(
    spatial_bicycle_model, vehicle.state, path.as_array_with_curvature())

for k in range(simulation_steps):
    steering_angle = mpc.compute_steering_angle(
        spatial_bicycle_model, vehicle.state, path.as_array_with_curvature())

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
