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

Q = sparse.diags([0.5, 0.5])
Qn = Q
R = 0.1*sparse.eye(1)
prediction_horizon = 3
velocity = 5

with open('./src/paths/simulated_waypoints.json') as f:
    waypoints = json.load(f)
    waypoints_x = waypoints['waypoints_x']
    waypoints_y = waypoints['waypoints_y']

path = Path(waypoints_x=waypoints_x, waypoints_y=waypoints_y)
path_curvatures = path.path_curvature

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

mpc = MPC(Q=Q, R=R, Qn=Qn, prediction_horizon=prediction_horizon,
          kappa_tilde_min=-np.inf, kappa_tilde_max=np.inf)

spatial_bicycle_model = SpatialBicycleModel(ds=velocity*dt)

for k in range(simulation_steps):
    A, B, reference_curvature = spatial_bicycle_model.get_linearized_matrices(
        vehicle.state, path.as_array(), path_curvatures)

    state = spatial_bicycle_model.get_state(vehicle.state, path.as_array())
    print('State: e_y={}, e_psi={}'.format(state[0], state[1]))

    result = mpc.compute_steering_angle(A, B, state)

    _, nu = B.shape
    k_tilde = result.x[-prediction_horizon*nu:-(prediction_horizon-1)*nu]

    if k_tilde[0] is None:
        print('Problem infeasible...')
        vehicle.send_commands(velocity=0, steering_angle=0)
    else:
        curvature = k_tilde + reference_curvature
        steering_angle = float(np.arctan2(curvature * wheelbase, 1))
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
