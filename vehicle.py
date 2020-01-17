import numpy as np
import matplotlib.pyplot as plt

import math


class Vehicle(object):
    def __init__(
            self, wheelbase, initial_x, initial_y, initial_theta, dt,
            max_velocity, max_steering_angle):
        self.wheelbase = wheelbase
        self.x = initial_x
        self.y = initial_y
        self.theta = initial_theta
        self.dt = dt

        self.MAX_VELOCITY = max_velocity
        self.MAX_STEERING_ANGLE = max_steering_angle

    def send_commands(self, velocity, steering_angle):
        velocity = self._limit_velocity(velocity)
        steering_angle = self._limit_steering_angle(steering_angle)

        self.x = self.x + velocity * self.dt * np.cos(self.theta)
        self.y = self.y + velocity * self.dt * np.sin(self.theta)
        self.theta = self.theta + velocity * self.dt * \
            np.tan(steering_angle) / self.wheelbase

    def _limit_velocity(self, velocity):
        if velocity <= self.MAX_VELOCITY:
            return velocity
        else:
            print('Limiting velocity. Input was: {:.3f}'.format(velocity))
            return self.MAX_VELOCITY

    def _limit_steering_angle(self, steering_angle):
        if abs(steering_angle) <= self.MAX_STEERING_ANGLE:
            return steering_angle
        else:
            print(
                'Limiting steering angle. Input was : {:.3f}'.format(
                    steering_angle))
            return self.MAX_STEERING_ANGLE if steering_angle >= 0 \
                else -self.MAX_STEERING_ANGLE


if __name__ == '__main__':
    np.random.seed(0)
    wheelbase = 1
    max_velocity = 10
    max_steering_angle = math.pi/4

    initial_x = 0
    initial_y = 0
    initial_theta = 0

    simulation_time = 10
    dt = 0.1
    simulation_steps = int(simulation_time / dt)

    vehicle = Vehicle(
        wheelbase=wheelbase, initial_x=initial_x, initial_y=initial_y,
        initial_theta=initial_theta, dt=dt, max_velocity=max_velocity,
        max_steering_angle=max_steering_angle)

    vehicle_x = np.zeros(simulation_steps)
    vehicle_y = np.zeros(simulation_steps)

    for k in range(simulation_steps):
        velocity = np.random.rand() * max_velocity
        steering_angle = (2 * np.random.rand() - 1) * max_steering_angle
        vehicle.send_commands(velocity=velocity, steering_angle=steering_angle)

        vehicle_x[k] = vehicle.x
        vehicle_y[k] = vehicle.y

    plt.plot(vehicle_x, vehicle_y)
    plt.show()
