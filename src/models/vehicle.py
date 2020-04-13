import numpy as np
import matplotlib.pyplot as plt

import math


class Vehicle(object):
    def __init__(
            self, wheelbase, initial_state, dt, max_velocity,
            max_steering_angle):
        self.wheelbase = wheelbase
        self.state = initial_state
        self.dt = dt

        self.MAX_VELOCITY = max_velocity
        self.MAX_STEERING_ANGLE = max_steering_angle

    def send_commands(self, velocity, steering_angle):
        velocity = self._limit_velocity(velocity)
        steering_angle = self._limit_steering_angle(steering_angle)

        u = np.array(
            [velocity * self.dt * np.cos(self.state[2]),
             velocity * self.dt * np.sin(self.state[2]),
             velocity * self.dt * np.tan(steering_angle) / self.wheelbase])

        self.state = self.state + u

    def _limit_velocity(self, velocity):
        if velocity <= self.MAX_VELOCITY:
            return velocity

        print('Limiting velocity. Input was: {:.3f}'.format(velocity))
        return self.MAX_VELOCITY

    def _limit_steering_angle(self, steering_angle):
        if abs(steering_angle) <= self.MAX_STEERING_ANGLE:
            return steering_angle

        print(
            'Limiting steering angle. Input was: {:.3f}'
            .format(np.rad2deg(steering_angle))
        )

        return self.MAX_STEERING_ANGLE if steering_angle >= 0 \
            else -self.MAX_STEERING_ANGLE


if __name__ == '__main__':
    np.random.seed(0)
    wheelbase = 1
    max_velocity = 10
    max_steering_angle = math.pi / 4

    initial_state = np.array([0, 0, 0])

    simulation_time = 10
    dt = 0.1
    simulation_steps = int(simulation_time / dt)

    vehicle = Vehicle(
        wheelbase=wheelbase, initial_state=initial_state, dt=dt,
        max_velocity=max_velocity, max_steering_angle=max_steering_angle)

    vehicle_x = np.zeros(simulation_steps)
    vehicle_y = np.zeros(simulation_steps)

    for k in range(simulation_steps):
        velocity = np.random.rand() * max_velocity
        steering_angle = (2 * np.random.rand() - 1) * max_steering_angle
        vehicle.send_commands(velocity=velocity, steering_angle=steering_angle)

        vehicle_x[k] = vehicle.state[0]
        vehicle_y[k] = vehicle.state[1]

    plt.plot(vehicle_x, vehicle_y)
    plt.show()
