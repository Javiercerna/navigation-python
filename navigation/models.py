import math
from abc import ABC, abstractmethod
from typing import Any


class VehicleModel(ABC):
    @staticmethod
    @abstractmethod
    def calculate_next_state(*args, **kwargs):
        pass


class UnicycleModel(VehicleModel):
    @staticmethod
    def calculate_next_state(
            state: Any,
            linear_velocity: float,
            angular_velocity: float,
            dt: float
        ) -> Any:
        x = state.x + linear_velocity * dt * math.cos(state.theta)
        y = state.y + linear_velocity * dt * math.sin(state.theta)
        theta = state.theta + angular_velocity * dt

        return type(state)(x=x, y=y, theta=theta)


class KinematicBicycleModel(VehicleModel):
    @staticmethod
    def calculate_next_state(
            state: Any,
            linear_velocity: float,
            steering_angle: float,
            dt: float,
            wheelbase: float
        ) -> Any:
        curvature = KinematicBicycleModel.steering_angle_to_curvature(
            steering_angle, wheelbase
        )

        x = state.x + linear_velocity * dt * math.cos(state.theta)
        y = state.y + linear_velocity * dt * math.sin(state.theta)
        theta = state.theta + linear_velocity * dt * curvature

        return type(state)(x=x, y=y, theta=theta)

    @staticmethod
    def steering_angle_to_curvature(steering_angle: float, wheelbase: float) -> float:
        return math.tan(steering_angle) / wheelbase

    @staticmethod
    def curvature_to_steering_angle(curvature: float, wheelbase: float) -> float:
        return math.atan2(curvature * wheelbase, 1)


class SpatialBicycleModel(VehicleModel):
    @staticmethod
    def calculate_next_state(
            state: Any,
            curvature: float,
            curvature_ref: float,
            ds: float
        ) -> Any:
        curvature_tilde = curvature - curvature_ref

        e_z = state.e_z + ds * state.e_theta
        e_theta = state.e_theta - (curvature_ref ** 2) * state.e_z + curvature_tilde * ds

        return type(state)(e_z=e_z, e_theta=e_theta)
