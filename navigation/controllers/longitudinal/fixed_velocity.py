from typing import Union

from navigation.controllers.base import LongitudinalController


class FixedLinearVelocityController(LongitudinalController):
    def __init__(self, fixed_linear_velocity: float) -> None:
        self.fixed_linear_velocity = fixed_linear_velocity

    def calculate_linear_velocity(self, *args, **kwargs) -> Union[float, None]:
        return self.fixed_linear_velocity
