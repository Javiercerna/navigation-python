from typing import Union

from navigation.controllers.base import LongitudinalController


class FakeLongitudinalController(LongitudinalController):
    def calculate_linear_velocity(self) -> Union[float, None]:
        return 5.0
