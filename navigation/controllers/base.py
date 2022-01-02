from abc import ABC, abstractmethod
from typing import Union


class Controller(ABC):
    @abstractmethod
    def calculate_steering_angle(self, *args, **kwargs) -> Union[float, None]:
        pass

    @abstractmethod
    def calculate_linear_velocity(self, *args, **kwargs) -> Union[float, None]:
        pass


class LateralController(ABC):
    @abstractmethod
    def calculate_steering_angle(self, *args, **kwargs) -> Union[float, None]:
        pass


class LongitudinalController(ABC):
    @abstractmethod
    def calculate_linear_velocity(self, *args, **kwargs) -> Union[float, None]:
        pass


class DecoupledController(Controller):
    def __init__(
            self,
            lateral_controller: LateralController,
            longitudinal_controller: LongitudinalController,
        ) -> None:
        self.lateral_controller = lateral_controller
        self.longitudinal_controller = longitudinal_controller

    def calculate_steering_angle(self, *args, **kwargs) -> Union[float, None]:
        return self.lateral_controller.calculate_steering_angle(*args, **kwargs)

    def calculate_linear_velocity(self, *args, **kwargs) -> Union[float, None]:
        return self.longitudinal_controller.calculate_linear_velocity(*args, **kwargs)
