from navigation.models import VehicleModel
from navigation.utils import State


class Vehicle:
    def __init__(
            self,
            initial_state: State,
            model: type[VehicleModel],
            input_limits: dict,
            dimensions: dict,
        ) -> None:
        self.state = initial_state
        self.model = model
        self.input_limits = input_limits
        self.dimensions = dimensions

    def update_state(self, linear_velocity: float, steering_angle: float, dt: float):
        linear_velocity = self.limit_input('linear_velocity', linear_velocity)
        steering_angle = self.limit_input('steering_angle', steering_angle)

        self.state = self.model.calculate_next_state(
            self.state, linear_velocity, steering_angle, dt, self.dimensions['wheelbase']
        )

    def limit_input(self, input_name, input_value):
        lower_limit = self.input_limits[f'{input_name}_min']
        upper_limit = self.input_limits[f'{input_name}_max']

        if input_value < lower_limit:
            return lower_limit
        elif input_value > upper_limit:
            return upper_limit
        else:
            return input_value
