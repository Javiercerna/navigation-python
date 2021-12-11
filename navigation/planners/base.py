from abc import ABC, abstractmethod

import numpy as np


class Planner(ABC):
    @abstractmethod
    def calculate_reference(self, *args, **kwargs) -> np.ndarray:
        pass
