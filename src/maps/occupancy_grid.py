import numpy as np
import matplotlib.pyplot as plt


class OccupancyGrid(object):
    EMPTY = 0
    FULL = 100

    def __init__(self, width, height, resolution=1):
        ## The map resolution [m/cell]
        self.resolution = resolution
        ## Map width [cells]
        self.width = width
        ## Map height [cells]
        self.height = height
        ## Map data, occupancy probabilities in the range [0, 100]. Unknown is -1
        self.grid_data = np.zeros((self.width, self.height))
        ## Grid visualization options
        self.grid_visualization_options = {
            'cmap': 'gray_r',
            'origin': 'lower',
            'aspect': 'equal',
            'extent': [
                0 - self.resolution / 2.0,
                self.width * self.resolution - self.resolution / 2.0,
                0 - self.resolution / 2.0,
                self.height * self.resolution - self.resolution / 2.0
            ],
            'vmin': 0,
            'vmax': 100
        }

    @property
    def total_width_m(self):
        return self.width * self.resolution

    @property
    def total_height_m(self):
        return self.height * self.resolution

    def is_node_within_map(self, node):
        check_x = node[0] >= 0 and node[0] < self.width * self.resolution
        check_y = node[1] >= 0 and node[1] < self.height * self.resolution

        return check_x and check_y

    def get_closest_cell(self, node):
        closest_x = int(node[0] / self.resolution + 0.5) * self.resolution
        closest_y = int(node[1] / self.resolution + 0.5) * self.resolution

        # Preserve other indices (e.g. orientation)
        return (closest_x, closest_y) + node[2:]

    def show(self):
        # Flip axes to visualize correctly with imshow
        plt.imshow(self.grid_data.T, **self.grid_visualization_options)
        plt.show()


if __name__ == '__main__':
    width = 10
    height = 10

    grid = OccupancyGrid(width, height)

    for _ in range(width * height // 2):
        random_width = np.random.randint(width)
        random_height = np.random.randint(height)
        random_value = np.random.randint(100)
        grid.grid_data[random_width, random_height] = random_value

    print(grid.grid_data)

    grid.show()
