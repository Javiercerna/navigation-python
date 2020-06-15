import numpy as np
import matplotlib.pyplot as plt


class OccupancyGrid(object):
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
            'cmap': 'gray',
            'origin': 'lower',
            'aspect': 'equal',
            'extent': [
                0 - self.resolution / 2,
                self.width * self.resolution - self.resolution / 2,
                0 - self.resolution / 2,
                self.height * self.resolution - self.resolution / 2
            ],
            'vmin': 0,
            'vmax': 100
        }

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
