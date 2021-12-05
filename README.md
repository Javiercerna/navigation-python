# Navigation Python

Example path planners and controllers. To run simulations on them, I had to define a
vehicle, vehicle models and a simulation environment.

## Install

1. Clone this repository.
2. Install Docker.
3. Build the Docker container: `make build`

## Use

Run the simulation using `make run_simulation`.

## Customization

- The entry file is `run_simulation.py`, which is where e.g. controllers can be swapped for others.
- All user-defined parameters are included in `parameters.yml`.
- New paths can be added to the `data/` folder. Include a new folder that contains two
  files: `x.txt` and `y.txt`, which define a reference path. Then, update the constant
  `reference_folder` in `parameters.yml` with the appropriate name.

## Docker vs virtualenv
While using virtualenv, I found this issue:

```
Matplotlib is currently using agg, which is a non-GUI backend, so cannot show the figure.
  plt.show()
```

Since I wanted to keep this project reproducible, I opted for Docker, and installed
python3-tk in the Dockerfile.

You can easily skip Docker, as long as you manually install python3-tk or another backend
in your host OS.
