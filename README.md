# Navigation Python

Example path planners and controllers. To run simulations on them, I had to define a
vehicle, vehicle models and a simulation environment.

## Install

1. Clone this repository.
2. Install Docker.
3. Build the Docker container: `make build`

## Use

To run a simulation which uses predefined values, run
```
make run_simulation
```

To run a GUI which allows modifying the waypoints/planner/controller, run
```
make run_gui
```

![image](https://user-images.githubusercontent.com/7322431/152688184-9531fbb4-670b-47c0-89a4-f628b8877f6c.png)

## Customization

- The entry file for the non-GUI approach is `run_simulation.py`,
  which is where e.g. controllers can be swapped for others.
- All user-defined parameters are included in `parameters.yml`.
- To add new waypoints, create a .txt file in `data/waypoints` (it will be shown in the GUI).
- To add new references, create a .txt file in `data/references` (it will be shown in the
  GUI as part of the FixedReferencePlanner options).

**Note: The simulation can run only with a `referencesX.txt` file if the FixedReferencePlanner is selected.
For other planners, please include a `waypointsX.txt` file instead.**
