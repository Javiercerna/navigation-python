from navigation.utils import get_reference_filenames, get_waypoint_filenames


PLANNER = 'Planner'
LATERAL_CONTROLLER = 'Lateral Controller'
LONGITUDINAL_CONTROLLER = 'Longitudinal Controller'
WAYPOINTS = 'Waypoints'

REQUIRED_OPTIONS = [PLANNER, LATERAL_CONTROLLER, LONGITUDINAL_CONTROLLER]


def get_simulation_options() -> dict:
    return {
        WAYPOINTS: get_waypoint_filenames(),
        PLANNER: [
            *[
                f'FixedReferencePlanner ({reference_filename})'
                for reference_filename in get_reference_filenames()
            ],
            'SplinePlanner',
        ],
        LATERAL_CONTROLLER: ['PurePursuit'],
        LONGITUDINAL_CONTROLLER: ['FixedLinearVelocityController'],
    }
