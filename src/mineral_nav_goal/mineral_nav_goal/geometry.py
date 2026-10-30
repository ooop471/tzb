import math


def grasp_goal(robot_x, robot_y, target_x, target_y, standoff_m):
    delta_x = target_x - robot_x
    delta_y = target_y - robot_y
    distance = math.hypot(delta_x, delta_y)
    if not math.isfinite(distance) or distance < 1e-6:
        raise ValueError('Target and robot positions must differ')
    heading = math.atan2(delta_y, delta_x)
    return (
        target_x - standoff_m * delta_x / distance,
        target_y - standoff_m * delta_y / distance,
        heading,
    )


def fits_wheel_corridor(object_width_m, inner_width_m, side_clearance_m):
    return (
        math.isfinite(object_width_m)
        and object_width_m > 0.0
        and object_width_m + 2.0 * side_clearance_m <= inner_width_m
    )
