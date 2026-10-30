import math

from mineral_nav_goal.geometry import fits_wheel_corridor, grasp_goal


def test_goal_faces_target_at_requested_standoff():
    goal_x, goal_y, heading = grasp_goal(0.0, 0.0, 1.0, 1.0, 0.30)
    assert math.isclose(math.hypot(1.0 - goal_x, 1.0 - goal_y), 0.30)
    assert math.isclose(heading, math.pi / 4)


def test_wheel_corridor_accounts_for_clearance():
    assert fits_wheel_corridor(0.20, 0.30, 0.03)
    assert not fits_wheel_corridor(0.25, 0.30, 0.03)
