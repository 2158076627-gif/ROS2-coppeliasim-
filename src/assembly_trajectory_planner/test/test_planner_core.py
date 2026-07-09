from assembly_trajectory_planner.planner_core import AssemblyTrajectoryPlanner, Pose3D, waypoint_names


def test_planner_generates_named_assembly_stages():
    planner = AssemblyTrajectoryPlanner()
    target = Pose3D(position=(0.4, 0.0, 0.2))

    waypoints = planner.plan(target)

    assert waypoint_names(waypoints) == [
        'observe',
        'pre_align',
        'contact_search',
        'insert',
        'hold',
        'retract',
    ]
    assert waypoints[0].pose.position[2] > target.position[2]
    assert waypoints[3].pose.position[2] < target.position[2]
