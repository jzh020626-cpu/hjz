from pathlib import Path
import math
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from wing_alignment_system.multi_tracer_return_home import (
    AVOIDING,
    DONE,
    GO_HOME,
    PAUSED_SAFE,
    RESUME_HOME,
    WAIT_ENTRY,
    PlannerConfig,
    PoseState,
    RobotRuntime,
    build_default_home_goals,
    choose_yield_robot,
    goal_changed,
    map_mocap_point_to_world,
    path_conflict,
    select_avoidance_goal,
    startup_gate_open,
    startup_gate_update,
    update_robot_mode,
    MultiTracerReturnHomePlanner,
)


def test_map_mocap_point_uses_mm_and_negated_z_for_world_y():
    xw, yw = map_mocap_point_to_world(14540.0, 3320.0)

    assert xw == 14.54
    assert yw == -3.32


def test_default_home_goals_match_requested_slots():
    homes = build_default_home_goals()

    assert homes["tracer1"].x == 14.54
    assert homes["tracer1"].y == -3.32
    assert homes["tracer1"].yaw_deg == 180.0
    assert homes["tracer2"].y == -2.32
    assert homes["tracer3"].y == -1.32


def test_non_conflicting_home_paths_keep_all_targets_on_home_slots():
    planner = MultiTracerReturnHomePlanner(PlannerConfig())
    planner.update_pose("tracer1", 12.0, -3.32, math.pi, 1.0)
    planner.update_pose("tracer2", 12.0, -2.32, math.pi, 1.0)
    planner.update_pose("tracer3", 12.0, -1.32, math.pi, 1.0)

    planner.evaluate(1.1)

    assert planner.global_state == GO_HOME
    assert planner.robots["tracer1"].active_goal == planner.robots["tracer1"].home_goal
    assert planner.robots["tracer2"].active_goal == planner.robots["tracer2"].home_goal
    assert planner.robots["tracer3"].active_goal == planner.robots["tracer3"].home_goal


def test_crossing_paths_conflict_when_predicted_spacing_is_below_clearance():
    assert path_conflict(
        start_a=(0.0, 0.0),
        goal_a=(2.0, 0.0),
        start_b=(1.0, -1.0),
        goal_b=(1.0, 1.0),
        clearance_m=1.8,
    )


def test_farther_robot_yields_when_paths_conflict():
    homes = build_default_home_goals()
    runtimes = {
        "tracer1": RobotRuntime(
            name="tracer1",
            home_goal=homes["tracer1"],
            pose=PoseState(x=14.0, y=-3.32, yaw=math.pi, stamp_sec=1.0),
        ),
        "tracer2": RobotRuntime(
            name="tracer2",
            home_goal=homes["tracer2"],
            pose=PoseState(x=8.0, y=-2.32, yaw=math.pi, stamp_sec=1.0),
        ),
    }

    assert choose_yield_robot("tracer1", "tracer2", runtimes, distance_tie_tol_m=0.05) == "tracer2"


def test_heading_error_breaks_distance_tie_for_yield_choice():
    homes = build_default_home_goals()
    runtimes = {
        "tracer1": RobotRuntime(
            name="tracer1",
            home_goal=homes["tracer1"],
            pose=PoseState(x=13.0, y=-3.32, yaw=math.pi, stamp_sec=1.0),
        ),
        "tracer2": RobotRuntime(
            name="tracer2",
            home_goal=homes["tracer2"],
            pose=PoseState(x=13.01, y=-2.32, yaw=0.0, stamp_sec=1.0),
        ),
    }

    assert choose_yield_robot("tracer1", "tracer2", runtimes, distance_tie_tol_m=0.05) == "tracer1"


def test_fixed_priority_breaks_full_tie_for_yield_choice():
    homes = build_default_home_goals()
    runtimes = {
        "tracer1": RobotRuntime(
            name="tracer1",
            home_goal=homes["tracer1"],
            pose=PoseState(x=13.0, y=-3.32, yaw=math.pi, stamp_sec=1.0),
        ),
        "tracer2": RobotRuntime(
            name="tracer2",
            home_goal=homes["tracer2"],
            pose=PoseState(x=13.0, y=-2.32, yaw=math.pi, stamp_sec=1.0),
        ),
    }

    assert choose_yield_robot("tracer1", "tracer2", runtimes, distance_tie_tol_m=0.05) == "tracer2"


def test_avoidance_goal_prefers_side_with_larger_path_separation():
    homes = build_default_home_goals()
    yielder = RobotRuntime(
        name="tracer2",
        home_goal=homes["tracer2"],
        pose=PoseState(x=1.0, y=0.0, yaw=0.0, stamp_sec=1.0),
    )
    keeper = RobotRuntime(
        name="tracer1",
        home_goal=homes["tracer1"],
        pose=PoseState(x=2.0, y=-1.0, yaw=math.pi / 2.0, stamp_sec=1.0),
    )

    goal = select_avoidance_goal(
        yielder=yielder,
        yielder_target=(3.0, 0.0),
        keeper=keeper,
        keeper_target=(2.0, 2.0),
        clearance_m=1.8,
        lateral_offset_m=1.15,
        staging_profile_code=1.0,
    )

    assert goal.y < 0.0
    assert goal.profile_code == 1.0


def test_reaching_avoidance_goal_returns_robot_to_resume_home_mode():
    homes = build_default_home_goals()
    runtime = RobotRuntime(
        name="tracer1",
        home_goal=homes["tracer1"],
        pose=PoseState(x=5.0, y=-3.0, yaw=math.pi, stamp_sec=2.0),
        mode=AVOIDING,
        avoidance_goal=homes["tracer1"].__class__(x=5.05, y=-3.02, yaw_deg=170.0, profile_code=1.0),
        avoidance_hold_until_sec=1.5,
    )

    update_robot_mode(runtime, now_sec=2.0, waypoint_reached_tol_m=0.1)

    assert runtime.mode == RESUME_HOME
    assert runtime.avoidance_goal is None


def test_identical_goal_is_not_treated_as_changed():
    goal = build_default_home_goals()["tracer1"]

    assert goal_changed(goal, goal, pos_tol_m=0.02, yaw_tol_deg=1.0) is False


def test_active_avoidance_goal_is_kept_stable_after_hold_window_expires():
    planner = MultiTracerReturnHomePlanner(
        PlannerConfig(owner_near_home_radius_m=0.2, pose_timeout_sec=5.0)
    )
    planner.update_pose("tracer1", 11.50, -4.50, math.radians(34.0), 10.0)
    planner.update_pose("tracer2", 12.20, -2.35, math.radians(35.0), 10.0)
    planner.update_pose("tracer3", 12.00, -0.39, math.radians(116.0), 10.0)

    planner.evaluate(10.0)
    first_goal = planner.robots["tracer3"].avoidance_goal
    assert first_goal is not None

    planner.update_pose("tracer1", 11.60, -4.45, math.radians(34.0), 10.5)
    planner.update_pose("tracer2", 12.18, -2.34, math.radians(35.0), 10.5)
    planner.update_pose("tracer3", 11.98, -0.37, math.radians(116.0), 10.5)
    planner.evaluate(11.2)

    second_goal = planner.robots["tracer3"].avoidance_goal
    assert second_goal == first_goal


def test_robot_near_home_becomes_entry_owner_and_others_queue_on_wait_line():
    planner = MultiTracerReturnHomePlanner(
        PlannerConfig(queue_x_backoff_m=1.0, owner_near_home_radius_m=0.8)
    )
    planner.update_pose("tracer1", 14.10, -3.10, math.radians(17.0), 1.0)
    planner.update_pose("tracer2", 11.88, -2.57, math.radians(30.0), 1.0)
    planner.update_pose("tracer3", 11.90, -1.14, math.radians(24.0), 1.0)

    planner.evaluate(1.1)

    assert planner.entry_owner == "tracer1"
    assert planner.robots["tracer1"].active_goal == planner.robots["tracer1"].home_goal
    assert planner.robots["tracer2"].mode == WAIT_ENTRY
    assert planner.robots["tracer3"].mode == WAIT_ENTRY
    assert planner.robots["tracer2"].active_goal.x == 13.54
    assert planner.robots["tracer2"].active_goal.y == -2.32
    assert planner.robots["tracer3"].active_goal.x == 13.54
    assert planner.robots["tracer3"].active_goal.y == -1.32


def test_waiting_line_owner_holds_until_done_then_releases_next_robot():
    planner = MultiTracerReturnHomePlanner(
        PlannerConfig(queue_x_backoff_m=1.0, owner_near_home_radius_m=0.8, pose_timeout_sec=5.0)
    )
    planner.update_pose("tracer1", 14.10, -3.10, math.radians(17.0), 1.0)
    planner.update_pose("tracer2", 13.54, -2.32, math.pi, 1.0)
    planner.update_pose("tracer3", 13.54, -1.32, math.pi, 1.0)
    planner.evaluate(1.1)
    assert planner.entry_owner == "tracer1"

    planner.update_goal_reached("tracer1", True)
    planner.update_pose("tracer1", 14.54, -3.32, math.pi, 2.0)
    planner.update_pose("tracer2", 13.54, -2.32, math.pi, 2.0)
    planner.update_pose("tracer3", 13.54, -1.32, math.pi, 2.0)
    planner.evaluate(2.1)

    assert planner.entry_owner == "tracer2"
    assert planner.robots["tracer2"].active_goal == planner.robots["tracer2"].home_goal
    assert planner.robots["tracer3"].mode == WAIT_ENTRY


def test_startup_gate_requires_all_fresh_poses_for_hold_window():
    homes = build_default_home_goals()
    runtimes = {
        "tracer1": RobotRuntime("tracer1", homes["tracer1"], pose=PoseState(0.0, 0.0, 0.0, 1.0)),
        "tracer2": RobotRuntime("tracer2", homes["tracer2"], pose=PoseState(0.0, 0.0, 0.0, 1.0)),
        "tracer3": RobotRuntime("tracer3", homes["tracer3"], pose=PoseState(0.0, 0.0, 0.0, 1.0)),
    }

    ready_since = startup_gate_update(runtimes, now_sec=1.1, pose_timeout_sec=0.3, ready_since_sec=None)

    assert ready_since == 1.1
    assert not startup_gate_open(ready_since, now_sec=1.5, hold_sec=1.0)
    assert startup_gate_open(ready_since, now_sec=2.2, hold_sec=1.0)


def test_startup_gate_resets_when_any_pose_goes_stale():
    homes = build_default_home_goals()
    runtimes = {
        "tracer1": RobotRuntime("tracer1", homes["tracer1"], pose=PoseState(0.0, 0.0, 0.0, 1.0)),
        "tracer2": RobotRuntime("tracer2", homes["tracer2"], pose=PoseState(0.0, 0.0, 0.0, 1.0)),
        "tracer3": RobotRuntime("tracer3", homes["tracer3"], pose=PoseState(0.0, 0.0, 0.0, 0.0)),
    }

    ready_since = startup_gate_update(runtimes, now_sec=1.1, pose_timeout_sec=0.3, ready_since_sec=0.5)

    assert ready_since is None


def test_pose_timeout_puts_planner_into_paused_safe_state():
    planner = MultiTracerReturnHomePlanner(PlannerConfig(pose_timeout_sec=0.3))
    planner.update_pose("tracer1", 12.0, -3.32, math.pi, 1.0)
    planner.update_pose("tracer2", 12.0, -2.32, math.pi, 0.5)
    planner.update_pose("tracer3", 12.0, -1.32, math.pi, 1.0)

    planner.evaluate(1.1)

    assert planner.global_state == PAUSED_SAFE
    assert all(runtime.stop_requested for runtime in planner.robots.values())


def test_planner_marks_robot_done_after_home_goal_is_reached():
    planner = MultiTracerReturnHomePlanner(PlannerConfig())
    planner.update_pose("tracer1", 14.54, -3.32, math.pi, 1.0)
    planner.update_pose("tracer2", 12.0, -2.32, math.pi, 1.0)
    planner.update_pose("tracer3", 12.0, -1.32, math.pi, 1.0)
    planner.update_goal_reached("tracer1", True)

    planner.evaluate(1.1)

    assert planner.robots["tracer1"].mode == DONE
