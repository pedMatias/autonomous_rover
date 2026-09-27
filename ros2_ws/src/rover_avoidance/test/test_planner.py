import math

import numpy as np
import pytest

from rover_avoidance.planner import (
    arc_is_clear,
    choose_heading,
    choose_search_step,
    command_for_heading,
    depth_to_base_points,
    filtered_yaw,
    quaternion_rotate,
    relative_goal_to_odom,
)


class CameraInfo:
    width = 5
    height = 5
    k = [100.0, 0.0, 2.0, 0.0, 100.0, 2.0, 0.0, 0.0, 1.0]


def test_relative_goal_is_frozen_in_odom_coordinates():
    assert relative_goal_to_odom(2.0, 1.0, 3.0, 4.0, math.pi / 2) == pytest.approx(
        (2.0, 6.0)
    )


def test_relative_goal_rejects_nonfinite_values():
    with pytest.raises(ValueError):
        relative_goal_to_odom(float("nan"), 0.0, 0.0, 0.0, 0.0)


def test_quaternion_rotation_maps_optical_forward_to_base_forward():
    # live_bringup's optical-frame rotation is RPY=(-pi/2, 0, -pi/2).
    q = (-0.5, 0.5, -0.5, 0.5)
    assert quaternion_rotate([0.0, 0.0, 1.0], q) == pytest.approx([1.0, 0.0, 0.0])


def test_depth_projection_skips_invalid_and_respects_transform():
    depth = np.full((5, 5), 1000, dtype=np.uint16)
    depth[0, 0] = 0
    points, unknown, fraction = depth_to_base_points(
        depth,
        CameraInfo(),
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0, 1.0),
        decimation=1,
        missing_depth_blocks_paths=True,
    )
    assert fraction == pytest.approx(1.0)
    assert points.shape == (24, 3)
    assert points[:, 2] == pytest.approx(np.ones(24))
    assert unknown.shape[0] > 0


def test_depth_hole_blocks_corridor_in_conservative_mode():
    info = CameraInfo()
    info.width = 80
    info.height = 60
    info.k = [90.0, 0.0, 40.0, 0.0, 90.0, 30.0, 0.0, 0.0, 1.0]
    depth = np.full((60, 80), 1500, dtype=np.uint16)
    depth[16:55, 30:50] = 0
    points, unknown, coverage = depth_to_base_points(
        depth,
        info,
        (0.0, 0.0, 0.0),
        (-0.5, 0.5, -0.5, 0.5),
        decimation=8,
        unknown_range_m=1.2,
        missing_depth_blocks_paths=True,
    )
    assert coverage > 0.08
    assert len(points) > 0
    assert len(unknown) > 0
    assert choose_heading(0.0, 1.0, points, unknown_points=unknown) != pytest.approx(0.0)


def test_depth_holes_are_a_stereo_dropout_not_an_obstacle_by_default():
    # A textureless indoor scene drops out on the ceiling and near floor. Those
    # pixels must not veto every corridor, or the rover can never drive.
    info = CameraInfo()
    info.width = 80
    info.height = 60
    info.k = [90.0, 0.0, 40.0, 0.0, 90.0, 30.0, 0.0, 0.0, 1.0]
    depth = np.full((60, 80), 1500, dtype=np.uint16)
    depth[16:55, 30:50] = 0
    depth[0:8, :] = 0
    points, unknown, coverage = depth_to_base_points(
        depth,
        info,
        (0.0, 0.0, 0.0),
        (-0.5, 0.5, -0.5, 0.5),
        decimation=8,
        unknown_range_m=1.2,
    )
    assert coverage > 0.08
    assert unknown.shape == (0, 3)
    assert choose_heading(0.0, 1.0, points, unknown_points=unknown) == pytest.approx(0.0)


def test_beyond_range_returns_are_unknown_in_conservative_mode_only():
    info = CameraInfo()
    info.width = 80
    info.height = 60
    info.k = [90.0, 0.0, 40.0, 0.0, 90.0, 30.0, 0.0, 0.0, 1.0]
    depth = np.full((60, 80), 1500, dtype=np.uint16)
    depth[8:25, :] = 6000

    points, unknown, coverage = depth_to_base_points(
        depth,
        info,
        (0.0, 0.0, 0.12),
        (-0.5, 0.5, -0.5, 0.5),
        decimation=8,
        max_depth_m=3.0,
    )
    assert coverage > 0.08
    assert unknown.shape == (0, 3)
    assert choose_heading(0.0, 1.0, points, unknown_points=unknown) == pytest.approx(0.0)

    _, conservative_unknown, _ = depth_to_base_points(
        depth,
        info,
        (0.0, 0.0, 0.12),
        (-0.5, 0.5, -0.5, 0.5),
        decimation=8,
        max_depth_m=3.0,
        missing_depth_blocks_paths=True,
    )
    assert conservative_unknown.shape[0] > 0


def test_measured_obstacle_blocks_a_mostly_dropped_out_frame():
    # Only the small central patch returns depth; the rest is a dropout. The
    # measured patch must still steer the rover away from it.
    info = CameraInfo()
    info.width = 80
    info.height = 60
    info.k = [90.0, 0.0, 40.0, 0.0, 90.0, 30.0, 0.0, 0.0, 1.0]
    depth = np.zeros((60, 80), dtype=np.uint16)
    depth[24:37, 36:45] = 500

    points, unknown, _ = depth_to_base_points(
        depth,
        info,
        (0.0, 0.0, 0.12),
        (-0.5, 0.5, -0.5, 0.5),
        decimation=8,
        min_depth_m=0.2,
        max_depth_m=3.0,
    )
    assert unknown.shape == (0, 3)
    assert points.shape[0] > 0
    heading = choose_heading(0.0, 1.0, points, unknown_points=unknown)
    assert heading is not None
    assert abs(heading) >= 0.3


def test_choose_heading_goes_around_center_obstacle():
    points = np.array([[0.9, 0.0, 0.2]], dtype=np.float32)
    heading = choose_heading(0.0, 2.0, points, rover_half_width=0.2)
    assert heading is not None
    assert abs(heading) >= 0.3


def test_forward_turn_checks_arc_instead_of_straight_corridor():
    obstacle = np.array([[0.5, 0.0, 0.2]], dtype=np.float32)
    assert choose_heading(0.7, 2.0, obstacle) == pytest.approx(0.7)
    assert not arc_is_clear(obstacle[:, :2], 0.7, 0.35, 0.65, 1.2, 0.22)
    assert choose_heading(
        0.7, 2.0, obstacle,
        steering={"max_linear_speed": 0.35, "min_angular_speed": 0.5},
    ) != pytest.approx(0.7)


def test_debug_candidates_match_the_actual_heading_decision():
    obstacle = np.array([[0.9, 0.0, 0.2]], dtype=np.float32)
    steering = {"max_linear_speed": 0.35, "min_angular_speed": 0.5}
    expected = choose_heading(0.0, 2.0, obstacle, steering=steering)
    candidates = []
    considered = []
    selected = choose_heading(
        0.0, 2.0, obstacle, steering=steering,
        candidates_debug=candidates, considered_points=considered,
    )
    assert selected == expected
    np.testing.assert_allclose(considered, obstacle)
    assert len(candidates) == 7
    assert any(not candidate["clear"] and candidate["blocked_point"] is not None for candidate in candidates)
    assert any(candidate["clear"] and candidate["heading"] == selected for candidate in candidates)
    assert all(len(candidate["path"]) > 1 for candidate in candidates)


def test_tighter_experimental_footprint_allows_a_close_side_path():
    obstacle = np.array([[0.7, y, 0.2] for y in np.linspace(-0.2, 0.2, 9)])
    steering = {"max_linear_speed": 0.35, "min_angular_speed": 0.5}
    assert choose_heading(0.0, 10.0, obstacle, rover_half_width=0.22, steering=steering) is None
    assert choose_heading(0.0, 10.0, obstacle, rover_half_width=0.18, steering=steering) is not None
    obstacle[:, 0] = 0.6
    assert choose_heading(0.0, 10.0, obstacle, rover_half_width=0.18, steering=steering) is None


def test_forward_turn_keeps_outer_wheel_below_limit():
    linear, angular = command_for_heading(0.7, max_linear_speed=0.49)
    assert linear + abs(angular) * 0.172 / 2 <= 0.5
    assert linear - abs(angular) * 0.172 / 2 >= 0.05
    with pytest.raises(ValueError):
        command_for_heading(0.7, max_linear_speed=0.5)


def test_close_obstacle_is_not_ignored():
    assert choose_heading(0.0, 1.0, [[0.04, 0.0, 0.2]]) != pytest.approx(0.0)


def test_choose_heading_returns_none_for_wall_blocking_all_candidates():
    points = np.array(
        [[x, y, 0.2] for x in (0.25, 0.5, 0.8) for y in np.linspace(-1.0, 1.0, 21)],
        dtype=np.float32,
    )
    assert choose_heading(0.0, 2.0, points, rover_half_width=0.2) is None


def test_search_step_stops_before_collision_with_extra_margin():
    path = [(float(distance), 0.0) for distance in np.arange(0.0, 1.425, 0.025)]
    candidates = [{"heading": 0.0, "clear": False, "path": path}]
    step = choose_search_step(candidates, [[1.2, 0.0, 0.2]], 0.18)
    assert step is not None
    assert 0.6 < step["distance"] < 0.75
    assert step["point"] == pytest.approx((step["distance"], 0.0))
    assert step["path"][-1] == step["point"]


def test_search_step_caps_travel_and_refuses_nearby_obstacle():
    path = [(float(distance), 0.0) for distance in np.arange(0.0, 1.825, 0.025)]
    candidates = [{"heading": 0.0, "clear": False, "path": path}]
    assert choose_search_step(candidates, [[1.6, 0.0, 0.2]], 0.18)["distance"] <= 1.0
    assert choose_search_step(candidates, [[0.4, 0.0, 0.2]], 0.18) is None


def test_choose_heading_ignores_points_outside_obstacle_height_band():
    points = np.array([[0.5, 0.0, 1.5]], dtype=np.float32)
    assert choose_heading(0.0, 2.0, points) == pytest.approx(0.0)


def test_choose_heading_rejects_bad_point_shape():
    with pytest.raises(ValueError):
        choose_heading(0.0, 1.0, np.zeros((3, 2)))


def test_filtered_yaw_averages_single_count_wobble():
    # One encoder count of left/right mismatch is 0.01 / 0.172 = 0.0581 rad.
    count = 0.01 / 0.172
    out = filtered_yaw(0.0, count, 0.05, 0.3)
    assert abs(out) < abs(count) * 0.5
    # Sustained rotation is tracked rather than ignored.
    yaw = 0.0
    for _ in range(40):
        yaw = filtered_yaw(yaw, count, 0.05, 0.3)
    assert yaw == pytest.approx(count, abs=0.005)


def test_filtered_yaw_handles_first_sample_and_wrapping():
    assert filtered_yaw(None, 0.4, 0.05, 0.3) == pytest.approx(0.4)
    assert filtered_yaw(0.4, 0.4, float("nan"), 0.3) == pytest.approx(0.4)
    wrapped = filtered_yaw(math.pi - 0.02, -math.pi + 0.02, 0.05, 0.3)
    assert abs(abs(wrapped) - math.pi) < 0.02


def test_filtered_yaw_rejects_bad_parameters():
    with pytest.raises(ValueError):
        filtered_yaw(0.0, 0.1, 0.05, 0.0)
    with pytest.raises(ValueError):
        filtered_yaw(0.0, float("nan"), 0.05, 0.3)


def test_turn_command_keeps_both_wheels_forward():
    linear, angular = command_for_heading(0.15, max_linear_speed=0.35, min_angular_speed=0.5)
    assert linear == pytest.approx(0.35)
    assert angular == pytest.approx(0.5)
    assert linear - angular * 0.172 / 2 > 0.05
    assert linear + angular * 0.172 / 2 <= 0.5
    assert command_for_heading(0.15, max_linear_speed=0.35, max_angular_speed=0.3, min_angular_speed=0.5)[1] == pytest.approx(0.3)
    assert command_for_heading(0.6, max_linear_speed=0.35, min_angular_speed=0.5)[1] == pytest.approx(0.65)


def test_command_rejects_negative_min_angular_speed():
    with pytest.raises(ValueError):
        command_for_heading(0.5, min_angular_speed=-0.1)


def test_command_turns_forward_until_path_is_aligned():
    assert command_for_heading(0.5, max_linear_speed=0.35) == pytest.approx((0.35, 0.6))
    assert command_for_heading(0.05, max_linear_speed=0.35) == pytest.approx((0.35, 0.0))


def test_command_hysteresis_keeps_turning_until_well_aligned():
    # Between the release band (0.06) and the align tolerance (0.12) the old
    # single-threshold behaviour chattered every tick because wheel-count yaw
    # moves in ~3.3 degree steps.
    assert command_for_heading(0.10, max_linear_speed=0.35) == pytest.approx((0.35, 0.0))
    linear, angular = command_for_heading(0.10, max_linear_speed=0.35, was_aligning=True)
    assert linear == pytest.approx(0.35)
    assert angular == pytest.approx(0.12)
    assert command_for_heading(0.05, max_linear_speed=0.35, was_aligning=True) == pytest.approx((0.35, 0.0))


def test_command_rejects_bad_release_ratio():
    with pytest.raises(ValueError):
        command_for_heading(0.5, release_ratio=1.0)
    with pytest.raises(ValueError):
        command_for_heading(0.5, release_ratio=float("nan"))


def test_command_respects_angular_speed_limit():
    assert command_for_heading(3.0, max_linear_speed=0.35, max_angular_speed=0.4) == pytest.approx((0.35, 0.4))