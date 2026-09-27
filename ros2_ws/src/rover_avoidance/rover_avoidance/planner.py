"""Pure depth and local steering helpers used by the avoidance node."""

import math

import cv2
import numpy as np

from rover_bridge.protocol import depth_png_bytes


def quaternion_rotate(vector, quaternion):
    """Rotate a 3D vector by an (x, y, z, w) quaternion."""
    q = np.asarray(quaternion, dtype=np.float64)
    v = np.asarray(vector, dtype=np.float64)
    if q.shape != (4,) or v.shape[-1] != 3 or not np.all(np.isfinite(q)):
        raise ValueError("invalid vector or quaternion")
    norm = np.linalg.norm(q)
    if norm < 1e-12:
        raise ValueError("zero-length quaternion")
    q /= norm
    qv = q[:3]
    # Equivalent to q * [v, 0] * conjugate(q), vectorized over points.
    return v + 2.0 * np.cross(qv, np.cross(qv, v) + q[3] * v)


def depth_to_base_points(
    depth,
    camera_info,
    translation,
    rotation,
    decimation=8,
    min_depth_m=0.2,
    max_depth_m=3.0,
    unknown_range_m=1.2,
    missing_depth_blocks_paths=False,
):
    """Project valid depth and unknown rays into base_link XYZ points.

    Pixels without a measurement (zero, or farther than ``max_depth_m``) are
    stereo dropouts, not detected obstacles. By default they contribute no
    points, so only measured returns can block a candidate path. Set
    ``missing_depth_blocks_paths`` to reproduce the earlier conservative
    policy, where each missing ray is sampled at a fixed horizontal distance
    and the planner declines to drive through unobserved space.

    A zero reading is never evidence of free space: a textureless or
    absorptive obstacle can return nothing while being physically present, so
    the rover's speed limit and the bridge command watchdog remain the
    safeguards for that case.
    """
    if depth.ndim != 2 or depth.dtype != np.uint16:
        raise ValueError("depth image must be a uint16 single-channel image")
    if len(camera_info.k) != 9:
        raise ValueError("camera info K must contain nine values")
    if decimation < 1 or not 0.0 < min_depth_m < max_depth_m:
        raise ValueError("invalid depth projection settings")
    if not np.isfinite(unknown_range_m) or unknown_range_m <= 0.0:
        raise ValueError("unknown ray range must be positive")

    height, width = depth.shape
    info_width = float(camera_info.width or width)
    info_height = float(camera_info.height or height)
    fx = float(camera_info.k[0]) * width / info_width
    fy = float(camera_info.k[4]) * height / info_height
    cx = float(camera_info.k[2]) * width / info_width
    cy = float(camera_info.k[5]) * height / info_height
    if not all(np.isfinite(value) and value > 0.0 for value in (fx, fy)):
        raise ValueError("camera info has invalid focal lengths")

    rows = np.arange(0, height, decimation, dtype=np.float32)
    columns = np.arange(0, width, decimation, dtype=np.float32)
    column_grid, row_grid = np.meshgrid(columns, rows)
    sampled = depth[::decimation, ::decimation]
    z = sampled.astype(np.float32) / 1000.0
    valid = np.isfinite(z) & (z >= min_depth_m) & (z <= max_depth_m)
    valid_count = int(np.count_nonzero(valid))
    # A sparse or missing depth image is unknown, never evidence of free space.
    roi = sampled[max(0, sampled.shape[0] // 4):, :]
    roi_valid = (roi >= min_depth_m * 1000.0) & (roi <= max_depth_m * 1000.0)
    valid_fraction = float(np.count_nonzero(roi_valid)) / max(roi.size, 1)

    translation = np.asarray(translation, dtype=np.float64)
    if translation.shape != (3,) or not np.all(np.isfinite(translation)):
        raise ValueError("camera translation must contain three finite values")

    base = np.empty((0, 3), dtype=np.float32)
    if valid_count:
        valid_z = z[valid]
        optical = np.column_stack((
            (column_grid[valid] - cx) * valid_z / fx,
            (row_grid[valid] - cy) * valid_z / fy,
            valid_z,
        ))
        base = quaternion_rotate(optical, rotation) + translation
        base = base.astype(np.float32, copy=False)

    unknown = ~valid if missing_depth_blocks_paths else np.zeros_like(valid)
    unknown_base = np.empty((0, 3), dtype=np.float32)
    if np.any(unknown):
        rays = np.column_stack((
            (column_grid[unknown] - cx) / fx,
            (row_grid[unknown] - cy) / fy,
            np.ones(int(np.count_nonzero(unknown)), dtype=np.float32),
        ))
        rays = quaternion_rotate(rays, rotation)
        horizontal_norm = np.hypot(rays[:, 0], rays[:, 1])
        ray_valid = horizontal_norm > 1e-6
        rays = rays[ray_valid]
        horizontal_norm = horizontal_norm[ray_valid]
        if rays.size:
            sample_step = 0.1
            distances = np.arange(
                sample_step,
                float(unknown_range_m) + sample_step * 0.5,
                sample_step,
                dtype=np.float32,
            )
            if distances.size == 0 or distances[-1] < unknown_range_m:
                distances = np.append(distances, float(unknown_range_m))
            scale = distances[None, :] / horizontal_norm[:, None]
            samples = rays[:, None, :] * scale[:, :, None] + translation
            unknown_base = samples.reshape(-1, 3).astype(np.float32, copy=False)

    return base, unknown_base, valid_fraction


def obstacle_points_in_view(points, rover_half_width, obstacle_min_height, obstacle_max_height, lookahead_m):
    points = np.asarray(points, dtype=np.float32)
    if points.size == 0:
        points = np.empty((0, 3), dtype=np.float32)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("obstacle points must have shape (N, 3)")
    return points[
        (points[:, 0] > 0.0)
        & (points[:, 2] >= obstacle_min_height)
        & (points[:, 2] <= obstacle_max_height)
        & (np.hypot(points[:, 0], points[:, 1]) <= lookahead_m + rover_half_width)
    ]


def choose_heading(
    goal_heading,
    goal_distance,
    obstacle_points,
    rover_half_width=0.22,
    obstacle_min_height=0.04,
    obstacle_max_height=0.75,
    lookahead_m=1.2,
    max_heading=0.9,
    unknown_points=None,
    steering=None,
    candidates_debug=None,
    considered_points=None,
):
    """Choose the least-deviating collision-free local path.

    Points use base_link convention (x forward, y left, z up). Returns a
    heading relative to the rover, or None if each candidate corridor is
    blocked. This intentionally stops at complex obstacles rather than
    pretending to plan a complete route around them.
    """
    if not math.isfinite(goal_heading) or not math.isfinite(goal_distance):
        raise ValueError("goal pose must be finite")
    obstacles = obstacle_points_in_view(
        obstacle_points, rover_half_width, obstacle_min_height, obstacle_max_height, lookahead_m,
    )
    unknown = np.asarray(
        np.empty((0, 3), dtype=np.float32)
        if unknown_points is None else unknown_points,
        dtype=np.float32,
    )
    if unknown.size == 0:
        unknown = np.empty((0, 3), dtype=np.float32)
    if unknown.ndim != 2 or unknown.shape[1] != 3:
        raise ValueError("unknown points must have shape (N, 3)")
    unknown = unknown[
        (unknown[:, 0] > 0.0)
        & (unknown[:, 2] >= obstacle_min_height)
        & (unknown[:, 2] <= obstacle_max_height)
        & (np.hypot(unknown[:, 0], unknown[:, 1]) <= lookahead_m + rover_half_width)
    ]
    if considered_points is not None:
        considered_points.extend(obstacles.tolist())
    candidate_offsets = (0.0, -0.35, 0.35, -0.7, 0.7, -0.9, 0.9)
    candidates = []
    path_length = min(max(goal_distance, 0.0), lookahead_m)
    for offset in candidate_offsets:
        heading = float(np.clip(goal_heading + offset, -max_heading, max_heading))
        if steering is not None:
            linear, angular = command_for_heading(heading, **steering)
            path_points = np.concatenate((obstacles, unknown))
            path = [] if candidates_debug is not None else None
            collision = [] if candidates_debug is not None else None
            clear = arc_is_clear(
                path_points, heading, linear, angular,
                path_length + rover_half_width, rover_half_width,
                path_samples=path, collision=collision,
            )
            if candidates_debug is not None:
                candidates_debug.append({
                    "heading": heading,
                    "linear": linear,
                    "angular": angular,
                    "clear": clear,
                    "path": path,
                    "blocked_point": collision[0] if collision else None,
                })
            if not clear:
                continue
            candidates.append((abs(heading - goal_heading) + 0.03 * abs(heading), heading))
            continue
        forward = obstacles[:, 0] * math.cos(heading) + obstacles[:, 1] * math.sin(heading)
        lateral = -obstacles[:, 0] * math.sin(heading) + obstacles[:, 1] * math.cos(heading)
        blocked = np.any(
            (forward > 0.0)
            & (forward < path_length + rover_half_width)
            & (np.abs(lateral) < rover_half_width)
        )
        unknown_forward = (
            unknown[:, 0] * math.cos(heading)
            + unknown[:, 1] * math.sin(heading)
        )
        unknown_lateral = (
            -unknown[:, 0] * math.sin(heading)
            + unknown[:, 1] * math.cos(heading)
        )
        unknown_crosses_path = np.any(
            (unknown_forward > 0.0)
            & (unknown_forward < path_length + rover_half_width)
            & (np.abs(unknown_lateral) < rover_half_width)
        )
        blocked = blocked or unknown_crosses_path
        if not blocked:
            # Deviation from the goal is the primary score; small turns are preferred.
            candidates.append((abs(heading - goal_heading) + 0.03 * abs(heading), heading))
    if not candidates:
        return None
    return min(candidates)[1]


def arc_is_clear(points, heading, linear, angular, length, half_width, path_samples=None, collision=None):
    """Check the footprint along a forward arc followed by its tangent."""
    if linear <= 0.0:
        return False
    distance_to_turn = (
        min(length, abs(heading * linear / angular)) if angular else 0.0
    )
    radius = linear / angular if angular else 0.0
    clear = True
    for distance in np.linspace(0.0, length, max(2, math.ceil(length / 0.025) + 1)):
        turned = min(distance, distance_to_turn)
        angle = turned * angular / linear
        x = radius * math.sin(angle) + (distance - turned) * math.cos(angle)
        y = radius * (1.0 - math.cos(angle)) + (distance - turned) * math.sin(angle)
        if path_samples is not None:
            path_samples.append((float(x), float(y)))
        hits = (points[:, 0] - x) ** 2 + (points[:, 1] - y) ** 2 < (half_width + 0.025) ** 2
        if np.any(hits):
            if clear and collision is not None:
                collision.append(tuple(float(value) for value in points[np.flatnonzero(hits)[0]]))
            clear = False
            if path_samples is None:
                return False
    return clear


def choose_search_step(candidates, obstacle_points, rover_half_width, max_step_m=1.0, stop_margin_m=0.3):
    """Find a bounded clear prefix of a rejected path for preview only."""
    if not all(math.isfinite(value) and value > 0.0 for value in (rover_half_width, max_step_m, stop_margin_m)):
        raise ValueError("search distances must be finite and positive")
    points = np.asarray(obstacle_points, dtype=np.float32)
    if points.size == 0:
        points = np.empty((0, 3), dtype=np.float32)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("obstacle points must have shape (N, 3)")
    radius = rover_half_width + 0.025
    best = None
    for candidate in candidates:
        path = candidate["path"]
        if len(path) < 2:
            continue
        distances = [0.0]
        for previous, current in zip(path, path[1:]):
            distances.append(distances[-1] + math.dist(previous, current))
        first_hit = next((
            distance for (x, y), distance in zip(path, distances)
            if np.any((points[:, 0] - x) ** 2 + (points[:, 1] - y) ** 2 < radius ** 2)
        ), distances[-1] + stop_margin_m)
        safe_distance = min(max_step_m, first_hit - stop_margin_m)
        last_index = max((index for index, distance in enumerate(distances) if distance <= safe_distance), default=0)
        if distances[last_index] < 0.15:
            continue
        step = {
            "heading": candidate["heading"],
            "distance": distances[last_index],
            "point": path[last_index],
            "path": path[:last_index + 1],
        }
        if best is None or (step["distance"], -abs(step["heading"])) > (best["distance"], -abs(best["heading"])):
            best = step
    return best


def relative_goal_to_odom(x, y, odom_x, odom_y, yaw):
    """Freeze a base_link-relative planar goal in the odom frame."""
    values = (x, y, odom_x, odom_y, yaw)
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("goal and odometry values must be finite")
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return (
        odom_x + cosine * x - sine * y,
        odom_y + sine * x + cosine * y,
    )


def normalize_angle(angle):
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def filtered_yaw(previous, measured, dt, time_constant_s):
    """Low-pass a wheel-derived yaw.

    The bridge integrates wheel encoders whose distance unit is 1 cm, so a
    single count of left/right mismatch is about 3.3 degrees of yaw on this
    rover. That quantization makes the raw estimate random-walk across the
    steering tolerance, which reads as a jerk. This exponential filter keeps
    sustained rotation while averaging the count-sized wobble.
    """
    if not math.isfinite(float(measured)):
        raise ValueError("measured yaw must be finite")
    if not math.isfinite(float(time_constant_s)) or float(time_constant_s) <= 0.0:
        raise ValueError("yaw filter time constant must be positive")
    if previous is None or not math.isfinite(float(previous)):
        return normalize_angle(float(measured))
    if not math.isfinite(float(dt)) or float(dt) <= 0.0:
        return normalize_angle(float(previous))
    alpha = 1.0 - math.exp(-float(dt) / float(time_constant_s))
    return normalize_angle(float(previous) + alpha * normalize_angle(float(measured) - float(previous)))


def command_for_heading(
    heading,
    max_linear_speed=0.12,
    max_angular_speed=0.65,
    heading_gain=1.2,
    align_tolerance=0.12,
    release_ratio=0.5,
    was_aligning=False,
    min_angular_speed=0.0,
    wheel_separation_m=0.172,
    max_wheel_speed_mps=0.5,
    min_wheel_speed_mps=0.05,
):
    """Return a forward arc or straight-forward command for a checked path.

    ``wheel-telemetry`` yaw arrives quantized in individual encoder counts
    (one count on this rover is about 3.3 degrees), so a single threshold makes
    the controller chatter between turning and driving. ``was_aligning`` adds
    hysteresis: a rover that is already turning keeps turning until the heading
    is inside ``align_tolerance * release_ratio``, while a driving rover starts
    turning once the heading exceeds ``align_tolerance``.

    Turning retains forward speed so neither side has to reverse or scrub in
    place. ``min_angular_speed`` preserves a decisive turn instead of a small
    proportional correction; stop turning by ending the request.
    """
    values = (heading, max_linear_speed, max_angular_speed, heading_gain, align_tolerance)
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("command settings must be finite")
    if max_linear_speed <= 0.0 or max_angular_speed <= 0.0 or heading_gain <= 0.0:
        raise ValueError("command speeds and gain must be positive")
    if align_tolerance < 0.0:
        raise ValueError("align tolerance must not be negative")
    if not math.isfinite(float(release_ratio)) or not 0.0 <= float(release_ratio) < 1.0:
        raise ValueError("release_ratio must be finite and within [0, 1)")
    if not math.isfinite(float(min_angular_speed)) or float(min_angular_speed) < 0.0:
        raise ValueError("min_angular_speed must be finite and not negative")
    if not all(math.isfinite(float(value)) for value in (wheel_separation_m, max_wheel_speed_mps, min_wheel_speed_mps)):
        raise ValueError("wheel settings must be finite")
    if wheel_separation_m <= 0.0 or min_wheel_speed_mps < 0.0 or not min_wheel_speed_mps < max_linear_speed < max_wheel_speed_mps:
        raise ValueError("forward speed must keep both wheels above the minimum and below the maximum")
    tolerance = (
        align_tolerance * float(release_ratio) if was_aligning else align_tolerance
    )
    if abs(heading) > tolerance:
        angular = float(np.clip(heading_gain * heading, -max_angular_speed, max_angular_speed))
        floor = min(float(min_angular_speed), float(max_angular_speed))
        if floor > 0.0 and abs(angular) < floor:
            angular = math.copysign(floor, heading)
        angular_limit = min(
            2.0 * (max_linear_speed - min_wheel_speed_mps) / wheel_separation_m,
            2.0 * (max_wheel_speed_mps - max_linear_speed) / wheel_separation_m,
        )
        angular = float(np.clip(angular, -angular_limit, angular_limit))
        return float(max_linear_speed), angular
    return float(max_linear_speed), 0.0


def decode_depth_message(message):
    """Decode compressedDepth PNG to its uint16 depth image."""
    png = depth_png_bytes(bytes(message.data))
    depth = cv2.imdecode(np.frombuffer(png, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
    if depth is None or depth.ndim != 2 or depth.dtype != np.uint16:
        raise ValueError("compressed depth did not decode to a uint16 image")
    return depth