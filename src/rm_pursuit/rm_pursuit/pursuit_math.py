from dataclasses import dataclass
import math
from typing import Optional


@dataclass(frozen=True)
class PlanarPoint:
    x: float
    y: float


@dataclass(frozen=True)
class PursuitGoal:
    x: float
    y: float
    yaw: float


def compute_standoff_goal(
    robot: PlanarPoint,
    target: PlanarPoint,
    target_velocity: PlanarPoint,
    standoff_distance: float,
    prediction_horizon_sec: float,
    max_target_distance: float,
) -> Optional[PursuitGoal]:
    values = (
        robot.x,
        robot.y,
        target.x,
        target.y,
        target_velocity.x,
        target_velocity.y,
        standoff_distance,
        prediction_horizon_sec,
        max_target_distance,
    )
    if not all(math.isfinite(value) for value in values):
        return None
    if standoff_distance < 0.0 or prediction_horizon_sec < 0.0:
        return None

    predicted_x = target.x + target_velocity.x * prediction_horizon_sec
    predicted_y = target.y + target_velocity.y * prediction_horizon_sec
    robot_to_target_x = predicted_x - robot.x
    robot_to_target_y = predicted_y - robot.y
    target_distance = math.hypot(robot_to_target_x, robot_to_target_y)
    if target_distance <= 1.0e-6 or target_distance > max_target_distance:
        return None

    travel = max(0.0, target_distance - standoff_distance)
    unit_x = robot_to_target_x / target_distance
    unit_y = robot_to_target_y / target_distance
    goal_x = robot.x + unit_x * travel
    goal_y = robot.y + unit_y * travel
    yaw = math.atan2(predicted_y - goal_y, predicted_x - goal_x)
    return PursuitGoal(goal_x, goal_y, yaw)
