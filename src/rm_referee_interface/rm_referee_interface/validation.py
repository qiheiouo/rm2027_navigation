from dataclasses import dataclass
import math
from typing import Tuple


@dataclass(frozen=True)
class RefereeSnapshot:
    stamp_sec: float
    valid: bool
    game_progress: int
    stage_remain_time: int
    robot_id: int
    current_hp: int
    self_outpost_hp: int
    enemy_outpost_hp: int
    projectile_allowance_17mm: int
    remaining_gold_coin: int


def validate_snapshot(
    snapshot: RefereeSnapshot,
    now_sec: float,
    max_source_age_sec: float,
    max_stage_time_sec: int,
    max_hp: int,
) -> Tuple[bool, str]:
    numeric = (snapshot.stamp_sec, now_sec, max_source_age_sec)
    if not all(math.isfinite(value) for value in numeric):
        return False, "non-finite time"
    if snapshot.stamp_sec <= 0.0:
        return False, "zero timestamp"
    if not snapshot.valid:
        return False, "source marked invalid"
    age = now_sec - snapshot.stamp_sec
    if age < -max_source_age_sec:
        return False, "timestamp is too far in the future"
    if age > max_source_age_sec:
        return False, "source state is stale"
    if not 0 <= snapshot.game_progress <= 5:
        return False, "game_progress outside referee range"
    if not 0 <= snapshot.stage_remain_time <= max_stage_time_sec:
        return False, "stage_remain_time outside configured range"
    hp_fields = (
        snapshot.current_hp,
        snapshot.self_outpost_hp,
        snapshot.enemy_outpost_hp,
    )
    if any(value < 0 or value > max_hp for value in hp_fields):
        return False, "HP field outside configured range"
    if not 0 <= snapshot.robot_id <= 255:
        return False, "robot_id outside uint8 range"
    return True, "accepted"
