from rm_referee_interface.validation import RefereeSnapshot, validate_snapshot


def make_snapshot(**overrides):
    values = {
        "stamp_sec": 100.0,
        "valid": True,
        "game_progress": 4,
        "stage_remain_time": 300,
        "robot_id": 7,
        "current_hp": 400,
        "self_outpost_hp": 1500,
        "enemy_outpost_hp": 1500,
        "projectile_allowance_17mm": 100,
        "remaining_gold_coin": 0,
    }
    values.update(overrides)
    return RefereeSnapshot(**values)


def validate(snapshot):
    return validate_snapshot(snapshot, 100.1, 0.5, 900, 10000)


def test_accepts_fresh_valid_state():
    assert validate(make_snapshot()) == (True, "accepted")


def test_rejects_stale_or_future_state():
    assert validate(make_snapshot(stamp_sec=99.0))[0] is False
    assert validate(make_snapshot(stamp_sec=101.0))[0] is False


def test_rejects_invalid_source_and_ranges():
    assert validate(make_snapshot(valid=False))[0] is False
    assert validate(make_snapshot(game_progress=6))[0] is False
    assert validate(make_snapshot(current_hp=10001))[0] is False
