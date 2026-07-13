from rm_system_monitor.readiness import RequirementPolicy, evaluate_readiness


def test_navigation_and_mission_requirements_are_separate():
    policy = RequirementPolicy(require_referee=True, require_chassis_mode=True)
    navigation_ready, mission_ready, missing = evaluate_readiness(
        policy,
        {"lio", "obstacle_input", "global_localization", "nav2_action"},
    )
    assert navigation_ready is True
    assert mission_ready is False
    assert missing == ["referee_state", "chassis_authority"]


def test_all_required_inputs_pass():
    policy = RequirementPolicy(
        require_referee=True,
        require_chassis_mode=True,
        require_serial_transport=True,
    )
    available = {
        "lio",
        "obstacle_input",
        "global_localization",
        "nav2_action",
        "referee_state",
        "chassis_authority",
        "serial_transport",
    }
    assert evaluate_readiness(policy, available) == (True, True, [])
