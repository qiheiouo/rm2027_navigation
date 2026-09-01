from dataclasses import dataclass
from typing import List, Set, Tuple


@dataclass(frozen=True)
class RequirementPolicy:
    require_lio: bool = True
    require_obstacle_input: bool = True
    require_localization: bool = True
    require_nav2: bool = True
    require_referee: bool = False
    require_chassis_mode: bool = False
    require_serial_transport: bool = False


def evaluate_readiness(
    policy: RequirementPolicy, available: Set[str]
) -> Tuple[bool, bool, List[str]]:
    navigation_requirements = (
        (policy.require_lio, "lio"),
        (policy.require_obstacle_input, "obstacle_input"),
        (policy.require_localization, "global_localization"),
        (policy.require_nav2, "nav2_action"),
        (policy.require_nav2, "local_costmap"),
        (policy.require_nav2, "global_costmap"),
    )
    mission_requirements = (
        (policy.require_referee, "referee_state"),
        (policy.require_chassis_mode, "chassis_authority"),
        (policy.require_serial_transport, "serial_transport"),
    )
    missing_navigation = [
        name for required, name in navigation_requirements if required and name not in available
    ]
    missing_mission = [
        name for required, name in mission_requirements if required and name not in available
    ]
    missing = missing_navigation + missing_mission
    return not missing_navigation, not missing, missing
