from launch import LaunchDescription
from launch.actions import LogInfo


def generate_launch_description():
    # Phase 1 bringup skeleton only.
    #
    # Intentionally not connected here yet:
    # - FAST-LIO backend
    # - real serial hardware
    # - referee interface
    # - competition BT or mission logic
    # - full Nav2 bringup
    #
    # Add includes only after each package skeleton passes build checks.
    return LaunchDescription([
        LogInfo(msg="[phase1_bringup] Skeleton only. No runtime stack is launched yet."),
    ])
