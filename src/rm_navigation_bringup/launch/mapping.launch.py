from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration


TRUE_VALUES = {"1", "true", "yes", "on"}


def _mapping_guard(context, *args, **kwargs):
    enable_mapping = (
        LaunchConfiguration("enable_mapping").perform(context).strip().lower()
        in TRUE_VALUES
    )
    if enable_mapping:
        raise RuntimeError(
            "No managed mapping backend is integrated yet. FAST-LIO's upstream "
            "PCD saver writes to a build-time source path and does not create a "
            "versioned rm_map_tools bundle, so automatic mapping is intentionally "
            "blocked until a controlled exporter is implemented."
        )

    return [LogInfo(msg=(
        "[mapping] Safe boundary only: no driver, LIO, Nav2, chassis, or mapping "
        "backend was started. Set enable_mapping:=true only after this guard is "
        "replaced by a managed PCD + occupancy bundle pipeline."
    ))]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "enable_mapping",
            default_value="false",
            description="Safety gate. No managed mapping backend exists yet.",
        ),
        OpaqueFunction(function=_mapping_guard),
    ])
