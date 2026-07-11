from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


TRUE_VALUES = {"1", "true", "yes", "on"}


def _mapping_setup(context, *args, **kwargs):
    del args, kwargs
    enabled = (
        LaunchConfiguration("enable_mapping").perform(context).strip().lower()
        in TRUE_VALUES
    )
    if not enabled:
        return [
            LogInfo(msg=(
                "[mapping] enable_mapping:=false; no OctoMap server or map "
                "exporter was started."
            ))
        ]

    octomap_params = LaunchConfiguration("octomap_params")
    pointcloud_topic = LaunchConfiguration("pointcloud_topic")
    sampled_pointcloud_topic = LaunchConfiguration("sampled_pointcloud_topic")
    registered_cloud_topic = LaunchConfiguration("registered_cloud_topic")
    occupancy_topic = LaunchConfiguration("occupancy_topic")
    map_frame = LaunchConfiguration("map_frame")
    base_frame = LaunchConfiguration("base_frame")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return [
        LogInfo(msg=[
            "[mapping] Managed mapping enabled. OctoMap input=",
            pointcloud_topic,
            ", registered cloud=",
            registered_cloud_topic,
            ". This launch owns no driver, LIO, Nav2, serial, referee, or BT.",
        ]),
        Node(
            package="rm_map_tools",
            executable="pointcloud_sampler_node",
            name="mapping_pointcloud_sampler",
            output="screen",
            parameters=[{
                "input_topic": pointcloud_topic,
                "output_topic": sampled_pointcloud_topic,
                "recording_status_topic": "/mapping/recording",
                "output_rate": ParameterValue(
                    LaunchConfiguration("octomap_input_rate"), value_type=float
                ),
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            }],
        ),
        Node(
            package="octomap_server",
            executable="octomap_server_node",
            name="octomap_server",
            output="screen",
            parameters=[
                octomap_params,
                {
                    "frame_id": map_frame,
                    "base_frame_id": base_frame,
                    "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                },
            ],
            remappings=[
                ("cloud_in", sampled_pointcloud_topic),
                ("projected_map", occupancy_topic),
                ("octomap_binary", "/mapping/octomap_binary"),
                ("octomap_full", "/mapping/octomap_full"),
                ("octomap_point_cloud_centers", "/mapping/octomap_point_cloud_centers"),
                ("occupied_cells_vis_array", "/mapping/occupied_cells_vis_array"),
                ("free_cells_vis_array", "/mapping/free_cells_vis_array"),
            ],
        ),
        Node(
            package="rm_map_tools",
            executable="mapping_session_node",
            name="mapping_session",
            output="screen",
            parameters=[{
                "registered_cloud_topic": registered_cloud_topic,
                "occupancy_topic": occupancy_topic,
                "map_frame": map_frame,
                "output_root": LaunchConfiguration("output_root"),
                "map_id": LaunchConfiguration("map_id"),
                "revision": LaunchConfiguration("revision"),
                "voxel_size": ParameterValue(
                    LaunchConfiguration("pcd_voxel_size"), value_type=float
                ),
                "sample_period_sec": ParameterValue(
                    LaunchConfiguration("sample_period_sec"), value_type=float
                ),
                "min_observations": ParameterValue(
                    LaunchConfiguration("min_observations"), value_type=int
                ),
                "autostart": ParameterValue(
                    LaunchConfiguration("autostart"), value_type=bool
                ),
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            }],
        ),
    ]


def generate_launch_description():
    default_octomap_params = PathJoinSubstitution([
        FindPackageShare("rm_map_tools"),
        "config",
        "mapping_octomap.yaml",
    ])
    return LaunchDescription([
        DeclareLaunchArgument(
            "enable_mapping",
            default_value="false",
            description="Explicit safety gate for the managed mapping backends.",
        ),
        DeclareLaunchArgument(
            "pointcloud_topic",
            default_value="/points/obstacles",
            description="Sensor-frame PointCloud2 used by OctoMap for occupied/free rays.",
        ),
        DeclareLaunchArgument(
            "registered_cloud_topic",
            default_value="/lio/cloud_registered",
            description="World-registered PointCloud2 accumulated into the PCD artifact.",
        ),
        DeclareLaunchArgument(
            "sampled_pointcloud_topic", default_value="/mapping/sensor_cloud"
        ),
        DeclareLaunchArgument("octomap_input_rate", default_value="5.0"),
        DeclareLaunchArgument(
            "occupancy_topic", default_value="/mapping/projected_map"
        ),
        DeclareLaunchArgument("map_frame", default_value="map"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("octomap_params", default_value=default_octomap_params),
        DeclareLaunchArgument("output_root", default_value="/tmp/rm27_maps"),
        DeclareLaunchArgument("map_id", default_value="rm_field"),
        DeclareLaunchArgument("revision", default_value="auto"),
        DeclareLaunchArgument("pcd_voxel_size", default_value="0.05"),
        DeclareLaunchArgument("sample_period_sec", default_value="0.20"),
        DeclareLaunchArgument("min_observations", default_value="2"),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        OpaqueFunction(function=_mapping_setup),
    ])
