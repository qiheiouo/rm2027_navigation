"""Default-disabled bundle boundary; no driver, adapter, TF owner or motion output."""
import math
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def validate_config(path):
    params = yaml.safe_load(Path(path).read_text())["/**"]["ros__parameters"]
    common, mapping = params["common"], params["mapping"]
    if (common["multi_lidar"] is not True or common["lid_topic"] != "/livox/left/lidar"
            or common["lid_topic2"] != "/livox/right/lidar"
            or common["imu_topic"] != "/livox/lio_imu"):
        raise RuntimeError("candidate requires two canonical clouds and exactly one selected IMU")
    if mapping["extrinsic_imu_to_lidars"] is not True or mapping["extrinsic_est_en"] is not False:
        raise RuntimeError("candidate requires measured fixed IMU-to-both-LiDAR transforms")
    for key in ("extrinsic_T", "extrinsic_R", "extrinsic_T2", "extrinsic_R2",
                "extrinsic_T_L2_wrt_L1", "extrinsic_R_L2_wrt_L1",
                "extrinsic_T_L1_wrt_drone", "extrinsic_R_L1_wrt_drone"):
        values = mapping.get(key)
        size = 3 if key.startswith("extrinsic_T") else 9
        if (not isinstance(values, list) or len(values) != size
                or any(type(v) is not float or not math.isfinite(v) for v in values)):
            raise RuntimeError(f"{key}: missing finite measured ROS double transform ({size} decimal values)")
        if size == 9:
            rows = [values[i:i+3] for i in (0, 3, 6)]
            for i in range(3):
                for j in range(3):
                    if abs(sum(rows[i][k]*rows[j][k] for k in range(3))-(i == j)) > 1e-6:
                        raise RuntimeError(f"{key}: rotation is not orthonormal")
            det = (values[0]*(values[4]*values[8]-values[5]*values[7])
                   - values[1]*(values[3]*values[8]-values[5]*values[6])
                   + values[2]*(values[3]*values[7]-values[4]*values[6]))
            if abs(det-1) > 1e-6:
                raise RuntimeError(f"{key}: rotation must have determinant +1")
    # Backend computes L1<-L2 from the two common-IMU transforms in this mode.
    # Reject contradictory manually recorded L1<-L2 calibration as well.
    r1, r2 = mapping["extrinsic_R"], mapping["extrinsic_R2"]
    expected_r = [sum(r1[k*3+i]*r2[k*3+j] for k in range(3)) for i in range(3) for j in range(3)]
    delta = [mapping["extrinsic_T2"][i]-mapping["extrinsic_T"][i] for i in range(3)]
    expected_t = [sum(r1[k*3+i]*delta[k] for k in range(3)) for i in range(3)]
    if (any(abs(a-b)>1e-6 for a,b in zip(expected_r,mapping["extrinsic_R_L2_wrt_L1"]))
            or any(abs(a-b)>1e-6 for a,b in zip(expected_t,mapping["extrinsic_T_L2_wrt_L1"]))):
        raise RuntimeError("L1<-L2 must equal inverse(IMU<-L1) * (IMU<-L2)")


def _launch_setup(context):
    if LaunchConfiguration("enable_experiment").perform(context).lower() != "true":
        return [LogInfo(msg="dual candidate disabled: zero nodes, no hardware output")]
    config = LaunchConfiguration("config_file").perform(context).strip()
    revision = LaunchConfiguration("calibration_revision").perform(context).strip()
    if not config or not revision:
        raise RuntimeError("explicit config_file and calibration_revision required; template is uncalibrated")
    validate_config(config)
    share = Path(get_package_share_directory("rm_lio_bringup"))
    return [LogInfo(msg=f"EXPERIMENTAL backend only, calibration={revision}; not a navigation profile"),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/"launch/fast_lio_multi_phase2a.launch.py")),
                launch_arguments={"use_backend":"true", "sensor_mode":"dual", "update_method":"bundle",
                    "config_file":config, "publish_tf_results":"false", "scan_publish_en":"false",
                    "use_sim_time":LaunchConfiguration("use_sim_time")}.items())]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("enable_experiment", default_value="false"),
        DeclareLaunchArgument("config_file", default_value=""),
        DeclareLaunchArgument("calibration_revision", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        OpaqueFunction(function=_launch_setup)])
