import ast
from pathlib import Path


SOURCE_ROOT = Path(__file__).resolve().parents[2]
MAPPING_NODE = SOURCE_ROOT / "rm_map_tools/rm_map_tools/mapping_session_node.py"
GENERIC_MAPPING = SOURCE_ROOT / "rm_navigation_bringup/launch/mapping.launch.py"
OLD_CAR_VALIDATION = (
    SOURCE_ROOT
    / "rm_navigation_bringup/launch/old_car_2026_validation.launch.py"
)
OLD_CAR_WRAPPER = SOURCE_ROOT / "rm_navigation_launch/launch/old_car_mapping.launch.py"


def _tree(path: Path) -> ast.Module:
    return ast.parse(path.read_text(encoding="utf-8"), filename=str(path))


def _launch_literal_defaults(path: Path) -> dict[str, object]:
    defaults = {}
    for node in ast.walk(_tree(path)):
        if not isinstance(node, ast.Call):
            continue
        if not isinstance(node.func, ast.Name) or node.func.id != "DeclareLaunchArgument":
            continue
        if not node.args or not isinstance(node.args[0], ast.Constant):
            continue
        for keyword in node.keywords:
            if keyword.arg == "default_value" and isinstance(keyword.value, ast.Constant):
                defaults[str(node.args[0].value)] = keyword.value.value
    return defaults


def _parameter_literal_defaults(path: Path) -> dict[str, object]:
    defaults = {}
    for node in ast.walk(_tree(path)):
        if not isinstance(node, ast.Call):
            continue
        if not isinstance(node.func, ast.Attribute) or node.func.attr != "declare_parameter":
            continue
        if (
            len(node.args) >= 2
            and isinstance(node.args[0], ast.Constant)
            and isinstance(node.args[1], ast.Constant)
        ):
            defaults[str(node.args[0].value)] = node.args[1].value
    return defaults


def _dict_string_values(path: Path, selected_key: str) -> list[object]:
    values = []
    for node in ast.walk(_tree(path)):
        if not isinstance(node, ast.Dict):
            continue
        for key, value in zip(node.keys, node.values):
            if isinstance(key, ast.Constant) and key.value == selected_key:
                if isinstance(value, ast.Constant):
                    values.append(value.value)
                elif isinstance(value, ast.Name):
                    values.append(value.id)
                elif isinstance(value, ast.Call):
                    if (
                        isinstance(value.func, ast.Name)
                        and value.func.id == "LaunchConfiguration"
                        and value.args
                        and isinstance(value.args[0], ast.Constant)
                    ):
                        values.append(value.args[0].value)
                    else:
                        values.append(
                            value.func.id if isinstance(value.func, ast.Name) else None
                        )
    return values


def test_ray_recording_is_default_off_at_node_and_all_launch_layers():
    node_defaults = _parameter_literal_defaults(MAPPING_NODE)
    assert node_defaults["record_ray_observations"] is False
    assert node_defaults["ray_allow_degraded_save"] is False
    assert _launch_literal_defaults(GENERIC_MAPPING)["record_ray_observations"] == "false"
    assert "ray_allow_degraded_save" not in _launch_literal_defaults(GENERIC_MAPPING)
    assert (
        _launch_literal_defaults(OLD_CAR_VALIDATION)[
            "mapping_record_ray_observations"
        ]
        == "false"
    )

    wrapper_tree = _tree(OLD_CAR_WRAPPER)
    assignments = {
        target.id: node.value.value
        for node in wrapper_tree.body
        if isinstance(node, ast.Assign)
        and len(node.targets) == 1
        and isinstance((target := node.targets[0]), ast.Name)
        and isinstance(node.value, ast.Constant)
    }
    assert assignments["RECORD_RAY_OBSERVATIONS"] is False


def test_generic_mapping_binds_ray_input_to_the_sampled_sensor_cloud():
    assert "sampled_pointcloud_topic" in _dict_string_values(
        GENERIC_MAPPING, "ray_cloud_topic"
    )
    assert "ray_source_frame" in _dict_string_values(
        GENERIC_MAPPING, "ray_source_frame"
    )


def test_old_car_layers_pass_only_the_verified_left_lidar_frame():
    assert "mid360_left_frame" in _dict_string_values(
        OLD_CAR_VALIDATION, "ray_source_frame"
    )
    assert "record_ray_observations" in _dict_string_values(
        OLD_CAR_WRAPPER, "mapping_record_ray_observations"
    )


def test_old_car_layers_expose_bounded_ray_capacity_tuning():
    validation_defaults = _launch_literal_defaults(OLD_CAR_VALIDATION)
    assert validation_defaults["mapping_ray_sample_period_sec"] == "0.50"

    argument_pairs = {
        "ray_sample_period_sec": "mapping_ray_sample_period_sec",
        "ray_min_range": "mapping_ray_min_range",
        "ray_max_range": "mapping_ray_max_range",
        "ray_voxel_size": "mapping_ray_voxel_size",
        "ray_max_frames": "mapping_ray_max_frames",
        "ray_max_rays_per_frame": "mapping_ray_max_rays_per_frame",
        "ray_max_total_rays": "mapping_ray_max_total_rays",
        "ray_max_bytes": "mapping_ray_max_bytes",
    }
    for wrapper_argument, validation_argument in argument_pairs.items():
        assert wrapper_argument in _dict_string_values(
            OLD_CAR_WRAPPER, validation_argument
        )
        assert validation_argument in _dict_string_values(
            OLD_CAR_VALIDATION, wrapper_argument
        )

    wrapper_tree = _tree(OLD_CAR_WRAPPER)
    assignments = {
        target.id: node.value.value
        for node in wrapper_tree.body
        if isinstance(node, ast.Assign)
        and len(node.targets) == 1
        and isinstance((target := node.targets[0]), ast.Name)
        and isinstance(node.value, ast.Constant)
    }
    assert assignments["RAY_SAMPLE_PERIOD_SEC"] == "0.50"
    assert assignments["RAY_MAX_TOTAL_RAYS"] == "10000000"
    assert assignments["RAY_MAX_BYTES"] == "536870912"


def test_sidecar_spool_and_subscription_are_created_only_in_enabled_branch():
    tree = _tree(MAPPING_NODE)
    enabled_branch = next(
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.If)
        and isinstance(node.test, ast.Attribute)
        and isinstance(node.test.value, ast.Name)
        and node.test.value.id == "self"
        and node.test.attr == "_ray_enabled"
    )
    branch_calls = list(ast.walk(enabled_branch))
    assert any(
        isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "RaySidecarRecorder"
        for node in branch_calls
    )
    assert any(
        isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "create_subscription"
        and any(
            isinstance(argument, ast.Attribute)
            and argument.attr == "_ray_cloud_topic"
            for argument in node.args
        )
        for node in branch_calls
    )
