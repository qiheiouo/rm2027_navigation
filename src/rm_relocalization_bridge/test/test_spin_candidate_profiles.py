import os
import re
from pathlib import Path


# ament's APPEND_ENV keeps platform path-list semantics and may prefix an
# otherwise empty variable with ':'. This variable carries one source path.
SOURCE = Path(os.environ["RM_RELOCALIZATION_SOURCE_DIR"].lstrip(os.pathsep))
CONFIG = SOURCE / "config"
BRINGUP = SOURCE.parent / "rm_navigation_bringup" / "launch"


def _value(text: str, key: str) -> str:
    match = re.search(rf"^\s*{re.escape(key)}:\s*([^#\n]+)", text, re.MULTILINE)
    assert match, f"missing YAML key: {key}"
    return match.group(1).strip()


def test_baseline_amcl_profile_is_unchanged_and_conservative():
    baseline = (CONFIG / "amcl_2d.yaml").read_text(encoding="utf-8")
    assert _value(baseline, "alpha4") == "0.2"
    assert _value(baseline, "resample_interval") == "1"
    assert _value(baseline, "recovery_alpha_fast") == "0.0"
    assert _value(baseline, "recovery_alpha_slow") == "0.0"


def test_experimental_profile_changes_only_the_selected_motion_noise_term():
    baseline = (CONFIG / "amcl_2d.yaml").read_text(encoding="utf-8")
    candidate = (
        CONFIG / "amcl_2d_spin_robust_candidate.yaml"
    ).read_text(encoding="utf-8")
    assert _value(candidate, "alpha4") == "0.02"
    assert _value(candidate, "alpha4") != _value(baseline, "alpha4")
    for key in (
        "alpha1",
        "alpha2",
        "alpha3",
        "alpha5",
        "resample_interval",
        "recovery_alpha_fast",
        "recovery_alpha_slow",
    ):
        assert _value(candidate, key) == _value(baseline, key)


def test_deskew_is_opt_in_and_candidate_keeps_ten_hertz():
    baseline = (CONFIG / "pointcloud_to_scan_2d.yaml").read_text(encoding="utf-8")
    candidate = (
        CONFIG / "pointcloud_to_scan_2d_spin_robust_candidate.yaml"
    ).read_text(encoding="utf-8")
    assert "deskew:" not in baseline
    assert _value(candidate, "enabled") == "true"
    assert _value(candidate, "timestamp_interpretation") == "absolute_nanoseconds"
    assert _value(candidate, "reference_time_policy") == "header_stamp"
    assert _value(candidate, "failure_policy") == "drop_frame"
    assert _value(candidate, "max_publish_rate_hz") == "10.0"


def test_normal_launch_defaults_to_baseline_and_candidate_has_own_entry_point():
    baseline_launch = (
        BRINGUP / "old_car_2026_amcl_relocalization.launch.py"
    ).read_text(encoding="utf-8")
    candidate_launch = (
        BRINGUP / "old_car_2026_amcl_spin_candidate.launch.py"
    ).read_text(encoding="utf-8")
    assert '"amcl_2d.yaml"' in baseline_launch
    assert '"pointcloud_to_scan_2d.yaml"' in baseline_launch
    assert "spin_robust_candidate" not in baseline_launch
    assert '"amcl_2d_spin_robust_candidate.yaml"' in candidate_launch
    assert '"pointcloud_to_scan_2d_spin_robust_candidate.yaml"' in candidate_launch
    assert "Serial, controller, mission and Nav2 motion remain disabled" in candidate_launch
