from types import SimpleNamespace

from rm_system_monitor.hardware_acceptance import (
    BLOCKED,
    collect_preflight,
    evaluate_topic,
    FAIL,
    PASS,
    TopicStats,
)


def message(stamp_ns, valid=True):
    stamp = SimpleNamespace(
        sec=stamp_ns // 1_000_000_000,
        nanosec=stamp_ns % 1_000_000_000,
    )
    return SimpleNamespace(header=SimpleNamespace(stamp=stamp), valid=valid)


def rule(required=True):
    return {
        "step": 1,
        "name": "/fixture/status",
        "type": "fixture/msg/Status",
        "min_rate_hz": 5.0,
        "max_age_sec": 0.5,
        "max_publishers": 1,
        "required": required,
        "fields": {"valid": True},
    }


def publishers(count=1):
    return [{"gid": str(index)} for index in range(count)]


def test_healthy_topic_passes_rate_stamp_and_field_checks():
    stats = TopicStats()
    for index in range(10):
        stats.record(
            message(10_000_000_000 + index * 100_000_000),
            "one",
            ["valid"],
            20.0 + index * 0.1,
        )
    result = evaluate_topic(
        rule(), stats, publishers(), ["fixture/msg/Status"], 11_000_000_000
    )
    assert result["status"] == PASS
    assert result["metrics"]["rate_hz"] > 9.0


def test_timestamp_regression_and_duplicate_publishers_fail():
    stats = TopicStats()
    stats.record(message(10_000_000_000), "one", ["valid"], 20.0)
    stats.record(message(9_000_000_000), "two", ["valid"], 20.1)
    result = evaluate_topic(
        rule(), stats, publishers(2), ["fixture/msg/Status"], 10_000_000_000
    )
    assert result["status"] == FAIL
    assert result["metrics"]["timestamp_regressions"] == 1
    assert any("publisher count" in reason for reason in result["reasons"])


def test_missing_optional_topic_warns_but_required_topic_blocks():
    missing_required = evaluate_topic(rule(), TopicStats(), [], [], 0)
    missing_optional = evaluate_topic(rule(False), TopicStats(), [], [], 0)
    assert missing_required["status"] == BLOCKED
    assert missing_optional["status"] != BLOCKED


def test_preflight_reports_candidate_gates_without_hiding_missing_files(tmp_path):
    (tmp_path / "required.txt").write_text("present\n")
    (tmp_path / "candidate.yaml").write_text(
        "hardware_motion_allowed: false\nformal_profile_available: false\n"
    )
    plan = {
        "preflight": {
            "files": ["required.txt", "missing.txt"],
            "packages": [],
            "candidate_manifest": "candidate.yaml",
            "pending_gates": [
                {
                    "path": "hardware_motion_allowed",
                    "expected": True,
                    "blocks_before_step": 7,
                    "reason": "motion remains disabled",
                }
            ],
        }
    }
    report = collect_preflight(plan, tmp_path)
    assert report["decision"] == "STOP"
    assert report["next_blocked_step"] == 7
    assert any(
        item["name"] == "missing.txt" and item["status"] == FAIL
        for item in report["checks"]
    )
