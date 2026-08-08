import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

from rm_navigation_integrity.regression import (
    compare_summaries,
    load_jsonl,
    load_scenario_definition,
    summarize_records,
    write_json,
)


def _record(stamp, state, reasons, jump, agreement):
    return {
        "schema_version": 1,
        "state": state,
        "reasons": reasons,
        "metrics": {
            "stamp_sec": stamp,
            "global_translation_jump_m": jump,
            "scan_map_agreement": agreement,
            "valid_scan_points": 80,
            "scan_points_in_map": 70,
        },
    }


class RegressionTest(unittest.TestCase):
    def test_jsonl_summary_preserves_raw_extrema_and_counts(self):
        records = [
            _record(10.0, "GOOD", [], 0.1, 0.8),
            _record(12.0, "SUSPECT", ["SCAN_MAP_AGREEMENT_LOW"], 0.4, 0.3),
        ]
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "run.metrics.jsonl"
            source.write_text(
                "\n".join(json.dumps(record) for record in records) + "\n",
                encoding="utf-8",
            )
            summary = summarize_records(load_jsonl(source))
        self.assertEqual(summary["evaluation_status"], "RAW_METRICS_ONLY")
        self.assertEqual(summary["state_counts"], {"GOOD": 1, "SUSPECT": 1})
        self.assertEqual(
            summary["reason_counts"], {"SCAN_MAP_AGREEMENT_LOW": 1}
        )
        self.assertAlmostEqual(summary["time_range"]["duration_sec"], 2.0)
        self.assertEqual(
            summary["extrema"]["global_translation_jump_m"]["max"], 0.4
        )
        self.assertEqual(summary["extrema"]["scan_map_agreement"]["min"], 0.3)

    def test_comparison_reports_delta_without_pass_fail(self):
        baseline = summarize_records([_record(1.0, "GOOD", [], 0.2, 0.7)])
        current = summarize_records([_record(1.0, "GOOD", [], 0.3, 0.8)])
        comparison = compare_summaries(current, baseline)
        self.assertEqual(
            comparison["comparison_status"], "DELTA_ONLY_NO_PASS_FAIL"
        )
        self.assertAlmostEqual(
            comparison["metric_deltas"]["global_translation_jump_m"]["delta"],
            0.1,
        )
        self.assertAlmostEqual(
            comparison["metric_deltas"]["scan_map_agreement"]["delta"], 0.1
        )

    @unittest.skipUnless(importlib.util.find_spec("yaml"), "PyYAML unavailable")
    def test_scenario_catalog_contains_high_spin_and_no_threshold_claims(self):
        root = Path(__file__).parents[1]
        scenario = load_scenario_definition(
            root / "config" / "regression_scenarios.yaml", "T05"
        )
        self.assertEqual(scenario["name"], "high-speed stationary spin")
        self.assertIn("scan_map_agreement", scenario["required_metrics"])
        self.assertNotIn("pass_threshold", scenario)

    def test_writer_creates_machine_readable_json(self):
        with tempfile.TemporaryDirectory() as directory:
            destination = Path(directory) / "nested" / "summary.json"
            write_json(destination, {"schema_version": 1, "value": 3})
            document = json.loads(destination.read_text(encoding="utf-8"))
        self.assertEqual(document["value"], 3)

    def test_empty_jsonl_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "empty.metrics.jsonl"
            source.write_text("\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "no records"):
                load_jsonl(source)


if __name__ == "__main__":
    unittest.main()
