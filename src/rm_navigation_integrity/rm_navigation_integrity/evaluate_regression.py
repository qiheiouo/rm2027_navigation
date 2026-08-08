"""CLI for raw navigation integrity metric summaries and baseline deltas."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from .regression import (
    compare_summaries,
    load_jsonl,
    load_scenario_definition,
    summarize_records,
    write_json,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Summarize integrity JSONL without inventing pass/fail thresholds."
    )
    parser.add_argument("--input", required=True, help="integrity *.metrics.jsonl")
    parser.add_argument("--output", required=True, help="new summary JSON path")
    parser.add_argument("--baseline", help="optional prior summary JSON")
    parser.add_argument("--scenario", help="optional scenario ID such as T05")
    parser.add_argument(
        "--definitions", help="scenario YAML, required when --scenario is used"
    )
    return parser


def main(argv=None) -> int:
    arguments = build_parser().parse_args(argv)
    source = Path(arguments.input).resolve()
    destination = Path(arguments.output).resolve()
    if source == destination:
        raise ValueError("output must not overwrite the input JSONL")
    summary = summarize_records(load_jsonl(source))
    summary["source"] = str(source)
    if arguments.scenario:
        if not arguments.definitions:
            raise ValueError("--definitions is required with --scenario")
        summary["scenario"] = load_scenario_definition(
            arguments.definitions, arguments.scenario
        )
    if arguments.baseline:
        with Path(arguments.baseline).open("r", encoding="utf-8") as stream:
            baseline = json.load(stream)
        summary["baseline_comparison"] = compare_summaries(summary, baseline)
        summary["baseline_source"] = str(Path(arguments.baseline).resolve())
    write_json(destination, summary)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
