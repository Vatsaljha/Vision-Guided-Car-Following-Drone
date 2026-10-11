#!/usr/bin/env python3
"""Summarize documented car-following simulation trials from a CSV."""

import argparse
import csv
import json
import sys
from pathlib import Path

REQUIRED = {
    "trial_id", "planned_laps", "completed_laps", "tracking_seconds",
    "eligible_tracking_seconds", "loss_events", "total_recovery_seconds",
}


def safe_percent(numerator, denominator):
    return round(100.0 * numerator / denominator, 2) if denominator > 0 else None


def number(row, key, row_num, allow_decimal=False):
    try:
        value = float(row[key]) if allow_decimal else int(row[key])
    except (KeyError, TypeError, ValueError):
        raise ValueError(f"row {row_num}: invalid numeric value for {key!r}")
    if value < 0:
        raise ValueError(f"row {row_num}: {key!r} cannot be negative")
    return value


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_file", type=Path)
    parser.add_argument("--json", dest="json_file", type=Path)
    args = parser.parse_args()
    if not args.csv_file.is_file():
        parser.error(f"CSV file not found: {args.csv_file}")

    trial_count = successful_trials = 0
    planned_laps = completed_laps = 0
    tracking_seconds = eligible_seconds = 0.0
    loss_events = 0
    recovery_seconds = 0.0

    try:
        with args.csv_file.open(newline="", encoding="utf-8-sig") as handle:
            reader = csv.DictReader(handle)
            if not reader.fieldnames or not REQUIRED.issubset(reader.fieldnames):
                parser.error("CSV is missing one or more required columns: " + ", ".join(sorted(REQUIRED)))
            for row_num, row in enumerate(reader, start=2):
                if not row or all(not (value or "").strip() for value in row.values()):
                    continue
                planned = number(row, "planned_laps", row_num)
                completed = number(row, "completed_laps", row_num)
                tracked = number(row, "tracking_seconds", row_num, True)
                eligible = number(row, "eligible_tracking_seconds", row_num, True)
                losses = number(row, "loss_events", row_num)
                recovery = number(row, "total_recovery_seconds", row_num, True)
                if planned <= 0:
                    raise ValueError(f"row {row_num}: planned_laps must be greater than zero")
                if completed > planned:
                    raise ValueError(f"row {row_num}: completed_laps cannot exceed planned_laps")
                if tracked > eligible:
                    raise ValueError(f"row {row_num}: tracking_seconds cannot exceed eligible_tracking_seconds")
                if recovery > 0 and losses == 0:
                    raise ValueError(f"row {row_num}: recovery time given but loss_events is zero")
                trial_count += 1
                successful_trials += int(completed >= planned)
                planned_laps += int(planned)
                completed_laps += int(completed)
                tracking_seconds += tracked
                eligible_seconds += eligible
                loss_events += int(losses)
                recovery_seconds += recovery
    except (OSError, csv.Error, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    if trial_count == 0:
        print("ERROR: No trial rows found; no metrics computed.", file=sys.stderr)
        return 2

    results = {
        "source_csv": str(args.csv_file),
        "trials": trial_count,
        "successful_trials": successful_trials,
        "planned_laps": planned_laps,
        "completed_laps": completed_laps,
        "lap_completion_rate_percent": safe_percent(completed_laps, planned_laps),
        "successful_trial_rate_percent": safe_percent(successful_trials, trial_count),
        "tracking_availability_percent": safe_percent(tracking_seconds, eligible_seconds),
        "tracking_seconds": round(tracking_seconds, 3),
        "eligible_tracking_seconds": round(eligible_seconds, 3),
        "marker_loss_events": loss_events,
        "mean_recovery_time_seconds": round(recovery_seconds / loss_events, 3) if loss_events else None,
        "total_recovery_seconds": round(recovery_seconds, 3),
        "warning": "Keep failed trials and document test conditions; do not combine unrelated metrics into one score.",
    }

    print(f"Trials: {trial_count} ({successful_trials} completed all planned laps)")
    print(f"Lap completion rate: {results['lap_completion_rate_percent']:.2f}%")
    print(f"Successful-trial rate: {results['successful_trial_rate_percent']:.2f}%")
    availability = results["tracking_availability_percent"]
    print(f"Tracking availability: {'N/A' if availability is None else f'{availability:.2f}%'}")
    recovery = results["mean_recovery_time_seconds"]
    print(f"Mean recovery time: {'N/A' if recovery is None else f'{recovery:.3f} s'}")
    print(results["warning"])

    if args.json_file:
        args.json_file.parent.mkdir(parents=True, exist_ok=True)
        args.json_file.write_text(json.dumps(results, indent=2) + "\n", encoding="utf-8")
        print(f"JSON written to: {args.json_file}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
