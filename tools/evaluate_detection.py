#!/usr/bin/env python3
"""Evaluate binary ID-7 detections against manually verified frame labels.

CSV columns required: frame_id, ground_truth_id7, predicted_id7
Both label columns accept 1/0, true/false, yes/no, or present/absent.
"""

import argparse
import csv
import json
import sys
from pathlib import Path

TRUE_VALUES = {"1", "true", "yes", "y", "present", "detected"}
FALSE_VALUES = {"0", "false", "no", "n", "absent", "not_detected"}


def parse_binary(value: str, column: str, row_num: int) -> int:
    normalized = value.strip().lower()
    if normalized in TRUE_VALUES:
        return 1
    if normalized in FALSE_VALUES:
        return 0
    raise ValueError(
        f"row {row_num}: {column} must be a binary label; got {value!r}"
    )


def ratio(numerator: int, denominator: int):
    return numerator / denominator if denominator else None


def percentage(value):
    return None if value is None else round(value * 100.0, 2)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_file", type=Path, help="CSV with verified labels")
    parser.add_argument(
        "--json", dest="json_file", type=Path,
        help="optional path to save the results as JSON",
    )
    args = parser.parse_args()

    if not args.csv_file.is_file():
        parser.error(f"CSV file not found: {args.csv_file}")

    tp = fp = tn = fn = 0
    rows = 0
    try:
        with args.csv_file.open(newline="", encoding="utf-8-sig") as handle:
            reader = csv.DictReader(handle)
            required = {"ground_truth_id7", "predicted_id7"}
            if not reader.fieldnames or not required.issubset(reader.fieldnames):
                parser.error(
                    "CSV must have columns: ground_truth_id7,predicted_id7 "
                    "(frame_id and notes are recommended)."
                )
            for row_num, row in enumerate(reader, start=2):
                if not row or all(not (value or "").strip() for value in row.values()):
                    continue
                gt = parse_binary(row.get("ground_truth_id7", ""), "ground_truth_id7", row_num)
                pred = parse_binary(row.get("predicted_id7", ""), "predicted_id7", row_num)
                rows += 1
                if gt and pred:
                    tp += 1
                elif not gt and pred:
                    fp += 1
                elif not gt and not pred:
                    tn += 1
                else:
                    fn += 1
    except (OSError, csv.Error, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    if rows == 0:
        print("ERROR: No labeled rows found; no metrics computed.", file=sys.stderr)
        return 2

    precision = ratio(tp, tp + fp)
    recall = ratio(tp, tp + fn)
    specificity = ratio(tn, tn + fp)
    accuracy = ratio(tp + tn, rows)
    f1 = ratio(2 * tp, 2 * tp + fp + fn)

    results = {
        "source_csv": str(args.csv_file),
        "frames_evaluated": rows,
        "confusion_matrix": {"TP": tp, "FP": fp, "TN": tn, "FN": fn},
        "metrics_percent": {
            "accuracy": percentage(accuracy),
            "precision": percentage(precision),
            "recall": percentage(recall),
            "specificity": percentage(specificity),
            "f1_score": percentage(f1),
        },
        "warning": "Results are only as reliable as the frame labels and test sampling protocol.",
    }

    print(f"Frames evaluated: {rows}")
    print(f"TP={tp}  FP={fp}  TN={tn}  FN={fn}")
    print("Metrics (%):")
    for name, value in results["metrics_percent"].items():
        print(f"  {name:12s}: {'N/A' if value is None else f'{value:.2f}%'}")
    print(results["warning"])

    if args.json_file:
        args.json_file.parent.mkdir(parents=True, exist_ok=True)
        args.json_file.write_text(json.dumps(results, indent=2) + "\n", encoding="utf-8")
        print(f"JSON written to: {args.json_file}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
