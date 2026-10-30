#!/usr/bin/env python3
"""Extract paired PCA and Ceres local observability log records to CSV."""

from __future__ import annotations

import argparse
import csv
import math
import re
import statistics
from pathlib import Path


NUMBER = r"[-+0-9.eE]+|[-+]?(?:inf|nan)"
VALID_RE = re.compile(
    r"2D local observability: time=(?P<timestamp>\d+) "
    r"pca_points=(?P<pca_point_count>\d+) "
    r"pca_valid=(?P<pca_valid>true|false) "
    rf"pca_lambda_min=(?P<pca_lambda_min>{NUMBER}) "
    rf"pca_lambda_max=(?P<pca_lambda_max>{NUMBER}) "
    rf"pca_score=(?P<pca_score>{NUMBER}) "
    r"pca_degenerate=(?P<pca_degenerate>true|false) "
    rf"pca_direction=\((?P<pca_direction_x>{NUMBER}),(?P<pca_direction_y>{NUMBER})\) "
    rf"scale_length=(?P<scale_length>{NUMBER}) "
    r"measurement_residuals=(?P<measurement_residuals>\d+) "
    r"parameters=(?P<num_parameters>\d+) "
    rf"measurement_eigenvalues=\((?P<hessian_lambda_min>{NUMBER}),"
    rf"(?P<hessian_lambda_middle>{NUMBER}),(?P<hessian_lambda_max>{NUMBER})\) "
    rf"measurement_condition=(?P<hessian_condition_number>{NUMBER}) "
    rf"weak_direction=\((?P<weak_direction_x>{NUMBER}),"
    rf"(?P<weak_direction_y>{NUMBER}),(?P<weak_direction_yaw>{NUMBER})\) "
    r"objective_residuals=(?P<objective_residuals>\d+) "
    rf"objective_eigenvalues=\((?P<objective_lambda_min>{NUMBER}),"
    rf"(?P<objective_lambda_middle>{NUMBER}),(?P<objective_lambda_max>{NUMBER})\) "
    rf"objective_condition=(?P<objective_condition_number>{NUMBER}) "
    rf"scan_match_ms=(?P<scan_match_duration_ms>{NUMBER}) "
    rf"diagnostic_ms=(?P<diagnostic_duration_ms>{NUMBER}) valid=true"
)
INVALID_RE = re.compile(
    r"2D local observability unavailable: time=(?P<timestamp>\d+) "
    r"pca_valid=(?P<pca_valid>true|false) reason=(?P<reason>\S*)"
)

FIELDS = [
    "timestamp",
    "pca_point_count",
    "pca_valid",
    "pca_lambda_min",
    "pca_lambda_max",
    "pca_score",
    "pca_degenerate",
    "pca_direction_x",
    "pca_direction_y",
    "hessian_lambda_min",
    "hessian_lambda_middle",
    "hessian_lambda_max",
    "hessian_condition_number",
    "weak_direction_x",
    "weak_direction_y",
    "weak_direction_yaw",
    "objective_lambda_min",
    "objective_lambda_middle",
    "objective_lambda_max",
    "objective_condition_number",
    "measurement_residuals",
    "objective_residuals",
    "num_parameters",
    "scale_length",
    "scan_match_duration_ms",
    "diagnostic_duration_ms",
    "diagnostic_valid",
    "invalid_reason",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("inputs", nargs="+", type=Path)
    parser.add_argument("-o", "--output", required=True, type=Path)
    return parser.parse_args()


def extract(paths: list[Path]) -> tuple[list[dict[str, str]], dict[str, int]]:
    rows: list[dict[str, str]] = []
    counts = {"valid": 0, "invalid": 0, "malformed": 0}
    for path in paths:
        for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
            if "2D local observability unavailable:" in line:
                match = INVALID_RE.search(line)
                if match is None:
                    counts["malformed"] += 1
                    continue
                row = {field: "" for field in FIELDS}
                row.update(match.groupdict())
                row["diagnostic_valid"] = "false"
                row["invalid_reason"] = (
                    row["reason"] or "no_active_submap_or_match_result"
                )
                del row["reason"]
                rows.append(row)
                counts["invalid"] += 1
                continue
            if "2D local observability:" not in line:
                continue
            match = VALID_RE.search(line)
            if match is None:
                counts["malformed"] += 1
                continue
            values = match.groupdict()
            numeric = [float(value) for key, value in values.items()
                       if key not in {"timestamp", "pca_point_count", "measurement_residuals",
                                      "num_parameters", "objective_residuals", "pca_valid",
                                      "pca_degenerate", "hessian_condition_number",
                                      "objective_condition_number"}]
            if int(values["pca_point_count"]) < 2 or not all(
                math.isfinite(value) for value in numeric
            ):
                counts["malformed"] += 1
                continue
            row = {field: "" for field in FIELDS}
            row.update(values)
            row.update({
                "diagnostic_valid": "true",
                "invalid_reason": "",
            })
            rows.append(row)
            counts["valid"] += 1
    return rows, counts


def pearson(xs: list[float], ys: list[float]) -> float:
    if len(xs) < 2:
        return float("nan")
    mean_x, mean_y = statistics.mean(xs), statistics.mean(ys)
    covariance = sum((x - mean_x) * (y - mean_y) for x, y in zip(xs, ys))
    scale = math.sqrt(
        sum((x - mean_x) ** 2 for x in xs)
        * sum((y - mean_y) ** 2 for y in ys)
    )
    return covariance / scale if scale > 0.0 else float("nan")


def write_csv(path: Path, rows: list[dict[str, str]]) -> None:
    if path.exists():
        raise FileExistsError(f"Refusing to overwrite existing output: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDS, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def report(rows: list[dict[str, str]], counts: dict[str, int]) -> None:
    print(f"valid_samples={counts['valid']}")
    print(f"invalid_samples={counts['invalid']}")
    print(f"malformed_records={counts['malformed']}")
    valid_rows = [row for row in rows if row["diagnostic_valid"] == "true"]
    if not valid_rows:
        return
    pca_rows = [row for row in valid_rows if row["pca_valid"] == "true"]
    pca = [float(row["pca_score"]) for row in pca_rows]
    finite_condition = [
        (float(row["pca_score"]), float(row["hessian_condition_number"]))
        for row in pca_rows
        if row["pca_valid"] == "true"
        if math.isfinite(float(row["hessian_condition_number"]))
        and float(row["hessian_condition_number"]) > 0.0
    ]
    print(f"pca_score_min={min(pca):.9g}")
    print(f"pca_score_median={statistics.median(pca):.9g}")
    print(f"pca_score_max={max(pca):.9g}")
    print(f"finite_hessian_condition_samples={len(finite_condition)}")
    if finite_condition:
        correlation = pearson(
            [x for x, _ in finite_condition],
            [math.log10(y) for _, y in finite_condition],
        )
        print(
            "pearson_pca_score_vs_log10_measurement_condition="
            f"{correlation:.9g}"
        )
    for field in ("scan_match_duration_ms", "diagnostic_duration_ms"):
        values = sorted(float(row[field]) for row in valid_rows)
        p95 = values[min(len(values) - 1, math.ceil(0.95 * len(values)) - 1)]
        print(f"{field}_median={statistics.median(values):.9g}")
        print(f"{field}_p95={p95:.9g}")
        print(f"{field}_max={max(values):.9g}")


def main() -> int:
    args = parse_args()
    missing = [path for path in args.inputs if not path.is_file()]
    if missing:
        raise FileNotFoundError(missing[0])
    rows, counts = extract(args.inputs)
    write_csv(args.output, rows)
    report(rows, counts)
    print(f"csv={args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
