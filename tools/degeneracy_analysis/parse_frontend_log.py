#!/usr/bin/env python3
"""Extract Cartographer frontend degeneracy observations into CSV."""

from __future__ import annotations

import argparse
import csv
import math
import re
import statistics
from pathlib import Path


OBSERVATION_RE = re.compile(
    r"2D scan-match degeneracy observation: "
    r"time=(?P<timestamp>\d+) "
    r"points=(?P<point_count>\d+) "
    r"lambda_max=(?P<lambda_max>[-+0-9.eE]+) "
    r"lambda_min=(?P<lambda_min>[-+0-9.eE]+) "
    r"score=(?P<score>[-+0-9.eE]+) "
    r"degenerate=(?P<degenerate>true|false) "
    r"direction=\((?P<direction_x>[-+0-9.eE]+),"
    r"(?P<direction_y>[-+0-9.eE]+)\)"
)
UNAVAILABLE_RE = re.compile(
    r"2D scan-match degeneracy observation unavailable: "
    r"time=(?P<timestamp>\d+) points=(?P<point_count>\d+)"
)

FIELDNAMES = [
    "timestamp",
    "point_count",
    "lambda_max",
    "lambda_min",
    "degeneracy_score",
    "is_degenerate",
    "direction_x",
    "direction_y",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Extract frontend degeneracy log lines into CSV."
    )
    parser.add_argument("inputs", nargs="+", type=Path, help="input log files")
    parser.add_argument("-o", "--output", required=True, type=Path)
    return parser.parse_args()


def _finite(values: list[float]) -> bool:
    return all(math.isfinite(value) for value in values)


def extract(inputs: list[Path]) -> tuple[list[dict[str, object]], int, int]:
    rows: list[dict[str, object]] = []
    unavailable = 0
    malformed = 0
    for path in inputs:
        if not path.is_file():
            raise FileNotFoundError(path)
        for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
            if "2D scan-match degeneracy observation unavailable:" in line:
                if UNAVAILABLE_RE.search(line):
                    unavailable += 1
                else:
                    malformed += 1
                continue
            match = OBSERVATION_RE.search(line)
            if match is None:
                if "2D scan-match degeneracy observation:" in line:
                    malformed += 1
                continue
            values = match.groupdict()
            numeric = [
                float(values["lambda_max"]),
                float(values["lambda_min"]),
                float(values["score"]),
                float(values["direction_x"]),
                float(values["direction_y"]),
            ]
            if int(values["point_count"]) < 2 or not _finite(numeric):
                malformed += 1
                continue
            rows.append(
                {
                    "timestamp": values["timestamp"],
                    "point_count": values["point_count"],
                    "lambda_max": values["lambda_max"],
                    "lambda_min": values["lambda_min"],
                    "degeneracy_score": values["score"],
                    "is_degenerate": values["degenerate"],
                    "direction_x": values["direction_x"],
                    "direction_y": values["direction_y"],
                }
            )
    return rows, unavailable, malformed


def write_csv(output: Path, rows: list[dict[str, object]]) -> None:
    if output.exists():
        raise FileExistsError(f"Refusing to overwrite existing output: {output}")
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDNAMES, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def print_summary(rows: list[dict[str, object]], unavailable: int, malformed: int) -> None:
    scores = [float(row["degeneracy_score"]) for row in rows]
    degenerate = sum(row["is_degenerate"] == "true" for row in rows)
    print(f"valid_logged_observations={len(rows)}")
    print(f"unavailable_logged_observations={unavailable}")
    print(f"malformed_observation_lines={malformed}")
    print(f"degenerate_logged_observations={degenerate}")
    print(
        "degenerate_ratio="
        f"{(degenerate / len(rows)) if rows else float('nan'):.6g}"
    )
    if scores:
        print(f"score_min={min(scores):.9g}")
        print(f"score_median={statistics.median(scores):.9g}")
        print(f"score_max={max(scores):.9g}")


def main() -> int:
    args = parse_args()
    rows, unavailable, malformed = extract(args.inputs)
    write_csv(args.output, rows)
    print_summary(rows, unavailable, malformed)
    print(f"csv={args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
