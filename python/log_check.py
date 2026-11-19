# -*- coding: utf-8 -*-
"""Checks that run after a bench log is closed.

They rewrite that CSV. They do not run while rows are still being written.
Add a function to CHECKS for the next check.
"""
from __future__ import annotations

import csv
from pathlib import Path

import ndi_ascii


def run_checks(path: Path) -> dict:
    report: dict = {}
    for check in CHECKS:
        report.update(check(path))
    return report


def _points(raw: str) -> list[tuple[float, float, float]]:
    text = (raw or "").strip()
    if not text:
        return []
    try:
        found = ndi_ascii.parse_tx_strays(text)
    except (RuntimeError, ValueError):
        found = []
    if found or " " not in text:
        return found
    # Older rows stored the reply's line break as a space.
    try:
        return ndi_ascii.parse_tx_strays(text.replace(" ", "\n", 1))
    except (RuntimeError, ValueError):
        return []


def _dist(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2


def _time(row: dict, index: int) -> float:
    try:
        return float(row.get("time_s") or index)
    except ValueError:
        return float(index)


def check_ndi_markers(path: Path) -> dict:
    """One marker per row.

    No marker: fill x, y, z from the nearest rows that have one.
    Several markers: keep the one nearest the single-marker track.
    The raw reply is left as captured.
    """
    with path.open(encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        fieldnames = list(reader.fieldnames or [])
        rows = list(reader)
    if not rows or "x" not in fieldnames or "ndi_raw" not in fieldnames:
        return {"ndi_not_one": 0}
    parsed = [_points(row.get("ndi_raw") or "") for row in rows]
    if not any(parsed):
        return {"ndi_not_one": 0, "ndi_present": False}
    chosen: list[tuple[float, float, float] | None] = [None] * len(rows)
    not_one = 0
    for index, points in enumerate(parsed):
        if len(points) == 1:
            chosen[index] = points[0]
        else:
            not_one += 1
    singles = [index for index, points in enumerate(parsed) if len(points) == 1]
    for index, points in enumerate(parsed):
        if len(points) < 2:
            continue
        anchor = _nearest_single(index, rows, singles, chosen)
        if anchor is None:
            chosen[index] = points[0]
        else:
            chosen[index] = min(points, key=lambda point: _dist(point, anchor))
    detected = [index for index, point in enumerate(chosen) if point is not None]
    for index, points in enumerate(parsed):
        if points:
            continue
        left = max((item for item in detected if item < index), default=None)
        right = min((item for item in detected if item > index), default=None)
        if left is None and right is None:
            continue
        if left is None:
            chosen[index] = chosen[right]
            continue
        if right is None:
            chosen[index] = chosen[left]
            continue
        t_row = _time(rows[index], index)
        t_left = _time(rows[left], left)
        t_right = _time(rows[right], right)
        span = t_right - t_left
        blend = 0.0 if span <= 0 else (t_row - t_left) / span
        start = chosen[left]
        end = chosen[right]
        chosen[index] = tuple(start[axis] + (end[axis] - start[axis]) * blend for axis in range(3))
    for index, point in enumerate(chosen):
        if point is None:
            continue
        rows[index]["x"] = f"{point[0]:.2f}"
        rows[index]["y"] = f"{point[1]:.2f}"
        rows[index]["z"] = f"{point[2]:.2f}"
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames, lineterminator="\n", extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)
    return {"ndi_not_one": not_one}


def _nearest_single(
    index: int,
    rows: list[dict],
    singles: list[int],
    chosen: list[tuple[float, float, float] | None],
) -> tuple[float, float, float] | None:
    best = None
    best_dt = None
    here = _time(rows[index], index)
    for other in singles:
        if chosen[other] is None:
            continue
        gap = abs(_time(rows[other], other) - here)
        if best_dt is None or gap < best_dt:
            best = chosen[other]
            best_dt = gap
    return best


CHECKS = [check_ndi_markers]
