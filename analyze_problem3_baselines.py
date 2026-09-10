from __future__ import annotations

import argparse
import json
from collections import defaultdict
from pathlib import Path
from statistics import mean, median

from geometry import Bearing, Point, distance, estimate_source_position, problem3_scan_points


def percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    if not ordered:
        return float("nan")
    index = round((len(ordered) - 1) * fraction)
    return ordered[index]


def analyze_run(run_dir: Path) -> dict[str, object]:
    records = [json.loads(line) for line in (run_dir / "run_log.jsonl").read_text(encoding="utf-8").splitlines()]
    directions: dict[int, list[Bearing]] = defaultdict(list)
    clear_points: dict[int, Point] = {}
    first_scan_time = None
    scan_end_time = None
    measure_count = 0

    for record in records:
        if record.get("action") == "measure":
            measure_count += 1
            if first_scan_time is None:
                first_scan_time = record.get("virtual_time_s")
            scan_end_time = record.get("virtual_time_s")
            if record.get("measure_result") == "direction":
                channel = int(record["channel"])
                directions[channel].append(
                    Bearing(
                        channel=channel,
                        point=Point(float(record["x"]), float(record["y"])),
                        angle_deg=float(record["svd_deg"]),
                    )
                )
        elif record.get("action") == "clear" and record.get("clear_result") == "success":
            clear_points[int(record["channel"])] = Point(float(record["x"]), float(record["y"]))

    errors_by_count: dict[int, list[float]] = defaultdict(list)
    stability_by_count: dict[int, list[float]] = defaultdict(list)
    per_channel: list[dict[str, object]] = []
    for channel, target in sorted(clear_points.items()):
        bearings = directions[channel]
        estimates: list[Point | None] = [None]
        estimates.extend(estimate_source_position(bearings[:count]) for count in range(2, len(bearings) + 1))
        channel_errors: dict[int, float] = {}
        for count in range(2, len(bearings) + 1):
            estimate = estimates[count - 1]
            if estimate is None:
                continue
            error = distance(estimate, target)
            errors_by_count[count].append(error)
            channel_errors[count] = error
            if count >= 3:
                previous = estimates[count - 2]
                if previous is not None:
                    stability_by_count[count].append(distance(previous, estimate))
        per_channel.append(
            {
                "channel": channel,
                "bearings": len(bearings),
                "early_error_m": channel_errors,
            }
        )

    final_time = max((record.get("virtual_time_s") or 0.0) for record in records)
    return {
        "run": run_dir.name,
        "measure_count": measure_count,
        "signal_count": len(clear_points),
        "scan_end_time": scan_end_time,
        "final_time": final_time,
        "directions": sum(len(items) for items in directions.values()),
        "errors_by_count": errors_by_count,
        "stability_by_count": stability_by_count,
        "per_channel": per_channel,
        "records": records,
        "clear_points": clear_points,
    }


def point_key(point: Point) -> tuple[float, float]:
    return round(point.x, 3), round(point.y, 3)


def alternating_scan_points() -> list[Point]:
    baseline = problem3_scan_points()
    inner = baseline[1:9]
    outer = baseline[9:17]
    return [baseline[0], *(point for pair in zip(inner, outer) for point in pair)]


def path_length(points: list[Point]) -> float:
    return sum(distance(first, second) for first, second in zip(points, points[1:]))


def replay_early_stop(result: dict[str, object], minimum_bearings: int) -> dict[str, object]:
    measurements = {
        (round(float(record["x"]), 3), round(float(record["y"]), 3), int(record["channel"])): record
        for record in result["records"]
        if record.get("action") == "measure"
    }
    collected: dict[int, list[Bearing]] = defaultdict(list)
    stopped: set[int] = set()
    measure_count = 0

    for point in alternating_scan_points():
        for channel in range(1, 21):
            if channel in stopped:
                continue
            measure_count += 1
            record = measurements[(*point_key(point), channel)]
            if record.get("measure_result") != "direction":
                continue
            collected[channel].append(
                Bearing(channel, point, float(record["svd_deg"]))
            )
            if len(collected[channel]) >= minimum_bearings:
                stopped.add(channel)

    errors = []
    clear_points = result["clear_points"]
    for channel, target in clear_points.items():
        estimate = estimate_source_position(collected[channel])
        if estimate is not None:
            errors.append(distance(estimate, target))

    return {
        "measure_count": measure_count,
        "detected": len([items for items in collected.values() if len(items) >= 2]),
        "stopped": len(stopped),
        "errors": errors,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Replay Problem 3 logs and assess early triangulation accuracy.")
    parser.add_argument("runs", nargs="+", type=Path, help="Run directories containing run_log.jsonl")
    args = parser.parse_args()

    analyses = [analyze_run(path) for path in args.runs]
    all_errors: dict[int, list[float]] = defaultdict(list)
    all_stability: dict[int, list[float]] = defaultdict(list)
    for result in analyses:
        print(
            f"{result['run']}: signals={result['signal_count']}, measures={result['measure_count']}, "
            f"directions={result['directions']}, scan_end={result['scan_end_time']:.3f}, "
            f"final={result['final_time']:.3f}"
        )
        for count, values in result["errors_by_count"].items():
            all_errors[count].extend(values)
        for count, values in result["stability_by_count"].items():
            all_stability[count].extend(values)

    print("\nEarly estimate distance from the successful baseline clear point:")
    for count in sorted(all_errors):
        values = all_errors[count]
        stability = all_stability.get(count, [])
        stable_text = ""
        if stability:
            stable_text = f", update median/p90/max={median(stability):.1f}/{percentile(stability, 0.9):.1f}/{max(stability):.1f} m"
        print(
            f"  {count} bearings: n={len(values)}, error mean/median/p90/max="
            f"{mean(values):.1f}/{median(values):.1f}/{percentile(values, 0.9):.1f}/{max(values):.1f} m"
            f"{stable_text}"
        )

    baseline_points = problem3_scan_points()
    fast_points = alternating_scan_points()
    saved_distance = path_length(baseline_points) - path_length(fast_points)
    print(
        f"\nAlternating route: {path_length(baseline_points):.1f} -> {path_length(fast_points):.1f} m, "
        f"saving {saved_distance:.1f} m ({saved_distance / 5.0:.1f} virtual seconds at 5 m/s)."
    )
    for minimum_bearings in (2, 3, 4):
        replays = [replay_early_stop(result, minimum_bearings) for result in analyses]
        counts = [item["measure_count"] for item in replays]
        errors = [error for item in replays for error in item["errors"]]
        predicted = [
            float(result["final_time"]) - saved_distance / 5.0 - (340 - item["measure_count"]) * 6.0
            for result, item in zip(analyses, replays)
        ]
        print(
            f"  stop at {minimum_bearings}: measures={counts}, located={[item['detected'] for item in replays]}, "
            f"estimate error median/p90/max={median(errors):.1f}/{percentile(errors, 0.9):.1f}/{max(errors):.1f} m, "
            f"predicted final mean/max={mean(predicted):.1f}/{max(predicted):.1f}"
        )

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
