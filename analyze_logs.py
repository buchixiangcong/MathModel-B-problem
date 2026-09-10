from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Summarize a robot run_log.jsonl file.")
    parser.add_argument("path", nargs="?", help="Path to run_log.jsonl or a run directory. Defaults to latest logs/run_*.")
    parser.add_argument("--total-sources", type=int, help="Source count shown by the simulator after a practice run.")
    parser.add_argument("--case-code", help="Test case code shown by the simulator, for the paper table row.")
    return parser.parse_args()


def latest_log() -> Path:
    runs = sorted(Path("logs").glob("run_*"), key=lambda p: p.stat().st_mtime, reverse=True)
    if not runs:
        raise FileNotFoundError("No logs/run_* directory found.")
    return runs[0] / "run_log.jsonl"


def resolve_log_path(path_arg: str | None) -> Path:
    if path_arg is None:
        return latest_log()
    path = Path(path_arg)
    if path.is_dir():
        path = path / "run_log.jsonl"
    return path


def load_records(path: Path) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                records.append(json.loads(line))
    return records


def main() -> int:
    args = parse_args()
    path = resolve_log_path(args.path)
    records = load_records(path)

    cleared_channels: list[int] = []
    detected_channels: set[int] = set()
    measure_count = 0
    clear_count = 0
    final_virtual_time = None
    first_real_timestamp_ms = None
    last_real_timestamp_ms = None

    for record in records:
        response = record.get("response_json") or {}
        real_timestamp_ms = response.get("real_timestamp_ms")
        if record.get("accepted") is True and real_timestamp_ms is not None:
            if first_real_timestamp_ms is None:
                first_real_timestamp_ms = real_timestamp_ms
            last_real_timestamp_ms = real_timestamp_ms
        if record.get("accepted") is True and response.get("virtual_time_s") is not None:
            final_virtual_time = response.get("virtual_time_s")

        channel = record.get("channel")
        if record.get("action") == "measure":
            measure_count += 1
            if response.get("measure_result") in {"direction", "near"} and channel is not None:
                detected_channels.add(int(channel))
        elif record.get("action") == "clear":
            clear_count += 1
            if response.get("clear_result") == "success" and channel is not None:
                cleared_channels.append(int(channel))

    cleared_unique = sorted(set(cleared_channels))
    print("log:", path)
    print("measure requests:", measure_count)
    print("clear requests:", clear_count)
    print("detected channels:", sorted(detected_channels))
    print("cleared channels:", cleared_unique)
    print("cleared count:", len(cleared_unique))
    if args.total_sources is not None:
        if not 10 <= args.total_sources <= 16:
            raise ValueError("--total-sources must be between 10 and 16 for Problem 3.")
        print("total sources:", args.total_sources)
        print("cleared proportion:", len(cleared_unique) / args.total_sources)
    print("final virtual time:", final_virtual_time)
    program_runtime_s = None
    if first_real_timestamp_ms is not None and last_real_timestamp_ms is not None:
        program_runtime_s = (last_real_timestamp_ms - first_real_timestamp_ms) / 1000.0
        print("program runtime:", program_runtime_s)
    if final_virtual_time is not None and cleared_unique:
        average_clear_time = final_virtual_time / len(cleared_unique)
        print("average clear time:", average_clear_time)
        if args.case_code:
            print(
                "paper table row:",
                f"{args.case_code}\t{len(cleared_unique)}\t{average_clear_time:.6f}\t{program_runtime_s}",
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
