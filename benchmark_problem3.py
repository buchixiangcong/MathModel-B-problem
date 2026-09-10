from __future__ import annotations

import argparse
import json
import statistics
import time
from pathlib import Path

from problem3.local_simulator import LocalOmnidirectionalSimulator
from problem3.problem3_strategy import Problem3Strategy
from run_problem3 import strategy_config


def main() -> int:
    parser = argparse.ArgumentParser(description="Offline batch test for Question 3.")
    parser.add_argument("--cases", type=int, default=20)
    parser.add_argument("--start-seed", type=int, default=0)
    parser.add_argument("--fast", action="store_true")
    parser.add_argument("--output", type=Path, help="optional JSON report path")
    args = parser.parse_args()

    records = []
    failures = []
    started = time.perf_counter()
    for seed in range(args.start_seed, args.start_seed + args.cases):
        simulator = LocalOmnidirectionalSimulator.random_case(seed)
        try:
            summary = Problem3Strategy(
                simulator, strategy_config(args.fast)
            ).run()
            uncleared = simulator.uncleared_channels()
            record = {
                "seed": seed,
                "source_count": len(simulator.sources),
                "cleared_count": len(summary.cleared_channels),
                "clear_ratio": len(summary.cleared_channels) / len(simulator.sources),
                "virtual_time_s": summary.virtual_time_s,
                "average_time_s": summary.average_time_per_cleared_source_s,
                "measure_count": summary.measure_count,
                "uncleared_channels": uncleared,
            }
            records.append(record)
            if uncleared:
                failures.append(record)
        except Exception as exc:
            failures.append({"seed": seed, "error": f"{type(exc).__name__}: {exc}"})
        print(
            f"case {seed}: "
            + ("PASS" if not failures or failures[-1].get("seed") != seed else "FAIL"),
            flush=True,
        )

    elapsed = time.perf_counter() - started
    output = {
        "case_count": args.cases,
        "passed_count": args.cases - len(failures),
        "failed_count": len(failures),
        "overall_clear_ratio": (
            sum(record["cleared_count"] for record in records)
            / sum(record["source_count"] for record in records)
            if records
            else 0.0
        ),
        "mean_virtual_time_s": statistics.mean(
            record["virtual_time_s"] for record in records
        )
        if records
        else None,
        "max_virtual_time_s": max(
            (record["virtual_time_s"] for record in records), default=None
        ),
        "mean_average_time_per_source_s": statistics.mean(
            record["average_time_s"] for record in records
        )
        if records
        else None,
        "wall_time_s": elapsed,
        "failures": failures,
        "records": records,
    }
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(output, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
    print(json.dumps(output, ensure_ascii=False, indent=2))
    return 0 if not failures else 2


if __name__ == "__main__":
    raise SystemExit(main())
