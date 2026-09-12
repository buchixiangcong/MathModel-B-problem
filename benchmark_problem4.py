from __future__ import annotations

import argparse
import json
import statistics
import time
from pathlib import Path

from local_simulator_problem4 import LocalMixedSimulator
from problem4.problem4_strategy import Problem4Strategy, StrategyConfig
from run_problem4 import strategy_config


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Offline benchmark for Question 4.")
    parser.add_argument("--cases", type=int, default=20)
    parser.add_argument("--start-seed", type=int, default=0)
    parser.add_argument("--fast", action="store_true")
    parser.add_argument("--output", type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.cases <= 0:
        raise SystemExit("--cases must be positive")
    config: StrategyConfig = strategy_config(args.fast)
    records = []
    started = time.perf_counter()
    for seed in range(args.start_seed, args.start_seed + args.cases):
        simulator = LocalMixedSimulator.random_case(seed)
        strategy = Problem4Strategy(simulator, config)
        error = ""
        try:
            summary = strategy.run()
        except Exception as exc:
            summary = None
            error = f"{type(exc).__name__}: {exc}"
        records.append(
            {
                "seed": seed,
                "source_count": len(simulator.sources),
                "cleared_count": len(simulator.sources) - len(simulator.uncleared_channels()),
                "virtual_time_s": simulator.virtual_time_s,
                "measure_count": strategy.measure_count,
                "uncleared_channels": simulator.uncleared_channels(),
                "error": error,
            }
        )
    passed = [record for record in records if not record["uncleared_channels"] and not record["error"]]
    result = {
        "case_count": len(records),
        "passed_count": len(passed),
        "mean_virtual_time_s": statistics.mean(record["virtual_time_s"] for record in passed) if passed else None,
        "max_virtual_time_s": max((record["virtual_time_s"] for record in passed), default=None),
        "mean_measure_count": statistics.mean(record["measure_count"] for record in passed) if passed else None,
        "wall_time_s": time.perf_counter() - started,
        "records": records,
    }
    encoded = json.dumps(result, ensure_ascii=False, indent=2) + "\n"
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(encoded, encoding="utf-8")
    print(encoded, end="")
    return 0 if len(passed) == len(records) else 3


if __name__ == "__main__":
    raise SystemExit(main())
