from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime
from pathlib import Path

from local_simulator_problem4 import LocalMixedSimulator
from problem4.problem4_strategy import Problem4Strategy, StrategyConfig, summary_to_dict
from robot_client import RobotClient, RobotClientError, config_from_env


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the CUMCM 2026 B Question 4 strategy.")
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--local", action="store_true")
    mode.add_argument("--official", action="store_true")
    parser.add_argument("--seed", type=int, default=20260911)
    parser.add_argument("--source-count", type=int, choices=range(10, 17))
    parser.add_argument("--robot-id")
    parser.add_argument("--base-url", default=None)
    parser.add_argument("--log-dir", default="logs")
    parser.add_argument("--fast", action="store_true")
    parser.add_argument("--confirm-enter", choices=("REHEARSAL", "FORMAL"))
    return parser.parse_args()


def strategy_config(fast: bool) -> StrategyConfig:
    if fast:
        return StrategyConfig(lattice_margin=0.0, strict_discovery_certificate=False,
                              circle_side_count=120, candidate_angle_count=12, candidate_radial_levels=2,
                              source_edge_subdivisions=3, source_interior_levels=1, error_sample_count=2,
                              max_grid_localization_rounds=6, maximum_localization_measurements=80,
                              bootstrap_observation_target=4)
    return StrategyConfig()


def main() -> int:
    args = parse_args()
    stamp = datetime.now().strftime("problem4_%Y%m%d_%H%M%S")
    decision_log = Path(args.log_dir) / stamp / "decision_log.jsonl"
    if args.local:
        robot = LocalMixedSimulator.random_case(args.seed, args.source_count)
        truth = sorted(robot.sources)
        strategy = Problem4Strategy(robot, strategy_config(args.fast), decision_log)
        try:
            summary = strategy.run()
        except Exception as exc:
            print(f"local run failed: {exc}", file=sys.stderr)
            print("truth channels:", truth, file=sys.stderr)
            print("uncleared:", robot.uncleared_channels(), file=sys.stderr)
            return 2
        output = summary_to_dict(summary)
        output.update(truth_channels=truth, uncleared_channels=robot.uncleared_channels())
        decision_log.parent.mkdir(parents=True, exist_ok=True)
        (decision_log.parent / "summary.json").write_text(json.dumps(output, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(output, ensure_ascii=False, indent=2))
        return 0 if not robot.uncleared_channels() else 3
    if not args.robot_id or args.confirm_enter is None:
        print("official mode requires --robot-id and --confirm-enter REHEARSAL", file=sys.stderr)
        return 2
    if args.confirm_enter == "FORMAL":
        print("FORMAL is intentionally disabled in this implementation; use REHEARSAL first.", file=sys.stderr)
        return 2
    try:
        with RobotClient(config_from_env(robot_id=args.robot_id, base_url=args.base_url, log_dir=args.log_dir)) as robot:
            strategy = Problem4Strategy(robot, strategy_config(args.fast), robot.run_dir / "decision_log.jsonl")
            summary = strategy.run()
            output = summary_to_dict(summary)
            output["declared_test_mode"] = args.confirm_enter
            (robot.run_dir / "summary.json").write_text(json.dumps(output, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
            print(json.dumps(output, ensure_ascii=False, indent=2))
            print("raw logs:", robot.run_dir)
        return 0
    except (RobotClientError, RuntimeError, ValueError) as exc:
        print(f"official rehearsal failed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
