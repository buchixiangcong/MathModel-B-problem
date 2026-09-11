from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime
from pathlib import Path

from problem3.local_simulator import LocalOmnidirectionalSimulator
from problem3.problem3_strategy import (
    Problem3Strategy,
    StrategyConfig,
    summary_to_dict,
)
from robot_client import RobotClient, RobotClientError, config_from_env


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run the CUMCM 2026 B Question 3 strategy."
    )
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--local", action="store_true", help="run an offline random case")
    mode.add_argument("--official", action="store_true", help="connect to the official simulator")
    parser.add_argument("--seed", type=int, default=20260910)
    parser.add_argument("--source-count", type=int, choices=range(10, 17))
    parser.add_argument("--robot-id", help="team id used by the official simulator")
    parser.add_argument("--base-url", default=None)
    parser.add_argument("--log-dir", default="logs")
    parser.add_argument("--fast", action="store_true", help="use a smaller candidate grid")
    parser.add_argument(
        "--confirm-enter",
        choices=("REHEARSAL", "FORMAL"),
        help="required in official mode to prevent accidental /enter calls",
    )
    return parser.parse_args()


def strategy_config(fast: bool) -> StrategyConfig:
    if not fast:
        return StrategyConfig()
    return StrategyConfig(
        circle_side_count=180,
        candidate_angle_count=16,
        candidate_radial_levels=4,
        source_edge_subdivisions=4,
        source_interior_levels=2,
        error_sample_count=3,
        route_candidate_tracks=3,
    )


def main() -> int:
    args = parse_args()
    log_root = Path(args.log_dir)
    run_stamp = datetime.now().strftime("problem3_%Y%m%d_%H%M%S")
    decision_log = log_root / run_stamp / "decision_log.jsonl"

    if args.local:
        robot = LocalOmnidirectionalSimulator.random_case(
            args.seed, args.source_count
        )
        truth_channels = sorted(robot.sources)
        strategy = Problem3Strategy(
            robot, strategy_config(args.fast), decision_log
        )
        try:
            summary = strategy.run()
        except Exception as exc:
            print(f"local run failed: {exc}", file=sys.stderr)
            print("truth channels:", truth_channels, file=sys.stderr)
            print("uncleared:", robot.uncleared_channels(), file=sys.stderr)
            return 2
        output = summary_to_dict(summary)
        output["truth_channels"] = truth_channels
        output["uncleared_channels"] = robot.uncleared_channels()
        decision_log.parent.mkdir(parents=True, exist_ok=True)
        (decision_log.parent / "summary.json").write_text(
            json.dumps(output, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        print(json.dumps(output, ensure_ascii=False, indent=2))
        return 0 if not robot.uncleared_channels() else 3

    if not args.robot_id:
        print("--robot-id is required in --official mode", file=sys.stderr)
        return 2
    if args.confirm_enter is None:
        print(
            "official mode requires --confirm-enter REHEARSAL or FORMAL; "
            "this guard prevents accidental use of a test opportunity",
            file=sys.stderr,
        )
        return 2

    client_config = config_from_env(
        robot_id=args.robot_id,
        base_url=args.base_url,
        log_dir=args.log_dir,
    )
    try:
        with RobotClient(client_config) as robot:
            # Keep strategy decisions beside the client's raw request logs.
            decision_log = robot.run_dir / "decision_log.jsonl"
            strategy = Problem3Strategy(
                robot, strategy_config(args.fast), decision_log
            )
            summary = strategy.run()
            output = summary_to_dict(summary)
            output["declared_test_mode"] = args.confirm_enter
            (robot.run_dir / "summary.json").write_text(
                json.dumps(output, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
            print(json.dumps(output, ensure_ascii=False, indent=2))
            print("raw logs:", robot.run_dir)
        return 0
    except (RobotClientError, RuntimeError, ValueError) as exc:
        print(f"official rehearsal failed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
