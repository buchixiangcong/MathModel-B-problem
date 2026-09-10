from __future__ import annotations

import argparse
import sys

from robot_client import RobotClient, RobotClientError, config_from_env
from strategy import Problem3FastStrategy, parse_channels


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the optimized Problem 3 search-and-clear strategy.")
    parser.add_argument("--robot-id", help="Current team id used by the simulator login.")
    parser.add_argument("--base-url", default=None, help="Simulator address, default http://127.0.0.1:2026")
    parser.add_argument("--log-dir", default="logs", help="Directory for generated logs.")
    parser.add_argument("--channels", default="1-20", help="Channels to scan, e.g. 1-20 or 1,3,8.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    config = config_from_env(robot_id=args.robot_id, base_url=args.base_url, log_dir=args.log_dir)
    strategy = Problem3FastStrategy(channels=parse_channels(args.channels))

    with RobotClient(config) as client:
        try:
            result = strategy.run(client)
        except RobotClientError as exc:
            print(f"strategy failed: {exc}", file=sys.stderr)
            print(f"logs: {client.run_dir}", file=sys.stderr)
            return 2

    print("\n=== Problem 3 optimized summary ===")
    print("known signal channels:", result.known_signal_channels)
    print("cleared channels:", result.cleared_channels)
    print("cleared count:", len(result.cleared_channels))
    print("final virtual time:", result.final_virtual_time_s)
    if result.final_virtual_time_s is not None and result.cleared_channels:
        print("average clear time:", result.final_virtual_time_s / len(result.cleared_channels))
    print("logs:", result.run_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
