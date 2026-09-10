from __future__ import annotations

import argparse
import sys

from robot_client import RobotClient, RobotClientError, config_from_env


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Minimal simulator connection test.")
    parser.add_argument("--robot-id", help="Current team id used by the simulator login.")
    parser.add_argument("--base-url", default=None, help="Simulator address, default http://127.0.0.1:2026")
    parser.add_argument("--log-dir", default="logs", help="Directory for generated logs.")
    parser.add_argument("--x", type=float, default=0.0, help="Test x coordinate.")
    parser.add_argument("--y", type=float, default=0.0, help="Test y coordinate.")
    parser.add_argument("--channel", type=int, default=1, help="Test channel.")
    parser.add_argument(
        "--try-clear",
        action="store_true",
        help="Also send one clear request at the same position and channel.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    config = config_from_env(robot_id=args.robot_id, base_url=args.base_url, log_dir=args.log_dir)

    with RobotClient(config) as client:
        try:
            enter_response = client.enter()
            print("enter:", enter_response)

            measure_response = client.measure(args.x, args.y, args.channel)
            print("measure:", measure_response)

            if args.try_clear:
                clear_response = client.clear(args.x, args.y, args.channel)
                print("clear:", clear_response)

            exit_response = client.exit()
            print("exit:", exit_response)
            print("logs:", client.run_dir)
            return 0
        except RobotClientError as exc:
            print(f"request failed: {exc}", file=sys.stderr)
            print("logs:", client.run_dir, file=sys.stderr)
            return 2


if __name__ == "__main__":
    raise SystemExit(main())
