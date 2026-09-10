from __future__ import annotations

import unittest

from geometry import Point, distance, problem3_fast_scan_points, problem3_scan_points
from strategy import ChannelState, Problem3Strategy, clear_route_length, optimize_clear_order


def route_length(points):
    return sum(distance(first, second) for first, second in zip(points, points[1:]))


class FakeMeasureClient:
    def __init__(self):
        self.calls = []

    def measure(self, x, y, channel):
        self.calls.append((x, y, channel))
        if channel == 1:
            return {"measure_result": "direction", "svd_deg": 45.0}
        return {"measure_result": "no_signal"}


class Problem3FastStrategyTests(unittest.TestCase):
    def test_alternating_route_keeps_coverage_and_shortens_path(self):
        baseline = problem3_scan_points()
        optimized = problem3_fast_scan_points()

        self.assertEqual(set(baseline), set(optimized))
        self.assertEqual(baseline[0], optimized[0])
        self.assertEqual(baseline[-1], optimized[-1])
        self.assertLess(route_length(optimized), route_length(baseline) - 4000.0)

    def test_bearing_limit_skips_only_locked_channels(self):
        client = FakeMeasureClient()
        strategy = Problem3Strategy(channels=[1, 2], bearing_limit=4)

        strategy.scan_all_points(client)

        channel_one_calls = [call for call in client.calls if call[2] == 1]
        channel_two_calls = [call for call in client.calls if call[2] == 2]
        self.assertEqual(len(channel_one_calls), 4)
        self.assertEqual(len(channel_two_calls), len(problem3_scan_points()))
        self.assertEqual(len(strategy.state[1].bearings), 4)

    def test_clear_route_optimizer_removes_large_detour(self):
        start = Point(0.0, 0.0)
        states = [
            ChannelState(1, last_estimate=Point(1.0, 0.0)),
            ChannelState(2, last_estimate=Point(0.0, 10.0)),
            ChannelState(3, last_estimate=Point(2.0, 0.0)),
        ]
        bad_length = clear_route_length(start, states)

        optimized = optimize_clear_order(start, states)

        self.assertEqual({st.channel for st in optimized}, {1, 2, 3})
        self.assertLess(clear_route_length(start, optimized), bad_length - 5.0)


if __name__ == "__main__":
    unittest.main()
