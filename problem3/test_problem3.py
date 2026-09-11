from __future__ import annotations

import math
import sys
import unittest
from itertools import permutations
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PROBLEM1 = ROOT / "problem1" / "problem1"
if str(PROBLEM1) not in sys.path:
    sys.path.insert(0, str(PROBLEM1))

from problem1_geometry import Point, minimum_enclosing_circle

from problem3.local_simulator import LocalOmnidirectionalSimulator, OmnidirectionalSource
from problem3.problem3_strategy import (
    Problem3Strategy,
    SourceTrack,
    StrategyConfig,
    coverage_points,
    coverage_worst_distance,
)


class Problem3Tests(unittest.TestCase):
    def test_small_far_from_origin_triangle_has_stable_circle(self) -> None:
        points = [
            Point(1738.8344650840322, 466.17991197649366),
            Point(1738.8768195146372, 466.13536494245363),
            Point(1738.8586476910682, 466.19873762243884),
        ]
        circle = minimum_enclosing_circle(points)
        for point in points:
            self.assertLessEqual(
                math.hypot(point.x - circle.center.x, point.y - circle.center.y),
                circle.radius + 1e-9,
            )

    def test_seven_points_cover_dense_target_disk(self) -> None:
        points = coverage_points(1140.0)
        worst_sampled = 0.0
        for radial_index in range(101):
            radius = 1800.0 * radial_index / 100
            for angular_index in range(720):
                angle = 2.0 * math.pi * angular_index / 720
                x = radius * math.cos(angle)
                y = radius * math.sin(angle)
                nearest = min(math.hypot(x - point.x, y - point.y) for point in points)
                worst_sampled = max(worst_sampled, nearest)
        self.assertLessEqual(worst_sampled, 1000.0)
        self.assertAlmostEqual(
            worst_sampled, coverage_worst_distance(1140.0), places=6
        )

    def test_invalid_coverage_radius_is_rejected(self) -> None:
        simulator = LocalOmnidirectionalSimulator.random_case(1, 10)
        with self.assertRaises(ValueError):
            Problem3Strategy(
                simulator, StrategyConfig(coverage_ring_radius=1000.0)
            )

    def test_segment_guaranteed_interval_is_inside_every_reception_disk(self) -> None:
        simulator = LocalOmnidirectionalSimulator.random_case(1, 10)
        strategy = Problem3Strategy(simulator, self.fast_config())
        region = [
            Point(-100.0, -50.0),
            Point(100.0, -50.0),
            Point(100.0, 50.0),
            Point(-100.0, 50.0),
        ]
        start = Point(-1500.0, 0.0)
        end = Point(1500.0, 0.0)
        interval = strategy._segment_guaranteed_interval(start, end, region)
        self.assertIsNotNone(interval)
        assert interval is not None
        for fraction in interval:
            point = Point(
                start.x + fraction * (end.x - start.x),
                start.y + fraction * (end.y - start.y),
            )
            self.assertTrue(strategy._point_guarantees_reception(point, region))

    def test_exact_open_route_matches_brute_force(self) -> None:
        simulator = LocalOmnidirectionalSimulator.random_case(1, 10)
        strategy = Problem3Strategy(simulator, self.fast_config())
        start = Point(0.0, 0.0)
        targets = [
            Point(2.0, 0.0),
            Point(2.0, 100.0),
            Point(3.0, 0.0),
        ]
        order = strategy._optimal_open_route_order(start, targets)
        route_length = sum(
            strategy._distance(
                start if route_index == 0 else targets[order[route_index - 1]],
                targets[target_index],
            )
            for route_index, target_index in enumerate(order)
        )
        brute_force_length = min(
            sum(
                strategy._distance(
                    start if route_index == 0 else targets[candidate[route_index - 1]],
                    targets[target_index],
                )
                for route_index, target_index in enumerate(candidate)
            )
            for candidate in permutations(range(len(targets)))
        )
        self.assertAlmostEqual(route_length, brute_force_length, places=9)

    def test_adaptive_ring_route_preserves_short_hexagon_path(self) -> None:
        simulator = LocalOmnidirectionalSimulator.random_case(1, 10)
        strategy = Problem3Strategy(simulator, self.fast_config())
        strategy.tracks[1] = SourceTrack(
            channel=1,
            region=[
                Point(1390.0, 790.0),
                Point(1410.0, 800.0),
                Point(1395.0, 810.0),
            ],
        )
        ring = coverage_points(1140.0)[1:]
        route = strategy._choose_coverage_ring_route(ring)
        self.assertEqual(
            {(round(point.x, 6), round(point.y, 6)) for point in route},
            {(round(point.x, 6), round(point.y, 6)) for point in ring},
        )
        path_length = math.hypot(route[0].x, route[0].y) + sum(
            math.hypot(second.x - first.x, second.y - first.y)
            for first, second in zip(route, route[1:])
        )
        self.assertAlmostEqual(path_length, 6.0 * 1140.0, places=6)

    def test_boundary_sources_with_minimum_reception_radius_are_cleared(self) -> None:
        sources = []
        for channel in range(1, 13):
            angle = 2.0 * math.pi * channel / 12
            sources.append(
                OmnidirectionalSource(
                    channel,
                    1800.0 * math.cos(angle),
                    1800.0 * math.sin(angle),
                    1000.0,
                )
            )
        simulator = LocalOmnidirectionalSimulator(sources)
        strategy = Problem3Strategy(simulator, self.fast_config())
        summary = strategy.run()
        self.assertEqual(simulator.uncleared_channels(), [])
        self.assertEqual(summary.cleared_channels, list(range(1, 13)))

    def test_random_cases_clear_every_source(self) -> None:
        for seed in range(8):
            simulator = LocalOmnidirectionalSimulator.random_case(seed)
            strategy = Problem3Strategy(simulator, self.fast_config())
            summary = strategy.run()
            self.assertEqual(simulator.uncleared_channels(), [], msg=f"seed={seed}")
            self.assertEqual(
                summary.discovered_channels,
                sorted(simulator.sources),
                msg=f"seed={seed}",
            )

    @staticmethod
    def fast_config() -> StrategyConfig:
        return StrategyConfig(
            circle_side_count=180,
            candidate_angle_count=12,
            candidate_radial_levels=3,
            source_edge_subdivisions=3,
            source_interior_levels=2,
            error_sample_count=3,
            maximum_localization_measurements=10,
        )


if __name__ == "__main__":
    unittest.main()
