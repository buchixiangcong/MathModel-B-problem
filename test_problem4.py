from __future__ import annotations

import math
import unittest

from local_simulator_problem4 import LocalMixedSimulator, MixedSource
from problem1.problem1.problem1_geometry import Point
from problem4.problem4_strategy import Problem4Strategy, StrategyConfig, nearest_station_convex_hull_contains, polar_mesh_points, triangular_lattice_points


class Problem4Tests(unittest.TestCase):
    def test_lattice_has_directional_discovery_certificate(self) -> None:
        stations = triangular_lattice_points(1000.0, 2800.0)
        self.assertEqual(len(stations), 31)
        for radial in range(0, 1801, 150):
            for index in range(72):
                angle = 2.0 * math.pi * index / 72.0
                source = Point(radial * math.cos(angle), radial * math.sin(angle))
                self.assertTrue(nearest_station_convex_hull_contains(source, stations))

    def test_polar_mesh_has_directional_discovery_certificate(self) -> None:
        stations = polar_mesh_points()
        self.assertEqual(len(stations), 25)
        for radial in range(0, 1801, 25):
            for index in range(360):
                angle = 2.0 * math.pi * index / 360.0
                source = Point(radial * math.cos(angle), radial * math.sin(angle))
                self.assertTrue(nearest_station_convex_hull_contains(source, stations))

    def test_local_simulator_directional_half_plane(self) -> None:
        simulator = LocalMixedSimulator([MixedSource(1, 0.0, 0.0, 1200.0, 0.0)])
        simulator.enter()
        self.assertEqual(simulator.measure(100.0, 0.0, 1)["measure_result"], "direction")
        self.assertEqual(simulator.measure(-100.0, 0.0, 1)["measure_result"], "no_signal")

    def test_random_mixed_cases_clear(self) -> None:
        for seed in range(5):
            simulator = LocalMixedSimulator.random_case(seed, 10)
            strategy = Problem4Strategy(simulator, StrategyConfig(circle_side_count=120, candidate_angle_count=10,
                candidate_radial_levels=2, source_edge_subdivisions=3, source_interior_levels=1,
                error_sample_count=2, maximum_localization_measurements=80, max_grid_localization_rounds=8))
            summary = strategy.run()
            self.assertEqual(simulator.uncleared_channels(), [], msg=f"seed={seed}, summary={summary}")

    def test_all_directional_boundary_sources_with_minimum_radius(self) -> None:
        sources = []
        for index in range(16):
            angle = 2.0 * math.pi * index / 16.0
            sources.append(
                MixedSource(
                    channel=index + 1,
                    x=1800.0 * math.cos(angle),
                    y=1800.0 * math.sin(angle),
                    reception_radius=1000.0,
                    directional_axis_deg=(math.degrees(angle) + 180.0 + 37.0 * (index % 3)) % 360.0,
                )
            )
        simulator = LocalMixedSimulator(sources)
        strategy = Problem4Strategy(
            simulator,
            StrategyConfig(
                circle_side_count=120,
                candidate_angle_count=10,
                candidate_radial_levels=2,
                source_edge_subdivisions=3,
                source_interior_levels=1,
                error_sample_count=2,
            ),
        )
        summary = strategy.run()
        self.assertEqual(simulator.uncleared_channels(), [])
        self.assertEqual(summary.cleared_channels, list(range(1, 17)))


if __name__ == "__main__":
    unittest.main()
