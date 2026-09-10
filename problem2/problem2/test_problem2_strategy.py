import math
import unittest

from problem2_strategy import (
    BearingObservation,
    HalfPlane,
    Point,
    clip_polygon_halfplane,
    first_feasible_region,
    guaranteed_core_boundary,
    optimize_second_station,
)


class Problem2StrategyTests(unittest.TestCase):
    def test_halfplane_clipping(self) -> None:
        square = [Point(-1, -1), Point(1, -1), Point(1, 1), Point(-1, 1)]
        clipped = clip_polygon_halfplane(square, HalfPlane(1, 0, 0, -1, "test"))
        self.assertEqual(len(clipped), 4)
        self.assertTrue(all(point.x >= -1e-9 for point in clipped))

    def test_outer_first_region_contains_exact_radial_boundary(self) -> None:
        first = BearingObservation(0.0, 0.0, 30.0)
        polygon, excess = first_feasible_region(
            first, 1.0, Point(0.0, 0.0), 1800.0, 1500.0, 360
        )
        self.assertLess(excess, 0.1)
        for angle_deg in (29.0, 29.5, 30.0, 30.5, 31.0):
            angle = math.radians(angle_deg)
            point = Point(1500.0 * math.cos(angle), 1500.0 * math.sin(angle))
            # Convex cross-product membership test.
            for index, first_vertex in enumerate(polygon):
                second_vertex = polygon[(index + 1) % len(polygon)]
                cross = (second_vertex.x - first_vertex.x) * (
                    point.y - first_vertex.y
                ) - (second_vertex.y - first_vertex.y) * (
                    point.x - first_vertex.x
                )
                self.assertGreaterEqual(cross, -1e-6)

    def test_guaranteed_core_really_guarantees_all_polygon_points(self) -> None:
        first = BearingObservation(0.0, 0.0, 30.0)
        polygon, _ = first_feasible_region(
            first, 1.0, Point(0.0, 0.0), 1800.0, 1500.0, 360
        )
        _, boundary = guaranteed_core_boundary(polygon, 1000.0, 96)
        self.assertTrue(boundary)
        for candidate in boundary:
            for vertex in polygon:
                self.assertLessEqual(
                    math.dist((candidate.x, candidate.y), (vertex.x, vertex.y)),
                    1000.0 + 1e-6,
                )

    def test_strategy_outputs_safe_recommendation(self) -> None:
        first = BearingObservation(0.0, 0.0, 30.0)
        result = optimize_second_station(
            first,
            circle_side_count=180,
            candidate_angle_count=16,
            candidate_radial_levels=4,
            source_edge_subdivisions=4,
            source_interior_levels=2,
            error_sample_count=3,
        )
        self.assertEqual(result.status, "ok")
        self.assertIsNotNone(result.optimum)
        for vertex in result.first_region:
            self.assertLessEqual(
                math.dist(
                    (result.optimum.point.x, result.optimum.point.y),
                    (vertex.x, vertex.y),
                ),
                1000.0 + 1e-6,
            )
        self.assertGreater(result.optimum.worst_diameter, 0.0)
        self.assertTrue(result.near_optimal)
        self.assertIsNotNone(result.center_baseline)
        self.assertIsNotNone(result.perpendicular_baseline)
        self.assertLessEqual(
            result.optimum.worst_diameter,
            result.perpendicular_baseline.worst_diameter + 1e-7,
        )

    def test_rotational_covariance_of_objective(self) -> None:
        options = dict(
            circle_side_count=180,
            candidate_angle_count=24,
            candidate_radial_levels=4,
            source_edge_subdivisions=4,
            source_interior_levels=2,
            error_sample_count=3,
        )
        first = optimize_second_station(BearingObservation(0.0, 0.0, 30.0), **options)
        rotated = optimize_second_station(BearingObservation(0.0, 0.0, 90.0), **options)
        self.assertAlmostEqual(
            first.optimum.worst_diameter,
            rotated.optimum.worst_diameter,
            places=6,
        )
        self.assertAlmostEqual(
            first.first_region_mec.radius,
            rotated.first_region_mec.radius,
            places=6,
        )


if __name__ == "__main__":
    unittest.main()
