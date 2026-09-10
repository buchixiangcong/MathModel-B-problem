import math
import random
import unittest

from problem1_geometry import (
    BearingObservation,
    Circle,
    Point,
    _circle_from_two,
    _circumcircle,
    analyze_observations,
    convex_hull,
    minimum_enclosing_circle,
    polygon_diameter,
)


class Problem1GeometryTests(unittest.TestCase):
    def test_symmetric_crossing_has_bounded_quadrilateral(self) -> None:
        observations = [
            BearingObservation(-1000.0, 0.0, 45.0),
            BearingObservation(1000.0, 0.0, 135.0),
        ]
        result = analyze_observations(observations)
        self.assertEqual(result.status, "bounded")
        self.assertEqual(len(result.vertices), 4)
        self.assertIsNotNone(result.diameter)
        self.assertIsNotNone(result.minimum_enclosing_circle)
        self.assertTrue(result.diameter_circle_covers)

    def test_single_observation_is_unbounded(self) -> None:
        result = analyze_observations([BearingObservation(0.0, 0.0, 359.5)])
        self.assertEqual(result.status, "unbounded")
        self.assertIsNone(result.diameter)

    def test_opposing_outward_sectors_are_empty(self) -> None:
        observations = [
            BearingObservation(-100.0, 0.0, 180.0),
            BearingObservation(100.0, 0.0, 0.0),
        ]
        result = analyze_observations(observations)
        self.assertEqual(result.status, "empty")

    def test_equilateral_triangle_disproves_general_circle_claim(self) -> None:
        side = 2.0
        vertices = [
            Point(0.0, 0.0),
            Point(side, 0.0),
            Point(side / 2.0, math.sqrt(3.0)),
        ]
        diameter = polygon_diameter(vertices)
        circle = minimum_enclosing_circle(vertices)
        self.assertAlmostEqual(diameter.length, side, places=10)
        self.assertAlmostEqual(circle.radius, side / math.sqrt(3.0), places=10)
        self.assertGreater(circle.radius, diameter.length / 2.0)

    def test_rotating_calipers_matches_brute_force(self) -> None:
        generator = random.Random(20260910)
        for _ in range(100):
            cloud = [
                Point(generator.uniform(-100.0, 100.0), generator.uniform(-100.0, 100.0))
                for _ in range(30)
            ]
            hull = convex_hull(cloud)
            result = polygon_diameter(hull)
            brute_force = max(
                math.dist((first.x, first.y), (second.x, second.y))
                for i, first in enumerate(hull)
                for second in hull[i + 1 :]
            )
            self.assertAlmostEqual(result.length, brute_force, places=9)

    def test_minimum_circle_contains_all_random_hull_vertices(self) -> None:
        generator = random.Random(12345)
        cloud = [
            Point(generator.uniform(-50.0, 80.0), generator.uniform(-30.0, 100.0))
            for _ in range(60)
        ]
        hull = convex_hull(cloud)
        circle = minimum_enclosing_circle(hull)
        for point in hull:
            self.assertLessEqual(
                math.dist((point.x, point.y), (circle.center.x, circle.center.y)),
                circle.radius + 1e-7,
            )

    def test_welzl_circle_matches_exhaustive_support_search(self) -> None:
        generator = random.Random(712367)

        def contains(circle: Circle, point: Point) -> bool:
            return math.dist(
                (point.x, point.y), (circle.center.x, circle.center.y)
            ) <= circle.radius + 1e-7

        for _ in range(30):
            cloud = [
                Point(generator.uniform(-20.0, 20.0), generator.uniform(-20.0, 20.0))
                for _ in range(12)
            ]
            hull = convex_hull(cloud)
            candidates: list[Circle] = []
            for i, first in enumerate(hull):
                for j, second in enumerate(hull[i + 1 :], start=i + 1):
                    pair_circle = _circle_from_two(first, second)
                    if all(contains(pair_circle, point) for point in hull):
                        candidates.append(pair_circle)
                    for third in hull[j + 1 :]:
                        triple_circle = _circumcircle(first, second, third)
                        if triple_circle is not None and all(
                            contains(triple_circle, point) for point in hull
                        ):
                            candidates.append(triple_circle)

            exhaustive_radius = min(circle.radius for circle in candidates)
            welzl_radius = minimum_enclosing_circle(hull).radius
            self.assertAlmostEqual(welzl_radius, exhaustive_radius, places=8)


if __name__ == "__main__":
    unittest.main()
