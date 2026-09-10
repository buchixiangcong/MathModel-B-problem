#!/usr/bin/env python3
"""Robust second-station design for CUMCM 2026 Problem B, Question 2.

The first bearing feasible set is conservatively enclosed by a convex polygon.
Candidate stations are restricted to a proof-safe reception core. A finite,
refinable minimax search then evaluates the worst post-intersection diameter.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Sequence


PROJECT_ROOT = Path(__file__).resolve().parents[2]
PROBLEM1_DIR = PROJECT_ROOT / "problem1" / "problem1"
if str(PROBLEM1_DIR) not in sys.path:
    sys.path.insert(0, str(PROBLEM1_DIR))

from problem1_geometry import (  # noqa: E402
    BearingObservation,
    Circle,
    HalfPlane,
    Point,
    bearing_halfplanes,
    convex_hull,
    minimum_enclosing_circle,
    polygon_area,
    polygon_diameter,
)


EPS = 1e-8


@dataclass(frozen=True)
class CandidateEvaluation:
    point: Point
    worst_diameter: float
    worst_mec_radius: float
    distance_from_first: float
    worst_source: Point
    worst_error_deg: float
    crossing_angle_deg: float


@dataclass
class StrategyResult:
    status: str
    first_region: list[Point]
    first_region_area: float
    approximation_excess_m: float
    first_region_mec: Circle
    guaranteed_core_boundary: list[Point]
    candidates: list[CandidateEvaluation]
    optimum: Optional[CandidateEvaluation]
    near_optimal: list[CandidateEvaluation]
    center_baseline: Optional[CandidateEvaluation]
    perpendicular_baseline: Optional[CandidateEvaluation]
    message: str


def _distance(first: Point, second: Point) -> float:
    return math.hypot(first.x - second.x, first.y - second.y)


def _dot(first: Point, second: Point) -> float:
    return first.x * second.x + first.y * second.y


def _sub(first: Point, second: Point) -> Point:
    return Point(first.x - second.x, first.y - second.y)


def _point_on_segment(first: Point, second: Point, fraction: float) -> Point:
    return Point(
        first.x + fraction * (second.x - first.x),
        first.y + fraction * (second.y - first.y),
    )


def _inside_halfplane(point: Point, halfplane: HalfPlane) -> bool:
    lhs = halfplane.a * point.x + halfplane.b * point.y
    return lhs >= halfplane.c - EPS * max(1.0, abs(lhs), abs(halfplane.c))


def clip_polygon_halfplane(
    polygon: Sequence[Point], halfplane: HalfPlane
) -> list[Point]:
    """Clip a convex polygon by a*x+b*y>=c."""

    if not polygon:
        return []
    output: list[Point] = []
    previous = polygon[-1]
    previous_inside = _inside_halfplane(previous, halfplane)

    for current in polygon:
        current_inside = _inside_halfplane(current, halfplane)
        if current_inside != previous_inside:
            direction = _sub(current, previous)
            denominator = halfplane.a * direction.x + halfplane.b * direction.y
            if abs(denominator) > 1e-14:
                numerator = halfplane.c - (
                    halfplane.a * previous.x + halfplane.b * previous.y
                )
                fraction = min(1.0, max(0.0, numerator / denominator))
                output.append(_point_on_segment(previous, current, fraction))
        if current_inside:
            output.append(current)
        previous = current
        previous_inside = current_inside

    return convex_hull(output) if output else []


def circumscribed_disk_polygon(
    center: Point, radius: float, side_count: int
) -> list[Point]:
    """Return a regular polygon containing the entire closed disk."""

    if radius <= 0.0:
        raise ValueError("disk radius must be positive")
    if side_count < 12:
        raise ValueError("circle_side_count must be at least 12")
    vertex_radius = radius / math.cos(math.pi / side_count)
    offset = math.pi / side_count
    return [
        Point(
            center.x + vertex_radius * math.cos(offset + 2.0 * math.pi * index / side_count),
            center.y + vertex_radius * math.sin(offset + 2.0 * math.pi * index / side_count),
        )
        for index in range(side_count)
    ]


def disk_outer_halfplanes(
    center: Point, radius: float, side_count: int, label: str
) -> list[HalfPlane]:
    """Tangent half-planes whose intersection contains the given disk."""

    halfplanes: list[HalfPlane] = []
    for index in range(side_count):
        angle = 2.0 * math.pi * index / side_count
        nx, ny = math.cos(angle), math.sin(angle)
        # n dot (X-C) <= R, rewritten in the >= convention.
        halfplanes.append(
            HalfPlane(
                -nx,
                -ny,
                -radius - nx * center.x - ny * center.y,
                -1,
                label,
            )
        )
    return halfplanes


def first_feasible_region(
    first: BearingObservation,
    angle_error_deg: float,
    target_center: Point,
    target_radius: float,
    maximum_reception_radius: float,
    circle_side_count: int,
) -> tuple[list[Point], float]:
    """Construct a conservative polygonal enclosure of the first feasible set."""

    polygon = circumscribed_disk_polygon(
        target_center, target_radius, circle_side_count
    )
    constraints = list(
        disk_outer_halfplanes(
            first.station,
            maximum_reception_radius,
            circle_side_count,
            "maximum-reception-disk",
        )
    )
    constraints.extend(bearing_halfplanes(first, 0, angle_error_deg))
    for halfplane in constraints:
        polygon = clip_polygon_halfplane(polygon, halfplane)
        if not polygon:
            raise ValueError("the first observation has an empty feasible region")

    target_excess = target_radius * (1.0 / math.cos(math.pi / circle_side_count) - 1.0)
    reception_excess = maximum_reception_radius * (
        1.0 / math.cos(math.pi / circle_side_count) - 1.0
    )
    return convex_hull(polygon), max(target_excess, reception_excess)


def guaranteed_core_boundary(
    first_region: Sequence[Point],
    minimum_reception_radius: float,
    angle_count: int,
) -> tuple[Circle, list[Point]]:
    """Boundary of intersection B(V_k, R_min), sampled along radial rays."""

    enclosing = minimum_enclosing_circle(first_region)
    if enclosing.radius > minimum_reception_radius + EPS:
        return enclosing, []

    boundary: list[Point] = []
    center = enclosing.center
    for index in range(angle_count):
        angle = 2.0 * math.pi * index / angle_count
        direction = Point(math.cos(angle), math.sin(angle))
        upper = math.inf
        for vertex in first_region:
            relative = _sub(vertex, center)
            projection = _dot(relative, direction)
            perpendicular_sq = max(0.0, _dot(relative, relative) - projection * projection)
            discriminant = minimum_reception_radius**2 - perpendicular_sq
            if discriminant < -EPS:
                return enclosing, []
            upper = min(upper, projection + math.sqrt(max(0.0, discriminant)))
        boundary.append(
            Point(center.x + upper * direction.x, center.y + upper * direction.y)
        )
    return enclosing, boundary


def generate_candidate_points(
    center: Point,
    boundary: Sequence[Point],
    radial_levels: int,
) -> list[Point]:
    if not boundary:
        return []
    if radial_levels < 2:
        raise ValueError("candidate_radial_levels must be at least 2")
    candidates = [center]
    # Keep optimization samples slightly inside the certified boundary to
    # absorb floating-point and polygonal approximation tolerances.
    fractions = [0.95 * level / (radial_levels - 1) for level in range(1, radial_levels)]
    for boundary_point in boundary:
        for fraction in fractions:
            candidates.append(_point_on_segment(center, boundary_point, fraction))
    return candidates


def point_in_convex_polygon(point: Point, polygon: Sequence[Point]) -> bool:
    if len(polygon) < 3:
        return False
    for index, first in enumerate(polygon):
        second = polygon[(index + 1) % len(polygon)]
        cross = (second.x - first.x) * (point.y - first.y) - (
            second.y - first.y
        ) * (point.x - first.x)
        if cross < -EPS:
            return False
    return True


def generate_source_scenarios(
    polygon: Sequence[Point], edge_subdivisions: int, interior_levels: int
) -> list[Point]:
    """Deterministic boundary/interior discretization of the first region."""

    if edge_subdivisions < 2 or interior_levels < 1:
        raise ValueError("scenario resolutions are too small")
    centroid = Point(
        sum(point.x for point in polygon) / len(polygon),
        sum(point.y for point in polygon) / len(polygon),
    )
    scenarios = [centroid]
    for index, first in enumerate(polygon):
        second = polygon[(index + 1) % len(polygon)]
        for edge_index in range(edge_subdivisions):
            boundary_point = _point_on_segment(
                first, second, edge_index / edge_subdivisions
            )
            scenarios.append(boundary_point)
            for level in range(1, interior_levels):
                scenarios.append(
                    _point_on_segment(
                        centroid, boundary_point, level / interior_levels
                    )
                )
    return scenarios


def crossing_angle_deg(first: Point, second: Point, source: Point) -> float:
    first_vector = _sub(first, source)
    second_vector = _sub(second, source)
    first_length = math.hypot(first_vector.x, first_vector.y)
    second_length = math.hypot(second_vector.x, second_vector.y)
    if first_length <= EPS or second_length <= EPS:
        return 0.0
    cosine = _dot(first_vector, second_vector) / (first_length * second_length)
    angle = math.degrees(math.acos(min(1.0, max(-1.0, cosine))))
    return min(angle, 180.0 - angle)


def evaluate_candidate(
    candidate: Point,
    first_station: Point,
    first_region: Sequence[Point],
    source_scenarios: Sequence[Point],
    angle_error_deg: float,
    error_sample_count: int,
    optical_radius: float,
) -> CandidateEvaluation:
    if error_sample_count < 2:
        raise ValueError("error_sample_count must be at least 2")
    errors = [
        -angle_error_deg + 2.0 * angle_error_deg * index / (error_sample_count - 1)
        for index in range(error_sample_count)
    ]

    worst_diameter = -1.0
    worst_mec = -1.0
    worst_source = source_scenarios[0]
    worst_error = 0.0
    worst_crossing_angle = 0.0

    for source in source_scenarios:
        if _distance(candidate, source) <= optical_radius:
            scenario_diameter = 0.0
            scenario_mec = 0.0
            scenario_error = 0.0
        else:
            true_bearing = math.degrees(
                math.atan2(source.y - candidate.y, source.x - candidate.x)
            ) % 360.0
            scenario_diameter = -1.0
            scenario_mec = -1.0
            scenario_error = 0.0
            for error in errors:
                measured = (true_bearing + error) % 360.0
                polygon = list(first_region)
                observation = BearingObservation(candidate.x, candidate.y, measured)
                for halfplane in bearing_halfplanes(
                    observation, 1, angle_error_deg
                ):
                    polygon = clip_polygon_halfplane(polygon, halfplane)
                    if not polygon:
                        break
                if not polygon:
                    continue
                diameter = polygon_diameter(polygon).length
                mec_radius = minimum_enclosing_circle(polygon).radius
                if diameter > scenario_diameter:
                    scenario_diameter = diameter
                    scenario_error = error
                scenario_mec = max(scenario_mec, mec_radius)

        if scenario_diameter > worst_diameter:
            worst_diameter = scenario_diameter
            worst_source = source
            worst_error = scenario_error
            worst_crossing_angle = crossing_angle_deg(
                first_station, candidate, source
            )
        worst_mec = max(worst_mec, scenario_mec)

    return CandidateEvaluation(
        point=candidate,
        worst_diameter=worst_diameter,
        worst_mec_radius=worst_mec,
        distance_from_first=_distance(first_station, candidate),
        worst_source=worst_source,
        worst_error_deg=worst_error,
        crossing_angle_deg=worst_crossing_angle,
    )


def optimize_second_station(
    first: BearingObservation,
    angle_error_deg: float = 1.0,
    target_center: Point = Point(0.0, 0.0),
    target_radius: float = 1800.0,
    minimum_reception_radius: float = 1000.0,
    maximum_reception_radius: float = 1500.0,
    optical_radius: float = 5.0,
    circle_side_count: int = 360,
    candidate_angle_count: int = 48,
    candidate_radial_levels: int = 6,
    source_edge_subdivisions: int = 10,
    source_interior_levels: int = 4,
    error_sample_count: int = 5,
    near_optimal_tolerance: float = 0.05,
) -> StrategyResult:
    if minimum_reception_radius > maximum_reception_radius:
        raise ValueError("minimum reception radius cannot exceed maximum")
    if not 0.0 <= near_optimal_tolerance <= 1.0:
        raise ValueError("near_optimal_tolerance must lie in [0,1]")

    first_region, approximation_excess = first_feasible_region(
        first,
        angle_error_deg,
        target_center,
        target_radius,
        maximum_reception_radius,
        circle_side_count,
    )
    first_mec, core_boundary = guaranteed_core_boundary(
        first_region, minimum_reception_radius, candidate_angle_count
    )
    if not core_boundary:
        return StrategyResult(
            status="no-guaranteed-core",
            first_region=first_region,
            first_region_area=polygon_area(first_region),
            approximation_excess_m=approximation_excess,
            first_region_mec=first_mec,
            guaranteed_core_boundary=[],
            candidates=[],
            optimum=None,
            near_optimal=[],
            center_baseline=None,
            perpendicular_baseline=None,
            message=(
                "The conservative first feasible region cannot be enclosed by "
                "a radius-R_min circle, so the proof-safe candidate core is empty."
            ),
        )

    candidates = generate_candidate_points(
        first_mec.center, core_boundary, candidate_radial_levels
    )
    sources = generate_source_scenarios(
        first_region, source_edge_subdivisions, source_interior_levels
    )
    evaluations = [
        evaluate_candidate(
            candidate,
            first.station,
            first_region,
            sources,
            angle_error_deg,
            error_sample_count,
            optical_radius,
        )
        for candidate in candidates
    ]
    # Diameter is primary. Travel distance only breaks numerically equivalent ties.
    optimum = min(
        evaluations,
        key=lambda evaluation: (
            round(evaluation.worst_diameter, 9),
            evaluation.distance_from_first,
        ),
    )
    threshold = optimum.worst_diameter * (1.0 + near_optimal_tolerance)
    near_optimal = [
        evaluation
        for evaluation in evaluations
        if evaluation.worst_diameter <= threshold + EPS
    ]

    center_baseline = evaluate_candidate(
        first_mec.center,
        first.station,
        first_region,
        sources,
        angle_error_deg,
        error_sample_count,
        optical_radius,
    )
    perpendicular_directions = [
        math.radians(first.bearing_deg - 90.0),
        math.radians(first.bearing_deg + 90.0),
    ]
    perpendicular_points: list[Point] = []
    for target_angle in perpendicular_directions:
        target_direction = Point(math.cos(target_angle), math.sin(target_angle))
        boundary_point = max(
            core_boundary,
            key=lambda point: _dot(
                _sub(point, first_mec.center), target_direction
            )
            / max(EPS, _distance(point, first_mec.center)),
        )
        perpendicular_points.append(
            _point_on_segment(first_mec.center, boundary_point, 0.95)
        )
    perpendicular_baseline = min(
        (
            evaluate_candidate(
                point,
                first.station,
                first_region,
                sources,
                angle_error_deg,
                error_sample_count,
                optical_radius,
            )
            for point in perpendicular_points
        ),
        key=lambda evaluation: evaluation.worst_diameter,
    )
    return StrategyResult(
        status="ok",
        first_region=first_region,
        first_region_area=polygon_area(first_region),
        approximation_excess_m=approximation_excess,
        first_region_mec=first_mec,
        guaranteed_core_boundary=core_boundary,
        candidates=evaluations,
        optimum=optimum,
        near_optimal=near_optimal,
        center_baseline=center_baseline,
        perpendicular_baseline=perpendicular_baseline,
        message=(
            "The optimum minimizes sampled worst-case post-intersection diameter "
            "inside the proof-safe reception core."
        ),
    )


def _point_dict(point: Point) -> dict[str, float]:
    return {"x": point.x, "y": point.y}


def _evaluation_dict(evaluation: CandidateEvaluation) -> dict[str, object]:
    return {
        "point": _point_dict(evaluation.point),
        "worst_diameter": evaluation.worst_diameter,
        "worst_mec_radius": evaluation.worst_mec_radius,
        "distance_from_first": evaluation.distance_from_first,
        "worst_source": _point_dict(evaluation.worst_source),
        "worst_error_deg": evaluation.worst_error_deg,
        "crossing_angle_deg": evaluation.crossing_angle_deg,
    }


def result_to_dict(result: StrategyResult) -> dict[str, object]:
    return {
        "status": result.status,
        "message": result.message,
        "first_region": [_point_dict(point) for point in result.first_region],
        "first_region_area": result.first_region_area,
        "circle_outer_approximation_excess_m": result.approximation_excess_m,
        "first_region_minimum_enclosing_circle": {
            "center": _point_dict(result.first_region_mec.center),
            "radius": result.first_region_mec.radius,
        },
        "guaranteed_core_boundary": [
            _point_dict(point) for point in result.guaranteed_core_boundary
        ],
        "candidate_count": len(result.candidates),
        "optimum": None if result.optimum is None else _evaluation_dict(result.optimum),
        "near_optimal_count": len(result.near_optimal),
        "near_optimal_candidates": [
            _evaluation_dict(evaluation) for evaluation in result.near_optimal
        ],
        "baselines": {
            "first_region_mec_center": (
                None
                if result.center_baseline is None
                else _evaluation_dict(result.center_baseline)
            ),
            "perpendicular_heuristic": (
                None
                if result.perpendicular_baseline is None
                else _evaluation_dict(result.perpendicular_baseline)
            ),
        },
    }


def plot_strategy(
    first: BearingObservation,
    result: StrategyResult,
    output_path: Path,
) -> None:
    try:
        from PIL import Image, ImageDraw
    except ImportError as exc:
        raise RuntimeError("Pillow is required for --plot") from exc

    points = [first.station] + result.first_region + result.guaranteed_core_boundary
    if result.optimum is not None:
        points.append(result.optimum.point)
    x_min = min(point.x for point in points)
    x_max = max(point.x for point in points)
    y_min = min(point.y for point in points)
    y_max = max(point.y for point in points)
    span = max(x_max - x_min, y_max - y_min, 100.0)
    x_min -= 0.12 * span
    x_max += 0.12 * span
    y_min -= 0.12 * span
    y_max += 0.12 * span

    width = height = 1500
    margin = 120
    scale = min(
        (width - 2 * margin) / (x_max - x_min),
        (height - 2 * margin) / (y_max - y_min),
    )

    def pixel(point: Point) -> tuple[float, float]:
        return (
            margin + (point.x - x_min) * scale,
            height - margin - (point.y - y_min) * scale,
        )

    image = Image.new("RGB", (width, height), "white")
    draw = ImageDraw.Draw(image, "RGBA")
    draw.rectangle(
        [(margin, margin), (width - margin, height - margin)],
        outline=(80, 80, 80, 255),
        width=2,
    )
    if len(result.first_region) >= 3:
        draw.polygon(
            [pixel(point) for point in result.first_region],
            fill=(214, 39, 40, 50),
            outline=(140, 29, 24, 255),
            width=4,
        )
    if len(result.guaranteed_core_boundary) >= 3:
        draw.polygon(
            [pixel(point) for point in result.guaranteed_core_boundary],
            fill=(31, 119, 180, 45),
            outline=(31, 119, 180, 255),
            width=4,
        )

    for evaluation in result.candidates:
        px, py = pixel(evaluation.point)
        draw.ellipse([px - 2, py - 2, px + 2, py + 2], fill=(150, 150, 150, 150))
    for evaluation in result.near_optimal:
        px, py = pixel(evaluation.point)
        draw.ellipse([px - 5, py - 5, px + 5, py + 5], fill=(255, 127, 14, 230))

    first_px = pixel(first.station)
    draw.ellipse(
        [first_px[0] - 9, first_px[1] - 9, first_px[0] + 9, first_px[1] + 9],
        fill=(20, 20, 20, 255),
    )
    draw.text((first_px[0] + 12, first_px[1] - 22), "S1", fill=(20, 20, 20, 255))

    if result.optimum is not None:
        optimum_px = pixel(result.optimum.point)
        draw.ellipse(
            [
                optimum_px[0] - 11,
                optimum_px[1] - 11,
                optimum_px[0] + 11,
                optimum_px[1] + 11,
            ],
            fill=(44, 160, 44, 255),
        )
        draw.text(
            (optimum_px[0] + 14, optimum_px[1] - 22),
            "recommended S2",
            fill=(20, 90, 20, 255),
        )

    title = "Question 2 robust second-station design"
    if result.optimum is not None:
        title += f" | worst D={result.optimum.worst_diameter:.2f} m"
    draw.text((margin, 42), title, fill=(20, 20, 20, 255))
    draw.text(
        (margin, height - 55),
        "red: first feasible region   blue: guaranteed reception core   orange: 5% near-optimal   green: recommendation",
        fill=(40, 40, 40, 255),
    )
    image.save(output_path)


def load_input(path: Path) -> tuple[BearingObservation, dict[str, object]]:
    data = json.loads(path.read_text(encoding="utf-8"))
    first_data = data["first_observation"]
    first = BearingObservation(
        float(first_data["x"]),
        float(first_data["y"]),
        float(first_data["bearing_deg"]),
    )
    options = {key: value for key, value in data.items() if key != "first_observation"}
    if "target_center" in options:
        center = options["target_center"]
        options["target_center"] = Point(float(center["x"]), float(center["y"]))
    return first, options


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--plot", type=Path)
    arguments = parser.parse_args()

    first, options = load_input(arguments.input)
    result = optimize_second_station(first, **options)
    serialized = json.dumps(result_to_dict(result), ensure_ascii=False, indent=2)
    if arguments.output:
        arguments.output.write_text(serialized + "\n", encoding="utf-8")
    else:
        print(serialized)
    if arguments.plot:
        plot_strategy(first, result, arguments.plot)


if __name__ == "__main__":
    main()
