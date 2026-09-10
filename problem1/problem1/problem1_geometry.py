#!/usr/bin/env python3
"""Geometry solver for CUMCM 2026 Problem B, Question 1.

The solver treats every bearing as a closed angular sector with a bounded
error, intersects the corresponding half-planes, computes the convex
positioning polygon, and evaluates both its diameter and enclosing circles.
Only the Python standard library is required unless --plot is used.
"""

from __future__ import annotations

import argparse
import json
import math
import random
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional, Sequence


ANGLE_EPS = 1e-12
FEASIBILITY_EPS = 1e-8
POINT_EPS = 1e-7


@dataclass(frozen=True)
class Point:
    x: float
    y: float


@dataclass(frozen=True)
class BearingObservation:
    x: float
    y: float
    bearing_deg: float

    @property
    def station(self) -> Point:
        return Point(self.x, self.y)


@dataclass(frozen=True)
class HalfPlane:
    """Closed half-plane a*x + b*y >= c with a unit normal."""

    a: float
    b: float
    c: float
    observation_index: int
    boundary: str


@dataclass(frozen=True)
class Circle:
    center: Point
    radius: float


@dataclass(frozen=True)
class DiameterResult:
    length: float
    first: Point
    second: Point


@dataclass
class PositioningResult:
    status: str
    vertices: list[Point]
    area: Optional[float]
    diameter: Optional[DiameterResult]
    minimum_enclosing_circle: Optional[Circle]
    diameter_circle: Optional[Circle]
    diameter_circle_covers: Optional[bool]
    coverage_margin: Optional[float]
    message: str


def _sub(p: Point, q: Point) -> Point:
    return Point(p.x - q.x, p.y - q.y)


def _cross(p: Point, q: Point) -> float:
    return p.x * q.y - p.y * q.x


def _cross3(o: Point, a: Point, b: Point) -> float:
    return _cross(_sub(a, o), _sub(b, o))


def _distance_sq(p: Point, q: Point) -> float:
    dx = p.x - q.x
    dy = p.y - q.y
    return dx * dx + dy * dy


def _distance(p: Point, q: Point) -> float:
    return math.sqrt(_distance_sq(p, q))


def bearing_halfplanes(
    observation: BearingObservation,
    observation_index: int,
    angle_error_deg: float = 1.0,
) -> tuple[HalfPlane, HalfPlane]:
    """Convert one bearing sector into its two supporting half-planes."""

    if not 0.0 < angle_error_deg < 90.0:
        raise ValueError("angle_error_deg must lie strictly between 0 and 90")

    lower = math.radians((observation.bearing_deg - angle_error_deg) % 360.0)
    upper = math.radians((observation.bearing_deg + angle_error_deg) % 360.0)

    # cross(e_lower, X-S) >= 0
    lower_normal = Point(-math.sin(lower), math.cos(lower))
    lower_hp = HalfPlane(
        lower_normal.x,
        lower_normal.y,
        lower_normal.x * observation.x + lower_normal.y * observation.y,
        observation_index,
        "lower",
    )

    # cross(X-S, e_upper) >= 0
    upper_normal = Point(math.sin(upper), -math.cos(upper))
    upper_hp = HalfPlane(
        upper_normal.x,
        upper_normal.y,
        upper_normal.x * observation.x + upper_normal.y * observation.y,
        observation_index,
        "upper",
    )
    return lower_hp, upper_hp


def build_halfplanes(
    observations: Sequence[BearingObservation], angle_error_deg: float = 1.0
) -> list[HalfPlane]:
    if not observations:
        raise ValueError("at least one bearing observation is required")
    halfplanes: list[HalfPlane] = []
    for index, observation in enumerate(observations):
        halfplanes.extend(
            bearing_halfplanes(observation, index, angle_error_deg)
        )
    return halfplanes


def _line_intersection(first: HalfPlane, second: HalfPlane) -> Optional[Point]:
    det = first.a * second.b - second.a * first.b
    if abs(det) <= ANGLE_EPS:
        return None
    x = (first.c * second.b - first.b * second.c) / det
    y = (first.a * second.c - first.c * second.a) / det
    return Point(x, y)


def _satisfies(point: Point, halfplane: HalfPlane) -> bool:
    lhs = halfplane.a * point.x + halfplane.b * point.y
    scale = max(1.0, abs(lhs), abs(halfplane.c))
    return lhs >= halfplane.c - FEASIBILITY_EPS * scale


def _append_unique(points: list[Point], candidate: Point) -> None:
    threshold_sq = POINT_EPS * POINT_EPS
    if all(_distance_sq(candidate, point) > threshold_sq for point in points):
        points.append(candidate)


def feasible_intersection_vertices(halfplanes: Sequence[HalfPlane]) -> list[Point]:
    """Enumerate all feasible intersections of pairs of boundary lines."""

    vertices: list[Point] = []
    for i, first in enumerate(halfplanes):
        for second in halfplanes[i + 1 :]:
            candidate = _line_intersection(first, second)
            if candidate is None:
                continue
            if all(_satisfies(candidate, hp) for hp in halfplanes):
                _append_unique(vertices, candidate)
    return vertices


def has_unbounded_recession_direction(halfplanes: Sequence[HalfPlane]) -> bool:
    """Return whether a nonzero d satisfies a_j dot d >= 0 for all j."""

    # If the recession cone is nontrivial, one of its extreme rays lies on a
    # supporting line a_j dot d = 0. Test both directions of every such line.
    for hp in halfplanes:
        for direction in (Point(hp.b, -hp.a), Point(-hp.b, hp.a)):
            if all(
                other.a * direction.x + other.b * direction.y >= -ANGLE_EPS
                for other in halfplanes
            ):
                return True
    return False


def convex_hull(points: Iterable[Point]) -> list[Point]:
    """Andrew monotone-chain hull in counterclockwise order."""

    ordered = sorted(points, key=lambda point: (point.x, point.y))
    unique: list[Point] = []
    for point in ordered:
        if not unique or _distance_sq(point, unique[-1]) > POINT_EPS**2:
            unique.append(point)
    if len(unique) <= 1:
        return unique

    def build_half(sequence: Iterable[Point]) -> list[Point]:
        half: list[Point] = []
        for point in sequence:
            while len(half) >= 2 and _cross3(half[-2], half[-1], point) <= ANGLE_EPS:
                half.pop()
            half.append(point)
        return half

    lower = build_half(unique)
    upper = build_half(reversed(unique))
    return lower[:-1] + upper[:-1]


def polygon_area(vertices: Sequence[Point]) -> float:
    if len(vertices) < 3:
        return 0.0
    doubled = sum(
        _cross(vertices[index], vertices[(index + 1) % len(vertices)])
        for index in range(len(vertices))
    )
    return abs(doubled) / 2.0


def polygon_diameter(vertices: Sequence[Point]) -> DiameterResult:
    """Compute a convex polygon's diameter by rotating calipers."""

    count = len(vertices)
    if count == 0:
        raise ValueError("diameter is undefined for an empty point set")
    if count == 1:
        return DiameterResult(0.0, vertices[0], vertices[0])
    if count == 2:
        return DiameterResult(_distance(vertices[0], vertices[1]), vertices[0], vertices[1])

    best_sq = -1.0
    best_pair = (vertices[0], vertices[1])

    def consider(first: Point, second: Point) -> None:
        nonlocal best_sq, best_pair
        distance_sq = _distance_sq(first, second)
        if distance_sq > best_sq:
            best_sq = distance_sq
            best_pair = (first, second)

    opposite = 1
    for index in range(count):
        next_index = (index + 1) % count
        steps = 0
        while steps < count:
            next_opposite = (opposite + 1) % count
            current_area = abs(
                _cross3(vertices[index], vertices[next_index], vertices[opposite])
            )
            next_area = abs(
                _cross3(vertices[index], vertices[next_index], vertices[next_opposite])
            )
            if next_area > current_area + ANGLE_EPS:
                opposite = next_opposite
                steps += 1
            else:
                break

        consider(vertices[index], vertices[opposite])
        consider(vertices[next_index], vertices[opposite])

        next_opposite = (opposite + 1) % count
        current_area = abs(
            _cross3(vertices[index], vertices[next_index], vertices[opposite])
        )
        next_area = abs(
            _cross3(vertices[index], vertices[next_index], vertices[next_opposite])
        )
        if abs(next_area - current_area) <= ANGLE_EPS:
            consider(vertices[index], vertices[next_opposite])
            consider(vertices[next_index], vertices[next_opposite])

    return DiameterResult(math.sqrt(best_sq), best_pair[0], best_pair[1])


def _circle_from_one(point: Point) -> Circle:
    return Circle(point, 0.0)


def _circle_from_two(first: Point, second: Point) -> Circle:
    center = Point((first.x + second.x) / 2.0, (first.y + second.y) / 2.0)
    return Circle(center, _distance(first, second) / 2.0)


def _circumcircle(first: Point, second: Point, third: Point) -> Optional[Circle]:
    ax, ay = first.x, first.y
    bx, by = second.x, second.y
    cx, cy = third.x, third.y
    denominator = 2.0 * (
        ax * (by - cy) + bx * (cy - ay) + cx * (ay - by)
    )
    scale = max(
        1.0,
        _distance_sq(first, second),
        _distance_sq(second, third),
        _distance_sq(third, first),
    )
    if abs(denominator) <= ANGLE_EPS * scale:
        return None

    first_norm = ax * ax + ay * ay
    second_norm = bx * bx + by * by
    third_norm = cx * cx + cy * cy
    ux = (
        first_norm * (by - cy)
        + second_norm * (cy - ay)
        + third_norm * (ay - by)
    ) / denominator
    uy = (
        first_norm * (cx - bx)
        + second_norm * (ax - cx)
        + third_norm * (bx - ax)
    ) / denominator
    center = Point(ux, uy)
    return Circle(center, _distance(center, first))


def _circle_contains(circle: Circle, point: Point) -> bool:
    tolerance = FEASIBILITY_EPS * max(1.0, circle.radius)
    return _distance(point, circle.center) <= circle.radius + tolerance


def _trivial_minimum_circle(boundary: Sequence[Point]) -> Optional[Circle]:
    if not boundary:
        return None
    if len(boundary) == 1:
        return _circle_from_one(boundary[0])

    candidates: list[Circle] = []
    for i, first in enumerate(boundary):
        for second in boundary[i + 1 :]:
            circle = _circle_from_two(first, second)
            if all(_circle_contains(circle, point) for point in boundary):
                candidates.append(circle)

    if len(boundary) == 3:
        circumcircle = _circumcircle(boundary[0], boundary[1], boundary[2])
        if circumcircle is not None and all(
            _circle_contains(circumcircle, point) for point in boundary
        ):
            candidates.append(circumcircle)

    if not candidates:
        raise ArithmeticError("could not construct a circle for boundary points")
    return min(candidates, key=lambda circle: circle.radius)


def minimum_enclosing_circle(points: Sequence[Point], seed: int = 20260910) -> Circle:
    """Expected-linear-time Welzl algorithm with deterministic shuffling."""

    if not points:
        raise ValueError("minimum enclosing circle is undefined for no points")

    shuffled = list(points)
    random.Random(seed).shuffle(shuffled)

    def welzl(count: int, boundary: list[Point]) -> Optional[Circle]:
        if count == 0 or len(boundary) == 3:
            return _trivial_minimum_circle(boundary)
        point = shuffled[count - 1]
        circle = welzl(count - 1, boundary)
        if circle is not None and _circle_contains(circle, point):
            return circle
        return welzl(count - 1, boundary + [point])

    result = welzl(len(shuffled), [])
    if result is None:
        raise ArithmeticError("minimum enclosing circle construction failed")
    return result


def analyze_observations(
    observations: Sequence[BearingObservation], angle_error_deg: float = 1.0
) -> PositioningResult:
    halfplanes = build_halfplanes(observations, angle_error_deg)
    feasible_vertices = feasible_intersection_vertices(halfplanes)

    if not feasible_vertices:
        return PositioningResult(
            status="empty",
            vertices=[],
            area=None,
            diameter=None,
            minimum_enclosing_circle=None,
            diameter_circle=None,
            diameter_circle_covers=None,
            coverage_margin=None,
            message="The bearing sectors have no common feasible point.",
        )

    hull = convex_hull(feasible_vertices)
    if has_unbounded_recession_direction(halfplanes):
        return PositioningResult(
            status="unbounded",
            vertices=hull,
            area=None,
            diameter=None,
            minimum_enclosing_circle=None,
            diameter_circle=None,
            diameter_circle_covers=None,
            coverage_margin=None,
            message="The feasible intersection is unbounded, so its diameter is infinite.",
        )

    diameter = polygon_diameter(hull)
    enclosing_circle = minimum_enclosing_circle(hull)
    diameter_circle = _circle_from_two(diameter.first, diameter.second)
    max_distance = max(_distance(diameter_circle.center, point) for point in hull)
    margin = diameter_circle.radius - max_distance
    covers = margin >= -FEASIBILITY_EPS * max(1.0, diameter.length)
    status = "degenerate" if len(hull) < 3 or polygon_area(hull) <= POINT_EPS else "bounded"

    return PositioningResult(
        status=status,
        vertices=hull,
        area=polygon_area(hull),
        diameter=diameter,
        minimum_enclosing_circle=enclosing_circle,
        diameter_circle=diameter_circle,
        diameter_circle_covers=covers,
        coverage_margin=margin,
        message=(
            "The diameter circle covers the positioning region."
            if covers
            else "The diameter circle does not cover the positioning region."
        ),
    )


def _point_dict(point: Point) -> dict[str, float]:
    return {"x": point.x, "y": point.y}


def result_to_dict(result: PositioningResult) -> dict[str, object]:
    output: dict[str, object] = {
        "status": result.status,
        "message": result.message,
        "vertices": [_point_dict(point) for point in result.vertices],
        "area": result.area,
        "diameter_circle_covers": result.diameter_circle_covers,
        "coverage_margin": result.coverage_margin,
    }
    if result.diameter is not None:
        output["diameter"] = {
            "length": result.diameter.length,
            "first": _point_dict(result.diameter.first),
            "second": _point_dict(result.diameter.second),
        }
    else:
        output["diameter"] = None

    for name, circle in (
        ("minimum_enclosing_circle", result.minimum_enclosing_circle),
        ("diameter_circle", result.diameter_circle),
    ):
        output[name] = (
            None
            if circle is None
            else {"center": _point_dict(circle.center), "radius": circle.radius}
        )
    return output


def plot_result(
    observations: Sequence[BearingObservation],
    result: PositioningResult,
    angle_error_deg: float,
    output_path: Path,
) -> None:
    try:
        import matplotlib.pyplot as plt
        from matplotlib.patches import Circle as CirclePatch
        from matplotlib.patches import Polygon
    except ImportError:
        _plot_result_with_pillow(
            observations, result, angle_error_deg, output_path
        )
        return

    figure, axis = plt.subplots(figsize=(8, 8), constrained_layout=True)
    all_points = [observation.station for observation in observations] + result.vertices
    if not all_points:
        all_points = [Point(0.0, 0.0)]
    xs = [point.x for point in all_points]
    ys = [point.y for point in all_points]
    span = max(max(xs) - min(xs), max(ys) - min(ys), 100.0)
    ray_length = 1.2 * span

    for index, observation in enumerate(observations):
        axis.scatter(observation.x, observation.y, color="black", s=35, zorder=5)
        axis.annotate(
            f"S{index + 1}",
            (observation.x, observation.y),
            xytext=(5, 5),
            textcoords="offset points",
        )
        for angle, style, alpha in (
            (observation.bearing_deg, "-", 0.8),
            (observation.bearing_deg - angle_error_deg, "--", 0.45),
            (observation.bearing_deg + angle_error_deg, "--", 0.45),
        ):
            radians = math.radians(angle)
            axis.plot(
                [observation.x, observation.x + ray_length * math.cos(radians)],
                [observation.y, observation.y + ray_length * math.sin(radians)],
                color="#555555",
                linestyle=style,
                alpha=alpha,
                linewidth=1.2,
            )

    if len(result.vertices) >= 3:
        polygon = Polygon(
            [(point.x, point.y) for point in result.vertices],
            closed=True,
            facecolor="#d62728",
            edgecolor="#8c1d18",
            alpha=0.23,
            label="Positioning region",
        )
        axis.add_patch(polygon)

    if result.diameter is not None:
        axis.plot(
            [result.diameter.first.x, result.diameter.second.x],
            [result.diameter.first.y, result.diameter.second.y],
            color="#d62728",
            linewidth=2.2,
            label=f"Diameter D={result.diameter.length:.3f}",
        )

    if result.minimum_enclosing_circle is not None:
        circle = result.minimum_enclosing_circle
        axis.add_patch(
            CirclePatch(
                (circle.center.x, circle.center.y),
                circle.radius,
                fill=False,
                edgecolor="#1f77b4",
                linewidth=1.8,
                label=f"MEC r={circle.radius:.3f}",
            )
        )

    if result.diameter_circle is not None:
        circle = result.diameter_circle
        axis.add_patch(
            CirclePatch(
                (circle.center.x, circle.center.y),
                circle.radius,
                fill=False,
                edgecolor="#2ca02c",
                linestyle=":",
                linewidth=1.8,
                label="Circle with diameter D",
            )
        )

    axis.set_aspect("equal", adjustable="box")
    axis.set_xlabel("x / m")
    axis.set_ylabel("y / m")
    axis.set_title(f"Bearing-sector intersection ({result.status})")
    axis.grid(True, linestyle=":", linewidth=0.7, alpha=0.55)
    handles, labels = axis.get_legend_handles_labels()
    if handles:
        axis.legend(loc="best")
    figure.savefig(output_path, dpi=220)
    plt.close(figure)


def _plot_result_with_pillow(
    observations: Sequence[BearingObservation],
    result: PositioningResult,
    angle_error_deg: float,
    output_path: Path,
) -> None:
    """Dependency-light plotting fallback used when matplotlib is absent."""

    try:
        from PIL import Image, ImageDraw
    except ImportError as exc:
        raise RuntimeError("--plot requires either matplotlib or Pillow") from exc

    width = height = 1400
    margin = 110
    points = [observation.station for observation in observations] + result.vertices
    if not points:
        points = [Point(0.0, 0.0)]
    xs = [point.x for point in points]
    ys = [point.y for point in points]

    for circle in (result.minimum_enclosing_circle, result.diameter_circle):
        if circle is not None:
            xs.extend([circle.center.x - circle.radius, circle.center.x + circle.radius])
            ys.extend([circle.center.y - circle.radius, circle.center.y + circle.radius])

    x_min, x_max = min(xs), max(xs)
    y_min, y_max = min(ys), max(ys)
    x_span = max(x_max - x_min, 100.0)
    y_span = max(y_max - y_min, 100.0)
    x_min -= 0.08 * x_span
    x_max += 0.08 * x_span
    y_min -= 0.08 * y_span
    y_max += 0.08 * y_span
    scale = min(
        (width - 2 * margin) / (x_max - x_min),
        (height - 2 * margin) / (y_max - y_min),
    )

    def pixel(point: Point) -> tuple[float, float]:
        px = margin + (point.x - x_min) * scale
        py = height - margin - (point.y - y_min) * scale
        return px, py

    image = Image.new("RGB", (width, height), "white")
    draw = ImageDraw.Draw(image, "RGBA")

    for fraction in (0.0, 0.25, 0.5, 0.75, 1.0):
        x = margin + fraction * (width - 2 * margin)
        y = margin + fraction * (height - 2 * margin)
        draw.line([(x, margin), (x, height - margin)], fill=(215, 215, 215, 255), width=1)
        draw.line([(margin, y), (width - margin, y)], fill=(215, 215, 215, 255), width=1)

    ray_length = 1.5 * max(x_max - x_min, y_max - y_min)
    for index, observation in enumerate(observations):
        start = observation.station
        for angle, fill, line_width in (
            (observation.bearing_deg, (70, 70, 70, 230), 4),
            (observation.bearing_deg - angle_error_deg, (120, 120, 120, 150), 2),
            (observation.bearing_deg + angle_error_deg, (120, 120, 120, 150), 2),
        ):
            radians = math.radians(angle)
            end = Point(
                start.x + ray_length * math.cos(radians),
                start.y + ray_length * math.sin(radians),
            )
            draw.line([pixel(start), pixel(end)], fill=fill, width=line_width)
        station_pixel = pixel(start)
        radius = 8
        draw.ellipse(
            [
                station_pixel[0] - radius,
                station_pixel[1] - radius,
                station_pixel[0] + radius,
                station_pixel[1] + radius,
            ],
            fill=(20, 20, 20, 255),
        )
        draw.text(
            (station_pixel[0] + 12, station_pixel[1] - 24),
            f"S{index + 1}",
            fill=(20, 20, 20, 255),
        )

    if len(result.vertices) >= 3:
        draw.polygon(
            [pixel(point) for point in result.vertices],
            fill=(214, 39, 40, 60),
            outline=(140, 29, 24, 255),
            width=4,
        )

    if result.diameter is not None:
        draw.line(
            [pixel(result.diameter.first), pixel(result.diameter.second)],
            fill=(214, 39, 40, 255),
            width=6,
        )

    def draw_circle(circle: Circle, color: tuple[int, int, int, int], width_px: int) -> None:
        center_x, center_y = pixel(circle.center)
        radius_px = circle.radius * scale
        draw.ellipse(
            [
                center_x - radius_px,
                center_y - radius_px,
                center_x + radius_px,
                center_y + radius_px,
            ],
            outline=color,
            width=width_px,
        )

    if result.minimum_enclosing_circle is not None:
        draw_circle(result.minimum_enclosing_circle, (31, 119, 180, 255), 4)
    if result.diameter_circle is not None:
        draw_circle(result.diameter_circle, (44, 160, 44, 180), 2)

    draw.rectangle(
        [(margin, margin), (width - margin, height - margin)],
        outline=(80, 80, 80, 255),
        width=2,
    )
    diameter_text = (
        "undefined" if result.diameter is None else f"{result.diameter.length:.3f} m"
    )
    circle_text = (
        "undefined"
        if result.minimum_enclosing_circle is None
        else f"{result.minimum_enclosing_circle.radius:.3f} m"
    )
    draw.text(
        (margin, 35),
        f"Bearing-sector intersection | status={result.status} | D={diameter_text} | MEC r={circle_text}",
        fill=(20, 20, 20, 255),
    )
    draw.text(
        (margin, height - 55),
        "red: positioning region/diameter   blue: minimum enclosing circle   green: diameter circle",
        fill=(40, 40, 40, 255),
    )
    image.save(output_path)


def load_input(path: Path) -> tuple[list[BearingObservation], float]:
    data = json.loads(path.read_text(encoding="utf-8"))
    angle_error_deg = float(data.get("angle_error_deg", 1.0))
    observations = [
        BearingObservation(
            x=float(item["x"]),
            y=float(item["y"]),
            bearing_deg=float(item["bearing_deg"]),
        )
        for item in data["observations"]
    ]
    return observations, angle_error_deg


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="JSON file containing observations")
    parser.add_argument("--output", type=Path, help="optional JSON result path")
    parser.add_argument("--plot", type=Path, help="optional PNG plot path")
    arguments = parser.parse_args()

    observations, angle_error_deg = load_input(arguments.input)
    result = analyze_observations(observations, angle_error_deg)
    serialized = json.dumps(result_to_dict(result), ensure_ascii=False, indent=2)
    if arguments.output:
        arguments.output.write_text(serialized + "\n", encoding="utf-8")
    else:
        print(serialized)
    if arguments.plot:
        plot_result(observations, result, angle_error_deg, arguments.plot)


if __name__ == "__main__":
    main()
