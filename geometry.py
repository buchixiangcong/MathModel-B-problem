from __future__ import annotations

import itertools
import math
from dataclasses import dataclass
from statistics import median
from typing import Iterable


ARENA_RADIUS_M = 1800.0


@dataclass(frozen=True)
class Point:
    x: float
    y: float


@dataclass(frozen=True)
class Bearing:
    channel: int
    point: Point
    angle_deg: float


def deg_to_unit(angle_deg: float) -> tuple[float, float]:
    rad = math.radians(angle_deg)
    return math.cos(rad), math.sin(rad)


def distance(a: Point, b: Point) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)


def angle_diff_deg(a: float, b: float) -> float:
    diff = abs((a - b + 180.0) % 360.0 - 180.0)
    return diff


def ring_points(radius: float, count: int, *, offset_deg: float = 0.0) -> list[Point]:
    return [
        Point(
            radius * math.cos(math.radians(offset_deg + 360.0 * i / count)),
            radius * math.sin(math.radians(offset_deg + 360.0 * i / count)),
        )
        for i in range(count)
    ]


def problem3_scan_points() -> list[Point]:
    """
    Coverage-oriented probe layout for omnidirectional sources.

    Center + 8 points at 1000 m + 8 offset points at 1500 m gives most
    sources at least two useful bearings even when the receive radius is
    near its lower bound.
    """

    return [Point(0.0, 0.0)] + ring_points(1000.0, 8) + ring_points(1500.0, 8, offset_deg=22.5)


def problem3_fast_scan_points() -> list[Point]:
    """Visit the same coverage points while alternating between the two rings."""

    inner = ring_points(1000.0, 8)
    outer = ring_points(1500.0, 8, offset_deg=22.5)
    alternating = [point for pair in zip(inner, outer) for point in pair]
    return [Point(0.0, 0.0), *alternating]


def line_intersection(a: Bearing, b: Bearing) -> Point | None:
    v1 = deg_to_unit(a.angle_deg)
    v2 = deg_to_unit(b.angle_deg)
    p = a.point
    q = b.point
    denom = cross(v1, v2)
    if abs(denom) < 1e-9:
        return None

    q_minus_p = (q.x - p.x, q.y - p.y)
    t = cross(q_minus_p, v2) / denom
    u = cross(q_minus_p, v1) / denom

    # Measurements are directions from detector to source. Very negative t/u
    # means the mathematical intersection lies behind a detector.
    if t < -20.0 or u < -20.0:
        return None

    return Point(p.x + t * v1[0], p.y + t * v1[1])


def cross(a: tuple[float, float], b: tuple[float, float]) -> float:
    return a[0] * b[1] - a[1] * b[0]


def estimate_source_position(bearings: Iterable[Bearing]) -> Point | None:
    usable = list(bearings)
    if len(usable) < 2:
        return None

    intersections: list[Point] = []
    for a, b in itertools.combinations(usable, 2):
        angle_gap = angle_diff_deg(a.angle_deg, b.angle_deg)
        if angle_gap < 12.0 or angle_gap > 168.0:
            continue
        p = line_intersection(a, b)
        if p is None:
            continue
        if math.hypot(p.x, p.y) <= ARENA_RADIUS_M + 350.0:
            intersections.append(p)

    if intersections:
        return Point(median([p.x for p in intersections]), median([p.y for p in intersections]))

    return least_squares_intersection(usable)


def least_squares_intersection(bearings: list[Bearing]) -> Point | None:
    # Line equation: n dot x = n dot p, where n is perpendicular to direction.
    ata00 = ata01 = ata11 = 0.0
    atb0 = atb1 = 0.0

    for bearing in bearings:
        vx, vy = deg_to_unit(bearing.angle_deg)
        nx, ny = -vy, vx
        rhs = nx * bearing.point.x + ny * bearing.point.y
        ata00 += nx * nx
        ata01 += nx * ny
        ata11 += ny * ny
        atb0 += nx * rhs
        atb1 += ny * rhs

    det = ata00 * ata11 - ata01 * ata01
    if abs(det) < 1e-9:
        return None

    x = (atb0 * ata11 - ata01 * atb1) / det
    y = (ata00 * atb1 - atb0 * ata01) / det
    return Point(x, y)


def clear_candidates(center: Point, *, radii: tuple[float, ...] = (0.0, 12.0, 24.0, 36.0)) -> list[Point]:
    candidates = [center]
    for radius in radii:
        if radius == 0:
            continue
        for i in range(8):
            angle = 2.0 * math.pi * i / 8
            candidates.append(Point(center.x + radius * math.cos(angle), center.y + radius * math.sin(angle)))
    return candidates


def polygon_diameter(points: Iterable[Point]) -> float:
    pts = list(points)
    if len(pts) < 2:
        return 0.0
    return max(distance(a, b) for a, b in itertools.combinations(pts, 2))


def localization_polygon(
    observations: Iterable[Bearing],
    *,
    angle_error_deg: float = 1.0,
    initial_limit: float = 3000.0,
) -> list[Point]:
    """
    Half-plane clipping approximation for Problem 1.

    Each bearing creates a wedge between angle-error lower and upper rays.
    The returned vertices describe the intersection polygon.
    """

    poly = [
        Point(-initial_limit, -initial_limit),
        Point(initial_limit, -initial_limit),
        Point(initial_limit, initial_limit),
        Point(-initial_limit, initial_limit),
    ]

    for obs in observations:
        lower = obs.angle_deg - angle_error_deg
        upper = obs.angle_deg + angle_error_deg
        lower_v = deg_to_unit(lower)
        upper_v = deg_to_unit(upper)
        poly = clip_half_plane(poly, obs.point, lower_v, keep_left=True)
        poly = clip_half_plane(poly, obs.point, upper_v, keep_left=False)
        if not poly:
            return []

    return poly


def clip_half_plane(poly: list[Point], origin: Point, direction: tuple[float, float], *, keep_left: bool) -> list[Point]:
    if not poly:
        return []

    def value(p: Point) -> float:
        return cross(direction, (p.x - origin.x, p.y - origin.y))

    def inside(p: Point) -> bool:
        val = value(p)
        return val >= -1e-9 if keep_left else val <= 1e-9

    def segment_intersection(a: Point, b: Point) -> Point:
        va = value(a)
        vb = value(b)
        t = va / (va - vb)
        return Point(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y))

    output: list[Point] = []
    prev = poly[-1]
    prev_inside = inside(prev)
    for curr in poly:
        curr_inside = inside(curr)
        if curr_inside:
            if not prev_inside:
                output.append(segment_intersection(prev, curr))
            output.append(curr)
        elif prev_inside:
            output.append(segment_intersection(prev, curr))
        prev = curr
        prev_inside = curr_inside
    return output
