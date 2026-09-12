from __future__ import annotations

import json
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Protocol, Sequence

ROOT = Path(__file__).resolve().parents[1]
PROBLEM1 = ROOT / "problem1" / "problem1"
PROBLEM2 = ROOT / "problem2" / "problem2"
for module_dir in (PROBLEM1, PROBLEM2):
    if str(module_dir) not in sys.path:
        sys.path.insert(0, str(module_dir))

from problem1_geometry import BearingObservation, Point, bearing_halfplanes, convex_hull, minimum_enclosing_circle, polygon_area
from problem2_strategy import circumscribed_disk_polygon, disk_outer_halfplanes, clip_polygon_halfplane, generate_source_scenarios, evaluate_candidate, guaranteed_core_boundary, generate_candidate_points


class RobotInterface(Protocol):
    current_position: tuple[float, float]
    def enter(self) -> dict[str, Any]: ...
    def measure(self, x: float, y: float, channel: int) -> dict[str, Any]: ...
    def clear(self, x: float, y: float, channel: int) -> dict[str, Any]: ...
    def exit(self) -> dict[str, Any]: ...


@dataclass(frozen=True)
class StrategyConfig:
    target_radius: float = 1800.0
    lattice_spacing: float = 1000.0
    lattice_margin: float = 1000.0
    strict_discovery_certificate: bool = True
    guaranteed_reception_radius: float = 1000.0
    maximum_reception_radius: float = 1500.0
    angle_error_deg: float = 1.0
    clear_decision_radius: float = 19.0
    circle_side_count: int = 180
    candidate_angle_count: int = 16
    candidate_radial_levels: int = 3
    source_edge_subdivisions: int = 4
    source_interior_levels: int = 2
    error_sample_count: int = 3
    maximum_localization_measurements: int = 80
    max_grid_localization_rounds: int = 6
    bootstrap_observation_target: int = 4
    near_optimal_tolerance: float = 0.25


@dataclass
class SourceTrack:
    channel: int
    observations: list[BearingObservation] = field(default_factory=list)
    no_signal_positions: list[Point] = field(default_factory=list)
    measured_positions: list[Point] = field(default_factory=list)
    region: list[Point] = field(default_factory=list)
    cleared: bool = False
    clear_position: Point | None = None
    localization_measurements: int = 0


@dataclass
class RunSummary:
    discovered_channels: list[int]
    cleared_channels: list[int]
    absent_channels: list[int]
    virtual_time_s: float
    average_time_per_cleared_source_s: float
    measure_count: int
    clear_attempt_count: int
    decision_log_path: str | None


def triangular_lattice_points(spacing: float = 1000.0, extent: float = 2800.0) -> list[Point]:
    """Finite triangular lattice extended one reception radius past the disk."""
    if spacing <= 0.0:
        raise ValueError("spacing must be positive")
    height = spacing * math.sqrt(3.0) / 2.0
    limit = int(math.ceil(extent / height)) + 1
    points: list[Point] = []
    for row in range(-limit, limit + 1):
        y = row * height
        offset = 0.5 * spacing if row & 1 else 0.0
        for column in range(-limit, limit + 1):
            x = column * spacing + offset
            if math.hypot(x, y) <= extent + 1e-9:
                points.append(Point(x, y))
    return points


def point_in_convex_polygon(point: Point, polygon: Sequence[Point]) -> bool:
    if len(polygon) < 3:
        return len(polygon) == 1 and point == polygon[0]
    signs = []
    for first, second in zip(polygon, polygon[1:] + polygon[:1]):
        signs.append((second.x - first.x) * (point.y - first.y) - (second.y - first.y) * (point.x - first.x))
    return min(signs) >= -1e-7 or max(signs) <= 1e-7


def nearest_station_convex_hull_contains(source: Point, stations: Sequence[Point], radius: float = 1000.0) -> bool:
    nearby = [station for station in stations if math.hypot(station.x - source.x, station.y - source.y) <= radius + 1e-7]
    return point_in_convex_polygon(source, convex_hull(nearby))


class Problem4Strategy:
    def __init__(self, robot: RobotInterface, config: StrategyConfig | None = None, decision_log_path: Path | None = None):
        self.robot = robot
        self.config = config or StrategyConfig()
        self.tracks: dict[int, SourceTrack] = {}
        self.absent_channels: set[int] = set()
        self.measure_count = 0
        self.clear_attempt_count = 0
        self.last_virtual_time_s = 0.0
        self.decision_log_path = decision_log_path
        if decision_log_path is not None:
            decision_log_path.parent.mkdir(parents=True, exist_ok=True)
        if self.config.lattice_spacing > self.config.guaranteed_reception_radius:
            raise ValueError("lattice spacing must not exceed guaranteed reception radius")
        if (
            self.config.strict_discovery_certificate
            and self.config.lattice_margin < self.config.lattice_spacing
        ):
            raise ValueError("lattice margin must be at least one lattice spacing")
        self.stations = triangular_lattice_points(self.config.lattice_spacing, self.config.target_radius + self.config.lattice_margin)

    def run(self) -> RunSummary:
        self._update_time(self.robot.enter())
        try:
            self._discovery_scan()
            self._localize_and_clear()
            self.absent_channels = set(range(1, 21)) - set(self.tracks)
            self._update_time(self.robot.exit())
        except Exception:
            try:
                self.robot.exit()
            except Exception:
                pass
            raise
        cleared = sorted(channel for channel, track in self.tracks.items() if track.cleared)
        return RunSummary(sorted(self.tracks), cleared, sorted(self.absent_channels), self.last_virtual_time_s,
                          self.last_virtual_time_s / len(cleared) if cleared else math.inf,
                          self.measure_count, self.clear_attempt_count,
                          None if self.decision_log_path is None else str(self.decision_log_path))

    def _discovery_scan(self) -> None:
        unseen = set(range(1, 21))
        stations = self._route_stations(self.stations)
        self._log("discovery_plan", station_count=len(stations), lattice_spacing=self.config.lattice_spacing,
                  coverage_certificate="triangular-cell convex-hull proof")
        for index, station in enumerate(stations):
            # Keep a discovered channel in the scan only until it has enough
            # positive bearings for the normal localization phase. Repeating
            # it after that point is redundant; channels that do not collect
            # enough bearings fall back to the guaranteed-safe bootstrap ring.
            channels = sorted(
                unseen
                | {
                    channel
                    for channel, track in self.tracks.items()
                    if not track.cleared
                    and len(track.observations) < self.config.bootstrap_observation_target
                }
            )
            for channel in channels:
                outcome = self._measure(station, channel, "discovery")
                if outcome != "no_signal":
                    unseen.discard(channel)
                if outcome == "near":
                    self._clear_at(station, channel, "near discovery")
            if not unseen:
                break
        self.absent_channels = unseen

    def _route_stations(self, stations: Sequence[Point]) -> list[Point]:
        remaining = list(stations)
        current = Point(*self.robot.current_position)
        route: list[Point] = []
        while remaining:
            next_index = min(range(len(remaining)), key=lambda i: (self._distance(current, remaining[i]), i))
            current = remaining.pop(next_index)
            route.append(current)
        # Open-path 2-opt removes crossings and keeps the first station at the
        # origin.  For the regular lattice this reaches the 12-edge path.
        improved = True
        while improved:
            improved = False
            for left in range(1, len(route) - 1):
                for right in range(left + 1, len(route)):
                    previous = route[left - 1]
                    before = self._distance(previous, route[left])
                    after = self._distance(previous, route[right])
                    if right + 1 < len(route):
                        before += self._distance(route[right], route[right + 1])
                        after += self._distance(route[left], route[right + 1])
                    if after < before - 1e-9:
                        route[left : right + 1] = reversed(route[left : right + 1])
                        improved = True
        return route

    def _localize_and_clear(self) -> None:
        while True:
            pending = [track for track in self.tracks.values() if not track.cleared]
            if not pending:
                return
            current = Point(*self.robot.current_position)
            track = min(
                pending,
                key=lambda item: (
                    self._distance(current, minimum_enclosing_circle(item.region).center),
                    item.channel,
                ),
            )
            self._localize_one(track)

    def _localize_one(self, track: SourceTrack) -> None:
        attempts_before = track.localization_measurements
        while not track.cleared:
            if not track.region:
                raise RuntimeError(f"empty region for channel {track.channel}")
            enclosing = minimum_enclosing_circle(track.region)
            if enclosing.radius <= self.config.clear_decision_radius:
                self._clear_at(enclosing.center, track.channel, "MEC radius certified")
                return
            if track.localization_measurements >= self.config.maximum_localization_measurements:
                raise RuntimeError(
                    f"channel {track.channel} exceeded localization measurement limit"
                )
            if len(track.observations) < self.config.bootstrap_observation_target:
                self._bootstrap_positive_neighbors(track)
                continue
            if enclosing.radius < 480.0:
                count_before = track.localization_measurements
                self._certified_distance_ring_probe(track)
                if track.cleared:
                    return
                if track.localization_measurements > count_before:
                    continue
            point = self._choose_localization_point(track)
            outcome = self._measure(point, track.channel, "directed_localization")
            track.localization_measurements += 1
            if outcome == "near":
                self._clear_at(point, track.channel, "near localization")
                return
            if track.localization_measurements == attempts_before:
                raise RuntimeError(f"no localization progress for channel {track.channel}")

    def _certified_distance_ring_probe(self, track: SourceTrack) -> None:
        """Probe a ring that contains the source and is wholly within 1000 m."""
        enclosing = minimum_enclosing_circle(track.region)
        if enclosing.radius >= 480.0:
            return
        ring_radius = min(
            1000.0 - enclosing.radius - 1.0,
            max(100.0, 1.25 * enclosing.radius),
        )
        candidates = [
            Point(
                enclosing.center.x + ring_radius * math.cos(2.0 * math.pi * index / 8.0),
                enclosing.center.y + ring_radius * math.sin(2.0 * math.pi * index / 8.0),
            )
            for index in range(8)
        ]
        candidates = self._short_ring_route(candidates)
        for point in candidates:
            if self._already_measured_here(track, point, tolerance=1.0):
                continue
            outcome = self._measure(point, track.channel, "certified_distance_ring")
            track.localization_measurements += 1
            if outcome == "near":
                self._clear_at(point, track.channel, "near certified ring")
                return
            enclosing = minimum_enclosing_circle(track.region)
            if enclosing.radius <= self.config.clear_decision_radius:
                return

    def _short_ring_route(self, candidates: Sequence[Point]) -> list[Point]:
        current = Point(*self.robot.current_position)
        count = len(candidates)
        options: list[tuple[float, list[Point]]] = []
        for start in range(count):
            for direction in (1, -1):
                route = [candidates[(start + direction * offset) % count] for offset in range(count)]
                length = self._distance(current, route[0]) + sum(
                    self._distance(first, second)
                    for first, second in zip(route, route[1:])
                )
                options.append((length, route))
        return min(options, key=lambda item: item[0])[1]

    def _bootstrap_positive_neighbors(self, track: SourceTrack) -> None:
        """Create a second positive station when discovery found only one.

        Move 300 m along the measured source ray and probe a 120 m ring there.
        Even at 1000 m initial range and one-degree bearing error, every ring
        point is within 821 m of the source.  A half-plane through the source
        intersects this ring, so several probes provide new positive bearings.
        """
        if not track.observations:
            return
        anchors = list(track.observations)
        for first in anchors:
            theta = math.radians(first.bearing_deg)
            center = Point(
                first.x + 300.0 * math.cos(theta),
                first.y + 300.0 * math.sin(theta),
            )
            candidates = [
                Point(
                    center.x + 120.0 * math.cos(theta + 2.0 * math.pi * index / 12.0),
                    center.y + 120.0 * math.sin(theta + 2.0 * math.pi * index / 12.0),
                )
                for index in range(12)
            ]
            candidates = self._short_ring_route(candidates)
            for point in candidates:
                if len(track.observations) >= self.config.bootstrap_observation_target or track.localization_measurements >= self.config.maximum_localization_measurements:
                    return
                if self._already_measured_here(track, point, tolerance=1.0):
                    continue
                outcome = self._measure(point, track.channel, "single_station_bootstrap")
                track.localization_measurements += 1
                if outcome == "near":
                    self._clear_at(point, track.channel, "near bootstrap")
                    return

    def _choose_localization_point(self, track: SourceTrack) -> Point:
        # Every positive station lies in both the source's (unknown) reception
        # disk and its directional half-plane.  Both sets are convex, hence any
        # convex combination of positive stations is guaranteed positive too.
        positive_stations = [observation.station for observation in track.observations]
        if len(positive_stations) >= 2:
            guaranteed_candidates: list[Point] = []
            fractions = (0.15, 0.25, 0.35, 0.50, 0.65, 0.75, 0.85)
            for first_index, first_station in enumerate(positive_stations):
                for second_station in positive_stations[first_index + 1 :]:
                    for fraction in fractions:
                        guaranteed_candidates.append(
                            Point(
                                first_station.x + fraction * (second_station.x - first_station.x),
                                first_station.y + fraction * (second_station.y - first_station.y),
                            )
                        )
            guaranteed_candidates = [
                point
                for point in guaranteed_candidates
                if not self._already_measured_here(track, point, tolerance=1.0)
            ]
            if guaranteed_candidates:
                chosen = self._score_localization_candidates(track, guaranteed_candidates)
                self._log(
                    "positive_hull_localization_selected",
                    channel=track.channel,
                    point=self._point_dict(chosen),
                    positive_station_count=len(positive_stations),
                )
                return chosen

        boundary_result = guaranteed_core_boundary(track.region, self.config.guaranteed_reception_radius, self.config.candidate_angle_count)
        _, boundary = boundary_result
        candidates: list[Point] = []
        if boundary:
            enclosing, _ = boundary_result
            candidates.extend(generate_candidate_points(enclosing.center, boundary, self.config.candidate_radial_levels))
        candidates.extend(self.stations)
        candidates = [point for point in candidates if not self._already_measured_here(track, point)]
        if not candidates:
            raise RuntimeError(f"no localization candidate for channel {track.channel}")
        return self._score_localization_candidates(track, candidates)

    def _score_localization_candidates(
        self, track: SourceTrack, candidates: Sequence[Point]
    ) -> Point:
        scenarios = generate_source_scenarios(
            track.region,
            self.config.source_edge_subdivisions,
            self.config.source_interior_levels,
        )
        first = track.observations[0].station
        scored: list[tuple[float, float, Point]] = []
        for point in candidates:
            if self._distance(point, first) < 25.0:
                continue
            try:
                evaluation = evaluate_candidate(point, first, track.region, scenarios, self.config.angle_error_deg, self.config.error_sample_count, optical_radius=5.0)
            except (RuntimeError, ValueError):
                continue
            scored.append((evaluation.worst_mec_radius, self._distance(Point(*self.robot.current_position), point), point))
        if not scored:
            return min(candidates, key=lambda point: self._distance(Point(*self.robot.current_position), point))
        best_radius = min(item[0] for item in scored)
        acceptable = [
            item
            for item in scored
            if item[0] <= best_radius * (1.0 + self.config.near_optimal_tolerance) + 1e-9
        ]
        chosen = min(acceptable, key=lambda item: (item[1], item[0]))
        self._log("localization_candidate_selected", channel=track.channel, point=self._point_dict(chosen[2]), predicted_worst_mec_radius=chosen[0])
        return chosen[2]

    def _measure(self, point: Point, channel: int, phase: str) -> str:
        response = self.robot.measure(point.x, point.y, channel)
        self.measure_count += 1
        self._update_time(response)
        result = response["measure_result"]
        self._log("measurement", phase=phase, channel=channel, point=self._point_dict(point), result=result, svd_deg=response.get("svd_deg"))
        track = self.tracks.get(channel)
        if result == "direction":
            if track is None:
                track = SourceTrack(channel)
                self.tracks[channel] = track
            track.observations.append(BearingObservation(point.x, point.y, float(response["svd_deg"])))
            track.measured_positions.append(point)
            track.region = self._rebuild_region(track.observations)
            self._log_region(track)
        elif result == "near":
            if track is None:
                track = SourceTrack(channel, region=self._initial_region())
                self.tracks[channel] = track
            track.measured_positions.append(point)
            track.region = [point]
        elif result == "no_signal" and track is not None:
            track.no_signal_positions.append(point)
            track.measured_positions.append(point)
        else:
            if result != "no_signal":
                raise RuntimeError(f"unknown measure result: {result!r}")
        return result

    def _rebuild_region(self, observations: Sequence[BearingObservation]) -> list[Point]:
        region = self._initial_region()
        for index, observation in enumerate(observations):
            constraints = list(disk_outer_halfplanes(observation.station, self.config.maximum_reception_radius, self.config.circle_side_count, "positive-reception"))
            constraints.extend(bearing_halfplanes(observation, index, self.config.angle_error_deg))
            for halfplane in constraints:
                region = clip_polygon_halfplane(region, halfplane)
                if not region:
                    raise RuntimeError("direction observations produced empty region")
        return convex_hull(region)

    def _initial_region(self) -> list[Point]:
        return circumscribed_disk_polygon(Point(0.0, 0.0), self.config.target_radius, self.config.circle_side_count)

    def _clear_at(self, point: Point, channel: int, reason: str) -> None:
        track = self.tracks[channel]
        certificate = minimum_enclosing_circle(track.region).radius if track.region else math.inf
        if reason == "MEC radius certified" and certificate > self.config.clear_decision_radius + 1e-8:
            raise RuntimeError(f"invalid clear certificate for channel {channel}: {certificate}")
        response = self.robot.clear(point.x, point.y, channel)
        self.clear_attempt_count += 1
        self._update_time(response)
        self._log("clear", channel=channel, point=self._point_dict(point), reason=reason, certified_mec_radius=certificate, result=response.get("clear_result"))
        if response.get("clear_result") != "success":
            raise RuntimeError(f"clear failed for channel {channel}: {response}")
        track.cleared = True
        track.clear_position = point

    def _log_region(self, track: SourceTrack) -> None:
        enclosing = minimum_enclosing_circle(track.region)
        self._log("region_updated", channel=track.channel, vertex_count=len(track.region), area=polygon_area(track.region), mec_center=self._point_dict(enclosing.center), mec_radius=enclosing.radius)

    def _update_time(self, response: dict[str, Any]) -> None:
        if response.get("accepted") is True and "virtual_time_s" in response:
            self.last_virtual_time_s = float(response["virtual_time_s"])

    @staticmethod
    def _distance(first: Point, second: Point) -> float:
        return math.hypot(first.x - second.x, first.y - second.y)

    @staticmethod
    def _already_measured_here(track: SourceTrack, point: Point, tolerance: float = 1e-6) -> bool:
        return any(math.hypot(point.x - old.x, point.y - old.y) <= tolerance for old in track.measured_positions)

    @staticmethod
    def _point_dict(point: Point) -> dict[str, float]:
        return {"x": point.x, "y": point.y}

    def _log(self, event: str, **data: Any) -> None:
        if self.decision_log_path is not None:
            with self.decision_log_path.open("a", encoding="utf-8") as stream:
                stream.write(json.dumps({"event": event, **data}, ensure_ascii=False) + "\n")


def summary_to_dict(summary: RunSummary) -> dict[str, Any]:
    return {"discovered_channels": summary.discovered_channels, "cleared_channels": summary.cleared_channels,
            "absent_channels": summary.absent_channels, "virtual_time_s": summary.virtual_time_s,
            "average_time_per_cleared_source_s": summary.average_time_per_cleared_source_s,
            "measure_count": summary.measure_count, "clear_attempt_count": summary.clear_attempt_count,
            "decision_log_path": summary.decision_log_path}
