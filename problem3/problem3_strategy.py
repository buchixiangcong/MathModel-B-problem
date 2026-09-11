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

from problem1_geometry import (  # noqa: E402
    BearingObservation,
    Point,
    bearing_halfplanes,
    convex_hull,
    minimum_enclosing_circle,
    polygon_area,
)
from problem2_strategy import (  # noqa: E402
    clip_polygon_halfplane,
    circumscribed_disk_polygon,
    disk_outer_halfplanes,
    evaluate_candidate,
    generate_candidate_points,
    generate_source_scenarios,
    guaranteed_core_boundary,
)


class RobotInterface(Protocol):
    current_position: tuple[float, float]

    def enter(self) -> dict[str, Any]: ...
    def measure(self, x: float, y: float, channel: int) -> dict[str, Any]: ...
    def clear(self, x: float, y: float, channel: int) -> dict[str, Any]: ...
    def exit(self) -> dict[str, Any]: ...


@dataclass(frozen=True)
class StrategyConfig:
    target_radius: float = 1800.0
    coverage_ring_radius: float = 1140.0
    guaranteed_reception_radius: float = 1000.0
    maximum_reception_radius: float = 1500.0
    angle_error_deg: float = 1.0
    clear_decision_radius: float = 19.0
    circle_side_count: int = 360
    candidate_angle_count: int = 24
    candidate_radial_levels: int = 5
    source_edge_subdivisions: int = 6
    source_interior_levels: int = 3
    error_sample_count: int = 5
    near_optimal_tolerance: float = 0.20
    maximum_localization_measurements: int = 8
    opportunistic_min_crossing_angle_deg: float = 15.0
    probe_known_channels_during_coverage: bool = True
    coverage_probe_center_distance_m: float = 1200.0
    probe_along_coverage_segments: bool = True
    coverage_segment_sample_count: int = 9
    coverage_segment_improvement_ratio: float = 0.90
    exact_clear_route_limit: int = 16
    source_tour_planning: bool = True
    localization_clear_leg_weight: float = 1.5


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


def coverage_points(ring_radius: float = 1140.0) -> list[Point]:
    return [Point(0.0, 0.0)] + [
        Point(
            ring_radius * math.cos(index * math.pi / 3.0),
            ring_radius * math.sin(index * math.pi / 3.0),
        )
        for index in range(6)
    ]


def coverage_worst_distance(
    ring_radius: float, target_radius: float = 1800.0
) -> float:
    """Worst distance to the seven-point set; attained on the outer boundary."""

    return math.sqrt(
        target_radius**2
        + ring_radius**2
        - 2.0 * target_radius * ring_radius * math.cos(math.pi / 6.0)
    )


def serpentine_channel_order(channels: Sequence[int], scan_index: int) -> list[int]:
    return sorted(channels, reverse=bool(scan_index % 2))


class Problem3Strategy:
    def __init__(
        self,
        robot: RobotInterface,
        config: StrategyConfig | None = None,
        decision_log_path: Path | None = None,
    ):
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

        worst = coverage_worst_distance(
            self.config.coverage_ring_radius, self.config.target_radius
        )
        if worst > self.config.guaranteed_reception_radius + 1e-9:
            raise ValueError(
                f"coverage ring is invalid: worst distance {worst:.3f} m exceeds "
                f"{self.config.guaranteed_reception_radius:.3f} m"
            )
        if self.config.coverage_segment_sample_count < 3:
            raise ValueError("coverage_segment_sample_count must be at least 3")
        if not 0.0 < self.config.coverage_segment_improvement_ratio <= 1.0:
            raise ValueError(
                "coverage_segment_improvement_ratio must be in (0, 1]"
            )
        if self.config.exact_clear_route_limit < 1:
            raise ValueError("exact_clear_route_limit must be positive")
        if self.config.localization_clear_leg_weight < 0.0:
            raise ValueError("localization_clear_leg_weight must be nonnegative")

    def run(self) -> RunSummary:
        enter_response = self.robot.enter()
        self._update_time(enter_response)
        self._log("enter", response=enter_response)
        try:
            self._coverage_scan()
            self._active_localization()
            uncleared = sorted(
                channel for channel, track in self.tracks.items() if not track.cleared
            )
            if uncleared:
                raise RuntimeError(f"strategy ended with uncleared channels: {uncleared}")
            exit_response = self.robot.exit()
            self._update_time(exit_response)
            self._log("exit", response=exit_response)
        except Exception as exc:
            self._log("abort", error=f"{type(exc).__name__}: {exc}")
            try:
                exit_response = self.robot.exit()
                self._update_time(exit_response)
                self._log("exit_after_abort", response=exit_response)
            except Exception as exit_exc:
                self._log(
                    "exit_after_abort_failed",
                    error=f"{type(exit_exc).__name__}: {exit_exc}",
                )
            raise

        cleared_channels = sorted(
            channel for channel, track in self.tracks.items() if track.cleared
        )
        return RunSummary(
            discovered_channels=sorted(self.tracks),
            cleared_channels=cleared_channels,
            absent_channels=sorted(self.absent_channels),
            virtual_time_s=self.last_virtual_time_s,
            average_time_per_cleared_source_s=(
                self.last_virtual_time_s / len(cleared_channels)
                if cleared_channels
                else math.inf
            ),
            measure_count=self.measure_count,
            clear_attempt_count=self.clear_attempt_count,
            decision_log_path=(
                None if self.decision_log_path is None else str(self.decision_log_path)
            ),
        )

    def _coverage_scan(self) -> None:
        unseen = set(range(1, 21))
        points = coverage_points(self.config.coverage_ring_radius)
        for scan_index, point in enumerate(points):
            if scan_index > 0:
                self._coverage_segment_measurements(points[scan_index - 1], point)
            self._log(
                "coverage_point_start",
                scan_index=scan_index,
                point=self._point_dict(point),
                unseen_channels=sorted(unseen),
            )
            self._coverage_opportunistic_measurements(point)
            for channel in serpentine_channel_order(list(unseen), scan_index):
                outcome = self._measure(point, channel, phase="coverage")
                if outcome == "no_signal":
                    continue
                unseen.remove(channel)
                if outcome == "near":
                    self._clear_at(point, channel, reason="near during coverage")
                if len(self.tracks) == 16:
                    # The statement gives 16 as a hard upper bound. Once 16
                    # distinct channels are found, every other channel is absent.
                    self.absent_channels = unseen
                    self._log(
                        "coverage_stopped_at_upper_bound",
                        absent_channels=sorted(unseen),
                    )
                    return
            self._log(
                "coverage_point_end",
                scan_index=scan_index,
                remaining_unseen=sorted(unseen),
            )
            if scan_index == 0:
                points[1:] = self._choose_coverage_ring_route(points[1:])
        self.absent_channels = unseen
        self._log("coverage_complete", absent_channels=sorted(unseen))

    def _choose_coverage_ring_route(self, ring: Sequence[Point]) -> list[Point]:
        """Choose among equal-length hexagon paths using a source-tour proxy."""

        if not self.tracks:
            return list(ring)
        targets = [
            minimum_enclosing_circle(track.region).center
            for track in self.tracks.values()
            if track.region
        ]
        options: list[tuple[float, int, int, list[Point]]] = []
        for start in range(len(ring)):
            for direction in (1, -1):
                route = [
                    ring[(start + direction * offset) % len(ring)]
                    for offset in range(len(ring))
                ]
                score = self._greedy_open_route_length(route[-1], targets)
                options.append((score, start, direction, route))
        score, start, direction, route = min(
            options, key=lambda item: (item[0], item[1], -item[2])
        )
        self._log(
            "coverage_route_selected",
            start_index=start,
            direction=direction,
            predicted_followup_route_m=score,
            route=[self._point_dict(point) for point in route],
        )
        return route

    def _greedy_open_route_length(
        self, start: Point, targets: Sequence[Point]
    ) -> float:
        current = start
        remaining = list(targets)
        total = 0.0
        while remaining:
            next_index = min(
                range(len(remaining)),
                key=lambda index: self._distance(current, remaining[index]),
            )
            target = remaining.pop(next_index)
            total += self._distance(current, target)
            current = target
        return total

    def _optimal_open_route_order(
        self, start: Point, targets: Sequence[Point]
    ) -> list[int]:
        """Exact shortest open route from start through every target."""

        count = len(targets)
        if count <= 1:
            return list(range(count))
        if count > self.config.exact_clear_route_limit:
            remaining = list(range(count))
            order: list[int] = []
            current = start
            while remaining:
                index = min(
                    remaining,
                    key=lambda item: (self._distance(current, targets[item]), item),
                )
                remaining.remove(index)
                order.append(index)
                current = targets[index]
            return order

        full_mask = (1 << count) - 1
        costs: dict[tuple[int, int], float] = {}
        parents: dict[tuple[int, int], int | None] = {}
        for index, target in enumerate(targets):
            state = (1 << index, index)
            costs[state] = self._distance(start, target)
            parents[state] = None

        for mask in range(1, full_mask + 1):
            for last in range(count):
                state = (mask, last)
                current_cost = costs.get(state)
                if current_cost is None:
                    continue
                remaining = full_mask ^ mask
                while remaining:
                    bit = remaining & -remaining
                    next_index = bit.bit_length() - 1
                    next_state = (mask | bit, next_index)
                    candidate_cost = current_cost + self._distance(
                        targets[last], targets[next_index]
                    )
                    previous_cost = costs.get(next_state)
                    if previous_cost is None or candidate_cost < previous_cost - 1e-9:
                        costs[next_state] = candidate_cost
                        parents[next_state] = last
                    remaining ^= bit

        last = min(
            range(count),
            key=lambda index: (costs[(full_mask, index)], index),
        )
        order: list[int] = []
        mask = full_mask
        while True:
            order.append(last)
            previous = parents[(mask, last)]
            if previous is None:
                break
            mask ^= 1 << last
            last = previous
        order.reverse()
        return order

    def _coverage_segment_measurements(self, start: Point, end: Point) -> None:
        """Use guaranteed-reception points on a mandatory coverage segment."""

        if not self.config.probe_along_coverage_segments:
            return
        scheduled: list[tuple[float, int, Point, float]] = []
        for track in self.tracks.values():
            if track.cleared or not track.region:
                continue
            current_radius = minimum_enclosing_circle(track.region).radius
            if current_radius <= self.config.clear_decision_radius:
                continue
            interval = self._segment_guaranteed_interval(start, end, track.region)
            if interval is None:
                continue
            lower, upper = interval
            candidates: list[tuple[float, Point]] = []
            for sample_index in range(self.config.coverage_segment_sample_count):
                fraction = lower + (upper - lower) * sample_index / (
                    self.config.coverage_segment_sample_count - 1
                )
                point = Point(
                    start.x + fraction * (end.x - start.x),
                    start.y + fraction * (end.y - start.y),
                )
                if self._already_measured_here(track, point, tolerance=1.0):
                    continue
                if (
                    self._minimum_crossing_angle(track, point)
                    < self.config.opportunistic_min_crossing_angle_deg
                ):
                    continue
                candidates.append((fraction, point))
            if not candidates:
                continue

            scenarios = generate_source_scenarios(
                track.region,
                self.config.source_edge_subdivisions,
                self.config.source_interior_levels,
            )
            first_station = track.observations[0].station
            evaluations = [
                (
                    fraction,
                    evaluate_candidate(
                        point,
                        first_station,
                        track.region,
                        scenarios,
                        self.config.angle_error_deg,
                        self.config.error_sample_count,
                        optical_radius=5.0,
                    ),
                )
                for fraction, point in candidates
            ]
            fraction, chosen = min(
                evaluations,
                key=lambda item: (item[1].worst_mec_radius, item[0]),
            )
            if (
                chosen.worst_mec_radius
                > self.config.clear_decision_radius
                and chosen.worst_mec_radius
                > current_radius * self.config.coverage_segment_improvement_ratio
            ):
                continue
            scheduled.append(
                (fraction, track.channel, chosen.point, chosen.worst_mec_radius)
            )

        for fraction, channel, point, predicted_radius in sorted(scheduled):
            track = self.tracks[channel]
            if track.cleared or self._already_measured_here(track, point, tolerance=1.0):
                continue
            if not self._point_guarantees_reception(point, track.region):
                continue
            self._log(
                "coverage_segment_probe_selected",
                channel=channel,
                segment_fraction=fraction,
                point=self._point_dict(point),
                predicted_worst_mec_radius=predicted_radius,
            )
            outcome = self._measure(point, channel, phase="coverage_segment_probe")
            if outcome == "near":
                self._clear_at(point, channel, reason="near coverage segment probe")
            elif outcome == "no_signal":
                raise RuntimeError(
                    f"guaranteed segment point returned no_signal for channel {channel}"
                )

    def _segment_guaranteed_interval(
        self, start: Point, end: Point, region: Sequence[Point]
    ) -> tuple[float, float] | None:
        """Return segment parameters whose points are within 1000 m of P."""

        direction_x = end.x - start.x
        direction_y = end.y - start.y
        quadratic = direction_x * direction_x + direction_y * direction_y
        if quadratic <= 1e-20:
            return (
                (0.0, 0.0)
                if self._point_guarantees_reception(start, region)
                else None
            )
        radius = self.config.guaranteed_reception_radius - 1e-5
        lower = 0.0
        upper = 1.0
        for vertex in region:
            relative_x = start.x - vertex.x
            relative_y = start.y - vertex.y
            linear = 2.0 * (
                relative_x * direction_x + relative_y * direction_y
            )
            constant = relative_x * relative_x + relative_y * relative_y - radius * radius
            discriminant = linear * linear - 4.0 * quadratic * constant
            if discriminant < 0.0:
                return None
            root = math.sqrt(discriminant)
            vertex_lower = (-linear - root) / (2.0 * quadratic)
            vertex_upper = (-linear + root) / (2.0 * quadratic)
            lower = max(lower, vertex_lower)
            upper = min(upper, vertex_upper)
            if lower > upper + 1e-12:
                return None
        return max(0.0, lower), min(1.0, upper)

    def _opportunistic_measurements(self, point: Point) -> None:
        for track in list(self.tracks.values()):
            if track.cleared or self._already_measured_here(track, point):
                continue
            if not self._point_guarantees_reception(point, track.region):
                continue
            if (
                self._minimum_crossing_angle(track, point)
                < self.config.opportunistic_min_crossing_angle_deg
            ):
                continue
            outcome = self._measure(point, track.channel, phase="opportunistic")
            if outcome == "near":
                self._clear_at(point, track.channel, reason="near opportunistic")

    def _coverage_opportunistic_measurements(self, point: Point) -> None:
        if not self.config.probe_known_channels_during_coverage:
            self._opportunistic_measurements(point)
            return
        for track in list(self.tracks.values()):
            if track.cleared or self._already_measured_here(track, point):
                continue
            center = minimum_enclosing_circle(track.region).center
            if (
                self._distance(point, center)
                > self.config.coverage_probe_center_distance_m
            ):
                continue
            if (
                self._minimum_crossing_angle(track, point)
                < self.config.opportunistic_min_crossing_angle_deg
            ):
                continue
            outcome = self._measure(point, track.channel, phase="coverage_probe")
            if outcome == "near":
                self._clear_at(point, track.channel, reason="near coverage probe")

    def _active_localization(self) -> None:
        while True:
            pending = [track for track in self.tracks.values() if not track.cleared]
            if not pending:
                return
            current = Point(*self.robot.current_position)
            centers = [minimum_enclosing_circle(track.region).center for track in pending]
            if self.config.source_tour_planning and len(pending) > 1:
                source_order = self._optimal_open_route_order(current, centers)
                track_index = source_order[0]
            else:
                track_index = min(
                    range(len(pending)),
                    key=lambda index: (
                        self._distance(current, centers[index]),
                        pending[index].channel,
                    ),
                )
                source_order = [track_index]
            track = pending[track_index]
            enclosing = minimum_enclosing_circle(track.region)
            self._log(
                "source_tour_selected",
                channels=[pending[index].channel for index in source_order],
                first_channel=track.channel,
            )
            if enclosing.radius <= self.config.clear_decision_radius:
                self._clear_at(
                    enclosing.center, track.channel, reason="MEC radius certified"
                )
                continue
            if (
                track.localization_measurements
                >= self.config.maximum_localization_measurements
            ):
                raise RuntimeError(
                    f"channel {track.channel} exceeded localization measurement limit"
                )
            point, predicted_radius = self._choose_localization_point(
                track, log_event="localization_candidate_evaluated"
            )
            self._log(
                "localization_track_selected",
                channel=track.channel,
                point=self._point_dict(point),
                travel_distance_m=self._distance(current, point),
                predicted_worst_mec_radius=predicted_radius,
                source_tour_channels=[pending[index].channel for index in source_order],
            )
            outcome = self._measure(point, track.channel, phase="active_localization")
            track.localization_measurements += 1
            if outcome == "near":
                self._clear_at(point, track.channel, reason="near active localization")
            elif outcome == "no_signal":
                raise RuntimeError(
                    f"guaranteed point returned no_signal for channel {track.channel}"
                )
            # Reuse the station for any other channel whose entire feasible
            # region is inside the guaranteed reception disk. These extra
            # readings cost seconds, not another long robot trip.
            self._opportunistic_measurements(point)
            if not track.cleared:
                enclosing = minimum_enclosing_circle(track.region)
                if enclosing.radius <= self.config.clear_decision_radius:
                    self._clear_at(
                        enclosing.center,
                        track.channel,
                        reason="MEC radius certified",
                    )

    def _choose_localization_point(
        self,
        track: SourceTrack,
        log_event: str = "localization_point_selected",
    ) -> tuple[Point, float]:
        enclosing, boundary = guaranteed_core_boundary(
            track.region,
            self.config.guaranteed_reception_radius,
            self.config.candidate_angle_count,
        )
        if not boundary:
            raise RuntimeError(
                f"guaranteed reception core is empty for channel {track.channel}; "
                f"region MEC radius={enclosing.radius:.3f}"
            )
        candidates = generate_candidate_points(
            enclosing.center,
            boundary,
            self.config.candidate_radial_levels,
        )
        candidates = [
            point
            for point in candidates
            if not self._already_measured_here(track, point, tolerance=1.0)
        ]
        if not candidates:
            raise RuntimeError(f"no new candidate point for channel {track.channel}")
        scenarios = generate_source_scenarios(
            track.region,
            self.config.source_edge_subdivisions,
            self.config.source_interior_levels,
        )
        first_station = track.observations[0].station
        evaluations = [
            evaluate_candidate(
                point,
                first_station,
                track.region,
                scenarios,
                self.config.angle_error_deg,
                self.config.error_sample_count,
                optical_radius=5.0,
            )
            for point in candidates
        ]
        best_radius = min(item.worst_mec_radius for item in evaluations)
        threshold = best_radius * (1.0 + self.config.near_optimal_tolerance)
        current = Point(*self.robot.current_position)
        acceptable = [
            item for item in evaluations if item.worst_mec_radius <= threshold + 1e-8
        ]
        chosen = min(
            acceptable,
            key=lambda item: (
                self._distance(current, item.point)
                + self.config.localization_clear_leg_weight
                * self._distance(item.point, enclosing.center),
                item.worst_mec_radius,
            ),
        )
        self._log(
            log_event,
            channel=track.channel,
            point=self._point_dict(chosen.point),
            predicted_worst_mec_radius=chosen.worst_mec_radius,
            best_sampled_worst_mec_radius=best_radius,
            current_region_mec_radius=enclosing.radius,
            candidate_count=len(evaluations),
        )
        return chosen.point, chosen.worst_mec_radius

    def _measure(self, point: Point, channel: int, phase: str) -> str:
        response = self.robot.measure(point.x, point.y, channel)
        self.measure_count += 1
        self._update_time(response)
        result = response["measure_result"]
        self._log(
            "measurement",
            phase=phase,
            channel=channel,
            point=self._point_dict(point),
            result=result,
            svd_deg=response.get("svd_deg"),
            virtual_time_s=response.get("virtual_time_s"),
        )
        if result == "direction":
            observation = BearingObservation(
                point.x, point.y, float(response["svd_deg"])
            )
            track = self.tracks.setdefault(channel, SourceTrack(channel=channel))
            track.observations.append(observation)
            track.measured_positions.append(point)
            track.region = self._rebuild_region(
                track.observations, track.no_signal_positions
            )
            enclosing = minimum_enclosing_circle(track.region)
            self._log(
                "region_updated",
                channel=channel,
                vertex_count=len(track.region),
                area=polygon_area(track.region),
                mec_center=self._point_dict(enclosing.center),
                mec_radius=enclosing.radius,
            )
        elif result == "near":
            track = self.tracks.setdefault(channel, SourceTrack(channel=channel))
            track.measured_positions.append(point)
        elif result == "no_signal":
            track = self.tracks.get(channel)
            if track is not None and track.region:
                track.no_signal_positions.append(point)
                track.measured_positions.append(point)
                track.region = self._convex_outer_after_no_signal(
                    track.region,
                    point,
                    self.config.guaranteed_reception_radius,
                )
                if not track.region:
                    raise RuntimeError(
                        f"no-signal observation emptied channel {channel} region"
                    )
                enclosing = minimum_enclosing_circle(track.region)
                self._log(
                    "region_updated",
                    channel=channel,
                    cause="no_signal_exclusion",
                    vertex_count=len(track.region),
                    area=polygon_area(track.region),
                    mec_center=self._point_dict(enclosing.center),
                    mec_radius=enclosing.radius,
                )
        else:
            raise RuntimeError(f"unknown measure result: {result!r}")
        return result

    def _clear_at(self, point: Point, channel: int, reason: str) -> None:
        track = self.tracks.get(channel)
        certificate_radius = (
            minimum_enclosing_circle(track.region).radius
            if track is not None and track.region
            else None
        )
        if reason == "MEC radius certified" and (
            certificate_radius is None
            or certificate_radius > self.config.clear_decision_radius + 1e-8
        ):
            raise RuntimeError(
                f"invalid clear certificate for channel {channel}: "
                f"MEC radius={certificate_radius}"
            )
        response = self.robot.clear(point.x, point.y, channel)
        self.clear_attempt_count += 1
        self._update_time(response)
        self._log(
            "clear",
            channel=channel,
            point=self._point_dict(point),
            reason=reason,
            certified_mec_radius=certificate_radius,
            clear_decision_radius=self.config.clear_decision_radius,
            result=response.get("clear_result"),
            virtual_time_s=response.get("virtual_time_s"),
        )
        if response.get("clear_result") != "success":
            raise RuntimeError(
                f"certified clear failed for channel {channel} at {point}: {response}"
            )
        track = self.tracks.setdefault(channel, SourceTrack(channel=channel))
        track.cleared = True
        track.clear_position = point

    def _rebuild_region(
        self,
        observations: Sequence[BearingObservation],
        no_signal_positions: Sequence[Point] = (),
    ) -> list[Point]:
        region = circumscribed_disk_polygon(
            Point(0.0, 0.0),
            self.config.target_radius,
            self.config.circle_side_count,
        )
        for index, observation in enumerate(observations):
            constraints = list(
                disk_outer_halfplanes(
                    observation.station,
                    self.config.maximum_reception_radius,
                    self.config.circle_side_count,
                    "positive-reception",
                )
            )
            constraints.extend(
                bearing_halfplanes(
                    observation, index, self.config.angle_error_deg
                )
            )
            for halfplane in constraints:
                region = clip_polygon_halfplane(region, halfplane)
                if not region:
                    raise RuntimeError(
                        "bearing observations produced an empty feasible region"
                    )
        region = convex_hull(region)
        for point in no_signal_positions:
            region = self._convex_outer_after_no_signal(
                region, point, self.config.guaranteed_reception_radius
            )
        return region

    def _convex_outer_after_no_signal(
        self, region: Sequence[Point], station: Point, radius: float
    ) -> list[Point]:
        """Convex hull of a convex polygon after removing an open disk."""

        candidates: list[Point] = []
        radius_sq = radius * radius
        tolerance = 1e-8 * max(1.0, radius_sq)
        for index, first in enumerate(region):
            second = region[(index + 1) % len(region)]
            first_dx = first.x - station.x
            first_dy = first.y - station.y
            if first_dx * first_dx + first_dy * first_dy >= radius_sq - tolerance:
                candidates.append(first)

            edge_x = second.x - first.x
            edge_y = second.y - first.y
            a = edge_x * edge_x + edge_y * edge_y
            if a <= 1e-20:
                continue
            b = 2.0 * (first_dx * edge_x + first_dy * edge_y)
            c = first_dx * first_dx + first_dy * first_dy - radius_sq
            discriminant = b * b - 4.0 * a * c
            if discriminant < -tolerance:
                continue
            root = math.sqrt(max(0.0, discriminant))
            for fraction in ((-b - root) / (2.0 * a), (-b + root) / (2.0 * a)):
                if -1e-10 <= fraction <= 1.0 + 1e-10:
                    fraction = min(1.0, max(0.0, fraction))
                    candidates.append(
                        Point(
                            first.x + fraction * edge_x,
                            first.y + fraction * edge_y,
                        )
                    )
        return convex_hull(candidates) if candidates else []

    def _point_guarantees_reception(
        self, point: Point, region: Sequence[Point]
    ) -> bool:
        return max(self._distance(point, vertex) for vertex in region) <= (
            self.config.guaranteed_reception_radius - 1e-6
        )

    def _minimum_crossing_angle(self, track: SourceTrack, point: Point) -> float:
        if not track.observations:
            return 90.0
        center = minimum_enclosing_circle(track.region).center
        candidate_angle = math.atan2(center.y - point.y, center.x - point.x)
        best = 180.0
        for observation in track.observations:
            previous_angle = math.atan2(
                center.y - observation.y, center.x - observation.x
            )
            difference = abs(
                math.degrees(
                    math.atan2(
                        math.sin(candidate_angle - previous_angle),
                        math.cos(candidate_angle - previous_angle),
                    )
                )
            )
            acute = min(difference, 180.0 - difference)
            best = min(best, acute)
        return best

    @staticmethod
    def _already_measured_here(
        track: SourceTrack, point: Point, tolerance: float = 1e-6
    ) -> bool:
        return any(
            math.hypot(point.x - previous.x, point.y - previous.y) <= tolerance
            for previous in track.measured_positions
        )

    @staticmethod
    def _distance(first: Point, second: Point) -> float:
        return math.hypot(first.x - second.x, first.y - second.y)

    @staticmethod
    def _point_dict(point: Point) -> dict[str, float]:
        return {"x": point.x, "y": point.y}

    def _update_time(self, response: dict[str, Any]) -> None:
        if response.get("accepted") is True and "virtual_time_s" in response:
            self.last_virtual_time_s = float(response["virtual_time_s"])

    def _log(self, event: str, **data: Any) -> None:
        if self.decision_log_path is None:
            return
        with self.decision_log_path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps({"event": event, **data}, ensure_ascii=False) + "\n")


def summary_to_dict(summary: RunSummary) -> dict[str, Any]:
    return {
        "discovered_channels": summary.discovered_channels,
        "cleared_channels": summary.cleared_channels,
        "absent_channels": summary.absent_channels,
        "virtual_time_s": summary.virtual_time_s,
        "average_time_per_cleared_source_s": summary.average_time_per_cleared_source_s,
        "measure_count": summary.measure_count,
        "clear_attempt_count": summary.clear_attempt_count,
        "decision_log_path": summary.decision_log_path,
    }
