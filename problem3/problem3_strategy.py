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
    near_optimal_tolerance: float = 0.03
    maximum_localization_measurements: int = 8
    opportunistic_min_crossing_angle_deg: float = 15.0
    route_candidate_tracks: int = 3


@dataclass
class SourceTrack:
    channel: int
    observations: list[BearingObservation] = field(default_factory=list)
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
        if self.config.route_candidate_tracks < 1:
            raise ValueError("route_candidate_tracks must be positive")

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
            self._log(
                "coverage_point_start",
                scan_index=scan_index,
                point=self._point_dict(point),
                unseen_channels=sorted(unseen),
            )
            self._opportunistic_measurements(point)
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

    def _opportunistic_measurements(self, point: Point) -> None:
        for track in list(self.tracks.values()):
            if track.cleared or self._already_measured_here(track, point):
                continue
            if not self._point_guarantees_reception(point, track.region):
                continue
            if self._minimum_crossing_angle(track, point) < self.config.opportunistic_min_crossing_angle_deg:
                continue
            outcome = self._measure(point, track.channel, phase="opportunistic")
            if outcome == "near":
                self._clear_at(point, track.channel, reason="near opportunistic")

    def _active_localization(self) -> None:
        while True:
            pending = [track for track in self.tracks.values() if not track.cleared]
            if not pending:
                return

            clearable = [
                track
                for track in pending
                if minimum_enclosing_circle(track.region).radius
                <= self.config.clear_decision_radius
            ]
            if clearable:
                current = Point(*self.robot.current_position)
                track = min(
                    clearable,
                    key=lambda item: self._distance(
                        current, minimum_enclosing_circle(item.region).center
                    ),
                )
                center = minimum_enclosing_circle(track.region).center
                self._clear_at(center, track.channel, reason="MEC radius certified")
                continue

            current = Point(*self.robot.current_position)
            # Evaluate a short list of nearby tracks jointly. Choosing only the
            # nearest MEC can send the robot to a point that is far from the
            # track's actual minimax candidate. The shortlist keeps CPU cost
            # bounded while allowing one-step route-aware decisions.
            shortlist = sorted(
                pending,
                key=lambda item: (
                    self._distance(
                        current, minimum_enclosing_circle(item.region).center
                    ),
                    minimum_enclosing_circle(item.region).radius,
                    item.channel,
                ),
            )[: self.config.route_candidate_tracks]
            options = []
            for candidate_track in shortlist:
                if (
                    candidate_track.localization_measurements
                    >= self.config.maximum_localization_measurements
                ):
                    continue
                candidate_point, predicted_radius = self._choose_localization_point(
                    candidate_track, log_event="localization_candidate_evaluated"
                )
                options.append(
                    (
                        self._distance(current, candidate_point),
                        predicted_radius,
                        candidate_track.channel,
                        candidate_track,
                        candidate_point,
                    )
                )
            if not options:
                raise RuntimeError("no eligible localization track")
            _, _, _, track, point = min(
                options, key=lambda item: (item[0], item[1], item[2])
            )
            self._log(
                "localization_track_selected",
                channel=track.channel,
                point=self._point_dict(point),
                travel_distance_m=self._distance(current, point),
                shortlist_channels=[item.channel for item in shortlist],
            )
            if track.localization_measurements >= self.config.maximum_localization_measurements:
                raise RuntimeError(
                    f"channel {track.channel} exceeded localization measurement limit"
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
                self._distance(current, item.point),
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
            track.region = self._rebuild_region(track.observations)
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
        elif result != "no_signal":
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
        self, observations: Sequence[BearingObservation]
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
        return convex_hull(region)

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
