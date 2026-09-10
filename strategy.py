from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass, field
from typing import Iterable

from geometry import (
    Bearing,
    Point,
    clear_candidates,
    distance,
    estimate_source_position,
    problem3_fast_scan_points,
    problem3_scan_points,
)
from robot_client import RobotClient, RobotClientError


@dataclass
class ChannelState:
    channel: int
    bearings: list[Bearing] = field(default_factory=list)
    no_signal_count: int = 0
    cleared: bool = False
    clear_attempts: int = 0
    last_estimate: Point | None = None


@dataclass
class StrategyResult:
    cleared_channels: list[int]
    known_signal_channels: list[int]
    final_virtual_time_s: float | None
    run_dir: str


class Problem3Strategy:
    """
    Baseline strategy for Problem 3, where every source is omnidirectional.

    The strategy scans a fixed coverage layout, estimates source positions
    from bearing intersections, and attempts local clear probes near each
    estimate. It is intentionally conservative for the first practice runs.
    """

    def __init__(
        self,
        channels: Iterable[int] = range(1, 21),
        scan_points: Iterable[Point] | None = None,
        bearing_limit: int | None = None,
    ):
        self.channels = list(channels)
        self.scan_points = list(scan_points) if scan_points is not None else problem3_scan_points()
        self.bearing_limit = bearing_limit
        self.state: dict[int, ChannelState] = {ch: ChannelState(ch) for ch in self.channels}

    def run(self, client: RobotClient) -> StrategyResult:
        entered = False
        try:
            client.enter()
            entered = True
            self.scan_all_points(client)
            self.clear_estimated_sources(client)
            client.exit()
        except RobotClientError:
            if entered:
                self.try_exit(client)
            raise

        cleared = sorted(ch for ch, st in self.state.items() if st.cleared)
        known = sorted(ch for ch, st in self.state.items() if st.bearings or st.cleared)
        return StrategyResult(
            cleared_channels=cleared,
            known_signal_channels=known,
            final_virtual_time_s=client.last_virtual_time_s,
            run_dir=str(client.run_dir),
        )

    def scan_all_points(self, client: RobotClient) -> None:
        for point_index, point in enumerate(self.scan_points, start=1):
            print(f"[scan] point {point_index}/{len(self.scan_points)}: ({point.x:.1f}, {point.y:.1f})")
            for channel in self.channels:
                st = self.state[channel]
                if st.cleared or self.has_enough_bearings(st):
                    continue

                response = client.measure(point.x, point.y, channel)
                result = response.get("measure_result")
                if result == "direction":
                    angle = float(response["svd_deg"])
                    st.bearings.append(Bearing(channel=channel, point=point, angle_deg=angle))
                    print(f"  channel {channel:02d}: direction {angle:.2f} deg")
                    if self.has_enough_bearings(st):
                        print(f"  channel {channel:02d}: position locked, skipping later scans")
                elif result == "near":
                    print(f"  channel {channel:02d}: near, trying clear")
                    self.try_clear_at(client, channel, point)
                else:
                    st.no_signal_count += 1

    def has_enough_bearings(self, st: ChannelState) -> bool:
        return self.bearing_limit is not None and len(st.bearings) >= self.bearing_limit

    def clear_estimated_sources(self, client: RobotClient) -> None:
        pending = [
            st for st in self.state.values()
            if not st.cleared and len(st.bearings) >= 2
        ]

        for st in pending:
            st.last_estimate = estimate_source_position(st.bearings)
        pending = [st for st in pending if st.last_estimate is not None]

        current = Point(*client.current_position)
        pending = optimize_clear_order(current, pending)
        if pending:
            route_channels = ",".join(str(st.channel) for st in pending)
            print(
                f"[clear-route] channels={route_channels}, "
                f"estimated travel={clear_route_length(current, pending):.1f} m"
            )

        for next_state in pending:
            assert next_state.last_estimate is not None
            self.clear_with_refinement(client, next_state)
            current = Point(*client.current_position)

    def clear_with_refinement(self, client: RobotClient, st: ChannelState) -> None:
        for round_index in range(3):
            estimate = estimate_source_position(st.bearings)
            if estimate is None:
                return
            st.last_estimate = estimate
            print(
                f"[clear] channel {st.channel:02d}, round {round_index + 1}, "
                f"estimate=({estimate.x:.1f}, {estimate.y:.1f}), bearings={len(st.bearings)}"
            )

            for candidate in clear_candidates(estimate):
                if self.try_clear_at(client, st.channel, candidate):
                    return

            # If clear failed, take one more bearing at the estimate and retry.
            response = client.measure(estimate.x, estimate.y, st.channel)
            result = response.get("measure_result")
            if result == "near":
                self.try_clear_at(client, st.channel, estimate)
                return
            if result == "direction":
                st.bearings.append(
                    Bearing(
                        channel=st.channel,
                        point=estimate,
                        angle_deg=float(response["svd_deg"]),
                    )
                )

    def try_clear_at(self, client: RobotClient, channel: int, point: Point) -> bool:
        st = self.state[channel]
        st.clear_attempts += 1
        response = client.clear(point.x, point.y, channel)
        if response.get("clear_result") == "success":
            st.cleared = True
            print(f"  channel {channel:02d}: cleared at ({point.x:.1f}, {point.y:.1f})")
            return True
        return False

    @staticmethod
    def try_exit(client: RobotClient) -> None:
        try:
            client.exit()
        except Exception:
            pass


class Problem3FastStrategy(Problem3Strategy):
    """Lower-cost strategy validated by replaying the four baseline runs."""

    TARGET_BEARINGS = 4

    def __init__(self, channels: Iterable[int] = range(1, 21)):
        super().__init__(
            channels=channels,
            scan_points=problem3_fast_scan_points(),
            bearing_limit=self.TARGET_BEARINGS,
        )


def distance_to_estimate(current: Point, st: ChannelState) -> float:
    if st.last_estimate is None:
        return float("inf")
    dx = current.x - st.last_estimate.x
    dy = current.y - st.last_estimate.y
    return (dx * dx + dy * dy) ** 0.5


def clear_route_length(start: Point, route: Iterable[ChannelState]) -> float:
    total = 0.0
    current = start
    for st in route:
        if st.last_estimate is None:
            return float("inf")
        total += distance(current, st.last_estimate)
        current = st.last_estimate
    return total


def optimize_clear_order(start: Point, states: Iterable[ChannelState]) -> list[ChannelState]:
    """Build an open route with multi-start nearest neighbor and 2-opt."""

    available = [st for st in states if st.last_estimate is not None]
    if len(available) < 2:
        return available

    best_route: list[ChannelState] | None = None
    best_length = float("inf")
    for first in available:
        route = [first]
        remaining = [st for st in available if st is not first]
        current = first.last_estimate
        assert current is not None

        while remaining:
            next_state = min(remaining, key=lambda st: distance_to_estimate(current, st))
            remaining.remove(next_state)
            route.append(next_state)
            assert next_state.last_estimate is not None
            current = next_state.last_estimate

        route = improve_clear_order_2opt(start, route)
        candidate_length = clear_route_length(start, route)
        if candidate_length < best_length:
            best_route = route
            best_length = candidate_length

    assert best_route is not None
    return best_route


def improve_clear_order_2opt(start: Point, route: list[ChannelState]) -> list[ChannelState]:
    route = list(route)
    improved = True
    while improved:
        improved = False
        for left in range(len(route)):
            previous = start if left == 0 else route[left - 1].last_estimate
            first = route[left].last_estimate
            assert previous is not None and first is not None
            for right in range(left + 1, len(route)):
                last = route[right].last_estimate
                assert last is not None
                old_length = distance(previous, first)
                new_length = distance(previous, last)
                if right + 1 < len(route):
                    following = route[right + 1].last_estimate
                    assert following is not None
                    old_length += distance(last, following)
                    new_length += distance(first, following)
                if new_length + 1e-9 < old_length:
                    route[left:right + 1] = reversed(route[left:right + 1])
                    improved = True
                    break
            if improved:
                break
    return route


def parse_channels(spec: str) -> list[int]:
    spec = spec.strip()
    if not spec:
        return list(range(1, 21))

    channels: set[int] = set()
    for part in spec.split(","):
        part = part.strip()
        if "-" in part:
            left, right = part.split("-", 1)
            channels.update(range(int(left), int(right) + 1))
        else:
            channels.add(int(part))

    invalid = [ch for ch in channels if ch < 1 or ch > 20]
    if invalid:
        raise ValueError(f"Invalid channels: {invalid}")
    return sorted(channels)
