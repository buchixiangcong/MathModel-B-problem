from __future__ import annotations

import hashlib
import math
import random
from dataclasses import dataclass
from typing import Any, Sequence


@dataclass
class OmnidirectionalSource:
    channel: int
    x: float
    y: float
    reception_radius: float
    cleared: bool = False


class LocalOmnidirectionalSimulator:
    """In-process simulator implementing the four methods used by the strategy."""

    def __init__(self, sources: Sequence[OmnidirectionalSource]):
        channels = [source.channel for source in sources]
        if len(channels) != len(set(channels)):
            raise ValueError("source channels must be unique")
        self.sources = {source.channel: source for source in sources}
        self.current_position = (0.0, 0.0)
        self.current_measure_channel = 1
        self.virtual_time_s = 0.0
        self.entered = False
        self.exited = False
        self.actions: list[dict[str, Any]] = []

    @classmethod
    def random_case(
        cls,
        seed: int,
        source_count: int | None = None,
        target_radius: float = 1800.0,
    ) -> "LocalOmnidirectionalSimulator":
        generator = random.Random(seed)
        count = source_count if source_count is not None else generator.randint(10, 16)
        channels = generator.sample(range(1, 21), count)
        sources: list[OmnidirectionalSource] = []
        for channel in channels:
            radius = target_radius * math.sqrt(generator.random())
            angle = generator.uniform(0.0, 2.0 * math.pi)
            sources.append(
                OmnidirectionalSource(
                    channel=channel,
                    x=radius * math.cos(angle),
                    y=radius * math.sin(angle),
                    reception_radius=generator.uniform(1000.0, 1500.0),
                )
            )
        return cls(sources)

    def enter(self) -> dict[str, Any]:
        if self.entered or self.exited:
            raise RuntimeError("invalid enter")
        self.entered = True
        return self._record(
            "enter",
            {
                "accepted": True,
                "virtual_time_s": self.virtual_time_s,
                "max_virtual_duration_s": 360000,
                "max_real_duration_s": 1200,
                "remaining_real_duration_s": 1200,
            },
        )

    def measure(self, x: float, y: float, channel: int) -> dict[str, Any]:
        self._require_running()
        self._move_to(x, y)
        if channel != self.current_measure_channel:
            self.virtual_time_s += 1.0
        self.current_measure_channel = channel
        self.virtual_time_s += 5.0

        source = self.sources.get(channel)
        response: dict[str, Any] = {
            "accepted": True,
            "virtual_time_s": self.virtual_time_s,
        }
        if source is None or source.cleared:
            response["measure_result"] = "no_signal"
        else:
            distance = math.hypot(source.x - x, source.y - y)
            if distance > source.reception_radius + 1e-9:
                response["measure_result"] = "no_signal"
            elif distance <= 5.0 + 1e-9:
                response["measure_result"] = "near"
            else:
                true_bearing = math.degrees(math.atan2(source.y - y, source.x - x)) % 360.0
                error = self._fixed_error(channel, x, y)
                response["measure_result"] = "direction"
                response["svd_deg"] = round((true_bearing + error) % 360.0, 2) % 360.0
        return self._record(
            "measure", response, x=float(x), y=float(y), channel=channel
        )

    def clear(self, x: float, y: float, channel: int) -> dict[str, Any]:
        self._require_running()
        self._move_to(x, y)
        source = self.sources.get(channel)
        success = (
            source is not None
            and not source.cleared
            and math.hypot(source.x - x, source.y - y) <= 20.0 + 1e-9
        )
        self.virtual_time_s += 5.0 if success else 3.0
        if success:
            source.cleared = True
        return self._record(
            "clear",
            {
                "accepted": True,
                "virtual_time_s": self.virtual_time_s,
                "clear_result": "success" if success else "no_target_in_range",
            },
            x=float(x),
            y=float(y),
            channel=channel,
        )

    def exit(self) -> dict[str, Any]:
        self._require_running()
        self.exited = True
        return self._record(
            "exit",
            {
                "accepted": True,
                "virtual_time_s": self.virtual_time_s,
                "exit_reason": "user_exit",
            },
        )

    def uncleared_channels(self) -> list[int]:
        return sorted(
            channel for channel, source in self.sources.items() if not source.cleared
        )

    def _move_to(self, x: float, y: float) -> None:
        self.virtual_time_s += math.hypot(
            x - self.current_position[0], y - self.current_position[1]
        ) / 5.0
        self.current_position = (float(x), float(y))

    def _require_running(self) -> None:
        if not self.entered or self.exited:
            raise RuntimeError("simulator is not running")

    @staticmethod
    def _fixed_error(channel: int, x: float, y: float) -> float:
        key = f"{channel}:{x:.6f}:{y:.6f}".encode("ascii")
        integer = int.from_bytes(hashlib.sha256(key).digest()[:8], "big")
        # Leave half a display unit for rounding to two decimal places, so the
        # reported bearing still obeys the official [-1, 1] degree bound.
        return 1.99 * integer / (2**64 - 1) - 0.995

    def _record(
        self,
        action: str,
        response: dict[str, Any],
        **parameters: Any,
    ) -> dict[str, Any]:
        response.setdefault("real_timestamp_ms", 0)
        self.actions.append(
            {"action": action, **parameters, "response": dict(response)}
        )
        return response
