from __future__ import annotations

import csv
import json
import os
import time
import uuid
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


class RobotClientError(RuntimeError):
    """Raised when a simulator request cannot be completed."""


@dataclass(frozen=True)
class RobotConfig:
    base_url: str = "http://127.0.0.1:2026"
    arena_id: str = "default"
    robot_id: str = ""
    timeout_s: float = 5.0
    max_retries: int = 2
    retry_delay_s: float = 0.35
    log_dir: Path = Path("logs")


class RobotClient:
    """
    Thin control layer for the simulator.

    Algorithm code should call enter(), measure(), clear(), and exit().
    This class handles request ids, retries, response validation, and logs.
    """

    CSV_FIELDS = [
        "step_id",
        "request_id",
        "action",
        "x",
        "y",
        "channel",
        "accepted",
        "measure_result",
        "svd_deg",
        "clear_result",
        "virtual_time_s",
        "real_timestamp_ms",
        "http_status",
        "error",
    ]

    def __init__(self, config: RobotConfig):
        if not config.robot_id:
            raise ValueError("robot_id is required. Pass --robot-id or set ROBOT_ID.")

        self.config = config
        self.base_url = config.base_url.rstrip("/")
        self.step_id = 0
        self.last_virtual_time_s: float | None = None
        self.current_position: tuple[float, float] = (0.0, 0.0)
        self.current_measure_channel = 1

        run_name = datetime.now().strftime("run_%Y%m%d_%H%M%S")
        self.run_dir = config.log_dir / run_name
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self.jsonl_path = self.run_dir / "run_log.jsonl"
        self.csv_path = self.run_dir / "summary.csv"

        self._csv_file = self.csv_path.open("w", newline="", encoding="utf-8-sig")
        self._csv_writer = csv.DictWriter(self._csv_file, fieldnames=self.CSV_FIELDS)
        self._csv_writer.writeheader()
        self._csv_file.flush()

    def close(self) -> None:
        self._csv_file.close()

    def __enter__(self) -> "RobotClient":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def enter(self) -> dict[str, Any]:
        payload = self._base_payload("enter")
        return self._post_action("enter", "/enter", payload)

    def measure(self, x: float, y: float, channel: int) -> dict[str, Any]:
        self._validate_position(x, y)
        self._validate_channel(channel)
        payload = self._position_payload("measure", x, y, channel)
        response = self._post_action("measure", "/measure", payload, x=x, y=y, channel=channel)
        if response.get("accepted") is True:
            self.current_position = (float(x), float(y))
            self.current_measure_channel = int(channel)
        return response

    def clear(self, x: float, y: float, channel: int) -> dict[str, Any]:
        self._validate_position(x, y)
        self._validate_channel(channel)
        payload = self._position_payload("clear", x, y, channel)
        response = self._post_action("clear", "/clear", payload, x=x, y=y, channel=channel)
        if response.get("accepted") is True:
            self.current_position = (float(x), float(y))
        return response

    def exit(self) -> dict[str, Any]:
        payload = self._base_payload("exit")
        return self._post_action("exit", "/exit", payload)

    def _base_payload(self, action: str) -> dict[str, Any]:
        return {
            "arena_id": self.config.arena_id,
            "robot_id": self.config.robot_id,
            "request_id": self._new_request_id(action),
        }

    def _position_payload(self, action: str, x: float, y: float, channel: int) -> dict[str, Any]:
        payload = self._base_payload(action)
        payload["position"] = {"x": float(x), "y": float(y)}
        payload["channel"] = int(channel)
        return payload

    def _new_request_id(self, action: str) -> str:
        self.step_id += 1
        suffix = uuid.uuid4().hex[:8]
        return f"{self.step_id:05d}-{action}-{suffix}"

    def _post_action(
        self,
        action: str,
        path: str,
        payload: dict[str, Any],
        *,
        x: float | None = None,
        y: float | None = None,
        channel: int | None = None,
    ) -> dict[str, Any]:
        response: dict[str, Any] | None = None
        http_status: int | None = None
        error = ""

        for attempt in range(self.config.max_retries + 1):
            try:
                http_status, response = self._post_json(path, payload)
                error = ""
                break
            except HTTPError as exc:
                http_status = exc.code
                error = self._read_http_error(exc)
                break
            except (URLError, TimeoutError, OSError) as exc:
                error = f"{type(exc).__name__}: {exc}"
                if attempt >= self.config.max_retries:
                    break
                time.sleep(self.config.retry_delay_s * (attempt + 1))

        if response is None:
            response = {
                "accepted": False,
                "real_timestamp_ms": None,
                "virtual_time_s": None,
                "error": error or "No response",
            }

        if response.get("accepted") is True and "virtual_time_s" in response:
            self.last_virtual_time_s = response["virtual_time_s"]

        self._write_log(
            action=action,
            payload=payload,
            response=response,
            x=x,
            y=y,
            channel=channel,
            http_status=http_status,
            error=error,
        )

        if http_status is not None and http_status != 200:
            raise RobotClientError(f"{action} failed with HTTP {http_status}: {error}")
        if response.get("accepted") is not True:
            raise RobotClientError(f"{action} was not accepted: {response}")

        return response

    def _post_json(self, path: str, payload: dict[str, Any]) -> tuple[int, dict[str, Any]]:
        body = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
        request = Request(
            self.base_url + path,
            data=body,
            headers={"Content-Type": "application/json; charset=utf-8"},
            method="POST",
        )
        with urlopen(request, timeout=self.config.timeout_s) as http_response:
            raw = http_response.read().decode("utf-8")
            return http_response.status, json.loads(raw)

    @staticmethod
    def _read_http_error(exc: HTTPError) -> str:
        try:
            body = exc.read().decode("utf-8", errors="replace")
        except Exception:
            body = ""
        return body or str(exc)

    def _write_log(
        self,
        *,
        action: str,
        payload: dict[str, Any],
        response: dict[str, Any],
        x: float | None,
        y: float | None,
        channel: int | None,
        http_status: int | None,
        error: str,
    ) -> None:
        record = {
            "step_id": self.step_id,
            "request_id": payload.get("request_id"),
            "action": action,
            "x": x,
            "y": y,
            "channel": channel,
            "payload": payload,
            "response_json": response,
            "accepted": response.get("accepted"),
            "measure_result": response.get("measure_result"),
            "svd_deg": response.get("svd_deg"),
            "clear_result": response.get("clear_result"),
            "virtual_time_s": response.get("virtual_time_s"),
            "real_timestamp_ms": response.get("real_timestamp_ms"),
            "http_status": http_status,
            "error": error or response.get("error", ""),
            "created_at": datetime.now().isoformat(timespec="seconds"),
        }

        with self.jsonl_path.open("a", encoding="utf-8") as f:
            f.write(json.dumps(record, ensure_ascii=False) + "\n")

        csv_record = {field: record.get(field, "") for field in self.CSV_FIELDS}
        self._csv_writer.writerow(csv_record)
        self._csv_file.flush()

    @staticmethod
    def _validate_position(x: float, y: float) -> None:
        for name, value in (("x", x), ("y", y)):
            if not isinstance(value, (int, float)):
                raise ValueError(f"{name} must be a number")
            if value != value or value in (float("inf"), float("-inf")):
                raise ValueError(f"{name} must be finite")
            if abs(value) > 2_000_000:
                raise ValueError(f"{name} coordinate is out of simulator range")

    @staticmethod
    def _validate_channel(channel: int) -> None:
        if not isinstance(channel, int):
            raise ValueError("channel must be an integer")
        if not 1 <= channel <= 20:
            raise ValueError("channel must be between 1 and 20")


def config_from_env(
    *,
    robot_id: str | None = None,
    base_url: str | None = None,
    log_dir: str | Path | None = None,
) -> RobotConfig:
    return RobotConfig(
        robot_id=robot_id or os.getenv("ROBOT_ID", ""),
        base_url=base_url or os.getenv("SIM_BASE_URL", "http://127.0.0.1:2026"),
        log_dir=Path(log_dir or os.getenv("ROBOT_LOG_DIR", "logs")),
    )
