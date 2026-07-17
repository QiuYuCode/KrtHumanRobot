"""SQLite storage shared by task, navigation, and Web entry points."""

from __future__ import annotations

import json
import sqlite3
from contextlib import contextmanager
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterator

import yaml


SCHEMA_VERSION = 4
PARALLEL_TASKS = {"play_audio", "arm_group", "gripper", "wait"}
TASKS = PARALLEL_TASKS | {"sequence", "parallel", "speak", "describe", "photo"}


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


@dataclass(frozen=True)
class WaypointRecord:
    name: str
    frame_id: str
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float
    routine: str = ""


class RobotDatabase:
    """Small transactional repository; callers open no long-lived connections."""

    def __init__(self, path: str) -> None:
        self.path = Path(path).expanduser()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.initialize()

    @contextmanager
    def connect(self) -> Iterator[sqlite3.Connection]:
        connection = sqlite3.connect(self.path, timeout=5.0)
        connection.row_factory = sqlite3.Row
        connection.execute("PRAGMA foreign_keys = ON")
        connection.execute("PRAGMA busy_timeout = 5000")
        try:
            yield connection
        finally:
            connection.close()

    def initialize(self) -> None:
        with self.connect() as connection:
            connection.execute("PRAGMA journal_mode = WAL")
            version = int(connection.execute("PRAGMA user_version").fetchone()[0])
            if version > SCHEMA_VERSION:
                raise RuntimeError(f"数据库版本过新: {version}")
            if version == 0:
                connection.executescript(
                    """
                    CREATE TABLE routines (
                      id INTEGER PRIMARY KEY,
                      name TEXT NOT NULL UNIQUE,
                      spec_json TEXT NOT NULL,
                      created_at TEXT NOT NULL,
                      updated_at TEXT NOT NULL
                    );
                    CREATE TABLE waypoints (
                      id INTEGER PRIMARY KEY,
                      name TEXT NOT NULL UNIQUE,
                      frame_id TEXT NOT NULL,
                      x REAL NOT NULL, y REAL NOT NULL, z REAL NOT NULL,
                      qx REAL NOT NULL, qy REAL NOT NULL,
                      qz REAL NOT NULL, qw REAL NOT NULL,
                      routine_id INTEGER REFERENCES routines(id) ON DELETE SET NULL,
                      created_at TEXT NOT NULL,
                      updated_at TEXT NOT NULL
                    );
                    CREATE TABLE media (
                      media_key TEXT PRIMARY KEY,
                      display_name TEXT NOT NULL,
                      filename TEXT NOT NULL UNIQUE,
                      size_bytes INTEGER NOT NULL,
                      duration_sec REAL NOT NULL,
                      sample_rate INTEGER NOT NULL,
                      channels INTEGER NOT NULL,
                      created_at TEXT NOT NULL
                    );
                    CREATE TABLE action_groups (
                      name TEXT PRIMARY KEY,
                      arm_target TEXT NOT NULL,
                      samples_json TEXT NOT NULL,
                      repeat_count INTEGER NOT NULL DEFAULT 1,
                      created_at TEXT NOT NULL,
                      updated_at TEXT NOT NULL
                    );
                    CREATE TABLE gripper_actions (
                      name TEXT PRIMARY KEY,
                      targets_json TEXT NOT NULL,
                      created_at TEXT NOT NULL,
                      updated_at TEXT NOT NULL
                    );
                    CREATE TABLE gripper_settings (
                      side TEXT PRIMARY KEY CHECK(side IN ('left', 'right')),
                      adapter_type TEXT NOT NULL,
                      adapter_index INTEGER NOT NULL,
                      device_id INTEGER NOT NULL,
                      listen_enabled INTEGER NOT NULL,
                      realtime_response_enabled INTEGER NOT NULL,
                      updated_at TEXT NOT NULL
                    );
                    PRAGMA user_version = 4;
                    """
                )
                connection.commit()
                return
            if version == 1:
                connection.executescript(
                    """
                    CREATE TABLE action_groups (
                      name TEXT PRIMARY KEY,
                      arm_target TEXT NOT NULL,
                      samples_json TEXT NOT NULL,
                      repeat_count INTEGER NOT NULL DEFAULT 1,
                      created_at TEXT NOT NULL,
                      updated_at TEXT NOT NULL
                    );
                    PRAGMA user_version = 2;
                    """
                )
                connection.commit()
                version = 2
            if version == 2:
                connection.executescript(
                    """
                    CREATE TABLE gripper_actions (
                      name TEXT PRIMARY KEY,
                      targets_json TEXT NOT NULL,
                      created_at TEXT NOT NULL,
                      updated_at TEXT NOT NULL
                    );
                    PRAGMA user_version = 3;
                    """
                )
                connection.commit()
                version = 3
            if version == 3:
                connection.executescript(
                    """
                    CREATE TABLE IF NOT EXISTS gripper_settings (
                      side TEXT PRIMARY KEY CHECK(side IN ('left', 'right')),
                      adapter_type TEXT NOT NULL,
                      adapter_index INTEGER NOT NULL,
                      device_id INTEGER NOT NULL,
                      listen_enabled INTEGER NOT NULL,
                      realtime_response_enabled INTEGER NOT NULL,
                      updated_at TEXT NOT NULL
                    );
                    PRAGMA user_version = 4;
                    """
                )
                connection.commit()

    def is_empty(self) -> bool:
        with self.connect() as connection:
            return not any(
                connection.execute(f"SELECT 1 FROM {table} LIMIT 1").fetchone()
                for table in (
                    "routines", "waypoints", "media", "action_groups", "gripper_actions"
                )
            )

    @staticmethod
    def _normalize_gripper_settings(side: str, values: dict[str, Any]) -> dict[str, Any]:
        if side not in {"left", "right"}:
            raise ValueError("side 必须是 left 或 right")
        allowed = {
            "adapter_type", "adapter_index", "device_id",
            "listen_enabled", "realtime_response_enabled",
        }
        unknown = set(values) - allowed
        if unknown:
            raise ValueError(f"不支持的夹爪设置: {', '.join(sorted(unknown))}")
        normalized = dict(values)
        if "adapter_type" in normalized:
            normalized["adapter_type"] = str(normalized["adapter_type"]).upper()
            if normalized["adapter_type"] not in {"ZLG_MINI", "ZLG_200U"}:
                raise ValueError("adapter_type 不受支持")
        if "adapter_index" in normalized:
            normalized["adapter_index"] = int(normalized["adapter_index"])
            if not 0 <= normalized["adapter_index"] <= 15:
                raise ValueError("adapter_index 必须在 0 到 15 之间")
        if "device_id" in normalized:
            normalized["device_id"] = int(normalized["device_id"])
            if not 1 <= normalized["device_id"] <= 255:
                raise ValueError("device_id 必须在 1 到 255 之间")
        for key in ("listen_enabled", "realtime_response_enabled"):
            if key in normalized:
                normalized[key] = bool(normalized[key])
        return normalized

    def ensure_gripper_settings(self, defaults: dict[str, dict[str, Any]]) -> None:
        now = utc_now()
        with self.connect() as connection:
            for side, values in defaults.items():
                normalized = self._normalize_gripper_settings(side, values)
                missing = {
                    "adapter_type", "adapter_index", "device_id",
                    "listen_enabled", "realtime_response_enabled",
                } - set(normalized)
                if missing:
                    raise ValueError(f"夹爪默认设置缺少: {', '.join(sorted(missing))}")
                connection.execute(
                    """INSERT OR IGNORE INTO gripper_settings(
                         side, adapter_type, adapter_index, device_id,
                         listen_enabled, realtime_response_enabled, updated_at
                       ) VALUES(?, ?, ?, ?, ?, ?, ?)""",
                    (
                        side, normalized["adapter_type"], normalized["adapter_index"],
                        normalized["device_id"], int(normalized["listen_enabled"]),
                        int(normalized["realtime_response_enabled"]), now,
                    ),
                )
            connection.commit()

    def list_gripper_settings(self) -> list[dict[str, Any]]:
        with self.connect() as connection:
            rows = connection.execute(
                "SELECT * FROM gripper_settings ORDER BY side"
            ).fetchall()
        result = []
        for row in rows:
            item = dict(row)
            item["listen_enabled"] = bool(item["listen_enabled"])
            item["realtime_response_enabled"] = bool(
                item["realtime_response_enabled"]
            )
            result.append(item)
        return result

    def get_gripper_settings(self, side: str) -> dict[str, Any]:
        self._normalize_gripper_settings(side, {})
        matches = [item for item in self.list_gripper_settings() if item["side"] == side]
        if not matches:
            raise KeyError(f"夹爪设置不存在: {side}")
        return matches[0]

    def update_gripper_settings(self, side: str, changes: dict[str, Any]) -> None:
        normalized = self._normalize_gripper_settings(side, changes)
        if not normalized:
            return
        assignments = ", ".join(f"{key}=?" for key in normalized)
        values = [
            int(value) if isinstance(value, bool) else value
            for value in normalized.values()
        ]
        with self.connect() as connection:
            cursor = connection.execute(
                f"UPDATE gripper_settings SET {assignments}, updated_at=? WHERE side=?",
                [*values, utc_now(), side],
            )
            if cursor.rowcount == 0:
                raise KeyError(f"夹爪设置不存在: {side}")
            connection.commit()

    def list_gripper_actions(self) -> list[dict[str, Any]]:
        with self.connect() as connection:
            rows = connection.execute(
                "SELECT name, targets_json, created_at, updated_at "
                "FROM gripper_actions ORDER BY name"
            ).fetchall()
        actions = []
        for row in rows:
            action = dict(row)
            action["targets"] = json.loads(action.pop("targets_json"))
            actions.append(action)
        return actions

    def get_gripper_action(self, name: str) -> dict[str, Any]:
        with self.connect() as connection:
            row = connection.execute(
                "SELECT name, targets_json FROM gripper_actions WHERE name = ?", (name,)
            ).fetchone()
        if row is None:
            raise KeyError(f"夹爪动作不存在: {name}")
        return {"name": row["name"], "targets": json.loads(row["targets_json"])}

    def save_gripper_action(self, name: str, targets: list[dict[str, Any]]) -> None:
        name = validate_name(name, "夹爪动作")
        normalized = validate_gripper_targets(targets)
        encoded = json.dumps(normalized, ensure_ascii=False, separators=(",", ":"))
        now = utc_now()
        with self.connect() as connection:
            connection.execute(
                """INSERT INTO gripper_actions(name, targets_json, created_at, updated_at)
                   VALUES(?, ?, ?, ?)
                   ON CONFLICT(name) DO UPDATE SET
                     targets_json=excluded.targets_json, updated_at=excluded.updated_at""",
                (name, encoded, now, now),
            )
            connection.commit()

    def rename_gripper_action(self, name: str, new_name: str) -> None:
        name = validate_name(name, "夹爪动作")
        new_name = validate_name(new_name, "夹爪动作")
        if name == new_name:
            return
        now = utc_now()
        with self.connect() as connection:
            connection.execute("BEGIN IMMEDIATE")
            if connection.execute(
                "SELECT 1 FROM gripper_actions WHERE name = ?", (name,)
            ).fetchone() is None:
                raise KeyError(f"夹爪动作不存在: {name}")
            if connection.execute(
                "SELECT 1 FROM gripper_actions WHERE name = ?", (new_name,)
            ).fetchone() is not None:
                raise ValueError("夹爪动作名称已存在")
            connection.execute(
                "UPDATE gripper_actions SET name=?, updated_at=? WHERE name=?",
                (new_name, now, name),
            )
            for row in connection.execute("SELECT id, spec_json FROM routines").fetchall():
                spec = json.loads(row["spec_json"])
                if _rename_gripper_action_refs(spec, name, new_name):
                    connection.execute(
                        "UPDATE routines SET spec_json=?, updated_at=? WHERE id=?",
                        (
                            json.dumps(spec, ensure_ascii=False, separators=(",", ":")),
                            now,
                            row["id"],
                        ),
                    )
            connection.commit()

    def delete_gripper_action(self, name: str) -> None:
        with self.connect() as connection:
            connection.execute("BEGIN IMMEDIATE")
            rows = connection.execute(
                "SELECT name, spec_json FROM routines"
            ).fetchall()
            for row in rows:
                if _uses_gripper_action(json.loads(row["spec_json"]), name):
                    raise ValueError(f"夹爪动作正在被 routine 使用: {row['name']}")
            cursor = connection.execute("DELETE FROM gripper_actions WHERE name = ?", (name,))
            if cursor.rowcount == 0:
                raise KeyError(f"夹爪动作不存在: {name}")
            connection.commit()

    def list_action_groups(self) -> list[dict[str, Any]]:
        with self.connect() as connection:
            rows = connection.execute(
                "SELECT name, arm_target, samples_json, repeat_count, created_at, updated_at "
                "FROM action_groups ORDER BY name"
            ).fetchall()
        groups = []
        for row in rows:
            group = dict(row)
            group["sample_count"] = len(json.loads(group.pop("samples_json")))
            groups.append(group)
        return groups

    def get_action_group(self, name: str) -> dict[str, Any]:
        with self.connect() as connection:
            row = connection.execute(
                "SELECT name, arm_target, samples_json, repeat_count "
                "FROM action_groups WHERE name = ?", (name,)
            ).fetchone()
        if row is None:
            raise KeyError(f"动作组不存在: {name}")
        group = dict(row)
        group["samples"] = json.loads(group.pop("samples_json"))
        return group

    def save_action_group(
        self,
        name: str,
        arm_target: str,
        samples: list[dict[str, Any]],
        repeat_count: int = 1,
    ) -> None:
        name = validate_name(name, "动作组")
        if arm_target not in {"left", "right", "both", "unknown"}:
            raise ValueError("动作组 arm_target 必须是 left/right/both/unknown")
        if not samples:
            raise ValueError("动作组至少需要一个关节采样")
        for sample in samples:
            if (
                not isinstance(sample, dict)
                or not sample.get("name")
                or not sample.get("position")
            ):
                raise ValueError("动作组关节采样无效")
        if repeat_count <= 0:
            raise ValueError("动作组 repeat_count 必须大于 0")
        now = utc_now()
        encoded = json.dumps(samples, ensure_ascii=False, separators=(",", ":"))
        with self.connect() as connection:
            connection.execute(
                """INSERT INTO action_groups(
                       name, arm_target, samples_json, repeat_count, created_at, updated_at)
                   VALUES(?, ?, ?, ?, ?, ?)
                   ON CONFLICT(name) DO UPDATE SET
                     arm_target=excluded.arm_target, samples_json=excluded.samples_json,
                     repeat_count=excluded.repeat_count, updated_at=excluded.updated_at""",
                (name, arm_target, encoded, repeat_count, now, now),
            )
            connection.commit()

    def rename_action_group(self, name: str, new_name: str) -> None:
        name = validate_name(name, "动作组")
        new_name = validate_name(new_name, "动作组")
        if name == new_name:
            return
        now = utc_now()
        with self.connect() as connection:
            if connection.execute(
                "SELECT 1 FROM action_groups WHERE name = ?", (name,)
            ).fetchone() is None:
                raise KeyError(f"动作组不存在: {name}")
            if connection.execute(
                "SELECT 1 FROM action_groups WHERE name = ?", (new_name,)
            ).fetchone() is not None:
                raise ValueError("动作组名称已存在")
            connection.execute(
                "UPDATE action_groups SET name=?, updated_at=? WHERE name=?",
                (new_name, now, name),
            )
            for row in connection.execute("SELECT id, spec_json FROM routines").fetchall():
                spec = json.loads(row["spec_json"])
                if _rename_action_group_refs(spec, name, new_name):
                    connection.execute(
                        "UPDATE routines SET spec_json=?, updated_at=? WHERE id=?",
                        (
                            json.dumps(spec, ensure_ascii=False, separators=(",", ":")),
                            now,
                            row["id"],
                        ),
                    )
            connection.commit()

    def migrate_action_groups(self, groups_file: str) -> int:
        """Import legacy YAML action groups once without overwriting database groups."""
        path = Path(groups_file).expanduser()
        if not path.is_file():
            return 0
        data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        groups = data.get("groups", {})
        if not isinstance(groups, dict):
            raise ValueError("旧动作组 YAML 缺少 groups 映射")
        imported = 0
        for name, group in groups.items():
            if not isinstance(group, dict):
                continue
            samples = []
            for step in group.get("steps", []):
                payload_file = step.get("payload_file") if isinstance(step, dict) else None
                if not payload_file:
                    continue
                payload_path = (path.parent / str(payload_file)).resolve()
                try:
                    payload = yaml.safe_load(payload_path.read_text(encoding="utf-8")) or {}
                except OSError as exc:
                    raise ValueError(f"无法读取旧动作组采样: {payload_path}") from exc
                samples.append({
                    "name": list(payload.get("name", [])),
                    "position": [float(value) for value in payload.get("position", [])],
                    "velocity": [float(value) for value in payload.get("velocity", [])],
                    "effort": [float(value) for value in payload.get("effort", [])],
                    "hold_sec": float(step.get("hold_sec", 0.02)),
                    "timeout_sec": float(step.get("timeout_sec", 8.0)),
                    "wait_reach": bool(step.get("wait_reach", False)),
                })
            if not samples:
                continue
            with self.connect() as connection:
                exists = connection.execute(
                    "SELECT 1 FROM action_groups WHERE name = ?", (str(name),)
                ).fetchone()
            if exists is None:
                self.save_action_group(
                    str(name), str(group.get("arm_target", "unknown")), samples,
                    int(group.get("repeat_count", 1)),
                )
                imported += 1
        return imported

    def list_routines(self) -> list[dict[str, Any]]:
        with self.connect() as connection:
            rows = connection.execute(
                "SELECT name, spec_json, created_at, updated_at FROM routines ORDER BY name"
            ).fetchall()
        return [
            {"name": row["name"], "spec": json.loads(row["spec_json"]),
             "created_at": row["created_at"], "updated_at": row["updated_at"]}
            for row in rows
        ]

    def get_routine(self, name: str) -> dict[str, Any]:
        with self.connect() as connection:
            row = connection.execute(
                "SELECT spec_json FROM routines WHERE name = ?", (name,)
            ).fetchone()
        if row is None:
            raise KeyError(f"routine 不存在: {name}")
        return json.loads(row["spec_json"])

    def save_routine(self, name: str, spec: dict[str, Any]) -> None:
        name = validate_name(name, "routine")
        validate_routine(spec)
        encoded = json.dumps(spec, ensure_ascii=False, separators=(",", ":"))
        now = utc_now()
        with self.connect() as connection:
            connection.execute("BEGIN IMMEDIATE")
            self._validate_gripper_action_refs(spec, connection)
            connection.execute(
                """INSERT INTO routines(name, spec_json, created_at, updated_at)
                   VALUES(?, ?, ?, ?)
                   ON CONFLICT(name) DO UPDATE SET
                     spec_json=excluded.spec_json, updated_at=excluded.updated_at""",
                (name, encoded, now, now),
            )
            connection.commit()

    def _validate_gripper_action_refs(
        self, spec: Any, connection: sqlite3.Connection | None = None
    ) -> None:
        if not isinstance(spec, dict):
            return
        if spec.get("type") == "gripper" and str(spec.get("action_name", "")).strip():
            action_name = str(spec["action_name"]).strip()
            if connection is None:
                self.get_gripper_action(action_name)
            elif connection.execute(
                "SELECT 1 FROM gripper_actions WHERE name = ?", (action_name,)
            ).fetchone() is None:
                raise KeyError(f"夹爪动作不存在: {action_name}")
        for step in spec.get("steps", []):
            self._validate_gripper_action_refs(step, connection)

    def delete_routine(self, name: str) -> None:
        with self.connect() as connection:
            cursor = connection.execute("DELETE FROM routines WHERE name = ?", (name,))
            if cursor.rowcount == 0:
                raise KeyError(f"routine 不存在: {name}")
            connection.commit()

    def list_waypoints(self) -> list[WaypointRecord]:
        with self.connect() as connection:
            rows = connection.execute(
                """SELECT w.name, w.frame_id, w.x, w.y, w.z,
                          w.qx, w.qy, w.qz, w.qw, COALESCE(r.name, '') AS routine
                   FROM waypoints w LEFT JOIN routines r ON r.id = w.routine_id
                   ORDER BY w.id"""
            ).fetchall()
        return [WaypointRecord(**dict(row)) for row in rows]

    def save_waypoint(self, waypoint: WaypointRecord) -> None:
        validate_name(waypoint.name, "点位")
        with self.connect() as connection:
            routine_id = self._routine_id(connection, waypoint.routine)
            now = utc_now()
            connection.execute(
                """INSERT INTO waypoints(
                       name, frame_id, x, y, z, qx, qy, qz, qw,
                       routine_id, created_at, updated_at)
                   VALUES(?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                   ON CONFLICT(name) DO UPDATE SET
                     frame_id=excluded.frame_id, x=excluded.x, y=excluded.y,
                     z=excluded.z, qx=excluded.qx, qy=excluded.qy,
                     qz=excluded.qz, qw=excluded.qw,
                     routine_id=excluded.routine_id, updated_at=excluded.updated_at""",
                (*list(asdict(waypoint).values())[:-1], routine_id, now, now),
            )
            connection.commit()

    def bind_waypoint(self, name: str, routine: str) -> None:
        with self.connect() as connection:
            routine_id = self._routine_id(connection, routine)
            cursor = connection.execute(
                "UPDATE waypoints SET routine_id=?, updated_at=? WHERE name=?",
                (routine_id, utc_now(), name),
            )
            if cursor.rowcount == 0:
                raise KeyError(f"点位不存在: {name}")
            connection.commit()

    def delete_waypoint(self, name: str) -> None:
        with self.connect() as connection:
            cursor = connection.execute("DELETE FROM waypoints WHERE name = ?", (name,))
            if cursor.rowcount == 0:
                raise KeyError(f"点位不存在: {name}")
            connection.commit()

    def add_media(self, record: dict[str, Any]) -> None:
        with self.connect() as connection:
            connection.execute(
                """INSERT INTO media(media_key, display_name, filename, size_bytes,
                   duration_sec, sample_rate, channels, created_at)
                   VALUES(?, ?, ?, ?, ?, ?, ?, ?)""",
                (record["media_key"], record["display_name"], record["filename"],
                 int(record["size_bytes"]), float(record["duration_sec"]),
                 int(record["sample_rate"]), int(record["channels"]), utc_now()),
            )
            connection.commit()

    def list_media(self) -> list[dict[str, Any]]:
        with self.connect() as connection:
            return [dict(row) for row in connection.execute(
                "SELECT * FROM media ORDER BY display_name"
            ).fetchall()]

    def get_media(self, media_key: str) -> dict[str, Any]:
        with self.connect() as connection:
            row = connection.execute(
                "SELECT * FROM media WHERE media_key = ?", (media_key,)
            ).fetchone()
        if row is None:
            raise KeyError(f"媒体不存在: {media_key}")
        return dict(row)

    def delete_media(self, media_key: str) -> dict[str, Any]:
        for routine in self.list_routines():
            if media_key in json.dumps(routine["spec"], ensure_ascii=False):
                raise ValueError(f"媒体正在被 routine 使用: {routine['name']}")
        record = self.get_media(media_key)
        with self.connect() as connection:
            connection.execute("DELETE FROM media WHERE media_key = ?", (media_key,))
            connection.commit()
        return record

    @staticmethod
    def _routine_id(connection: sqlite3.Connection, routine: str) -> int | None:
        if not routine:
            return None
        row = connection.execute("SELECT id FROM routines WHERE name = ?", (routine,)).fetchone()
        if row is None:
            raise KeyError(f"routine 不存在: {routine}")
        return int(row["id"])


def validate_name(value: str, label: str) -> str:
    value = str(value).strip()
    if not value or len(value) > 80 or any(char in value for char in "/\\\0"):
        raise ValueError(f"{label}名称无效")
    return value


def _rename_action_group_refs(spec: Any, name: str, new_name: str) -> bool:
    if not isinstance(spec, dict):
        return False
    if spec.get("type") == "arm_group" and spec.get("group_name") == name:
        spec["group_name"] = new_name
        return True
    changed = False
    for step in spec.get("steps", []):
        if isinstance(step, dict):
            changed = _rename_action_group_refs(step, name, new_name) or changed
    return changed


def _rename_gripper_action_refs(spec: Any, name: str, new_name: str) -> bool:
    if not isinstance(spec, dict):
        return False
    changed = False
    if spec.get("type") == "gripper" and spec.get("action_name") == name:
        spec["action_name"] = new_name
        changed = True
    for step in spec.get("steps", []):
        changed = _rename_gripper_action_refs(step, name, new_name) or changed
    return changed


def _uses_gripper_action(spec: Any, name: str) -> bool:
    if not isinstance(spec, dict):
        return False
    if spec.get("type") == "gripper" and spec.get("action_name") == name:
        return True
    return any(_uses_gripper_action(step, name) for step in spec.get("steps", []))


def validate_gripper_targets(targets: Any) -> list[dict[str, int | str]]:
    if not isinstance(targets, list) or not 1 <= len(targets) <= 2:
        raise ValueError("夹爪动作必须包含一到两个目标")
    normalized = []
    sides = set()
    for target in targets:
        if not isinstance(target, dict) or target.get("side") not in {"left", "right"}:
            raise ValueError("夹爪目标 side 必须是 left/right")
        side = str(target["side"])
        if side in sides:
            raise ValueError("左右手目标不能重复")
        sides.add(side)
        item: dict[str, int | str] = {"side": side}
        for key, low, high in (
            ("finger_id", 0, 3), ("position", 0, 1000), ("speed", 0, 1000),
            ("force", 0, 255), ("wait_time", 0, 255),
        ):
            try:
                value = int(target[key])
            except (KeyError, TypeError, ValueError) as exc:
                raise ValueError(f"夹爪目标缺少有效 {key}") from exc
            if not low <= value <= high:
                raise ValueError(f"gripper.{key} 超出范围 {low}-{high}")
            item[key] = value
        normalized.append(item)
    return normalized


def validate_routine(spec: Any, *, parallel: bool = False) -> None:
    if not isinstance(spec, dict):
        raise ValueError("routine 必须是 object")
    task = str(spec.get("type", "")).strip()
    if task not in TASKS or (parallel and task not in PARALLEL_TASKS):
        raise ValueError(f"不支持的 routine task: {task}")
    if task in {"sequence", "parallel"}:
        steps = spec.get("steps")
        if not isinstance(steps, list) or not steps:
            raise ValueError(f"{task}.steps 必须是非空列表")
        if parallel and task in {"sequence", "parallel"}:
            raise ValueError("parallel 不允许嵌套")
        for step in steps:
            validate_routine(step, parallel=task == "parallel")
    elif task == "speak" and not str(spec.get("text", "")).strip():
        raise ValueError("speak 缺少 text")
    elif task == "play_audio" and not str(spec.get("media_key", "")).strip():
        raise ValueError("play_audio 缺少 media_key")
    elif task == "arm_group":
        if not str(spec.get("group_name", "")).strip():
            raise ValueError("arm_group 缺少 group_name")
        if str(spec.get("arm_target", "left")) not in {"left", "right", "both"}:
            raise ValueError("arm_target 必须是 left/right/both")
    elif task == "gripper":
        if str(spec.get("action_name", "")).strip():
            return
        if str(spec.get("side", "")) not in {"left", "right"}:
            raise ValueError("gripper.side 必须是 left/right")
        for key, low, high in (
            ("finger_id", 0, 3), ("position", 0, 1000), ("speed", 0, 1000),
            ("force", 0, 255), ("wait_time", 0, 255),
        ):
            value = int(spec.get(key, 0 if key == "finger_id" else -1))
            if not low <= value <= high:
                raise ValueError(f"gripper.{key} 超出范围 {low}-{high}")
