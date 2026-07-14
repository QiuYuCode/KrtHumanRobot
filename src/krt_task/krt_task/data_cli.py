"""Import and export robot operational data."""

from __future__ import annotations

import argparse
import os
import shutil
import uuid
import wave
from pathlib import Path
from typing import Any

import yaml

from krt_task.robot_db import RobotDatabase, WaypointRecord


def wav_info(path: Path) -> tuple[float, int, int]:
    with wave.open(str(path), "rb") as handle:
        if handle.getcomptype() != "NONE" or handle.getsampwidth() != 2:
            raise ValueError(f"仅支持 16-bit PCM WAV: {path}")
        rate = int(handle.getframerate())
        channels = int(handle.getnchannels())
        return handle.getnframes() / float(rate), rate, channels


def import_yaml(args: argparse.Namespace) -> int:
    target = Path(args.db).expanduser()
    current = RobotDatabase(str(target))
    if not current.is_empty():
        raise RuntimeError("目标数据库不是空库，拒绝导入")
    temporary = target.with_name(f".{target.name}.{uuid.uuid4().hex}.tmp")
    database = RobotDatabase(str(temporary))
    routines = load_yaml(args.routines).get("routines", {}) if args.routines else {}
    waypoints = load_yaml(args.waypoints).get("waypoints", []) if args.waypoints else []
    media_dir = Path(args.media_dir).expanduser()
    media_dir.mkdir(parents=True, exist_ok=True)

    try:
        for name, spec in routines.items():
            database.save_routine(
                str(name), migrate_media(spec, database, media_dir)
            )
        for item in waypoints:
            position = item.get("position", {}) or {}
            orientation = item.get("orientation", {}) or {}
            routine = str(item.get("routine", "") or "")
            database.save_waypoint(WaypointRecord(
                name=str(item["name"]),
                frame_id=str(item.get("frame_id", "map")),
                x=float(position.get("x", 0.0)),
                y=float(position.get("y", 0.0)),
                z=float(position.get("z", 0.0)),
                qx=float(orientation.get("x", 0.0)),
                qy=float(orientation.get("y", 0.0)),
                qz=float(orientation.get("z", 0.0)),
                qw=float(orientation.get("w", 1.0)),
                routine=routine,
            ))
        for suffix in ("-wal", "-shm"):
            Path(f"{target}{suffix}").unlink(missing_ok=True)
        os.replace(temporary, target)
    finally:
        temporary.unlink(missing_ok=True)
        Path(f"{temporary}-wal").unlink(missing_ok=True)
        Path(f"{temporary}-shm").unlink(missing_ok=True)
    print(f"已导入 {len(routines)} 个 routine，{len(waypoints)} 个点位")
    return 0


def export_yaml(args: argparse.Namespace) -> int:
    database = RobotDatabase(args.db)
    output = Path(args.output).expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    data = {
        "routines": {row["name"]: row["spec"] for row in database.list_routines()},
        "waypoints": [waypoint_to_dict(row) for row in database.list_waypoints()],
        "media": database.list_media(),
    }
    output.write_text(
        yaml.safe_dump(data, allow_unicode=True, sort_keys=False), encoding="utf-8"
    )
    print(f"已导出: {output}")
    return 0


def load_yaml(path: str) -> dict[str, Any]:
    source = Path(path).expanduser()
    if not source.is_file():
        raise FileNotFoundError(source)
    with source.open(encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise ValueError(f"YAML 根节点必须是 map: {source}")
    return data


def migrate_media(spec: Any, database: RobotDatabase, media_dir: Path) -> Any:
    if isinstance(spec, list):
        return [migrate_media(item, database, media_dir) for item in spec]
    if not isinstance(spec, dict):
        return spec
    migrated = {key: migrate_media(value, database, media_dir) for key, value in spec.items()}
    if migrated.get("type") != "play_audio" or migrated.get("media_key"):
        return migrated
    source = Path(str(migrated.pop("file", migrated.pop("file_path", "")))).expanduser()
    if not source.is_file():
        raise FileNotFoundError(f"routine 媒体不存在: {source}")
    duration, rate, channels = wav_info(source)
    media_key = uuid.uuid4().hex
    filename = f"{media_key}.wav"
    shutil.copy2(source, media_dir / filename)
    database.add_media({
        "media_key": media_key, "display_name": source.name, "filename": filename,
        "size_bytes": source.stat().st_size, "duration_sec": duration,
        "sample_rate": rate, "channels": channels,
    })
    migrated["media_key"] = media_key
    return migrated


def waypoint_to_dict(row: WaypointRecord) -> dict[str, Any]:
    return {
        "name": row.name, "frame_id": row.frame_id,
        "position": {"x": row.x, "y": row.y, "z": row.z},
        "orientation": {"x": row.qx, "y": row.qy, "z": row.qz, "w": row.qw},
        "routine": row.routine,
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="KRT robot.db data tool")
    parser.add_argument("--db", default="~/maps/krt_robot.db")
    commands = parser.add_subparsers(dest="command", required=True)
    importer = commands.add_parser("import-yaml")
    importer.add_argument("--waypoints", default="~/maps/waypoints.yaml")
    importer.add_argument("--routines", default="~/maps/routines.yaml")
    importer.add_argument("--media-dir", default="~/music")
    exporter = commands.add_parser("export-yaml")
    exporter.add_argument("output")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    raise SystemExit(import_yaml(args) if args.command == "import-yaml" else export_yaml(args))
