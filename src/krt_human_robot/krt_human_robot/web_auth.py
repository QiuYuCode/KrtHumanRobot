"""Authentication database for the local robot console."""

from __future__ import annotations

import getpass
import sqlite3
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from werkzeug.security import check_password_hash, generate_password_hash


def now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


class AuthDatabase:
    def __init__(self, path: str) -> None:
        self.path = Path(path).expanduser()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        with self.connect() as connection:
            connection.executescript(
                """
                CREATE TABLE IF NOT EXISTS users (
                  id INTEGER PRIMARY KEY,
                  username TEXT NOT NULL UNIQUE,
                  password_hash TEXT NOT NULL,
                  role TEXT NOT NULL CHECK(role IN ('admin', 'operator')),
                  enabled INTEGER NOT NULL DEFAULT 1,
                  created_at TEXT NOT NULL
                );
                CREATE TABLE IF NOT EXISTS audit_logs (
                  id INTEGER PRIMARY KEY,
                  username TEXT NOT NULL,
                  ip TEXT NOT NULL,
                  action TEXT NOT NULL,
                  object_name TEXT NOT NULL,
                  success INTEGER NOT NULL,
                  detail TEXT NOT NULL,
                  created_at TEXT NOT NULL
                );
                """
            )

    def connect(self) -> sqlite3.Connection:
        connection = sqlite3.connect(self.path, timeout=5.0)
        connection.row_factory = sqlite3.Row
        connection.execute("PRAGMA busy_timeout = 5000")
        return connection

    def create_user(self, username: str, password: str, role: str) -> None:
        username = username.strip()
        if not username or len(username) > 64 or role not in {"admin", "operator"}:
            raise ValueError("用户名或角色无效")
        if len(password) < 6:
            raise ValueError("密码至少 6 个字符")
        with self.connect() as connection:
            connection.execute(
                "INSERT INTO users(username,password_hash,role,created_at) VALUES(?,?,?,?)",
                (username, generate_password_hash(password), role, now()),
            )
            connection.commit()

    def needs_setup(self) -> bool:
        with self.connect() as connection:
            return connection.execute("SELECT 1 FROM users LIMIT 1").fetchone() is None

    def create_initial_admin(
        self, username: str, password: str
    ) -> dict[str, Any] | None:
        username = username.strip()
        if not username or len(username) > 64:
            raise ValueError("用户名无效")
        if len(password) < 6:
            raise ValueError("密码至少 6 个字符")
        with self.connect() as connection:
            connection.execute("BEGIN IMMEDIATE")
            if connection.execute("SELECT 1 FROM users LIMIT 1").fetchone():
                return None
            cursor = connection.execute(
                "INSERT INTO users(username,password_hash,role,created_at) "
                "VALUES(?,?,?,?)",
                (username, generate_password_hash(password), "admin", now()),
            )
            row = connection.execute(
                "SELECT * FROM users WHERE id=?", (cursor.lastrowid,)
            ).fetchone()
            connection.commit()
        return dict(row)

    def authenticate(self, username: str, password: str) -> dict[str, Any] | None:
        with self.connect() as connection:
            row = connection.execute(
                "SELECT * FROM users WHERE username=? AND enabled=1", (username,)
            ).fetchone()
        return dict(row) if row and check_password_hash(row["password_hash"], password) else None

    def get_user(self, user_id: int) -> dict[str, Any] | None:
        with self.connect() as connection:
            row = connection.execute("SELECT * FROM users WHERE id=?", (user_id,)).fetchone()
        return dict(row) if row else None

    def list_users(self) -> list[dict[str, Any]]:
        with self.connect() as connection:
            rows = connection.execute(
                "SELECT id,username,role,enabled,created_at FROM users ORDER BY username"
            ).fetchall()
        return [dict(row) for row in rows]

    def update_user(self, user_id: int, *, enabled: bool | None = None,
                    password: str | None = None) -> None:
        if password is not None and len(password) < 6:
            raise ValueError("密码至少 6 个字符")
        updates, values = [], []
        if enabled is not None:
            updates.append("enabled=?")
            values.append(int(enabled))
        if password is not None:
            updates.append("password_hash=?")
            values.append(generate_password_hash(password))
        if not updates:
            return
        with self.connect() as connection:
            values.append(user_id)
            cursor = connection.execute(
                f"UPDATE users SET {', '.join(updates)} WHERE id=?", values
            )
            if cursor.rowcount == 0:
                raise KeyError("用户不存在")
            connection.commit()

    def audit(self, username: str, ip: str, action: str, object_name: str,
              success: bool, detail: str = "") -> None:
        with self.connect() as connection:
            connection.execute(
                """INSERT INTO audit_logs(username,ip,action,object_name,success,detail,created_at)
                   VALUES(?,?,?,?,?,?,?)""",
                (username, ip, action, object_name, int(success), detail[:500], now()),
            )
            connection.commit()

    def list_audit(self, limit: int = 200) -> list[dict[str, Any]]:
        with self.connect() as connection:
            rows = connection.execute(
                "SELECT * FROM audit_logs ORDER BY id DESC LIMIT ?", (min(limit, 500),)
            ).fetchall()
        return [dict(row) for row in rows]


def create_admin() -> None:
    import argparse
    import os

    parser = argparse.ArgumentParser(description="创建 KRT Web 管理员")
    parser.add_argument("username")
    parser.add_argument("--db", default=os.environ.get(
        "KRT_WEB_DB", "~/.local/share/krt_human_robot/web.db"
    ))
    args = parser.parse_args()
    password = getpass.getpass("密码（至少 6 位）: ")
    if password != getpass.getpass("确认密码: "):
        raise SystemExit("两次密码不一致")
    AuthDatabase(args.db).create_user(args.username, password, "admin")
    print(f"管理员已创建: {args.username}")
