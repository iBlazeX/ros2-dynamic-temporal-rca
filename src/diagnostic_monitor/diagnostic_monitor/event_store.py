"""SQLite timestamped event collection and storage.

Stores telemetry samples, detected anomalies, diagnosis reports and discovered
graph snapshots. Ensures persistent, queryable diagnostic event logs with
indexing on timestamp and node.
"""

import json
import sqlite3
import threading
import time
from typing import Any, Dict, List, Optional


class EventStore:
    """Thread-safe SQLite storage for diagnostic telemetry, anomalies, and RCA results."""

    def __init__(self, db_path: str = ':memory:'):
        self.db_path = db_path
        self._lock = threading.Lock()
        self._conn = sqlite3.connect(db_path, check_same_thread=False, timeout=5.0)
        self._conn.row_factory = sqlite3.Row
        # High-rate telemetry is buffered and flushed in batches: committing every
        # sample (with fsync) makes the monitor itself the slowest node in the
        # graph and it then measures its own backlog as "latency".
        self._telemetry_buf: List[tuple] = []
        try:
            if db_path != ':memory:':
                self._conn.execute('PRAGMA journal_mode=WAL')
            self._conn.execute('PRAGMA synchronous=NORMAL')
        except sqlite3.DatabaseError:
            pass
        self._init_tables()

    def _init_tables(self):
        with self._lock:
            cur = self._conn.cursor()
            cur.execute("""
                CREATE TABLE IF NOT EXISTS telemetry_events (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    timestamp REAL NOT NULL,
                    node TEXT NOT NULL,
                    topic TEXT NOT NULL,
                    metric TEXT NOT NULL,
                    value REAL NOT NULL
                )
            """)
            cur.execute("""
                CREATE TABLE IF NOT EXISTS anomalies (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    timestamp REAL NOT NULL,
                    node TEXT NOT NULL,
                    topic TEXT NOT NULL,
                    metric TEXT NOT NULL,
                    value REAL NOT NULL,
                    baseline_mean REAL NOT NULL,
                    baseline_std REAL NOT NULL,
                    z_score REAL NOT NULL,
                    severity REAL NOT NULL,
                    description TEXT NOT NULL,
                    onset_time REAL
                )
            """)
            # Migrate pre-existing databases that lack the onset_time column
            cols = [r[1] for r in cur.execute("PRAGMA table_info(anomalies)").fetchall()]
            if 'onset_time' not in cols:
                cur.execute("ALTER TABLE anomalies ADD COLUMN onset_time REAL")
            cur.execute("""
                CREATE TABLE IF NOT EXISTS diagnoses (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    timestamp REAL NOT NULL,
                    root_cause TEXT NOT NULL,
                    confidence REAL NOT NULL,
                    score REAL NOT NULL,
                    breakdown_json TEXT NOT NULL,
                    rank_list_json TEXT NOT NULL,
                    explanation TEXT NOT NULL
                )
            """)
            cur.execute("""
                CREATE TABLE IF NOT EXISTS graph_snapshots (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    timestamp REAL NOT NULL,
                    graph_json TEXT NOT NULL
                )
            """)
            cur.execute("CREATE INDEX IF NOT EXISTS idx_telem_time ON telemetry_events (timestamp)")
            cur.execute("CREATE INDEX IF NOT EXISTS idx_telem_node ON telemetry_events (node)")
            cur.execute("CREATE INDEX IF NOT EXISTS idx_anom_time ON anomalies (timestamp)")
            cur.execute("CREATE INDEX IF NOT EXISTS idx_anom_node ON anomalies (node)")
            cur.execute("CREATE INDEX IF NOT EXISTS idx_diag_time ON diagnoses (timestamp)")
            self._conn.commit()

    # ------------------------------------------------------------------ writes

    def insert_telemetry(self, timestamp: float, node: str, topic: str, metric: str, value: float):
        """Buffers a telemetry sample; call flush() (or wait for the periodic flush) to persist."""
        with self._lock:
            self._telemetry_buf.append((timestamp, node, topic, metric, float(value)))
            if len(self._telemetry_buf) >= 500:
                self._flush_locked()

    def _flush_locked(self):
        if not self._telemetry_buf:
            return
        cur = self._conn.cursor()
        cur.executemany(
            "INSERT INTO telemetry_events (timestamp, node, topic, metric, value) VALUES (?, ?, ?, ?, ?)",
            self._telemetry_buf
        )
        self._telemetry_buf = []
        self._conn.commit()

    def flush(self):
        with self._lock:
            self._flush_locked()

    def insert_anomaly(
        self, timestamp: float, node: str, topic: str, metric: str,
        value: float, baseline_mean: float, baseline_std: float,
        z_score: float, severity: float, description: str,
        onset_time: Optional[float] = None,
    ) -> int:
        with self._lock:
            cur = self._conn.cursor()
            cur.execute(
                """
                INSERT INTO anomalies
                (timestamp, node, topic, metric, value, baseline_mean, baseline_std, z_score, severity,
                 description, onset_time)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                """,
                (timestamp, node, topic, metric, float(value), float(baseline_mean),
                 float(baseline_std), float(z_score), float(severity), description, onset_time)
            )
            self._conn.commit()
            return cur.lastrowid

    def insert_diagnosis(
        self, timestamp: float, root_cause: str, confidence: float,
        score: float, breakdown: Dict[str, Any], rank_list: List[Dict[str, Any]],
        explanation: str
    ) -> int:
        with self._lock:
            cur = self._conn.cursor()
            cur.execute(
                """
                INSERT INTO diagnoses
                (timestamp, root_cause, confidence, score, breakdown_json, rank_list_json, explanation)
                VALUES (?, ?, ?, ?, ?, ?, ?)
                """,
                (timestamp, root_cause, float(confidence), float(score),
                 json.dumps(breakdown), json.dumps(rank_list), explanation)
            )
            self._conn.commit()
            return cur.lastrowid

    def insert_graph_snapshot(self, timestamp: float, graph_dict: Dict[str, Any]) -> int:
        with self._lock:
            cur = self._conn.cursor()
            cur.execute(
                "INSERT INTO graph_snapshots (timestamp, graph_json) VALUES (?, ?)",
                (timestamp, json.dumps(graph_dict))
            )
            self._conn.commit()
            return cur.lastrowid

    # ------------------------------------------------------------------- reads

    @staticmethod
    def _decode_diag(row) -> Dict[str, Any]:
        d = dict(row)
        d['breakdown'] = json.loads(d['breakdown_json'])
        d['rank_list'] = json.loads(d['rank_list_json'])
        return d

    def get_recent_anomalies(self, window_sec: float, current_time: Optional[float] = None) -> List[Dict[str, Any]]:
        if current_time is None:
            current_time = time.time()
        return self.get_anomalies_between(current_time - window_sec, current_time + 1e9)

    def get_anomalies_between(self, t_start: float, t_end: float) -> List[Dict[str, Any]]:
        with self._lock:
            cur = self._conn.cursor()
            cur.execute(
                "SELECT * FROM anomalies WHERE timestamp >= ? AND timestamp <= ? ORDER BY timestamp ASC, id ASC",
                (t_start, t_end)
            )
            return [dict(r) for r in cur.fetchall()]

    def get_all_anomalies(self) -> List[Dict[str, Any]]:
        with self._lock:
            cur = self._conn.cursor()
            cur.execute("SELECT * FROM anomalies ORDER BY timestamp ASC")
            return [dict(r) for r in cur.fetchall()]

    def get_latest_diagnosis(self) -> Optional[Dict[str, Any]]:
        with self._lock:
            cur = self._conn.cursor()
            cur.execute("SELECT * FROM diagnoses ORDER BY id DESC LIMIT 1")
            row = cur.fetchone()
            return self._decode_diag(row) if row else None

    def get_diagnoses_between(self, t_start: float, t_end: float) -> List[Dict[str, Any]]:
        with self._lock:
            cur = self._conn.cursor()
            cur.execute(
                "SELECT * FROM diagnoses WHERE timestamp >= ? AND timestamp <= ? ORDER BY id ASC",
                (t_start, t_end)
            )
            return [self._decode_diag(r) for r in cur.fetchall()]

    def get_latest_graph(self, before_time: Optional[float] = None) -> Optional[Dict[str, Any]]:
        """Returns the most recent discovered graph snapshot (optionally at/before a time)."""
        with self._lock:
            cur = self._conn.cursor()
            if before_time is None:
                cur.execute("SELECT * FROM graph_snapshots ORDER BY id DESC LIMIT 1")
            else:
                cur.execute(
                    "SELECT * FROM graph_snapshots WHERE timestamp <= ? ORDER BY id DESC LIMIT 1",
                    (before_time,)
                )
            row = cur.fetchone()
            if row:
                d = dict(row)
                d['graph'] = json.loads(d['graph_json'])
                return d
            return None

    def counts(self) -> Dict[str, int]:
        with self._lock:
            cur = self._conn.cursor()
            out = {}
            for table in ('telemetry_events', 'anomalies', 'diagnoses', 'graph_snapshots'):
                cur.execute(f"SELECT COUNT(*) AS c FROM {table}")
                out[table] = cur.fetchone()['c']
            return out

    def close(self):
        with self._lock:
            try:
                self._flush_locked()
            except sqlite3.DatabaseError:
                pass
            self._conn.close()
