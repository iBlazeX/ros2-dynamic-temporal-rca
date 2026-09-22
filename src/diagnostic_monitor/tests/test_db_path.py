"""DB-path resolution: monitor, CLI and controller must agree on one default
that does not depend on the current working directory."""

import os
import sys

import pytest

from diagnostic_monitor import db_path as dbp
from diagnostic_monitor.event_store import EventStore


def test_priority_explicit_over_param_over_env(tmp_path):
    env = {dbp.ENV_VAR: str(tmp_path / 'env.db')}
    share = str(tmp_path / 'ws' / 'install' / 'diagnostic_monitor' / 'share' / 'diagnostic_monitor')
    assert dbp.resolve_db_path('cli.db', 'param.db', env, share) == 'cli.db'
    assert dbp.resolve_db_path(None, 'param.db', env, share) == 'param.db'
    assert dbp.resolve_db_path(None, '', env, share) == str(tmp_path / 'env.db')      # '' = unset
    assert dbp.resolve_db_path(None, None, {}, share) == str(tmp_path / 'ws' / 'events.db')


def test_workspace_root_derivation():
    assert dbp.workspace_root_from_share_dir('/home/u/ros2_rca_ws/install/diagnostic_monitor/share/diagnostic_monitor') \
        == '/home/u/ros2_rca_ws'
    # merged install layout
    assert dbp.workspace_root_from_share_dir('/home/u/ws/install/share/diagnostic_monitor') == '/home/u/ws'
    # system install: no colcon workspace
    assert dbp.workspace_root_from_share_dir('/opt/ros/jazzy/share/diagnostic_monitor') is None
    assert dbp.workspace_root_from_share_dir(None) is None


def test_fallback_when_not_in_a_workspace(tmp_path):
    p = dbp.default_db_path(share_dir='/opt/ros/jazzy/share/diagnostic_monitor', home=str(tmp_path))
    assert p == str(tmp_path / '.ros' / 'rca' / 'events.db')
    dbp.ensure_parent_dir(p)
    assert (tmp_path / '.ros' / 'rca').is_dir()


def test_default_is_independent_of_cwd(tmp_path, monkeypatch):
    monkeypatch.delenv(dbp.ENV_VAR, raising=False)
    monkeypatch.chdir(tmp_path)
    a = dbp.resolve_db_path()
    monkeypatch.chdir(tmp_path.parent)
    b = dbp.resolve_db_path()
    assert a == b and os.path.isabs(a) and a.endswith('events.db')
    assert not a.startswith(str(tmp_path))          # never <cwd>/events.db


def test_default_points_into_installed_workspace():
    """When running from a colcon workspace the default is <ws>/events.db."""
    share = dbp._package_share_dir()
    if share is None or dbp.workspace_root_from_share_dir(share) is None:
        pytest.skip('package not installed in a colcon workspace')
    ws = dbp.workspace_root_from_share_dir(share)
    assert dbp.resolve_db_path(env={}) == os.path.join(ws, 'events.db')
    assert os.path.isdir(os.path.join(ws, 'install'))


def test_cli_reads_monitor_default_from_any_cwd(tmp_path, monkeypatch, capsys):
    """Simulates the original bug: monitor writes to the resolved default, CLI is
    started from ~ (a different cwd) with no --db and must read the same file."""
    from diagnostic_monitor import rca_cli
    db = tmp_path / 'ws' / 'events.db'
    db.parent.mkdir()
    monkeypatch.setenv(dbp.ENV_VAR, str(db))            # deterministic default for the test
    # "monitor" side: opens the default and writes
    store = EventStore(dbp.resolve_db_path(ros_param=''))
    store.insert_anomaly(1.0, 'sensor_node', '/t', 'm', 1, 0, 1, 3, 0.5, 'x')
    store.insert_diagnosis(1.5, 'sensor_node', 0.9, 0.9, {}, [], 'e')
    store.close()
    # "CLI" side: different cwd, no --db
    other = tmp_path / 'home'
    other.mkdir()
    monkeypatch.chdir(other)
    monkeypatch.setattr(sys, 'argv', ['rca_cli', '--action', 'summary'])
    rca_cli.main()
    out = capsys.readouterr().out
    assert f'[db: {db}]' in out
    assert 'Detected Anomalies: 1' in out and 'Diagnoses:          1' in out
    assert not (other / 'events.db').exists()          # no stray DB created in cwd


def test_cli_refuses_to_create_empty_store(tmp_path, monkeypatch, capsys):
    from diagnostic_monitor import rca_cli
    monkeypatch.setenv(dbp.ENV_VAR, str(tmp_path / 'missing.db'))
    monkeypatch.setattr(sys, 'argv', ['rca_cli', '--action', 'summary'])
    rc = rca_cli.main()
    assert rc == 2
    assert 'No event store found' in capsys.readouterr().out
    assert not (tmp_path / 'missing.db').exists()


def test_cli_explicit_db_override(tmp_path, monkeypatch, capsys):
    from diagnostic_monitor import rca_cli
    db = tmp_path / 'explicit.db'
    EventStore(str(db)).close()
    monkeypatch.setenv(dbp.ENV_VAR, str(tmp_path / 'env.db'))
    monkeypatch.setattr(sys, 'argv', ['rca_cli', '--db', str(db), '--action', 'summary'])
    rca_cli.main()
    assert f'[db: {db}]' in capsys.readouterr().out
