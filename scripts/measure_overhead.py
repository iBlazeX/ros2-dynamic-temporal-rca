#!/usr/bin/env python3
"""Process-level overhead measurement for the RCA stack (both simulator backends).

Measurement method
------------------
CPU is measured from /proc/<pid>/stat: (utime+stime) is read at the start and at
the end of the sampling window and divided by the elapsed wall time and the
kernel clock tick. This is the CPU actually consumed during the window. It is
NOT `ps %cpu`, which reports an average over the whole process lifetime and
would be dominated by start-up cost. RSS is sampled once per second from
/proc/<pid>/statm and reported as the mean and the maximum.

CPU is reported three ways so "85 %" is never ambiguous:
  cpu_percent_sum      raw aggregate over the group's processes; one fully busy
                       core is 100 %, so this can exceed 100 % on a multi-core host
  core_equivalents     cpu_percent_sum / 100  (how many cores' worth of work)
  percent_of_machine   cpu_percent_sum / (100 * cores)  (share of total capacity)

Process uniqueness
------------------
Every group is resolved to explicit PIDs, which are recorded in the artifact.
`--expect group=count` fails the measurement when the count does not match, so a
stray second monitor or TUI cannot silently contaminate the numbers. Use
`--list` to see what is currently running before measuring.

    python3 scripts/measure_overhead.py --list
    python3 scripts/measure_overhead.py --label gz_monitor_tui --duration 30 \
        --expect diagnostic_monitor=1 --expect tui=1 --out runs/gazebo/overhead.json

Results are appended to the JSON file (one entry per label).
"""

import argparse
import json
import os
import re
import sqlite3
import subprocess
import sys
import time

CLK_TCK = os.sysconf('SC_CLK_TCK')
PAGE_KB = os.sysconf('SC_PAGE_SIZE') / 1024.0

GROUPS = {
    # rca_sim backend: the world/sensor process and the controlled rca_test_system rig
    'rca_sim': r'rca_sim/lib/rca_sim/sim_world|rca_test_system/lib/rca_test_system/',
    # Gazebo backend: the gz server, the ros_gz bridge and the rca_gazebo sensor drivers
    'gazebo': r'gz sim |gz-sim-server|ros_gz_bridge/parameter_bridge|rca_gazebo/lib/rca_gazebo/',
    # Processing pipeline, shared verbatim by both backends
    'pipeline': r'rca_sim/lib/rca_sim/(perception|localization|planning|control)_node',
    'diagnostic_monitor': r'diagnostic_monitor/lib/diagnostic_monitor/monitor_node',
    'tui': r'diagnostic_tui/lib/diagnostic_tui/rca_tui',
}
# Backwards-compatible alias: older artifacts used "simulation" for the rca_sim group.
# The evaluation layer (scenario runner / experiment controller / this script)
# is never part of the measured system overhead.
EXCLUDE = r"scenario_runner|experiment_controller|measure_overhead"

GROUP_ALIASES = {'simulation': 'rca_sim'}


def _cmdlines():
    """pid -> command line for every readable process, excluding this one."""
    out = {}
    me = os.getpid()
    for pid in os.listdir('/proc'):
        if not pid.isdigit() or int(pid) == me:
            continue
        try:
            with open(f'/proc/{pid}/cmdline', 'rb') as f:
                cmd = f.read().replace(b'\0', b' ').decode(errors='ignore').strip()
        except OSError:
            continue
        if cmd:
            out[int(pid)] = cmd
    return out


def _cpu_ticks(pid):
    try:
        with open(f'/proc/{pid}/stat') as f:
            fields = f.read().rsplit(')', 1)[1].split()
        return int(fields[11]) + int(fields[12])      # utime + stime (after comm/state)
    except (OSError, IndexError, ValueError):
        return None


def _rss_mb(pid):
    try:
        with open(f'/proc/{pid}/statm') as f:
            return int(f.read().split()[1]) * PAGE_KB / 1024.0
    except (OSError, IndexError, ValueError):
        return None


def find_groups():
    """Returns {group: [(pid, cmdline), ...]} for the currently running processes."""
    cmds = _cmdlines()
    found = {g: [] for g in GROUPS}
    for pid, cmd in sorted(cmds.items()):
        if re.search(EXCLUDE, cmd):
            continue          # evaluation layer, not part of the measured system
        for g, pat in GROUPS.items():
            if re.search(pat, cmd):
                found[g].append((pid, cmd))
                break
    return found


def sample_groups(duration, interval=1.0):
    """CPU from /proc stat deltas over the window; RSS sampled every `interval`."""
    groups = find_groups()
    pids = {g: [p for p, _ in v] for g, v in groups.items()}
    t0 = time.time()
    start = {p: _cpu_ticks(p) for v in pids.values() for p in v}
    rss = {p: [] for p in start}
    n = max(1, int(duration / interval))
    for _ in range(n):
        time.sleep(interval)
        for p in rss:
            m = _rss_mb(p)
            if m is not None:
                rss[p].append(m)
    elapsed = time.time() - t0
    end = {p: _cpu_ticks(p) for p in start}

    cores = os.cpu_count() or 1
    out = {}
    for g, plist in pids.items():
        procs = []
        for p in plist:
            if start.get(p) is None or end.get(p) is None:
                continue                                   # process ended mid-window
            cpu = (end[p] - start[p]) / CLK_TCK / elapsed * 100.0
            samples = rss[p] or [0.0]
            procs.append({
                'pid': p,
                'cmd': dict(groups[g])[p][:120],
                'cpu_percent': round(cpu, 1),
                'rss_mb_mean': round(sum(samples) / len(samples), 1),
                'rss_mb_max': round(max(samples), 1),
            })
        total_cpu = sum(x['cpu_percent'] for x in procs)
        out[g] = {
            'processes': len(procs),
            'pids': [x['pid'] for x in procs],
            'cpu_percent_sum': round(total_cpu, 1),
            'core_equivalents': round(total_cpu / 100.0, 2),
            'percent_of_machine': round(total_cpu / (100.0 * cores) * 100.0, 1),
            'rss_mb_sum_mean': round(sum(x['rss_mb_mean'] for x in procs), 1),
            'rss_mb_sum_max': round(sum(x['rss_mb_max'] for x in procs), 1),
            'per_process': procs,
        }
        # aliases kept so older readers of the artifact keep working
        out[g]['cpu_percent_mean'] = out[g]['cpu_percent_sum']
        out[g]['rss_mb_mean'] = out[g]['rss_mb_sum_mean']
    for alias, target in GROUP_ALIASES.items():
        out[alias] = out[target]
    out['_window_sec'] = round(elapsed, 2)
    return out


def machine_info():
    model, cores = '', os.cpu_count() or 1
    try:
        for line in open('/proc/cpuinfo'):
            if line.startswith('model name'):
                model = line.split(':', 1)[1].strip()
                break
    except OSError:
        pass
    try:
        load1, load5, load15 = os.getloadavg()
    except OSError:
        load1 = load5 = load15 = None
    mem_total_mb = None
    try:
        for line in open('/proc/meminfo'):
            if line.startswith('MemTotal'):
                mem_total_mb = round(int(line.split()[1]) / 1024.0)
                break
    except OSError:
        pass
    return {'logical_cores': cores, 'cpu_model': model, 'mem_total_mb': mem_total_mb,
            'load_avg_1_5_15': [load1, load5, load15],
            'note': 'cpu_percent_sum is raw process CPU where one busy core = 100 %; '
                    f'the machine has {cores} logical cores, i.e. {100 * cores} % capacity'}


def db_rates(db, duration):
    if not os.path.exists(db):
        return None
    c = sqlite3.connect(db)

    def counts():
        return tuple(c.execute(f'select count(*) from {t}').fetchone()[0]
                     for t in ('telemetry_events', 'anomalies', 'diagnoses'))

    a = counts()
    time.sleep(duration)
    b = counts()
    size = os.path.getsize(db) + (os.path.getsize(db + '-wal') if os.path.exists(db + '-wal') else 0)
    return {'window_sec': duration,
            'telemetry_rows_per_sec': round((b[0] - a[0]) / duration, 1),
            'anomaly_rows_per_sec': round((b[1] - a[1]) / duration, 2),
            'diagnosis_rows_per_sec': round((b[2] - a[2]) / duration, 2),
            'db_size_mb': round(size / 1e6, 2)}


def dashboard_rate(duration):
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String
    except ImportError:
        return None
    rclpy.init()
    n = Node('overhead_probe')
    got = []
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
    n.create_subscription(String, '/rca/dashboard_state', lambda m: got.append(len(m.data)), qos)
    t0 = time.time()
    while time.time() - t0 < duration:
        rclpy.spin_once(n, timeout_sec=0.2)
    n.destroy_node()
    rclpy.shutdown()
    return {'window_sec': duration, 'messages_per_sec': round(len(got) / duration, 2),
            'mean_bytes': int(sum(got) / len(got)) if got else 0}


def rca_compute_time(db):
    """Times RCAEngine.diagnose + Explainer on the densest recorded evidence window."""
    try:
        from diagnostic_monitor.anomaly_detector import Anomaly
        from diagnostic_monitor.explainer import Explainer
        from diagnostic_monitor.graph_engine import DependencyGraph
        from diagnostic_monitor.rca_engine import RCAEngine
    except ImportError:
        return None
    if not os.path.exists(db):
        return None
    c = sqlite3.connect(db)
    c.row_factory = sqlite3.Row
    snap = c.execute('select graph_json from graph_snapshots order by id desc limit 1').fetchone()
    if not snap:
        return None
    graph = DependencyGraph.from_dict(json.loads(snap[0]))
    rows = [dict(r) for r in c.execute('select * from anomalies order by timestamp')]
    if not rows:
        return {'note': 'no anomalies recorded yet'}
    best, best_i = 0, 0
    ts = [r['timestamp'] for r in rows]
    j = 0
    for i in range(len(ts)):
        while ts[i] - ts[j] > 6.0:
            j += 1
        if i - j + 1 > best:
            best, best_i = i - j + 1, i
    window = [Anomaly.from_dict(r) for r in rows[max(0, best_i - best + 1):best_i + 1]]
    engine = RCAEngine()
    reps = 20
    t0 = time.perf_counter()
    for _ in range(reps):
        d = engine.diagnose(window, graph, window[-1].timestamp + 0.1)
        Explainer.explain(d, graph, window)
    per = (time.perf_counter() - t0) / reps * 1000.0
    return {'anomalies_in_window': len(window), 'graph_nodes': len(graph.nodes),
            'repetitions': reps, 'ms_per_diagnosis': round(per, 2)}


def print_listing():
    groups = find_groups()
    for g, procs in groups.items():
        print(f'{g:<20} {len(procs)} process(es)')
        for pid, cmd in procs:
            print(f'    {pid:>8}  {cmd[:110]}')
    return groups


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--label')
    ap.add_argument('--duration', type=float, default=30.0)
    ap.add_argument('--db', default=None)
    ap.add_argument('--out', default='runs/simulation/overhead.json')
    ap.add_argument('--list', action='store_true', help='list matching processes and exit')
    ap.add_argument('--expect', action='append', default=[], metavar='GROUP=N',
                    help='required process count for a group; the run fails if it differs')
    a = ap.parse_args()

    if a.list:
        print_listing()
        return 0
    if not a.label:
        ap.error('--label is required unless --list is given')

    groups = find_groups()
    problems = []
    for spec in a.expect:
        g, _, n = spec.partition('=')
        g = GROUP_ALIASES.get(g, g)
        if g not in groups:
            problems.append(f'unknown group {g}')
            continue
        if len(groups[g]) != int(n):
            problems.append(f'{g}: expected {n} process(es), found {len(groups[g])} '
                            f'-> {[p for p, _ in groups[g]]}')
    if problems:
        print('PROCESS UNIQUENESS CHECK FAILED (measurement would be contaminated):', file=sys.stderr)
        for p in problems:
            print('  ' + p, file=sys.stderr)
        print_listing()
        return 2

    if a.db is None:
        try:
            from diagnostic_monitor.db_path import resolve_db_path
            a.db = resolve_db_path()
        except ImportError:
            a.db = 'events.db'

    print(f'[{a.label}] sampling {a.duration:.0f}s ...')
    procs = sample_groups(a.duration)
    entry = {
        'label': a.label, 'timestamp': time.time(), 'duration_sec': a.duration,
        'method': 'CPU from /proc/<pid>/stat utime+stime deltas over the window; '
                  'RSS sampled every 1 s from /proc/<pid>/statm',
        'machine': machine_info(),
        'processes': procs,
    }
    if procs['diagnostic_monitor']['processes'] > 0:
        entry['event_store'] = db_rates(a.db, min(15.0, a.duration))
        entry['dashboard_topic'] = dashboard_rate(min(15.0, a.duration))
        entry['rca_compute'] = rca_compute_time(a.db)
    data = []
    if os.path.exists(a.out):
        try:
            data = json.load(open(a.out))
        except ValueError:
            data = []
    data.append(entry)
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    json.dump(data, open(a.out, 'w'), indent=2)
    print(json.dumps({k: v for k, v in entry.items() if k != 'processes'}, indent=2))
    for g in ('rca_sim', 'gazebo', 'pipeline', 'diagnostic_monitor', 'tui'):
        p = procs[g]
        print(f'  {g:<20} {p["processes"]}p  cpu {p["cpu_percent_sum"]:6.1f}% raw '
              f'({p["core_equivalents"]:.2f} cores, {p["percent_of_machine"]:.1f}% of machine)  '
              f'rss {p["rss_mb_sum_mean"]:7.1f} MB  pids {p["pids"]}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
