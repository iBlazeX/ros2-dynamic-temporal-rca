#!/usr/bin/env bash
# Probes each degradation mechanism against the running simulation + monitor.
# usage: probe_faults.sh "<target> <type> <param> <duration> <observe_sec>" ...
cd ~/ros2_rca_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash
DB=events.db
for spec in "$@"; do
  set -- $spec
  target=$1; type=$2; param=$3; dur=$4; obs=$5
  # wait until quiet (no diagnosis in the last 2 s), max 25 s
  for i in $(seq 1 50); do
    q=$(python3 -c "import sqlite3,time;c=sqlite3.connect('$DB');print(c.execute('select count(*) from diagnoses where timestamp>?',(time.time()-2.0,)).fetchone()[0])")
    [ "$q" = "0" ] && break; sleep 0.5
  done
  t0=$(python3 -c "import time;print(time.time())")
  ros2 run rca_test_system fault_injector --target $target --type $type --param $param --duration $dur >/dev/null 2>&1
  sleep $obs
  python3 - "$target" "$type" "$t0" <<'EOF'
import sqlite3, sys, time, json
target, ftype, t0 = sys.argv[1], sys.argv[2], float(sys.argv[3])
c = sqlite3.connect('events.db'); c.row_factory = sqlite3.Row
an = c.execute("select node, metric, min(timestamp)-? as first from anomalies where timestamp>? group by node, metric order by first", (t0, t0)).fetchall()
dg = c.execute("select timestamp, root_cause, breakdown_json from diagnoses where timestamp>? order by id", (t0,)).fetchall()
roots = [d['root_cause'] for d in dg]
print(f"=== {target} {ftype}: first anomaly +{an[0]['first']:.2f}s" if an else f"=== {target} {ftype}: NO ANOMALIES", end='')
print(f" | first diag +{dg[0]['timestamp']-t0:.2f}s | roots: {sorted(set(roots))} | final={roots[-1] if roots else None}" if dg else " | no diagnosis")
for a in an: print(f"    +{a['first']:5.2f}s {a['node']:<18} {a['metric']}")
if dg:
    bd = json.loads(dg[-1]['breakdown_json']); print("    chain:", ' -> '.join(bd.get('propagation_chain', [])))
EOF
  ros2 run rca_test_system fault_injector --target all --type clear >/dev/null 2>&1
done
