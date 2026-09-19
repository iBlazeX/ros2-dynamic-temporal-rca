# Dynamic Dependency-Aware Temporal Root-Cause Analysis for ROS 2 Robotic Systems

Review 1 prototype (ROS 2 Jazzy, Ubuntu 24.04, Python 3.12, WSL2).

Pipeline under test: `sensor_node → perception_node → localization_node → navigation_node`
(a controlled experimental system, not a real robot stack).

End-to-end flow that the prototype demonstrates:

```
launch system → discover graph → monitor normal operation → inject fault
→ observe cascading symptoms → diagnose probable root cause → show propagation chain + evidence
```

## Core formulation

Candidate root causes `C` are ranked with

```
R(C) = wD·D(C) + wT·T(C) + wS·S(C) + wA·A(C)        (default wD=0.30 wT=0.30 wS=0.25 wA=0.15)
```

| Term | Meaning | Definition |
|---|---|---|
| `D(C)` dependency | rewards upstream candidates that reach many symptoms and have no anomalous upstream parents | `|Desc(C)∩A| / max(1,|A|−[C∈A]) · (1 − |Anc(C)∩A| / |A|)` |
| `T(C)` temporal | earliest anomaly *onset* in the window (evidence, not proof of causality) | `max(0, 1 − (t_first(C) − t_min) / window)`; candidates with no direct anomaly get 0.05 |
| `S(C)` symptom coverage | fraction of anomalous nodes explained by `C` | `|(Desc(C) ∪ {C}) ∩ A| / |A|` |
| `A(C)` severity | max normalised severity on `C` (z-score / crash) | `min(1, z / 3·z_thr)` |

`A` = set of anomalous nodes in the sliding RCA window (6 s). Candidates = anomalous nodes ∪ their
ancestors in the **discovered** graph. Setting any weight to `0.0` disables that term (ablation).

### Research constraints honoured

1. **Ground truth isolation** – `diagnostic_monitor` never subscribes to `/rca/fault_command`; the graph
   discoverer excludes `fault_injector` / `experiment_controller` and `/rca/*` topics. Ground truth exists
   only in `experiment_controller.py` (evaluation layer).
2. **No hard-coded topology** – `graph_engine.RosGraphDiscoverer` builds `G=(V,E)` from
   `get_node_names_and_namespaces`, `get_topic_names_and_types`, `get_publishers_info_by_topic`,
   `get_subscriptions_info_by_topic`. Message types are resolved dynamically; metrics are duck-typed on
   message fields. The monitor stores every discovered graph in the DB (`rca_cli --action graph`).
3. **Temporal order is evidence** – explanations say so explicitly; ties are resolved by `D`/`S`.
4. **Real delays** – latency faults use real wall-clock publication delay (queued publish) or
   `time.sleep` in callbacks. Downstream nodes retain the source acquisition `header.stamp` (standard ROS
   practice), so end-to-end pipeline delay is a physically measured quantity on every hop.
5. No ML / LLM / recovery. Deterministic, interpretable scoring.

## Packages

```
src/rca_test_system/          target pipeline + fault injection (+ evaluation layer)
  sensor_node.py              LaserScan @10 Hz; faults: latency, dropout, degradation (NaNs), crash
  perception_node.py          PoseArray; faults: processing_delay, dropout, degradation, crash
  localization_node.py        PoseStamped; faults: localization_failure (jump), latency, crash
  navigation_node.py          TwistStamped; faults: navigation_failure, latency, crash
  fault_injector.py           CLI: publishes JSON fault commands on /rca/fault_command
  experiment_controller.py    benchmark: scenarios, ground truth, metrics, baselines, ablation, JSON export
  launch/{pipeline,system}.launch.py, config/rca_params.yaml
src/diagnostic_monitor/       RCA engine (no ground-truth access)
  graph_engine.py             dynamic graph discovery, descendants/ancestors, stale-edge retention for crashed nodes
  anomaly_detector.py         running mean/std baselines with variance floor, warm-up, 2-sample confirmation,
                              starvation timeouts, node-crash detection, duplicate / out-of-order guards
  event_store.py              SQLite (WAL, batched telemetry): telemetry, anomalies, diagnoses, graph snapshots
  rca_engine.py               candidate generation + R(C) scoring + deterministic ranking
  explainer.py                propagation chain (BFS over discovered edges) + evidence narrative
  evaluator.py                Top-1/Top-k, MRR, detection/diagnosis latency, false-diagnosis rate, chain accuracy
  baselines.py                5 comparison methods + ablation configs (offline, same evidence)
  monitor_node.py             ROS 2 node tying it together; rca_cli.py inspection CLI
scripts/system.sh             start/stop/status helper for the full system
runs/                         logs, archived DBs and results_*.json from real runs
```

### Anomaly evidence model

Every anomaly carries `timestamp` (record time, used for the 6 s sliding window) and `onset_time`
(physical start of the deviation, used for temporal ordering):

* statistical: onset = first sample of the confirmed outlier streak (2 consecutive samples with
  `z > 3` **and** an absolute deviation above a per-metric physical floor – single scheduling hiccups
  are never evidence);
* timeout: onset = last message time + threshold (`max(0.45 s, 3·nominal period)`), re-emitted while
  starvation persists;
* crash: onset = last time the node was seen alive; re-emitted every 2 s while the node stays absent
  (ROS 2 graph only drops a dead participant after the DDS liveliness lease, ~20 s with Fast DDS).

## Build & test

```bash
cd ~/ros2_rca_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
python3 -m pytest src/diagnostic_monitor/tests src/rca_test_system/tests -q     # 38 tests
```

## Review 1 demo

Terminal 1 – launch the whole system (4 pipeline nodes + monitor):

```bash
cd ~/ros2_rca_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch rca_test_system system.launch.py
```

(or in the background: `scripts/system.sh start`, `scripts/system.sh stop`)

Terminal 2 – show what the monitor discovered and that it is quiet:

```bash
cd ~/ros2_rca_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 run diagnostic_monitor rca_cli --action graph
ros2 run diagnostic_monitor rca_cli --action summary        # 0 anomalies / 0 diagnoses when nominal
ros2 run diagnostic_monitor rca_cli --action watch          # live: prints whenever the diagnosis changes
```

Terminal 3 – Experiment 1: sensor fault → `sensor_node` is the probable root cause

```bash
cd ~/ros2_rca_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 run rca_test_system fault_injector --target sensor_node --type latency --param 0.40 --duration 10
sleep 3 && ros2 run diagnostic_monitor rca_cli --action latest
```

Experiment 2 (after ~15 s so the window is clean): localization fault → `localization_node`

```bash
ros2 run rca_test_system fault_injector --target localization_node --type localization_failure --param 50 --duration 10
sleep 3 && ros2 run diagnostic_monitor rca_cli --action latest
```

Other faults: `--type dropout|degradation|processing_delay|navigation_failure|crash`,
`--target perception_node|navigation_node`. `rca_cli --action timeline --last 30` prints the ordered
anomaly / diagnosis timeline; `--action anomalies` lists raw evidence.

### Automated benchmark (all scenarios, metrics, baselines, ablation)

With the system running (Terminal 1):

```bash
ros2 run rca_test_system experiment_controller --warmup 15 --out runs/results.json
ros2 run rca_test_system experiment_controller --warmup 15 --include-crash   # adds the destructive crash scenario last
```

Scenarios: sensor latency, localization failure, sensor dropout, navigation failure, sensor degradation,
perception processing delay, (perception crash). For each scenario the controller reports Top-1 /
Top-k, rank, MRR, detection latency (first anomaly after injection), diagnosis latency (first diagnosis),
false-diagnosis rate (wrong diagnoses among all issued after injection), propagation-chain accuracy
(Jaccard vs expected affected set + order check) and nominal false alarms. It then re-ranks the *same
recorded evidence* with the five baselines (independent diagnostics, temporal-only, static dependency +
anomaly, static dependency + temporal, proposed dynamic + temporal + symptom + anomaly) and with each
scoring term ablated. Results are written to JSON.

## Results from a real run (`runs/results_run5.json`, live ROS 2 Jazzy on WSL2)

Full transcript: `runs/bench_run5.log`. 15 s fault-free warm-up: 0 false alarms.

| Scenario | Truth | Predicted | Top-1 | Det. lat. | Diag. lat. | FDR | Chain acc. |
|---|---|---|---|---|---|---|---|
| 1 Sensor latency (0.4 s) | sensor_node | sensor_node | PASS | 0.53 s | 0.79 s | 0.0 | 1.0 |
| 2 Localization failure | localization_node | localization_node | PASS | 0.11 s | 0.16 s | 0.0 | 1.0 |
| 3 Sensor dropout | sensor_node | sensor_node | PASS | 0.54 s | 0.54 s | 0.0 | 1.0 |
| 4 Navigation failure | navigation_node | navigation_node | PASS | 0.16 s | 0.42 s | 0.0 | 1.0 |
| 5 Sensor degradation (NaNs) | sensor_node | sensor_node | PASS | 0.14 s | 0.30 s | 0.0 | 1.0 |
| 6 Perception processing delay | perception_node | perception_node | PASS | 0.82 s | 1.18 s | 0.0 | 1.0 |
| 7 Perception crash | perception_node | perception_node | PASS | 0.56 s | 0.56 s | 0.0 | 1.0 |

Aggregate: Top-1 = Top-3 = 100 %, MRR = 1.0, mean detection latency 0.41 s, mean diagnosis latency
0.56 s, false-diagnosis rate 0.0 (over 81 diagnoses issued after injection), chain accuracy 1.0.

Baselines on identical evidence (Top-1): independent 2/7 (MRR 0.52) · temporal-only 7/7 ·
static-dep+anomaly 7/7 · static-dep+temporal 7/7 · proposed 7/7. Ablation: every single-term removal
still gives 7/7 on these scenarios (the scenarios are not adversarial: the true root is always
earliest, most upstream and most severe; see limitations).

## Known limitations

* Only rate, end-to-end latency and simple payload-validity metrics are monitored; semantic symptoms
  (e.g. a frozen pose after upstream loss) are invisible, so the chain for `sensor degradation` ends at
  `perception_node`.
* Graph-level crash detection is bounded by the DDS liveliness lease (~20 s); the crash is diagnosed
  earlier through downstream starvation timeouts.
* In the current 4-node chain the runtime graph equals the design-time graph, so the static-graph
  baselines match the proposed method on the live scenarios; the advantage of dynamic discovery is shown
  in `tests/test_baselines_evaluator.py` (runtime topology differing from the static one).
* Sub-10 ms onset differences between hops are within scheduling noise; temporal order is reported as
  supporting evidence only.
