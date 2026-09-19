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
scripts/verify_results.py     re-derives a run's reported numbers from its SQLite store (log == JSON == DB)
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
python3 -m pytest src/diagnostic_monitor/tests src/rca_test_system/tests -q     # 43 tests
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
ros2 run rca_test_system experiment_controller --warmup 15 --soak 30 --out runs/results.json
ros2 run rca_test_system experiment_controller --warmup 15 --soak 30 --include-crash   # adds the destructive crash scenario last
```

The run starts with a fault-free **warm-up** (`--warmup`) and a fault-free **nominal soak** (`--soak`,
default 30 s) in the same execution; every fault-free window (warm-up, soak, and the short pre-injection
gaps) is recorded in the artifact with its duration, anomaly count and diagnosis count. Any diagnosis
issued in a fault-free window is a false alarm (`nominal_false_alarms`, `nominal_false_alarms_per_min`).
The printed aggregate and the saved `aggregate` in the JSON come from one function
(`ExperimentController.compute_aggregate`); `tests/test_experiment_report.py` fails if they drift.
`scripts/verify_results.py <bench.log> <results.json> <events.db>` re-derives every reported number
from the SQLite store and checks the three agree.

Scenarios: sensor latency, localization failure, sensor dropout, navigation failure, sensor degradation,
perception processing delay, (perception crash). For each scenario the controller reports Top-1 /
Top-k, rank, MRR, detection latency (first anomaly after injection), diagnosis latency (first diagnosis),
false-diagnosis rate (wrong diagnoses among all issued after injection), propagation-chain accuracy
(Jaccard vs expected affected set + order check) and nominal false alarms. It then re-ranks the *same
recorded evidence* with the five baselines (independent diagnostics, temporal-only, static dependency +
anomaly, static dependency + temporal, proposed dynamic + temporal + symptom + anomaly) and with each
scoring term ablated. Results are written to JSON.

## Results from a real run (`runs/results_run6.json`, live ROS 2 Jazzy on WSL2)

Full transcript: `runs/bench_run6.log`; event store `runs/events_run6.db`; verified with
`scripts/verify_results.py` (printed summary == JSON == DB recount).

Fault-free (nominal) observation in the same run: warm-up 15.0 s + soak 30.0 s + pre-injection gaps
4.6 s = **49.6 s, 0 anomalies, 0 diagnoses → 0 false alarms (0.0 / min)**.

| Scenario | Truth | Predicted | Top-1 | Det. lat. | Diag. lat. | FDR | Chain acc. (observable) | Physical coverage |
|---|---|---|---|---|---|---|---|---|
| 1 Sensor latency (0.4 s) | sensor_node | sensor_node | PASS | 0.58 s | 0.67 s | 0.0 | 1.0 | 1.0 |
| 2 Localization failure | localization_node | localization_node | PASS | 0.15 s | 0.54 s | 0.0 | 1.0 | 1.0 |
| 3 Sensor dropout | sensor_node | sensor_node | PASS | 0.42 s | 0.42 s | 0.0 | 1.0 | 1.0 |
| 4 Navigation failure | navigation_node | navigation_node | PASS | 0.11 s | 0.30 s | 0.0 | 1.0 | 1.0 |
| 5 Sensor degradation (NaNs) | sensor_node | sensor_node | PASS | 0.19 s | 0.68 s | 0.0 | 1.0 | **0.67** (localization_node affected but unobserved) |
| 6 Perception processing delay | perception_node | perception_node | PASS | 0.87 s | 1.06 s | 0.0 | 1.0 | 1.0 |
| 7 Perception crash | perception_node | perception_node | PASS | 0.43 s | 0.43 s | 0.0 | 1.0 | 1.0 |

Aggregate: Top-1 = Top-3 = 100 %, MRR = 1.0, mean detection latency 0.39 s, mean diagnosis latency
0.59 s, false-diagnosis rate 0.0 (over 83 diagnoses issued after injection), observable chain accuracy
1.0, physical chain coverage 0.95.

**Observable vs physical chains.** Chain accuracy is scored against the *observable* affected set (nodes
whose failure the monitor's metrics can see). Where the physically affected set is larger, it is declared
separately as `physical_chain` in the scenario definition and reported as `physical_chain_coverage` with
the unobserved nodes listed; the prediction is never credited for them. In scenario 5 localization's
pose freezes but no monitored metric changes, so it is reported as *physically affected, unobserved*,
not as detected.

Baselines on identical evidence (Top-1): independent 2/7 (MRR 0.52) · temporal-only 7/7 ·
static-dep+anomaly 7/7 · static-dep+temporal 7/7 · proposed 7/7. Ablation: every single-term removal
still gives 7/7 on these scenarios (the scenarios are not adversarial: the true root is always
earliest, most upstream and most severe; see limitations).

## Known limitations

* Only rate, end-to-end latency and simple payload-validity metrics are monitored; semantic symptoms
  (e.g. a frozen pose after upstream loss) are invisible. This is made explicit through the
  observable-vs-physical chain reporting (scenario 5: physical coverage 0.67).
* Graph-level crash detection is bounded by the DDS liveliness lease (~20 s); the crash is diagnosed
  earlier through downstream starvation timeouts.
* In the current 4-node chain the runtime graph equals the design-time graph, so the static-graph
  baselines match the proposed method on the live scenarios; the advantage of dynamic discovery is shown
  in `tests/test_baselines_evaluator.py` (runtime topology differing from the static one).
* Sub-10 ms onset differences between hops are within scheduling noise; temporal order is reported as
  supporting evidence only.
