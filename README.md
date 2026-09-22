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

## Architecture

```
ROS 2 runtime (sensor -> perception -> localization -> navigation, discovered at runtime)
      |  application topics only
diagnostic_monitor (Python)   graph discovery . anomaly detection . temporal evidence . RCA . explanation
      |                                   |                               |
   SQLite events.db              /rca/diagnosis_report            /rca/dashboard_state   (JSON, latched, <= 2 Hz)
   (persistent evidence:         (unchanged)                                |
    CLI, experiments, replay)                                   diagnostic_tui / rca_tui  (C++17 + FTXUI)
                                                                 visualisation only: parses and renders,
                                                                 contains no RCA logic
```

The diagnostic monitor is the single source of truth. The TUI never reads SQLite and never
recomputes anything: it subscribes to `/rca/dashboard_state` (schema documented in
`diagnostic_monitor/dashboard_state.py`), validates `schema_version`, keeps the latest state behind a
mutex (ROS executor thread -> UI thread) and redraws. The dashboard topic carries no ground truth
because the monitor has none.

## Packages

```
src/diagnostic_tui/           C++17 FTXUI dashboard: `ros2 run diagnostic_tui rca_tui`
  src/dashboard_state.cpp     JSON parsing (nlohmann), schema check, thread-safe state, DAG layering, view models
  src/render.cpp              header / dynamic graph / diagnosis / timeline panels
  src/rca_tui_main.cpp        rclcpp subscriber thread + FTXUI loop
  test/test_dashboard_state.cpp   gtest: parsing, defaults, invalid/unsupported input, stale state, layout, rendering
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
  dashboard_state.py          builds the /rca/dashboard_state JSON (schema v1) from the live monitor state
  db_path.py                  single DB-path resolver shared by monitor, CLI and experiment controller
src/rca_sim/                  custom 2-D kinematic simulation backend (world, sensors, pipeline, scenarios)
src/rca_gazebo/               Gazebo Harmonic simulation backend
  worlds/rca_arena.sdf        generated arena + differential-drive robot with gpu_lidar / camera / IMU
  scripts/gen_world.py        regenerates the world from rca_sim's arena so both backends share the scene
  config/bridge.yaml          every ros_gz_bridge mapping (all raw topics live under /gz)
  config/scenarios.yaml       18 Gazebo scenarios with evaluation-only ground truth
  rca_gazebo/gz_drivers.py    lidar/camera/imu/odometry driver nodes + the /cmd_vel actuation sink
  rca_gazebo/scenario_runner.py  Gazebo benchmark (reuses the rca_sim runner and the shared evaluator)
  launch/gazebo_sim.launch.py, launch/gazebo_system.launch.py
scripts/system.sh             start/stop/status helper for the controlled rca_test_system
scripts/sim.sh                start/stop/status helper for the rca_sim backend
scripts/gazebo_sim.sh         start/stop/status/topics helper for the Gazebo backend
scripts/verify_results.py     re-derives a run's reported numbers from its SQLite store (log == JSON == DB)
scripts/measure_overhead.py   per-process CPU (/proc deltas) + RSS, SQLite/dashboard rates, RCA compute time
scripts/graph_probe.py        runtime dynamic-graph validation + DDS lease measurement (either backend)
scripts/tui_demo.sh           captures live TUI frames while scenarios run (either backend)
scripts/inspect_db.py         prints the monitor's discovered graph and recorded events
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

Requirements: ROS 2 Jazzy, g++ >= 13, CMake >= 3.22, `nlohmann-json3-dev` (Ubuntu package),
network access on the first build of `diagnostic_tui` (FTXUI is fetched from a pinned commit).

```bash
sudo apt install nlohmann-json3-dev            # once
cd ~/ros2_rca_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
python3 -m pytest src/diagnostic_monitor/tests src/rca_test_system/tests src/rca_sim/tests -q   # 70 Python tests
colcon test && colcon test-result --verbose                                     # + 10 C++ gtests
```

FTXUI is pinned in `src/diagnostic_tui/CMakeLists.txt` to tag **v5.0.0**
(commit `cdf28903a7781f97ba94d30b79c3a4b0c97ccce7`) and linked statically; nothing is installed
system-wide. Offline builds: clone that tag and pass `--cmake-args -DFTXUI_SOURCE_DIR=/path/to/FTXUI`.

## Event store (SQLite) path

All tools resolve the same default, independent of the current directory
(`diagnostic_monitor/db_path.py`):

1. explicit `--db PATH` (CLI / experiment controller)
2. explicit ROS parameter `db_path` on the monitor (launch arg `db_path:=...`; empty = auto)
3. environment variable `RCA_DB_PATH`
4. `<colcon workspace>/events.db`, the workspace being derived from where `diagnostic_monitor` is
   installed (`<ws>/install/diagnostic_monitor/...`) -> `~/ros2_rca_ws/events.db` here
5. `~/.ros/rca/events.db` if the package is not installed inside a colcon workspace

So `ros2 run diagnostic_monitor rca_cli --action summary` works from any directory and reads the
database the running monitor writes. The CLI prints `[db: ...]` and refuses to create an empty store
if the file does not exist (exit code 2) instead of silently reporting zeros.

## Review 1 demo

Terminal 1 – launch the whole system (4 pipeline nodes + monitor):

```bash
cd ~/ros2_rca_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch rca_test_system system.launch.py
```

(or in the background: `scripts/system.sh start`, `scripts/system.sh stop`)

Terminal 2 – the live dashboard (recommended for the demo):

```bash
cd ~/ros2_rca_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 run diagnostic_tui rca_tui
```

TUI controls: `G` graph focus . `D` diagnosis focus . `T` timeline focus . `Up/Down` scroll the
focused panel (evidence / timeline) . `R` reset scroll / refresh . `Q` or `Esc` quit. The header shows
`ROS LIVE`, `WAITING FOR DIAGNOSTIC MONITOR`, `ROS STATE STALE` (no state for > 3 s) or `BAD INPUT`;
node markers: `[ ]` normal, `[!]` anomalous, `[*]` probable root cause, `[X]` missing from the ROS graph.

Terminal 3 – show what the monitor discovered and that it is quiet (CLI view of the same state):

```bash
cd ~/ros2_rca_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 run diagnostic_monitor rca_cli --action graph
ros2 run diagnostic_monitor rca_cli --action summary        # 0 anomalies / 0 diagnoses when nominal
ros2 run diagnostic_monitor rca_cli --action watch          # live: prints whenever the diagnosis changes
```

Terminal 3 – Experiment 1: sensor fault → `sensor_node` is the probable root cause
(watch the TUI: graph turns `[*] sensor_node -> [!] ... -> [!] ...`, the diagnosis panel fills, the
timeline shows onset order; after the fault expires the window drains and the screen returns to
`SYSTEM NORMAL`)

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

Experiment 3 (crash): `ros2 run rca_test_system fault_injector --target perception_node --type crash`
-> diagnosis switches to `perception_node` within ~0.5 s from downstream starvation; ~20 s later the
node is shown `[X]` missing when the DDS liveliness lease expires. (Restart the system afterwards.)

### 3-5 minute professor demo sequence

1. T1 `ros2 launch rca_test_system system.launch.py`; T2 `ros2 run diagnostic_tui rca_tui`.
   Point out: graph boxes/edges come from ROS 2 graph discovery, `SYSTEM NORMAL`, `ROOT CAUSE None`.
2. T3 inject sensor latency (Experiment 1). Within ~1 s: all four nodes anomalous, `sensor_node` ranked
   first with the R(C) breakdown, propagation chain, evidence text and onset-ordered timeline.
   Note the wording: temporal order is supporting evidence, not proof of causality.
3. Wait ~15 s: recovery to `SYSTEM NORMAL` as the RCA window drains.
4. T3 inject localization failure (Experiment 2): only `localization_node -> navigation_node` anomalous,
   the root cause changes accordingly; sensor and perception stay `[ ]`.
5. Optional: perception crash (Experiment 3), then `ros2 run diagnostic_monitor rca_cli --action timeline`
   from any directory to show the same evidence from the persistent SQLite store.

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

(Regression after the simulation work - validity metrics, same-direction confirmation, data-flow-only
graph discovery: `runs/results_run7.json` / `runs/bench_run7.log`, again 7/7 Top-1, FDR 0, 0 false alarms.)

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

## Simulation environment (`rca_sim`)

Second experimental environment next to the controlled `rca_test_system` benchmark. Both are
diagnosed by the *same, unchanged* `diagnostic_monitor` and rendered by the same TUI.

```
                 Review 1
                     |
        +------------+------------+
        |                         |
 rca_test_system (controlled)   rca_sim (simulation)
        |                         |
        +------------+------------+
                     |
             diagnostic_monitor  ->  RCA engine  ->  /rca/dashboard_state  ->  rca_tui
```

**Simulator.** No Gazebo is installed on the target machine (`gz` missing, only the `ros_gz_*`
bridge packages without `libgz-sim`, no OSRF apt source, WSL2 without a display), so `rca_sim` is a
self-contained lightweight kinematic simulator written with `rclpy` and standard message types
(`sensor_msgs`, `nav_msgs`, `geometry_msgs`). Version: `rca_sim 0.1.0` in this repository; no extra
packages beyond ROS 2 Jazzy desktop + `python3-yaml` are required.

**Topology** (every node is a real ROS 2 node; edges are discovered by the monitor at runtime):

```
lidar_node  ----/sim/lidar/scan----->  perception_node ---/perception/obstacles---> localization_node ---/localization/pose---> planning_node ---/planning/path---> control_node ---/cmd_vel---> sim_world
camera_node ---/sim/camera/image--->  perception_node ---/perception/obstacles---> planning_node
imu_node    ---/sim/imu/data------->  localization_node
odometry_node --/sim/odom---------->  localization_node
```

`sim_world` hosts the 2-D kinematic world (20 m arena, 12 obstacles, differential drive) and the four
sensor nodes in one process; the physical feedback loop actuation -> world -> sensors is closed inside
that process, as on a real robot, so the ROS graph is a DAG. The processing nodes (`perception`,
`localization`, `planning`, `control`) are separate processes with launch `respawn` (4 s).
Downstream nodes retain the source acquisition stamp, so end-to-end latency is physical.

**Runtime degradation mechanisms** (all produce real symptoms; the command channel
`/rca/fault_command` is excluded from the monitor):

| class | fault types | mechanism |
|---|---|---|
| sensor | `noise`, `bias`, `dropout`, `stale`, `rate` | measurement model: Gaussian noise + invalid returns, offset, suppressed publication, re-published old measurement (old stamp), reduced update rate |
| communication | `latency`, `jitter`, `loss` | outbound link model in front of each publisher: real delayed dispatch, random delay, dropped messages |
| workload | `workload`, `cpu_pressure` | real CPU work per callback; busy threads in the process |
| node failure | `crash` (abrupt exit), `stop` (clean exit, respawned), `restart` (sensor node re-created after N s) | process / node lifecycle |
| behaviour | `degradation`, `localization_failure`, `navigation_failure`, `control_failure`, `topic_change` | empty/truncated outputs, pose jump + covariance spike, garbage goals, erratic commands, planner switches its pose source topic at runtime |

Cascades are physical: e.g. a LiDAR problem -> perception cannot cluster (empty set) -> localization
loses landmark corrections and its covariance grows -> planner publishes an empty path (safety) ->
controller commands a stop. The monitor observes rate, end-to-end latency, reordering, invalid
returns, empty outputs, pose uncertainty, stale repeats and zero commands - never a fault label.

### Launch

```bash
cd ~/ros2_rca_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch rca_sim sim_system.launch.py            # simulation + diagnostic monitor
ros2 launch rca_sim sim.launch.py                   # simulation only
#   args: seed:=7  camera_enabled:=true  respawn_delay:=4.0
scripts/sim.sh start|stop|status                     # background helper
```

Nominal check (no faults): `ros2 run diagnostic_monitor rca_cli --action graph` shows the 9-node
graph; a 148 s fault-free soak recorded 0 anomalies and 0 diagnoses (`runs/simulation/nominal_soak.log`).

Manual degradation with the existing injector (any sim node / fault type):

```bash
ros2 run rca_test_system fault_injector --target lidar_node --type noise --param 8 --duration 10
ros2 run rca_test_system fault_injector --target perception_node --type crash
ros2 run rca_test_system fault_injector --target odometry_node --type stale --duration 10
```

### Scenarios and benchmark

`src/rca_sim/config/scenarios.yaml` defines 20 reproducible scenarios (single faults S01-S06,
communication S07-S09, workload S10, dynamic graph S11-S13, compound S14-S16, ambiguous S17-S20) with
fault list (target, type, param, onset, duration) and *evaluation-only* ground truth (`root`,
`accepted_roots`, `observable_chain`, `physical_chain`, `expect_no_diagnosis`). The monitor, RCA engine,
dashboard and TUI never read this file (`rca_sim/tests/test_sim.py` asserts it).

```bash
ros2 run rca_sim scenario_runner --repetitions 5 --warmup 15 --soak 30      # all scenarios
ros2 run rca_sim scenario_runner --scenarios lidar_noise,perception_crash --repetitions 1
ros2 run rca_sim scenario_runner --groups ambiguous,compound
```

The runner (evaluation layer, subclass of the controlled-benchmark controller) waits for a quiet
monitor, records fault-free windows, injects the faults at their onsets, observes, and evaluates the
recorded evidence with the existing evaluator (Top-1/Top-3/MRR, detection and diagnosis latency,
false-diagnosis rate, observable-chain accuracy, physical-chain coverage), then re-ranks the identical
evidence with the five baselines and the ablations. Results: `runs/simulation/<stamp>/results.json`
(+ `runner.log`), written incrementally, never overwriting earlier runs. Reproducibility: same launch
`seed`, same YAML, same repetition count; per-run world/noise randomness is seeded from the launch seed.

### Demo scenarios (professor)

With `ros2 launch rca_sim sim_system.launch.py` in T1 and `ros2 run diagnostic_tui rca_tui` in T2
(never restarted):

1. **Demo 1** `ros2 run rca_sim scenario_runner --scenarios lidar_noise --repetitions 1 --warmup 5 --soak 0`
   - nominal graph -> LiDAR invalid returns -> perception empty -> `lidar_node` identified, timeline shows
   the onset order, recovery to `SYSTEM NORMAL`.
2. **Demo 2** `... --scenarios perception_crash ...` - process crash: downstream starvation ->
   `perception_node` identified within ~0.7 s; launch respawns it, the graph recovers.
3. **Demo 3** `... --scenarios camera_dropout_gated_perception ...` - the low-rate (5 Hz) camera
   stops; perception's multi-modal gating empties the obstacle set *before* the camera's own
   starvation timeout is confirmable, so the downstream symptom is observed first. Temporal-only and
   independent diagnostics pick `perception_node`; the dependency structure ranks `camera_node`
   first. (`odometry_stale_cascade` is a second demo-quality ambiguous case.)

### Results (`runs/simulation/bench2/`, 20 scenarios x 5 repetitions, launch seed 7)

Full log `runner.log`, artifact `results.json`, event store `events.db`. Fault-free observation in
the same run: 84.5 s, 0 anomalies, 0 false alarms. (`runs/simulation/bench1/` is an earlier full run
before the camera-gated perception and the same-direction confirmation refinement: 93/95.)

| | Top-1 | Top-3 | MRR | det. lat. | diag. lat. | FDR | chain acc. | phys. cov. |
|---|---|---|---|---|---|---|---|---|
| proposed (live, 100 runs) | 97 % | 98 % | 0.979 | 0.59 s | 0.77 s | 0.003 | 0.90 | 0.67 |

The three misses are recovery-phase attributions in `localization_restart` (3/5) and
`camera_disappear_reappear` (4/5): the observation window (10 s) extends past the moment the
restarted node is back, and the lingering downstream stale-path symptom is then attributed downstream.

Baselines re-ranking the *identical recorded evidence* (95 runs with evidence; the graph-change
scenario produces none): independent diagnostics **62 %**, temporal-only **95 %**, static dependency +
anomaly 99 %, static dependency + temporal 100 %, proposed 99 %. Temporal-only fails every run of
`camera_dropout_gated_perception` (downstream symptom observed first) and independent diagnostics
fail wherever the downstream symptom is at least as severe as the root's (dropout, crash, degradation,
stale odometry). Ablations (each term removed): 99-100 %; the one run where `no_D`/`no_S` differ from
the full model is a restart run whose runtime graph transiently lacked the restarted node's edges.
Static-graph baselines and the proposed method agree on every scenario except during that transient,
because the discovered runtime graph equals the design graph whenever all nodes are alive - a property
of this small system, not evidence of general superiority.

### Overhead

Measured with `scripts/overhead.sh` on the WSL2 machine (8 cores), 30 s windows, `runs/simulation/overhead.json`:

| configuration | simulation (5 proc) | diagnostic_monitor | TUI |
|---|---|---|---|
| simulation only | 70.6 % CPU, 1010 MB | - | - |
| + diagnostic monitor (nominal / fault) | 84 % / 78 % | 13.0 % / 12.1 % CPU, 206 MB | - |
| + TUI (nominal / fault) | 83 % / 85 % | 12.6 % / 12.8 % CPU, 207 MB | 3 % CPU, ~160 MB |

Monitor: 375 telemetry rows/s into SQLite (WAL, batched), 5 anomaly rows/s + 1.6 diagnoses/s during a
fault, dashboard 0.6-0.8 msg/s (2.3 kB nominal, 15 kB during a fault), RCA compute 0.1-0.2 ms per
diagnosis (95 anomalies, 9 nodes). CPU percentages are sums over processes (100 % = one core).


`scripts/overhead.sh` (uses `scripts/measure_overhead.py`) samples CPU/RSS per process group, SQLite
write rate, `/rca/dashboard_state` rate and RCA compute time into `runs/simulation/overhead.json`.

## Professor demo: the same story on either backend

Both demos show the same sequence without requiring any knowledge of ROS internals:

```
NORMAL -> FAULT INJECTION -> ANOMALIES -> CASCADING SYMPTOMS -> ROOT-CAUSE RANKING -> EVIDENCE -> RECOVERY
```

The TUI is started once and is **never restarted**; everything on screen updates live.

### Demo A - custom simulator (`rca_sim`)

```bash
# terminal 1
scripts/sim.sh start                 # or: ros2 launch rca_sim sim_system.launch.py
# terminal 2
ros2 run diagnostic_tui rca_tui
# terminal 3 - inject an upstream sensor fault and watch the cascade
ros2 run rca_test_system fault_injector --target lidar_node --type noise --param 8 --duration 20
```

### Demo B - Gazebo

```bash
# terminal 1
scripts/gazebo_sim.sh start          # or: ros2 launch rca_gazebo gazebo_system.launch.py
# terminal 2
ros2 run diagnostic_tui rca_tui
# terminal 3 - the equivalent upstream sensor fault, in a real simulator
ros2 run rca_test_system fault_injector --target lidar_node --type noise --param 8 --duration 20
```

What to look at, in order:

1. **Header** - `SYSTEM NORMAL`, the backend label (`rca_sim` or `gazebo`), node/edge/topic counts.
2. **Graph panel** - the dependency graph, discovered at runtime from the ROS 2 graph APIs, not
   declared anywhere. Each node shows `in graph / ALIVE`.
3. **Inject** - within about half a second the LiDAR node turns `[!]`, then perception, then the
   nodes downstream of it: the cascade is visible as it spreads.
4. **Diagnosis panel** - the probable root cause with its confidence and the four score components
   (dependency D, temporal T, symptom coverage S, severity A), the propagation chain and the
   evidence text produced by the monitor.
5. **Timeline** - the onset order of the observed symptoms. Temporal order is *supporting evidence
   for* the ranking, never a proof of physical causality.
6. **Recovery** - when the fault expires the anomalies age out of the RCA window and the header
   returns to `RECOVERING` and then `NORMAL`.

Scripted versions that capture the frames for the record:

```bash
scripts/tui_demo.sh rca_sim          # -> runs/simulation/tui_demo/
scripts/tui_demo.sh gazebo           # -> runs/gazebo/tui_demo/
```

Three demo-quality scenarios per backend are listed under `demos:` in each `scenarios.yaml`.
Nothing on the TUI is ground truth: the scenario name, the injected fault and its target are never
published, and `scripts/tui_frames.py` checks every captured frame for leaks.

## Simulation backends

The diagnostic core is backend-agnostic. Two simulated robot environments feed the *same*
`diagnostic_monitor`, the same RCA engine, the same evaluator, the same baselines and ablations,
the same SQLite schema and the same TUI binary:

```
                              DIAGNOSTIC MONITOR  (unchanged RCA core)
                                        |
                        +---------------+---------------+
                        |                               |
                     rca_sim                          Gazebo
        2-D kinematic simulator (rclpy)      gz sim 8 (Harmonic) + ros_gz bridge
                        |                               |
                        v                               v
                  ROS 2 topics                    ROS 2 topics
                        |                               |
                        +---------------+---------------+
                                        v
                       dynamic graph -> anomalies -> temporal evidence -> RCA
                                        v
                                     CLI / TUI
```

The monitor is never told which backend is running. It receives exactly two deployment settings:
a presentation label (`simulator:=gazebo`) that it copies into `/rca/dashboard_state`, and the list
of raw simulator-transport topic prefixes that are not application data flow
(`excluded_topic_prefixes:=['/gz/']`). Neither names a node, an edge, a scenario or a fault.

Every benchmark artifact carries `"simulator": "rca_sim"` or `"simulator": "gazebo"`; results from
the two backends are reported separately and never pooled into one number.

| | `rca_sim` | Gazebo |
|---|---|---|
| physics | 2-D kinematic integration, ray-cast LiDAR | gz sim 8.11 (Harmonic), rigid-body, `gpu_lidar` + rendered camera |
| robot | point-mass differential kinematics | differential drive, two wheels + caster, `gz-sim-diff-drive-system` |
| sensors | LiDAR 10 Hz, camera 5 Hz, IMU 20 Hz, odometry 20 Hz (in-process nodes) | same rates, produced by Gazebo sensors, bridged, then re-published by ROS driver nodes |
| pipeline | `perception_node -> localization_node -> planning_node -> control_node` | the **same four `rca_sim` executables**, reused verbatim |
| fault injection | `rca_sim/degradation.py` runtime primitives | the same primitives, applied in the `rca_gazebo` drivers and the reused pipeline nodes |
| purpose | fast, fully deterministic, cheap to repeat | a real simulator with real rendering, physics and scheduling |

## Gazebo backend (`rca_gazebo`)

### Architecture

```
gz sim 8 (headless, software rendering)   worlds/rca_arena.sdf
    |  gz transport: /scan /camera/image /imu /model/rca_bot/odometry
    v
ros_gz_bridge parameter_bridge (config/bridge.yaml)        one process, node "gz_bridge"
    |  ROS 2, everything inside the /gz namespace (excluded from monitoring)
    v
rca_gazebo drivers      lidar_node  camera_node  imu_node  odometry_node
    |  ROS 2 application topics: /sim/lidar/scan /sim/camera/image /sim/imu/data /sim/odom
    v
rca_sim pipeline        perception_node -> localization_node -> planning_node -> control_node
    |  /perception/obstacles  /localization/pose  /planning/path  /cmd_vel
    v
rca_gazebo sim_world (actuation sink) --> /gz/cmd_vel --> bridge --> DiffDrive
```

The dependency graph the monitor **discovers at runtime** (from `events.db`, not declared anywhere):

```
lidar_node  ---\
camera_node ----> perception_node --> localization_node --> planning_node --> control_node --> sim_world
imu_node    ------------------------^                    ^
odometry_node -----------------------^                   |
                perception_node -------------------------+
```

Nine nodes, nine edges. The `gz_bridge` process publishes only `/gz/...` topics, so it has no
application-topic endpoint and never appears as a graph node.

### World and robot

`worlds/rca_arena.sdf` is generated by `src/rca_gazebo/scripts/gen_world.py` from
`rca_sim.world.World.default_arena(7)`, so both backends use the *same scene geometry*: a 20 x 20 m
arena bounded by four walls, twelve cylindrical obstacles at identical coordinates, and the robot
starting at (-6, -6) heading 45 deg. Regenerate with:

```bash
python3 src/rca_gazebo/scripts/gen_world.py --seed 7
```

The robot `rca_bot` is a differential-drive base (0.45 x 0.32 x 0.16 m, 6 kg) with two 0.1 m wheels
(0.38 m separation) and a caster. Sensors, with the rates declared in the SDF:

| sensor | SDF type | rate | configuration |
|---|---|---|---|
| LiDAR | `gpu_lidar` | 10 Hz | 90 samples over +/-90 deg, range 0.12-15 m |
| camera | `camera` | 5 Hz | 64 x 48, hFOV 70 deg, RGB |
| IMU | `imu` | 20 Hz | angular rate + linear acceleration |
| odometry | `gz-sim-diff-drive-system` | 20 Hz | wheel odometry, `odom` -> `base_link` |

Measured ROS-side rates on this machine (`ros2 topic hz`, headless software rendering):
`/sim/lidar/scan` 8.9 Hz, `/sim/camera/image` 4.8 Hz, `/sim/imu/data` 24 Hz, `/sim/odom` 18 Hz,
`/perception/obstacles` 9.7 Hz, `/localization/pose` 17.8 Hz, `/planning/path` 5.0 Hz,
`/cmd_vel` 20 Hz. Gazebo scheduling makes these differ slightly from the nominal SDF rates.

### Bridges

Every bridge is declared in `src/rca_gazebo/config/bridge.yaml` and is listed here:

| gz topic | gz type | ROS topic | ROS type | direction |
|---|---|---|---|---|
| `/scan` | `gz.msgs.LaserScan` | `/gz/scan` | `sensor_msgs/msg/LaserScan` | GZ -> ROS |
| `/camera/image` | `gz.msgs.Image` | `/gz/image` | `sensor_msgs/msg/Image` | GZ -> ROS |
| `/camera/camera_info` | `gz.msgs.CameraInfo` | `/gz/camera_info` | `sensor_msgs/msg/CameraInfo` | GZ -> ROS |
| `/imu` | `gz.msgs.IMU` | `/gz/imu` | `sensor_msgs/msg/Imu` | GZ -> ROS |
| `/model/rca_bot/odometry` | `gz.msgs.Odometry` | `/gz/odom` | `nav_msgs/msg/Odometry` | GZ -> ROS |
| `/model/rca_bot/tf` | `gz.msgs.Pose_V` | `/gz/tf` | `tf2_msgs/msg/TFMessage` | GZ -> ROS |
| `/clock` | `gz.msgs.Clock` | `/gz/clock` | `rosgraph_msgs/msg/Clock` | GZ -> ROS |
| `/model/rca_bot/cmd_vel` | `gz.msgs.Twist` | `/gz/cmd_vel` | `geometry_msgs/msg/Twist` | ROS -> GZ |

The monitor only ever sees standard ROS 2 messages on the application topics; no Gazebo-specific
type reaches it.

**Time base.** Gazebo stamps its sensor messages with *simulation* time, which is a different clock
from the monitor's wall clock; `wall_now - sim_stamp` would be meaningless. Each `rca_gazebo` driver
therefore stamps the message with the ROS wall clock at the moment it enters the ROS system, and
every downstream node propagates that stamp unchanged (as in `rca_sim`). The latency the monitor
measures is consequently the **real ROS-side pipeline delay from sensor driver to subscriber**; the
un-instrumented Gazebo-to-bridge hop is not included in it. Latency faults are still real delayed
publication, never timestamp manipulation.

### Fault injection

Faults are the shared runtime primitives from `rca_sim/degradation.py`, applied inside the Gazebo
sensor drivers and the reused pipeline nodes over the `/rca/fault_command` channel, which the
monitor excludes from discovery and never subscribes to. Nothing writes a fault label anywhere the
diagnostic side can read.

| required scenario | mechanism | where |
|---|---|---|
| LiDAR latency | delayed publication through `DegradableLink` | `lidar_node` |
| LiDAR dropout | received scan not republished | `lidar_node` |
| camera dropout | received image not republished | `camera_node` |
| perception processing delay | real `sleep` per callback | `perception_node` |
| localization failure | pose jump + covariance spike | `localization_node` |
| navigation/planning failure | empty or garbage paths | `planning_node` |
| control failure | erratic velocity commands | `control_node` |
| node crash / stop | abrupt `os._exit` / clean exit | any node, respawned by launch |
| process restart | clean exit + `respawn_delay` | any node |
| communication delay | queued publication with real delay | any node's outbound link |
| communication loss | probabilistic drop before publication | any node's outbound link |
| stale / repeated sensor data | re-publish the previous message (repeated stamp) | any sensor driver |
| sensor degradation (noise/bias) | measurement corruption, invalid returns | any sensor driver |
| rate reduction | keep every k-th message | any node's outbound link |
| runtime topic change | planner switches pose source at runtime | `planning_node` |

### Running it

```bash
# start Gazebo + bridge + drivers + pipeline + diagnostic monitor
scripts/gazebo_sim.sh start
scripts/gazebo_sim.sh status        # one process per node, no duplicates
scripts/gazebo_sim.sh topics        # application topics the monitor can see

# live console (same binary as for rca_sim)
ros2 run diagnostic_tui rca_tui

# inject a fault by hand (evaluation layer; the monitor is not told)
ros2 run rca_test_system fault_injector --target lidar_node --type noise --param 8 --duration 20

# full Gazebo benchmark, single command
ros2 run rca_gazebo scenario_runner --benchmark-only --repetitions 3 --warmup 25 --soak 60 \
    --out runs/gazebo/bench2/results.json

scripts/gazebo_sim.sh stop
```

With `gui:=true` the same launch file opens the Gazebo GUI instead of the headless server:

```bash
scripts/gazebo_sim.sh start runs/gazebo/gui.log gui:=true
```

## Results

Results from the two backends are reported separately and are never pooled. Every artifact carries
its `simulator` field, and every table below is generated from the artifact by
`scripts/report_summary.py`, so the report cannot drift from the data.

> **Functional validation, not a superiority claim.** These benchmarks show that the system works
> end to end on two independent simulators and that dependency-aware ranking is decisively better
> than dependency-blind ranking. They do **not** establish general superiority of the *dynamic*
> graph over a *static* design-time graph: on both backends the two static-graph baselines match
> the proposed method exactly, because in this nine-node pipeline the runtime graph equals the
> design-time graph whenever every node is alive. The advantage of runtime discovery shows up only
> when the runtime topology differs from the design-time one - demonstrated by the runtime
> topic-endpoint change in `scripts/graph_probe.py` and by
> `tests/test_baselines_evaluator.py`, not by these accuracy numbers.

### Gazebo backend

### Artifact `runs/gazebo/bench2/results.json`  (simulator: **gazebo**)

| quantity | numerator | denominator | value |
|---|---|---|---|
| Top-1 accuracy | 51 | 54 runs | 0.9444 |
| Top-3 accuracy | 51 | 54 runs | 0.9444 |
| mean reciprocal rank | sum of 1/rank | 54 runs | 0.9444 |
| post-injection false-diagnosis rate | 0 | 802 diagnoses | 0.0000 |
| nominal false alarms | 4 | 113.1 s fault-free in 56 windows | 2.1214 /min |
| observable chain accuracy (mean) | - | 54 runs | 0.871 |
| chain order accuracy | 48 | 54 runs | 0.889 |
| physical chain coverage (mean) | - | 54 runs | 0.640 |

| latency (s) | n | mean | min | p25 | median | p75 | max | std |
|---|---|---|---|---|---|---|---|---|
| detection | 48 | 0.582 | 0.062 | 0.188 | 0.486 | 0.671 | 3.369 | 0.582 |
| diagnosis (any root) | 48 | 0.762 | 0.141 | 0.465 | 0.663 | 0.808 | 3.369 | 0.548 |
| diagnosis (correct root) | 48 | 0.762 | 0.141 | 0.465 | 0.663 | 0.808 | 3.369 | 0.548 |

| cohort | n | Top-1 | accuracy |
|---|---|---|---|
| ALL RUNS (proposed, live) | 54 | 51 | 0.9444 |
| COMMON-EVIDENCE SUBSET (proposed) | 51 | 48 | 0.9412 |
| &nbsp;&nbsp;baseline `independent` | 51 | 27 | 0.5294 (MRR 0.726) |
| &nbsp;&nbsp;baseline `temporal_only` | 51 | 42 | 0.8235 (MRR 0.882) |
| &nbsp;&nbsp;baseline `static_dep_anom` | 51 | 48 | 0.9412 (MRR 0.941) |
| &nbsp;&nbsp;baseline `static_dep_temp` | 51 | 48 | 0.9412 (MRR 0.941) |
| &nbsp;&nbsp;baseline `proposed` | 51 | 48 | 0.9412 (MRR 0.941) |

Excluded from the common-evidence subset: **3** run(s); reasons: {'scenario declares no ground-truth root cause (expect_no_diagnosis)': 3}; scenarios: ['planning_topic_change']. Denominators match: **True**.

| ablation | weights | Top-1 |
|---|---|---|
| `full` | {'wD': 0.3, 'wT': 0.3, 'wS': 0.25, 'wA': 0.15} | 48/51 (94 %) |
| `no_D` | {'wD': 0.0, 'wT': 0.3, 'wS': 0.25, 'wA': 0.15} | 48/51 (94 %) |
| `no_T` | {'wD': 0.3, 'wT': 0.0, 'wS': 0.25, 'wA': 0.15} | 48/51 (94 %) |
| `no_S` | {'wD': 0.3, 'wT': 0.3, 'wS': 0.0, 'wA': 0.15} | 48/51 (94 %) |
| `no_A` | {'wD': 0.3, 'wT': 0.3, 'wS': 0.25, 'wA': 0.0} | 48/51 (94 %) |

| scenario | runs | Top-1 | Top-3 | MRR | det. (s) | diag. (s) | chain | phys. cov. |
|---|---|---|---|---|---|---|---|---|
| `lidar_latency` | 3 | 3 | 3 | 1.000 | 0.533 | 0.728 | 1.00 | 0.40 |
| `lidar_dropout` | 3 | 3 | 3 | 1.000 | 0.788 | 0.788 | 1.00 | 0.60 |
| `lidar_noise` | 3 | 3 | 3 | 1.000 | 0.135 | 0.323 | 1.00 | 0.40 |
| `camera_dropout` | 3 | 3 | 3 | 1.000 | 0.562 | 0.872 | 0.60 | 1.00 |
| `odometry_stale` | 3 | 3 | 3 | 1.000 | 0.088 | 0.407 | 1.00 | 0.75 |
| `imu_rate_drop` | 3 | 0 | 0 | 0.000 | - | - | 0.00 | 0.00 |
| `perception_delay` | 3 | 3 | 3 | 1.000 | 0.827 | 0.827 | 1.00 | 0.25 |
| `perception_degradation` | 3 | 3 | 3 | 1.000 | 0.181 | 0.520 | 1.00 | 0.50 |
| `localization_failure` | 3 | 3 | 3 | 1.000 | 0.089 | 0.399 | 1.00 | 1.00 |
| `navigation_failure` | 3 | 3 | 3 | 1.000 | 0.349 | 0.594 | 1.00 | 1.00 |
| `control_failure` | 3 | 3 | 3 | 1.000 | 1.731 | 1.960 | 1.00 | 1.00 |
| `perception_link_latency` | 3 | 3 | 3 | 1.000 | 0.571 | 0.679 | 0.50 | 0.25 |
| `lidar_link_loss` | 3 | 3 | 3 | 1.000 | 1.730 | 1.730 | 1.00 | 0.67 |
| `perception_crash` | 3 | 3 | 3 | 1.000 | 0.600 | 0.600 | 1.00 | 0.50 |
| `localization_restart` | 3 | 3 | 3 | 1.000 | 0.449 | 0.630 | 1.00 | 0.67 |
| `planning_topic_change` | 3 | 3 | 3 | 1.000 | - | - | 1.00 | 1.00 |
| `lidar_noise_plus_perception_workload` | 3 | 3 | 3 | 1.000 | 0.181 | 0.388 | 0.78 | 0.53 |
| `camera_dropout_plus_localization_failure` | 3 | 3 | 3 | 1.000 | 0.495 | 0.753 | 0.80 | 1.00 |

**Misses (3/54):**

* `imu_rate_drop` rep 0: predicted `None` (truth `imu_node`), anomalous nodes observed []
* `imu_rate_drop` rep 1: predicted `None` (truth `imu_node`), anomalous nodes observed []
* `imu_rate_drop` rep 2: predicted `None` (truth `imu_node`), anomalous nodes observed []

**Reading these numbers.**

* Every one of the three misses is the same scenario, `imu_rate_drop`, and it produced **no
  evidence at all** (zero anomalies observed, so no method could rank anything). Reducing a 20 Hz
  IMU to 5 Hz changes the inter-arrival time from 0.050 s to 0.200 s, a deviation of 0.15 s, which
  is below the detector's 0.18 s physical-deviation floor for inter-arrival - the floor that stops
  ordinary scheduling jitter from being reported. This is a genuine, documented sensitivity limit
  of the monitor, not a ranking failure, and the scenario is deliberately kept in the benchmark
  rather than re-tuned to make the number look better.
* `nominal false alarms = 4` is the pooled figure over **all** fault-free windows. Split by kind:
  the dedicated fault-free observation (25 s warm-up + 60 s soak, 85.0 s) recorded **0 anomalies
  and 0 diagnoses**; the 4 diagnoses all fall in the 28.1 s made up of 54 very short (mean 0.5 s)
  pre-injection windows, and in each case **zero anomalies were recorded inside the window** - they
  are residual diagnoses computed from the previous scenario's evidence, which is still inside the
  6 s RCA window while it drains. Both numbers are reported; neither is hidden.
* Detection latency is heavily skewed (median 0.49 s, max 3.37 s): the long tail is
  `control_failure` and `lidar_link_loss`, where the symptom has to accumulate before it clears the
  confirmation and deviation floors.
* All five ablations score identically here. On these scenarios each individual scoring term is
  sufficient on its own, so the ablation does not separate them; it is reported as measured.

### `rca_sim` backend

### Artifact `runs/simulation/bench3/results.json`  (simulator: **rca_sim**)

| quantity | numerator | denominator | value |
|---|---|---|---|
| Top-1 accuracy | 97 | 100 runs | 0.9700 |
| Top-3 accuracy | 98 | 100 runs | 0.9800 |
| mean reciprocal rank | sum of 1/rank | 100 runs | 0.9790 |
| post-injection false-diagnosis rate | 10 | 1668 diagnoses | 0.0060 |
| nominal false alarms | 12 | 137.8 s fault-free in 102 windows | 5.2243 /min |
| observable chain accuracy (mean) | - | 100 runs | 0.901 |
| chain order accuracy | 92 | 100 runs | 0.920 |
| physical chain coverage (mean) | - | 100 runs | 0.670 |

| latency (s) | n | mean | min | p25 | median | p75 | max | std |
|---|---|---|---|---|---|---|---|---|
| detection | 95 | 0.480 | 0.056 | 0.157 | 0.489 | 0.682 | 2.592 | 0.405 |
| diagnosis (any root) | 95 | 0.653 | 0.064 | 0.361 | 0.582 | 0.894 | 2.592 | 0.406 |
| diagnosis (correct root) | 95 | 0.674 | 0.064 | 0.361 | 0.582 | 0.894 | 3.091 | 0.480 |

| cohort | n | Top-1 | accuracy |
|---|---|---|---|
| ALL RUNS (proposed, live) | 100 | 97 | 0.9700 |
| COMMON-EVIDENCE SUBSET (proposed) | 95 | 92 | 0.9684 |
| &nbsp;&nbsp;baseline `independent` | 95 | 53 | 0.5579 (MRR 0.761) |
| &nbsp;&nbsp;baseline `temporal_only` | 95 | 89 | 0.9368 (MRR 0.968) |
| &nbsp;&nbsp;baseline `static_dep_anom` | 95 | 95 | 1.0000 (MRR 1.000) |
| &nbsp;&nbsp;baseline `static_dep_temp` | 95 | 95 | 1.0000 (MRR 1.000) |
| &nbsp;&nbsp;baseline `proposed` | 95 | 95 | 1.0000 (MRR 1.000) |

Excluded from the common-evidence subset: **5** run(s); reasons: {'scenario declares no ground-truth root cause (expect_no_diagnosis)': 5}; scenarios: ['planning_topic_change']. Denominators match: **True**.

| ablation | weights | Top-1 |
|---|---|---|
| `full` | {'wD': 0.3, 'wT': 0.3, 'wS': 0.25, 'wA': 0.15} | 95/95 (100 %) |
| `no_D` | {'wD': 0.0, 'wT': 0.3, 'wS': 0.25, 'wA': 0.15} | 95/95 (100 %) |
| `no_T` | {'wD': 0.3, 'wT': 0.0, 'wS': 0.25, 'wA': 0.15} | 95/95 (100 %) |
| `no_S` | {'wD': 0.3, 'wT': 0.3, 'wS': 0.0, 'wA': 0.15} | 95/95 (100 %) |
| `no_A` | {'wD': 0.3, 'wT': 0.3, 'wS': 0.25, 'wA': 0.0} | 95/95 (100 %) |

| scenario | runs | Top-1 | Top-3 | MRR | det. (s) | diag. (s) | chain | phys. cov. |
|---|---|---|---|---|---|---|---|---|
| `lidar_noise` | 5 | 5 | 5 | 1.000 | 0.155 | 0.384 | 1.00 | 0.40 |
| `lidar_dropout` | 5 | 5 | 5 | 1.000 | 0.648 | 0.648 | 1.00 | 0.60 |
| `perception_degradation` | 5 | 5 | 5 | 1.000 | 0.163 | 0.506 | 1.00 | 0.50 |
| `perception_crash` | 5 | 5 | 5 | 1.000 | 0.762 | 0.762 | 1.00 | 0.50 |
| `localization_failure` | 5 | 5 | 5 | 1.000 | 0.069 | 0.325 | 1.00 | 1.00 |
| `control_failure` | 5 | 5 | 5 | 1.000 | 1.160 | 1.381 | 1.00 | 1.00 |
| `perception_link_latency` | 5 | 5 | 5 | 1.000 | 0.555 | 0.736 | 0.50 | 0.25 |
| `lidar_link_jitter` | 5 | 5 | 5 | 1.000 | 0.535 | 0.801 | 1.00 | 0.67 |
| `lidar_link_loss` | 5 | 5 | 5 | 1.000 | 1.279 | 1.365 | 1.00 | 0.67 |
| `perception_workload` | 5 | 5 | 5 | 1.000 | 0.756 | 1.021 | 1.00 | 0.33 |
| `localization_restart` | 5 | 3 | 3 | 0.680 | 0.315 | 0.380 | 0.80 | 0.53 |
| `camera_disappear_reappear` | 5 | 4 | 5 | 0.900 | 0.335 | 0.341 | 0.93 | 0.56 |
| `planning_topic_change` | 5 | 5 | 5 | 1.000 | - | - | 1.00 | 1.00 |
| `lidar_noise_plus_perception_jitter` | 5 | 5 | 5 | 1.000 | 0.156 | 0.265 | 1.00 | 0.40 |
| `lidar_noise_plus_perception_workload` | 5 | 5 | 5 | 1.000 | 0.175 | 0.325 | 0.93 | 0.44 |
| `perception_plus_localization_degradation` | 5 | 5 | 5 | 1.000 | 0.176 | 0.379 | 0.50 | 1.00 |
| `odometry_stale_cascade` | 5 | 5 | 5 | 1.000 | 0.092 | 0.420 | 1.00 | 0.75 |
| `workload_then_localization_failure` | 5 | 5 | 5 | 1.000 | 0.670 | 0.943 | 1.00 | 1.00 |
| `camera_dropout_gated_perception` | 5 | 5 | 5 | 1.000 | 0.581 | 0.884 | 0.60 | 1.00 |
| `lidar_dropout_with_localization_stop` | 5 | 5 | 5 | 1.000 | 0.536 | 0.536 | 0.75 | 0.80 |

**Misses (3/100):**

* `localization_restart` rep 0: predicted `planning_node` (truth `localization_node`), anomalous nodes observed ['localization_node', 'planning_node']
* `camera_disappear_reappear` rep 3: predicted `perception_node` (truth `camera_node`), anomalous nodes observed ['camera_node', 'localization_node', 'perception_node']
* `localization_restart` rep 4: predicted `planning_node` (truth `localization_node`), anomalous nodes observed ['localization_node', 'planning_node']

**Reading these numbers - the 97 % vs 99 % question.**

An earlier draft of this report quoted "proposed 97 %" next to "baselines on a 95-run subset, proposed 99 %"
without saying why the denominators differed. The full reconciliation:

* **100 runs** were executed (20 scenarios x 5 repetitions). The proposed method, running live,
  got **97/100 = 97.0 %**.
* **5 runs are excluded from the baseline comparison**: all five are `planning_topic_change`, the
  graph-change scenario that deliberately declares *no* root cause (`expect_no_diagnosis: true`).
  There is nothing to rank, so "Top-1" is undefined for every method equally. The exclusion rule
  is written in the scenario file, is fixed before the comparison, and is method-independent; the
  excluded count, reason and scenario key are printed in every run and stored in the artifact under
  `cohorts.excluded_from_common_subset`. The proposed method scored 5/5 on those runs under their
  own criterion (raise no diagnosis at all), which is why the all-runs figure (97/100) is slightly
  higher than the subset figure.
* On the **95-run common-evidence subset** the live proposed method scored **92/95 = 96.8 %**.
* The `proposed` row in the baseline table scores **95/95 = 100 %** on exactly the same 95 runs.
  That row is the *same algorithm with the same weights re-ranking the recorded evidence offline*,
  once, over the whole observation window. The live figure is lower because a live run is scored on
  the **last** diagnosis inside its observation window, and in the restart scenarios that window
  deliberately extends past the moment the failed node is back: the three misses are
  `localization_restart` (2/5) and `camera_disappear_reappear` (1/5), where the lingering
  downstream stale-path symptom during recovery is attributed downstream. The gap is
  live-versus-offline scoring of the same method, not a change of denominator and not a different
  algorithm.
* Runs in which the proposed method produced no diagnosis at all are **kept** in the cohort and
  counted as a miss for every method; the offline comparison is no longer skipped for them (that
  method-dependent exclusion was the actual defect).

Dependency-blind ranking is clearly worse (`independent` 55.8 %, `temporal_only` 93.7 %). The two
static-dependency baselines score 100 % - i.e. **equal to or better than** the proposed method on
this scenario set - so these results do not support a superiority claim for the dynamic graph;
see the note at the top of this section.

`nominal false alarms = 12` over 137.8 s is again a pooled figure: the dedicated 85.0 s fault-free
observation (warm-up + soak) recorded **0 anomalies and 0 diagnoses**, and all 12 diagnoses came
from a single 8.0 s pre-injection window whose evidence was still draining out of the 6 s RCA
window after the preceding `camera_dropout_gated_perception` scenario.

### Dynamic graph validation (runtime, both backends)

`scripts/graph_probe.py` drives real runtime changes against a live backend and reads the result
back out of the monitor's own recorded graph snapshots (`runs/gazebo/dynamic_graph.json`,
`runs/simulation/dynamic_graph.json`):

| runtime change | observed by the monitor | gazebo | rca_sim |
|---|---|---|---|
| a new node joins the running system | it appears in the discovered graph | True | True |
| ...with a real data-flow edge | the edge `lidar_node -> aux_probe_node` appears | True | True |
| a node switches its pose source at runtime | is the old edge `localization_node -> planning_node` still present? (expected: no) | False | False |
| ...and the new edge appears | `odometry_node -> planning_node` | True | True |
| the graph differs from the baseline during the change | edge set changed | True | True |
| after the change is undone | node set back to baseline | True | True |
| | edge set back to baseline | True | True |
| a process crashes | seconds until the first anomaly (runtime starvation) | 0.413 | 0.628 |
| | graph snapshots recorded during the probe | 6 | 5 |
| a node is SIGKILLed | seconds until it leaves the ROS graph (DDS lease) | 20.04 | 20.04 |
| a node exits cleanly | seconds until it leaves the ROS graph | 0.2 | 0.2 |
| ...and is restarted | seconds until it is back in the ROS graph | 7.22 | 6.21 |
| | is it back in the monitor's discovered graph? | True | True |

The measured DDS participant lease is the number quoted throughout this README as "roughly 20 s": a process killed without a clean DDS unregister keeps its ROS graph membership for that long. Runtime starvation reports the same failure in under a second. A node that exits *cleanly* unregisters from DDS immediately and leaves the graph in about 0.2 s, which is why a clean stop and a crash look completely different at the graph level while looking the same at the data-flow level. Note also that when a crashed node is respawned by the launcher after 4 s it never leaves the graph at all, so graph membership alone would have shown nothing.


### Overhead

`scripts/overhead.sh <backend>` asserts the expected process count of every group before each
measurement (`--expect diagnostic_monitor=1 --expect tui=1 ...`) and aborts rather than measure a
contaminated system; all PIDs are recorded in the artifact. An earlier measurement was contaminated
by a second backend instance and a stray TUI and has been **re-run**, not edited: the contaminated
numbers over-reported the monitor by roughly 3x and the TUI by roughly 2x.

CPU is taken from `/proc/<pid>/stat` `utime+stime` deltas over the sampling window, not from
`ps %cpu` (which averages over the whole process lifetime). Three normalisations are given so no
percentage is ambiguous:

* **raw %** - sum over the group's processes, where one fully busy core is 100 %;
* **core-equivalents** - raw / 100;
* **% of machine** - raw / (100 x logical cores).


**Gazebo backend** (`runs/gazebo/overhead.json`) - 8 logical cores (800 % total capacity), 11th Gen Intel(R) Core(TM) i7-11370H @ 3.30GHz, 7816 MB RAM; 30 s windows.

| configuration | simulator backend | pipeline (4 nodes) | diagnostic monitor | TUI |
|---|---|---|---|---|
| `gazebo_only` | 7p, 136.9 % raw (1.37 cores, 17.1 % of machine), 854 MB | 4p, 54.0 % raw (0.54 cores, 6.8 % of machine), 296 MB | - | - |
| `gazebo_monitor_nominal` | 7p, 131.8 % raw (1.32 cores, 16.5 % of machine), 855 MB | 4p, 50.2 % raw (0.50 cores, 6.3 % of machine), 298 MB | 1p, 16.5 % raw (0.17 cores, 2.1 % of machine), 79 MB | - |
| `gazebo_monitor_fault` | 7p, 137.0 % raw (1.37 cores, 17.1 % of machine), 856 MB | 4p, 52.3 % raw (0.52 cores, 6.5 % of machine), 298 MB | 1p, 17.1 % raw (0.17 cores, 2.1 % of machine), 80 MB | - |
| `gazebo_monitor_tui_nominal` | 7p, 136.9 % raw (1.37 cores, 17.1 % of machine), 857 MB | 4p, 51.7 % raw (0.52 cores, 6.5 % of machine), 299 MB | 1p, 16.9 % raw (0.17 cores, 2.1 % of machine), 80 MB | 1p, 2.4 % raw (0.02 cores, 0.3 % of machine), 33 MB |
| `gazebo_monitor_tui_fault` | 7p, 142.1 % raw (1.42 cores, 17.8 % of machine), 854 MB | 4p, 54.4 % raw (0.54 cores, 6.8 % of machine), 300 MB | 1p, 19.7 % raw (0.20 cores, 2.5 % of machine), 80 MB | 1p, 3.4 % raw (0.03 cores, 0.4 % of machine), 33 MB |

During a fault: SQLite 369.1 telemetry rows/s, 9.13 anomaly rows/s, 2.0 diagnosis rows/s (DB 16.42 MB); `/rca/dashboard_state` 0.6 msg/s at 15721 bytes; RCA compute 0.17 ms per diagnosis over 91 anomalies and 9 nodes.

**rca_sim backend** (`runs/simulation/overhead.json`) - 8 logical cores (800 % total capacity), 11th Gen Intel(R) Core(TM) i7-11370H @ 3.30GHz, 7816 MB RAM; 30 s windows.

| configuration | simulator backend | pipeline (4 nodes) | diagnostic monitor | TUI |
|---|---|---|---|---|
| `rca_sim_only` | 1p, 57.9 % raw (0.58 cores, 7.2 % of machine), 78 MB | 4p, 43.3 % raw (0.43 cores, 5.4 % of machine), 292 MB | - | - |
| `rca_sim_monitor_nominal` | 1p, 48.6 % raw (0.49 cores, 6.1 % of machine), 78 MB | 4p, 44.7 % raw (0.45 cores, 5.6 % of machine), 293 MB | 1p, 15.5 % raw (0.15 cores, 1.9 % of machine), 78 MB | - |
| `rca_sim_monitor_fault` | 1p, 47.8 % raw (0.48 cores, 6.0 % of machine), 78 MB | 4p, 42.8 % raw (0.43 cores, 5.3 % of machine), 293 MB | 1p, 16.4 % raw (0.16 cores, 2.0 % of machine), 79 MB | - |
| `rca_sim_monitor_tui_nominal` | 1p, 49.3 % raw (0.49 cores, 6.2 % of machine), 79 MB | 4p, 45.3 % raw (0.45 cores, 5.7 % of machine), 294 MB | 1p, 15.6 % raw (0.16 cores, 1.9 % of machine), 79 MB | 1p, 2.3 % raw (0.02 cores, 0.3 % of machine), 32 MB |
| `rca_sim_monitor_tui_fault` | 1p, 47.3 % raw (0.47 cores, 5.9 % of machine), 79 MB | 4p, 42.8 % raw (0.43 cores, 5.3 % of machine), 295 MB | 1p, 16.8 % raw (0.17 cores, 2.1 % of machine), 79 MB | 1p, 2.5 % raw (0.03 cores, 0.3 % of machine), 32 MB |

During a fault: SQLite 375.0 telemetry rows/s, 10.27 anomaly rows/s, 2.0 diagnosis rows/s (DB 17.59 MB); `/rca/dashboard_state` 0.73 msg/s at 16503 bytes; RCA compute 0.19 ms per diagnosis over 95 anomalies and 9 nodes.

## Metric definitions (every rate carries its numerator and denominator)

All of these are computed in `diagnostic_monitor/evaluator.py`, which is the evaluation layer.
The monitor and the RCA engine never see any of them.

### Accuracy

| metric | numerator | denominator | level |
|---|---|---|---|
| `top1_accuracy` | `top1_correct` runs | `total_experiments` | run |
| `top3_accuracy` | runs whose true root is in the first 3 candidates | `total_experiments` | run |
| `mean_reciprocal_rank` | sum of 1/rank (0 when absent) | `total_experiments` | run |

A run counts once regardless of how many diagnoses it produced.

### Two different false-diagnosis quantities (never both called "FDR")

**Post-injection false-diagnosis rate** - diagnosis-level, pooled over every diagnosis issued after
a fault was injected:

```
post_injection_false_diagnoses          # diagnoses naming a node that is not the true root
post_injection_diagnoses                # all diagnoses issued after injection
post_injection_false_diagnosis_rate = post_injection_false_diagnoses / post_injection_diagnoses
```

**Nominal false-alarm rate** - window-level, measured while *no fault was active at all*. Any
diagnosis in a fault-free window is a false alarm by definition:

```
nominal_false_alarms                    # diagnoses issued in fault-free windows
nominal_fault_free_windows              # number of such windows
nominal_fault_free_sec                  # total fault-free wall-clock time observed
nominal_false_alarm_rate_per_min = nominal_false_alarms / (nominal_fault_free_sec / 60)
```

The JSON key `false_diagnosis_rate` is kept as a backwards-compatible alias of
`post_injection_false_diagnosis_rate` and of nothing else.

### Latency (reported as a distribution, not only a mean)

`detection_latency_sec` is *first anomaly recorded after injection* - `t_inject`;
`diagnosis_latency_sec` is *first diagnosis of any root after injection* - `t_inject`;
`correct_diagnosis_latency_sec` is *first diagnosis naming the true root* - `t_inject`.
`t_inject` is the physical fault onset (the moment the fault command takes effect in the target
node), not a bookkeeping timestamp.

Each of the three is summarised in the aggregate as
`{n, mean, min, max, std, median, p25, p75}` under `detection_latency_stats`,
`diagnosis_latency_stats` and `correct_diagnosis_latency_stats`, and per scenario in
`per_scenario`. The underlying calculation is unchanged; only the reporting was widened.

### Propagation chain: observable vs physical

Two different node sets, kept separate on purpose:

* **observable chain** - the nodes whose symptom the monitor's metrics can actually observe.
* **physical chain** - the nodes physically affected in the simulated robot, which may include
  nodes whose symptom no monitored metric can see.

```
chain_accuracy          = |predicted & observable| / |predicted | observable|     (Jaccard, per run)
physical_chain_coverage = |predicted & physical|   / |physical|                   (recall, per run)
chain_order_correct     = the observable nodes appear in the predicted chain in the expected
                          relative order AND all of them are present
```

`chain_accuracy_numerator`/`chain_accuracy_denominator` and
`physical_chain_numerator`/`physical_chain_denominator` are stored per run. Aggregates are
**unweighted means over runs** (`avg_chain_accuracy`, `avg_physical_chain_coverage`), each with its
sample size (`chain_accuracy_n`, `physical_chain_coverage_n`). `chain_order_accuracy` is
`chain_order_correct_count / chain_order_n`.

A physically affected node that the monitor cannot observe is listed in
`physically_affected_unobserved` and **is never credited to the prediction**: it can only lower
`physical_chain_coverage`, never raise `chain_accuracy`. The TUI shows only observed state, so it
cannot imply that an unobserved physical effect was seen.

### Cohorts: no silent denominator switching

The proposed method runs live; the baselines and ablations re-rank the *identical recorded
evidence* offline. Every artifact and every printed summary therefore reports two cohorts:

* **ALL RUNS** - every executed run, including runs where the proposed method produced no
  diagnosis at all (those count as a miss).
* **COMMON-EVIDENCE SUBSET** - the runs that were also scored by the baselines.

The inclusion criterion for the subset is fixed before the comparison and is
**method-independent**: a run is included iff its scenario declares a ground-truth root cause, i.e.
iff it is not an `expect_no_diagnosis` graph-change scenario (there is no root to rank, so
"accuracy" is undefined for every method equally). A run in which the proposed method failed to
diagnose anything is still scored for every method and stays in the cohort. `denominators_match`
in the artifact asserts that the proposed method and each baseline used the same denominator.

This is what an earlier draft of this report got wrong: it quoted the proposed method's Top-1 over
all runs and the baselines' Top-1 over a smaller subset, because the offline comparison used to be
skipped whenever the proposed method produced no diagnosis - a method-dependent exclusion. Both
runners now score the baselines on every ground-truth run, so the two figures are directly
comparable and the excluded count and reason are printed explicitly.

### What is monitored (and what is not)

The monitor computes, per (publishing node, topic), only these quantities:

| metric | meaning |
|---|---|
| `inter_arrival` | seconds between consecutive messages (publication rate) |
| `latency` | `receive_time - header.stamp`, i.e. end-to-end age of the measurement |
| `out_of_order` | a header stamp older than the previous one on the same topic (evidence, severity 0.4) |
| `stale_repeat` | the header stamp repeated exactly: a re-published (stale) measurement |
| `nan_ratio` | fraction of NaN/Inf returns in a `LaserScan` (invalid measurements) |
| `obstacle_empty` | the perception output contains no detections at all |
| `path_empty` | the planner output contains no poses at all ("no plan") |
| `pose_uncertainty` | `sqrt(cov[0] + cov[7])` of a pose-with-covariance message |
| `linear_vel` | commanded linear velocity |
| `cmd_zero` | the command is exactly zero (a commanded stop) |
| node starvation | no message within the learned inter-arrival timeout |

These are message-level freshness, ordering and **payload-validity** indicators. They are not
"generic": `obstacle_empty`, `path_empty`, `pose_uncertainty`, `cmd_zero` and `nan_ratio` are
domain-specific readings of specific robotics message types, selected because an *empty* or
*invalid* output is a fault symptom while the *content* of a valid output is not. Scene content
(how many obstacles are visible, how fast the robot is driving) legitimately varies on a moving
robot and is deliberately not treated as an anomaly. Semantic symptoms that no such indicator
captures - for example a pose that is frozen but still well-formed and confident - are invisible to
the monitor, which is exactly what the observable-vs-physical chain reporting makes explicit.

## ROS communication graph vs physical feedback

The discovered graph is a **ROS communication/dependency graph**: an edge `u -> v` means `u`
publishes a topic that `v` subscribes to. In both backends that graph is a **DAG**, ending at the
actuation sink `sim_world`.

The robot itself is **not** acyclic. Commands change the robot's motion, which changes what the
sensors measure, which changes the commands. That loop is closed **inside the simulator's world
process** (the `gz sim` server, or the `rca_sim` world process) exactly as it is closed through
physics on a real robot. It is a physical feedback loop, not a ROS communication dependency, so it
does not appear as an edge in the monitored graph.

So: *the ROS communication/dependency graph is represented as a DAG for the monitored runtime
relationships, while physical robot feedback is closed within the simulator's world process.* The
graph model is not modified to hide or to create this distinction, and no claim is made that the
robot as a physical system is acyclic.

## Crash visibility: DDS lease vs runtime liveness

A crashed process does **not** disappear from the ROS 2 graph immediately. `rmw_fastrtps_cpp`
removes a participant only after its DDS liveliness lease expires, which on this machine is
measured at roughly 20 s (see "Measured DDS lease" below). Graph membership is therefore a late
and unreliable crash detector.

What actually detects the crash quickly is **runtime liveness**: the monitor learns each
(node, topic) inter-arrival distribution and raises a starvation anomaly when the stream stops,
typically within a few hundred milliseconds. The downstream nodes that depended on the dead node
then starve in turn, and the dependency-aware ranking attributes the cascade to the node that
stopped first.

Both concepts are published separately in `/rca/dashboard_state` and shown separately in the TUI,
so a viewer is never told that a crashed node vanished instantly:

```json
{"name": "perception_node", "status": "MISSING",
 "in_ros_graph": true,        // still a DDS participant: the lease has not expired yet
 "liveness": "STARVED"}       // but no data has arrived within its learned timeout
```

The TUI renders this as `in graph / STARVED` on the node card.

## Known limitations

* The dashboard/TUI is a visualisation layer; all RCA logic, evidence and wording come from
  `diagnostic_monitor`. The TUI never touches SQLite (SQLite remains the persistent store for the CLI,
  experiments and reproducibility).
* Temporal ordering in the timeline and in T(C) is supporting evidence for a probable root cause,
  never proof of causality.
* The Review 1 scenarios demonstrate functionality; they do not establish general superiority over all
  baselines (see the baseline/ablation discussion above).
* `out_of_order` observations after a latency fault expires are real: newly created scans publish
  immediately while still-delayed older scans drain later. They are bounded (at most one per delayed
  message, severity 0.4), flagged `informational` in the dashboard and clear with the window;
  `tests/test_edge_cases.py::test_post_fault_queue_drain_reordering_is_bounded_and_informational`
  pins this behaviour.

* Only rate, end-to-end latency, reordering and payload-*validity* metrics are monitored (invalid
  returns, empty outputs, stale repeats, pose uncertainty, zero commands); scene content such as the
  number of detected objects or the driving speed legitimately varies on a moving robot and is not
  treated as an anomaly. Semantic symptoms (e.g. a frozen pose) are invisible. This is made explicit
  through the observable-vs-physical chain reporting.
* `rca_sim` is a kinematic 2-D simulator, not a physics engine; it is kept as the fast, fully
  deterministic backend. The Gazebo backend adds real rigid-body physics and real rendered sensors,
  but runs headless with software OpenGL on this machine, so its real-time factor and sensor rates
  are a few percent below nominal (measured rates are listed in the Gazebo section). The
  perception/localization/planning/control nodes are deliberately simple and are shared by both
  backends. Robot node
  crashes take ~20 s to disappear from the ROS graph (DDS liveliness lease); the RCA relies on
  downstream starvation timeouts in the meantime. After a root cause has *recovered* (restart
  scenarios), lingering downstream symptoms can be attributed to the downstream node for a few
  seconds (S11 in the simulation benchmark) - the observation window there deliberately covers
  the recovery phase.
* The two-sample confirmation requires both samples to deviate in the same direction: a scheduling
  stall followed by its catch-up burst is one event, not persistence.
* Graph-level crash detection is bounded by the DDS participant liveliness lease, **measured at
  20.04 s** on this machine with `rmw_fastrtps_cpp` (`scripts/graph_probe.py`, recorded in
  `runs/*/dynamic_graph.json`). Runtime starvation reports the same failure in 0.4-0.9 s. When a
  crashed node is respawned by the launcher after 4 s it never leaves the ROS graph at all, so graph
  membership alone would show nothing; `/rca/dashboard_state` therefore publishes `in_ros_graph` and
  `liveness` as two separate fields and the TUI renders both.
* In this nine-node pipeline the runtime graph equals the design-time graph whenever every node is
  alive, so the static-graph baselines match (and on `rca_sim` slightly exceed) the proposed method
  on the live scenarios. The advantage of runtime discovery is demonstrated separately, by the
  runtime topic-endpoint change in `scripts/graph_probe.py` (the
  `localization_node -> planning_node` edge disappears and `odometry_node -> planning_node` appears
  while the system keeps running) and by `tests/test_baselines_evaluator.py`. It is **not**
  demonstrated by the accuracy numbers, and no such claim is made.
* Gazebo-specific: the backend runs headless with software OpenGL (`LIBGL_ALWAYS_SOFTWARE=1`), so
  the `gz sim` server is the dominant cost (~1.4 cores of an 8-core machine) and the measured ROS
  sensor rates sit a few percent below the SDF rates. The `ros_gz_bridge` topics live in the `/gz`
  namespace and are excluded from monitoring because they carry Gazebo *simulation-time* stamps;
  each sensor driver re-stamps with the ROS wall clock on receipt, so the measured latency is the
  real ROS-side pipeline delay and does **not** include the Gazebo-to-bridge hop.
* Gazebo-specific: the diagnostic monitor is attached `monitor_delay_sec` (60 s by default) after
  launch. Before that the robot has no trustworthy pose, so the planner publishes an empty path and
  the controller a zero command - the designed safety behaviour. A monitor started at t=0 learns
  that state as its nominal baseline and then reports ordinary driving as anomalous for the rest of
  the session. The baseline must be learned from nominal operation; this is a deployment setting,
  not a change to the detector.
* The IMU rate-reduction scenario (`imu_rate_drop`, 20 Hz -> 5 Hz) is below the detector's 0.18 s
  inter-arrival deviation floor and produces no evidence at all, so it is missed by every method.
  It is kept in the benchmark as a documented sensitivity limit rather than re-tuned.
* A node absent for longer than `forget_missing_after_sec` (60 s, ten times the RCA window) is
  forgotten instead of being re-reported as crashed forever. Within the RCA window crash evidence
  is produced exactly as before.
* Sub-10 ms onset differences between hops are within scheduling noise; temporal order is reported as
  supporting evidence only.
