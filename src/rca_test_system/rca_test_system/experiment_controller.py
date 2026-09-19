"""Automated Experiment Controller and Benchmark Evaluator for RCA Research.

Orchestrates: settle -> inject fault -> observe cascade -> collect the monitor's
diagnoses from the event store -> evaluate against ground truth.

GROUND TRUTH ISOLATION: this module is the *evaluation layer*. It is the only
component that knows which node was faulted. The diagnostic_monitor never
receives this information (it does not subscribe to /rca/fault_command and the
graph discoverer excludes it).

Offline comparison: after each scenario, the exact same recorded evidence
(anomalies + discovered graph snapshot) is re-ranked by the baseline methods and
by ablated weight configurations, so the comparison is controlled and
deterministic.
"""

import argparse
import json
import os
import time
from typing import Any, Dict, List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from diagnostic_monitor.anomaly_detector import Anomaly
from diagnostic_monitor.baselines import BASELINE_NAMES, ABLATION_CONFIGS, run_ablations, run_all_baselines
from diagnostic_monitor.evaluator import RCAEvaluator
from diagnostic_monitor.event_store import EventStore
from diagnostic_monitor.graph_engine import DependencyGraph
from diagnostic_monitor.rca_engine import CandidateScore, DiagnosisResult


# ----------------------------------------------------------------------------
# Ground truth scenario definitions (EVALUATION LAYER ONLY).
# `expected_chain` is the set/order of nodes expected to show symptoms; it is
# used only to score propagation-chain accuracy, never given to the monitor.
# ----------------------------------------------------------------------------
SCENARIOS: List[Dict[str, Any]] = [
    dict(key='sensor_latency', name='Scenario 1: Sensor Latency',
         target='sensor_node', fault='latency', param=0.40,
         expected_chain=['sensor_node', 'perception_node', 'localization_node', 'navigation_node']),
    dict(key='localization_failure', name='Scenario 2: Localization Failure',
         target='localization_node', fault='localization_failure', param=50.0,
         expected_chain=['localization_node', 'navigation_node']),
    dict(key='sensor_dropout', name='Scenario 3: Sensor Dropout',
         target='sensor_node', fault='dropout', param=1.0,
         expected_chain=['sensor_node', 'perception_node', 'localization_node', 'navigation_node']),
    dict(key='navigation_failure', name='Scenario 4: Navigation Failure',
         target='navigation_node', fault='navigation_failure', param=1.0,
         expected_chain=['navigation_node']),
    dict(key='sensor_degradation', name='Scenario 5: Sensor Degradation (NaNs)',
         target='sensor_node', fault='degradation', param=0.0,
         # Observable chain (used for chain accuracy): the monitor has no pose
         # displacement metric, so localization's frozen pose is invisible to it.
         expected_chain=['sensor_node', 'perception_node'],
         # Physical chain (reported separately as coverage, never credited):
         # localization *is* affected (pose freezes after 3 empty inputs).
         physical_chain=['sensor_node', 'perception_node', 'localization_node']),
    dict(key='perception_delay', name='Scenario 6: Perception Processing Delay',
         target='perception_node', fault='processing_delay', param=0.40,
         expected_chain=['perception_node', 'localization_node', 'navigation_node']),
    # Crash is destructive (node does not come back) -> only run when requested, and last.
    dict(key='perception_crash', name='Scenario 7: Perception Node Crash',
         target='perception_node', fault='crash', param=0.0, destructive=True,
         expected_chain=['perception_node', 'localization_node', 'navigation_node']),
]

# Static "design-time" dependency chain used ONLY by the static-graph baselines
# (evaluation layer). The proposed method never sees this; it uses the graph
# discovered at runtime and recorded by the monitor.
STATIC_DESIGN_GRAPH = {
    'nodes': ['sensor_node', 'perception_node', 'localization_node', 'navigation_node'],
    'edges': [
        {'from': 'sensor_node', 'to': 'perception_node', 'topics': ['/sensor/scan']},
        {'from': 'perception_node', 'to': 'localization_node', 'topics': ['/perception/obstacles']},
        {'from': 'localization_node', 'to': 'navigation_node', 'topics': ['/localization/pose']},
    ],
}


def _diag_from_record(rec: Dict[str, Any]) -> DiagnosisResult:
    candidates = [
        CandidateScore(
            node=c['node'], total_score=c['total_score'], d_score=c['d_score'],
            t_score=c['t_score'], s_score=c['s_score'], a_score=c['a_score'],
            first_anomaly_time=c.get('first_anomaly_time'),
            direct_anomaly_count=c.get('direct_anomaly_count', 0),
            reachable_anomalies=set(c.get('reachable_anomalies', [])),
            upstream_anomalies=set(c.get('upstream_anomalies', [])),
        )
        for c in rec.get('rank_list', [])
    ]
    bd = rec.get('breakdown', {})
    return DiagnosisResult(
        timestamp=rec['timestamp'], root_cause=rec['root_cause'], confidence=rec['confidence'],
        candidates=candidates, anomalous_nodes=set(bd.get('anomalous_nodes', [])),
        weights={}, explanation=rec['explanation'],
        propagation_chain=bd.get('propagation_chain', []),
    )


class ExperimentController(Node):

    def __init__(self, db_path: str = 'events.db', k: int = 3):
        super().__init__('experiment_controller')
        self.cmd_pub = self.create_publisher(String, '/rca/fault_command', 10)
        self.db_path = db_path
        self.store = EventStore(db_path)
        self.k = k
        self.eval_results: List[Dict[str, Any]] = []
        self.baseline_results: List[Dict[str, Any]] = []
        self.ablation_results: List[Dict[str, Any]] = []
        self.rca_window_sec = 6.0   # must match the monitor's rca_window_sec parameter
        self.t_last_clear: Optional[float] = None
        # Fault-free observation windows measured from the monitor's event store:
        # [{'name', 't_start', 't_end', 'duration_sec', 'anomalies', 'diagnoses'}]
        self.nominal_windows: List[Dict[str, Any]] = []
        self.get_logger().info(f'ExperimentController ready with DB: {db_path}')

    # --------------------------------------------------------- nominal windows

    def record_nominal_window(self, name: str, t_start: float, t_end: float) -> Dict[str, Any]:
        """Counts monitor anomalies / diagnoses in a window during which no fault was active."""
        anoms = self.store.get_anomalies_between(t_start, t_end)
        diags = self.store.get_diagnoses_between(t_start, t_end)
        w = {
            'name': name, 't_start': t_start, 't_end': t_end,
            'duration_sec': round(max(0.0, t_end - t_start), 3),
            'anomalies': len(anoms), 'diagnoses': len(diags),
            'anomalous_nodes': sorted({a['node'] for a in anoms}),
            'diagnosed_roots': sorted({d['root_cause'] for d in diags}),
        }
        self.nominal_windows.append(w)
        return w

    def run_nominal_soak(self, soak_sec: float) -> Dict[str, Any]:
        """Observes the fault-free system for `soak_sec` and records false alarms.

        Runs inside the same benchmark execution so the nominal behaviour lands in
        the same artifact as the fault scenarios. The monitor is not told anything.
        """
        print('\n' + '=' * 72)
        print(f'NOMINAL SOAK: observing fault-free operation for {soak_sec:.0f}s')
        print('=' * 72)
        self.clear_faults()
        t0 = time.time()
        time.sleep(soak_sec)
        w = self.record_nominal_window('nominal_soak', t0, time.time())
        print(f"  duration {w['duration_sec']:.1f}s | anomalies {w['anomalies']} | "
              f"diagnoses (false alarms) {w['diagnoses']}"
              + (f" | anomalous nodes {w['anomalous_nodes']}" if w['anomalies'] else '')
              + (f" | diagnosed roots {w['diagnosed_roots']}" if w['diagnoses'] else ''))
        return w

    def compute_aggregate(self) -> Dict[str, Any]:
        """Single source of truth for the aggregate used by print_summary() and save()."""
        return RCAEvaluator.aggregate_metrics(self.eval_results, k=self.k,
                                              nominal_windows=self.nominal_windows)

    # ----------------------------------------------------------- fault control

    def _publish_cmd(self, payload: Dict[str, Any]):
        msg = String()
        msg.data = json.dumps(payload)
        for _ in range(4):
            self.cmd_pub.publish(msg)
            time.sleep(0.05)

    def inject_fault(self, target_node: str, fault_type: str, param: float, duration: float):
        self._publish_cmd({'target_node': target_node, 'fault_type': fault_type,
                           'param': param, 'duration': duration, 'timestamp': time.time()})
        self.get_logger().info(f'>>> Injected {fault_type} on {target_node} (param={param}, dur={duration}s)')

    def clear_faults(self):
        self._publish_cmd({'target_node': 'all', 'fault_type': 'clear', 'timestamp': time.time()})
        self.t_last_clear = time.time()
        self.get_logger().info('>>> Cleared all faults.')

    def wait_until_quiet(self, max_wait: float, quiet_sec: float = 2.0) -> bool:
        """Waits until the monitor has issued no diagnosis for `quiet_sec` (system nominal)."""
        start = time.time()
        while time.time() - start < max_wait:
            now = time.time()
            recent = self.store.get_diagnoses_between(now - quiet_sec, now + 1e9)
            if not recent and now - start >= quiet_sec:
                return True
            time.sleep(0.5)
        return False

    # -------------------------------------------------------------- scenario

    def run_scenario(self, sc: Dict[str, Any], observation_sec: float, duration: float,
                     settle_sec: float) -> Dict[str, Any]:
        name, target, fault, param = sc['name'], sc['target'], sc['fault'], sc['param']
        print('\n' + '=' * 72)
        print(f'STARTING {name}')
        print(f'Ground truth (evaluation layer only): target={target} fault={fault} param={param}')
        print('=' * 72)

        # 1. Clear lingering faults and wait for the monitor to go quiet
        self.clear_faults()
        t_settle_start = time.time()
        quiet = self.wait_until_quiet(max_wait=settle_sec + 10.0, quiet_sec=2.0)
        if not quiet:
            print('  (warning: monitor still reporting diagnoses before injection)')
        t_inject = time.time()
        # Fault-free (pre-injection) window: starts once the previous fault's evidence
        # has left the RCA window. Usually short; the dedicated nominal soak is the
        # main source of nominal evidence.
        nominal_start = max(t_settle_start, (self.t_last_clear or 0.0) + self.rca_window_sec + 1.0)
        nominal_w = self.record_nominal_window(f'pre_{sc["key"]}', nominal_start, t_inject)
        nominal_diags = nominal_w['diagnoses']
        nominal_len = nominal_w['duration_sec']

        # 2. Inject fault
        self.inject_fault(target, fault, param, duration)

        # 3. Observe cascade propagation
        print(f'Observing fault propagation for {observation_sec:.1f}s ...')
        time.sleep(observation_sec)
        t_end = time.time()

        # 4. Collect evidence recorded by the monitor in the injection window
        anoms = self.store.get_anomalies_between(t_inject, t_end)
        diags = self.store.get_diagnoses_between(t_inject, t_end)
        graph_snap = self.store.get_latest_graph(before_time=t_end)
        dyn_graph = DependencyGraph.from_dict(graph_snap['graph']) if graph_snap else DependencyGraph()

        first_anom_t = anoms[0]['timestamp'] if anoms else None
        first_diag_t = diags[0]['timestamp'] if diags else None
        first_correct_t = next((d['timestamp'] for d in diags if d['root_cause'] == target), None)
        roots_after = [d['root_cause'] for d in diags]

        if not diags:
            print('!! No diagnosis generated in the observation window.')
            res = {
                'scenario': name, 'key': sc['key'], 'ground_truth': target, 'fault_type': fault,
                'predicted_root_cause': None, 'top1_correct': False, f'top{self.k}_correct': False,
                'top3_correct': False, 'ground_truth_rank': -1, 'reciprocal_rank': 0.0,
                'detection_latency_sec': None, 'diagnosis_latency_sec': None,
                'correct_diagnosis_latency_sec': None, 'false_diagnosis_rate': None,
                'num_diagnoses_after_injection': 0, 'num_anomalies': len(anoms),
                'chain_accuracy': 0.0, 'chain_order_correct': False,
                'physical_chain_coverage': 0.0,
                'nominal_false_alarms': nominal_diags, 'nominal_window_sec': nominal_len,
                'explanation': 'No diagnosis recorded in window.',
            }
        else:
            final = _diag_from_record(diags[-1])
            res = RCAEvaluator.evaluate_single(
                diagnosis=final, ground_truth_node=target, fault_injection_time=t_inject,
                fault_type=fault, k=self.k, expected_chain=sc['expected_chain'],
                physical_chain=sc.get('physical_chain', sc['expected_chain']),
                first_anomaly_time=first_anom_t, first_diagnosis_time=first_diag_t,
                first_correct_diagnosis_time=first_correct_t, diagnoses_after_injection=roots_after,
            )
            res.update({'scenario': name, 'key': sc['key'], 'num_anomalies': len(anoms),
                        't_inject': t_inject, 't_end': t_end,
                        'nominal_false_alarms': nominal_diags, 'nominal_window_sec': nominal_len,
                        'explanation': final.explanation,
                        'anomalous_nodes_observed': sorted({a['node'] for a in anoms}),
                        'discovered_graph': graph_snap['graph'] if graph_snap else None})

            print(f'DIAGNOSIS: predicted root cause = [{res["predicted_root_cause"]}]  '
                  f'(ground truth = {target})  Top-1: {"PASS" if res["top1_correct"] else "FAIL"}  '
                  f'rank={res["ground_truth_rank"]}')
            print(f'  detection latency  : {res["detection_latency_sec"]}s  (first anomaly after injection)')
            print(f'  diagnosis latency  : {res["diagnosis_latency_sec"]}s  '
                  f'(correct root first named at {res["correct_diagnosis_latency_sec"]}s)')
            print(f'  false diag. rate   : {res["false_diagnosis_rate"]}  over {len(diags)} diagnoses')
            print(f'  chain accuracy     : {res["chain_accuracy"]} (observable chain {res["expected_chain"]})  '
                  f'predicted={res["predicted_chain"]}')
            if res['physical_chain'] != res['expected_chain']:
                print(f'  physical coverage  : {res["physical_chain_coverage"]} of physical chain {res["physical_chain"]}; '
                      f'NOT observed by any monitor metric: {res["physically_affected_unobserved"]}')
            print(f'  nominal false alarms before injection: {nominal_diags} (in {nominal_len:.1f}s fault-free window)')
            print('\n' + final.explanation)

            # 5. Offline controlled comparison on identical evidence
            self._offline_comparison(sc, anoms, dyn_graph, t_end)

        # 6. Clear fault and cool down
        self.clear_faults()
        self.eval_results.append(res)
        return res

    # ------------------------------------------------- baselines & ablations

    def _offline_comparison(self, sc, anom_dicts, dyn_graph: DependencyGraph, t_end: float):
        anomalies = [Anomaly.from_dict(a) for a in anom_dicts]
        static_graph = DependencyGraph.from_dict(STATIC_DESIGN_GRAPH)
        target = sc['target']
        window = t_end - min(a.onset for a in anomalies) + 1.0 if anomalies else 6.0

        b = run_all_baselines(anomalies, dyn_graph, static_graph, current_time=t_end, time_window_sec=window)
        row = {'scenario': sc['name'], 'key': sc['key'], 'ground_truth': target}
        print('\nBaseline comparison on identical recorded evidence:')
        for m in BASELINE_NAMES:
            rank = RCAEvaluator.rank_of(b[m], target)
            row[m] = {'ranking': b[m], 'rank': rank, 'top1': rank == 1}
            print(f'  {m:<16} top1={"PASS" if rank == 1 else "FAIL"} rank={rank:<3} ranking={b[m]}')
        self.baseline_results.append(row)

        ab = run_ablations(anomalies, dyn_graph, current_time=t_end, time_window_sec=window)
        row = {'scenario': sc['name'], 'key': sc['key'], 'ground_truth': target}
        print('Ablation (each scoring term disabled in turn):')
        for cfg in ABLATION_CONFIGS:
            rank = RCAEvaluator.rank_of(ab[cfg], target)
            row[cfg] = {'ranking': ab[cfg], 'rank': rank, 'top1': rank == 1}
            print(f'  {cfg:<8} top1={"PASS" if rank == 1 else "FAIL"} rank={rank:<3} ranking={ab[cfg]}')
        self.ablation_results.append(row)

    # ---------------------------------------------------------------- report

    def print_summary(self):
        print('\n' + '=' * 100)
        print('EXPERIMENT EVALUATION BENCHMARK RESULTS (proposed method, live ROS 2 run)')
        print('=' * 100)
        hdr = f'{"Scenario":<40} {"Truth":<18} {"Predicted":<18} {"Top1":<5} {"Rank":<4} {"Det.":<6} {"Diag.":<6} {"FDR":<6} {"Chain"}'
        print(hdr)
        print('-' * 100)
        for r in self.eval_results:
            print(f"{r['scenario']:<40} {r['ground_truth']:<18} {str(r['predicted_root_cause']):<18} "
                  f"{'PASS' if r['top1_correct'] else 'FAIL':<5} {str(r['ground_truth_rank']):<4} "
                  f"{str(r.get('detection_latency_sec')):<6} {str(r.get('diagnosis_latency_sec')):<6} "
                  f"{str(r.get('false_diagnosis_rate')):<6} {r.get('chain_accuracy')}")
        print('-' * 100)
        agg = self.compute_aggregate()
        print(json.dumps(agg, indent=2))

        if self.nominal_windows:
            print('\nNOMINAL (fault-free) WINDOWS observed in this run:')
            for w in self.nominal_windows:
                print(f"  {w['name']:<26} {w['duration_sec']:6.1f}s  anomalies={w['anomalies']:<3} "
                      f"diagnoses(false alarms)={w['diagnoses']}")
            print(f"  total fault-free time {agg['nominal_fault_free_sec']:.1f}s, "
                  f"false alarms {agg['nominal_false_alarms']} "
                  f"({agg['nominal_false_alarms_per_min']} per min)")
        unobs = [r for r in self.eval_results if r.get('physically_affected_unobserved')]
        if unobs:
            print('\nPHYSICALLY AFFECTED BUT UNOBSERVED NODES (not credited to the prediction):')
            for r in unobs:
                print(f"  {r['scenario']}: physical={r['physical_chain']} observable={r['expected_chain']} "
                      f"unobserved={r['physically_affected_unobserved']} "
                      f"(physical coverage {r['physical_chain_coverage']})")

        if self.baseline_results:
            print('\n' + '=' * 100)
            print('BASELINE COMPARISON (Top-1 accuracy across scenarios, identical evidence)')
            print('=' * 100)
            n = len(self.baseline_results)
            for m in BASELINE_NAMES:
                hits = sum(1 for r in self.baseline_results if r[m]['top1'])
                mrr = sum((1.0 / r[m]['rank']) if r[m]['rank'] > 0 else 0.0 for r in self.baseline_results) / n
                print(f'  {m:<16} top1 = {hits}/{n} ({100.0*hits/n:.0f}%)   MRR = {mrr:.3f}')

        if self.ablation_results:
            print('\n' + '=' * 100)
            print('ABLATION (Top-1 accuracy with each term disabled)')
            print('=' * 100)
            n = len(self.ablation_results)
            for cfg, w in ABLATION_CONFIGS.items():
                hits = sum(1 for r in self.ablation_results if r[cfg]['top1'])
                print(f'  {cfg:<8} {w}  top1 = {hits}/{n} ({100.0*hits/n:.0f}%)')
        return agg

    def save(self, path: str):
        out = {
            'generated_at': time.time(),
            'db_path': self.db_path,
            'results': self.eval_results,
            'aggregate': self.compute_aggregate(),
            'nominal_windows': self.nominal_windows,
            'baselines': self.baseline_results,
            'ablations': self.ablation_results,
        }
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        with open(path, 'w') as f:
            json.dump(out, f, indent=2, default=str)
        print(f'\nResults written to {path}')


def main(args=None):
    parser = argparse.ArgumentParser(description='RCA Experiment Controller')
    parser.add_argument('--db', type=str, default='events.db', help='Path to SQLite database')
    parser.add_argument('--warmup', type=float, default=6.0, help='Initial warm-up wait time in seconds')
    parser.add_argument('--observation', type=float, default=6.0, help='Observation window after injection (s)')
    parser.add_argument('--duration', type=float, default=9.0, help='Fault duration (s), > observation')
    parser.add_argument('--settle', type=float, default=8.0, help='Max settle time between scenarios (s)')
    parser.add_argument('--k', type=int, default=3, help='k for Top-k accuracy')
    parser.add_argument('--soak', type=float, default=30.0,
                        help='Fault-free nominal soak duration (s) run before the scenarios; 0 disables')
    parser.add_argument('--scenarios', type=str, default='',
                        help='Comma-separated scenario keys to run (default: all non-destructive)')
    parser.add_argument('--include-crash', action='store_true', help='Also run the destructive crash scenario last')
    parser.add_argument('--out', type=str, default='runs/results.json', help='JSON output path')
    parsed, ros_args = parser.parse_known_args()

    selected = [s for s in SCENARIOS if not s.get('destructive')]
    if parsed.scenarios:
        keys = [k.strip() for k in parsed.scenarios.split(',') if k.strip()]
        selected = [s for s in SCENARIOS if s['key'] in keys]
    if parsed.include_crash and not any(s.get('destructive') for s in selected):
        selected += [s for s in SCENARIOS if s.get('destructive')]

    rclpy.init(args=ros_args)
    controller = ExperimentController(db_path=parsed.db, k=parsed.k)

    print(f'Waiting {parsed.warmup}s for nominal system warm-up and graph discovery...')
    t_w0 = time.time()
    time.sleep(parsed.warmup)
    w = controller.record_nominal_window('warmup', t_w0, time.time())
    print(f"Fault-free warm-up {w['duration_sec']:.1f}s: anomalies {w['anomalies']}, "
          f"diagnoses (false alarms) {w['diagnoses']}")
    snap = controller.store.get_latest_graph()
    if snap:
        g = snap['graph']
        print(f"Monitor's discovered graph: {g['nodes']}  "
              f"edges: {[(e['from'], e['to']) for e in g['edges']]}")
    else:
        print('WARNING: monitor has not recorded a graph snapshot yet.')

    if parsed.soak > 0:
        controller.run_nominal_soak(parsed.soak)

    for sc in selected:
        controller.run_scenario(sc, observation_sec=parsed.observation,
                                duration=parsed.duration, settle_sec=parsed.settle)

    controller.print_summary()
    controller.save(parsed.out)

    controller.store.close()
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
