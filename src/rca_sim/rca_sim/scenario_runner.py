"""Simulation scenario runner and evaluation layer (rca_sim).

Reads config/scenarios.yaml, drives the running simulation through
``/rca/fault_command`` (real runtime degradation, no labels anywhere the monitor
can see), collects the diagnostic monitor's recorded evidence from the event
store and evaluates it with the EXISTING evaluator, baselines and ablations.

This is the only place in rca_sim that knows the ground truth.

    ros2 launch rca_sim sim_system.launch.py          # terminal 1
    ros2 run rca_sim scenario_runner --repetitions 5   # terminal 2

Results go to runs/simulation/<timestamp>/results.json (never overwritten).
"""

import argparse
import copy
import json
import os
import time
from datetime import datetime
from typing import Any, Dict, List, Optional

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory

from diagnostic_monitor.anomaly_detector import Anomaly
from diagnostic_monitor.baselines import ABLATION_CONFIGS, BASELINE_NAMES, run_ablations, run_all_baselines
from diagnostic_monitor.db_path import HELP_TEXT as DB_HELP, resolve_db_path
from diagnostic_monitor.evaluator import RCAEvaluator
from diagnostic_monitor.graph_engine import DependencyGraph
from rca_test_system.experiment_controller import ExperimentController, _diag_from_record


def load_scenarios(path: Optional[str] = None) -> Dict[str, Any]:
    if path is None:
        path = os.path.join(get_package_share_directory('rca_sim'), 'config', 'scenarios.yaml')
    with open(path) as f:
        cfg = yaml.safe_load(f)
    defaults = cfg.get('defaults', {})
    for sc in cfg['scenarios']:
        sc.setdefault('observation_sec', defaults.get('observation_sec', 8.0))
        sc.setdefault('group', 'single')
        sc.setdefault('benchmark', True)
        for ft in sc['faults']:
            ft.setdefault('duration', defaults.get('fault_duration', 10.0))
            ft.setdefault('onset', 0.0)
            ft.setdefault('param', 0.0)
        gt = sc['ground_truth']
        gt.setdefault('accepted_roots', [gt['root']] if gt.get('root') else [])
        gt.setdefault('expect_no_diagnosis', False)
        gt.setdefault('observable_chain', [])
        gt.setdefault('physical_chain', gt['observable_chain'])
    return cfg


def rank_of_any(ranking: List[str], accepted: List[str]) -> int:
    """Best (lowest) rank among the accepted roots, -1 if none is ranked."""
    ranks = [RCAEvaluator.rank_of(ranking, r) for r in accepted]
    ranks = [r for r in ranks if r > 0]
    return min(ranks) if ranks else -1


class SimScenarioRunner(ExperimentController):
    """Reuses the controlled-benchmark controller (fault channel, nominal windows,
    evidence collection, aggregate persistence) for YAML-driven simulation scenarios."""

    def __init__(self, db_path: str, cfg: Dict[str, Any], k: int = 3, simulator: str = None):
        super().__init__(db_path=db_path, k=k)
        self.cfg = cfg
        # Which backend produced the evidence. Recorded in every artifact so
        # Gazebo results are never pooled with rca_sim results by accident.
        self.simulator = simulator or str(cfg.get('environment', {}).get('simulator', 'unknown'))
        self.static_graph = DependencyGraph.from_dict(cfg['environment']['static_design_graph'])
        self.settle_sec = float(cfg.get('defaults', {}).get('settle_sec', 25.0))

    # ------------------------------------------------------------- scenario

    def run_sim_scenario(self, sc: Dict[str, Any], rep: int, launch_seed: int) -> Dict[str, Any]:
        gt = sc['ground_truth']
        accepted = list(gt['accepted_roots'])
        print('\n' + '=' * 78)
        print(f'{sc["name"]}   [rep {rep}]')
        print(f'  faults: ' + '; '.join(f'{f["target"]}:{f["type"]}(p={f["param"]}, +{f["onset"]}s, {f["duration"]}s)'
                                        for f in sc['faults']))
        print('=' * 78)

        self.clear_faults()
        t_settle = time.time()
        if not self.wait_until_quiet(max_wait=self.settle_sec, quiet_sec=2.0):
            print('  (warning: monitor still reporting diagnoses before injection)')
        t_inject = time.time()
        nominal_start = max(t_settle, (self.t_last_clear or 0.0) + self.rca_window_sec + 1.0)
        nominal_w = self.record_nominal_window(f'pre_{sc["key"]}_r{rep}', nominal_start, t_inject)

        # inject faults at their onsets (real runtime mechanisms)
        for ft in sorted(sc['faults'], key=lambda f: f['onset']):
            wait = t_inject + float(ft['onset']) - time.time()
            if wait > 0:
                time.sleep(wait)
            self.inject_fault(ft['target'], ft['type'], float(ft['param']), float(ft['duration']))
        obs = float(sc['observation_sec'])
        rest = t_inject + obs - time.time()
        if rest > 0:
            time.sleep(rest)
        t_end = time.time()

        anoms = self.store.get_anomalies_between(t_inject, t_end)
        diags = self.store.get_diagnoses_between(t_inject, t_end)
        snap = self.store.get_latest_graph(before_time=t_end)
        dyn_graph = DependencyGraph.from_dict(snap['graph']) if snap else DependencyGraph()
        roots_after = [d['root_cause'] for d in diags]

        res: Dict[str, Any] = {
            'scenario': sc['name'], 'key': sc['key'], 'group': sc['group'], 'rep': rep,
            'launch_seed': launch_seed, 'faults': sc['faults'],
            'ground_truth': gt.get('root'), 'accepted_roots': accepted,
            'expect_no_diagnosis': bool(gt['expect_no_diagnosis']),
            't_inject': t_inject, 't_end': t_end, 'num_anomalies': len(anoms),
            'num_diagnoses_after_injection': len(diags),
            'anomalous_nodes_observed': sorted({a['node'] for a in anoms}),
            'nominal_false_alarms': nominal_w['diagnoses'], 'nominal_window_sec': nominal_w['duration_sec'],
            'discovered_graph': snap['graph'] if snap else None,
        }

        if gt['expect_no_diagnosis']:
            # a pure graph change: correct iff the monitor raised no diagnosis at all
            res.update({
                'predicted_root_cause': roots_after[-1] if roots_after else None,
                'top1_correct': not diags, f'top{self.k}_correct': not diags, 'top3_correct': not diags,
                'ground_truth_rank': 1 if not diags else -1, 'reciprocal_rank': 1.0 if not diags else 0.0,
                'detection_latency_sec': None, 'diagnosis_latency_sec': None,
                'correct_diagnosis_latency_sec': None,
                'false_diagnosis_rate': (1.0 if diags else 0.0),
                'chain_accuracy': 1.0 if not diags else 0.0, 'chain_order_correct': not diags,
                'physical_chain_coverage': 1.0 if not diags else 0.0,
                'predicted_chain': [], 'expected_chain': [], 'physical_chain': [],
                'physically_affected_unobserved': [],
                'explanation': diags[-1]['explanation'] if diags else 'no diagnosis (expected)',
                'num_false_diagnoses_after_injection': len(diags),
                'evaluated_offline': False,
                'exclusion_reason': 'scenario declares no ground-truth root cause (expect_no_diagnosis)',
            })
            print(f'  graph-change scenario: diagnoses raised = {len(diags)}  -> '
                  f'{"PASS" if not diags else "FALSE ALARM: " + str(sorted(set(roots_after)))}')
        elif not diags:
            res.update({
                'predicted_root_cause': None, 'top1_correct': False, f'top{self.k}_correct': False,
                'top3_correct': False, 'ground_truth_rank': -1, 'reciprocal_rank': 0.0,
                'detection_latency_sec': round(anoms[0]['timestamp'] - t_inject, 3) if anoms else None,
                'diagnosis_latency_sec': None, 'correct_diagnosis_latency_sec': None,
                'false_diagnosis_rate': None, 'chain_accuracy': 0.0, 'chain_order_correct': False,
                'physical_chain_coverage': 0.0, 'predicted_chain': [],
                'expected_chain': gt['observable_chain'], 'physical_chain': gt['physical_chain'],
                'physically_affected_unobserved': list(gt['physical_chain']),
                'explanation': 'No diagnosis recorded in window.',
                'num_false_diagnoses_after_injection': 0,
                'evaluated_offline': True,
            })
            print('  !! no diagnosis in the observation window')
            # Scored offline anyway: the comparison cohort must not depend on
            # whether the proposed method happened to produce a diagnosis.
            self._offline_comparison_sim(sc, rep, anoms, dyn_graph, t_end, accepted)
        else:
            final = _diag_from_record(diags[-1])
            first_correct = next((d['timestamp'] for d in diags if d['root_cause'] in accepted), None)
            res.update(RCAEvaluator.evaluate_single(
                diagnosis=final, ground_truth_node=gt['root'], fault_injection_time=t_inject,
                fault_type='+'.join(f['type'] for f in sc['faults']), k=self.k,
                expected_chain=gt['observable_chain'], physical_chain=gt['physical_chain'],
                first_anomaly_time=anoms[0]['timestamp'] if anoms else None,
                first_diagnosis_time=diags[0]['timestamp'], first_correct_diagnosis_time=first_correct,
                diagnoses_after_injection=roots_after,
            ))
            # multiple injected roots: any accepted root counts (evaluation-layer rule)
            ranking = res['ranking']
            rank = rank_of_any(ranking, accepted)
            wrong = sum(1 for r in roots_after if r not in accepted)
            res.update({
                'ground_truth_rank': rank, 'top1_correct': rank == 1,
                f'top{self.k}_correct': 0 < rank <= self.k, 'top3_correct': 0 < rank <= 3,
                'reciprocal_rank': round(1.0 / rank, 4) if rank > 0 else 0.0,
                'false_diagnosis_rate': round(wrong / len(roots_after), 4),
                'explanation': final.explanation,
                'num_false_diagnoses_after_injection': wrong,
                'evaluated_offline': True,
            })
            print(f'  predicted={res["predicted_root_cause"]}  accepted={accepted}  '
                  f'top1={"PASS" if res["top1_correct"] else "FAIL"} rank={rank}  '
                  f'det={res["detection_latency_sec"]}s diag={res["diagnosis_latency_sec"]}s '
                  f'FDR={res["false_diagnosis_rate"]} chain_acc={res["chain_accuracy"]} '
                  f'phys_cov={res["physical_chain_coverage"]}')
            print(f'  anomalous nodes: {res["anomalous_nodes_observed"]}  chain: {res["predicted_chain"]}')
            self._offline_comparison_sim(sc, rep, anoms, dyn_graph, t_end, accepted)

        self.clear_faults()
        self.eval_results.append(res)
        return res

    # ------------------------------------------------ baselines & ablations

    def _offline_comparison_sim(self, sc, rep, anom_dicts, dyn_graph, t_end, accepted):
        anomalies = [Anomaly.from_dict(a) for a in anom_dicts]
        window = t_end - min(a.onset for a in anomalies) + 1.0 if anomalies else 6.0
        b = run_all_baselines(anomalies, dyn_graph, self.static_graph, current_time=t_end, time_window_sec=window)
        row = {'scenario': sc['name'], 'key': sc['key'], 'rep': rep, 'ground_truth': sc['ground_truth'].get('root'),
               'accepted_roots': accepted}
        print('  baselines (identical evidence):', end='')
        for m in BASELINE_NAMES:
            rank = rank_of_any(b[m], accepted)
            row[m] = {'ranking': b[m], 'rank': rank, 'top1': rank == 1}
            print(f'  {m}={"PASS" if rank == 1 else "FAIL"}', end='')
        print()
        self.baseline_results.append(row)

        ab = run_ablations(anomalies, dyn_graph, current_time=t_end, time_window_sec=window)
        row = {'scenario': sc['name'], 'key': sc['key'], 'rep': rep, 'ground_truth': sc['ground_truth'].get('root'),
               'accepted_roots': accepted}
        print('  ablation:', end='')
        for cfg in ABLATION_CONFIGS:
            rank = rank_of_any(ab[cfg], accepted)
            row[cfg] = {'ranking': ab[cfg], 'rank': rank, 'top1': rank == 1}
            print(f'  {cfg}={"PASS" if rank == 1 else "FAIL"}', end='')
        print()
        self.ablation_results.append(row)

    # ---------------------------------------------------------------- report

    def per_scenario_summary(self) -> List[Dict[str, Any]]:
        out = []
        for key in dict.fromkeys(r['key'] for r in self.eval_results):
            rs = [r for r in self.eval_results if r['key'] == key]
            def avg(k):
                v = [r[k] for r in rs if r.get(k) is not None]
                return round(sum(v) / len(v), 3) if v else None
            row = {'key': key, 'scenario': rs[0]['scenario'], 'group': rs[0]['group'], 'runs': len(rs),
                   'top1': sum(1 for r in rs if r['top1_correct']), 'top3': sum(1 for r in rs if r['top3_correct']),
                   'mrr': avg('reciprocal_rank'), 'detection_latency': avg('detection_latency_sec'),
                   'diagnosis_latency': avg('diagnosis_latency_sec'), 'fdr': avg('false_diagnosis_rate'),
                   'chain_accuracy': avg('chain_accuracy'), 'physical_coverage': avg('physical_chain_coverage')}
            bs = [b for b in self.baseline_results if b['key'] == key]
            row['evaluated_offline'] = len(bs)      # 0 for no-evidence (graph-change) scenarios
            for m in BASELINE_NAMES:
                row[f'baseline_{m}_top1'] = sum(1 for b in bs if b[m]['top1']) if bs else None
            abs_ = [b for b in self.ablation_results if b['key'] == key]
            for cfg in ABLATION_CONFIGS:
                row[f'ablation_{cfg}_top1'] = sum(1 for b in abs_ if b[cfg]['top1']) if abs_ else None
            out.append(row)
        return out

    def print_sim_summary(self):
        print('\n' + '=' * 118)
        print('SIMULATION BENCHMARK (proposed method, live; baselines/ablations on identical recorded evidence)')
        print('=' * 118)
        hdr = (f'{"scenario":<38} {"n":>2} {"top1":>4} {"top3":>4} {"MRR":>5} {"det":>5} {"diag":>5} {"FDR":>5} '
               f'{"chain":>5} {"phys":>5} | ' + ' '.join(f'{m[:6]:>6}' for m in BASELINE_NAMES) + ' | '
               + ' '.join(f'{c:>5}' for c in ABLATION_CONFIGS))
        print(hdr)
        print('-' * 118)
        for r in self.per_scenario_summary():
            print(f'{r["key"]:<38} {r["runs"]:>2} {r["top1"]:>4} {r["top3"]:>4} {str(r["mrr"]):>5} '
                  f'{str(r["detection_latency"]):>5} {str(r["diagnosis_latency"]):>5} {str(r["fdr"]):>5} '
                  f'{str(r["chain_accuracy"]):>5} {str(r["physical_coverage"]):>5} | '
                  + ' '.join(f'{str(r[f"baseline_{m}_top1"] if r[f"baseline_{m}_top1"] is not None else "-"):>6}'
                             for m in BASELINE_NAMES) + ' | '
                  + ' '.join(f'{str(r[f"ablation_{c}_top1"] if r[f"ablation_{c}_top1"] is not None else "-"):>5}'
                             for c in ABLATION_CONFIGS))
        print('-' * 118)
        agg = self.compute_aggregate()
        print(json.dumps(agg, indent=2))
        self.print_cohort(self.compute_cohort())
        n = len(self.baseline_results)
        if n:
            print('\nBASELINES (Top-1 over all evaluated runs, identical evidence):')
            for m in BASELINE_NAMES:
                hits = sum(1 for b in self.baseline_results if b[m]['top1'])
                mrr = sum((1.0 / b[m]['rank']) if b[m]['rank'] > 0 else 0.0 for b in self.baseline_results) / n
                print(f'  {m:<16} top1 = {hits}/{n} ({100.0 * hits / n:.0f}%)  MRR = {mrr:.3f}')
            print('ABLATIONS (Top-1 over all evaluated runs):')
            for cfg, w in ABLATION_CONFIGS.items():
                hits = sum(1 for b in self.ablation_results if b[cfg]['top1'])
                print(f'  {cfg:<8} {w}  top1 = {hits}/{n} ({100.0 * hits / n:.0f}%)')
        if self.nominal_windows:
            print(f'\nNOMINAL: {agg["nominal_fault_free_sec"]:.1f}s fault-free observed, '
                  f'{agg["nominal_anomalies"]} anomalies, {agg["nominal_false_alarms"]} false alarms '
                  f'({agg["nominal_false_alarms_per_min"]}/min)')
        return agg

    def save_sim(self, path: str, launch_seed: int):
        out = {
            'generated_at': time.time(),
            'simulator': self.simulator,
            'environment': self.cfg['environment'],
            'launch_seed': launch_seed,
            'db_path': self.db_path,
            'results': self.eval_results,
            'per_scenario': self.per_scenario_summary(),
            'aggregate': self.compute_aggregate(),
            'cohorts': self.compute_cohort(),
            'nominal_windows': self.nominal_windows,
            'baselines': self.baseline_results,
            'ablations': self.ablation_results,
        }
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        with open(path, 'w') as f:
            json.dump(out, f, indent=2, default=str)
        print(f'\nResults written to {path}')


def main(args=None, argv_defaults=None):
    d = argv_defaults or {}
    parser = argparse.ArgumentParser(description='Scenario runner (evaluation layer)')
    parser.add_argument('--db', type=str, default=None, help=DB_HELP)
    parser.add_argument('--scenarios-file', type=str, default=d.get('scenarios_file'),
                        help='YAML scenario file (default: package config)')
    parser.add_argument('--simulator', type=str, default=d.get('simulator'),
                        help='Backend label recorded in the artifacts (default: from the YAML environment)')
    parser.add_argument('--benchmark-only', action='store_true',
                        help='Run only the scenarios flagged benchmark: true')
    parser.add_argument('--scenarios', type=str, default='', help='Comma-separated scenario keys (default: all)')
    parser.add_argument('--groups', type=str, default='', help='Comma-separated groups to run (default: all)')
    parser.add_argument('--repetitions', type=int, default=None, help='Repetitions per scenario (default: YAML)')
    parser.add_argument('--warmup', type=float, default=15.0, help='Fault-free warm-up (s)')
    parser.add_argument('--soak', type=float, default=30.0, help='Fault-free nominal soak (s), 0 disables')
    parser.add_argument('--launch-seed', type=int, default=7, help='Seed the simulation was launched with (recorded)')
    parser.add_argument('--out', type=str, default=None,
                        help='Output JSON (default: runs/simulation/<timestamp>/results.json)')
    parser.add_argument('--k', type=int, default=3)
    parsed, ros_args = parser.parse_known_args()

    cfg = load_scenarios(parsed.scenarios_file)
    scenarios = cfg['scenarios']
    if parsed.scenarios:
        keys = [k.strip() for k in parsed.scenarios.split(',') if k.strip()]
        scenarios = [s for s in scenarios if s['key'] in keys]
    if parsed.groups:
        groups = [g.strip() for g in parsed.groups.split(',') if g.strip()]
        scenarios = [s for s in scenarios if s['group'] in groups]
    if parsed.benchmark_only:
        scenarios = [s for s in scenarios if s.get('benchmark', True)]
    reps = parsed.repetitions or int(cfg.get('defaults', {}).get('repetitions', 5))
    out_dir = d.get('out_dir', os.path.join('runs', 'simulation'))
    out = parsed.out or os.path.join(out_dir, datetime.now().strftime('%Y%m%d_%H%M%S'), 'results.json')

    rclpy.init(args=ros_args)
    db_path = resolve_db_path(explicit=parsed.db)
    print(f'Event store: {db_path}')
    runner = SimScenarioRunner(db_path=db_path, cfg=cfg, k=parsed.k, simulator=parsed.simulator)
    print(f'Simulator backend: {runner.simulator}')

    print(f'Waiting {parsed.warmup}s (fault-free warm-up)...')
    t0 = time.time()
    time.sleep(parsed.warmup)
    w = runner.record_nominal_window('warmup', t0, time.time())
    print(f"warm-up {w['duration_sec']:.1f}s: anomalies {w['anomalies']}, diagnoses {w['diagnoses']}")
    snap = runner.store.get_latest_graph()
    if snap:
        g = snap['graph']
        print(f"discovered graph: {len(g['nodes'])} nodes {len(g['edges'])} edges: "
              f"{[(e['from'], e['to']) for e in g['edges']]}")
    if parsed.soak > 0:
        runner.run_nominal_soak(parsed.soak)

    for rep in range(reps):
        for sc in scenarios:
            runner.run_sim_scenario(copy.deepcopy(sc), rep, parsed.launch_seed)
            runner.save_sim(out, parsed.launch_seed)      # incremental: nothing lost on interruption

    runner.print_sim_summary()
    runner.save_sim(out, parsed.launch_seed)
    runner.store.close()
    runner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
