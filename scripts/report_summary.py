#!/usr/bin/env python3
"""Renders a benchmark artifact as the Markdown tables used in the README.

Everything printed here is read back from the saved results.json, so the README
cannot drift from the artifact.

    python3 scripts/report_summary.py runs/gazebo/bench2/results.json
"""
import json
import sys

from diagnostic_monitor.baselines import ABLATION_CONFIGS, BASELINE_NAMES


def fmt(v, nd=2):
    if v is None:
        return '-'
    if isinstance(v, float):
        return f'{v:.{nd}f}'
    return str(v)


def main(path):
    doc = json.load(open(path))
    agg, co = doc['aggregate'], doc['cohorts']
    n = agg['total_experiments']
    print(f"### Artifact `{path}`  (simulator: **{doc.get('simulator')}**)\n")

    print('| quantity | numerator | denominator | value |')
    print('|---|---|---|---|')
    print(f"| Top-1 accuracy | {agg['top1_correct']} | {n} runs | {fmt(agg['top1_accuracy'], 4)} |")
    print(f"| Top-3 accuracy | {agg['top3_correct']} | {n} runs | {fmt(agg['top3_accuracy'], 4)} |")
    print(f"| mean reciprocal rank | sum of 1/rank | {n} runs | {fmt(agg['mean_reciprocal_rank'], 4)} |")
    print(f"| post-injection false-diagnosis rate | {agg['post_injection_false_diagnoses']} "
          f"| {agg['post_injection_diagnoses']} diagnoses | {fmt(agg['post_injection_false_diagnosis_rate'], 4)} |")
    print(f"| nominal false alarms | {agg['nominal_false_alarms']} | "
          f"{fmt(agg['nominal_fault_free_sec'], 1)} s fault-free in {agg['nominal_fault_free_windows']} windows "
          f"| {fmt(agg['nominal_false_alarm_rate_per_min'], 4)} /min |")
    print(f"| observable chain accuracy (mean) | - | {agg['chain_accuracy_n']} runs "
          f"| {fmt(agg['avg_chain_accuracy'], 3)} |")
    print(f"| chain order accuracy | {agg['chain_order_correct_count']} | {agg['chain_order_n']} runs "
          f"| {fmt(agg['chain_order_accuracy'], 3)} |")
    print(f"| physical chain coverage (mean) | - | {agg['physical_chain_coverage_n']} runs "
          f"| {fmt(agg['avg_physical_chain_coverage'], 3)} |")

    print('\n| latency (s) | n | mean | min | p25 | median | p75 | max | std |')
    print('|---|---|---|---|---|---|---|---|---|')
    for label, key in (('detection', 'detection_latency_stats'),
                       ('diagnosis (any root)', 'diagnosis_latency_stats'),
                       ('diagnosis (correct root)', 'correct_diagnosis_latency_stats')):
        s = agg[key]
        print(f"| {label} | {s['n']} | {fmt(s['mean'], 3)} | {fmt(s['min'], 3)} | {fmt(s['p25'], 3)} "
              f"| {fmt(s['median'], 3)} | {fmt(s['p75'], 3)} | {fmt(s['max'], 3)} | {fmt(s['std'], 3)} |")

    a, s, x = co['all_runs'], co['common_evidence_subset'], co['excluded_from_common_subset']
    print('\n| cohort | n | Top-1 | accuracy |')
    print('|---|---|---|---|')
    print(f"| ALL RUNS (proposed, live) | {a['n']} | {a['proposed_top1_correct']} "
          f"| {fmt(a['proposed_top1_accuracy'], 4)} |")
    print(f"| COMMON-EVIDENCE SUBSET (proposed) | {s['n']} | {s['proposed_top1_correct']} "
          f"| {fmt(s['proposed_top1_accuracy'], 4)} |")
    for m in BASELINE_NAMES:
        b = s['baselines'][m]
        print(f"| &nbsp;&nbsp;baseline `{m}` | {b['n']} | {b['top1_correct']} "
              f"| {fmt(b['top1_accuracy'], 4)} (MRR {fmt(b['mean_reciprocal_rank'], 3)}) |")
    print(f"\nExcluded from the common-evidence subset: **{x['n']}** run(s); "
          f"reasons: {x['reasons'] or 'none'}; scenarios: {x['keys'] or 'none'}. "
          f"Denominators match: **{co['denominators_match']}**.")

    nb = len(doc['ablations'])
    if nb:
        print('\n| ablation | weights | Top-1 |')
        print('|---|---|---|')
        for cfg, w in ABLATION_CONFIGS.items():
            hits = sum(1 for r in doc['ablations'] if r[cfg]['top1'])
            print(f"| `{cfg}` | {w} | {hits}/{nb} ({100.0 * hits / nb:.0f} %) |")

    print('\n| scenario | runs | Top-1 | Top-3 | MRR | det. (s) | diag. (s) | chain | phys. cov. |')
    print('|---|---|---|---|---|---|---|---|---|')
    for r in doc['per_scenario']:
        print(f"| `{r['key']}` | {r['runs']} | {r['top1']} | {r['top3']} | {fmt(r['mrr'], 3)} "
              f"| {fmt(r['detection_latency'], 3)} | {fmt(r['diagnosis_latency'], 3)} "
              f"| {fmt(r['chain_accuracy'], 2)} | {fmt(r['physical_coverage'], 2)} |")

    misses = [r for r in doc['results'] if not r['top1_correct']]
    if misses:
        print(f"\n**Misses ({len(misses)}/{n}):**\n")
        for r in misses:
            print(f"* `{r['key']}` rep {r['rep']}: predicted `{r['predicted_root_cause']}` "
                  f"(truth `{r.get('ground_truth')}`), anomalous nodes observed "
                  f"{r.get('anomalous_nodes_observed')}")
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1]))
