#!/usr/bin/env python3
"""Prints an overhead artifact as a table with explicit CPU normalisation."""
import json
import sys

GROUPS = ('rca_sim', 'gazebo', 'pipeline', 'diagnostic_monitor', 'tui')


def main(path):
    data = json.load(open(path))
    if data:
        m = data[0]['machine']
        print(f"machine: {m['logical_cores']} logical cores ({100 * m['logical_cores']} % total "
              f"capacity), {m['cpu_model']}, {m['mem_total_mb']} MB RAM")
        print(f"method:  {data[0]['method']}\n")
    hdr = f"{'configuration':<30}"
    for g in GROUPS:
        hdr += f" | {g[:12]:<12}"
    print(hdr)
    print('-' * len(hdr))
    for e in data:
        line = f"{e['label']:<30}"
        for g in GROUPS:
            p = e['processes'].get(g, {})
            if not p or p.get('processes', 0) == 0:
                line += f" | {'-':<12}"
            else:
                line += f" | {p['processes']}p {p['cpu_percent_sum']:5.1f}%"
        print(line)
    print('\nraw CPU % (one busy core = 100 %) / core-equivalents / % of machine capacity / RSS MB:')
    for e in data:
        print(f"  {e['label']}")
        for g in GROUPS:
            p = e['processes'].get(g, {})
            if not p or p.get('processes', 0) == 0:
                continue
            print(f"    {g:<20} {p['processes']}p  {p['cpu_percent_sum']:6.1f} % raw  "
                  f"{p['core_equivalents']:5.2f} cores  {p['percent_of_machine']:5.1f} % of machine  "
                  f"{p['rss_mb_sum_mean']:7.1f} MB  pids {p['pids']}")
        for key in ('event_store', 'dashboard_topic', 'rca_compute'):
            if e.get(key):
                print(f"    {key:<20} {json.dumps(e[key])}")


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else 'runs/gazebo/overhead.json')
