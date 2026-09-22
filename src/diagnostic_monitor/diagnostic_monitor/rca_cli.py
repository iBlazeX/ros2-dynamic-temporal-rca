"""Command-line utility for inspecting the RCA event store: diagnoses, anomalies, graph."""

import argparse
import json
import os
import time
from .db_path import HELP_TEXT as DB_HELP, resolve_db_path
from .event_store import EventStore


def main():
    parser = argparse.ArgumentParser(description='RCA Diagnostic CLI')
    parser.add_argument('--db', type=str, default=None, help=DB_HELP)
    parser.add_argument(
        '--action', type=str, default='latest',
        choices=['latest', 'anomalies', 'diagnoses', 'summary', 'graph', 'timeline', 'watch'],
        help='Action to perform'
    )
    parser.add_argument('--last', type=float, default=30.0, help='Window in seconds for anomalies/timeline')
    parser.add_argument('--limit', type=int, default=40, help='Max rows to print')
    parser.add_argument('--json', action='store_true', help='Print raw JSON for latest diagnosis')
    args = parser.parse_args()

    db_path = resolve_db_path(explicit=args.db)
    if not os.path.exists(db_path):
        # Never silently create an empty store: that is exactly the bug where the
        # CLI reported zeros while the monitor was writing elsewhere.
        print(f'No event store found at {db_path}')
        print('Start the monitor first (ros2 launch rca_test_system system.launch.py), '
              'or point --db / $RCA_DB_PATH at the database the monitor is using.')
        return 2
    store = EventStore(db_path)
    now = time.time()
    if args.action != 'watch':
        print(f'[db: {db_path}]')

    if args.action == 'latest':
        latest = store.get_latest_diagnosis()
        if not latest:
            print('No diagnoses recorded in database yet.')
        elif args.json:
            print(json.dumps({k: v for k, v in latest.items() if not k.endswith('_json')}, indent=2))
        else:
            age = now - latest['timestamp']
            print(f"Latest Diagnosis (t={latest['timestamp']:.2f}, {age:.1f}s ago):")
            print(f"Root Cause: {latest['root_cause']} (Confidence: {latest['confidence']:.3f})")
            chain = latest['breakdown'].get('propagation_chain', [])
            if chain:
                print(f"Propagation chain: {' -> '.join(chain)}")
            print()
            print(latest['explanation'])

    elif args.action == 'anomalies':
        anoms = store.get_anomalies_between(now - args.last, now + 1e9)
        print(f'Anomalies in last {args.last:.0f}s: {len(anoms)}  (total in DB: {store.counts()["anomalies"]})')
        for a in anoms[-args.limit:]:
            print(f"[{a['timestamp']:.2f}] {a['node']:<18} {a['metric']:<34} val={a['value']:<8.3f} "
                  f"z={a['z_score']:<6.1f} sev={a['severity']:<5.2f} {a['description']}")

    elif args.action == 'diagnoses':
        rows = store.get_diagnoses_between(now - args.last, now + 1e9)
        print(f'Diagnoses in last {args.last:.0f}s: {len(rows)}')
        prev = None
        for r in rows[-args.limit:]:
            chain = ' -> '.join(r['breakdown'].get('propagation_chain', []))
            mark = '' if r['root_cause'] == prev else '  <- changed'
            print(f"t={r['timestamp']:.2f}  root={r['root_cause']:<18} conf={r['confidence']:.3f}  chain: {chain}{mark}")
            prev = r['root_cause']

    elif args.action == 'graph':
        snap = store.get_latest_graph()
        if not snap:
            print('No graph snapshot recorded yet.')
        else:
            g = snap['graph']
            print(f"Discovered graph (t={snap['timestamp']:.2f}):")
            print(f"  nodes: {g['nodes']}")
            for e in g['edges']:
                print(f"  {e['from']} -> {e['to']}   via {', '.join(e['topics'])}")

    elif args.action == 'timeline':
        anoms = store.get_anomalies_between(now - args.last, now + 1e9)
        diags = store.get_diagnoses_between(now - args.last, now + 1e9)
        events = [(a['timestamp'], 'ANOMALY', f"{a['node']:<18} {a['metric']}") for a in anoms]
        prev = None
        for d in diags:
            if d['root_cause'] != prev:
                events.append((d['timestamp'], 'DIAGNOSIS', f"root={d['root_cause']} conf={d['confidence']:.3f}"))
                prev = d['root_cause']
        events.sort()
        t0 = events[0][0] if events else now
        for t, kind, txt in events[-args.limit:]:
            print(f'+{t - t0:7.2f}s  {kind:<9} {txt}')

    elif args.action == 'summary':
        c = store.counts()
        print('=' * 40)
        print('RCA Event Store Summary')
        print('=' * 40)
        print(f"Telemetry Events:   {c['telemetry_events']}")
        print(f"Detected Anomalies: {c['anomalies']}")
        print(f"Diagnoses:          {c['diagnoses']}")
        print(f"Graph Snapshots:    {c['graph_snapshots']}")
        print('=' * 40)

    elif args.action == 'watch':
        # Live view: prints whenever the diagnosed root cause changes
        last_id = None
        prev_root = None
        print('Watching for diagnosis changes (Ctrl-C to stop)...')
        try:
            while True:
                latest = store.get_latest_diagnosis()
                if latest and latest['id'] != last_id:
                    last_id = latest['id']
                    if latest['root_cause'] != prev_root:
                        prev_root = latest['root_cause']
                        print(f"\n[t={latest['timestamp']:.2f}] {latest['explanation']}")
                elif not latest and prev_root is not None:
                    prev_root = None
                time.sleep(0.5)
        except KeyboardInterrupt:
            pass

    store.close()


if __name__ == '__main__':
    main()
