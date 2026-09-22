import json
import sys
from diagnostic_monitor.event_store import EventStore
from diagnostic_monitor.db_path import resolve_db_path

s = EventStore(resolve_db_path())
snap = s.get_latest_graph()
if snap:
    g = snap['graph']
    print('nodes:', sorted(g['nodes']))
    print('edges:', sorted((e['from'], e['to']) for e in g['edges']))
else:
    print('no graph snapshot')
print('counts:', s.counts())
anoms = s.get_anomalies_between(0, 1e18)
diags = s.get_diagnoses_between(0, 1e18)
print('anomalies:', len(anoms), 'diagnoses:', len(diags))
for a in anoms[-10:]:
    print('  A', a['node'], a['metric'], round(a['severity'], 2), a['description'][:70])
for d in diags[-5:]:
    print('  D', d['root_cause'], round(d['confidence'], 3))
s.close()
