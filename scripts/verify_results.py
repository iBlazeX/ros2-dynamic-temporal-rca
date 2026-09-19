"""Cross-checks printed summary, saved JSON and SQLite DB for a benchmark run."""
import json, re, sqlite3, sys
log_path, json_path, db_path = sys.argv[1:4]
log = open(log_path).read()
R = json.load(open(json_path))
db = sqlite3.connect(db_path); db.row_factory = sqlite3.Row

# 1. printed aggregate JSON block == saved aggregate
block = re.search(r'^\{\n.*?^\}', log, re.S | re.M).group(0)
printed = json.loads(block)
print("printed aggregate == saved aggregate:", printed == R["aggregate"])
if printed != R["aggregate"]:
    for k in sorted(set(printed) | set(R["aggregate"])):
        if printed.get(k) != R["aggregate"].get(k):
            print("   DIFF", k, printed.get(k), R["aggregate"].get(k))

# 2. nominal windows: recount from DB
print("\nnominal windows (JSON vs DB recount):")
tot = 0.0; fa = 0; an_tot = 0
for w in R["nominal_windows"]:
    a = db.execute("select count(*) from anomalies where timestamp>=? and timestamp<=?", (w["t_start"], w["t_end"])).fetchone()[0]
    d = db.execute("select count(*) from diagnoses where timestamp>=? and timestamp<=?", (w["t_start"], w["t_end"])).fetchone()[0]
    ok = (a == w["anomalies"] and d == w["diagnoses"])
    print(f"  {w['name']:<26} {w['duration_sec']:6.1f}s anomalies json={w['anomalies']} db={a}  diagnoses json={w['diagnoses']} db={d}  {'OK' if ok else 'MISMATCH'}")
    tot += w["duration_sec"]; fa += d; an_tot += a
agg = R["aggregate"]
print(f"  totals: fault-free {tot:.1f}s (agg {agg['nominal_fault_free_sec']}), anomalies {an_tot} (agg {agg['nominal_anomalies']}), "
      f"false alarms {fa} (agg {agg['nominal_false_alarms']}, per min {agg['nominal_false_alarms_per_min']})")

# 3. per-scenario: predicted root / latencies / FDR recomputed from DB
print("\nscenarios (JSON vs DB recount):")
allok = True
for r in R["results"]:
    ti, te, gt = r["t_inject"], r["t_end"], r["ground_truth"]
    an = db.execute("select timestamp from anomalies where timestamp>=? and timestamp<=? order by timestamp,id", (ti, te)).fetchall()
    dg = db.execute("select timestamp, root_cause from diagnoses where timestamp>=? and timestamp<=? order by id", (ti, te)).fetchall()
    det = round(an[0]["timestamp"] - ti, 3); dia = round(dg[0]["timestamp"] - ti, 3)
    fdr = round(sum(1 for d in dg if d["root_cause"] != gt) / len(dg), 4)
    ok = (dg[-1]["root_cause"] == r["predicted_root_cause"] and det == r["detection_latency_sec"]
          and dia == r["diagnosis_latency_sec"] and fdr == r["false_diagnosis_rate"])
    allok &= ok
    phys = f" phys_cov={r['physical_chain_coverage']} unobserved={r['physically_affected_unobserved']}" if r.get("physically_affected_unobserved") else ""
    print(f"  {r['key']:<22} pred={dg[-1]['root_cause']:<18} top1={r['top1_correct']} det={det} diag={dia} fdr={fdr} chain_acc={r['chain_accuracy']}{phys} {'OK' if ok else 'MISMATCH'}")
print("all scenario metrics reproduce from DB:", allok)
print("\naggregate:", json.dumps(agg, indent=1))
