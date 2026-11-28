# -*- coding: utf-8 -*-
"""자동 개선 반복 보조: trials.jsonl 집계와 채택 판정.

  loop_util.py summary <label>            : 그 label 의 맵별 기록·충돌·벌점·리셋
  loop_util.py judge <label> [dev|val]    : state.json 기준과 비교해 채택 기준 판정 (dev = 10·31, val = 61)
  loop_util.py baseline <label> [maps..]  : 그 label 의 기록을 state.json 기준으로 (흔들림 규칙 적용)
  loop_util.py stop                       : STOP 파일 있으면 STOP 출력
"""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
TRIALS = os.path.join(HERE, "results", "trials.jsonl")
STATE = os.path.join(HERE, "state.json")


def trials(label):
    out = []
    if os.path.exists(TRIALS):
        with open(TRIALS, encoding="utf-8") as f:
            for line in f:
                try:
                    r = json.loads(line)
                except Exception:
                    continue
                if r.get("label") == label:
                    out.append(r)
    return out


def agg(rs):
    by = {}
    for r in rs:
        key = r["map"] + ("b" if r.get("battle") else "")
        by.setdefault(key, []).append(r)
    res = {}
    for m, lst in by.items():
        times = [r.get("time") for r in lst]
        ok = all(r.get("finished") and r.get("time") for r in lst)
        n = len(lst)
        res[m] = {
            "n": n, "times": times, "all_finished": ok,
            "mean": round(sum(t for t in times if t) / max(1, len([t for t in times if t])), 3) if any(times) else None,
            "col": round(sum((r.get("col_events") or 0) for r in lst) / n, 2),
            "crash": round(sum((r.get("reverse_events") or 0) for r in lst) / n, 2),
            "pen": round(sum((r.get("penalty_frames") or 0) for r in lst) / n, 2),
            "reset": round(sum((r.get("reset_events") or 0) for r in lst) / n, 2),
            "errors": sum((r.get("brain_errors") or 0) for r in lst),
            "fails": [r.get("error") for r in lst if r.get("error")],
        }
        if m.endswith("b"):
            res[m]["won"] = [bool(r.get("time") and (not r.get("opp_time_official") or r["time"] < r["opp_time_official"]))
                             for r in lst]
            res[m]["opp_times"] = [r.get("opp_time_official") for r in lst]
    return res


def load_state():
    with open(STATE, encoding="utf-8") as f:
        return json.load(f)


def save_state(s):
    with open(STATE, "w", encoding="utf-8") as f:
        json.dump(s, f, ensure_ascii=False, indent=1)


def noise_rules(base):
    """흔들림: 10·31 = 최대−최소 (최소 평균의 0.5%), 61·161 = 10·31 상대 흔들림 중 큰 값 × 기록 (최소 1%)."""
    rel = []
    for m in ("10", "31"):
        b = base.get(m)
        if b and b.get("mean"):
            ts = [t for t in b["times"] if t]
            rng = max(ts) - min(ts) if len(ts) > 1 else 0.0
            b["noise"] = round(max(rng, 0.005 * b["mean"]), 3)
            rel.append(b["noise"] / b["mean"])
    for m in ("61", "161", "61b"):
        b = base.get(m)
        if b and b.get("mean"):
            b["noise"] = round(b["mean"] * max(max(rel) if rel else 0.0, 0.01), 3)
    return base


def judge(label, kind):
    s = load_state()
    base = s["baseline"]
    new = agg(trials(label))
    maps = ["10", "31"] if kind == "dev" else (["61"] if kind == "val" else kind.split(","))
    ok = True
    reasons = []
    deltas, noises = [], []
    for m in maps:
        b, n = base.get(m), new.get(m)
        if not n:
            ok = False
            reasons.append("{}: no trials".format(m))
            continue
        if not n["all_finished"]:
            ok = False
            reasons.append("{}: not finished {}".format(m, n["fails"]))
            continue
        d = n["mean"] - b["mean"]
        deltas.append(d)
        noises.append(b["noise"])
        if d > b["noise"]:
            ok = False
            reasons.append("{}: slower {:+.2f} > noise {:.2f}".format(m, d, b["noise"]))
        for k in ("col", "crash", "pen", "reset"):
            if n[k] > b.get(k, 0) + 1e-9:
                ok = False
                reasons.append("{}: {} {} > base {}".format(m, k, n[k], b.get(k, 0)))
        if n["errors"]:
            ok = False
            reasons.append("{}: brain errors {}".format(m, n["errors"]))
    if kind == "dev" and deltas and len(deltas) == len(maps):
        avg_d = sum(deltas) / len(deltas)
        avg_n = sum(noises) / len(noises)
        if avg_d > -avg_n:
            ok = False
            reasons.append("avg delta {:+.3f} not <= -{:.3f}".format(avg_d, avg_n))
        else:
            reasons.append("avg delta {:+.3f} <= -{:.3f}".format(avg_d, avg_n))
    print(json.dumps({"label": label, "kind": kind, "pass": ok, "reasons": reasons,
                      "new": {m: {k: new[m][k] for k in ("times", "mean", "col", "crash", "pen", "reset")} for m in new},
                      "base": {m: {k: base[m].get(k) for k in ("mean", "noise", "col", "crash", "pen", "reset")}
                               for m in maps if m in base}}, ensure_ascii=False))


def set_baseline(label, maps):
    s = load_state()
    new = agg(trials(label))
    for m in (maps or list(new)):
        if m in new:
            s["baseline"][m] = new[m]
    noise_rules(s["baseline"])
    save_state(s)
    print(json.dumps(s["baseline"], ensure_ascii=False))


def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else "summary"
    if cmd == "stop":
        if os.path.exists(os.path.join(HERE, "STOP")) or os.path.exists(os.path.join(HERE, "STOP.txt")):
            print("STOP")
        else:
            print("go")
    elif cmd == "summary":
        print(json.dumps(agg(trials(sys.argv[2])), ensure_ascii=False))
    elif cmd == "judge":
        judge(sys.argv[2], sys.argv[3] if len(sys.argv) > 3 else "dev")
    elif cmd == "baseline":
        set_baseline(sys.argv[2], sys.argv[3:])


if __name__ == "__main__":
    main()
