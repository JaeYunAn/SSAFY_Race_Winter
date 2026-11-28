# -*- coding: utf-8 -*-
"""주행 로그(logs/*.jsonl) 구간 분석.

사용:  .venv/Scripts/python.exe tune/analyze_log.py <로그...> [--laps 2] [--no-save] [--top 6]
  - 기준선 곡률(kr0)로 직선/코너를 자동으로 나눈다 (맵마다 처음 나눈 구간을 tune/best_sectors.json 에 저장해 계속 씀)
  - 구간마다: 소요 시간, 진입·최저·탈출 속도, 브레이크·가속 시작 지점, 도로 폭 사용, 조향 흔들림,
    미끄러짐(실제 곡률/목표 곡률), 마찰원 이론 속도 대비 비율, 충돌·벌점(추정)
  - 구간 최고 기록과 그때의 주행 특징을 best_sectors.json 에 모으고 '이론 최고 랩'과의 차이를 보여 준다
  - 시뮬레이터가 멈춘 프레임(stale)은 시간에서 뺀다
"""
import argparse
import json
import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.dirname(HERE)
BEST = os.path.join(HERE, "best_sectors.json")
K_CORNER = 0.004      # 이 곡률(반경 250 m)보다 크면 코너
MIN_CORNER = 15.0     # 이보다 짧은 코너는 무시 (m)
MERGE_GAP = 25.0      # 코너 사이 직선이 이보다 짧으면 한 코너로 (m)


def load(path):
    rows = []
    with open(path, encoding="utf-8") as f:
        for line in f:
            try:
                rows.append(json.loads(line))
            except Exception:
                pass
    return rows


def map_of(path):
    m = re.search(r"drive_map(\w+?)_\d{8}_", os.path.basename(path))
    return m.group(1) if m else "?"


def active_time(rows):
    """멈춘 프레임을 뺀 누적 시간."""
    out, acc = [], 0.0
    for i, r in enumerate(rows):
        if i:
            dt = r["t"] - rows[i - 1]["t"]
            if not r.get("stale") and 0 < dt < 1.0:
                acc += dt
        out.append(acc)
    return out


def lap_length(rows, laps):
    target = 100.0 / laps
    for r in rows:
        if (r.get("prog") or 0) >= target:
            return r["s"]
    fin = next((r["s"] for r in rows if (r.get("prog") or 0) >= 99.9), None)
    return fin / laps if fin else None


def segments_from(rows, L):
    """랩 위치 1 m 칸마다 kr0 평균 → 부드럽게 → 코너/직선 구간."""
    nb = int(L) + 1
    acc = [0.0] * nb
    cnt = [0] * nb
    for r in rows:
        if r.get("kr0") is None or r.get("stale") or r.get("state") != "DRIVE":
            continue
        i = int(r["s"] % L)
        if 0 <= i < nb:
            acc[i] += r["kr0"]
            cnt[i] += 1
    k = [acc[i] / cnt[i] if cnt[i] else None for i in range(nb)]
    last = 0.0                               # 빈 칸은 앞 값으로
    for i in range(nb):
        if k[i] is None:
            k[i] = last
        last = k[i]
    w = 7
    ks = [sum(k[max(0, i - w):i + w + 1]) / len(k[max(0, i - w):i + w + 1]) for i in range(nb)]
    corner = [abs(x) > K_CORNER for x in ks]
    runs = []
    i = 0
    while i < nb:
        j = i
        while j < nb and corner[j] == corner[i]:
            j += 1
        runs.append([i, j, corner[i]])
        i = j
    # 짧은 코너 → 직선, 짧은 직선(코너 사이) → 코너
    for r in runs:
        if r[2] and r[1] - r[0] < MIN_CORNER:
            r[2] = False
    for idx in range(1, len(runs) - 1):
        r = runs[idx]
        if not r[2] and r[1] - r[0] < MERGE_GAP and runs[idx - 1][2] and runs[idx + 1][2]:
            r[2] = True
    merged = []
    for r in runs:
        if merged and merged[-1][2] == r[2]:
            merged[-1][1] = r[1]
        else:
            merged.append(list(r))
    segs = []
    for a, b, c in merged:
        kk = ks[a:b] or [0.0]
        kmax = max(kk, key=abs)
        segs.append({"a": float(a), "b": float(min(b, L)), "type": "C" if c else "S", "kmax": round(kmax, 5)})
    return segs


def first_cross(rows, s_target, start=0):
    for i in range(max(start, 1), len(rows)):
        if rows[i - 1]["s"] < s_target <= rows[i]["s"]:
            return i
    return None


def interp_t(rows, ta, i, s_target):
    r0, r1 = rows[i - 1], rows[i]
    ds = r1["s"] - r0["s"]
    f = (s_target - r0["s"]) / ds if ds > 0 else 0.0
    return ta[i - 1] + f * (ta[i] - ta[i - 1])


def seg_stats(rows, ta, i0, i1, sa, sb, seg, half_est):
    sub = rows[i0:i1 + 1]
    if not sub:
        return None
    vs = [r.get("v") or 0.0 for r in sub]
    vmin = min(vs)
    imin = vs.index(vmin)
    # 브레이크 시작: 구간 시작 100 m 앞부터 최저 속도 지점까지 처음 br>0.05 (구간 시작 기준 m, 음수 = 앞)
    brk = None
    thr = None
    if seg["type"] == "C":
        j = i0
        while j > 0 and rows[j]["s"] > sa - 100.0:
            j -= 1
        for r in rows[j:i0 + imin + 1]:
            if (r.get("br") or 0) > 0.05:
                brk = round(r["s"] - sa, 1)
                break
        for r in sub[imin:]:
            if (r.get("th") or 0) > 0.5:
                thr = round(r["s"] - sa, 1)
                break
    else:
        # 직선: 다음 코너를 위한 브레이크를 직선 끝 몇 m 앞에서 밟기 시작했나 (+ = 끝보다 앞)
        for r in sub:
            if (r.get("br") or 0) > 0.05:
                brk = round(sb - r["s"], 1)
                break
    nmax = max(abs(r.get("n") or 0.0) for r in sub)
    sts = [r.get("st") or 0.0 for r in sub]
    dst = [abs(sts[k] - sts[k - 1]) for k in range(1, len(sts))]
    wob = sum(dst) / len(dst) if dst else 0.0
    sl = []
    for r in sub:
        kd, km = r.get("kdes"), r.get("kmeas")
        if kd is not None and km is not None and abs(kd) > 0.003 and (r.get("v") or 0) > 30:
            sl.append(km / kd)
    slip = sum(sl) / len(sl) if sl else None
    alat = sub[0].get("alat") or 10.0
    kmax = abs(seg["kmax"])
    fric = None
    if seg["type"] == "C" and kmax > 1e-4:
        vth = math.sqrt(alat / kmax) * 3.6
        fric = vmin / vth
    col = sum(1 for k in range(1, len(sub)) if sub[k].get("col") and not sub[k - 1].get("col"))
    pen = sum(1 for r in sub if half_est and abs(r.get("n") or 0) > half_est and (r.get("v") or 0) > 40)
    return {"vin": round(vs[0]), "vmin": round(vmin), "vout": round(vs[-1]), "brk": brk, "thr": thr,
            "nmax": round(nmax, 2), "wob": round(wob, 4), "slip": round(slip, 2) if slip is not None else None,
            "fric": round(fric, 2) if fric is not None else None, "col": col, "pen": pen}


def analyze(path, laps, best, save, top):
    rows = [r for r in load(path) if "s" in r and "t" in r]
    mp = map_of(path)
    if len(rows) < 50:
        print("{}: frames too few".format(path))
        return
    L = lap_length(rows, laps)
    if not L:
        print("{}: no finish / lap length".format(path))
        return
    ta = active_time(rows)
    lohi = [abs(x) for r in rows if r.get("lohi") for pair in r["lohi"] for x in pair]
    half_est = (max(lohi) + 1.75) if lohi else None        # 계획 허용 끝 B = half − 1.75 로 역산 (추정)
    mb = best.setdefault(mp, {})
    if not mb.get("segments") or abs(mb.get("lap_len", 0) - L) > 0.02 * L:
        mb.clear()
        mb["lap_len"] = round(L, 1)
        mb["segments"] = segments_from(rows, L)
        mb["best"] = {}
    segs = mb["segments"]
    Lm = mb["lap_len"]
    t0i = next((i for i, r in enumerate(rows) if r["s"] > 0.3), 0)
    fin = next((i for i, r in enumerate(rows) if (r.get("prog") or 0) >= 99.9), None)
    total = ta[fin] - ta[t0i] if fin else None
    print("=" * 100)
    print("{}  map {}  lap_len {:.0f} m  segments {}  finish {}  stale {:.1f}s".format(
        os.path.basename(path), mp, L, len(segs), "{:.2f}s".format(total) if total else "DNF",
        (rows[fin]["t"] - rows[t0i]["t"] - total) if fin else 0.0))
    table = []
    lap_times = []
    i_prev = t0i
    for lap in range(laps):
        lap_t = 0.0
        ok = True
        for si, seg in enumerate(segs):
            sa = lap * L + seg["a"] * L / Lm
            sb = lap * L + seg["b"] * L / Lm
            ia = first_cross(rows, sa, i_prev) if sa > 0.3 else t0i
            ib = first_cross(rows, sb, ia or i_prev) if ia is not None else None
            if ia is None or ib is None:
                ok = False
                break
            t_a = interp_t(rows, ta, ia, sa) if sa > 0.3 else ta[t0i]
            t_b = interp_t(rows, ta, ib, sb)
            st = seg_stats(rows, ta, ia, ib, sa, sb, seg, half_est)
            st.update({"lap": lap + 1, "seg": si, "type": seg["type"], "len": round(seg["b"] - seg["a"]),
                       "time": round(t_b - t_a, 3), "kmax": seg["kmax"]})
            table.append(st)
            lap_t += t_b - t_a
            i_prev = ib
        lap_times.append(round(lap_t, 2) if ok else None)
    print("laps:", lap_times)
    # 표: 구간마다 랩별 시간 + 마지막 랩 특징
    hdr = "{:>3} {:1} {:>4} {:>7} | {:>14} | {:>4} {:>4} {:>4} {:>6} {:>6} {:>5} {:>6} {:>5} {:>5} {:>3} {:>3} | {:>6}"
    print(hdr.format("seg", "T", "len", "kmax", "time/lap", "vin", "vmin", "vout", "brk@", "thr@", "|n|", "wob",
                     "slip", "fric", "col", "pen", "best"))
    by_seg = {}
    for st in table:
        by_seg.setdefault(st["seg"], []).append(st)
    losses = []
    for si, seg in enumerate(segs):
        lst = by_seg.get(si, [])
        if not lst:
            continue
        cur = min(lst, key=lambda x: x["time"])
        b = mb["best"].get(str(si))
        # 1랩 첫 구간(정지 출발)은 최고 기록 비교에서 뺌
        cand = [x for x in lst if not (x["lap"] == 1 and si == 0)]
        if save and cand:
            c = min(cand, key=lambda x: x["time"])
            if (b is None or c["time"] < b["time"]) and c["col"] == 0:
                c2 = dict(c)
                c2["log"] = os.path.basename(path)
                mb["best"][str(si)] = c2
                b = c2
        bt = b["time"] if b else None
        if bt is not None and cand:
            losses.append((min(x["time"] for x in cand) - bt, si))
        tl = "/".join("{:.2f}".format(x["time"]) for x in lst)
        print(hdr.format(si, seg["type"], cur["len"], "{:.4f}".format(seg["kmax"]), tl, cur["vin"], cur["vmin"],
                         cur["vout"], str(cur["brk"]), str(cur["thr"]), cur["nmax"], cur["wob"],
                         str(cur["slip"]), str(cur["fric"]), sum(x["col"] for x in lst), sum(x["pen"] for x in lst),
                         "{:.2f}".format(bt) if bt is not None else "-"))
    theo = None
    if mb["best"] and len(mb["best"]) == len(segs) - (0 if "0" in mb["best"] else 1):
        theo = sum(v["time"] for v in mb["best"].values())
    fl = [x for x in lap_times[1:] if x] or [x for x in lap_times if x]
    if theo:
        print("theoretical best lap {:.2f}s (from best sectors), this log best flying lap {}".format(
            theo, "{:.2f}s (+{:.2f})".format(min(fl), min(fl) - theo) if fl else "-"))
    losses.sort(reverse=True)
    if losses:
        print("largest losses vs best sector:", ", ".join(
            "seg{}({}) +{:.2f}s".format(si, segs[si]["type"], d) for d, si in losses[:top] if d > 0.005))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("logs", nargs="+")
    ap.add_argument("--laps", type=int, default=2)
    ap.add_argument("--no-save", action="store_true")
    ap.add_argument("--top", type=int, default=6)
    a = ap.parse_args()
    best = {}
    if os.path.exists(BEST):
        try:
            with open(BEST, encoding="utf-8") as f:
                best = json.load(f)
        except Exception:
            best = {}
    for p in a.logs:
        if not os.path.isabs(p) and not os.path.exists(p):
            p = os.path.join(WS, p)
        analyze(p, a.laps, best, not a.no_save, a.top)
    if not a.no_save:
        with open(BEST, "w", encoding="utf-8") as f:
            json.dump(best, f, ensure_ascii=False, indent=1)


if __name__ == "__main__":
    sys.exit(main())
