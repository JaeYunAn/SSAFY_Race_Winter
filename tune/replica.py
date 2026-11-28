# -*- coding: utf-8 -*-
"""실제 주행 로그의 도로 모양·장애물로 가짜 시뮬레이터 코스를 만들어 my_car.py 를 빠르게 여러 번 시험하는 도구.

사용:  .venv/Scripts/python.exe tune/replica.py <로그.jsonl> [--runs 4] [--laps 1] [--half 8.0] ['<Config json>' ...]
  - 로그의 2랩째 s·kr0 로 한 랩 중심선을 만들고(곡률 적분), 로그에 나온 장애물(거리·옆 위치)을 같은 자리에 둠
  - 차 물리는 실제 시뮬레이터 실측에 가깝게 (그립 20, 앞뒤 14, 페달 지연, 구동 공백, 조향 지연). 출발 시각을 조금씩 바꿔 여러 번
  - 결과: 덮어쓰기마다 완주 시간·충돌·벌점의 평균과 최악. 시간 숫자는 참고, '사고가 나는가'를 빨리 보는 용도
"""
import json
import math
import os
import sys
from multiprocessing import Pool

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.dirname(HERE)
OFF = os.path.join(WS, "offline_test")
sys.path.insert(0, OFF)

PHYS = dict(pedal_lag=0.25, brake=13.0, drive_gap=1.5, mu=20.0, mu_long=14.0,
            steer_delay=0.15, steer_lag=0.1, max_steer=0.25)


def extract(log):
    rows = [json.loads(l) for l in open(log, encoding="utf-8")]
    rows = [x for x in rows if x.get("state") == "DRIVE" and not x.get("stale") and not x.get("tp")]
    s_end = rows[-1]["s"]
    # lap length: from trials/laps unknown -> use api progress: find s where prog crosses 50% (2 laps)
    L = None
    for a, b in zip(rows, rows[1:]):
        if (a.get("prog") or 0) < 50.0 <= (b.get("prog") or 0):
            L = b["s"]
            break
    if L is None:
        L = s_end / 2.0
    prof = []
    last = -1.0
    for x in rows:
        s = x["s"] - L
        if 0.0 <= s <= L and s > last + 0.5:
            prof.append((s, x["kr0"]))
            last = s
    obs = []
    for x in rows:
        if x["s"] > L:
            break
        for d, m in (x.get("obs") or []):
            if 0 < d < 60:
                so = (x["s"] + d) % L
                if not any(abs(so - a) < 4.0 and abs(m - b) < 0.8 for a, b in obs):
                    obs.append((so, m))
    return L, prof, sorted(obs)


def build_track(L, prof, obs, half):
    import fakesim
    tr = object.__new__(fakesim.Track)
    tr.name = "replica"
    tr.width = 2 * half
    tr.half = half
    tr.res = 0.5
    S = [p[0] for p in prof]
    K = [p[1] for p in prof]

    def k_at(s):
        if s <= S[0]:
            return K[0]
        if s >= S[-1]:
            return K[-1]
        lo, hi = 0, len(S) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if S[mid] <= s:
                lo = mid
            else:
                hi = mid
        f = (s - S[lo]) / max(S[hi] - S[lo], 1e-9)
        return K[lo] + (K[hi] - K[lo]) * f
    x = y = h = 0.0
    tr.x, tr.y, tr.h = [], [], []
    n = int(L / tr.res)
    for i in range(n):
        tr.x.append(x)
        tr.y.append(y)
        tr.h.append(h)
        s = i * tr.res
        # 기준선 곡률 +오른쪽 = fakesim(NED: x 앞, y 오른쪽) 방향각 증가와 같은 부호
        k = k_at(s)
        hm = h + 0.5 * k * tr.res
        x += tr.res * math.cos(hm)
        y += tr.res * math.sin(hm)
        h += k * tr.res
    # 고리 닫기: 끝 방향이 2π 의 배수, 끝 위치가 출발점이 되도록 오차를 거리에 비례해 나눠 고침
    n = len(tr.x)
    tgt = 2 * math.pi * round(h / (2 * math.pi))
    eh = h - tgt
    hs = [tr.h[i] - eh * i / n for i in range(n)]
    xs, ys = [0.0], [0.0]
    for i in range(1, n):
        hm = 0.5 * (hs[i - 1] + hs[i])
        xs.append(xs[-1] + tr.res * math.cos(hm))
        ys.append(ys[-1] + tr.res * math.sin(hm))
    hm = 0.5 * (hs[-1] + tgt)
    ex = xs[-1] + tr.res * math.cos(hm)
    ey = ys[-1] + tr.res * math.sin(hm)
    tr.x = [xs[i] - ex * i / n for i in range(n)]
    tr.y = [ys[i] - ey * i / n for i in range(n)]
    tr.h = hs
    tr.n = n
    tr.length = tr.n * tr.res
    tr.close_err = (round(eh, 3), round(ex, 1), round(ey, 1))
    tr.obstacles = []
    tr.hidden = set()
    for s, m in obs:
        px, py, hh = tr.at(s)
        # fakesim 옆 위치: lat = -dx*sin(h) + dy*cos(h) (+오른쪽이 되도록 at() 의 좌표계 사용)
        tr.obstacles.append((px - m * math.sin(hh), py + m * math.cos(hh), hh, s, m))
    tr.waypoints = []
    s = 0.0
    while s < tr.length - 1.0:
        px, py, _ = tr.at(s)
        tr.waypoints.append([px, py, 0.0])
        s += 10.0
    return tr


def job(args):
    log, half, laps, over, delay = args
    import fakesim
    import run_race
    L, prof, obs = extract(log)
    tr = build_track(L, prof, obs, half)
    orig = fakesim.make_tracks

    def mk():
        t = orig()
        t["replica"] = tr
        return t
    fakesim.make_tracks = mk
    over2 = {"memory_shared_path": os.path.join(HERE, "baseline_car.json")}   # 실제 시험 주행과 같은 차 특성으로 출발
    over2.update(over or {})
    res = run_race.run(track_name="replica", laps=laps, phys=PHYS, cfg_over=over2, start_delay=delay)
    c = res["Car1"]
    return json.dumps(over, sort_keys=True), delay, {k: c.get(k) for k in ("finish", "collisions", "penalty_frames", "crashes", "errors")}


def main():
    args = sys.argv[1:]
    opts = {"--runs": 4, "--laps": 1, "--half": 8.0}
    for k in list(opts):
        if k in args:
            i = args.index(k)
            opts[k] = type(opts[k])(args[i + 1])
            del args[i:i + 2]
    log = args[0]
    overs = [{}] + [json.loads(a) for a in args[1:]]
    L, prof, obs = extract(log)
    print("lap %.0f m, profile %d pts, obstacles %d" % (L, len(prof), len(obs)))
    jobs = [(log, opts["--half"], opts["--laps"], o, 3.0 + 0.037 * r) for o in overs for r in range(opts["--runs"])]
    with Pool(processes=max(1, os.cpu_count() or 1), maxtasksperchild=1) as pool:
        rows = pool.map(job, jobs)
    by = {}
    for key, d, c in rows:
        by.setdefault(key, []).append(c)
    for o in overs:
        key = json.dumps(o, sort_keys=True)
        cs = by[key]
        fins = [c["finish"] for c in cs if c["finish"]]
        print("%-55s fin %s  mean %s  col %s  pen %s  crash %s  err %s" % (
            key[:54], len(fins), round(sum(fins) / len(fins), 2) if fins else None,
            [c["collisions"] for c in cs], [c["penalty_frames"] for c in cs], [c["crashes"] for c in cs],
            sum(c["errors"] or 0 for c in cs)))


if __name__ == "__main__":
    main()
