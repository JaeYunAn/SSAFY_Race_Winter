# -*- coding: utf-8 -*-
"""경로 계획(레이싱 라인) 품질을 실제 로그의 도로 모양으로 오프라인 재현하는 도구.

사용:  .venv/Scripts/python.exe tune/line_sim.py <로그.jsonl> ['<Config 덮어쓰기 json>' ...] [--mu 18]
  - 로그의 s·kr0(기준선 곡률)로 도로 중심선을 다시 만들고, 차가 계획 선을 그대로 따라간다고 보고 PathPlanner 를 0.1초마다 돌림
  - 결과: 도로 폭 사용 범위, 경로 곡률/도로 곡률 비, 코너별 최저 한계 속도, 간단한 속도 계획으로 낸 구간 시간
  - 장애물·조향 지연·미끄러짐은 없음 → '선 자체가 얼마나 빠른가'만 봄. 실제 판단은 run_trial.py
"""
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.dirname(HERE)
sys.path.insert(0, WS)
sys.path.insert(0, os.path.join(WS, "DrivingInterface"))


def load_road(path):
    rows = [json.loads(l) for l in open(path, encoding="utf-8")]
    pts = []
    last = -1.0
    for x in rows:
        if x.get("stale") or x.get("state") != "DRIVE" or x.get("tp"):
            continue
        s = x["s"]
        if s > last + 0.5:
            pts.append((s, x["kr0"]))
            last = s
    return pts


def main():
    args = [a for a in sys.argv[1:]]
    mu = 18.0
    if "--mu" in args:
        i = args.index("--mu")
        mu = float(args[i + 1])
        del args[i:i + 2]
    log = args[0]
    overs = [{}] + [json.loads(a) for a in args[1:]]
    import my_car as mc
    pts = load_road(log)
    S = [p[0] for p in pts]
    K = [p[1] for p in pts]
    # heading by integration
    TH = [0.0]
    for i in range(1, len(S)):
        TH.append(TH[-1] + 0.5 * (K[i] + K[i - 1]) * (S[i] - S[i - 1]))

    def th_at(s):
        return mc.lerp_list(S, TH, s)

    def k_at(s):
        return mc.lerp_list(S, K, s)

    s_start, s_end = S[0] + 50.0, S[-1] - 260.0
    print("road %.0f..%.0f m (sim %.0f..%.0f), mu %.1f" % (S[0], S[-1], s_start, s_end, mu))
    print("{:<52}{:>8}{:>8}{:>8}{:>8}{:>9}{:>9}".format("덮어쓰기", "n최소", "n최대", "k비", "코너v", "시간(s)", "ms"))
    import time as _t
    for over in overs:
        c = mc.Config()
        for k, v in over.items():
            setattr(c, k, v)
        half = 9.25
        planner = mc.PathPlanner(c)
        s0 = s_start
        n, psi = 0.0, 0.0
        v = 40.0
        log_k, log_n, log_s = [], [], []
        t_list = [k * c.plan_ds / v for k in range(c.plan_stations + 1)]
        tc = 0.0
        steps = 0
        while s0 < s_end:
            kd = [0.0]
            kt = [0.0]
            d = 10.0
            while d <= 260.0:
                kd.append(d)
                kt.append(th_at(s0 + d) - th_at(s0))
                d += 10.0
            fr = mc.Frame(v=v, n=n, psi=psi, obstacles=[], opponents=[], collided=False, half=half,
                          knot_d=kd, knot_th=kt, kr0=k_at(s0), s=s0)
            geo = mc.RoadGeometry(kd, kt)
            t0 = _t.perf_counter()
            plan = planner.plan(fr, geo, s0, t_list, True)
            tc += _t.perf_counter() - t0
            steps += 1
            ds = v * 0.1
            n_new = mc.lerp_list(plan.d, plan.n, ds)
            n2 = mc.lerp_list(plan.d, plan.n, ds + 1.0)
            psi = math.atan(n2 - n_new)
            kp = mc.lerp_list(plan.d, plan.kappa, ds)
            n = n_new
            s0 += ds
            log_s.append(s0)
            log_k.append(kp)
            log_n.append(n)
        # speed profile on the driven line (forward/backward pass): drive 4 m/s², brake 13 m/s², limit sqrt(mu/|k|), cap 70 m/s
        lim = [min(70.0, math.sqrt(mu / max(abs(kk), c.kappa_floor * abs(k_at(s)), 1e-6))) for s, kk in zip(log_s, log_k)]
        vv = list(lim)
        for i in range(1, len(vv)):
            dsi = log_s[i] - log_s[i - 1]
            vv[i] = min(vv[i], math.sqrt(vv[i - 1] ** 2 + 2 * 4.0 * dsi))
        for i in range(len(vv) - 2, -1, -1):
            dsi = log_s[i + 1] - log_s[i]
            vv[i] = min(vv[i], math.sqrt(vv[i + 1] ** 2 + 2 * 13.0 * dsi))
        T = sum((log_s[i] - log_s[i - 1]) / max(0.5 * (vv[i] + vv[i - 1]), 1.0) for i in range(1, len(vv)))
        # path/road curvature ratio in corners (|kr| > 0.008)
        num = den = 0.0
        for s, kp in zip(log_s, log_k):
            kr = abs(k_at(s))
            if kr > 0.008:
                num += abs(kp)
                den += kr
        # corner min speeds (local minima of vv below 55 m/s)
        mins = [vv[i] for i in range(1, len(vv) - 1) if vv[i] <= vv[i - 1] and vv[i] <= vv[i + 1] and vv[i] < 55]
        print("{:<52}{:>8.1f}{:>8.1f}{:>8.2f}{:>8.0f}{:>9.2f}{:>9.1f}".format(
            json.dumps(over)[:51], min(log_n), max(log_n), num / max(den, 1e-9),
            3.6 * sum(mins) / max(len(mins), 1), T, 1000 * tc / max(steps, 1)))


if __name__ == "__main__":
    main()
