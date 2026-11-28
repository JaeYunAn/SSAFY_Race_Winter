# -*- coding: utf-8 -*-
"""설정값 후보를 실제 주행 전에 오프라인(가짜 물리)으로 빠르게 거르는 도구.

사용:  .venv/Scripts/python.exe tune/screen.py '<json 덮어쓰기>' ['<json>' ...]
  - 각 json 마다 아래 5개 시험을 병렬로 돌려 완주 시간·충돌·벌점·복구를 보여 준다 (첫 줄은 늘 지금 기본값 {})
  - 가짜 물리는 실제 시뮬레이터 실측(2026-11-28)에 가깝게: 옆 그립 22, 앞뒤 14, 페달 지연 0.25초,
    브레이크 뒤 구동 공백, 조향 순수 지연 0.2초 + 1차 지연 0.1초
  - 시간 숫자는 참고만. 실제 판단은 run_trial.py 로 한다
"""
import json
import os
import sys
from multiprocessing import Pool

HERE = os.path.dirname(os.path.abspath(__file__))
OFF = os.path.join(os.path.dirname(HERE), "offline_test")
sys.path.insert(0, OFF)

REAL = dict(pedal_lag=0.25, brake=13.0, drive_gap=1.5, mu=22.0, mu_long=14.0)
CASES = [
    ("basic2", dict(track_name="basic", laps=2, phys=REAL)),
    ("speed1", dict(track_name="speed", laps=1, phys=REAL)),
    ("ssafy1", dict(track_name="ssafy", laps=1, phys=REAL)),
    ("germany1", dict(track_name="germany", laps=1, phys=REAL)),
    ("hidden2", dict(track_name="hidden", laps=2, phys=REAL)),
]


def job(args):
    name, kw, over = args
    import run_race
    kw = dict(kw)
    kw["cfg_over"] = over or None
    res = run_race.run(**kw)
    c = res["Car1"]
    return name, json.dumps(over, sort_keys=True), {k: c.get(k) for k in
                                                     ("finish", "collisions", "penalty_frames", "crashes", "errors")}


def main():
    overs = [{}] + [json.loads(a) for a in sys.argv[1:]]
    jobs = [(n, kw, o) for o in overs for n, kw in CASES]
    with Pool(processes=max(1, os.cpu_count() or 1), maxtasksperchild=1) as pool:
        rows = pool.map(job, jobs)
    by = {}
    for name, key, c in rows:
        by.setdefault(key, {})[name] = c
    print("{:<48}".format("덮어쓰기") + "".join("{:>18}".format(n) for n, _ in CASES) + "{:>9}{:>5}{:>5}".format("합계", "충돌", "벌점"))
    for o in overs:
        key = json.dumps(o, sort_keys=True)
        cells, tot, col, pen, bad = "", 0.0, 0, 0, 0
        for n, _ in CASES:
            c = by[key][n]
            f = c["finish"]
            if f:
                tot += f
            else:
                bad += 1
            col += c["collisions"] or 0
            pen += c["penalty_frames"] or 0
            cells += "{:>18}".format("{} c{} p{}".format("{:.1f}".format(f) if f else "DNF", c["collisions"], c["penalty_frames"]))
        print("{:<48}".format(key[:47]) + cells + "{:>9}{:>5}{:>5}".format("{:.1f}{}".format(tot, "*" if bad else ""), col, pen))


if __name__ == "__main__":
    main()
