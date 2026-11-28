# -*- coding: utf-8 -*-
"""설정 후보의 견고성(낯선 맵·다른 차에서 사고가 늘지 않는지)을 가짜 물리에서 여러 번 돌려 비교하는 도구.

사용:  .venv/Scripts/python.exe tune/robust.py '<기준 json>' '<후보 json>' [...]
  - 가짜 물리가 혼돈적이라 한 번 결과는 믿을 수 없음 → 시험 6종 × (그립 ±3%, 출발 시각 2가지) 6번씩
  - 결과: 덮어쓰기마다 미완주·리셋·충돌·벌점 합계와 완주 시간 합
  - json 의 값은 my_car.py 기본값 위에 덮어씀. 지금 기본값과 비교하려면 '{}' 를 넣음
"""
import json
import os
import sys
from multiprocessing import Pool

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "offline_test"))

R1 = dict(pedal_lag=0.25, brake=13.0, drive_gap=1.5, mu_long=13.0)
R2 = dict(R1, steer_lag=0.3, max_steer=0.25)
R3 = dict(R1, steer_delay=0.2, steer_lag=0.1, max_steer=0.25)
CASES = [
    ("ssafy1", dict(track_name="ssafy", laps=1), {}),
    ("speed1", dict(track_name="speed", laps=1), {}),
    ("basic2_real", dict(track_name="basic", laps=2), R1),
    ("ssafy1_real", dict(track_name="ssafy", laps=1), R1),
    ("germany_sdelay", dict(track_name="germany", laps=1), R2),
    ("ssafy_pdelay", dict(track_name="ssafy", laps=1), R3),
]
PERT = [(1.0, 3.0), (0.97, 3.0), (1.03, 3.0), (1.0, 3.05), (0.97, 3.05), (1.03, 3.05)]


def job(args):
    key, name, kw, phys, mu_f, delay = args
    import run_race
    p = dict(phys)
    p["mu"] = p.get("mu", 10.0) * mu_f
    kw = dict(kw)
    kw["phys"] = p
    kw["start_delay"] = delay
    over = json.loads(key)
    kw["cfg_over"] = over or None
    res = run_race.run(**kw)
    c = res["Car1"]
    return key, name, {k: c.get(k) for k in ("finish", "collisions", "penalty_frames", "resets", "crashes", "errors")}


def main():
    overs = [json.dumps(json.loads(a), sort_keys=True) for a in sys.argv[1:]] or ["{}"]
    jobs = [(o, n, kw, ph, mf, d) for o in overs for n, kw, ph in CASES for mf, d in PERT]
    with Pool(processes=max(1, os.cpu_count() or 1), maxtasksperchild=1) as pool:
        rows = pool.map(job, jobs)
    print("{:<60}{:>6}{:>7}{:>7}{:>7}{:>9}{:>6}".format("덮어쓰기", "미완주", "리셋", "충돌", "벌점", "시간합", "예외"))
    for o in overs:
        rs = [r for r in rows if r[0] == o]
        dnf = sum(1 for r in rs if not r[2]["finish"])
        per = {}
        for _, n, c in rs:
            per.setdefault(n, []).append(c)
        tsum = sum(c["finish"] for _, _, c in rs if c["finish"])
        print("{:<60}{:>6}{:>7}{:>7}{:>7}{:>9.1f}{:>6}".format(
            o[:59], dnf, sum(r[2]["resets"] or 0 for r in rs), sum(r[2]["collisions"] or 0 for r in rs),
            sum(r[2]["penalty_frames"] or 0 for r in rs), tsum, sum(r[2]["errors"] or 0 for r in rs)))
        print("    " + "  ".join("{} dnf{} col{} pen{}".format(n, sum(1 for c in cs if not c["finish"]),
                                                              sum(c["collisions"] or 0 for c in cs),
                                                              sum(c["penalty_frames"] or 0 for c in cs)) for n, cs in per.items()))


if __name__ == "__main__":
    main()
