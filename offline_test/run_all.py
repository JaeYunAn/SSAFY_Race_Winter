"""
run_all.py — my_car.py 전체 점검 (진짜 drive_controller.py + 가짜 시뮬레이터)
  python run_all.py
결과 표: 완주 시간, 충돌, 벌점 프레임, 리셋, 복구 횟수, 프레임당 계산 시간
※ 가짜 물리라 '시간 숫자'보다 '충돌 0·벌점 0·리셋 0·오류 0'이 핵심입니다.
"""
import json
import os
import sys
import time
from multiprocessing import Pool

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)


def job(args):
    name, kw = args
    import run_race
    import fakesim
    orig_load = run_race.load_module
    times = []

    def load(nm, path):
        m = orig_load(nm, path)
        if nm == "my_car":
            st = m.RaceBrain.step

            def step(self, raw):
                t0 = time.perf_counter()
                out = st(self, raw)
                times.append(time.perf_counter() - t0)
                return out
            m.RaceBrain.step = step
        return m
    run_race.load_module = load
    res = run_race.run(**kw)
    c = res["Car1"]
    c["ms_avg"] = round(1000 * sum(times) / max(len(times), 1), 2)
    c["ms_max"] = round(1000 * max(times or [0]), 1)
    if "Car2" in res:
        c["opp"] = res["Car2"]
    return name, res


CASES = [
    ("basic 1랩", dict(track_name="basic", laps=1)),
    ("speed 1랩 (경유점 위 장애물 1개 포함)", dict(track_name="speed", laps=1)),
    ("ssafy 1랩", dict(track_name="ssafy", laps=1)),
    ("germany 1랩", dict(track_name="germany", laps=1)),
    ("basic 2랩 (랩 길이 자동 감지)", dict(track_name="basic", laps=2)),
    ("ssafy 2랩", dict(track_name="ssafy", laps=2)),
    ("배틀: ssafy vs 기본 템플릿 봇", dict(track_name="ssafy", laps=1, battle="basic")),
    ("배틀: ssafy 같은 AI끼리", dict(track_name="ssafy", laps=1, battle="selfplay")),
    ("거꾸로 출발 (역주행 U턴)", dict(track_name="basic", laps=1, start_delay=0.1, spawn_yaw=3.14159)),
    ("조향 둔하고 미끄러운 차 (germany)", dict(track_name="germany", laps=1,
                                      phys=dict(max_steer=0.6, mu=9.0, steer_red=0.5, power=200.0))),
    ("그립 좋은 차 (speed)", dict(track_name="speed", laps=1, phys=dict(mu=12.5, max_steer=0.8, steer_red=0.2,
                                                                       power=300.0))),
    ("조향 부호 반대인 차 (basic)", dict(track_name="basic", laps=1, phys=dict(steer_dir=-1.0))),
    ("보이지 않는 장애물 (2랩)", dict(track_name="hidden", laps=2)),
    # 실제 시뮬레이터처럼 페달이 늦게 듣는 차 (2026-11-25 Map 10 실측: 지연 0.22~0.33초, 브레이크 1당 약 13 m/s²)
    ("페달 지연 0.25초 차 (basic 2랩)", dict(track_name="basic", laps=2, phys=dict(pedal_lag=0.25, brake=13.0))),
    ("페달 지연 0.25초 차 (ssafy 1랩)", dict(track_name="ssafy", laps=1, phys=dict(pedal_lag=0.25, brake=13.0))),
    # 실제 차에 가장 가까운 흉내 (2026-11-25 Map 10 실측): 페달 지연 + 브레이크 뒤 구동 공백 1.5초 + 타원 그립(앞뒤 13)
    ("실제 차 흉내 (basic 2랩)", dict(track_name="basic", laps=2,
                                  phys=dict(pedal_lag=0.25, brake=13.0, drive_gap=1.5, mu_long=13.0))),
    ("실제 차 흉내 (ssafy 1랩)", dict(track_name="ssafy", laps=1,
                                  phys=dict(pedal_lag=0.25, brake=13.0, drive_gap=1.5, mu_long=13.0))),
    # 조향까지 실제처럼 (조향 반응 지연 0.3초, 최대 조향각 작음 → 조향 1당 곡률 약 0.056): 직선 흔들림 점검용
    ("실제 차 흉내 + 조향 지연 (germany 1랩)", dict(track_name="germany", laps=1,
                                         phys=dict(pedal_lag=0.25, brake=13.0, drive_gap=1.5, mu_long=13.0,
                                                   steer_lag=0.3, max_steer=0.25))),
    # 시뮬레이터가 잠깐 멈추는 경우 (실측 1.1초×2). 0.8초로 해서 drive_controller 의 '속도 10번 같으면 리셋'은 피함
    ("시뮬레이터 멈춤 0.8초×3 (basic 2랩)", dict(track_name="basic", laps=2,
                                          phys=dict(freeze=[(12.0, 0.8), (25.0, 0.8), (40.0, 0.8)]))),
]
if not os.path.exists(os.path.join(os.path.dirname(HERE), "my_car_basic.py")):
    CASES = [c for c in CASES if c[1].get("battle") != "basic"]


def main():
    t0 = time.time()
    with Pool(processes=max(1, os.cpu_count() or 1), maxtasksperchild=1) as pool:
        rows = pool.map(job, CASES)
    # 연습 → 본선 메모리 (순서대로)
    mem = os.path.join(HERE, "_mem_test.json")
    if os.path.exists(mem):
        os.remove(mem)
    mem_rows = []
    for i in range(3):
        with Pool(processes=1, maxtasksperchild=1) as pool:
            mem_rows += pool.map(job, [("메모리 이어 달리기 {}회차 (germany)".format(i + 1),
                                        dict(track_name="germany", laps=1, mem=mem))])
    if os.path.exists(mem):
        os.remove(mem)
    print("{:<36}{:>8}{:>6}{:>6}{:>6}{:>6}{:>6}{:>8}{:>8}".format("시험", "완주(s)", "충돌", "벌점", "리셋", "복구", "U턴",
                                                             "ms평균", "ms최대"))
    for name, res in rows + mem_rows:
        c = res["Car1"]
        fin = "{:.2f}".format(c["finish"]) if c.get("finish") else "미완주"
        print("{:<36}{:>8}{:>6}{:>6}{:>6}{:>6}{:>6}{:>8}{:>8}".format(
            name, fin, c["collisions"], c["penalty_frames"], c["resets"], c.get("crashes", "-"), c.get("uturns", "-"),
            c["ms_avg"], c["ms_max"]))
        if "opp" in c:
            o = c["opp"]
            print("   └ 상대 차: 완주 {} 충돌 {}".format(o.get("finish") or "미완주", o["collisions"]))
    print("학습값(ssafy 1랩):", {k: rows[2][1]["Car1"][k] for k in ("a_lat", "a_brake", "a_drive", "gain60", "gain150")})
    print("오류(예외) 합계:", sum(r[1]["Car1"].get("errors", 0) for r in rows + mem_rows))
    print("총 소요 {:.0f}초".format(time.time() - t0))
    with open(os.path.join(HERE, "run_all_result.json"), "w", encoding="utf-8") as f:
        json.dump([r for r in rows + mem_rows], f, ensure_ascii=False, indent=1)


if __name__ == "__main__":
    main()
