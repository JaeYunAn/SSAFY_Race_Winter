"""
run_race.py — 진짜 drive_controller.py + my_car.py 를 가짜 시뮬레이터에서 실행 (시뮬레이터 없이 점검)
  ..\.venv\Scripts\python.exe run_race.py <트랙> [랩수] [옵션...]
    트랙: basic | speed | ssafy | germany
    옵션: battle(상대=기본 템플릿 봇) / selfplay(상대=같은 AI) / mem=파일 / log=파일 / slow(조향·그립 다른 차)
"""
import importlib.util
import json
import os
import shutil
import sys
import tempfile
import threading
import time as _real_time

HERE = os.path.dirname(os.path.abspath(__file__))
BOT = os.path.dirname(HERE)          # workspace 폴더 (my_car.py, DrivingInterface 가 있는 곳)
sys.path.insert(0, HERE)
sys.path.insert(0, BOT)
import fakesim  # noqa: E402


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


if not os.path.exists(os.path.join(BOT, "DrivingInterface", "drive_controller.py")):
    print("[준비] workspace/DrivingInterface 폴더가 없습니다.")


def run(track_name, laps=1, battle=None, mem=None, log=None, phys=None, limit=600.0, quiet=True, cfg_over=None,
        start_delay=3.0, spawn_yaw=0.0, start_glitch=False):
    docs = tempfile.mkdtemp()
    fakesim.install_fakes(docs)
    cars = {"Car1": (0.0, 0.0)} if not battle else {"Car1": (0.0, -4.0), "Car2": (0.0, 4.0)}
    settings = {"SettingsVersion": 1.2, "SimMode": "Car", "Algo": {"Map": "10"},
                "Vehicles": {k: {"VehicleType": "PhysXCar", "X": v[0], "Y": v[1], "Z": 0} for k, v in cars.items()}}
    os.makedirs(os.path.join(docs, "Airsim"), exist_ok=True)
    with open(docs + "\\Airsim\\settings.json", "w", encoding="utf-8") as f:
        json.dump(settings, f)
    track = fakesim.make_tracks()[track_name]
    fakesim.WORLD = fakesim.World(track, cars, laps=laps, phys=phys, start_delay=start_delay)
    W = fakesim.WORLD
    if spawn_yaw:
        for c in W.cars.values():
            c.yaw = spawn_yaw
    import DrivingInterface.drive_controller as dc
    dc.time = fakesim.FakeTime()
    counts = {"penalty_print": 0}

    def fake_print(*a, **k):
        if a and isinstance(a[0], str) and "Penalty" in a[0]:
            counts["penalty_print"] += 1
        elif not quiet:
            print(*a, **k)
    dc.print = fake_print
    os.environ.pop("SSAFY_LOCAL", None)   # 점검 중에는 memory.json / logs 를 만들지 않음
    my_car = load_module("my_car", os.path.join(BOT, "my_car.py"))
    my_car.time = fakesim.FakeTime()
    if start_glitch:
        # 출발 지점 도로 정보 엉터리 흉내 (2026-11-26 Map 61·70·71 실측): 첫 경유점을 지나 API 진행률이 바뀔 때까지
        #  track_forward_angles 가 엉터리 구간 기준(전부 −150° 어긋남), moving_angle 도 엉터리
        _st = my_car.RaceBrain.step

        def _glitch_step(self, raw):
            g = self.__dict__.setdefault("_glitch", {"p0": None, "d": 0.0})
            g["d"] += float(getattr(raw, "speed", 0.0) or 0.0) / 3.6 * 0.1
            prog = getattr(raw, "lap_progress", None)
            if g["p0"] is None:
                g["p0"] = prog
            if prog == g["p0"] and g["d"] < 25.0:
                raw.track_forward_angles = [a - 150 for a in (getattr(raw, "track_forward_angles", None) or [])]
                raw.moving_angle = -120.0 + 3.0 * g["d"]
            return _st(self, raw)
        my_car.RaceBrain.step = _glitch_step
    if quiet:
        my_car.print = lambda *a, **k: None
    clients = []
    c1 = my_car.DrivingClient()
    c1.brain_cfg.memory_path = mem
    c1.brain_cfg.log_path = log
    if cfg_over:
        for k, v in cfg_over.items():
            setattr(c1.brain_cfg, k, v)
    clients.append(c1)
    if battle == "basic":
        basic = load_module("basic_car", os.path.join(BOT, "my_car_basic.py"))
        basic.print = lambda *a, **k: None
        clients.append(basic.DrivingClient())
    elif battle == "selfplay":
        clients.append(my_car.DrivingClient())
    W.participants = len(clients)
    results = {}

    def runner(cl):
        try:
            results[cl.player_name] = cl.run()
        except Exception as e:
            import traceback
            traceback.print_exc()
            results[cl.player_name] = repr(e)
        finally:
            W.leave()

    def watchdog():
        while any(th.is_alive() for th in ths):
            if W.t > limit:
                for c in W.cars.values():
                    c.finished = True
            _real_time.sleep(0.2)

    ths = [threading.Thread(target=runner, args=(cl,), daemon=True) for cl in clients]
    t0 = _real_time.time()
    for th in ths:
        th.start()
    wd = threading.Thread(target=watchdog, daemon=True)
    wd.start()
    for th in ths:
        th.join()
    wall = _real_time.time() - t0
    out = {"track": track_name, "laps": laps, "track_len": round(track.length), "wall_s": round(wall, 1),
           "penalty_prints": counts["penalty_print"]}
    for cl in clients:
        car = W.cars[cl.player_name]
        r = {"finish": round(getattr(car, "finish_time", float("nan")), 2) if car.finished and hasattr(car, "finish_time") else None,
             "collisions": car.collisions, "resets": car.resets, "penalty_frames": car.penalty_frames}
        b = getattr(cl, "brain", None)
        if b is not None:
            r.update({"brain_laps": [x["time"] for x in b.laps], "crashes": b.crashes, "uturns": b.uturns,
                      "slips": b.slips, "a_lat": round(b.a_lat, 2), "a_brake": round(b.a_brake, 2),
                      "a_drive": round(b.a_drive, 2), "gain60": round(b.steer_gain.get(60 / 3.6), 4),
                      "gain150": round(b.steer_gain.get(150 / 3.6), 4), "lap_len": b.lap_len,
                      "resyncs": b.adapter.resyncs, "passes": b.adapter.passes, "errors": cl.brain_errors,
                      "sign_flips": b.sign_flips})
        results_name = cl.player_name
        out[results_name] = r
    return out


if __name__ == "__main__":
    tr = sys.argv[1] if len(sys.argv) > 1 else "basic"
    laps = int(sys.argv[2]) if len(sys.argv) > 2 and sys.argv[2].isdigit() else 1
    opts = sys.argv[3:] if len(sys.argv) > 3 else []
    battle = "basic" if "battle" in opts else ("selfplay" if "selfplay" in opts else None)
    mem = next((o.split("=", 1)[1] for o in opts if o.startswith("mem=")), None)
    log = next((o.split("=", 1)[1] for o in opts if o.startswith("log=")), None)
    phys = dict(max_steer=0.6, mu=9.0, steer_red=0.5, power=200.0) if "slow" in opts else None
    if "grip" in opts:
        phys = dict(mu=12.5, max_steer=0.8, steer_red=0.2, power=300.0)
    yaw = 3.14159 if "backward" in opts else 0.0
    print(json.dumps(run(tr, laps, battle, mem, log, phys, quiet="verbose" not in opts, spawn_yaw=yaw), ensure_ascii=False))
