# -*- coding: utf-8 -*-
"""시험 주행 한 번: 시뮬레이터를 다시 켜고, 맵이 뜨면 봇을 실행하고, 끝나면 정리해서 결과 한 줄을 남긴다.

사용:  .venv/Scripts/python.exe tune/run_trial.py <맵> [--battle] [--tune <json>] [--label <글>]
  맵: 10, 31, 61, 161 (--battle 이면 61 배틀, my_car.py vs my_car_basic.py)
  --tune: my_car.py Config 덮어쓰기 json (없으면 {} = 지금 코드 기본값). 시험 주행은 언제나 '서버 조건 재현' 모드
          (맵 학습값 안 읽고 안 씀, 차 특성은 tune/baseline_car.json)
결과: tune/results/trials.jsonl 에 한 줄 추가, 같은 줄을 마지막에 출력
"""
import argparse
import json
import os
import re
import socket
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.dirname(HERE)
ROOT = os.path.dirname(WS)
SIM_BAT = os.path.join(ROOT, "start_simulator.bat")
PY = os.path.join(WS, ".venv", "Scripts", "python.exe")
RESULTS = os.path.join(HERE, "results")
MENU = {"10": 1, "31": 2, "30": 3, "61": 4, "60": 5, "71": 6, "70": 7, "161": 8, "160": 9}
LIMIT = {"10": 300, "31": 300, "30": 300, "61": 480, "60": 480, "71": 480, "70": 480, "161": 480, "160": 480}
SIM_IMAGES = ["Algo-Win64-Shipping.exe", "Algo.exe"]

sys.path.insert(0, os.path.join(WS, "DrivingInterface"))


def log(msg):
    print("[trial {}] {}".format(time.strftime("%H:%M:%S"), msg), flush=True)


def docs_dir():
    try:
        out = subprocess.run(["powershell", "-NoProfile", "-Command", "[Environment]::GetFolderPath('MyDocuments')"],
                             stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, timeout=20).stdout.decode("utf-8", "replace").strip()
        if out:
            return out
    except Exception:
        pass
    return os.path.join(os.environ.get("USERPROFILE", ""), "Documents")


def start_sim_hidden(map_no, cars):
    """..\\start_simulator.bat 과 같은 settings.json 을 쓰고, 창 없이(-RenderOffScreen) 소리 없이 시뮬레이터를 켬.
    (사용자 요청 2026-11-28: 시험 주행 때 화면을 띄우지 않음. 배치 파일은 고치지 않음)"""
    algo = os.path.join(ROOT, "Simulator", "Algo.exe")
    d = os.path.join(docs_dir(), "AirSim")
    os.makedirs(d, exist_ok=True)
    if cars == 2:
        veh = ('    "Car1": { "VehicleType": "PhysXCar", "X": 0, "Y": -4, "Z": 0 },\n'
               '    "Car2": { "VehicleType": "PhysXCar", "X": 0, "Y": 4, "Z": 0 }\n')
    else:
        veh = '    "Car1": { "VehicleType": "PhysXCar", "X": 0, "Y": 0, "Z": 0 }\n'
    txt = ('{\n  "SettingsVersion": 1.2,\n  "SimMode": "Car",\n  "BGM": true,\n  "EngineSound": false,\n'
           '  "Algo": { "Map": "%s" },\n  "Vehicles": {\n%s  }\n}\n' % (map_no, veh))
    with open(os.path.join(d, "settings.json"), "w", encoding="ascii") as f:
        f.write(txt)
    env = dict(os.environ)
    env["OPENSSL_ia32cap"] = ":~0x20000000"
    res = ["-ResX=1280", "-ResY=480"] if cars == 2 else ["-ResX=640", "-ResY=480"]
    subprocess.Popen([algo] + res + ["-windowed", "-WinX=-4000", "-WinY=-4000", "-nosound"], cwd=os.path.dirname(algo),
                     env=env, stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                     creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))


def move_sim_offscreen():
    """시뮬레이터 창(제목 'Algo')을 포커스를 뺏지 않고 화면 밖으로 옮김. 숨기거나 최소화하면 렌더링·시간 흐름이
    바뀔 수 있어서 창은 그대로 두고 위치만 옮김. 옮긴 창 수를 돌려줌."""
    try:
        import ctypes
        from ctypes import wintypes
    except Exception:
        return 0
    user32 = ctypes.windll.user32
    found = []

    @ctypes.WINFUNCTYPE(wintypes.BOOL, wintypes.HWND, wintypes.LPARAM)
    def cb(hwnd, _):
        if user32.IsWindowVisible(hwnd):
            buf = ctypes.create_unicode_buffer(64)
            user32.GetWindowTextW(hwnd, buf, 64)
            if buf.value.strip() == "Algo":
                found.append(hwnd)
        return True
    user32.EnumWindows(cb, 0)
    SWP_NOSIZE, SWP_NOZORDER, SWP_NOACTIVATE = 0x0001, 0x0004, 0x0010
    for h in found:
        r = wintypes.RECT()
        user32.GetWindowRect(h, ctypes.byref(r))
        if r.left > -3000:
            user32.SetWindowPos(h, 0, -4000, -4000, 0, 0, SWP_NOSIZE | SWP_NOZORDER | SWP_NOACTIVATE)
    return len(found)


def kill_sim():
    for im in SIM_IMAGES:
        subprocess.run(["taskkill", "/F", "/IM", im], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def kill_pid(p):
    if p is not None and p.poll() is None:
        subprocess.run(["taskkill", "/F", "/T", "/PID", str(p.pid)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def port_open(port=41451):
    s = socket.socket()
    s.settimeout(1.0)
    try:
        s.connect(("127.0.0.1", port))
        return True
    except Exception:
        return False
    finally:
        s.close()


def api_client(timeout=5):
    import airsim
    return airsim.CarClient(timeout_value=timeout)


def plain(obj):
    """msgpack 객체를 json 으로 쓸 수 있게 (경유점 같은 큰 목록은 길이만)."""
    d = {}
    for k, v in getattr(obj, "__dict__", {}).items():
        if isinstance(v, (list, tuple)) and len(v) > 12:
            d[k] = "len={}".format(len(v))
        elif isinstance(v, (int, float, str, bool)) or v is None:
            d[k] = v
        elif isinstance(v, (list, tuple)):
            d[k] = [plain(x) if hasattr(x, "__dict__") else x for x in v]
        elif hasattr(v, "__dict__"):
            d[k] = plain(v)
        else:
            d[k] = str(v)
    return d


def wait_map(limit=150, hide=False):
    t0 = time.time()
    while time.time() - t0 < limit:
        if hide:
            move_sim_offscreen()
        if port_open():
            try:
                c = api_client()
                u = c.getAlgoUserAPI()
                if getattr(u, "ac_road_width_half", 0) and len(getattr(u, "wayPoints", []) or []) > 0:
                    time.sleep(3.0)
                    return True
            except Exception:
                pass
        time.sleep(2.0)
    return False


def start_bot(script, out_path, tune_path):
    env = dict(os.environ)
    env["SSAFY_LOCAL"] = "1"
    env["SSAFY_TUNE"] = tune_path
    env["PYTHONIOENCODING"] = "utf-8"
    env["PYTHONUNBUFFERED"] = "1"
    f = open(out_path, "w", encoding="utf-8")
    p = subprocess.Popen([PY, script], cwd=WS, env=env, stdout=f, stderr=subprocess.STDOUT,
                         stdin=subprocess.DEVNULL, creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
    return p, f


def read(path):
    try:
        with open(path, encoding="utf-8", errors="replace") as f:
            return f.read()
    except Exception:
        return ""


def parse_log(path):
    """봇 로그(jsonl) 요약: 완주 시간(로그 기준, 멈춘 프레임 뺌), 충돌·복구·리셋 사건 수."""
    out = {"log": os.path.relpath(path, WS) if path else None}
    if not path:
        return out
    rows = []
    with open(path, encoding="utf-8") as f:
        for line in f:
            try:
                rows.append(json.loads(line))
            except Exception:
                pass
    if not rows:
        return out
    t_start = next((r["t"] for r in rows if (r.get("s") or 0) > 0.3), None)
    t_fin = next((r["t"] for r in rows if (r.get("prog") or 0) >= 99.9), None)
    stale_t = 0.0
    col_ev = rev_ev = tp_ev = 0
    last_col = -9.0
    prev_state = "DRIVE"
    prev_tp = False
    for i, r in enumerate(rows):
        if i and r.get("stale") and (t_fin is None or r["t"] <= t_fin) and (t_start is None or r["t"] >= t_start):
            stale_t += r["t"] - rows[i - 1]["t"]
        if r.get("col"):
            if r["t"] - last_col > 1.0:
                col_ev += 1
            last_col = r["t"]
        st = r.get("state")
        if st == "REVERSE" and prev_state != "REVERSE":
            rev_ev += 1
        prev_state = st
        if r.get("tp") and not prev_tp:
            tp_ev += 1
        prev_tp = bool(r.get("tp"))
    out.update({
        "time_log": round(t_fin - t_start, 2) if t_start is not None and t_fin is not None else None,
        "stale_s": round(stale_t, 2),
        "col_events": col_ev, "reverse_events": rev_ev, "reset_events": tp_ev,
        "vmax": round(max((r.get("v") or 0) for r in rows), 1),
        "alat_end": rows[-1].get("alat"), "abrk_end": rows[-1].get("abrk"),
        "frames": len(rows),
    })
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("map")
    ap.add_argument("--battle", action="store_true")
    ap.add_argument("--tune", default=None)
    ap.add_argument("--label", default="")
    ap.add_argument("--show", action="store_true", help="예전처럼 ..\\start_simulator.bat 으로 창을 띄워 실행")
    a = ap.parse_args()
    m = str(a.map)
    if a.battle:
        m = "61"
    if m not in MENU:
        print("unknown map", m)
        return 2
    menu = 10 if a.battle else MENU[m]
    os.makedirs(RESULTS, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    tune = {}
    if a.tune:
        with open(a.tune, encoding="utf-8") as f:
            tune = json.load(f)
    tune_path = os.path.join(RESULTS, "tune_" + stamp + ".json")
    with open(tune_path, "w", encoding="utf-8") as f:
        json.dump(tune, f, ensure_ascii=False)
    rec = {"ts": stamp, "map": m, "battle": a.battle, "label": a.label, "tune": tune, "finished": False}
    bots = []
    try:
        log("map {}{} — restarting simulator".format(m, " battle" if a.battle else ""))
        kill_sim()
        time.sleep(2.0)
        if a.show:
            subprocess.run(["cmd", "/c", SIM_BAT, str(menu)], cwd=ROOT, stdin=subprocess.DEVNULL,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=90)
        else:
            start_sim_hidden(m, 2 if a.battle else 1)
            t_w = time.time()
            while time.time() - t_w < 40 and not move_sim_offscreen():   # 창이 생기자마자 화면 밖으로
                time.sleep(0.2)
        if not wait_map(hide=not a.show):
            rec["error"] = "sim_fail"
            raise RuntimeError("simulator not ready")
        log("map ready — starting bot")
        logs_before = set(os.listdir(os.path.join(WS, "logs"))) if os.path.isdir(os.path.join(WS, "logs")) else set()
        out1 = os.path.join(RESULTS, "bot_" + stamp + ".txt")
        p1, f1 = start_bot("my_car.py", out1, tune_path)
        bots.append((p1, f1, out1))
        if a.battle:
            t0 = time.time()
            while time.time() - t0 < 30 and "Player name" not in read(out1):
                time.sleep(0.5)
            time.sleep(2.0)
            out2 = os.path.join(RESULTS, "bot_" + stamp + "_basic.txt")
            p2, f2 = start_bot("my_car_basic.py", out2, tune_path)
            bots.append((p2, f2, out2))
        t0 = time.time()
        limit = LIMIT[m]
        while time.time() - t0 < limit and any(p.poll() is None for p, _, _ in bots):
            if not a.show:
                move_sim_offscreen()
            time.sleep(1.0)
        rec["wall_s"] = round(time.time() - t0, 1)
        rec["timeout"] = any(p.poll() is None for p, _, _ in bots)
        # 공식 기록 (시뮬레이터를 닫기 전에)
        try:
            c = api_client()
            names = ["Car1", "Car2"] if a.battle else [""]
            api = {}
            for nm in names:
                api[nm or "Car1"] = {"user": plain(c.getAlgoUserAPI(nm)), "admin": plain(c.getAlgoAdminAPI(nm))}
            rec["api"] = api
        except Exception as e:
            rec["api_error"] = str(e)
        for p, f, _ in bots:
            kill_pid(p)
            f.close()
        txt = read(out1)
        rec["bot_exit"] = bots[0][0].returncode
        rec["penalty_frames"] = txt.count("Penalty applied")
        rec["brain_errors"] = txt.count("brain error")
        rec["laps"] = [float(x) for x in re.findall(r"\[MyCar\] lap \d+ ([0-9.]+)s", txt)]
        rec["player"] = (re.findall(r"Player name : (\S+)", txt) or [None])[0]
        new_logs = sorted(set(os.listdir(os.path.join(WS, "logs"))) - logs_before)
        lp = os.path.join(WS, "logs", new_logs[-1]) if new_logs else None
        rec.update(parse_log(lp))
        u = rec.get("api", {}).get("Car1", {}).get("user", {})
        rec["finished"] = bool(u.get("ac_RaceComplete_YN")) and not rec["timeout"]
        adm = rec.get("api", {}).get("Car1", {}).get("admin", {})
        rec["time_official"] = round(adm["ac_PlayerRaceTime"], 2) if adm.get("ac_PlayerRaceTime") else None
        rec["time"] = rec["time_official"] if rec["time_official"] else rec.get("time_log")
        if a.battle:
            adm2 = rec.get("api", {}).get("Car2", {}).get("admin", {})
            rec["opp_time_official"] = round(adm2["ac_PlayerRaceTime"], 2) if adm2.get("ac_PlayerRaceTime") else None
            u2 = rec.get("api", {}).get("Car2", {}).get("user", {})
            rec["opp_finished"] = bool(u2.get("ac_RaceComplete_YN"))
        if a.battle and len(bots) > 1:
            t2 = read(bots[1][2])
            rec["opp_exit"] = bots[1][0].returncode
            rec["opp_penalty_frames"] = t2.count("Penalty applied")
            rec["opp_player"] = (re.findall(r"Player name : (\S+)", t2) or [None])[0]
    except Exception as e:
        rec.setdefault("error", str(e))
    finally:
        for p, f, _ in bots:
            kill_pid(p)
            try:
                f.close()
            except Exception:
                pass
        kill_sim()
    with open(os.path.join(RESULTS, "trials.jsonl"), "a", encoding="utf-8") as f:
        f.write(json.dumps(rec, ensure_ascii=False) + "\n")
    short = {k: rec.get(k) for k in ("ts", "map", "battle", "label", "finished", "time", "laps", "col_events",
                                     "reverse_events", "reset_events", "penalty_frames", "brain_errors", "error", "log")}
    print("RESULT " + json.dumps(short, ensure_ascii=False), flush=True)
    return 0 if rec.get("finished") else 1


if __name__ == "__main__":
    sys.exit(main())
