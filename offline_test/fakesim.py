"""
fakesim.py — 싸피레이스 시뮬레이터 '흉내' 환경 (로컬 시험용)

진짜 대회 템플릿의 drive_controller.py 를 그대로 실행하고, 그 아래의 airsim 통신만 가짜로 바꿉니다.
  - 센서 계산(to_middle, moving_angle, 도로 각도, 장애물·상대 거리, 진행률), 벌점(브레이크 0.9),
    1초 정지 리셋, 출발 신호, 0.1초 주기는 모두 진짜 drive_controller.py 코드가 처리
  - 이 파일은 차 물리, 트랙(10 m 경유점), 장애물, 충돌, 출발 신호, 랩 완주만 흉내 냄
  ※ 차 물리 값(그립, 가속, 조향 각 등)은 실제 게임과 다릅니다. 코드가 '대회 API에서 제대로 도는지' 확인용.
"""
import math
import os
import sys
import threading
import types

HERE = os.path.dirname(os.path.abspath(__file__))


# =====================================================================
# 트랙: 꼭짓점 + 모서리 반지름으로 닫힌 코스 생성
# =====================================================================
class Track:
    def __init__(self, name, vertices, radii, width, obstacles=(), res=0.5):
        self.name = name
        self.width = width
        self.half = width / 2.0
        pts = self._build(vertices, radii, res)
        # 첫 직선 중간에서 출발, 출발점 (0,0) 방향 북쪽(+x)으로 맞춤
        x0, y0, h0 = pts[0]
        ch, sh = math.cos(-h0), math.sin(-h0)
        self.x, self.y, self.h = [], [], []
        for x, y, h in pts:
            dx, dy = x - x0, y - y0
            self.x.append(dx * ch - dy * sh)
            self.y.append(dx * sh + dy * ch)
            self.h.append(h - h0)
        self.n = len(self.x)
        self.res = res
        self.length = self.n * res
        # 장애물 (트랙 거리 s, to_middle)
        self.obstacles = []
        # 경유점 위에 정확히 놓인 장애물은 대회 API가 알려 주지 못함(0으로 나눔) → 일부러 3.3 m 비켜 둠
        # 단, s 값 뒤에 '!'를 붙인 문자열이면 그대로 둠 (보이지 않는 장애물 시험용)
        # 's?' 문자열이면 API 목록에서 빼서 '보이지 않는 장애물'로 만듦 (물리 충돌은 있음)
        self.hidden = set()
        conv = []
        for s, m in obstacles:
            if isinstance(s, str):
                conv.append((float(s[:-1]), m, s.endswith("?")))
            else:
                conv.append((s + 3.3, m, False))
        for s, m, hid in conv:
            px, py, h = self.at(s)
            if hid:
                self.hidden.add(len(self.obstacles))
            self.obstacles.append((px - m * math.sin(h), py + m * math.cos(h), h, s, m))
        self.waypoints = []
        s = 0.0
        while s < self.length - 1.0:
            px, py, _ = self.at(s)
            self.waypoints.append([px, py, 0.0])
            s += 10.0

    @staticmethod
    def _build(V, R, res):
        n = len(V)
        tang = []
        for i in range(n):
            p0, p1, p2 = V[i - 1], V[i], V[(i + 1) % n]
            din = (p1[0] - p0[0], p1[1] - p0[1])
            dout = (p2[0] - p1[0], p2[1] - p1[1])
            a_in = math.atan2(din[1], din[0])
            a_out = math.atan2(dout[1], dout[0])
            phi = (a_out - a_in + math.pi) % (2 * math.pi) - math.pi
            r = R[i]
            t = r * math.tan(abs(phi) / 2.0)
            li = math.hypot(*din)
            lo = math.hypot(*dout)
            uin = (din[0] / li, din[1] / li)
            uout = (dout[0] / lo, dout[1] / lo)
            tin = (p1[0] - uin[0] * t, p1[1] - uin[1] * t)
            tout = (p1[0] + uout[0] * t, p1[1] + uout[1] * t)
            tang.append((tin, tout, a_in, phi, r))
        pts = []
        for i in range(n):
            # 직선: 이전 꼭짓점 tout → 이번 꼭짓점 tin
            _, tout_prev, _, _, _ = tang[i - 1]
            tin, tout, a_in, phi, r = tang[i]
            sx, sy = tout_prev
            L = math.hypot(tin[0] - sx, tin[1] - sy)
            steps = max(1, int(L / res))
            for k in range(steps):
                f = k / steps
                pts.append((sx + (tin[0] - sx) * f, sy + (tin[1] - sy) * f, a_in))
            # 호
            arc = abs(phi) * r
            steps = max(1, int(arc / res))
            sgn = 1.0 if phi > 0 else -1.0
            # 중심: tin 에서 진행 방향 오른쪽(sgn>0) 또는 왼쪽
            cx = tin[0] + r * math.cos(a_in + sgn * math.pi / 2)
            cy = tin[1] + r * math.sin(a_in + sgn * math.pi / 2)
            for k in range(steps):
                a = a_in + phi * k / steps
                px = cx + r * math.cos(a - sgn * math.pi / 2)
                py = cy + r * math.sin(a - sgn * math.pi / 2)
                pts.append((px, py, a))
        # 첫 직선 중간에서 시작하도록 회전 (꼭짓점 0의 tout 이후 직선)
        start = None
        cnt = 0
        for i, (x, y, a) in enumerate(pts):
            pass
        # 꼭짓점 1의 직선은 tang[0].tout → tang[1].tin
        tin1 = tang[1][0]
        tout0 = tang[0][1]
        mx, my = (tin1[0] + tout0[0]) / 2, (tin1[1] + tout0[1]) / 2
        best = min(range(len(pts)), key=lambda i: (pts[i][0] - mx) ** 2 + (pts[i][1] - my) ** 2)
        pts = pts[best:] + pts[:best]
        # 연속 방향으로 펼치기
        out = []
        prev = pts[0][2]
        acc = prev
        for x, y, a in pts:
            d = (a - prev + math.pi) % (2 * math.pi) - math.pi
            acc += d
            prev = a
            out.append((x, y, acc))
        return out

    def at(self, s):
        s = s % self.length
        i = int(s / self.res) % self.n
        j = (i + 1) % self.n
        f = s / self.res - int(s / self.res)
        x = self.x[i] + (self.x[j] - self.x[i]) * f
        y = self.y[i] + (self.y[j] - self.y[i]) * f
        dh = (self.h[j] - self.h[i] + math.pi) % (2 * math.pi) - math.pi
        return x, y, self.h[i] + dh * f

    def project(self, x, y, hint):
        """(x,y)에 가장 가까운 중심선 점 번호 (hint 근처 탐색) → (번호, 옆 거리 +오른쪽)"""
        best, bi = 1e18, hint
        for k in range(-60, 61):
            i = (hint + k) % self.n
            d = (self.x[i] - x) ** 2 + (self.y[i] - y) ** 2
            if d < best:
                best, bi = d, i
        h = self.h[bi]
        dx, dy = x - self.x[bi], y - self.y[bi]
        lat = -dx * math.sin(h) + dy * math.cos(h)
        return bi, lat


def make_tracks():
    T = {}
    # Basic Round 흉내: 폭 16 m, 완만한 코너, 장애물 없음 (~1.3 km)
    T["basic"] = Track("basic", [(0, 0), (420, 0), (420, 230), (250, 320), (60, 260), (0, 150)],
                       [70, 80, 60, 90, 60, 70], 16.0)
    # Speed Racing Block 흉내: 폭 16 m, 긴 직선 + 헤어핀 + 장애물 (~1.9 km)
    T["speed"] = Track("speed", [(0, 0), (650, 0), (650, 120), (420, 120), (330, 260), (60, 260), (0, 130)],
                       [35, 30, 40, 45, 50, 40, 45], 16.0,
                       obstacles=[(150, 0.0), (420, 3.0), (430, -3.0), ("780!", 0.0), (1050, 4.0), (1300, -2.5),
                                  (1310, 2.5), (1600, 0.0)])
    # SSAFY Track 흉내: 폭 22 m, 연속 직각 코너 + 다양한 장애물 (~2.4 km)
    T["ssafy"] = Track("ssafy", [(0, 0), (400, 0), (400, 200), (250, 200), (250, 330), (500, 330), (500, 520),
                                 (100, 520), (100, 300), (-60, 300), (-60, 120)],
                       [30, 25, 22, 22, 25, 30, 35, 28, 22, 25, 30], 22.0,
                       obstacles=[(120, 0.0), (125, 6.0), (300, -5.0), (310, 5.0), (520, 0.0), (700, -7.0),
                                  (705, 0.0), (710, 7.0), (980, 3.0), (1250, -4.0), (1500, 0.0), (1505, -8.0),
                                  (1800, 5.0), (2100, -3.0)])
    # Germany 흉내: 폭 14 m, 긴 직선 + 헤어핀 + 시케인 (~2.6 km)
    T["germany"] = Track("germany", [(0, 0), (700, 0), (760, 60), (700, 120), (560, 120), (500, 300), (200, 330),
                                     (120, 200), (30, 240), (-60, 120)],
                         [40, 18, 18, 35, 45, 40, 25, 25, 30, 40], 14.0,
                         obstacles=[(400, -2.0), (1300, 2.0)])
    # 보이지 않는 장애물 시험: API 목록에 없는 장애물이 직선 한가운데 있음
    T["hidden"] = Track("hidden", [(0, 0), (420, 0), (420, 230), (250, 320), (60, 260), (0, 150)],
                        [70, 80, 60, 90, 60, 70], 16.0, obstacles=[("203?", 0.0)])
    return T


# =====================================================================
# 차 물리
# =====================================================================
class Car:
    LEN, WID = 4.6, 2.0

    def __init__(self, name, x, y, spawn_x, spawn_y, track, p):
        self.name = name
        self.spawn = (spawn_x, spawn_y)
        self.track = track
        self.p = p
        self.reset_pos = (x, y)
        self.reset()
        self.collision_ts = 0
        self.has_collided = False
        self.api_enabled = False
        self.controls = None
        self.finished = False
        self.lap = 1
        self.collisions = 0
        self.resets = 0
        self.penalty_frames = 0

    def reset(self):
        self.pedal_q = []                 # 페달(스로틀·브레이크) 지연용 명령 기록 (시각, 스로틀, 브레이크)
        self.x, self.y = self.reset_pos
        self.yaw = 0.0
        self.v = 0.0
        self.steer_act = 0.0
        self.idx, _ = self.track.project(self.x, self.y, 0)
        self.s_total = self.idx * self.track.res if self.idx < self.track.n // 2 else (self.idx - self.track.n) * self.track.res

    def circles(self):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return [(self.x + c * o, self.y + s * o) for o in (-1.6, 0.0, 1.6)]

    def physics(self, h):
        p = self.p
        ctl = self.controls
        st = th = br = 0.0
        rev = False
        if ctl is not None:
            st = max(-1.0, min(1.0, float(ctl.steering))) * p.get("steer_dir", 1.0)
            th = max(-1.0, min(1.0, float(ctl.throttle)))
            br = max(0.0, min(1.0, float(ctl.brake)))
            rev = bool(getattr(ctl, "is_manual_gear", False)) and getattr(ctl, "manual_gear", 0) == -1
        lag = p.get("pedal_lag", 0.0)
        if lag > 0:
            # 실제 시뮬레이터 흉내: 스로틀·브레이크가 lag 초 늦게 들음 (2026-11-25 Map 10 실측 0.22~0.33초)
            now = WORLD.t if WORLD else 0.0
            self.pedal_q.append((now, th, br))
            while len(self.pedal_q) > 1 and self.pedal_q[1][0] <= now - lag:
                self.pedal_q.pop(0)
            if self.pedal_q[0][0] <= now - lag:
                th, br = self.pedal_q[0][1], self.pedal_q[0][2]
            else:
                th, br = 0.0, 0.0
        self.steer_act += (st - self.steer_act) * (1 - math.exp(-h / p["steer_lag"]))
        v = self.v
        _, lat = self.track.project(self.x, self.y, self.idx)
        off = abs(lat) > self.track.half + 0.3
        mu = p["mu_grass"] if off else p["mu"]
        # 조향: 바퀴 각 × 속도 감쇠
        red = 1.0 - p["steer_red"] * min(abs(v) / 33.0, 1.0)
        delta = self.steer_act * p["max_steer"] * red
        kap = math.tan(delta) / p["wheelbase"]
        # 종방향
        if rev:
            a_drive = th * p["rev_acc"]               # th < 0 → 뒤로
        else:
            a_drive = max(th, 0.0) * min(p["a_max"], p["power"] / max(abs(v), 3.0))
        a_brake = br * p["brake"]
        sgn = 1.0 if v > 0.05 else (-1.0 if v < -0.05 else 0.0)
        a_tire = a_drive - a_brake * sgn
        a_lat = v * v * kap
        tot = math.hypot(a_tire, a_lat)
        if tot > mu:
            sc = mu / tot
            a_tire *= sc
            kap *= sc
        a = a_tire - p["drag"] * v * abs(v) - p["roll"] * sgn - (1.5 * sgn if off else 0.0)
        nv = v + a * h
        if sgn != 0 and nv * v < 0 and abs(a_drive) < 1e-6:
            nv = 0.0                                   # 브레이크로 0을 넘어 뒤로 가지 않음
        if sgn == 0 and abs(a_drive) < 1e-6:
            nv = 0.0
        self.v = nv
        self.yaw += self.v * kap * h
        self.x += self.v * math.cos(self.yaw) * h
        self.y += self.v * math.sin(self.yaw) * h
        oi = self.idx
        self.idx, lat = self.track.project(self.x, self.y, self.idx)
        di = self.idx - oi
        if di > self.track.n / 2:
            di -= self.track.n
        elif di < -self.track.n / 2:
            di += self.track.n
        self.s_total += di * self.track.res
        # 벽 (도로 끝 + 4 m)
        wall = self.track.half + 4.0
        if abs(lat) > wall:
            hh = self.track.h[self.idx]
            push = abs(lat) - wall
            sg = 1.0 if lat > 0 else -1.0
            self.x -= -math.sin(hh) * push * sg
            self.y -= math.cos(hh) * push * sg
            self.v *= 0.3
            self.hit()
        # 장애물
        for ox, oy, oh, _, _ in self.track.obstacles:
            for cx, cy in self.circles():
                d = math.hypot(cx - ox, cy - oy)
                if d < 1.0 + 1.3:
                    nx, ny = (cx - ox) / max(d, 1e-6), (cy - oy) / max(d, 1e-6)
                    pen = 2.3 - d
                    self.x += nx * pen
                    self.y += ny * pen
                    hx, hy = math.cos(self.yaw), math.sin(self.yaw)
                    if (hx * nx + hy * ny) * self.v < 0:
                        self.v = 0.0
                    self.hit()
                    break

    def hit(self):
        self.collision_ts += 1
        self.has_collided = True
        t = WORLD.t if WORLD else 0.0
        if t - getattr(self, "last_hit_t", -9.0) > 0.5:
            self.collisions += 1
        self.last_hit_t = t


# =====================================================================
# 가짜 airsim
# =====================================================================
class World:
    def __init__(self, track, cars_cfg, laps=1, start_delay=3.0, phys=None):
        self.track = track
        self.laps = laps
        self.t = 0.0
        self.start_delay = start_delay
        p = dict(mu=10.0, mu_grass=6.5, max_steer=40 * math.pi / 180, wheelbase=2.9, steer_lag=0.08,
                 steer_red=0.35, a_max=7.0, power=230.0, rev_acc=4.0, brake=11.0, drag=0.0012, roll=0.15,
                 pedal_lag=0.0)
        if phys:
            p.update(phys)
        self.p = p
        self.cars = {}
        for name, (X, Y) in cars_cfg.items():
            self.cars[name] = Car(name, X, Y, X, Y, track, p)
        self.lock = threading.Condition()
        self.waiting = 0
        self.participants = 0
        self.gen = 0
        self.frozen_total = 0.0           # 시뮬레이터가 멈춰 있던 시간 합 (기록 시간에서 뺌)

    @property
    def started(self):
        return self.t >= self.start_delay

    def advance(self, dt):
        if not self.started:
            self.t += dt
            return
        # 실제 시뮬레이터 흉내: 출발 뒤 freeze=[(시작초, 길이초), ...] 동안 세계가 멈춤
        #  (2026-11-25 Map 10 실측: 1.1초씩 2번, 그동안 센서 값이 그대로). 봇은 계속 불리고 같은 값을 받음
        wt = self.t - self.start_delay
        if any(a <= wt < a + d for a, d in self.p.get("freeze", ())):
            self.t += dt
            self.frozen_total += dt
            return
        n = 4
        h = dt / n
        names = list(self.cars)
        for _ in range(n):
            for c in self.cars.values():
                c.has_collided = False
                if not c.finished:
                    c.physics(h)
            # 차끼리 충돌
            for i in range(len(names)):
                for j in range(i + 1, len(names)):
                    a, b = self.cars[names[i]], self.cars[names[j]]
                    hit = False
                    for ax, ay in a.circles():
                        for bx, by in b.circles():
                            d = math.hypot(ax - bx, ay - by)
                            if d < 2.0:
                                nx, ny = (ax - bx) / max(d, 1e-6), (ay - by) / max(d, 1e-6)
                                a.x += nx * (2.0 - d) / 2
                                a.y += ny * (2.0 - d) / 2
                                b.x -= nx * (2.0 - d) / 2
                                b.y -= ny * (2.0 - d) / 2
                                hit = True
                    if hit:
                        va, vb = a.v, b.v
                        a.v = 0.7 * va + 0.3 * vb
                        b.v = 0.7 * vb + 0.3 * va
                        a.hit()
                        b.hit()
            for c in self.cars.values():
                if not c.finished and c.s_total >= self.laps * self.track.length:
                    c.finished = True
                    c.finish_time = self.t - self.start_delay - self.frozen_total
                c.lap = min(self.laps, 1 + int(max(c.s_total, 0) // self.track.length))
            self.t += h
        for c in self.cars.values():
            _, lat = self.track.project(c.x, c.y, c.idx)
            if abs(lat) > self.track.half + 1.25 and abs(c.v) * 3.6 > 40 and not c.finished:
                c.penalty_frames += 1

    # 여러 봇이 같은 시계로 움직이도록 (모두 sleep 하면 한 번 진행)
    def sleep(self, dt):
        with self.lock:
            g = self.gen
            self.waiting += 1
            if self.waiting >= self.participants:
                self.advance(0.1)
                self.waiting = 0
                self.gen += 1
                self.lock.notify_all()
            else:
                while g == self.gen:
                    self.lock.wait(timeout=5)

    def leave(self):
        with self.lock:
            self.participants -= 1
            if self.waiting >= self.participants and self.participants > 0:
                self.advance(0.1)
                self.waiting = 0
                self.gen += 1
                self.lock.notify_all()


WORLD = None


class Obj:
    def __init__(self, **kw):
        self.__dict__.update(kw)


class Vector3r:
    def __init__(self, x_val=0.0, y_val=0.0, z_val=0.0):
        self.x_val, self.y_val, self.z_val = x_val, y_val, z_val


class CarControls:
    def __init__(self, throttle=0, steering=0, brake=0, handbrake=False, is_manual_gear=False, manual_gear=0,
                 gear_immediate=True):
        self.throttle, self.steering, self.brake = throttle, steering, brake
        self.handbrake, self.is_manual_gear, self.manual_gear = handbrake, is_manual_gear, manual_gear
        self.gear_immediate = gear_immediate


class CarClient:
    def __init__(self, *a, **k):
        self.vehicle = ""

    def confirmConnection(self):
        return True

    def _car(self, name):
        W = WORLD
        if name and name in W.cars:
            return W.cars[name]
        return W.cars[sorted(W.cars)[0]]

    def enableApiControl(self, flag, name=""):
        self.vehicle = name
        self._car(name).api_enabled = flag

    def isApiControlEnabled(self, name=""):
        return self._car(name).api_enabled

    def getCarState(self, name=""):
        c = self._car(name)
        pos = Vector3r(c.x - c.spawn[0], c.y - c.spawn[1], 0.0)
        return Obj(speed=c.v, kinematics_estimated=Obj(position=pos))

    def simGetCollisionInfo(self, name=""):
        c = self._car(name)
        return Obj(has_collided=c.has_collided, time_stamp=c.collision_ts)

    def getAlgoUserAPI(self, name=""):
        W = WORLD
        c = self._car(name)
        return Obj(wayPoints=[list(w) for w in W.track.waypoints],
                   ac_block_points=[[ox, oy, 0.0] for i, (ox, oy, _, _, _) in enumerate(W.track.obstacles)
                                    if i not in W.track.hidden],
                   ac_road_width_half=W.track.half, ac_player_current_lap=c.lap, ac_max_lap=W.laps,
                   ac_RaceComplete_YN=c.finished)

    def getAlgoAdminAPI(self, name=""):
        return Obj(ac_RaceStart_YN=WORLD.started)

    def setResetLocation(self, x, y, yaw, name=""):
        c = self._car(name)
        return True

    def reset(self):
        c = self._car(self.vehicle)
        if WORLD.started:
            c.resets += 1
        c.reset()

    def setCarControls(self, controls, name=""):
        c = self._car(name)
        c.controls = CarControls(controls.throttle, controls.steering, controls.brake, False,
                                 getattr(controls, "is_manual_gear", False), getattr(controls, "manual_gear", 0))
        if getattr(controls, "throttle", 0) < 0:
            c.controls.is_manual_gear, c.controls.manual_gear = True, -1

    def input_player_lap_progress(self, p, name=""):
        return True

    def ac_rematch_check(self, name=""):
        return True


class FakeTime:
    def time(self):
        return WORLD.t

    def monotonic(self):
        return WORLD.t

    def sleep(self, dt):
        WORLD.sleep(dt)


def install_fakes(docs_dir):
    airsim = types.ModuleType("airsim")
    airsim.CarClient = CarClient
    airsim.CarControls = CarControls
    airsim.Vector3r = Vector3r
    sys.modules["airsim"] = airsim
    win32com = types.ModuleType("win32com")
    shell_mod = types.ModuleType("win32com.shell")

    class _Shell:
        @staticmethod
        def SHGetFolderPath(a, b, c, d):
            return docs_dir

    class _Con:
        CSIDL_PERSONAL = 5

    shell_mod.shell = _Shell
    shell_mod.shellcon = _Con
    win32com.shell = shell_mod
    sys.modules["win32com"] = win32com
    sys.modules["win32com.shell"] = shell_mod
