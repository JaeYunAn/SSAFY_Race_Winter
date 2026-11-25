# =====================================================================
#  my_car.py — 싸피레이스(SSAFY RACE) 제출용 주행 AI
#
#  race_brain.py 의 기능을 대회 템플릿(DrivingClient) 한 파일 안에 그대로 옮긴 버전입니다.
#  제출 규칙: my_car.py 한 파일만 제출 / import·생성자·control_driving 수정 가능 /
#            사용자 함수·클래스 추가 가능 / 부모 클래스 값은 self.half_road_limit 만 사용
#
#  데이터 흐름 (위에서 아래로)
#   1. SsafyAdapter   : 대회 센서 → 내부 단위. 경유점(10 m 간격 고정점) 구조를 그대로 해석
#                       - 경유점 통과 감지, 앞뒤 경유점 사이의 '매끈한 기준선' 복원
#                       - 경유점을 지날 때 moving_angle·to_middle 이 튀는 현상 제거
#                       - 트랙 위치(m) 직접 계산, 바퀴 길이(랩 길이) 자동 감지
#   2. RoadGeometry   : 기준선 좌표·곡률
#   3. PathPlanner    : 2차 격자 DP(장애물 위상 결정) + 최소 곡률 평활화(레이싱 라인)
#                       움직이는 상대 차는 '내가 그 지점에 도착할 시각'의 위치로 예측해 회피
#   4. SpeedPlanner   : 마찰원 기반 전진·후진 패스 속도 프로파일 (가장 늦은 브레이킹)
#   5. Controllers    : 순수 추종 + 곡률 서보(적분) 조향, 피드포워드+P 가감속
#   6. Estimators     : 조향 이득(곡률형/요레이트형 자동 판별, 출발 직후 미세 위빙),
#                       횡 그립(AIMD, 국소/전체 구분), 브레이크 이득, 풀브레이크 감속 모델, 가속·저항
#   7. GripMap        : 트랙 위치(10 m 구간)별 그립 배율, 위험 구간 기억
#   8. Strategy       : 플랜 A/B(최고 랩 대비), 전력/관리, 침착 모드, 완만한 전환
#   9. Recovery       : 충돌 후진 탈출(돌아 나갈 거리 확보), 역주행 U턴, 출발 신호 전 대기,
#                       API가 알려 주지 않은 장애물에 부딪히면 그 자리를 기억해 다음부터 피함
#  10. Memory / Log   : 학습값 저장(파일 또는 코드 안 EMBEDDED_MEMORY), 프레임 로그(선택)
#
#  0.1초 제어 대응: 조향 지연 후보 5개 중 실제와 가장 잘 맞는 지연을 골라 학습,
#                  명령이 실제로 듣는 시점의 위치·방향을 예측해서 조향, 출발 직후 조향 부호 점검
#
#  쓰는 법
#   - 로컬 연습: run_bot.bat 으로 실행하면 SSAFY_LOCAL=1 이 켜져서 맵별로 자동 저장
#       memory_map<맵번호>.json : 그립·브레이크·조향 특성 + 그 맵의 최고 랩·고스트·위험 구간
#       memory_car.json         : 가장 최근 학습값 (처음 달리는 맵에서는 차 특성만 가져다 씀)
#       logs/drive_map<맵번호>_날짜_시간.jsonl : 주행 로그
#   - 제출: 이 파일 그대로 제출하면 서버에서는 저장 기능이 자동으로 꺼짐
#       → memory_map<맵번호>.json 내용을 아래 EMBEDDED_MEMORY 에 붙여 넣으면 서버에서도 그 차 특성으로 출발
#         (서버 맵을 코드가 알 수 없으므로 랩 길이·고스트·구간 그립 같은 맵 기록은 쓰지 않음)
#
#  센서 해석의 근거: 대회 템플릿 DrivingInterface/drive_controller.py 의 계산 코드
#   - to_middle      : 현재 두 경유점을 잇는 직선까지의 수직 거리 (+오른쪽)
#   - moving_angle   : 0.1초 동안 실제로 움직인 방향 vs 현재 경유점 구간 방향 (+오른쪽)
#   - track_forward_angles[k] : 다음 경유점부터 k번째 구간 방향 (현재 구간 기준, 정수 도)
#   - distance_to_way_points  : 앞 경유점 20개까지의 직선 거리
#   - 벌점: |to_middle| > half_road_limit 이고 40 km/h 초과면 브레이크 0.9 강제
#   - 1초간 속도가 똑같으면 시뮬레이터 reset, 30초 정지하면 경기 종료
# =====================================================================
from DrivingInterface.drive_controller import DrivingController

import bisect
import json
import math
import os
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Tuple

DEG = math.pi / 180.0
BIG = 1e6

# 연습 주행에서 저장한 학습값(memory_map<맵번호>.json 내용)을 여기에 붙여 넣으면 서버에서도 차 특성
# (조향 반응·가속·제동·그립)을 그 값으로 시작합니다. 맵 기록(랩 길이·고스트·구간 그립)은 쓰지 않습니다.
# 예) EMBEDDED_MEMORY = {"a_lat": 11.2, "a_brake": 10.1, ...}   (없으면 None)
EMBEDDED_MEMORY = None



def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def lerp_list(xs: Sequence[float], ys: Sequence[float], x: float) -> float:
    """정렬된 xs에서 선형 보간 (범위 밖은 끝값)."""
    if x <= xs[0]:
        return ys[0]
    if x >= xs[-1]:
        return ys[-1]
    lo, hi = 0, len(xs) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if xs[mid] <= x:
            lo = mid
        else:
            hi = mid
    t = (x - xs[lo]) / (xs[hi] - xs[lo])
    return ys[lo] + (ys[hi] - ys[lo]) * t



# =====================================================================
# 설정
# =====================================================================
@dataclass
class Config:
    dt: float = 0.1                     # 대회 제어 주기 0.1초
    dt_from_clock: bool = True          # 실제 시계로 프레임 간격을 잼 (처리가 늦어 주기가 늘어도 정확)
    half_road_width: float = 9.25       # 실행 시 self.half_road_limit(도로 절반 + 1.25)로 덮어씀
    car_length: float = 4.6
    car_width: float = 2.5              # 대회 API가 쓰는 차 폭(절반 1.25 m)
    obstacle_length: float = 2.0        # 장애물은 모든 맵에서 2 m × 2 m
    obstacle_width: float = 2.0
    edge_margin: float = 0.5            # 차 중심이 도로 끝선 안쪽으로 남길 여유(m). 벌점 경계는 끝선 +1.25 m
    angle_step: float = 10.0            # 경유점 간격 (m)
    max_speed_kmh: float = 250.0

    # 경로 계획
    plan_ds: float = 6.25
    plan_stations: int = 24             # 6.25 m × 24 = 150 m
    lanes: int = 11
    profile_length: float = 200.0
    kappa_floor: float = 0.85           # 레이싱 라인이 줄일 수 있는 곡률의 한계 (속도 계획은 보수적으로)
    w_curv: float = 4.0e3               # 곡률² 비용
    w_slope: float = 0.8                # 옆으로 움직이는 비용
    w_cons: float = 0.15                # 지난 계획과 다르면 드는 비용 (좌우 흔들림 방지)
    w_edge: float = 3.0
    w_near: float = 6.0                 # 장애물 근처 비용 (문호준 2단계 위험 비용)
    lat_buffer: float = 0.8             # 0.1초 제어라 추종 오차를 감안해 옆 여유를 넉넉히
    long_buffer: float = 2.0
    smooth_iters: int = 30

    # 초기 추정값 (일부러 보수적으로, 달리면서 실측으로 바뀜)
    a_lat_init: float = 9.0
    a_brake_init: float = 8.0
    a_drive_init: float = 6.0
    drag_init: float = 0.001
    steer_gain_init: float = 0.2        # 조향 1당 곡률(1/m). 바퀴 최대 각 약 40~45° 기준 추정
    steer_lag_match: float = 0.15       # 조향→측정 곡률 지연 추정(초)
    a_cmd_smooth: float = 0.5           # 가감속 명령 거르기 (1=안 거름)
    steer_smith: float = 0.30           # 조향 지연 보상 창 최대(초): 이 시간 안에 보낸 목표 곡률은 '아직 안 돈 몫'으로 예측에 더함. 0 이면 끔
    speed_lag: float = 0.25             # 스로틀·브레이크가 실제로 듣기까지 걸리는 시간(초). 2026-11-25 Map 10 실측 0.22~0.33
    gap_comp: bool = True               # 브레이크 뒤 구동 공백 보상 (제동 끝 무렵 목표를 학습값만큼 올림). 끄려면 False
    gap_max: float = 4.0                # 구동 공백 보상 학습값 최대 (m/s). 실측 평균 약 3.3
    brake_ellipse: bool = True          # 제동 방향 그립을 따로 학습해 속도 계획의 마찰원을 앞뒤로 긴 타원으로. 끄려면 False
    brake_cap_margin: float = 1.03      # 계획에는 실제로 낸 최대 제동보다 이만큼만 더 씀 (조금씩 늘려 감)
    sysid_weave: bool = True            # 출발 직후 미세 위빙으로 조향 특성 파악 (메모리 있으면 생략)
    sysid_amp: float = 0.05
    sysid_freq: float = 0.6
    reverse_brake: bool = False         # True: 강한 제동 때 throttle −1(후진 기어)도 같이 사용 (7기 팀 관찰)

    # 전략
    # 슈마허 + 알베르토 플랜 A/B 속도 배율 (학습한 그립에 곱함). 0.92/0.96 → 0.96/1.00 (2026-11-25)
    #  근거: Map 10 두 랩 동안 미끄러짐 0회, 코너 탈출 옆 가속 10.6 m/s² 까지 깨끗, 풀브레이크+옆 가속 합 약 13 m/s².
    #  a_lat 자체가 '깨끗하게 써 본 값의 5% 위'까지만 오르는 보수적 학습이라 배율까지 낮게 잡을 필요는 적음.
    #  미끄러지면 해밀턴 그립 지도·AIMD 가 그 자리/전체를 깎음
    factor_plan_a: float = 0.96
    factor_plan_b: float = 1.00
    factor_manage: float = 0.88
    factor_calm: float = 0.82
    factor_rate: float = 0.08           # 세나: 배율이 1초에 바뀔 수 있는 최대치
    race_with_others: bool = True
    practice_mode: bool = True
    bin_len: float = 10.0               # 그립 지도·고스트 구간 길이 (m)

    # 메모리·로그 (서버 제출 때는 둘 다 None 권장: 파일 쓰기가 막혀 있을 수 있음)
    memory_path: Optional[str] = None           # 이 맵의 학습값 (차 특성 + 랩 길이·고스트·구간 그립·숨은 장애물)
    memory_shared_path: Optional[str] = None    # 맵과 상관없이 쓸 차 특성(조향·가속·제동·그립)만 읽는 파일
    log_path: Optional[str] = None
    verbose: bool = True                # 랩이 끝날 때 한 줄 요약 출력


# =====================================================================
# 1. 센서 변환 (대회 API 전용)
# =====================================================================
@dataclass
class Frame:
    v: float                                  # m/s (후진이면 음수)
    n: float                                  # 매끈한 기준선 기준 옆 위치 (m, +오른쪽)
    psi: float                                # 기준선 대비 진행 방향 (rad, +오른쪽)
    obstacles: List[Tuple[float, float]]      # (거리 m, n m)
    opponents: List[Tuple[float, float, float]]  # (거리 m, n m, 속도 m/s)
    collided: bool
    half: float                               # 벌점 경계 (도로 절반 + 1.25)
    knot_d: List[float] = field(default_factory=list)   # 기준선 꼭짓점 거리 (내 옆 = 0)
    knot_th: List[float] = field(default_factory=list)  # 꼭짓점 방향 (rad, 내 위치 기준선 방향 = 0)
    kr0: float = 0.0                          # 내 위치 기준선 곡률
    kr_next: float = 0.0                      # 다음 구간 곡률
    s: float = 0.0                            # 출발점부터 기준선을 따라 온 거리 (m)
    moving_forward: bool = True
    progress_api: Optional[float] = None      # API lap_progress (%)
    n_raw: float = 0.0                        # API to_middle 그대로
    teleport: bool = False                    # 리셋 등으로 위치가 갑자기 바뀜
    lap_len_found: Optional[float] = None     # 이번 프레임에 랩 길이를 알아냈으면 그 값
    time: Optional[float] = None


def _get(raw: Any, name: str, default: Any = None) -> Any:
    if isinstance(raw, dict):
        return raw.get(name, default)
    return getattr(raw, name, default)


def _field(item: Any, name: str, idx: int, default: float = 0.0) -> float:
    if isinstance(item, dict):
        return float(item.get(name, default))
    if isinstance(item, (list, tuple)):
        return float(item[idx]) if len(item) > idx else default
    return float(getattr(item, name, default))


class SsafyAdapter:
    """대회 센서를 내부 표현으로 바꿈.

    경유점은 트랙 중앙선 위 약 10 m 간격의 '고정된' 점이고, 모든 센서는 내 차가 있는
    구간(앞뒤 두 경유점을 잇는 직선)을 기준으로 계산된다. 그래서 경유점을 지날 때마다
    기준 직선이 바뀌면서 moving_angle·to_middle·도로 각도가 한꺼번에 튄다.
    여기서는 꼭짓점(경유점)의 방향을 앞뒤 구간 방향의 평균으로 두고 그 사이를 선형으로 잇는
    '매끈한 기준선'을 만들어, 경유점을 지나도 값이 이어지게 한다."""

    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.L = cfg.angle_step
        self.prev_along: Optional[float] = None
        self.prev_th: Optional[List[float]] = None
        self.prev_dist: List[float] = []
        self.prev_seg_rel = 0.0          # 바로 전 구간의 방향 (현재 구간 기준, rad)
        self.wp_count = 0                # 출발 뒤 지난 경유점 수 (뒤로 지나면 감소)
        self.u0: Optional[float] = None  # 출발 순간 구간 안 위치
        self.seq: List[int] = []         # 경유점마다 꺾인 각도(도) 기록 → 랩 길이 감지
        self.lap_wp: Optional[int] = None
        self.s_per_prog: Optional[float] = None
        self.resyncs = 0
        self.passes = 0
        self.start_win: Optional[List[int]] = None
        self.cum_turn = 0

    # ---- 구간 기준 직선 대비 매끈한 기준선의 옆 위치 ----
    @staticmethod
    def _bulge(a0: float, a1: float, u: float, L: float) -> float:
        """구간 시작 방향 a0, 끝 방향 a1 (구간 직선 기준, rad)일 때 u 지점의 기준선 옆 위치."""
        return a0 * u + (a1 - a0) * u * u / (2.0 * L)

    def parse(self, raw: Any, dt: float, prev_throttle: float, half_limit: float) -> Frame:
        c, L = self.cfg, self.L
        fwd = bool(_get(raw, "moving_forward", True))
        v = float(_get(raw, "speed", 0.0) or 0.0) / 3.6
        if v > 0.0 and not fwd and prev_throttle < -0.05:
            v = -v                                       # 후진 기어로 뒤로 가는 중
        tm = float(_get(raw, "to_middle", 0.0) or 0.0)
        ma = float(_get(raw, "moving_angle", 0.0) or 0.0) * DEG
        ang = list(_get(raw, "track_forward_angles", []) or [])
        if not ang:
            ang = [0.0] * 20
        while len(ang) < 20:
            ang.append(ang[-1])
        th = [float(a) * DEG for a in ang[:20]]
        dist = list(_get(raw, "distance_to_way_points", []) or [])
        prog = _get(raw, "lap_progress", None)
        prog = float(prog) if prog is not None else None

        # ---- 다음 경유점까지 남은 기준선 거리 (along) 와 경유점 통과 감지 ----
        # 경유점 20개까지의 거리 목록을 지난 프레임과 비교: 한 칸 지났으면 새 목록[k] ≈ 옛 목록[k+1] − 이동거리.
        # 먼 경유점일수록 이 관계가 정확해서, 충돌·후진으로 속도 추정이 틀려도 통과 여부를 확실히 가려냄.
        a_mag = math.sqrt(max(dist[0] ** 2 - tm * tm, 0.0)) if dist else 0.5 * L
        teleport = False
        k_pass = 0
        if self.prev_along is None or not dist or not self.prev_dist:
            along = a_mag
        else:
            po_d = self.prev_dist
            best = None
            for h in (0, 1, -1, 2):
                ds_ = []
                for k in range(2, 17):
                    j = k + h
                    if 0 <= j < len(po_d) and k < len(dist):
                        ds_.append(po_d[j] - dist[k])
                if len(ds_) < 5:
                    continue
                mean = sum(ds_) / len(ds_)
                sd = math.sqrt(sum((x - mean) ** 2 for x in ds_) / len(ds_))
                score = sd + 0.5 * abs(mean - v * dt) + 0.2 * abs(h)
                if best is None or score < best[0]:
                    best = (score, h, mean)
            if best is None:
                along = a_mag
            else:
                score, k_pass, moved = best
                pred = self.prev_along - moved + k_pass * L
                along = a_mag if abs(a_mag - pred) <= abs(-a_mag - pred) else -a_mag
                if score > 6.0:
                    teleport = True
        if teleport:
            # 리셋 등으로 순간이동: API 진행률로 대략 위치를 다시 잡음
            self.resyncs += 1
            along = a_mag
            self.prev_seg_rel = 0.0
            if self.s_per_prog and prog is not None:
                s_target = self.s_per_prog * prog
                self.wp_count = int(round((s_target + (self.u0 or 0.0) - (L - along)) / L))
        elif k_pass != 0 and self.prev_th is not None:
            po = self.prev_th
            if k_pass >= 1:
                turns = [po[0]] if k_pass == 1 else [po[0], po[1] - po[0]]
                for tr in turns:
                    self.seq.append(int(round(tr / DEG)))
                self.prev_seg_rel = -(po[0] if k_pass == 1 else po[1] - po[0])
                self.passes += k_pass
            else:                                       # 뒤로 경유점을 넘어감 (후진)
                for _ in range(-k_pass):
                    if self.seq:
                        self.seq.pop()
                self.prev_seg_rel = 0.0
            self.wp_count += k_pass
        self.prev_along = along
        self.prev_th = th
        self.prev_dist = list(dist)

        # ---- 매끈한 기준선 ----
        u = L - along                                  # 현재 구간 시작점(뒤 경유점)부터 온 거리
        if self.u0 is None:
            self.u0 = u
        Vp = 0.5 * self.prev_seg_rel                   # 뒤 경유점 방향
        V = [0.5 * th[0]] + [0.5 * (th[k - 1] + th[k]) for k in range(1, 20)]   # 앞 경유점들 방향
        f = clamp(u / L, -0.6, 1.6)
        h_c = Vp + (V[0] - Vp) * f                     # 내 위치의 기준선 방향
        n = tm - self._bulge(Vp, V[0], u, L)
        u_mid = u - 0.5 * max(v, 0.0) * dt             # moving_angle 은 지난 0.1초 평균 방향 → 그 중간 지점 기준
        h_mid = Vp + (V[0] - Vp) * clamp(u_mid / L, -0.6, 1.6)
        psi = ma - h_mid
        kr0 = (V[0] - Vp) / L
        kr_next = (V[1] - V[0]) / L

        knot_d = [0.0]
        knot_th = [0.0]
        for k in range(20):
            dk = along + L * k
            if dk > 0.5:
                knot_d.append(dk)
                knot_th.append(V[k] - h_c)
        knot_d.append(along + L * 20)
        knot_th.append(th[19] - h_c)

        def lat_offset(d: float) -> float:
            if d < along:
                return self._bulge(Vp, V[0], clamp(u + d, -L, 2 * L), L)
            k = min(int((d - along) // L), 19)
            uu = d - along - k * L
            a0 = V[k] - th[k]
            a1 = (V[k + 1] if k + 1 < 20 else th[k]) - th[k]
            return self._bulge(a0, a1, uu, L)

        obstacles = []
        for o in _get(raw, "track_forward_obstacles", []) or []:
            d = _field(o, "dist", 0)
            m = _field(o, "to_middle", 1)
            obstacles.append((d, m - lat_offset(d)))
        opponents = []
        for o in _get(raw, "opponent_cars_info", []) or []:
            d = _field(o, "dist", 1)
            m = _field(o, "to_middle", 2)
            sp = _field(o, "speed", 3) / 3.6
            if d > -u:
                m -= lat_offset(d)
            opponents.append((d, m, sp))

        s = self.wp_count * L + u - self.u0
        if prog is not None and prog > 2.0 and s > 50.0 and not teleport:
            r = s / prog
            self.s_per_prog = r if self.s_per_prog is None else self.s_per_prog + 0.05 * (r - self.s_per_prog)

        if self.start_win is None:
            self.start_win = [int(round(a)) for a in ang[:20]]
        if k_pass >= 1:
            self.cum_turn += sum(self.seq[-k_pass:]) if len(self.seq) >= k_pass else 0
        lap_found = None
        if k_pass >= 1 and self.lap_wp is None and not teleport:
            lap_found = self._detect_lap([int(round(a)) for a in ang[:20]])

        return Frame(v=v, n=n, psi=psi, obstacles=obstacles, opponents=opponents,
                     collided=bool(_get(raw, "collided", False)), half=half_limit,
                     knot_d=knot_d, knot_th=knot_th, kr0=kr0, kr_next=kr_next, s=s,
                     moving_forward=fwd, progress_api=prog, n_raw=tm, teleport=teleport,
                     lap_len_found=lap_found)

    def _detect_lap(self, win: List[int]) -> Optional[float]:
        """한 바퀴 길이 감지.
        ① 지금 보이는 앞 20구간 각도가 출발할 때 본 것과 같고(±1도), 지금까지 꺾인 각도 합이 ±360도면
           바로 지금이 출발선 (출발선을 지나는 순간에 알아냄)
        ② ①이 불가능한 경우(출발 앞이 긴 직선 등) 꺾인 각도 기록에서 출발 직후와 같은 모양이 다시 나오면 한 바퀴"""
        sw = self.start_win or []
        if (self.wp_count >= 25 and sw and sum(abs(x) for x in sw) >= 15
                and abs(abs(self.cum_turn) - 360) < 25 and all(abs(a - b) <= 1 for a, b in zip(win, sw))):
            self.lap_wp = self.wp_count
            return self.wp_count * self.L
        seq = self.seq
        n = len(seq)
        for m in (30, 60, 90):
            W = n - m
            if W < 25:
                continue
            ref = seq[0:m]
            if sum(abs(x) for x in ref) < 20:
                continue
            if all(abs(a - b) <= 1 for a, b in zip(seq[W:n], ref)) and abs(abs(sum(seq[0:W])) - 360) < 25:
                self.lap_wp = W
                return W * self.L
        return None


class AlphaBeta:
    """잡음 섞인 값과 그 변화율을 함께 추정. 프레임 간격이 달라도 같은 시간 상수(tau)로 동작."""

    def __init__(self, tau: float):
        self.tau = tau
        self.x: Optional[float] = None
        self.r = 0.0

    def update(self, z: float, dt: float) -> Tuple[float, float]:
        if self.x is None:
            self.x = z
            return z, 0.0
        a = 1.0 - math.exp(-dt / self.tau)
        b = a * a / (2.0 - a)
        xp = self.x + self.r * dt
        res = z - xp
        self.x = xp + a * res
        self.r += b * res / dt
        return self.x, self.r

    def reset(self, z: float) -> None:
        self.x, self.r = z, 0.0



# =====================================================================
# 2. 도로 형상
# =====================================================================
class RoadGeometry:
    """꼭짓점(거리, 방향) 목록으로 기준선 좌표를 복원. x=앞, y=오른쪽, 내 옆 기준선 점이 원점.
    꼭짓점 사이 방향은 선형으로 바뀐다고 본다 (구간마다 곡률 일정)."""

    def __init__(self, ds: List[float], th: List[float]):
        self.ds = list(ds)
        self.th = list(th)
        self.cx = [0.0]
        self.cy = [0.0]
        for i in range(1, len(self.ds)):
            seg = self.ds[i] - self.ds[i - 1]
            mid = 0.5 * (self.th[i - 1] + self.th[i])
            self.cx.append(self.cx[-1] + seg * math.cos(mid))
            self.cy.append(self.cy[-1] + seg * math.sin(mid))
        self.length = self.ds[-1]

    def heading(self, d: float) -> float:
        if d >= self.length:          # 끝 너머는 마지막 방향으로 곧게
            return self.th[-1]
        return lerp_list(self.ds, self.th, max(d, 0.0))

    def curvature(self, d0: float, d1: float) -> float:
        if d1 - d0 < 1e-6:
            return 0.0
        return (self.heading(d1) - self.heading(d0)) / (d1 - d0)

    def point(self, d: float) -> Tuple[float, float]:
        if d >= self.length:
            h = self.th[-1]
            r = d - self.length
            return self.cx[-1] + r * math.cos(h), self.cy[-1] + r * math.sin(h)
        d = max(d, 0.0)
        i = max(0, min(bisect.bisect_right(self.ds, d) - 1, len(self.ds) - 2))
        r = d - self.ds[i]
        mid = 0.5 * (self.th[i] + self.heading(d))
        return self.cx[i] + r * math.cos(mid), self.cy[i] + r * math.sin(mid)

    def path_point(self, d: float, n: float) -> Tuple[float, float]:
        x, y = self.point(d)
        h = self.heading(d)
        return x - n * math.sin(h), y + n * math.cos(h)


# =====================================================================
# 3. 경로 계획: 2차 격자 DP + 최소 곡률 평활화
# =====================================================================
@dataclass
class Plan:
    d: List[float]
    n: List[float]
    kappa: List[float]        # 경로 곡률 (station 0..K)
    blocked: List[bool]       # 그 지점에 빠져나갈 틈이 없음
    lo: List[float]
    hi: List[float]


class PathPlanner:
    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.prev_abs: List[Tuple[float, float]] = []   # (누적거리, n) 지난 계획
        M = cfg.lanes
        self.M = M

    def _blocks(self, fr: Frame, d_list: List[float], t_list: List[float]) -> List[List[Tuple[float, float]]]:
        """각 지점에서 막힌 옆 구간 목록."""
        c = self.cfg
        out: List[List[Tuple[float, float]]] = [[] for _ in d_list]
        half_len_obs = 0.5 * (c.car_length + c.obstacle_length) + c.long_buffer
        half_w_obs = 0.5 * (c.car_width + c.obstacle_width) + c.lat_buffer
        half_len_obs += 0.5 * c.plan_ds          # 지점 사이(6.25 m)로 빠져나가는 틈이 없게 반 칸 더 막음
        for dist, m in fr.obstacles:
            for k, d in enumerate(d_list):
                if abs(d - dist) < half_len_obs:
                    out[k].append((m - half_w_obs, m + half_w_obs))
        half_len_car = c.car_length + c.long_buffer
        half_w_car = c.car_width + c.lat_buffer
        for dist, m, vo in fr.opponents:
            if dist < -c.car_length:
                continue                       # 뒤차는 피하지 않음
            for k, d in enumerate(d_list):
                pos = dist + vo * t_list[k]    # 내가 d에 도착할 때 상대 위치 예측
                if abs(d - pos) < half_len_car:
                    out[k].append((m - half_w_car, m + half_w_car))
        return out

    def plan(self, fr: Frame, geo: RoadGeometry, travel: float, t_list: List[float]) -> Plan:
        c = self.cfg
        K, M, ds = c.plan_stations, self.M, c.plan_ds
        B = max(0.5, fr.half - 0.5 * c.car_width - c.edge_margin)
        lanes = [-B + 2 * B * j / (M - 1) for j in range(M)]
        d_list = [k * ds for k in range(K + 1)]
        kr = [geo.curvature(max(0.0, d - ds / 2), d + ds / 2) for d in d_list]
        blocks = self._blocks(fr, d_list, t_list)

        # 지난 계획 (현재 위치 기준으로 옮김)
        prev_n = []
        for d in d_list:
            if self.prev_abs:
                prev_n.append(lerp_list([p[0] for p in self.prev_abs], [p[1] for p in self.prev_abs], travel + d))
            else:
                prev_n.append(fr.n)

        # 지점별 비용
        node = [[0.0] * M for _ in range(K + 1)]
        for k in range(1, K + 1):
            for j, nj in enumerate(lanes):
                cost = c.w_cons * (nj - prev_n[k]) ** 2
                edge = abs(nj) - (B - 1.0)
                if edge > 0:
                    cost += c.w_edge * edge * edge
                for lo, hi in blocks[k]:
                    if lo <= nj <= hi:
                        cost += BIG
                    else:
                        gap = lo - nj if nj < lo else nj - hi
                        if gap < 1.5:
                            cost += c.w_near * (1.5 - gap) ** 2
                node[k][j] = cost

        # 2차 DP (곡률 = 도로 곡률 + n'')
        inv = c.w_curv / (ds ** 3)
        n0 = fr.n
        n_m1 = fr.n - ds * math.tan(clamp(fr.psi, -1.2, 1.2))
        slope = [[c.w_slope * (lanes[b] - lanes[a]) ** 2 / ds for b in range(M)] for a in range(M)]
        # station 1
        cost1 = []
        for b in range(M):
            kap = kr[0] * ds * ds + lanes[b] - 2 * n0 + n_m1
            cost1.append(node[1][b] + inv * kap * kap + c.w_slope * (lanes[b] - n0) ** 2 / ds)
        # station 2
        dp = [[0.0] * M for _ in range(M)]
        for a in range(M):
            for b in range(M):
                kap = kr[1] * ds * ds + lanes[b] - 2 * lanes[a] + n0
                dp[a][b] = cost1[a] + node[2][b] + inv * kap * kap + slope[a][b]
        bps: List[List[List[int]]] = []
        for k in range(3, K + 1):
            krk = kr[k - 1] * ds * ds
            new = [[0.0] * M for _ in range(M)]
            bp = [[0] * M for _ in range(M)]
            nk = node[k]
            for a in range(M):
                la2 = 2 * lanes[a]
                col = [dp[p][a] for p in range(M)]
                for b in range(M):
                    base = krk + lanes[b] - la2
                    best = BIG * 100
                    bi = 0
                    for p in range(M):
                        kap = base + lanes[p]
                        v = col[p] + inv * kap * kap
                        if v < best:
                            best = v
                            bi = p
                    new[a][b] = best + nk[b] + slope[a][b]
                    bp[a][b] = bi
            dp = new
            bps.append(bp)
        # 역추적
        best = BIG * 100
        ba = bb = 0
        for a in range(M):
            for b in range(M):
                if dp[a][b] < best:
                    best, ba, bb = dp[a][b], a, b
        idx = [0] * (K + 1)
        idx[K], idx[K - 1] = bb, ba
        for k in range(K, 2, -1):
            p = bps[k - 3][idx[k - 1]][idx[k]]
            idx[k - 2] = p
        n = [n0] + [lanes[idx[k]] for k in range(1, K + 1)]

        # 통로(막히지 않은 구간) 결정
        lo_l, hi_l, blocked = [], [], []
        for k in range(K + 1):
            lo, hi = -B, B
            is_blocked = False
            for blo, bhi in blocks[k]:
                if blo <= n[k] <= bhi:
                    is_blocked = True
                elif bhi < n[k]:
                    lo = max(lo, bhi)
                elif blo > n[k]:
                    hi = min(hi, blo)
            if k == 0:
                lo, hi = n0, n0
            lo_l.append(lo)
            hi_l.append(hi)
            blocked.append(is_blocked and k > 0)

        # 최소 곡률 평활화 (Σ(κ_road + n'')² 를 통로 안에서 최소화, 가우스-자이델)
        ext = [n_m1] + n            # ext[i] = n_{i-1}
        kk = [kr[k] * ds * ds for k in range(K + 1)]
        for _ in range(c.smooth_iters):
            for k in range(1, K + 1):
                if blocked[k]:
                    continue
                i = k + 1           # ext 인덱스
                terms_num = 0.0
                terms_den = 0.0
                # c_{k-1} = kk[k-1] + n_k - 2n_{k-1} + n_{k-2}
                a1 = kk[k - 1] - 2 * ext[i - 1] + ext[i - 2]
                terms_num += -a1
                terms_den += 1.0
                if k <= K - 1:
                    # c_k = kk[k] + n_{k+1} - 2n_k + n_{k-1}
                    a2 = kk[k] + ext[i + 1] + ext[i - 1]
                    terms_num += 2 * a2
                    terms_den += 4.0
                if k <= K - 2:
                    a3 = kk[k + 1] + ext[i + 2] - 2 * ext[i + 1]
                    terms_num += -a3
                    terms_den += 1.0
                x = terms_num / terms_den
                ext[i] = clamp(x, lo_l[k], hi_l[k])
        n = ext[1:]
        kappa = []
        for k in range(K + 1):
            if k < K:
                kappa.append(kr[k] + (ext[k + 2] - 2 * ext[k + 1] + ext[k]) / (ds * ds))
            else:
                kappa.append(kr[k])
        self.prev_abs = [(travel + d, nn) for d, nn in zip(d_list, n)]
        return Plan(d_list, n, kappa, blocked, lo_l, hi_l)


# =====================================================================
# 4. 속도 계획: 마찰원 + 전진/후진 패스
# =====================================================================
@dataclass
class Profile:
    d: List[float]
    v: List[float]
    t: List[float]
    lim: List[float] = field(default_factory=list)   # 지점별 옆 그립 한계 속도 (제동·가속 패스 전)


class SpeedPlanner:
    def __init__(self, cfg: Config):
        self.cfg = cfg

    def build(self, plan: Plan, geo: RoadGeometry, v0: float, mu: List[float],
              a_brake: float, decel: "DecelModel", a_drive: float, drag: float, vcap: float,
              ax_ratio: float = 1.0) -> Profile:
        """ax_ratio: 제동 방향 그립 / 옆 그립 (1 = 원 모양 마찰원, 1보다 크면 앞뒤로 긴 타원)."""
        c = self.cfg
        ds = c.plan_ds
        N = int(c.profile_length / ds) + 1
        d_list = [k * ds for k in range(N)]
        kap = []
        for k, d in enumerate(d_list):
            road = geo.curvature(max(0.0, d - ds / 2), d + ds / 2)
            if k < len(plan.kappa):
                # 계획 경로 곡률을 쓰되, 도로 곡률의 일정 비율 아래로는 믿지 않음 (재계획 때 곡률이 커지는 문제 방지)
                kap.append(max(abs(plan.kappa[k]), c.kappa_floor * abs(road)))
            else:
                kap.append(abs(road))
        kap = [max(kap[max(0, k - 1):k + 2]) for k in range(N)]   # 앞뒤 지점 중 큰 곡률 사용
        v = []
        for k in range(N):
            ak = abs(kap[k])
            vk = vcap if ak < 1e-5 else min(vcap, math.sqrt(mu[k] / ak))
            if k < len(plan.blocked) and plan.blocked[k]:
                vk = 0.0
            v.append(vk)
        v_lim = list(v)
        # 후진 패스: 가장 늦은 브레이킹 지점 (베르스타펜) + 코너 안 브레이크는 마찰원(타원)만큼만
        #  타원: 옆으로 lat 를 쓰고 있으면 제동은 (앞뒤 한계) × √(1 − (lat/옆 한계)²) 까지
        for k in range(N - 2, -1, -1):
            lat = v[k + 1] ** 2 * abs(kap[k])
            ry = lat / max(mu[k], 1e-3)
            # 실측 풀브레이크 감속도와, 옆 그립을 쓰고 남은 마찰원 + (게임이 인정한 만큼의) 저항 중 작은 값
            vb2 = v[k + 1] ** 2
            tire = mu[k] * ax_ratio * math.sqrt(1.0 - ry * ry) if ry < 1.0 else 0.0
            ab = min(a_brake, tire) + decel.B * vb2          # 브레이크 이득(부분 제동으로 실측)과 마찰원 중 작은 값
            if decel.n >= 20:                                  # 풀브레이크 실측이 쌓였으면 그 값도 상한으로
                ab = min(ab, 0.97 * (decel.A + decel.B * vb2))
            ab = max(ab, 0.15 * a_brake)
            lim = math.sqrt(v[k + 1] ** 2 + 2 * ab * ds)
            if lim < v[k]:
                v[k] = lim
        # 전진 패스: 현재 속도에서 낼 수 있는 가속
        v[0] = min(v[0], max(v0, 0.0))
        for k in range(N - 1):
            lat = v[k] ** 2 * abs(kap[k])
            room = mu[k] ** 2 - lat ** 2
            aa = a_drive - drag * v[k] ** 2
            aa = min(aa, math.sqrt(room)) if room > 0 else 0.1
            aa = max(aa, 0.1)
            lim = math.sqrt(v[k] ** 2 + 2 * aa * ds)
            if lim < v[k + 1]:
                v[k + 1] = lim
        t = [0.0]
        for k in range(N - 1):
            t.append(t[-1] + ds / max(0.5 * (v[k] + v[k + 1]), 1.0))
        return Profile(d_list, v, t, v_lim)


# =====================================================================
# 6. 추정기 (조향 이득 / 그립 / 브레이크·가속·저항)
# =====================================================================
class SteerGain:
    """조향 1당 곡률(1/m)을 속도 구간별 최소제곱으로 추정.
    - 조향 명령은 실제 반응 지연(조향 지연+측정 필터)에 맞춰 같은 지연을 준 값과 비교
      → 코너 진입(측정이 늦음)과 탈출(측정이 큼)의 오차가 서로 상쇄됨
    - 데이터가 적은 속도 구간은 '곡률형(g=상수)'과 '요레이트형(g·v=상수)' 중
      잔차가 작은 모델로 채움 (게임이 어떤 조향 방식인지 몰라도 스스로 판별)."""

    def __init__(self, init: float, lam: float = 0.998):
        self.bins = 16                    # 0~320 km/h, 20 km/h 간격
        self.lam = lam
        self.init = init
        self.sxx = [0.0] * self.bins
        self.sxy = [0.0] * self.bins
        self.cnt = [0] * self.bins
        # 전체 모델용 합 (곡률형 / 요레이트형)
        self.Sxx = 0.0
        self.Sxy_c = 0.0
        self.Syy_c = 0.0
        self.Sxy_y = 0.0
        self.Syy_y = 0.0
        self.samples = 0
        self.prior = 0.05                 # 사전값 무게 (조향² 단위)

    def _bin(self, v: float) -> int:
        return int(clamp(v * 3.6 / 20.0, 0, self.bins - 1))

    @property
    def c_curv(self) -> float:
        return (self.Sxy_c + self.prior * self.init) / (self.Sxx + self.prior)

    @property
    def c_yaw(self) -> float:
        return (self.Sxy_y + self.prior * self.init * 15.0) / (self.Sxx + self.prior)

    def _resid(self, sxy: float, syy: float) -> float:
        if syy <= 1e-12 or self.Sxx <= 1e-9:
            return 1.0
        return max(0.0, syy - sxy * sxy / self.Sxx) / syy

    @property
    def e_curv(self) -> float:
        return self._resid(self.Sxy_c, self.Syy_c)

    @property
    def e_yaw(self) -> float:
        return self._resid(self.Sxy_y, self.Syy_y)

    def model(self, v: float) -> float:
        gc, gy = self.c_curv, self.c_yaw / max(v, 3.0)
        if self.Sxx < 0.01:                     # 데이터가 아주 적으면 두 모델 판별 불가 → 곡률형(사전값 위주)
            return gc
        ec, ey = self.e_curv, self.e_yaw
        if ec < 0.7 * ey:
            return gc
        if ey < 0.7 * ec:
            return gy
        # 두 모델이 비슷하게 맞으면(속도 범위가 좁을 때) 더 '민감한' 쪽을 택함:
        #  이득을 크게 잡으면 덜 꺾는 쪽으로 틀리고(적분기가 보정), 작게 잡으면 과조향→마찰원 포화
        return max(gc, gy)

    def get(self, v: float) -> float:
        b = self._bin(v)
        m = self.model(v)
        # 구간 데이터 무게: 조향² 합 0.05 이상이면 구간 값 위주
        w = clamp(self.sxx[b] / 0.05, 0.0, 1.0)
        g = m
        if self.sxx[b] > 1e-6:
            gb = self.sxy[b] / self.sxx[b]
            gb = clamp(gb, 0.4 * m, 2.5 * m)
            g = w * gb + (1 - w) * m
        # 증거가 적을 때는 사전값의 30% 아래로 내려가지 않게 (작게 잘못 배우면 과조향 → 위험)
        if self.Sxx < 0.3:
            g = max(g, 0.3 * self.init)
        return g

    KEYS = ("sxx", "sxy", "cnt", "Sxx", "Sxy_c", "Syy_c", "Sxy_y", "Syy_y", "samples")

    def to_dict(self) -> Dict[str, Any]:
        return {k: getattr(self, k) for k in self.KEYS}

    def from_dict(self, d: Dict[str, Any]) -> None:
        for k in self.KEYS:
            if k in d:
                setattr(self, k, d[k])

    def update(self, v: float, steer: float, kappa: float) -> None:
        if abs(steer) < 0.02 or v < 4.0:
            return
        b = self._bin(v)
        l = self.lam
        self.sxx[b] = l * self.sxx[b] + steer * steer
        self.sxy[b] = l * self.sxy[b] + steer * kappa
        self.cnt[b] += 1
        self.Sxx = l * self.Sxx + steer * steer
        self.Sxy_c = l * self.Sxy_c + steer * kappa
        self.Syy_c = l * self.Syy_c + kappa * kappa
        kv = kappa * v
        self.Sxy_y = l * self.Sxy_y + steer * kv
        self.Syy_y = l * self.Syy_y + kv * kv
        self.samples += 1


class DriveModel:
    """가속도 = a_drive·스로틀 − drag·v² 를 최소제곱(지수 망각)으로 함께 추정."""

    def __init__(self, a0: float, d0: float, prior: float = 5.0, lam: float = 0.995):
        self.lam = lam
        # 특징: (스로틀, −v²/1000) → 계수: (a_drive, drag·1000)
        self.sxx = [[prior, 0.0], [0.0, prior]]
        self.sxy = [prior * a0, prior * d0 * 1000.0]
        self.a_drive, self.drag = a0, d0

    def update(self, thr: float, v2: float, acc: float) -> None:
        x1, x2 = thr, -v2 / 1000.0
        l = self.lam
        s = self.sxx
        s[0][0] = l * s[0][0] + x1 * x1
        s[0][1] = l * s[0][1] + x1 * x2
        s[1][0] = s[0][1]
        s[1][1] = l * s[1][1] + x2 * x2
        self.sxy[0] = l * self.sxy[0] + x1 * acc
        self.sxy[1] = l * self.sxy[1] + x2 * acc
        det = s[0][0] * s[1][1] - s[0][1] * s[1][0]
        if abs(det) < 1e-9:
            return
        a = (s[1][1] * self.sxy[0] - s[0][1] * self.sxy[1]) / det
        d = (s[0][0] * self.sxy[1] - s[1][0] * self.sxy[0]) / det / 1000.0
        if 1.0 < a < 30.0 and 0.0 < d < 0.01:
            self.a_drive, self.drag = a, d


class DecelModel:
    """풀브레이크 때 실제로 나오는 '총 감속도' = A + B·v² (최소제곱, 지수 망각).
    - 브레이크 힘이 한계면: A=브레이크 힘, B=공기저항 (고속일수록 더 잘 섬)
    - 타이어 그립이 한계면: A≈그립, B≈0 (속도와 무관)
    게임 물리가 어느 쪽이든 실측으로 맞춰짐. 사전값 B=0 은 '저항 덕을 안 본다'는 보수적 가정."""

    def __init__(self, a0: float, prior: float = 4.0, lam: float = 0.997):
        self.lam = lam
        self.sxx = [[prior, 0.0], [0.0, prior]]
        self.sxy = [prior * a0, 0.0]
        self.A, self.B = a0, 0.0
        self.n = 0

    def update(self, v2: float, dec: float) -> None:
        x1, x2 = 1.0, v2 / 1000.0
        l = self.lam
        s = self.sxx
        s[0][0] = l * s[0][0] + x1 * x1
        s[0][1] = l * s[0][1] + x1 * x2
        s[1][0] = s[0][1]
        s[1][1] = l * s[1][1] + x2 * x2
        self.sxy[0] = l * self.sxy[0] + x1 * dec
        self.sxy[1] = l * self.sxy[1] + x2 * dec
        det = s[0][0] * s[1][1] - s[0][1] * s[1][0]
        self.n += 1
        if abs(det) < 1e-9:
            return
        A = (s[1][1] * self.sxy[0] - s[0][1] * self.sxy[1]) / det
        B = (s[0][0] * self.sxy[1] - s[1][0] * self.sxy[0]) / det / 1000.0
        B = clamp(B, 0.0, 0.01)
        if B == 0.0 or B == 0.01:                 # 경계면 A만 다시 맞춤
            A = (self.sxy[0] - s[0][1] * B * 1000.0) / s[0][0]
        if 2.0 < A < 40.0:
            self.A, self.B = A, B

    def at(self, v: float) -> float:
        return self.A + self.B * v * v

    KEYS = ("sxx", "sxy", "A", "B", "n")

    def to_dict(self) -> Dict[str, Any]:
        return {k: getattr(self, k) for k in self.KEYS}

    def from_dict(self, d: Dict[str, Any]) -> None:
        for k in self.KEYS:
            if k in d:
                setattr(self, k, d[k])



class GripMap:
    """트랙 위치(10 m 구간)별 그립 배율 (해밀턴: 위험 코너 기억 / 문제없으면 조금씩 올림).
    랩 길이를 알면 구간 번호를 한 바퀴 단위로 접어서 다음 랩에도 그대로 씀."""

    def __init__(self, bin_len: float):
        self.bin_len = bin_len
        self.nb: Optional[int] = None
        self.mult: Dict[int, float] = {}
        self.danger: Dict[int, int] = {}
        self.used: Dict[int, float] = {}

    def set_lap(self, lap_len: float) -> None:
        nb = max(1, int(round(lap_len / self.bin_len)))
        self.nb = nb
        m: Dict[int, float] = {}
        for k, v in self.mult.items():
            m[k % nb] = min(m.get(k % nb, 9.0), v)
        self.mult = m
        dg: Dict[int, int] = {}
        for k, v in self.danger.items():
            dg[k % nb] = max(dg.get(k % nb, 0), v)
        self.danger = dg
        us: Dict[int, float] = {}
        for k, v in self.used.items():
            us[k % nb] = max(us.get(k % nb, 0.0), v)
        self.used = us

    def idx(self, s: float) -> int:
        i = int(math.floor(s / self.bin_len))
        return i % self.nb if self.nb else i

    def get(self, s: float) -> float:
        return self.mult.get(self.idx(s), 1.0)

    def _around(self, s: float, width: int) -> List[int]:
        b = self.idx(s)
        out = []
        for k in range(-width, width + 1):
            i = b + k
            if self.nb:
                i %= self.nb
            elif i < 0:
                continue
            out.append(i)
        return out

    def cut(self, s: float, ratio: float, width: int, danger_laps: int) -> None:
        for i in self._around(s, width):
            self.mult[i] = clamp(min(self.mult.get(i, 1.0), ratio), 0.5, 1.2)
            self.danger[i] = max(self.danger.get(i, 0), danger_laps)

    def scale_around(self, s: float, width: int, scale: float) -> None:
        for i in self._around(s, width):
            self.mult[i] = min(1.0, self.mult.get(i, 1.0) * scale)

    def note_used(self, s: float, frac: float) -> None:
        i = self.idx(s)
        self.used[i] = max(self.used.get(i, 0.0), frac)

    def lap_end(self) -> None:
        for i in set(self.danger) | set(self.used):
            if self.danger.get(i, 0) > 0:
                self.danger[i] -= 1
            elif self.used.get(i, 0.0) > 0.8:
                self.mult[i] = min(1.15, self.mult.get(i, 1.0) * 1.01)
        self.used = {}

    def to_dict(self) -> Dict[str, Any]:
        return {"bin_len": self.bin_len, "nb": self.nb,
                "mult": {str(k): round(v, 4) for k, v in self.mult.items() if abs(v - 1.0) > 1e-4}}

    def from_dict(self, d: Dict[str, Any]) -> None:
        if not d or abs(float(d.get("bin_len", self.bin_len)) - self.bin_len) > 1e-6:
            return
        self.mult = {int(k): float(v) for k, v in d.get("mult", {}).items()}
        if d.get("nb"):
            self.nb = int(d["nb"])


class FrameLogger:
    """프레임마다 한 줄(JSON)로 기록. 로컬 분석용. 실패해도 주행에는 영향 없음."""

    def __init__(self, path: str):
        self.f = None
        try:
            self.f = open(path, "a", encoding="utf-8")
        except Exception:
            self.f = None
        self.n = 0

    def write(self, rec: Dict[str, Any]) -> None:
        if self.f is None:
            return
        try:
            self.f.write(json.dumps(rec, ensure_ascii=False) + "\n")
            self.n += 1
            if self.n % 50 == 0:
                self.f.flush()
        except Exception:
            self.f = None

    def flush(self) -> None:
        if self.f is not None:
            try:
                self.f.flush()
            except Exception:
                pass


# =====================================================================
# 메인 두뇌
# =====================================================================
class RaceBrain:
    DRIVE, REVERSE, SETTLE, UTURN = "DRIVE", "REVERSE", "SETTLE", "UTURN"

    def __init__(self, cfg: Optional[Config] = None, half_limit: Optional[float] = None):
        self.cfg = cfg or Config()
        c = self.cfg
        self.half = half_limit if half_limit else c.half_road_width
        self.adapter = SsafyAdapter(c)
        self.planner = PathPlanner(c)
        self.speed_planner = SpeedPlanner(c)
        self.steer_gain = SteerGain(c.steer_gain_init)
        self.grip = GripMap(c.bin_len)
        self.a_lat = c.a_lat_init
        self.a_brake = c.a_brake_init            # 브레이크 1당 타이어 감속도 (부분 브레이크로 추정)
        self.decel = DecelModel(c.a_brake_init + 1.0)   # 풀브레이크 총 감속도 모델
        self.a_drive = c.a_drive_init
        self.drag = c.drag_init
        self.lap_len: Optional[float] = None
        self.ghost: Optional[Dict[int, float]] = None   # 최고 랩의 구간별 통과 시간
        self.best_lap: Optional[float] = None
        self.mem_auto_sign = 1.0
        self.memory_loaded = False
        self.load_memory()
        if self.lap_len:
            self.grip.set_lap(self.lap_len)

        # 상태
        self.t = 0.0
        self.t_move = 0.0
        self.started = False
        self.finished = False
        self.travel = 0.0
        self.lap = 0
        self.lap_t = 0.0
        self.prev: Optional[Frame] = None
        self.f_v = AlphaBeta(0.06)
        self.f_psi = AlphaBeta(0.05)
        self.f_n = AlphaBeta(0.05)
        self.drive_model = DriveModel(self.a_drive, self.drag)
        self.hist: List[Tuple[float, float, float, float]] = []   # (시간, 속도, 스로틀, 브레이크)
        self.prev_steer = 0.0
        self.prev_throttle = 0.0
        self.prev_brake = 0.0
        self.integ = 0.0
        self.kappa_meas = 0.0
        self.kappa_des = 0.0
        self.kdes_sent: List[Tuple[float, float, float]] = []   # 최근 보낸 목표 곡률 (시각, 곡률, 프레임 간격)
        self.smith_w = 0.0                                       # 조향 지연 보상을 섞는 비율 (0~1)
        self.psi_rate = 0.0
        self.slip_frames = 0
        self.grow_frames = 0
        self.global_slips = 0
        self.best_clean = 0.0
        self.slip_spots: List[Tuple[float, float, float]] = []
        self.steer_filt = 0.0
        self.steer_lagged = 0.0
        self.steer_win: List[float] = []
        self.sg_xx = self.sg_xy = self.sg_yy = 0.0
        self.sg_n = 0
        # 조향 → 측정 곡률 사이 지연은 게임·주기마다 다름 → 후보 여러 개를 같이 추적해 가장 잘 맞는 것을 씀
        self.lag_taus = [0.08, 0.16, 0.25, 0.35, 0.5]
        self.lag_sig = [0.0] * len(self.lag_taus)
        self.lag_sxx = [0.0] * len(self.lag_taus)
        self.lag_sxy = [0.0] * len(self.lag_taus)
        self.lag_syy = 0.0
        self.lag_best = int(clamp(getattr(self, "mem_lag_best", 1), 0, len(self.lag_taus) - 1))   # 메모리에서 이어받음
        self.a_cmd_f = 0.0
        self.acc_sent: List[Tuple[float, float, float]] = []   # 최근에 보낸 가감속 명령 (시각, 기대 가속도, 프레임 간격)
        self.v_pred = 0.0
        self.last_recover_t = -9.0
        # 브레이크 뒤 구동 공백 학습 (gap_dv 는 차 특성이라 메모리에서 이어받음)
        if not hasattr(self, "gap_dv"):
            self.gap_dv = 0.0                  # 놓은 뒤 계획보다 더 떨어지는 속도 (m/s)
        self.brk_v0: Optional[float] = None    # 이번 감속을 시작할 때 속도
        self.brk_last_t = -9.0                 # 마지막으로 브레이크를 밟은 시각
        self.gap_track: Optional[List[float]] = None   # [놓은 시각, 계획 최저 속도, 지금까지 실제 최저 속도]
        self.gap_events = 0
        # 제동 방향 그립 (타원 마찰원 앞뒤 한계, m/s²). 처음엔 옆 그립과 같게 → 지금까지와 같은 원 모양에서 시작
        if not getattr(self, "brake_cap", None):
            self.brake_cap = self.a_lat
        self.prev_sig: Optional[Tuple[Any, Any, Tuple[Any, ...]]] = None   # 직전 프레임 센서 (멈춤 감지용)
        self.stale_now = False
        self.stale_frames = 0
        self.wall_prev: Optional[float] = None
        self.wall_dt = c.dt
        self.auto_sign = self.mem_auto_sign
        self.sign_flips = 0
        self.road_steady = True
        self.last_grow = 0.0
        self.state = self.DRIVE
        self.state_t = 0.0
        self.stuck_t = 0.0
        self.wrong_t = 0.0
        self.fwd_t = 0.0
        self.uturn_dir = 1.0
        self.calm_t = 0.0
        self.factor = c.factor_plan_a
        self.plan_mode = "A"
        self.t_list: List[float] = []
        self.lap_bin_t: Dict[int, float] = {}
        self.lap_clean = True
        self.s_key = 0.0
        self.prev_s_key: Optional[float] = None
        self.s_hist: List[Tuple[float, float]] = []      # (출발 뒤 시간, 트랙 위치) 랩 경계 시각 계산용
        # 기록
        self.laps: List[Dict[str, Any]] = []
        self.crashes = 0
        self.uturns = 0
        self.slips = 0
        self.lap_crashes = 0
        self.corner_log: List[List[float]] = []
        self._corner: Optional[List[float]] = None
        self.debug: Dict[str, Any] = {}
        self.logger = FrameLogger(c.log_path) if c.log_path else None
        self.last_col_t = -9.0
        self.last_save = 0.0
        if not hasattr(self, "phantoms"):
            self.phantoms: List[Tuple[float, float]] = []

    # ---------------- 메모리 ----------------
    @staticmethod
    def _read_json(p: Optional[str]) -> Optional[Dict[str, Any]]:
        if not p or not os.path.exists(p):
            return None
        try:
            with open(p, encoding="utf-8") as f:
                m = json.load(f)
            return m if isinstance(m, dict) else None
        except Exception:
            return None

    def load_memory(self) -> None:
        """학습값 불러오기.
        - 같은 맵 파일(memory_path)이 있으면: 차 특성 + 그 맵 기록(랩 길이·고스트·구간 그립·숨은 장애물) 모두 사용
        - 없으면 공용 파일(memory_shared_path) → 코드 안 EMBEDDED_MEMORY 순서로 '차 특성만' 사용
          (다른 맵의 랩 길이·고스트를 쓰면 랩 계산과 전략이 틀어지므로 맵 기록은 버림)"""
        m = self._read_json(self.cfg.memory_path)
        same_map = m is not None
        if m is None:
            m = self._read_json(self.cfg.memory_shared_path)
        if m is None and EMBEDDED_MEMORY:
            m = EMBEDDED_MEMORY
        if not m:
            return
        self.memory_loaded = True
        self.a_lat = m.get("a_lat", self.a_lat)
        self.a_brake = m.get("a_brake", self.a_brake)
        if m.get("decel"):
            self.decel.from_dict(m["decel"])
        self.a_drive = m.get("a_drive", self.a_drive)
        self.drag = m.get("drag", self.drag)
        sg = m.get("steer")
        if sg and "Sxx" in sg:
            self.steer_gain.from_dict(sg)
        self.mem_auto_sign = m.get("auto_sign", 1.0)
        self.gap_dv = float(m.get("gap_dv", 0.0))
        if m.get("lag_best") is not None:
            self.mem_lag_best = int(m["lag_best"])          # 학습한 조향 지연 후보 번호 (차 특성)
        if m.get("brake_cap"):
            self.brake_cap = float(m["brake_cap"])
        if not same_map:
            return
        self.lap_len = m.get("lap_len")
        if m.get("ghost"):
            self.ghost = {int(k): float(v) for k, v in m["ghost"].items()}
        self.best_lap = m.get("best_lap")
        if m.get("grip"):
            self.grip.from_dict(m["grip"])
        self.phantoms = [(float(a), float(b)) for a, b in m.get("phantoms", [])]

    def memory_dict(self) -> Dict[str, Any]:
        return {
            "a_lat": round(self.a_lat, 4), "a_brake": round(self.a_brake, 4),
            "a_drive": round(self.a_drive, 4), "drag": round(self.drag, 6),
            "lap_len": self.lap_len, "best_lap": self.best_lap,
            "ghost": {str(k): round(v, 3) for k, v in self.ghost.items()} if self.ghost else None,
            "grip": self.grip.to_dict(),
            "steer": self.steer_gain.to_dict(), "decel": self.decel.to_dict(), "auto_sign": self.auto_sign,
            "phantoms": [[round(a, 1), round(b, 2)] for a, b in self.phantoms],
            "gap_dv": round(self.gap_dv, 3), "brake_cap": round(self.brake_cap, 3), "lag_best": self.lag_best,
        }

    def save_memory(self, path: Optional[str] = None) -> None:
        p = path or self.cfg.memory_path
        if not p:
            return
        d = self.memory_dict()
        targets = [p]
        if path is None and self.cfg.memory_shared_path and self.cfg.memory_shared_path != p:
            targets.append(self.cfg.memory_shared_path)
        for q in targets:
            try:
                with open(q, "w", encoding="utf-8") as f:
                    json.dump(d, f, ensure_ascii=False)
            except Exception as e:
                print("[MyCar] memory save failed:", e)

    # ---------------- 보조 ----------------
    def _s_ahead(self, d: float) -> float:
        s = self.s_key + d
        return s % self.lap_len if self.lap_len else s

    def _sdist(self, a: float, b: float) -> float:
        d = abs(a - b)
        if self.lap_len:
            d = min(d, self.lap_len - d)
        return d

    def _set_lap_len(self, lap_len: float) -> None:
        """랩 길이를 알아낸 순간: 지나간 1랩을 소급해서 마감하고, 이후는 한 바퀴 단위로 접음."""
        if self.lap_len is not None:
            return
        self.lap_len = lap_len
        self.grip.set_lap(lap_len)
        self.phantoms = [(a % lap_len, b) for a, b in self.phantoms]
        t_cross = None
        for i in range(1, len(self.s_hist)):
            (t0, s0), (t1, s1) = self.s_hist[i - 1], self.s_hist[i]
            if s0 < lap_len <= s1:
                t_cross = t0 + (t1 - t0) * (lap_len - s0) / max(s1 - s0, 1e-6)
                break
        if t_cross is None or self.lap > 0:
            return
        nb = int(round(lap_len / self.cfg.bin_len))
        cur = {b - nb: t - t_cross for b, t in self.lap_bin_t.items() if b >= nb}
        self.lap_bin_t = {b: t for b, t in self.lap_bin_t.items() if b < nb}
        now_lap_t = self.t_move - t_cross
        self._on_lap_end(lap_time=t_cross)
        self.lap_bin_t = cur
        self.lap_t = now_lap_t
        self.prev_s_key = None

    def _on_lap_end(self, lap_time: Optional[float] = None) -> None:
        c = self.cfg
        lap_time = self.lap_t if lap_time is None else lap_time
        rec = {"lap": self.lap + 1, "time": round(lap_time, 2), "crashes": self.lap_crashes,
               "clean": self.lap_clean, "a_lat": round(self.a_lat, 2), "a_brake": round(self.a_brake, 2),
               "decel_A": round(self.decel.A, 2), "decel_B": round(self.decel.B, 5)}
        self.laps.append(rec)
        if self.lap_len:
            expected = self.lap_len / c.bin_len
        else:
            expected = (max(self.lap_bin_t) + 1) if self.lap_bin_t else 1
        if self.lap_clean and len(self.lap_bin_t) >= 0.9 * expected:
            if self.best_lap is None or lap_time < self.best_lap:
                self.best_lap = lap_time
                self.ghost = dict(self.lap_bin_t)
        if c.practice_mode:
            self.grip.lap_end()
        if c.verbose:
            print("[MyCar] lap {} {:.2f}s crash {} a_lat {:.2f} a_brake {:.2f}".format(
                rec["lap"], lap_time, self.lap_crashes, self.a_lat, self.a_brake))
        self.lap += 1
        self.save_memory()
        self.lap_t = 0.0
        self.lap_bin_t = {}
        self.lap_clean = True
        self.lap_crashes = 0
        self.plan_mode = "A"

    # ---------------- 매 프레임 ----------------
    def step(self, raw: Any) -> Tuple[float, float, float]:
        c = self.cfg
        dt = c.dt
        now = time.monotonic()
        if c.dt_from_clock and self.wall_prev is not None:
            self.wall_dt += 0.2 * (clamp(now - self.wall_prev, 0.02, 0.5) - self.wall_dt)
            dt = self.wall_dt
        self.wall_prev = now
        fr = self.adapter.parse(raw, dt, self.prev_throttle, self.half)
        self.t += dt
        # 시뮬레이터가 멈춘 프레임: 움직이는 중인데 속도·중앙 거리·경유점 거리가 직전과 똑같음
        sig = (_get(raw, "speed", None), _get(raw, "to_middle", None),
               tuple((_get(raw, "distance_to_way_points", None) or [])[:3]))
        self.stale_now = self.prev_sig is not None and sig == self.prev_sig and abs(fr.v) > 1.0
        self.prev_sig = sig

        # --- 출발 신호 전: 가속 페달만 밟고 대기 (학습·복구·계획 모두 쉼) ---
        if not self.started:
            if abs(fr.v) > 1.5:
                self.started = True
            else:
                self.prev = fr
                self.prev_steer, self.prev_throttle, self.prev_brake = 0.0, 1.0, 0.0
                self._log(fr, 0.0, 1.0, 0.0)
                return self._emit(0.0, 1.0, 0.0)

        # --- 시뮬레이터가 멈춘 동안: 마지막 명령을 그대로 내고 학습·조향 적분·랩 시간은 멈춤 ---
        #  2026-11-25 Map 10: 1.1초씩 2번 멈춤. 그동안 '브레이크를 밟았는데 안 줄었다'로 학습해 a_brake 9.29→7.75,
        #  조향 적분이 쌓였다가 풀린 뒤 차가 흔들림. 학습 창(hist)도 비워서 멈춘 프레임이 섞이지 않게 함
        if self.stale_now:
            self.stale_frames += 1
            self.hist = []
            self._log(fr, self.prev_steer, self.prev_throttle, self.prev_brake)
            return self._emit(self.prev_steer, self.prev_throttle, self.prev_brake)

        self.t_move += dt
        self.lap_t += dt
        self.travel += fr.v * dt
        geo = RoadGeometry(fr.knot_d, fr.knot_th)
        if fr.teleport:
            self.f_psi.reset(fr.psi)
            self.f_n.reset(fr.n)
            self.integ = 0.0
            self.a_cmd_f = 0.0
            self.acc_sent = []
            self.kdes_sent = []
            self.gap_track = None
            self.brk_v0 = None
            self.planner.prev_abs = []
            self.hist = []

        # --- 트랙 위치·랩 ---
        if fr.lap_len_found and self.lap_len is None:
            self._set_lap_len(fr.lap_len_found)
        s_key = fr.s % self.lap_len if self.lap_len else fr.s
        if (self.lap_len and self.prev_s_key is not None and not fr.teleport
                and self.prev_s_key > self.lap_len - 80.0 and s_key < 80.0):
            self._on_lap_end()
        self.s_key = s_key
        self.prev_s_key = s_key
        if self.lap_len is None:
            self.s_hist.append((self.t_move, fr.s))
            if len(self.s_hist) > 4000:
                self.s_hist = self.s_hist[-4000:]
        gb = int(s_key // c.bin_len)
        if gb not in self.lap_bin_t:
            self.lap_bin_t[gb] = self.lap_t
        # 학습값은 20초마다, 랩이 끝날 때, 완주할 때 저장 (여러 바퀴 경기는 API 진행률이 100에 안 닿을 수 있음)
        if self.cfg.memory_path and self.t_move - self.last_save > 20.0:
            self.save_memory()
            self.last_save = self.t_move
        if fr.progress_api is not None and fr.progress_api >= 99.9 and not self.finished:
            self.finished = True
            if self.lap_t > 5.0:
                self._on_lap_end()
            self.save_memory()
            if self.logger:
                self.logger.flush()

        # 기억해 둔 '보이지 않는 장애물'을 장애물 목록에 추가
        for ps, pn in self.phantoms:
            d = ps - s_key
            if self.lap_len:
                d = (d + 0.5 * self.lap_len) % self.lap_len - 0.5 * self.lap_len
            if -5.0 < d < 200.0:
                fr.obstacles.append((d, pn))
        if fr.collided:
            self.last_col_t = self.t

        # --- 측정: 잡음을 걸러 낸 속도·방향·위치와 그 변화율 ---
        v_f, a_long = self.f_v.update(fr.v, dt)
        psi_f, psi_rate = self.f_psi.update(fr.psi, dt)
        n_f, _ = self.f_n.update(fr.n, dt)
        fr.v, fr.psi, fr.n = v_f, psi_f, n_f
        self.psi_rate = psi_rate
        kr0 = fr.kr0
        self.road_steady = abs(kr0 - fr.kr_next) < 0.15 * abs(kr0) + 0.0008
        if fr.v > 3.0:
            denom = 1.0 - fr.n * kr0
            denom = denom if abs(denom) > 0.3 else 0.3
            km = psi_rate / fr.v + kr0 * math.cos(fr.psi) / denom
            self.kappa_meas += (1.0 - math.exp(-dt / 0.05)) * (km - self.kappa_meas)
        else:
            a_long = 0.0
        a_lat_meas = fr.v * fr.v * abs(self.kappa_meas)
        a_tot = math.hypot(a_lat_meas, a_long)

        # --- 학습 ---
        self._learn(fr, a_long, a_lat_meas, a_tot)

        # --- 복구 상태 기계 ---
        out = self._recovery(fr, dt)
        if out is not None:
            self.acc_sent = []                       # 복구 중 명령은 지연 예측에 쓰지 않음
            self.kdes_sent = []
            self.last_recover_t = self.t
            self.gap_track = None
            self.brk_v0 = None
            self.prev = fr
            self.prev_steer, self.prev_throttle, self.prev_brake = out
            self._log(fr, *out)
            return self._emit(*out)

        # --- 전략: 속도 배율 ---
        self._strategy(fr, dt)

        # --- 계획 ---
        if len(self.t_list) != c.plan_stations + 1:
            self.t_list = [k * c.plan_ds / max(fr.v, 5.0) for k in range(c.plan_stations + 1)]
        plan = self.planner.plan(fr, geo, self.travel, self.t_list)
        N = int(c.profile_length / c.plan_ds) + 1
        mu = [self.a_lat * self.grip.get(self._s_ahead(k * c.plan_ds)) * self.factor for k in range(N)]
        vcap = c.max_speed_kmh / 3.6
        # 제동 방향 그립 (타원 마찰원): 실측 제동+옆 가속 합이 약 13~14 로 옆 그립(약 11)보다 큼 (2026-11-25 Map 10).
        #  계획은 실제로 낸 최대 제동(brake_cap)보다 3%만 더 씀 → 낼 수 있으면 다음에 조금 더, 포화되면 그 값에 멈춤
        ax_ratio = 1.0
        if c.brake_ellipse:
            ax_plan = min(self.brake_cap * c.brake_cap_margin, self.a_brake)
            ax_ratio = clamp(ax_plan / max(self.a_lat, 1.0), 1.0, 1.5)
        prof = self.speed_planner.build(plan, geo, fr.v, mu, self.a_brake, self.decel, self.a_drive, self.drag, vcap,
                                        ax_ratio)
        self.t_list = prof.t[: c.plan_stations + 1]
        self.last_plan, self.last_prof, self.last_geo, self.last_mu = plan, prof, geo, mu

        # --- 조향: 순수 추종 + 곡률 서보 ---
        v = max(fr.v, 0.0)
        # 지연 보상: moving_angle 은 반 프레임 늦은 값이고, 이번 명령은 다음 0.1초 동안 적용됨
        #  → 명령이 실제로 듣는 시점(약 τ초 뒤)의 내 위치·방향을 예측해서 그 자리에서 추종
        # 직선 흔들림 방지 (조향 스미스 예측기): 조향 명령이 실제 곡률로 나타나기까지 약 0.44초 (2026-11-25 Map 10).
        #  예전에는 지금 회전 속도로 0.15초만 내다봐서, 이미 보냈지만 아직 안 돈 조향을 모르고 계속 더 꺾다가 넘어감
        #  → 직선에서 약 1.3초 주기로 좌우 흔들림(조향 ±0.15~0.29), 빠를수록 커짐.
        #  이제 최근 steer_smith 초 동안 보낸 목표 곡률만큼 '앞으로 더 돌 것'으로 보고 그 자세에서 목표점을 잡음.
        #  (측정 곡률을 쓰면 잡음이 커지고, 예측 시간만 늘리면 더 흔들려서 이 방식을 고름)
        #  장애물·다른 차가 가까우면 끔: 조향이 빠른 차에서는 덜 꺾게 되어 회피가 늦어질 수 있음
        #  비정상 자세(역주행, 46° 넘게 틀어짐, 복구 직후, 도로 끝 근처)에서도 끔
        normal_pose = (fr.moving_forward and abs(fr.psi) < 0.8 and self.t - self.last_recover_t > 1.5
                       and abs(fr.n) < fr.half - 1.0)            # 도로 끝 근처·바깥이면 예전처럼 빨리 돌아옴
        clear_road = (not any(0.0 < d < 80.0 and abs(m - fr.n) < 6.0 for d, m in fr.obstacles)
                      and not any(-10.0 < d < 60.0 for d, _, _ in fr.opponents))
        # 창 길이 = 학습한 조향 지연(측정 지연 포함) − 0.2초, 최대 steer_smith.
        #  조향이 빠른 차는 학습 지연이 짧아 창이 거의 0 → 예전과 같음 (빠른 차에 쓰면 코너에서 덜 꺾어 느려짐: 가짜 차 시험)
        #  실제 차는 0.5초(후보 최대)로 학습됨 → 창 0.3초
        win = clamp(self.lag_taus[self.lag_best] - 0.2, 0.0, c.steer_smith)
        # 예전 방식 예측 (0.15초, 지금 회전 속도 유지)
        tau = 0.5 * dt + 0.5 * dt + 0.05
        psi_p = fr.psi + psi_rate * tau
        d0 = v * tau * math.cos(fr.psi)
        n_p = fr.n + v * tau * math.sin(fr.psi + 0.5 * psi_rate * tau)
        # 켜고 끌 때 자세 예측이 갑자기 바뀌지 않게 섞는 비율을 천천히 바꿈 (켤 때 0.5초, 끌 때 0.3초)
        w_tgt = 1.0 if (win >= 0.95 * dt and normal_pose and clear_road) else 0.0   # 창에 명령이 한 프레임 이상 들어갈 때만
        rate = dt / 0.5 if w_tgt > self.smith_w else dt / 0.3
        self.smith_w = clamp(self.smith_w + clamp(w_tgt - self.smith_w, -rate, rate), 0.0, 1.0)
        if self.smith_w > 0.0:
            t2 = 0.5 * dt                                      # moving_angle 은 반 프레임 늦은 값
            psi_s = fr.psi + psi_rate * t2
            d_s = v * t2 * math.cos(fr.psi)
            n_s = fr.n + v * t2 * math.sin(fr.psi + 0.5 * psi_rate * t2)
            for (ts, kd, hh) in self.kdes_sent:
                if ts > self.t - win - 1e-6:
                    kr = geo.curvature(d_s, d_s + max(v * hh, 0.1))
                    psi_s += v * (kd - kr) * hh
                    n_s += v * hh * math.sin(psi_s)
                    d_s += v * hh * math.cos(psi_s)
            w = self.smith_w
            psi_p = (1.0 - w) * psi_p + w * psi_s
            n_p = (1.0 - w) * n_p + w * n_s
            d0 = (1.0 - w) * d0 + w * d_s
        cx0, cy0 = geo.path_point(d0, n_p)
        head = geo.heading(d0) + psi_p
        Ld = clamp(0.32 * v + 5.0, 6.0, 32.0)
        n_at = lerp_list(plan.d, plan.n, d0 + Ld)
        px, py = geo.path_point(d0 + Ld, n_at)
        dx, dy = px - cx0, py - cy0
        cp, sp = math.cos(head), math.sin(head)
        xc = dx * cp + dy * sp
        yc = -dx * sp + dy * cp
        k_pp = 2.0 * yc / max(xc * xc + yc * yc, 1.0)
        k_ff = lerp_list(plan.d, plan.kappa, min(v * 0.15, plan.d[-1]))
        self.kappa_des = 0.7 * k_pp + 0.3 * k_ff
        # 그립 한계 이상으로 조향하지 않음: 앞바퀴가 미끄러지면 브레이크까지 안 듣게 되므로
        k_lim = 1.08 * self.a_lat * self.grip.get(self.s_key) / max(v * v, 1.0)
        grip_capped = abs(self.kappa_des) > k_lim
        self.kappa_des = clamp(self.kappa_des, -k_lim, k_lim)
        self.kdes_sent.append((self.t, self.kappa_des, dt))   # 조향 지연 예측용 기록
        if len(self.kdes_sent) > 8:
            self.kdes_sent.pop(0)
        g = self.steer_gain.get(v)
        steer_ff = self.kappa_des / max(g, 1e-3)
        steer_ff += self._sysid_weave(fr)
        sat = abs(steer_ff + self.integ) >= 1.0 or grip_capped
        if v > 4.0 and not sat:
            self.integ += 1.2 * (self.kappa_des - self.kappa_meas) / max(g, 1e-3) * dt
        self.integ = clamp(self.integ * (1 - 0.3 * dt), -0.5, 0.5)
        steer = clamp(steer_ff + self.integ, -1.0, 1.0)

        # --- 가감속: 피드포워드 + P ---
        # 베르스타펜 다듬기 (브레이크 지연 보상): 스로틀·브레이크는 약 0.25초 늦게 듣는다 (실측).
        #  보상이 없으면 브레이크를 놓은 뒤에도 0.25초 동안 계속 감속해 코너마다 8~18 km/h 더 느려진다.
        #  → 이미 보냈지만 아직 듣지 않은 명령(최근 speed_lag 초)의 효과를 더해 '명령이 듣는 시점의 속도·위치'를
        #    예측하고, 그 상태로 가감속을 계산한다 (조향의 지연 예측과 같은 생각, 제어 이론의 스미스 예측기).
        tau_v = c.speed_lag
        # 자세가 비정상(역주행, 진행 방향이 46° 넘게 틀어짐, 복구·U턴 직후)이면 지연 보상 없이 예전 방식 그대로
        #  (벽 쪽을 향해 있을 때 브레이크를 일찍 놓으면 위험. 정상 코너링도 30° 가까이 틀어지므로 기준은 넉넉히)
        if not fr.moving_forward or abs(fr.psi) > 0.8 or self.t - self.last_recover_t < 1.5:
            tau_v = 0.0
        inc = sum(a * h for (ts, a, h) in self.acc_sent if ts > self.t - tau_v - 1e-6) if tau_v > 0 else 0.0
        # 장애물·앞차 앞에서 멈추거나 비켜야 할 때는 '브레이크를 덜 밟는 쪽'으로는 예측을 쓰지 않음
        #  (실제 지연이 생각보다 짧으면 덜 밟아서 부딪히게 됨. 일찍 밟는 쪽(inc > 0)은 그대로 씀)
        near_block = (any(0.0 < d < 60.0 and abs(m - fr.n) < 4.0 for d, m in fr.obstacles)
                      or any(0.0 < d < 40.0 for d, _, _ in fr.opponents)
                      or any(b for b, dd in zip(plan.blocked, plan.d) if dd < 80.0))
        if near_block:
            inc = max(inc, 0.0)
        v_c = max(v + inc, 0.0)
        self.v_pred = v_c
        d_c = max(v + 0.5 * inc, 0.0) * tau_v
        d_ref = max(v_c * 0.3, 2.0)
        d_t = min(d_c + d_ref, prof.d[-1])
        v_ref = lerp_list(prof.d, prof.v, d_t)
        # 베르스타펜 학습 (브레이크 뒤 구동 공백): 크게 감속한 뒤 브레이크를 놓으면 약 1초 동안 풀스로틀이어도
        #  구동력이 안 나온다 (2026-11-25 Map 10: 직선에서도 놓은 뒤 0~0.5초 −4.3, 0.5~1초 −1.9 m/s², 자동 변속기로 보임)
        #  → 코너마다 계획보다 평균 12 km/h 더 느려짐. 크게 감속하는 중(감속 시작보다 10 km/h 넘게 느려짐)과
        #    놓은 뒤 1.2초 동안은 목표를 '앞 1.2초 동안 계획한 최저 속도 + 학습값 gap_dv' 로 삼아
        #    (원래 목표보다 높을 때만) 제동 끝 무렵에 그만큼 일찍 놓는다. 제동 구간 한가운데에서는
        #    1.2초 뒤 계획 속도가 훨씬 낮아 올라가지 않음. 그 지점의 옆 그립 한계 속도는 넘지 않음 (코너 정점에서는 안 올림)
        depth = (self.brk_v0 - v) if self.brk_v0 is not None else 0.0
        braking_seq = self.prev_brake > 0.05 or self.t - self.brk_last_t < 1.2
        w_gap = clamp((depth - 2.8) / 2.8, 0.0, 1.0) if (braking_seq and tau_v > 0 and not near_block) else 0.0
        v_ref_gap = v_ref
        gap_capped = False
        if c.gap_comp and w_gap > 0.0 and self.gap_dv > 0.0:
            hz = v * 1.2
            v_min_ahead = min(lerp_list(prof.d, prof.v, min(d_t + hz * q, prof.d[-1])) for q in (0.0, 0.25, 0.5, 0.75, 1.0))
            v_lim_t = lerp_list(prof.d, prof.lim, d_t) if prof.lim else v_ref
            want = v_min_ahead + w_gap * self.gap_dv
            gap_capped = want > v_lim_t                  # 그립 한계에 막혀 다 못 올림
            v_ref_gap = max(v_ref, min(want, v_lim_t))
        a_raw = (v_ref_gap * v_ref_gap - v_c * v_c) / (2 * d_ref) + 1.0 * (v_ref_gap - v_c)
        self.a_cmd_f += c.a_cmd_smooth * (a_raw - self.a_cmd_f)
        if a_raw < self.a_cmd_f - 2.0 or (a_raw < 0.0 < self.a_cmd_f):     # 새 제동 요구는 바로 반영
            self.a_cmd_f = a_raw
        a_cmd = self.a_cmd_f
        drag_a = self.drag * v * v
        if a_cmd >= -drag_a:
            throttle = clamp((a_cmd + drag_a) / max(self.a_drive, 1.0), 0.0, 1.0)
            brake = 0.0
        else:
            throttle = 0.0
            brake = clamp((-a_cmd - drag_a) / max(self.a_brake, 1.0), 0.0, 1.0)
            if c.reverse_brake and brake >= 0.95 and v > 8.0:
                throttle = -1.0                          # 후진 기어 제동 (선택)
        if fr.v < -0.3:                                  # 아직 뒤로 가고 있으면 멈추고 출발
            throttle, brake = 0.0, 1.0
        # 구동 공백 학습: 크게 감속한 뒤 놓은 순간 '앞 1.2초 동안 계획한 최저 속도'를 적어 두고,
        #  1.2초 동안 실제 최저 속도와 비교해 gap_dv 를 고침 (공백이 없는 차·맵이면 0 으로 줄어듦)
        if brake > 0.05:
            if self.brk_v0 is None or self.t - self.brk_last_t > 1.5:
                self.brk_v0 = v                            # 새 감속 시작
            self.brk_last_t = self.t
            if self.gap_track is not None:                 # 놓은 뒤 다시 밟음 → 지금까지로 마감
                self._gap_finish()
        else:
            if (self.prev_brake > 0.05 and depth > 5.6 and tau_v > 0 and not near_block
                    and self.gap_track is None):
                hz = v * 1.2
                tgt = min(lerp_list(prof.d, prof.v, min(d_c + hz * q, prof.d[-1])) for q in (0.0, 0.25, 0.5, 0.75, 1.0))
                self.gap_track = [self.t, tgt, v, 1.0 if gap_capped else 0.0]
            if self.gap_track is not None:
                self.gap_track[2] = min(self.gap_track[2], v)
                if self.t - self.gap_track[0] > 1.2:
                    self._gap_finish()
            if self.t - self.brk_last_t > 1.5:
                self.brk_v0 = None
        # 이번 명령이 낼 것으로 기대하는 가속도를 기록 (다음 프레임들의 지연 예측에 씀)
        self.acc_sent.append((self.t, throttle * self.a_drive - brake * self.a_brake - drag_a, dt))
        if len(self.acc_sent) > 10:
            self.acc_sent.pop(0)

        # --- 기록 ---
        self._telemetry(fr, geo)
        self.grip.note_used(self.s_key, a_tot / max(self.a_lat * self.grip.get(self.s_key), 1.0))
        self.debug = {"mode": self.plan_mode, "factor": round(self.factor, 3), "v_ref": round(v_ref * 3.6, 1),
                      "k_des": self.kappa_des, "k_meas": self.kappa_meas, "n_target": n_at,
                      "a_lat_est": self.a_lat, "gain": g}
        self.prev = fr
        self.prev_steer, self.prev_throttle, self.prev_brake = steer, throttle, brake
        self._log(fr, steer, throttle, brake)
        return self._emit(steer, throttle, brake)

    def _gap_finish(self) -> None:
        """구동 공백 학습 한 번 마감: 계획한 최저 속도보다 실제로 더 떨어진 만큼 gap_dv 를 올리고, 덜 떨어졌으면 내림."""
        t0, tgt, vmin, capped = self.gap_track
        self.gap_track = None
        if self.t - t0 < 0.4:                 # 놓자마자 다시 밟은 경우는 판단하기 이름
            return
        err = tgt - vmin
        if err > 0 and capped > 0.5:          # 그립 한계에 막혀 다 못 올린 코너: 올려도 못 쓰므로 키우지 않음 (와인드업 방지)
            return
        self.gap_dv = clamp(self.gap_dv + 0.4 * err, 0.0, self.cfg.gap_max)
        self.gap_events += 1

    def _log(self, fr: Frame, steer: float, throttle: float, brake: float) -> None:
        if not self.logger:
            return
        self.logger.write({
            "t": round(self.t, 2), "s": round(fr.s, 1), "v": round(fr.v * 3.6, 1), "n": round(fr.n, 2),
            "n_raw": round(fr.n_raw, 2), "psi": round(fr.psi / DEG, 2), "kr0": round(fr.kr0, 5),
            "prog": fr.progress_api, "fwd": fr.moving_forward, "col": fr.collided,
            "obs": [[round(d, 1), round(m, 2)] for d, m in fr.obstacles[:3]],
            "opp": [[round(d, 1), round(m, 2), round(sp * 3.6, 1)] for d, m, sp in fr.opponents[:2]],
            "st": round(steer, 3), "th": round(throttle, 3), "br": round(brake, 3),
            "state": self.state, "mode": self.plan_mode, "fac": round(self.factor, 3),
            "vref": self.debug.get("v_ref"), "vpred": round(self.v_pred * 3.6, 1), "gapdv": round(self.gap_dv * 3.6, 1),
            "bcap": round(self.brake_cap, 2), "slag": self.lag_taus[self.lag_best], "smw": round(self.smith_w, 2),
            "kdes": round(self.kappa_des, 5), "kmeas": round(self.kappa_meas, 5),
            "alat": round(self.a_lat, 2), "abrk": round(self.a_brake, 2), "gain": round(self.steer_gain.get(max(fr.v, 0.0)), 4),
            "tp": fr.teleport, "stale": self.stale_now,
            "plan": [round(x, 2) for x in getattr(self, "last_plan", None).n[:12:2]] if getattr(self, "last_plan", None) else None,
            "lohi": [[round(a, 1), round(b, 1)] for a, b in zip(self.last_plan.lo[:12:2], self.last_plan.hi[:12:2])] if getattr(self, "last_plan", None) else None})

    def _sysid_weave(self, fr: Frame) -> float:
        """알베르토식 '첫 랩 전에 차를 파악': 출발 직후 아주 작은 조향 사인파(옆 이동 ~0.1 m).
        속도가 바뀌는 구간이라 '곡률형/요레이트형' 조향 모델을 첫 장애물 전에 판별할 수 있음.
        주변에 차·장애물이 있으면 하지 않음."""
        c = self.cfg
        if not c.sysid_weave or self.steer_gain.Sxx > 0.05:
            return 0.0
        t0, t1 = 0.6, 0.6 + 2.0 / c.sysid_freq
        tt = self.t_move - t0
        if tt < 0 or self.t_move > t1 or fr.v < 2.0:
            return 0.0
        for d, n, _ in fr.opponents:
            if abs(d) < 25.0 and abs(n - fr.n) < 5.0:
                return 0.0
        for d, n in fr.obstacles:
            if 0 < d < 50.0:
                return 0.0
        return c.sysid_amp * math.sin(2 * math.pi * c.sysid_freq * tt)

    def _emit(self, steer: float, throttle: float, brake: float) -> Tuple[float, float, float]:
        return clamp(steer * self.auto_sign, -1, 1), clamp(throttle, -1, 1), clamp(brake, 0, 1)

    # ---------------- 학습 ----------------
    def _learn(self, fr: Frame, a_long: float, a_lat_meas: float, a_tot: float) -> None:
        if self.prev is None or self.state != self.DRIVE or fr.teleport:
            self.slip_frames = 0
            return
        v = fr.v
        straight = a_lat_meas < 0.3 * self.a_lat
        # 조향 이득: 타이어에 여유가 있고 도로 곡률이 일정할 때, '반응 지연'을 맞춘 조향과 측정 곡률 비교
        dt = self.wall_dt if self.cfg.dt_from_clock else self.cfg.dt
        self.steer_filt += (self.prev_steer - self.steer_filt) * min(1.0, dt / 0.1)
        for j, tau in enumerate(self.lag_taus):
            self.lag_sig[j] += (self.prev_steer - self.lag_sig[j]) * (1.0 - math.exp(-dt / tau))
        self.steer_lagged = self.lag_sig[self.lag_best]
        # 출발 직후 조향 부호 빠른 점검: 부호가 반대면 차가 바로 돌아 버리므로 정밀 학습 조건을 기다리지 않음
        if (self.t_move < 15.0 and v > 3.0 and fr.moving_forward and self.t - self.last_col_t > 1.0
                and self.sign_flips == 0):
            x = self.lag_sig[1]
            y = self.kappa_meas
            self.sg_xx += x * x
            self.sg_xy += x * y
            self.sg_yy += y * y
            self.sg_n += 1 if abs(x) > 0.02 else 0
            r = self.sg_xy / math.sqrt(max(self.sg_xx * self.sg_yy, 1e-12))
            if self.sg_n >= 6 and self.sg_xx > 0.003 and r < -0.5:
                self.auto_sign = -self.auto_sign
                self.steer_gain = SteerGain(self.cfg.steer_gain_init)
                self.integ = 0.0
                self.lag_sig = [0.0] * len(self.lag_taus)
                self.sg_xx = self.sg_xy = self.sg_yy = 0.0
                self.sg_n = 0
                self.sign_flips += 1
        # 요구 하중은 이득을 작게 잘못 배웠어도 과소평가되지 않도록 사전값 이상으로 계산
        #  (과소평가 → 마찰원에 걸린 샘플이 섞임 → 이득을 더 작게 배우는 악순환 방지)
        g_gate = max(self.steer_gain.get(v), self.cfg.steer_gain_init)
        demand = abs(g_gate * self.steer_lagged) * v * v
        # 조향이 0.4초 동안 크게 변하지 않았을 때만 (빠른 회피 동작 중에는 지연 오차가 커서 이득을 작게 배움)
        self.steer_win.append(self.prev_steer)
        if len(self.steer_win) > max(3, int(round(0.4 / dt))):
            self.steer_win.pop(0)
        sw_mean = sum(self.steer_win) / len(self.steer_win)
        steady = max(self.steer_win) - min(self.steer_win) < 0.03 + 0.25 * abs(sw_mean)
        if (a_tot < 0.65 * self.a_lat and math.hypot(demand, a_long) < 0.95 * self.a_lat
                and abs(self.steer_lagged) < 0.95 and self.road_steady and steady
                and v > 5.0 and self.t - self.last_col_t > 1.5 and fr.moving_forward):
            self.steer_gain.update(v, self.steer_lagged, self.kappa_meas)
            # 지연 후보별 상관 (곡률형 기준) → 가장 잘 맞는 지연 선택
            lam = 0.995
            y = self.kappa_meas
            self.lag_syy = lam * self.lag_syy + y * y
            best_r, best_j = -9.0, self.lag_best
            rs = []
            for j, x in enumerate(self.lag_sig):
                self.lag_sxx[j] = lam * self.lag_sxx[j] + x * x
                self.lag_sxy[j] = lam * self.lag_sxy[j] + x * y
                r = self.lag_sxy[j] / math.sqrt(max(self.lag_sxx[j] * self.lag_syy, 1e-12))
                rs.append(r)
                if r > best_r:
                    best_r, best_j = r, j
            if self.lag_sxx[best_j] > 0.01 and best_j != self.lag_best:
                self.lag_best = best_j
            # 조향 부호 자동 판별: 어떤 지연으로 봐도 꺾은 방향과 반대로 돌면 부호를 뒤집고 다시 배움
            #  (출발 직후 40초 안에서만, 충분한 데이터가 쌓였을 때만)
            if (self.t_move < 40.0 and min(self.lag_sxx) > 0.02 and max(rs) < -0.4):
                self.auto_sign = -self.auto_sign
                self.steer_gain = SteerGain(self.cfg.steer_gain_init)
                self.integ = 0.0
                self.lag_sig = [0.0] * len(self.lag_taus)
                self.lag_sxx = [0.0] * len(self.lag_taus)
                self.lag_sxy = [0.0] * len(self.lag_taus)
                self.lag_syy = 0.0
                self.sign_flips += 1
        # 브레이크·가속·저항: 같은 명령이 이어진 구간의 평균 가속도로만 학습
        self.hist.append((self.t, v, self.prev_throttle, self.prev_brake))
        if len(self.hist) > 7:
            self.hist.pop(0)
        if len(self.hist) == 7 and straight:
            h = self.hist[1:]
            t0, v0 = h[0][0], h[0][1]
            t1, v1 = h[-1][0], h[-1][1]
            span = t1 - t0
            vm = 0.5 * (v0 + v1)
            acc = (v1 - v0) / span if span > 0 else 0.0
            ths = [x[2] for x in h]
            brs = [x[3] for x in h]
            # 베르스타펜: 실측 감속도에서 공기저항 몫을 빼고 '브레이크 1당 감속도'만 추정 (2026-11-25 실측으로 고침)
            #  - 브레이크는 약 speed_lag 초 늦게 들으므로 최근 3구간의 감속은 lag 프레임 앞선 명령과 짝지음
            #    (예전: 같은 시각 명령과 비교 → 밟기 시작한 직후 '안 줄었다'로 낮게 학습)
            #  - 예전 조건 '감속 < 0.85×옆 그립'은 실제 제동(1당 13~15)을 모두 걸러 한 번도 학습되지 않았음 → 뺌
            #  - 풀브레이크로 실제 나온 감속은 '브레이크 1당 효과'의 하한 (1로 12.5가 나왔으면 1당 12.5 이상)
            n_h = len(self.hist)
            lag_n = int(clamp(round(self.cfg.speed_lag / max(dt, 0.02)), 0, 3))
            hv = self.hist[-4:]                                    # 속도 3구간
            cmd = [x[3] for x in self.hist[n_h - 3 - lag_n:n_h - lag_n]]
            ths_b = [x[2] for x in self.hist[n_h - 4 - lag_n:]]
            span_b = hv[-1][0] - hv[0][0]
            if span_b > 0 and max(ths_b) <= 0 and min(cmd) > 0.05:
                vmb = 0.5 * (hv[0][1] + hv[-1][1])
                acc_b = (hv[-1][1] - hv[0][1]) / span_b
                tire = -acc_b - self.drag * vmb * vmb
                br_m = sum(cmd) / len(cmd)
                # 제동 방향 그립 학습 (타원 마찰원): 실제로 낸 감속을 '순수 제동'으로 환산
                #  포화(풀브레이크, 또는 명령한 만큼 감속이 안 나옴 = 그립 한계)면 그 값이 한계,
                #  여유가 있었으면 '적어도 이만큼은 됨' (한 번에 5% 넘게는 안 올림)
                if vmb > 8 and 0.0 < tire < 30.0 and self.t - self.last_col_t > 1.5:
                    a_y = max(self.a_lat * self.grip.get(self.s_key), 1.0)
                    r = min(a_lat_meas / a_y, 0.9)
                    eq = tire / math.sqrt(1.0 - r * r)
                    if min(cmd) > 0.95 or tire < 0.85 * br_m * self.a_brake:
                        self.brake_cap += 0.3 * (eq - self.brake_cap)
                    elif br_m > 0.3:
                        self.brake_cap = max(self.brake_cap, min(eq, self.brake_cap * 1.05))
                if vmb > 8 and min(cmd) > 0.95:
                    self.decel.update(vmb * vmb, -acc_b)          # 풀브레이크 실측 총감속도
                    if tire > self.a_brake:
                        self.a_brake += 0.3 * (min(tire, 30.0) - self.a_brake)
                elif vmb > 8 and 0.2 < br_m < 0.9:
                    meas = tire / br_m
                    if 2.0 < meas < 30.0:
                        w = 0.3 * min(1.0, br_m)
                        self.a_brake += w * (meas - self.a_brake)
            if max(brs) == 0 and min(ths) >= 0 and max(ths) - min(ths) < 0.1 and vm > 3:
                self.drive_model.update(sum(ths) / len(ths), vm * vm, acc)
                self.a_drive, self.drag = self.drive_model.a_drive, self.drive_model.drag
        # 그립: 조향으로 기대한 곡률보다 실제 곡률이 모자라고, 옆 그립을 많이 쓰는 중이면 미끄러짐
        k_pred = self.steer_gain.get(v) * self.steer_filt
        local = self.grip.get(self.s_key)
        load = self.a_lat * self.factor * local
        short_pred = k_pred * self.kappa_meas > 0 and abs(k_pred) > 1.2 * abs(self.kappa_meas) + 0.002
        short_des = self.kappa_des * self.kappa_meas > 0 and abs(self.kappa_des) > 1.12 * abs(self.kappa_meas) + 0.001
        slipping = v > 8 and a_lat_meas > 0.75 * load and (short_pred or short_des)
        tracking = abs(self.kappa_meas) >= 0.95 * abs(self.kappa_des)
        self.slip_frames = self.slip_frames + 1 if slipping else 0
        clean_hi = not slipping and tracking and a_lat_meas > 0.9 * load
        self.grow_frames = self.grow_frames + 1 if clean_hi else 0
        need = max(3, int(round(0.3 / dt)))              # 0.3초 동안 깨끗하게
        if self.grow_frames >= need:
            self.best_clean = max(self.best_clean, a_lat_meas)
        if self.slip_frames == max(2, int(round(0.3 / self.cfg.dt))):
            self.slips += 1
            sk = self.s_key
            ratio = a_tot / max(self.a_lat, 1.0)
            if self.prev_brake > 0.1:                    # 제동 중 미끄러짐 → 제동 방향 그립도 깎음
                self.brake_cap = max(0.8 * self.a_lat, self.brake_cap * 0.92)
            # 해밀턴: 먼저 '그 자리'만 깎음 (젖은 노면·범프 같은 국소 문제일 수 있음)
            local_before = self.grip.get(sk)
            self.grip.cut(sk, ratio * 0.97, 4, 2)
            self.slip_spots = [(t, q, r) for (t, q, r) in self.slip_spots if self.t - t < 60.0]
            if local_before >= 0.95 and all(self._sdist(q, sk) > 50.0 for (_, q, _) in self.slip_spots):
                self.slip_spots.append((self.t, sk, ratio))
            # 1분 안에 서로 다른 3곳 이상에서 미끄러졌다 → 국소 문제가 아니라 전체 그립 과대추정
            if len(self.slip_spots) >= 3:
                rs = sorted(r for (_, _, r) in self.slip_spots)
                med = rs[len(rs) // 2]
                new_lat = max(4.0, self.a_lat * min(1.0, med / self.factor * 0.98))
                scale = self.a_lat / new_lat
                for (_, q, _) in self.slip_spots:      # 전체를 깎은 만큼 그 자리 배율은 되돌림
                    self.grip.scale_around(q, 4, scale)
                self.a_lat = new_lat
                self.slip_spots = []
                self.global_slips += 1
        elif self.grow_frames >= need and self.t - self.last_grow > 1.0:
            grow = 1.012 if self.global_slips == 0 else 1.004
            # AIMD: 계획 하중을 다 쓰고도 안 미끄러지면 올림. 단, 실제로 깨끗하게 써 본 최대 하중의 5% 위까지만
            self.a_lat = max(self.a_lat, min(30.0, self.a_lat * grow, self.best_clean * 1.05))
            self.last_grow = self.t

    # ---------------- 복구 ----------------
    def _enter(self, state: str) -> None:
        self.state, self.state_t = state, 0.0
        self.integ = 0.0
        self.calm_t = 4.0
        self.lap_clean = False

    def _recovery(self, fr: Frame, dt: float) -> Optional[Tuple[float, float, float]]:
        if self.state == self.DRIVE:
            stuck = (fr.collided or (abs(fr.v) < 1.0 and self.prev_throttle > 0.3)) and self.t_move > 1.0
            self.stuck_t = self.stuck_t + dt if stuck else 0.0
            wrong = (not fr.moving_forward) and self.prev_throttle > 0.05 and fr.v > 1.0
            self.wrong_t = self.wrong_t + dt if wrong else 0.0
            if self.stuck_t > 0.4:
                self.stuck_t = 0.0
                self._enter(self.REVERSE)
                self.crashes += 1
                self.lap_crashes += 1
                self.grip.cut(self.s_key, self.grip.get(self.s_key) * 0.88, 5, 3)
                # 앞에 알려진 장애물·차가 없는데 막혔다 → 보이지 않는 장애물로 기억해서 다음엔 피함
                known = any(-2.0 < d < 8.0 and abs(m - fr.n) < 3.0 for d, m in fr.obstacles)
                known = known or any(-3.0 < d < 8.0 and abs(m - fr.n) < 3.0 for d, m, _ in fr.opponents)
                wall = abs(fr.n) > fr.half - 1.0
                if fr.collided and not known and not wall and fr.moving_forward:
                    ps = self.s_key + 0.5 * self.cfg.car_length + 1.0
                    if all(self._sdist(ps, a) > 5.0 or abs(fr.n - b) > 2.0 for a, b in self.phantoms):
                        self.phantoms.append((ps, fr.n))
            elif self.wrong_t > 0.5:
                # 역주행: 넓은 쪽으로 크게 돌아 U턴 (7기 팀 방식: 오른쪽에 있으면 오른쪽으로 꺾음)
                self.wrong_t = 0.0
                self._enter(self.UTURN)
                self.uturns += 1
                self.fwd_t = 0.0
                self.uturn_dir = 1.0 if fr.n_raw >= 0 else -1.0
            else:
                return None
        self.state_t += dt
        if self.state == self.UTURN:
            self.fwd_t = self.fwd_t + dt if fr.moving_forward else 0.0
            if self.fwd_t > 0.3:
                self.state = self.DRIVE
                return None
            if self.state_t > 1.0 and (fr.collided or abs(fr.v) < 0.5):
                self._enter(self.REVERSE)
            elif self.state_t > 8.0:
                self._enter(self.REVERSE)
            else:
                return self.uturn_dir, 0.5, 0.0
        if self.state == self.REVERSE:
            # 앞을 막은 장애물이 있으면 돌아 나갈 거리(약 12 m)가 생길 때까지 후진
            ahead_block = any(0 < d < 12 and abs(m - fr.n) < 3.0 for d, m in fr.obstacles)
            ahead_block = ahead_block or any(0 < d < 8 and abs(m - fr.n) < 2.5 for d, m, _ in fr.opponents)
            near_wall = abs(fr.n) > fr.half - 1.6
            steer = 0.0
            if near_wall:
                steer = clamp(fr.psi / 0.3, -1.0, 1.0)       # 후진하며 머리를 도로 쪽으로
            done = self.state_t > 0.5 and not ahead_block and not near_wall and abs(fr.psi) < 0.35
            if done or self.state_t > 4.0:
                self.state, self.state_t = self.SETTLE, 0.0
            return steer, -0.6, 0.0
        # SETTLE: 후진을 멈추고 정지
        if fr.v < -0.3 and self.state_t < 1.0:
            return 0.0, 0.0, 1.0
        self.state = self.DRIVE
        return None

    # ---------------- 전략 ----------------
    def _strategy(self, fr: Frame, dt: float) -> None:
        c = self.cfg
        # 슈마허 + 알베르토: 최고 랩 대비 늦으면(앞이 비었을 때) 플랜 B
        if self.ghost:
            gb = int(self.s_key // c.bin_len)
            if gb in self.ghost:
                delta = self.lap_t - self.ghost[gb]
                clear = (not any(0 < d < 60 for d, _ in fr.obstacles)
                         and not any(0 < d < 60 for d, _, _ in fr.opponents))
                if delta > 0.3 and clear:
                    self.plan_mode = "B"
                elif delta < -0.3:
                    self.plan_mode = "A"
        target = c.factor_plan_b if self.plan_mode == "B" else c.factor_plan_a
        # 베텔: 초반 이후 뒤차와 멀면 관리 주행
        behind = min([-d for d, _, _ in fr.opponents if d < 0] or [999.0])
        early = self.lap == 0 and self.s_key < 170.0
        if c.race_with_others and fr.opponents and not early and behind >= 40:
            target = min(target, c.factor_manage)
        # 베텔 + 문호준: 사고 뒤 침착 모드, 뒤차가 붙으면 해제
        if self.calm_t > 0:
            if behind < 20:
                self.calm_t = 0.0
            else:
                target = min(target, c.factor_calm)
                self.calm_t -= dt
        # 세나: 배율은 천천히 바꿈
        step = c.factor_rate * dt
        self.factor += clamp(target - self.factor, -step, step)

    # ---------------- 기록 ----------------
    def _telemetry(self, fr: Frame, geo: RoadGeometry) -> None:
        in_corner = abs(geo.heading(20.0)) > 6 * DEG
        v = fr.v * 3.6
        if in_corner:
            if self._corner is None:
                self._corner = [self.lap + 1, round(self.s_key, 1), round(v), round(v), 0.0]
            else:
                self._corner[3] = min(self._corner[3], round(v))
        elif self._corner is not None:
            if self._corner[4] == 0.0 or v > self._corner[4]:
                self._corner[4] = round(v)
            if abs(geo.heading(60.0)) > 6 * DEG:     # 다음 코너가 보이면 기록 확정
                self.corner_log.append(self._corner)
                self._corner = None


# =====================================================================
# 비상용 단순 주행 (주행 AI에서 예외가 나면 대신 운전 → 차가 멈추지 않게)
# =====================================================================
def fallback_drive(sensing_info: Any, half_limit: float) -> Tuple[float, float, float]:
    speed = float(_get(sensing_info, "speed", 0.0) or 0.0)
    angles = list(_get(sensing_info, "track_forward_angles", []) or [0] * 20)
    tm = float(_get(sensing_info, "to_middle", 0.0) or 0.0)
    ma = float(_get(sensing_info, "moving_angle", 0.0) or 0.0)
    idx = min(int(speed / 45), len(angles) - 1)
    ref = angles[idx] if idx > 0 else 0
    steer = (ref - ma) / (max(speed, 20.0) * 1.0) - tm / 80.0
    sharp = max([abs(a) for a in angles[:max(1, int(speed / 25))]] or [0])
    if sharp > 40 and speed > 70:
        return clamp(steer, -1, 1), 0.0, 0.6
    return clamp(steer, -1, 1), (0.7 if speed < 110 else 0.4), 0.0


# =====================================================================
# 대회 템플릿
# =====================================================================
def local_map_number() -> Optional[str]:
    """로컬 연습 때만 사용: 문서\\AirSim\\settings.json 의 Map 번호 (맵별로 학습값을 따로 저장하려고).
    대회 서버에서는 호출하지 않음."""
    docs = None
    try:
        from win32com.shell import shell, shellcon
        docs = shell.SHGetFolderPath(0, shellcon.CSIDL_PERSONAL, None, 0)
    except Exception:
        docs = None
    if not docs:
        docs = os.path.join(os.path.expanduser("~"), "Documents")
    try:
        with open(os.path.join(docs, "AirSim", "settings.json"), encoding="utf-8-sig") as f:
            value = json.load(f).get("Algo", {}).get("Map")
    except Exception:
        return None
    if value is None:
        return None
    text = "".join(ch for ch in str(value) if ch.isalnum())
    return text or None


class DrivingClient(DrivingController):
    def __init__(self):
        # =========================================================== #
        #  Area for member variables =============================== #
        # =========================================================== #
        # Editing area starts from here
        #

        self.is_debug = False

        # api or keyboard
        self.enable_api_control = True  # True(Controlled by code) /False(Controlled by keyboard)
        super().set_enable_api_control(self.enable_api_control)

        # 주행 AI 설정
        #  - run_bot.bat 로 실행하면(SSAFY_LOCAL=1) 맵별 학습값과 주행 로그를 저장
        #      memory_map<맵번호>.json : 그 맵의 학습값 (같은 맵을 다시 달릴 때 전부 사용)
        #      memory_car.json         : 가장 최근 학습값 (처음 달리는 맵에서는 차 특성만 가져다 씀)
        #      logs/drive_map<맵번호>_날짜_시간.jsonl : 0.1초마다 주행 기록
        #  - 서버에 제출하면 이 환경 변수가 없으므로 자동으로 모두 꺼짐 (파일 읽기·쓰기 없음)
        local = os.environ.get("SSAFY_LOCAL") == "1"
        mem_path = mem_shared = log_path = None
        if local:
            map_no = local_map_number()
            tag = "map" + map_no if map_no else "map"
            mem_path = "memory_" + tag + ".json"
            mem_shared = "memory_car.json"
            try:
                os.makedirs("logs", exist_ok=True)
                log_path = os.path.join("logs", "drive_" + tag + "_" + time.strftime("%Y%m%d_%H%M%S") + ".jsonl")
            except Exception:
                log_path = None
            print("[MyCar] local practice: map={} memory={} log={}".format(map_no, mem_path, log_path))
        self.brain_cfg = Config(memory_path=mem_path, memory_shared_path=mem_shared, log_path=log_path)
        self.brain = None
        self.brain_errors = 0

        #
        # Editing area ends
        # ==========================================================#
        super().__init__()

    def control_driving(self, car_controls, sensing_info):

        # =========================================================== #
        # Area for writing code about driving rule ================= #
        # =========================================================== #
        # Editing area starts from here
        #

        if self.brain is None:
            self.brain = RaceBrain(self.brain_cfg, self.half_road_limit)

        try:
            steer, throttle, brake = self.brain.step(sensing_info)
        except Exception as e:                        # 어떤 경우에도 차는 계속 달리게
            self.brain_errors += 1
            if self.brain_errors <= 3:
                import traceback
                print("[MyCar] brain error:", e)
                traceback.print_exc()
            steer, throttle, brake = fallback_drive(sensing_info, self.half_road_limit)

        car_controls.steering = steer
        car_controls.throttle = throttle
        car_controls.brake = brake

        if self.is_debug:
            print("[MyCar] steering:{}, throttle:{}, brake:{}".format(car_controls.steering, car_controls.throttle, car_controls.brake))

        #
        # Editing area ends
        # ==========================================================#
        return car_controls

    # ============================
    # If you have NOT changed the <settings.json> file
    # ===> player_name = ""
    #
    # If you changed the <settings.json> file
    # ===> player_name = "My car name" (specified in the json file)  ex) Car1
    # ============================
    def set_player_name(self):
        player_name = ""
        return player_name


if __name__ == '__main__':
    print("[MyCar] Start Bot! (PYTHON)")
    client = DrivingClient()
    return_code = client.run()
    print("[MyCar] End Bot! (PYTHON)")

    exit(return_code)
