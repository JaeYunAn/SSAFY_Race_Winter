# 싸피레이스 Winter 주행 AI 프로젝트

SSAFY 싸피레이스(AirSim 기반 3D 주행 시뮬레이터 "Algo")에서 차를 자동으로 운전하는 Python 봇이다. 설명과 답변은 한국어로 한다.

## 작업 방식 (모든 작업에서 지킨다)

일정: 튜토리얼 2026-11-26 16:00 / 레이스 11-27 ~ 12-27 / **최종 코드 제출 2026-12-27 23:59:59** / 결선 2027-01-08

1. 코드를 고치기 전에 무엇을 왜 바꿀지 먼저 설명하고 사용자의 확인을 받는다.
2. 한 번에 한 가지만 바꾼다. 바꾸기 전에 `offline_test/run_all.py` 결과를 받아 두고, 바꾼 뒤 다시 돌려서 완주 시간·충돌·벌점·예외를 전후 표로 비교해 보고한다.
3. 시뮬레이터는 사용자가 직접 켠다. 실제 주행 확인이 필요하면 맵 번호와 횟수를 알려 준다 (예: "Map 10 으로 2번").
4. 실제 주행 뒤에는 `logs/` 의 가장 최근 파일을 분석해서 근거로 쓴다.
   - s·prog·v 가 여러 프레임 동안 그대로인 구간은 시뮬레이터가 멈춘 것이다. 시간·가감속 계산에서 뺀다.
5. 아래 '대회 규칙'과 '코드 조건'을 지킨다: my_car.py 만 수정, `half_road_limit` 외 부모 멤버 금지, 좌표 하드코딩 금지, Python 3.7 문법, 서버에서 파일 쓰기 금지.
6. 큰 변경 전에는 `my_car.py` 를 `backup/my_car_YYYYMMDD_HHMMSS.py` 로 복사해 둔다.
7. my_car.py 의 레전드 전략은 지우거나 끄지 않는다.
8. 코드는 GitHub 비공개 저장소 `JaeYunAn/SSAFY_Race_Winter` (이 workspace 폴더가 루트, 브랜치 `main`) 로 집 노트북과 SSAFY PC 사이에서 맞춘다.
   - 작업을 시작할 때 `git status` 와 `git pull` 로 다른 PC에서 올린 변경이 있는지 먼저 확인한다.
   - **저장소 안의 파일을 고칠 때마다 커밋하고 바로 push 한다** (사용자 지시, 2026-11-25). 따로 묻지 않는다.
     - 커밋 메시지는 한국어로: 첫 줄에 한 줄 요약, 본문에 파일별로 무엇을 왜 바꿨는지.
     - my_car.py 를 고쳤으면 2번의 오프라인 시험 비교까지 끝낸 뒤 커밋하고, 본문에 전후 결과(완주 시간·충돌·벌점·예외)를 적는다.
     - push 전에 `git fetch` 로 다른 PC의 변경을 확인하고, 있으면 먼저 받는다. 충돌이 나면 push 하지 말고 사용자에게 알린다.
     - `.claude/settings.json` 이 `git fetch`·`git pull`·`git push` (와일드카드 없이 정확한 형태만) 를 허용한다. 이 명령들은 `&&` 로 다른 명령과 묶지 말고 **단독으로** 실행한다 (묶으면 허용 규칙에 안 맞아 자동 모드 검사에 막힘).
   - 학습값(`memory*.json`)·로그(`logs/`)는 git 에 없어서 PC마다 다르다. 로그를 분석할 때는 이 PC에서 달린 기록인지 생각한다.

| 전략 | 코드 위치 |
|---|---|
| 알베르토: 출발 직후 조향 흔들어 특성 파악 | `RaceBrain._sysid_weave` |
| 베르스타펜: 가장 늦은 브레이킹·트레일 브레이킹, 브레이크 감속 학습 | `SpeedPlanner.build` 후진 패스, `_learn` 의 제동 학습, `DecelModel` |
| 해밀턴: 10 m 구간별 그립 지도 | `GripMap` |
| 슈마허 + 알베르토: 최고 랩보다 느리면 플랜 B | `_strategy` (`factor_plan_a/b`) |
| 세나: 속도 배율 천천히 변경 | `_strategy` (`factor_rate`) |
| 베텔: 배틀에서 크게 앞서면 관리 주행 | `_strategy` (`factor_manage`) |
| 베텔 + 문호준: 사고 뒤 침착 모드, 추격받으면 해제 | `_strategy` (`calm_t`, `factor_calm`) |
| 문호준: 장애물 2단계 위험 비용 | `PathPlanner.plan` (`w_near`) |

## 고치는 파일

- **`my_car.py` 하나만 고친다.** 대회에는 이 파일 하나만 제출한다.
- `DrivingInterface/`, `my_car_basic.py`, `my_car_template.py` 는 `setup_python.bat` 이 각 PC의 SSAFY 템플릿에서 복사해 온다.
- `DrivingInterface/` 는 대회 제공 원본이다. 서버는 원본을 쓰므로 **절대 고치지 않는다.** (센서 계산 방식을 확인할 때 읽기만 한다)
- `my_car_basic.py`(대회 기본 예제), `my_car_template.py`(빈 템플릿)는 참고용이다.
- `offline_test/` 는 시뮬레이터 없이 my_car.py 를 시험하는 도구다 (가짜 물리 + 진짜 drive_controller.py).
- git 저장소에 들어 있는 것: my_car.py, offline_test/, .bat 스크립트, README.md, CLAUDE.md, requirements.txt, .gitignore, .gitattributes, .claude/settings.json (Claude Code 권한 설정)
- `.gitignore` 로 빼는 것: `DrivingInterface/`, `my_car_basic.py`, `my_car_template.py`, `.venv/`, `logs/`, `memory*.json`, `*.pdf`. 이 줄들을 지우지 않는다.
- `.gitattributes` 는 `.bat` 을 CRLF 로 고정한다 (LF 면 `goto` 가 깨질 수 있음).

## 대회 규칙 (어기면 실격 위험)

- my_car.py 에서 고칠 수 있는 곳: import, `DrivingClient.__init__`, `control_driving`, 직접 만든 함수·클래스
- 부모 클래스(DrivingController) 멤버 중 쓸 수 있는 것은 `self.half_road_limit` 하나뿐이다. `self.way_points` 등 다른 내부 값은 쓰지 않는다.
- 트랙 좌표를 코드에 직접 적어 넣지 않는다 (센서 값으로만 판단).
- 서버에서는 파일을 읽거나 쓰지 않는다. 학습값·로그 저장과 맵 번호 읽기(`local_map_number`)는 환경 변수 `SSAFY_LOCAL=1` 일 때만 한다 (이미 그렇게 되어 있음).

## 코드 조건

- **Python 3.7** 문법만 쓴다 (서버 기준). 쓰면 안 되는 것: `:=`, `match`, `list[int]`·`dict[str, int]` 같은 내장 제네릭 표기(→ `typing.List` 사용), `X | Y` 타입, 딕셔너리 `|` 합치기, f-string `=` 디버그, 위치 전용 매개변수 `/`
- 외부 패키지는 numpy 만 쓴다 (템플릿이 이미 쓰는 것). 나머지는 표준 라이브러리.
- `control_driving` 은 0.1초마다 불린다. 한 번 계산이 수 ms 안에 끝나야 한다 (현재 평균 약 3 ms).
- 예외가 나도 차가 멈추지 않도록 `control_driving` 의 try/except + `fallback_drive` 구조는 유지한다.

## my_car.py 구조

| 위치 | 역할 |
|---|---|
| `EMBEDDED_MEMORY` | 제출 전에 학습값을 붙여 넣는 곳. 서버 맵을 알 수 없으므로 차 특성만 쓰고 맵 기록은 버린다 |
| `class Config` | 튜닝 값 모음 (도로 여유, 경로 계획 비용, 초기 추정값, 속도 배율 등) |
| `class SsafyAdapter` | 대회 sensing_info → 내부 Frame 변환 (경유점 통과, 랩 감지, 기준선) |
| `RoadGeometry`, `PathPlanner` | 기준선 곡률, 격자 DP + 곡률 최소화 경로 |
| `SpeedPlanner` | 마찰원 기반 속도 계획 |
| `SteerGain`, `DriveModel`, `DecelModel`, `GripMap` | 조향 반응·가속·제동·구간별 그립 실시간 학습 |
| `FrameLogger` | logs/*.jsonl 기록 |
| `class RaceBrain` | 위 부품을 합친 주행 두뇌 (상태: DRIVE, 복구, U턴 등) |
| `fallback_drive` | 오류 시 비상 주행 |
| `class DrivingClient` | 대회 클래스. 생성자와 `control_driving` |

## 실행과 시험

- 시뮬레이터는 사용자가 직접 켠다: `..\start_simulator.bat` (맵 번호 선택, 저장소 밖 바로 위 폴더에 있음). Claude 는 시뮬레이터를 켜거나 끄지 않는다.
- 봇 실행 (시뮬레이터가 켜져 있을 때만): `run_bot.bat` 또는 `./.venv/Scripts/python.exe my_car.py`
- **코드를 고친 뒤에는 반드시 오프라인 시험을 돌린다** (시뮬레이터 불필요, 약 1분):
  `./.venv/Scripts/python.exe offline_test/run_all.py`
  - 시험 18개의 완주 시간·충돌·벌점·리셋·예외 합계가 나온다. 고치기 전과 비교해서 보고한다.
  - 기본 가짜 물리는 페달이 즉시 듣는다. 실제처럼 페달이 늦게 듣는 경우는 '페달 지연 0.25초 차' 시험 2개(`phys` 의 `pedal_lag`)로 본다.
  - 트랙 하나만: `./.venv/Scripts/python.exe offline_test/run_race.py basic 1` (트랙: basic, speed, ssafy, germany, hidden)
  - 출력에서 UnicodeEncodeError 가 나면 환경 변수 `PYTHONIOENCODING=utf-8` 을 붙여 실행한다.
- 가짜 물리는 실제 시뮬레이터와 다르다. 최종 판단은 실제 주행 로그로 한다.

## 학습값 파일 (로컬 연습 때만)

- `memory_map<맵번호>.json`: 그 맵의 학습값. 같은 맵을 다시 달리면 차 특성 + 랩 길이·고스트·구간 그립·숨은 장애물을 모두 쓴다.
- `memory_car.json`: 가장 최근 학습값. 처음 달리는 맵에서는 여기서 차 특성(조향·가속·제동·그립)만 가져온다.
- 맵 번호는 `문서\AirSim\settings.json` 의 `Algo.Map` 에서 읽는다 (`local_map_number`).
- 다른 맵의 랩 길이·고스트를 쓰면 랩 계산과 플랜 A/B 가 틀어지므로 맵 기록은 같은 맵에서만 쓴다.
- 학습값 파일은 git 에 올리지 않는다 (매 주행마다 바뀌어 두 PC에서 충돌함). 다른 PC로 옮길 때는 사용자가 직접 복사한다.

## 실제 주행 로그 (logs/drive_map<맵번호>_날짜_시간.jsonl, 0.1초당 한 줄)

| 키 | 뜻 |
|---|---|
| t | 봇 시작 후 시간(초) |
| s | 누적 주행 거리(m) |
| v | 속도(km/h) |
| n / n_raw | 기준선에서 옆으로 떨어진 거리(m, +오른쪽) / 보정 전 값 |
| psi | 진행 방향 오차(도) |
| kr0 | 기준선 곡률(1/m) |
| prog | 대회 API 진행률(%) |
| fwd / col / tp | 정방향 여부 / 충돌 / 순간이동(리셋) 감지 |
| obs / opp | 앞 장애물 [거리, 옆 위치] / 상대 차 [거리, 옆 위치, 속도] |
| st / th / br | 내보낸 조향 / 가속 / 브레이크 |
| state / mode / fac | 주행 상태 / 계획 모드 / 속도 배율 |
| vref | 목표 속도(km/h). 브레이크 지연 보상 뒤로는 '명령이 듣는 시점(약 0.25초 뒤) 위치'의 목표 속도 |
| vpred | 지연 보상용 예측 속도(km/h): 지금 속도 + 아직 듣지 않은 최근 명령의 효과 |
| kdes / kmeas | 목표 곡률 / 실제 곡률 |
| alat / abrk / gain | 학습된 코너 한계·제동(m/s²) / 조향 반응(1/m per steer) |
| plan / lohi | 앞 경로의 옆 위치 / 갈 수 있는 좌우 범위 |

- 대회 벌점: 도로 중심에서 `half_road_limit` 보다 멀고 속도 40 km/h 초과면 브레이크가 0.9로 강제된다. 로그에서는 가속 중 갑작스러운 감속으로 보인다.

## 첫 실제 주행 결과 (2026-11-24, 베이직 Map 10, 2랩)

- 대회 기록 1:45.72 (1랩 약 57.1초, 2랩 48.6초), 충돌 0, 벌점 흔적 없음, 최고 139 km/h
- 실제 조향 반응은 약 0.065~0.08 (초기값 0.2의 약 1/3), 지연 약 0.2~0.3초
- 학습값: 코너 9.6, 제동 9.3, 출발 가속 3.4 m/s²

## 보안

- 대회 가이드 PDF(`..\*.pdf`)는 대외비다. 내용을 외부로 옮기거나 저장소에 올리지 않는다.
- GitHub 저장소는 비공개로 유지한다. SSAFY 제공 파일(`DrivingInterface/`, 기본 예제)도 올리지 않는다.
