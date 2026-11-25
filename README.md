# 싸피레이스 작업 폴더 (workspace)

원본 템플릿(`MyCar_20261113`)은 그대로 두고, 이 폴더에서 코드를 고치고 실행합니다.

이 폴더는 GitHub 공개 저장소 [JaeYunAn/SSAFY_Race_Winter](https://github.com/JaeYunAn/SSAFY_Race_Winter) (브랜치 `main`)와 연결되어 있습니다. 코드는 git 으로 여러 PC에서 맞추고, SSAFY 제공 파일·학습값·로그는 PC마다 따로 둡니다.

## 폴더 구성

| 파일 | 설명 |
|---|---|
| `my_car.py` | 주행 AI (제출 파일). 자유롭게 고치면 됩니다 |
| `my_car_basic.py` | 대회 기본 예제 `1_Basic/my_car.py` 복사본 (비교·배틀 상대용) |
| `my_car_template.py` | 빈 템플릿 `Bot_Python/my_car.py` 복사본 (새로 시작할 때) |
| `DrivingInterface/` | 대회 제공 통신 모듈 원본 복사본. **고치지 말 것** (서버는 원본을 씀) |
| `setup_python.bat` | 처음 한 번: SSAFY 템플릿 파일 복사 + Python 3.7 전용 환경(`.venv`)과 패키지 설치 |
| `run_bot.bat` | 봇 실행 (기본 `my_car.py`). `run_bot.bat 파일이름.py` 로 다른 파일도 실행 |
| `run_bot_basic.bat` | 기본 예제 실행 |
| `run_offline_test.bat` | 시뮬레이터 없이 `my_car.py` 점검 (가짜 물리 + 진짜 drive_controller) |
| `offline_test/` | 오프라인 점검 코드 |

**git 저장소에 없는 것** (`.gitignore`, PC마다 따로 있음)

| 파일 | 이유 / 구하는 법 |
|---|---|
| `DrivingInterface/`, `my_car_basic.py`, `my_car_template.py` | SSAFY 제공 파일. `setup_python.bat` 이 그 PC의 템플릿에서 복사 |
| `.venv/` | PC마다 `setup_python.bat` 이 만듦 |
| `memory_*.json`, `logs/` | 주행할 때마다 바뀌는 학습값·로그. 옮기려면 직접 복사 |
| `*.pdf` | 대회 가이드 (대외비) |
| `..\start_simulator.bat` | 이 폴더 바깥(바로 위 폴더)에 둠. 작업 키트(`ssafy_race_kit.zip`)에 들어 있음 |

## 처음 한 번 (새 PC)

1. 저장소 받기: SSAFY 파일(`MyCar_...` 폴더와 시뮬레이터)이 있는 폴더에서 PowerShell 또는 Git Bash 를 열고
   ```
   git clone https://github.com/JaeYunAn/SSAFY_Race_Winter.git workspace
   ```
   - git 이 없으면 [git-scm.com](https://git-scm.com) 에서 받아 기본값으로 설치하고, 처음 한 번 `git config --global user.name "이름"`, `git config --global user.email "GitHub 이메일"`
   - 공개 저장소라서 받기(clone·pull)는 로그인 없이 됩니다. 처음 push 할 때 GitHub 로그인 창이 뜹니다 (쓰기 권한이 있는 계정만 올릴 수 있음)
   - `start_simulator.bat` 은 저장소에 없으므로 작업 키트에서 꺼내 `workspace` 바로 위 폴더에 둡니다
2. `workspace\setup_python.bat` 더블클릭
   - 이 PC의 SSAFY 템플릿(`MyCar_...\Template_Python`)에서 `DrivingInterface`, 기본 예제를 자동으로 복사합니다 (`workspace` 의 바로 위·그 위 폴더에서 찾음)
   - Python 3.7이 없으면 Y를 누르면 python.org에서 3.7.9를 받아 이 계정에만 설치합니다 (관리자 권한·PATH 변경 없음)
   - 기존 Python 3.12는 그대로 둡니다
   - 마지막에 `OK: Python 3.7.9 / numpy ...` 가 나오면 끝
3. (선택) 학습된 차 특성으로 출발하려면 다른 PC의 `memory_map*.json`, `memory_car.json` 을 이 폴더에 복사

## 매번 실행 순서

1. `SSAFY_RACE\start_simulator.bat` 더블클릭 → 맵 번호 입력
   - `문서\AirSim\settings.json` 을 자동으로 새로 쓰고, 인텔 CPU 충돌 회피 변수를 켜고, 시뮬레이터를 켭니다
   - 시뮬레이터 창 오른쪽 위에 `Car1` 이 보이면 준비 완료 (설정에 `"SimMode": "Car"` 가 들어 있어서 Choose Vehicle 창은 뜨지 않음)
   - UE4 Prerequisites 설치 창은 필요한 구성 요소가 없는 PC에서만 뜸 → 뜨면 설치
2. 시뮬레이터 화면이 뜨면 `workspace\run_bot.bat` 더블클릭

| 번호 | 맵 | 용도 |
|---|---|---|
| 1 | 10 베이직 | 대회 |
| 2 / 3 | 31 / 30 스피드 (장애물 있음/없음) | 대회 / 연습 |
| 4 / 5 | 61 / 60 싸피 (장애물 있음/없음) | 대회 / 연습 |
| 6 / 7 | 71 / 70 싸피 저사양 | 대회 / 연습 |
| 8 / 9 | 161 / 160 독일 (장애물 있음/없음) | 추가 맵 |
| 10 / 11 | 61 / 71 배틀 (차 2대) | 대회 |

**배틀**: 10번으로 시뮬레이터를 켠 뒤 `run_bot.bat` 를 두 번 실행합니다. 먼저 실행한 봇이 Car1, 나중 것이 Car2가 됩니다 (drive_controller 가 아직 제어되지 않은 차를 이름순으로 잡음). 상대를 기본 예제로 하려면 두 번째는 `run_bot_basic.bat`.

## VS Code에서 고치고 실행하기

1. VS Code → 파일 → 폴더 열기 → `SSAFY_RACE\workspace`
2. Python 확장이 없으면 설치 (확장 탭에서 `Python`, Microsoft)
3. 처음 한 번 인터프리터 선택: Ctrl+Shift+P → `Python: Select Interpreter` → `.venv` (Python 3.7.9) 선택
4. 실행: 터미널(Ctrl+`)을 열고 입력 (PowerShell 터미널은 앞에 `.\` 가 필요)
   - `.\run_bot.bat` → `my_car.py` 실행 (학습값·로그 저장 켜짐)
   - `.\run_bot.bat my_car_basic.py` → 다른 파일 실행
   - `.\run_offline_test.bat` → 시뮬레이터 없이 점검
   - `..\start_simulator.bat` → 시뮬레이터 켜기
5. 편집기 오른쪽 위 ▶(Run Python File)로 실행해도 돌아가지만, 이때는 학습값·로그 저장이 꺼집니다

## 코드를 고칠 때 지켜야 할 것 (대회 규칙)

- 제출은 `my_car.py` 하나
- 고칠 수 있는 곳: import, 생성자 `__init__`, `control_driving`, 직접 만든 함수·클래스
- 부모 클래스 멤버 중 쓸 수 있는 것: `self.half_road_limit` 만
- `DrivingInterface` 안의 파일은 고쳐도 서버에서는 원본이 돌아가므로 의미가 없습니다

## 학습값과 로그

- `run_bot.bat` / VS Code 작업으로 실행하면 `SSAFY_LOCAL=1` 이 켜져서
  - `memory_map<맵번호>.json`: 맵별 학습값 (그립·브레이크·조향 특성 + 그 맵의 최고 랩·고스트·위험 구간)
  - `memory_car.json`: 가장 최근 학습값. 처음 달리는 맵에서는 여기서 차 특성만 가져옵니다 (다른 맵의 랩 기록은 쓰지 않음)
  - `logs\drive_날짜_시간.jsonl`: 프레임별 주행 기록 (튜닝할 때 이 파일을 주면 분석 가능)
- 서버에서는 이 변수가 없으므로 파일을 하나도 쓰지 않습니다
- 학습값과 로그는 git 에 올리지 않습니다 (PC마다 따로 쌓임)
- 제출 전: `memory_map<맵번호>.json` 내용을 `my_car.py` 의 `EMBEDDED_MEMORY = None` 자리에 붙여 넣으면 서버에서도 학습된 차 특성으로 출발합니다 (서버 맵을 알 수 없으므로 랩 기록은 쓰지 않음)

## 시뮬레이터 조작

- F1 도움말, F8 키보드 주행 (봇을 한 번 돌린 뒤에는 Backspace로 초기화해야 가능)
- Backspace 리셋, F / B / `\` / `/` 카메라
- 이상하면 작업 관리자에서 `Algo-Win64-Shipping.exe` 종료 후 다시 실행 (`start_simulator.bat` 는 자동으로 종료하고 다시 켬)

## 여러 PC에서 코드 맞추기 (git)

| 언제 | VS Code | 터미널 (workspace 폴더에서) |
|---|---|---|
| 작업 시작 전: 최신 코드 받기 | 소스 제어(Ctrl+Shift+G) → **변경 내용 동기화** | `git pull` |
| 고친 뒤: 올리기 | 소스 제어에서 메시지 쓰고 **커밋** → **변경 내용 동기화** | `git add -A` → `git commit -m "무엇을 왜 바꿨는지"` → `git push` |

- **작업을 시작하기 전에 항상 먼저 pull 합니다.** 두 PC에서 같은 파일을 따로 고치면 충돌이 납니다.
- 학습값(`memory_*.json`)과 로그는 git 으로 옮겨지지 않습니다. 다른 PC의 학습값을 쓰려면 파일을 직접 복사합니다.
- 여러 사람이 쓰는 PC(SSAFY 등)라면 사용 후 제어판 → 자격 증명 관리자 → Windows 자격 증명에서 `git:https://github.com` 을 지웁니다.

**이미 `workspace` 폴더가 있는 PC를 저장소에 처음 연결할 때** (예전 키트로 쓰던 PC)

VS Code 를 닫고, `workspace` 가 들어 있는 폴더에서 PowerShell 로:
```
Rename-Item workspace workspace_old
git clone https://github.com/JaeYunAn/SSAFY_Race_Winter.git workspace
Copy-Item -Recurse workspace_old\.venv, workspace_old\DrivingInterface, workspace_old\logs workspace\
Copy-Item workspace_old\my_car_basic.py, workspace_old\my_car_template.py, workspace_old\memory_*.json workspace\
git diff --no-index workspace_old\my_car.py workspace\my_car.py
```
- 마지막 줄에서 아무것도 안 나오면 예전 `my_car.py` 와 저장소 것이 같습니다. 다르면 예전 것에만 있는 수정이 있다는 뜻이니 확인하고 옮깁니다.
- `.venv` 를 옮긴 뒤 실행이 안 되면 `workspace\setup_python.bat` 을 한 번 실행합니다.
- `workspace_old` 는 확인이 끝나면 지워도 됩니다.

## 문제 해결

| 증상 | 해결 |
|---|---|
| `Cannot parse JSON settings_json string` | `start_simulator.bat` 로 켜면 settings.json 을 ASCII로 새로 씀 |
| 시뮬레이터가 켜지자마자 꺼짐 | `start_simulator.bat` 가 `OPENSSL_ia32cap` 를 켜고 실행하므로 run.bat 대신 이것을 사용 |
| `No module named msgpackrpc` | `python my_car.py` 대신 `run_bot.bat` (= `.venv` Python 3.7) 로 실행 |
| `settings.json` 을 못 찾는다는 오류 | `start_simulator.bat` 를 한 번 실행하면 `문서\AirSim` 폴더가 만들어짐 |
| 봇이 안 움직임 | 시뮬레이터를 먼저 켜고 창 오른쪽 위에 `Car1` 이 보이는지 확인 |

> 대회 가이드 PDF는 대외비입니다. GitHub 등에 올리지 마세요 (`.gitignore` 에 `*.pdf` 포함). 공개 저장소이므로 대회 가이드 PDF 내용과 SSAFY 제공 파일은 절대 올리지 않습니다.
