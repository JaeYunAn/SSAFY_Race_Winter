# 자동 개선 반복 기록

형식: 본 것 → 판단 근거(숫자) → 바꾼 것 → 결과 → 채택/버림. 규칙은 CLAUDE.md '자동 개선 반복' 절.
시험 주행은 모두 서버 조건 재현 모드 (맵 학습값 안 씀, 차 특성은 tune/baseline_car.json: a_lat 14.0, a_brake 13.85, 조향 지연 0.5초).

## 0단계 (2026-11-28)

- 도구: tune/run_trial.py, tune/analyze_log.py, tune/loop_util.py. my_car.py 에 SSAFY_TUNE 입구와 전략 스위치(use_*), grip_scale, learn_grip 추가
  - 오프라인 시험 25개 전후 같음 (완주 시간·충돌·벌점·리셋·예외 모두 동일, 예외 0)
- 기록은 공식 기록(API ac_PlayerRaceTime). 첫 주행 Map 10: 공식 90.26초 / 로그 90.04초 (멈춤 0.98초), 랩 46.95·41.99, 충돌·벌점 0
