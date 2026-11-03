# Jetson Codex 인계 — Robot arm 1 (IK 보정판, 2026-10-27)

## 역할과 확정 사항

사용자는 통합 관제 프로젝트의 Robot arm 1 담당이다. 이 젯슨이 로봇 PC를 대신해 팔1 Action 서버(`/arm1/data`, ROS 도메인 40)를 실행한다. 관제 PC 쪽 프로그램은 다른 담당자 것이며 바꾸지 않는다.

- SELECT_A = 보라 세트 한 벌(보라 아래 세트 먼저, 없으면 보라 위). SELECT_B = 주황 세트 한 벌(주황 왼쪽 먼저, 없으면 오른쪽). 세트 선택은 D435 색 판정(`integration/arm1/vision_judge.py`, `vision-map.json`).
- 실행은 검증 계획(`integration/arm1/plans/`, 15Hz 관절 계획). PRECHECK에서 목적지 팔레트(버거)를 D435로 3장 측정(`jig_cells.py`) → m2 기준이면 원래 계획, m1이면 `plans/m1/` 검증 계획, 그 외에는 `ik/isaac_migration/real/real_plan_burger.py`로 부품별 IK 보정 계획을 만든다(별도 프로세스, 서버 Python, 제한 90초). 실물 추가 보정은 `ik/burger_m1_20261025/real_anchors.json`.
- ACT 모델(A newmask 15만, B2 25만)은 서버 시작 시 함께 로드된다. CUDA torch 필요.
- 완료: 목적지 확인 후 시작 자세 복귀, 복귀 확인 뒤 success 결과. 토크 유지.

## 설치 순서

1. `bash jetson-run.sh verify` (SHA256).
2. 10/08 패키지 환경이 동작하면 `bash install-jetson.sh --ik-only`: 그 venv를 찾아 `wheels/`의 ARM64 mujoco 3.3.7·scipy 1.18.1만 오프라인 설치, host_pkg 준비, doctor. 안 되면 원인을 보고하고 사용자와 정한다. JetPack/OS/torch를 바꾸지 않는다.
3. `doctor --require-cuda` → `ik-test` → `check-models --device cuda` → `benchmark --device cuda`. 모두 장치를 열지 않는다. ik-test의 IK 시간도 기록한다(서버 제한 90초).
4. 실제 팔은 사용자가 직접 실행한다: `bash run-arm1-server.sh`. PC의 서버(warehouse-arm1.service)와 동시에 켜지 않는다.

## 이식 수정 (PC 원본과의 차이, 이 사본에만)

- `model-map.json`: 계획·m1 계획·보정값·IK 스크립트 경로를 패키지 기준 상대 경로로. `retarget_python` 빈 값 = 서버 Python(`backend.py`에서 `sys.executable`).
- 계획 JSON의 `robot_calibration`을 `arm3-current/arm3/calibration/arm3.json` 상대 경로로(내용·SHA256 검사는 그대로).
- `jig_cells.py`·`make_jig_reference.py`의 카메라 보정 경로, `ik/.../real_lid_plan.py`의 ARM3 경로를 패키지 기준으로.
- `run_policy.py`의 데이터셋 경로(10/08과 같음). 상대 경로는 패키지 루트 기준이며 `jetson-run.sh`가 루트에서 실행한다.
- 모니터 GUI(`gui.py`, `sim_view.py`)는 넣지 않았다. PC에서 네트워크로 본다.

## 바꾸면 안 되는 것

- 차루코 카메라 이동 검사, 측정 거부 조건(점수·강체 잔차·허용 범위), 계획의 서보 한계 검사, 250ms 제어 지연 제한, 토크·온도·그리퍼 검사를 성능 문제의 우회책으로 끄지 않는다.
- `camera_transforms.json`과 `jig-reference-m2.png`는 지금의 D435 위치에서만 맞다. 카메라가 움직였으면 보정을 다시 해야 하며 검사를 끄면 안 된다.
- 가동 중인 다른 로봇 프로세스를 임의로 끄지 않는다. 서버 종료는 토크 해제.

## 검증 기록

PC 검증은 `PACKAGING_VALIDATION.md`. 젯슨에서 한 결과는 `JETSON_VALIDATION.md`에 따로 남기고 안 한 항목도 표시한다. 현장 순서: doctor → ik-test → check-models/benchmark → 서버 → SELECT_A(m2/m1/다른 위치) → SELECT_B → EMER_STOP/RESTART.
