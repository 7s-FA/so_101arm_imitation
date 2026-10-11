# SO-101 Robot arm 1 — ROS 2 Action 서버

모방학습 모델 A/B를 선택해 재료 한 세트(3개)를 목적지 지그로 운송하는 로봇 PC용 관제 연결 코드입니다. 관제가 Goal을 보내면 로봇 PC가 수행하고, 영상 완료 판정과 저장된 시작 자세 복귀를 확인한 뒤 최종 Result를 반환합니다.

이 저장소에는 **액션 서버, 관제 클라이언트, ROS 인터페이스, 모의 테스트**가 있습니다. Git 소스에는 학습 데이터·모델 가중치·카메라 영상·운영 로그·로봇별 보정값·기존 모방학습 런타임을 포함하지 않습니다. 운영 A/B 모델 가중치는 아래 Release 첨부파일로 제공합니다. 단독 체크아웃으로 모의 통신 시험이 가능하며, 실물 실행은 기존 `so101_imitation` 작업 폴더에 이 코드를 배치해야 합니다.

## 통신 규격

| 항목 | 값 |
|---|---|
| ROS | ROS 2 Jazzy / Ubuntu 24.04 / Python 3.12 |
| Action | `/arm1/data` |
| 타입 | `host_pkg/action/Arm` |
| 실물 도메인 | `40` |
| 모의 도메인 | `141`, localhost 한정 |
| 상태 토픽 | `/arm1/status` (`std_msgs/msg/String`, JSON) |

```text
# Goal
string command
---
# Result
bool success
string message
---
# Feedback
string message
```

| 명령 | 동작 |
|---|---|
| `SELECT_A` | 보라색 계열 재료 한 세트, 3개 운송 |
| `SELECT_B` | 주황색 계열 재료 한 세트, 3개 운송 |
| `EMER_STOP` | 현재 자세로 정지, 토크 유지; 정지 확인 후 응답 |
| `RESTART` | 정지했던 작업 단계에서 재개 |

정상 Result는 `success=true, message="IDLE"`, 실패 Result는 `success=false, message="ERROR"`입니다. 작업 Goal 접수나 `/arm1/status`의 IDLE 알림 자체는 작업 완료 Result가 아닙니다. 정지·재개 명령의 성공도 운송 작업 완료와 별개입니다.

## 최신 동작 정책 (2026-10-11)

- 일반 작업: `PRECHECK → HOMING → WORKING → RETURNING → IDLE`.
- 시작할 때 이미 완료 배치라면 새로운 카메라 영상으로 1초간 확인하고 운송 동작을 건너뜁니다. `PRECHECK → RETURNING → IDLE` 순서로 복귀한 뒤 정상 완료 결과를 보냅니다.
- 완료 후 토크를 유지하고 다음 명령을 기다립니다. **같은 명령이 다시 오면 다시 처리하여 요청마다 결과를 반환합니다.** 자체적으로 작업을 반복하거나 결과를 주기적으로 재전송하지 않습니다.
- 작업 진행 중에는 다른 운송 Goal을 거절합니다. Action 취소는 현재 자세 정지 후 취소 결과를 반환합니다.
- 초기 자세 복귀 시 약 5도/초(모터 56 encoder steps/s) 제한을 적용하고, 도착을 확인한 뒤 완료를 보냅니다. 30Hz 제어 주기와 최초 HOMING은 별개입니다.
- 카메라 일시 단절 시 자세를 유지하며 복구를 기다리고, 제한 시간을 초과하면 실패합니다.

## 코드 구성

```text
arm1-control.sh                 # 설치, 검사, 모의/실물 서버, CLI
run-arm1-server.sh              # 실물 관제 대기 실행기
integration/arm1/
  server.py                    # ROS Action, 피드백, Result, 상태 토픽
  engine.py                    # 단일 하드웨어 작업 스레드 및 상태 전이
  backend.py                   # 기존 모방학습 런타임과 연결
  arm1_node.py                 # 관제 PC용 비동기 Action 클라이언트
  client.py                    # 명령행 클라이언트
  model-map.json               # A/B 모델 및 완료 기준 경로
  interfaces/Arm.action
  ros_ws/src/host_pkg/          # 인터페이스 빌드 소스
  tests/                       # 하드웨어 없는 단위/ROS 왕복 테스트
```

## 단독 모의 시험

ROS Jazzy, `colcon`, ROS Python 및 인터페이스 빌드 도구가 설치된 Ubuntu 24.04에서 사용합니다. `setup`은 ROS 자체나 ML 패키지를 설치하지 않습니다. 기존 호환 `host_pkg`가 있으면 사용하며, 없으면 저장소 내부 `ros_ws`에 빌드합니다. 기존 인터페이스가 다르면 중단합니다.

```bash
git clone https://github.com/7s-FA/so_101arm_imitation.git
cd so_101arm_imitation
bash arm1-control.sh test
bash arm1-control.sh setup
bash arm1-control.sh test-ros
```

단위 테스트는 ROS/모델/카메라 없이 실행할 수 있습니다. `test-ros`는 도메인 141에서 모의 서버를 띄우고 검사 후 종료합니다.

직접 모의 서버를 실행하려면:

```bash
bash arm1-control.sh serve
```

다른 터미널에서 같은 저장소로 이동한 뒤:

```bash
bash arm1-control.sh send SELECT_A
bash arm1-control.sh send SELECT_B
```

`--hardware`가 없으면 실제 로봇을 제어하지 않습니다.

## 기존 로봇 PC에 적용

기존 `~/Downloads/so101_imitation`의 동일 상대 경로에 `arm1-control.sh`, `run-arm1-server.sh`, `integration/arm1/` 소스를 배치합니다. 기존 모델·프로젝트 설정·런타임은 보존합니다. 다른 폴더에서 하드웨어 모드를 실행하려면 그 폴더에도 아래 외부 자산을 준비해야 합니다.

필수 외부 자산:

- `arm3-current/.venv/`: PyTorch/LeRobot, 카메라·모터 드라이버 등 설치된 Python 환경.
- `arm3-current/arm3/`: `run_policy.py`, `record_purple.py`, `teleoperate_slow.py`, `start_pose.py`, `task_completion.py` 등 기존 런타임과 의존 모듈·설정.
- 해당 로봇의 카메라/포트 설정, 모터 보정값, 시작 자세, 모션 설정.
- `projects/model_A/`, `projects/model_B2/`: `project.json`, `task.json`, 완료 기준 JSON 및 참조 NPZ, 모델 체크포인트와 런타임이 참조하는 데이터셋 메타데이터·identity·시작 자세.

운영 모델 경로는 `integration/arm1/model-map.json`에서 지정합니다.

| 요청 | 모델 |
|---|---|
| SELECT_A | `projects/model_A/models/model_A_newmask_300k/checkpoints/150000/pretrained_model` |
| SELECT_B | `projects/model_B2/models/model_B2_300k_20261010/checkpoints/250000/pretrained_model` |

설정된 경로와 실제 모델·보정·카메라 identity가 일치해야 합니다. 이 저장소만 복제해 실물 모드를 켜면 외부 자산 검사에서 중단되는 것이 정상입니다.

```bash
cd ~/Downloads/so101_imitation
bash arm1-control.sh setup
bash arm1-control.sh check-models
bash arm1-control.sh serve --hardware
# 또는 bash run-arm1-server.sh
```

`ARM1 READY mode=HARDWARE` 이후 관제 명령을 기다립니다. 목표가 수신되면 별도의 START 입력 없이 동작합니다. 카메라 미리보기·녹화·수동 모델 실행과 동시에 사용하지 마세요. 양쪽 PC의 도메인과 네트워크를 맞춰야 합니다.

```bash
# 연결만 확인: Goal을 보내지 않음
bash arm1-control.sh check-connection --hardware
# 아래는 실제 로봇 명령
bash arm1-control.sh send SELECT_A --hardware
bash arm1-control.sh send SELECT_B --hardware
bash arm1-control.sh send EMER_STOP --hardware
bash arm1-control.sh send RESTART --hardware
```

서버 Ctrl+C는 토크를 해제하므로 팔을 받치고 종료합니다. 클라이언트만 종료하는 것은 로봇 정지가 아닙니다. 소프트웨어 정지는 하드웨어 비상정지 장치를 대체하지 않습니다. 복귀 경로는 자동 충돌 회피 경로가 아닙니다.

## 관제 측 연결

관제 PC에서는 `arm1_node.py`의 `Arm1ActionClient`를 사용하거나 동일 Action 규격으로 연결할 수 있습니다. `send_goal()`이 반환한 Future의 **최종 Result**를 기다린 뒤 다음 공정으로 넘어가세요. `success=true`를 받았는데도 같은 Goal을 재전송하면 서버는 새 요청으로 처리합니다.

운영 로그는 `integration/arm1/logs/`에 생성되며 Git에 포함되지 않습니다. 모델·완료 기준·코드 변경은 서버 재시작 후 적용됩니다.

## 모델 다운로드

[운영 모델 A 15만 / B 25만 스텝 Release](https://github.com/7s-FA/so_101arm_imitation/releases/tag/models-20261011)에서 다음 파일을 받으세요.

- `model_A_150000.tar.gz`
- `model_B_250000.tar.gz`
- `SHA256SUMS.txt`

각 압축에는 가중치·모델 설정·전처리/후처리 정규화 파일이 들어 있습니다. A의 학습 ROI도 포함합니다. 학습 데이터·시작 자세·보정값·완료 판정 참조와 기존 모방학습 런타임은 별도입니다. 기존 로봇 환경에서 압축을 풀어 사용하세요. 압축 내부는 `model-map.json`과 같은 `projects/.../pretrained_model` 경로이며 동일 경로가 있으면 덮어씁니다. `git clone`만으로 모델 첨부파일이 다운로드되지는 않습니다.

```bash
gh release download models-20261011 --repo 7s-FA/so_101arm_imitation --dir model-downloads
cd model-downloads
sha256sum -c SHA256SUMS.txt
tar -xzf model_A_150000.tar.gz -C ~/Downloads/so101_imitation
tar -xzf model_B_250000.tar.gz -C ~/Downloads/so101_imitation
```
