# Robot arm 1 관제 연결

2026-10-07 사용자 결정: 명령 한 번은 선택한 종류의 **한 세트, 부품 3개 운송**입니다. 여러 세트를 연속 실행하지 않습니다.

| 명령 | 현재 연결 모델 |
|---|---|
| `SELECT_A` | `model_A/models/model_A_newmask_300k`, 230개 에피소드·15만 스텝 (2026-10-11 최종 모델 지정) |
| `SELECT_B` | `model_B2/models/model_B2_300k_20261010`, 260개 에피소드·25만 스텝 (2026-10-10 교체) |
| `EMER_STOP` | 현재 자세로 정지하고 토크 유지; 정지 확인 뒤 요청 성공 보고 |
| `RESTART` | 정지한 작업 단계에서 이어서 진행; 새 작업을 시작하는 `r`과 다름 |

기존 `run-modelA.sh`, `run-modelB2.sh`는 그대로 사용합니다. 관제용 서버는 별도 실행기이며 수동 실행·녹화와 동시에 장치를 사용하면 안 됩니다.

2026-10-11: A의 30만 목표 학습을 중단하고 저장된 15만 체크포인트를 최종 운용 모델로 채택했습니다. `run-modelA.sh`와 관제 `SELECT_A` 모두 `model_A_newmask_300k/checkpoints/150000/pretrained_model`을 사용합니다. 체크포인트의 새 overview ROI `[171, 5, 637, 397]`와 손목 전체 영상 설정을 사용하며, 완료 판정과 완료 후 시작 자세 복귀는 유지합니다. 실행 중인 서버에는 재시작 후 적용됩니다.

완료 기준(2026-10-08 복원): A는 기존 성공 장면의 모양·색상 비교 기준, B는 주황색 2개·파란 원통 1개의 기존 크기·형태와 지그 내부 95% 포함 기준으로 복원했습니다. 그리퍼 이탈과 새로운 영상 1초 유지, 시작 자세 복귀 확인 후 관제 성공 전송은 유지합니다. 수동·관제 모두 적용되며 실행 중인 프로그램은 종료 후 다시 실행해야 합니다.

## 현재 상태와 다음 단계

구현 완료: `/arm1/data` Action 서버, A/B 선택, 완료 확인→시작 자세 복귀→결과 전송, 토크 유지 대기, 정지·재개, 중복 작업 거절, Action 취소, 모델 교체 설정, 관제 PC용 `arm1_node.py`.

검증 완료: 두 모델 CPU 로드·정규화·추론, 제어·오류 처리 모의 테스트 26개, localhost ROS 도메인 141에서 실제 Action 왕복 테스트 5개. 실제 카메라·모터를 사용한 관제 운전은 아직 하지 않았습니다. 별도 관제 PC와 네트워크 연결도 현장 확인 대상입니다.

**다음 할 일:** 관제 담당자에게 `Arm1_PC_Package.zip` 전달 → 동일 LAN/도메인 40에서 연결만 확인 → A 1세트 시험 → 새 B 모델 1세트 시험 → 현장 비상정지·재개·취소 확인 → 전체 공정 연결. 2026-10-08 확인 당시 RealSense가 USB에 인식되지 않았으므로 실제 운전 전 카메라 연결을 확인합니다.

## 설치/오프라인 검사

### 기존 관제가 완성된 경우: 로봇 PC에서만 실행

관제 프로그램이 아래 규격을 이미 사용한다면 관제 PC 파일을 교체하거나 ZIP을 전달할 필요가 없습니다. 로봇 PC에서 실행기를 켜 두세요.

```bash
cd ~/Downloads/so101_imitation
bash run-arm1-server.sh
```

`ARM1 READY mode=HARDWARE domain=40 action=/arm1/data`가 나오면 명령 대기 중입니다. 이 실행기는 기존 서버를 실제 장치 모드로 실행하며 명령을 직접 보내지 않습니다. 관제의 Goal이 들어오면 별도 `start` 입력 없이 동작합니다. 완료와 시작 자세 복귀를 확인한 뒤 Result를 보내고 다음 명령을 기다립니다. 터미널을 열어 둡니다.

관제와 일치해야 하는 규격: ROS 2 Action `/arm1/data`, 타입 `host_pkg/action/Arm`, 도메인 `40`, 명령 `SELECT_A`, `SELECT_B`, `EMER_STOP`, `RESTART`. 다른 도메인을 쓰는 관제에는 `bash run-arm1-server.sh --domain-id 번호`로 맞춥니다. 양쪽 PC의 ROS 통신이 가능한 네트워크가 필요합니다. 완성된 관제 프로그램과의 실제 호환성·원격 연결은 현장 확인 대상입니다.

실행 전 카메라 연결을 확인하고 수동 모델 실행·녹화·미리보기를 종료하세요. 서버 종료 시 토크가 해제되므로 팔을 받친 뒤 Ctrl+C를 누르세요. `SELECT_B`는 현재 `model_B2_121episodes` 20만 모델입니다.

이 PC에는 ROS Jazzy와 colcon이 있고, 이번 작업에서 프로젝트 내부 ROS 인터페이스를 빌드했습니다. `.bashrc`나 터틀봇 작업공간은 변경하지 않았습니다.

```bash
cd ~/Downloads/so101_imitation
bash arm1-control.sh setup
bash arm1-control.sh check-models
bash arm1-control.sh test
bash arm1-control.sh test-ros
```

`setup`은 현재 환경에 호환되는 `host_pkg`가 있으면 사용하고, 없으면 `integration/arm1/ros_ws`에만 빌드합니다. 기존 `host_pkg`가 있지만 Arm 정의가 다르거나 없으면 덮어쓰지 않고 중단합니다. 의존 Python 버전은 ROS Jazzy/Python 3.12입니다.

`check-models`는 CPU 기본값이며 장치를 열지 않습니다. GPU 점검은 `--device cuda`를 추가합니다. `test-ros`는 localhost 도메인 141에서 모의 서버를 띄우고 시험 후 종료합니다. 운영 도메인 40에는 모의 서버를 띄울 수 없습니다.

## 실제 관제 서버 시작 — 이 로봇 PC

카메라 미리보기·수동 실행·녹화를 종료하고 시작 배치와 복귀 경로를 확인한 뒤 실행합니다.

```bash
cd ~/Downloads/so101_imitation
bash arm1-control.sh serve --hardware
```

시작 시 모델만 준비합니다. `ARM1 READY mode=HARDWARE`가 표시되면 관제의 `SELECT_A/B`가 들어왔을 때 실제로 움직입니다. 매번 `start` 입력은 요구하지 않습니다. 명령 수신 전에는 모터에 쓰지 않습니다.

기본 설정: ROS Jazzy / 도메인 40 / Action `/arm1/data` / 타입 `host_pkg/action/Arm`. 상태 토픽 `/arm1/status`는 JSON 문자열이며 `state`, `phase`, `command`, `mock`을 담습니다. 모델·시작 자세·카메라·보정값의 일치 검사를 통과해야 운전합니다.

## 다른 터미널에서 연결 확인·작업 요청

연결만 확인(Goal 전송 없음):

```bash
bash arm1-control.sh check-connection --hardware
```

실제 동작 명령:

```bash
bash arm1-control.sh send SELECT_A --hardware
bash arm1-control.sh send SELECT_B --hardware
```

한 명령이 끝나기 전 다른 일반 작업은 거절합니다. 자동 반복하지 않습니다. 재료 배치를 복원하고 다음 `SELECT_A/B`를 보내면 새 작업을 시작합니다. 목적지가 이미 완료된 것으로 보이면 실패 결과를 보내고 기존 대기 자세를 유지합니다.

정지·재개는 별도 터미널에서:

```bash
bash arm1-control.sh send EMER_STOP --hardware
bash arm1-control.sh send RESTART --hardware
```

정지는 팔을 현재 측정 자세에 고정하고 피드백으로 정지 자세를 확인합니다. 재개 시 오래된 예측을 버리고 새 카메라 영상으로 다시 예측합니다. 시작 자세 이동·복귀 도중 멈췄다면 그 단계부터 재개합니다. 비상정지 중 새 작업은 거절합니다. `RESET`은 팔1 합의 프로토콜에 없어 구현하지 않았습니다.

Action 취소는 정지 후 원래 작업에 실패/취소 결과를 반환하고 토크를 유지합니다. 이때 `RESTART`는 취소된 작업을 복구하지 않고 정지 잠금만 해제하며, 이후 새 `SELECT_A/B`가 필요합니다.

## 결과·종료 의미

정상 작업 순서: `HOMING → WORKING → RETURNING → IDLE`. **재료 3개 완료 판정과 시작 자세 복귀가 모두 확인되어야** `success=true, message="IDLE"`를 보냅니다. 프로세스 종료나 Goal 접수만으로 완료 처리하지 않습니다. 완료 후에도 하드웨어 감시와 토크 유지는 계속됩니다.

Feedback은 팔2·팔3와 같은 `IDLE/RUNNING/PAUSED/ERROR`를 사용합니다. 팔1 엑셀에는 IDLE/ERROR만 적혀 있으므로 관제 담당자는 추가 RUNNING/PAUSED 상태를 표시할 수 있도록 반영해야 합니다. Action 필드 정의는 팀 파일과 동일합니다. 제어 요청의 성공은 원래 운송 작업 완료와 별개입니다.

클라이언트 Ctrl+C/종료는 서버 작업을 멈추지 않습니다. 정지는 `EMER_STOP`을 보내세요. **이 로봇 PC의 서버 Ctrl+C는 종료 정리 과정에서 토크를 해제**하므로 팔을 받쳐야 합니다. 통신 단절이 자동 정지를 뜻하지 않으며 새 관제 클라이언트의 로컬 상태는 이전 작업을 복원하지 않습니다. `/arm1/status`와 로봇 PC 로그로 확인하세요.

소프트웨어 정지는 추론/장치 통신 지연의 영향을 받으며 하드웨어 비상정지 장치를 대체하지 않습니다. 온도·과부하·통신 오류, 카메라 복구 3초 초과, 제어 250ms 초과 등은 오류 처리와 토크 해제로 종료합니다. 일반 작업은 정지 시간을 제외하고 600초 안에 완료·복귀하지 못하면 실패 처리합니다. 복귀 경로는 충돌 회피를 자동 계산하지 않습니다.

오류 상태에서는 새 작업·재개를 거절하므로 원인을 해결한 뒤 서버를 다시 시작합니다. 운전 기록은 `integration/arm1/logs/hardware_*.jsonl`에 남습니다.

## B 모델 교체

2026-10-10 사용자 요청으로 아래 25만 스텝 모델을 적용했습니다. 이후 교체할 때도 서버를 종료한 상태에서 `integration/arm1/model-map.json`의 `models.SELECT_B.model`을 바꾸세요.

```json
"model": "projects/model_B2/models/model_B2_300k_20261010/checkpoints/250000/pretrained_model"
```

경로가 실제로 생긴 뒤 변경해야 합니다. 같은 작업·카메라·시작 자세를 사용하는 경우 `project`와 `completion`은 유지합니다. `check-models` 통과 후 서버를 재실행합니다. 가동 중 설정 변경은 적용되지 않습니다. A/B 가중치·학습 데이터는 이 관제 구현에서 수정하지 않았습니다.

## 관제 담당자에게 전달

`Arm1_PC_Package.zip`에는 `arm1_node.py`, 단독 CLI, 인터페이스 정의 및 설명서가 있습니다. 기존 관제의 `from arm1_node import Arm1ActionClient` 연결에 사용합니다. 다른 팔·터틀봇 파일을 교체하지 마세요.

기존 전체 관제 코드의 `input()`/대기 루프를 ROS 콜백 안에서 실행하거나, 명령 접수를 완료로 취급하는 부분은 관제 측에서 정리해야 합니다. 전달 클라이언트의 `send_goal()`은 최종 Result를 담는 Future를 반환합니다. 웹·터틀봇 전체 공정 자동화는 이 팔1 서버가 담당하지 않습니다.


완료 후 복귀 속도 변경(2026-10-08): A/B 수동 테스트와 관제의 RETURNING 단계만 모터 속도를 56 encoder steps/s(약 4.92도/초)로 제한합니다. 앞서 설명한 10도/초 예시의 절반인 약 5도/초 기준이며, 기존에는 속도 제한값이 0이므로 실측한 기존 속도의 절반이라는 뜻은 아닙니다. 그리퍼도 모터 원시 단위로 같은 제한을 적용합니다. 최초 시작 자세 이동·학습 동작·30Hz 제어 주기는 유지합니다. 복귀 제한 시간은 120초이며 도착 확인 후 완료/토크 유지 순서는 같습니다. 변경 적용은 프로그램 재실행 후부터입니다. 실제 팔 동작 검증은 아직 하지 않았습니다.

2026-10-10: PC 관제 SELECT_B를 model_B2_300k_20261010/checkpoints/150000으로 교체. arm1-control.sh의 serve/check-models는 커널 580.173.02일 때만 프로젝트 내 동일 버전 CUDA 라이브러리를 적용합니다. 기존 Jetson USB 패키지는 10/08의 20만 모델 묶음이므로 이번 PC 교체와 구분합니다. 실행 중 서버에는 재시작 후 적용됩니다.

2026-10-10 최종 B 선택: 사용자 요청으로 SELECT_B를 model_B2_300k_20261010/checkpoints/250000/pretrained_model로 교체. 같은 프로젝트와 완료 기준을 유지하며 서버 재시작 후 적용. 로봇팔_실행_명령어.md의 B 수동 테스트도 이 25만 모델로 일치시킴.



