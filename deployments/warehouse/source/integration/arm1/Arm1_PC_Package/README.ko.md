# Robot arm 1 관제 PC 전달 파일

이 폴더는 관제 PC에서 **Robot arm 1 서버에 명령을 보내는 클라이언트**입니다. 학습 모델과 로봇 드라이버는 로봇 PC에 있으며 여기서 설치하지 않습니다. 팔2·팔3·터틀봇 파일을 덮어쓰지 마세요.

## 합의된 작업

- `/arm1/data`, `host_pkg/action/Arm`, ROS 2 Jazzy / Python 3.12 / 도메인 40.
- `SELECT_A`: A 한 세트(보라색 계열 3개) 운송. 여러 세트 연속 운송 아님.
- `SELECT_B`: B 한 세트(주황색 2개 + 파란 원통) 운송. 여러 세트 연속 운송 아님.
- 현재 A 모델은 modelA_final 100만, B 모델은 model_B2_121episodes (121개 에피소드·20만 스텝, 2026-10-08 교체). 모델 변경은 로봇 PC에서 함. 관제는 기존 SELECT_B 명령을 그대로 사용하며 모델 가중치를 받을 필요가 없음.
- 세 부품 완료 및 시작 자세 복귀 후 `success=true, message="IDLE"` 반환. 실패·취소는 false/ERROR.
- `EMER_STOP`: 현재 자세 정지 확인 후 제어 요청 성공 반환. 원래 운송 결과는 아직 대기 중.
- `RESTART`: 정지된 단계부터 재개. 처음부터 새 세트를 처리하는 명령이 아님.
- 팔1은 RESET 명령 미지원. 완료 후 다음 운송은 재료 배치를 복원하고 새 SELECT_A/B로 요청.

## ROS 환경

같은 LAN과 ROS 도메인 40을 사용합니다. 기존 정상 관제 작업공간의 setup을 먼저 적용하세요.

```bash
source /opt/ros/jazzy/setup.bash
# 여기에 기존 관제 작업공간의 install/setup.bash를 source
export ROS_DOMAIN_ID=40
unset ROS_LOCALHOST_ONLY
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
python3 -B interface_check.py
```

`host_pkg/action/Arm` 필드가 `interfaces/Arm.action`과 같아야 합니다. 기존 host_pkg가 있으면 다른 인터페이스를 유지하고 Arm만 추가/검증하세요. 기존 패키지를 Arm 전용 패키지로 통째로 바꾸면 안 됩니다. 이번 패키지는 기존 관제 환경을 자동 수정하지 않습니다.

## Goal 없는 연결 확인

로봇 PC에서 관제 서버를 시작한 뒤, 이 폴더에서:

```bash
python3 -B client.py --check-connection --hardware
```

## 수동 통합 시험

아래 SELECT 명령은 실제 로봇을 움직입니다. 관제 담당자와 로봇 담당자가 시작 배치를 확인한 뒤 실행하세요.

```bash
python3 -B client.py SELECT_A --hardware
python3 -B client.py SELECT_B --hardware
```

진행 중 비상정지·재개는 다른 터미널에서:

```bash
python3 -B client.py EMER_STOP --hardware
python3 -B client.py RESTART --hardware
```

클라이언트 Ctrl+C 또는 시간 초과는 서버 작업을 정지시키지 않습니다. 명시적으로 EMER_STOP을 보내야 합니다. 운영 요청에는 --hardware가 필요하며 생략하면 localhost 모의 도메인 141을 사용합니다.

## 기존 통합 main.py 연결

`arm1_node.py`를 기존 관제 프로그램이 import할 수 있는 위치에 두세요. 기존 같은 이름의 파일이 있으면 차이를 확인한 뒤 병합합니다.

```python
from arm1_node import Arm1ActionClient
arm1 = Arm1ActionClient()
executor.add_node(arm1)
future = arm1.send_goal("SELECT_A")

def finished(future):
    result = future.result()
    if result.success and result.message == "IDLE":
        # 로봇팔 완료·복귀가 확인됨. 여기서 다음 공정으로 진행.
        pass
future.add_done_callback(finished)
```

executor가 계속 돌아야 Feedback/Result가 수신됩니다. timer 콜백 안에서 input()이나 while/sleep으로 결과를 기다리지 마세요. 일반 작업 Goal 수락은 운송 완료가 아닙니다.

기존 `shared_data.shared_memory`가 있으면 다음 필드도 갱신합니다.

- 일반 작업: `arm1_end_flag`, `arm1_success`, `arm1_end_message`, `arm1_feedback_msg`
- 제어 요청: `arm1_control_end_flag`, `arm1_control_success`, `arm1_control_end_message`, `arm1_control_feedback_msg`

정지·재개 성공을 운송 완료로 처리하지 마세요. 제어 결과와 작업 결과는 분리되어 있습니다.

Feedback은 IDLE/RUNNING/PAUSED/ERROR입니다. 엑셀의 팔1 항목에 없던 RUNNING/PAUSED도 화면에서 처리해 주세요. Result.message는 IDLE 또는 ERROR입니다.

상태 조회: `ros2 topic echo /arm1/status` (JSON 문자열). 새 클라이언트의 로컬 busy 값만으로 다른 PC에서 시작한 작업이 없다고 판단하지 마세요. 서버 자체가 중복 일반 작업을 거절합니다.

## 검증 범위

로봇 PC에서 두 모델 오프라인 로드와 모의 제어 테스트, 실제 ROS localhost Action 왕복을 검증했습니다. 다른 관제 PC와의 네트워크 통신·실제 로봇 동작은 현장에서 확인해야 합니다. 전체 웹/터틀봇 공정 자동화는 이 파일에 포함되지 않습니다.
