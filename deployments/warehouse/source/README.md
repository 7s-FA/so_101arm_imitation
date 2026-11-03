# Robot arm 1 — Jetson 관제 서버 (IK 보정판, 2026-10-27)

이 폴더 전체를 USB로 젯슨에 옮겨 실행합니다. 젯슨이 **로봇 PC 역할 전체**를 맡습니다. 로봇팔, D435, 손목 카메라를 젯슨에 연결하고, 관제 서버(`/arm1/data`, ROS 도메인 40)를 젯슨에서 실행합니다.

## 10/08 패키지와 달라진 점

- **동작:** 관제 명령이 오면 ACT 추론 대신 실물에서 검증한 동작 계획 4개를 실행합니다(보라 아래 ep199, 보라 위 ep184, 주황 왼쪽 v9, 주황 오른쪽 ep257).
- **목적지 팔레트(버거) 보정:** 작업마다 D435로 위치를 측정합니다(차루코 보드로 카메라 이동 확인).
  - m2 기준 위치면 원래 계획을 씁니다.
  - m1이면 m1 검증 계획을 씁니다.
  - 그 외 위치면 부품별로 IK를 다시 계산한 계획을 그 자리에서 만듭니다(PC 약 12초).
- **세트 선택:** D435 색 판정으로 고릅니다. 보라 캡 칸 판정 영역 수정본입니다.
- **ACT 모델:** 서버가 시작할 때 A `model_A_newmask_300k` 15만, B `model_B2_300k_20261010` 25만을 불러옵니다(서버 구조상 필요). 그래서 CUDA torch 환경이 여전히 필요합니다.
- **새로 필요한 라이브러리:** IK 계산용 `mujoco 3.3.7`, `scipy 1.18.1`. ARM64 설치 파일을 `wheels/`에 넣어 두어 인터넷 없이 설치됩니다.

## 1. 옮기기

```bash
mkdir -p ~/robot3_ws
tar -xzf ~/Downloads/arm1_jetson_IK_20261027.tar.gz -C ~/robot3_ws
cd ~/robot3_ws/arm1_jetson_IK_20261027
bash jetson-run.sh verify
```

기존 `arm1_jetson_AB_20261008` 폴더는 지우지 않습니다(가상환경을 재사용합니다).

## 2. 설치 (10/08 환경이 동작할 때)

```bash
bash install-jetson.sh --ik-only
```

- 10/08 패키지가 쓰던 가상환경(`~/robot3_ws/arm1_jetson_*/.jetson-python`)을 자동으로 찾아 씁니다. 다른 곳이면 `--python /실제/venv/bin/python`을 붙입니다.
- 그 환경에 `mujoco`, `scipy`만 `wheels/`에서 오프라인으로 추가합니다. torch와 RealSense는 건드리지 않습니다.
- 이 폴더의 ROS 인터페이스(host_pkg)를 준비하고 `doctor`로 점검합니다.

처음부터 새로 설치해야 하면 10/08과 같은 방식인 `bash install-jetson.sh --system`을 쓰세요(인터넷 필요).

## 3. 장치 없이 확인

```bash
bash jetson-run.sh doctor --require-cuda
bash jetson-run.sh ik-test
bash jetson-run.sh check-models --device cuda
bash jetson-run.sh benchmark --device cuda
```

- `ik-test`는 저장된 m1 사진으로 팔레트를 측정하고, 저장된 실물 도킹 값으로 IK 보정 계획을 만듭니다. PC 결과와 같으면 통과입니다. 걸린 시간도 출력합니다(서버 제한 90초).
- 네 명령 모두 모터와 카메라를 열지 않습니다.

## 4. 관제 서버 실행

```bash
bash run-arm1-server.sh
```

- PC와 같은 명령입니다. 켜지는 순간부터 관제 명령에 따라 실제 팔이 움직일 수 있습니다.
- **PC 서버와 젯슨 서버를 동시에 켜지 마세요.** 같은 이름(`/arm1/data`, 도메인 40)이라 관제 명령이 섞입니다.
- 서버를 끄면 토크가 풀립니다. 끄기 전에 팔을 받쳐 주세요.

## 5. 모니터 GUI

GUI는 PC에서 그대로 실행합니다(같은 네트워크):

```bash
cd /home/jaywjd/Downloads/so101_imitation && bash arm1-control.sh gui --hardware
```

젯슨이 만든 IK 보정 계획 파일은 PC에 없으므로, 그 작업의 "실행 중 경로" 선만 표시되지 않습니다. 나머지(카메라, 측정, 가상 셀, 상태)는 그대로 보입니다.

## 전제 조건

- 같은 로봇팔, 같은 D435(시리얼 920312070850), 같은 손목 카메라, 같은 D435 고정 위치와 차루코 보드여야 합니다. 카메라가 움직이면 측정이 거부되고 작업이 ERROR로 끝납니다. 이때는 차루코 카메라 보정을 다시 해야 합니다(이 패키지 범위 밖).
- 장치 경로는 `arm3-current/arm3/devices.json`의 by-id 이름입니다. `doctor`가 젯슨에서 보이는지 알려 줍니다.
- 젯슨 실물 검증(카메라·모터 USB, 30Hz 제어 주기, IK 계산 시간, 관제 연결)은 아직 하지 않았습니다. `PACKAGING_VALIDATION.md`에 PC에서 한 검증만 적었습니다.

## 10/08 수동 실행 (참고)

`run-modelA.sh`, `run-modelB.sh`는 ACT 모델 수동 테스트입니다. 관제 서버의 계획 실행과는 별개입니다.
