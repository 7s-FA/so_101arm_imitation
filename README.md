# SO-101 팔1 · 자재창고 제어

상위 호스트의 A/B 제품 선택 명령을 받아, 자재창고에서 해당 재료를 운반 로봇에 공급하는 팔1 제어 코드입니다.

## 동작 흐름

호스트 제품 선택 → 재료 집기·놓기 → 완료 확인 → 결과 보고

| 항목 | 내용 |
|---|---|
| 실행 장치 | 자재창고 Jetson·SO-101 팔1 |
| 호스트 연결 | ROS 2 Action `/arm1/data` |
| 대표 명령 | `SELECT_A`, `SELECT_B` |
| 상태 보고 | `/arm1/status`, `/arm1/vision` |

## 현재 소스 위치

- [deployments/warehouse/source](deployments/warehouse/source): 실제 Jetson에서 수집한 실행 소스입니다.
- `integration/arm1`: 장치 스냅샷 내부의 호스트 연결 서버와 실행 제어입니다.
- `ik`, `arm3-current`: 장치 스냅샷 내부의 IK·보조 실행 코드입니다.
- 루트 프로젝트는 기존 개발 코드입니다. **현재 장치 복원 기준은 deployments의 소스**입니다.

[전체 통신 표](https://github.com/7s-FA/host_web/blob/host_total/README.md#통신-프로토콜) · [소스 출처와 제외 항목](HOST_TOTAL.md)

학습 가중치·모션 계획·운영 JSON·보정값·3D 형상·현장 검사 이미지는 별도 현장 데이터로 유지합니다. 소스만 내려받아 바로 실물 작업을 실행할 수 있는 구성은 아닙니다.
