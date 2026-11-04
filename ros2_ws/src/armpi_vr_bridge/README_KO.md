# armpi_vr_bridge — ArmPi Protocol v3 UDP 수신기 (C++ / ROS 2 Humble)

작성 2026-11-02. 구현 기준: `docs/01_PROTOCOL_V3_SPEC.md`, `docs/02_ROS2_INTERFACES.md`.

**증거 수준**: 코드 + 빌드 + 컨테이너(Ubuntu 22.04 arm64, ROS 2 Humble) 안의 루프백 단위·통합 시험까지.
실제 ArmPi Pro 로봇, 실제 Wi-Fi, Mac 조종기·VR 기기와의 상호 운용은 **시험하지 않았다**.
팔 기구학 치수와 작업 공간은 **placeholder** 다.

## 1. 구조

```
Mac·VR 조종기 ──UDP 5001 (JSON v3)──▶ UdpServer ─┬─▶ Session (세션 1개 + 차체/팔 채널)
           ◀── ACK (같은 소켓) ─────────────────┘        │
           ──UDP 5002 ARMPI_DISCOVER/1──▶ DiscoveryServer ─▶ ARMPI_HERE/1 (출발 포트 5002)
                                                          ▼
VrBridgeNode (rclcpp) ─ /armpi/cmd_vel, /armpi/arm/target, /armpi/estop_request, /armpi_vr_bridge/state
                      ◀ /armpi/control_state (armpi_control 의 게이트)
```

| 파일 | 내용 | ROS 의존 |
|---|---|---|
| `include/.../strict_json.hpp`, `src/strict_json.cpp` | 엄격한 JSON 파서(중복 키·NaN·정수/실수 구분·UTF-8·깊이·크기) | 없음 |
| `protocol.hpp/.cpp` | v3 패킷 파서(`command` 8필드, `arm_command` 10필드, 이벤트 4필드), ACK 직렬화 | 없음 |
| `net.hpp/.cpp` | IPv4·`Endpoint`(UUID 소유자 주소), `allowed_sources` CIDR 필터 | 없음 |
| `session.hpp/.cpp` | 세션 상태기계. 시계는 호출자가 `now` 로 넘긴다(시험에서 가짜 시계) | 없음(`armpi_kinematics` 링크) |
| `discovery.hpp/.cpp` | 검색 요청 검사·응답 문자열·SHA-256 식별자·URL 인코딩·초당 20회 제한 | 없음 |
| `rx_stats.hpp/.cpp` | 수신 주기·거부 사유·유실 추정 통계 | 없음 |
| `udp_server.hpp/.cpp` | POSIX UDP 소켓 + 수신 스레드 + 감시 스레드, 검색 소켓 | 없음 |
| `bridge_node.hpp/.cpp`, `main.cpp` | ROS 2 노드(발행·구독·QoS·매개변수·종료 정지 3회) | rclcpp |
| `config/vr_bridge.yaml` | 기본 매개변수(링크 길이는 공개 값, 관절 기준·범위는 시험용, 작업 공간은 placeholder) | |
| `launch/vr_bridge.launch.py` | 단독 실행용 launch (전체 로봇은 `armpi_bringup` 담당) | |
| `test/test_*.cpp` | gtest 단위 시험 6개 파일 | |
| `test/integration/test_bridge_integration.py` | 노드 실행 + UDP ↔ ROS 토픽 통합 시험(rclpy, pytest) | |

라이브러리 `armpi_vr_bridge_core`(정적)는 ROS 없이 빌드·시험된다. 노드는 이 위에 얇게 붙는다.

### 스레드와 잠금

- **수신 스레드**: `poll`(20 ms) → `recvfrom` → 출처 필터 → 세션 처리(세션 mutex 안) → 콜백(mutex 밖) → ACK.
  패킷이 없어도 매 반복 watchdog·lease·control_state 나이를 검사한다.
- **감시 스레드**: 수신 루프 heartbeat 가 0.5 s 넘게 멈추면 세션을 정지(새 중립 요구)시키고 출력 갱신,
  수신 스레드가 끝나 버렸으면 fault latch(재시작 필요). 콜백을 세션 lock 밖에서 부르므로 수신 스레드가
  콜백 안에서 막혀도 감시 스레드가 세션을 멈출 수 있다(시험 `StallMonitorStopsWhileReceiveThreadIsBlocked`).
- **ROS 실행기 스레드**: 50 Hz 만료 검사 타이머, 20 Hz 발행 타이머, `/armpi/control_state` 구독.
- 출력 발행은 `out_mutex_` 아래에서 "세션 최신 상태 읽기 + 발행"을 함께 해서 늦게 온 옛 목표가 정지를 덮지 못한다.

## 2. 동작 규칙 요약 (자세한 것은 spec)

- 세션 1개(소유자 = UUID + 송신 IP·포트, sequence, lease 1.0 s, retire 최대 1024, 비상정지 latch) +
  채널 2개(차체·팔: 각자 중립 확인, watchdog 0.4 s). sequence 는 `command`·`arm_command`·이벤트 공용.
- 패킷을 적용하기 **전에** 두 채널 watchdog, lease, control_state 나이를 검사한다.
- watchdog 은 그 채널에서 **인정된** 패킷만 갱신한다. 오류·중복·역순·비소유자·게이트 거부·workspace/IK 거부·이벤트는 갱신하지 않는다.
  lease 는 소유자의 새 번호 패킷(어느 채널·이벤트든)이 갱신한다.
- ACK `state` 는 그 패킷의 채널 상태(`command`·이벤트 → 차체, `arm_command` → 팔). 형식 오류 패킷도 `type` 이 `arm_command` 면 팔 채널 상태.
- 차체: `vx = forward×0.10`, `vy = strafe×0.10`(+왼쪽), `wz = turn×0.60`(+좌회전). 중립 = `control_active:false` 또는 세 축 0.
- 팔: 중립 = `control_active:false`(좌표 무시, 현재 위치 유지). 활성 목표가 작업 공간 상자 밖 → `workspace_limit`,
  IK 결과가 `kOk` 아님(`kOutOfReach`·`kJointLimit`·`kInvalid`) → `ik_unreachable`. 거부 시 이전 목표 유지.
- 제어기 게이트(`/armpi/control_state`): 미수신·2.0 s 초과·`udp_allowed:false` → 활성 명령 `not_owner`(중립은 수락),
  게이트가 닫히는 순간 움직이던 채널은 정지(새 중립 요구). `estop:true` → 세션 비상정지 상태(활성 `emergency_stop`),
  `fault:true` → `fault`. 형식이 틀린 control_state(불리언 `estop`·`fault`·`udp_allowed` 누락)는 무시한다(→ 2 s 뒤 차단).
- 비상정지: 소유자·비소유자(유효 UUID, 소유자가 있거나 retire 된 UUID) 모두 latch. 해제는 소유자만, 해제 뒤 두 채널 새 중립.
  유효 비상정지 = UDP latch **또는** 제어기 `estop`. UDP 해제 요청을 보내도 제어기가 `estop:true` 를 보고하는 동안은 비상정지 유지.
- 우선순위(활성 명령 거부 사유): `fault` > `emergency_stop` > `neutral_required` > `not_owner`(게이트) > `workspace_limit` > `ik_unreachable`.

## 3. ROS 인터페이스

| 이름 | 타입 | 방향 | QoS | 내용 |
|---|---|---|---|---|
| `/armpi/cmd_vel` | `geometry_msgs/TwistStamped` | 발행 | reliable, depth 1 | frame `base_link`. 20 Hz + 값 변경 시 즉시. 정지 = 0 |
| `/armpi/arm/target` | `armpi_interfaces/ArmTarget` | 발행 | reliable, depth 1 | frame `armpi_arm_base`. 수락된 `arm_command` 마다 1회 + 팔 active 동안 20 Hz. 비활성 전환 시 `active:false` 1회 |
| `/armpi/estop_request` | `std_msgs/String` | 발행 | reliable, depth 10 | `{"source":"udp","latch":true\|false}` |
| `/armpi_vr_bridge/state` | `std_msgs/String` | 발행 | reliable, transient_local, depth 1 | 2 Hz + 변화 시. `owner`, `owner_addr`, `base_state`, `arm_state`, `emergency_stop`, `packets{…}` + 진단 필드 |
| `/armpi/control_state` | `std_msgs/String` | 구독 | reliable, transient_local, depth 1 | `{"estop","estop_sources","fault","fault_reason","udp_allowed"}` |

종료(SIGINT/SIGTERM): rclcpp 기본 신호 처리기를 끄고 직접 받아, 수신·검색 정지 → `cmd_vel` 0 과
`ArmTarget active:false` 를 10 ms 간격으로 3회 발행 → 200 ms 대기 → rclcpp 종료.

## 4. 매개변수 (노드 `armpi_vr_bridge`, 기본값 = `config/vr_bridge.yaml`)

| 이름 | 기본 | 설명 |
|---|---|---|
| `bind_address` | `0.0.0.0` | 수신 주소 |
| `port` | 5001 | 조종 UDP 포트(ACK 도 같은 소켓) |
| `poll_interval_sec` | 0.02 | 수신 루프 poll 주기 |
| `rx_stall_timeout_sec` | 0.5 | 수신 루프 정지 감시 |
| `base_watchdog_sec` / `arm_watchdog_sec` | 0.4 / 0.4 | 채널 watchdog |
| `lease_sec` | 1.0 | 소유권 lease (두 watchdog 보다 커야 함) |
| `control_state_max_age_sec` | 2.0 | 이보다 오래된 control_state → 활성 명령 `not_owner` |
| `publish_rate_hz` | 20.0 | cmd_vel 주기 발행·팔 재발행 주기 |
| `state_rate_hz` | 2.0 | 상태 토픽 주기 |
| `stats_log_period_sec` | 10.0 | 수신 통계 로그 주기 |
| `accept_estop_from_any_client` | true | 비소유자 비상정지 latch |
| `retired_limit` | 1024 | retire UUID 기억 수 |
| `chassis.max_vx_m_s` / `max_vy_m_s` / `max_wz_rad_s` | 0.10 / 0.10 / 0.60 | 속도 상한(초기 보수값) |
| `arm.workspace.{x_min,x_max,y_min,y_max,z_min,z_max}` | 0.0, 0.30, −0.25, 0.25, −0.05, 0.35 | 작업 공간 상자(m) — **placeholder** |
| `arm.kinematics.{base_height_m, upper_arm_m, forearm_m, hand_m}` | 0.10, 0.10, 0.095, 0.12 | 링크 길이(m) — **placeholder** |
| `arm.kinematics.j{1..4}_sign`, `j{1..4}_offset` | 1.0, 0.0 | 관절 기준 — **placeholder** |
| `arm.kinematics.joint_min` / `joint_max` | [−3.2×4] / [3.2×4] | 관절 한계(rad) — **placeholder** |
| `arm.kinematics.min_wrist_height_m` / `min_elbow_bend_rad` | −1.0e9 / 0.0 | 손목 중심 최소 높이(m)·팔꿈치 최소 굽힘(rad), 라이브러리 기본 = 제한 없음 — **placeholder** |
| `allowed_sources` | (빈 목록 = 모두 허용) | CIDR 문자열 배열. 예 `["192.168.2.0/24", "10.42.0.5"]` |
| `discovery.enabled` / `port` / `name` / `max_replies_per_second` | true / 5002 / ""(hostname) / 20 | 검색 응답 |
| `frame_id` / `arm_frame_id` | `base_link` / `armpi_arm_base` | |
| `cmd_vel_topic`, `arm_target_topic`, `estop_request_topic`, `control_state_topic` | `/armpi/...` | 시험용으로 바꿀 수 있음 |

주의:
- 실수 매개변수에는 소수점을 쓴다(`1` 은 정수라 시작 실패: 확인함).
- Humble YAML 은 `allowed_sources: []` 를 노드 생성 단계에서 거부한다(확인함: "No parameter value set").
  빈 목록은 줄을 **생략**하거나 `[""]`(빈 항목은 무시 = 모두 허용)로 쓴다.
- 잘못된 CIDR, `lease ≤ watchdog`, 작업 공간 `min ≥ max`, 잘못된 기구학, 포트 사용 중이면 노드가 시작하지 않는다(종료 코드 1).
- 호스트 비트가 켜진 CIDR(`192.168.2.5/24`)은 네트워크 주소(`192.168.2.0/24`)로 맞춘다.

## 5. 빌드·시험

```bash
# Mac(Apple Silicon) 에서 colima·docker 로: 빌드 + gtest + 통합 시험
scripts/armpi_dev_build_and_test.sh all        # unit | integration | all
# 결과: work/logs/vr_bridge_<모드>_<시각>/
#   run.log, colcon_build.txt, build_armpi_vr_bridge.log, unit/*.gtest.xml, unit_summary.txt,
#   integration_junit.xml, integration.txt, itest_bridge_*.log(노드 출력), source_sha256.txt
```

스크립트는 `armpi-dev:humble`(= `ros:humble-ros-base-jammy` + procps·iproute2·python3-pytest·psmisc)에서
`--network none` 으로 돌고, 소스를 컨테이너 `/tmp/ws` 로 복사해 빌드하며(exFAT 공유 폴더의 ENOENT 회피),
결과는 `docker cp` 로 꺼낸다. 실제 로봇에 접속하지 않는다.

직접 빌드(로봇 또는 Ubuntu 22.04 + Humble):

```bash
cd ros2_ws && source /opt/ros/humble/setup.bash
colcon build --packages-up-to armpi_vr_bridge
colcon test --packages-select armpi_vr_bridge && colcon test-result --verbose
source install/setup.bash
ros2 launch armpi_vr_bridge vr_bridge.launch.py
# 통합 시험(포트 15001/15002/15011/15012, 127.0.0.1 만 사용)
ROS_LOCALHOST_ONLY=1 python3 -m pytest -q -s src/armpi_vr_bridge/test/integration/test_bridge_integration.py
```

### 시험 범위

- `test_strict_json`(12): 중복 키(모든 깊이·escape 키), 정수/실수 토큰, NaN/Infinity, 잘못된 숫자·구조, UTF-8(overlong·서로게이트·BOM),
  escape·제어 문자, 깊이·크기 제한, 큰 정수·1e400·1e-400(Python 과 같게 0.0).
- `test_protocol`(25): 명세 예시, protocol/type 규칙과 검사 순서, 각 필드 누락·추가, 중복 키, 자료형(불리언≠숫자, `2.0`≠정수),
  축·pitch·gripper 범위, sequence·UUID 규칙, 2048 byte 경계, 거부 ACK 의 식별자 되돌림, 팔 채널 ACK, ACK 정확한 8필드·순서.
- `test_session`(44, 가짜 시계): 중립 요구·속도 변환·strafe 부호, sequence(중복·역순·유실·공용·한계), 차체/팔 watchdog 경계·만료 전 검사·
  거부 패킷 비갱신, 채널 독립, workspace/IK 거부(목표 유지·watchdog 비갱신·관절 한계), lease 만료·같은 UUID 재획득·다른 UUID·새 포트,
  disconnect·retire·1024 제한, 비상정지(양 채널·비소유자·retire·해제·조종자 교체 후 유지), 게이트(미수신·2.0 s·udp_allowed·차단 시 정지·
  게이트 거부 비갱신), 제어기 estop/fault 반영, 내부 fault, force_stop, 설정 검증.
- `test_discovery`(10): SHA-256 표준 벡터, 식별자, machine-id 읽기, URL 인코딩, 이름 규칙, 응답 형식·길이, 잘못된 요청, 초당 20회.
- `test_source_filter`(7): IPv4 엄격 파싱, CIDR 일치·/0·/32·호스트 비트, 잘못된 항목 예외, 빈 목록.
- `test_udp_server`(12, 실제 루프백): ACK 왕복·출발 포트, 2048 초과·빈 데이터그램, 쓰레기 2000개 뒤 생존, 포트 중복 실패,
  출처 필터(차단·허용), 수신 루프 watchdog 지연(0.38~0.60 s), 수신 스레드 정지 감시, estop 콜백, 종료 정지, 검색 왕복·필터.
- 통합(10): 게이트 전 not_owner, 차체 변환·20 Hz·watchdog 지연, 팔 목표·거부 규칙·20 Hz 재발행·팔 watchdog·비활성 후 재발행 없음,
  비상정지 왕복(estop_request·가짜 제어기 latch·비소유자·해제), 제어기 fault·udp_allowed, control_state 2 s 초과, 검색 응답,
  상태 토픽 transient_local, allowed_sources(두 번째 노드), SIGINT 뒤 정지 3회.

### 교차 확인: Mac 쪽 독립 적합성 시험

별도로 작성한 `protocol_v3/conformance_test.py`(이 패키지 코드를 쓰지 않고 원시 JSON 으로 시험)를
컨테이너 안에서 이 노드(+ 가짜 control_state 발행기)에 대해 읽기 전용으로 실행했다(2026-11-02).

| 조건 | 결과 |
|---|---|
| normal | 통과 44 / 실패 0 / 건너뜀 9 (해석 시험 5개 + 다른 조건 전용 4개) |
| normal + `--interpretation` | 통과 49 / 실패 0 / 건너뜀 4 (fault·source_blocked·udp_blocked 조건 전용, rosbridge 필요) |
| control_missing (제어기 없음) | 통과 26 / 실패 0 / 건너뜀 27 |

기록: `work/logs/vr_bridge_conformance_20261102_220004/`. 같은 루프백 컨테이너 시험이며 실제 네트워크·로봇 시험이 아니다.

## 6. JSON 파서 선택 이유

`nlohmann-json3-dev` 대신 작은 자체 파서(`strict_json.cpp`, 약 400줄)를 썼다.

1. **정수/실수 구분이 어휘 단계에서 필요**: `sequence: 2.0`, `protocol: 3.0` 을 거부해야 한다. nlohmann 도 `2.0` 을 실수로 읽지만,
   64비트 범위를 넘는 정수는 실수로 바꿔 버려 "아주 큰 정수 protocol → `unsupported_protocol`"(v2 동작)과 구분하기 어렵다.
2. **중복 키를 모든 깊이에서 거부**: nlohmann 은 마지막 값으로 덮어쓴다. 콜백으로 깊이별 키 집합을 관리할 수는 있지만
   자체 파서에서는 `std::set` 한 줄로 끝난다.
3. **Python json 과 같은 결과**: `1e-400` → 0.0 수락, `1e400` → 유한하지 않음 거부, BOM·제어 문자 거부, 짝 없는 서로게이트 escape 허용 등
   `json.loads` 의 경계 동작을 시험으로 고정했다.
4. **의존성 없음**: 로봇 이미지에 apt 패키지를 더 넣지 않아도 되고 정적 라이브러리 하나로 끝난다.

## 8. 남은 문제·미구현

- 이 패키지 기본 yaml(`config/vr_bridge.yaml`)의 관절 기준·범위는 **시험용** 값이다. 로봇에서는 `armpi_bringup` 의 `config/armpi.yaml`(링크 길이)과 하드웨어 프로필(관절 기준·범위)에서 넣는다.
  현재 기본 팔 길이로는 명세 예시 `(0.15, 0, 0.12, −π/2)` 가 IK 범위 밖(`ik_unreachable`)이다.
- 실제 로봇·Wi-Fi 지연·유실은 시험하지 않았다. Mac 쪽 적합성 시험은 루프백에서만 통과했고, Mac 조종기 앱 자체·VR 클라이언트와의 실제 연결은 시험하지 않았다.
- `armpi_control` 이 아직 없어 `/armpi/control_state` 는 시험용 가짜 발행기로만 확인했다.
- 지연됐지만 번호가 증가한 패킷 판별 없음(v3 에도 송신 시각 없음), 재시작 시 retire 목록 소실, 암호화·인증 없음(v2 와 같음).
- `allowed_sources` 는 IPv4 출처 주소만 본다(위조 가능). 신뢰할 수 있는 LAN 전용.
