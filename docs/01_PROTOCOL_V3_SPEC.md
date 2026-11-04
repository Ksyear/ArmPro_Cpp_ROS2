# ArmPi Protocol v3 사양 (Protocol v2 확장)

작성 2026-11-02. 상태: **구현 기준 문서**(이 문서와 코드가 다르면 문서를 먼저 고친 뒤 코드를 맞춘다).
기반: 이전 프로젝트의 Protocol v2 규격.
v2에서 바뀌지 않은 규칙은 v2 문서를 그대로 따른다. 이 문서는 **v3 전체 규칙**을 다시 적되, v2와 다른 곳에 **[v3 변경]** 을 붙인다.

## 1. 전송

| 항목 | 값 |
|---|---|
| 조종 | UDP/IPv4, 로봇 포트 **5001**, 데이터그램 1개 = UTF-8 JSON 객체 1개, 최대 2048 byte |
| 응답 | 같은 소켓으로 **패킷을 보낸 주소·포트**에 ACK |
| 송신 주기 | 클라이언트는 채널(차체·팔)마다 평균 20 Hz(50 ms). 입력을 놓거나 포커스를 잃으면 즉시 1회 추가 |
| 검색 **[v3 변경]** | UDP 5002 브로드캐스트 `ARMPI_DISCOVER/1|<32 hex nonce>` → 응답 `ARMPI_HERE/1|<nonce>|<32 hex id>|<URL 인코딩 이름>|5001|3` (응답 출발 포트 5002, 초당 최대 20회) |
| 보안 | 암호화·인증 없음. **[v3 변경]** 로봇 설정 `allowed_sources`(CIDR 목록, 기본 빈 목록 = 모두 허용)에 없는 주소는 응답 없이 버린다 |

## 2. 메시지

공통: 엄격한 파서. 필드가 빠지거나, 더 있거나, 키가 중복되거나, 타입이 다르면 `invalid_packet`. 숫자 자리에 불리언·NaN·Infinity·문자열 불가. 정수 자리에 실수 불가(`2.0`도 불가).

### 2.1 `command` — 차체 (정확히 8필드) **[v3 변경: strafe 추가]**

```json
{"protocol":3,"type":"command","client_id":"d8b27f10-09a8-4c21-b649-1e7058d46b72","sequence":123,
 "control_active":true,"forward":0.6,"strafe":0.0,"turn":-0.3}
```

| 필드 | JSON 타입 | 허용값 | 의미 |
|---|---|---|---|
| protocol | 정수 | 3 | 다른 정수 → `unsupported_protocol` |
| type | 문자열 | `command` | |
| client_id | 문자열 | 하이픈 포함 36자 UUID(8-4-4-4-12 hex), nil UUID 불가, 대소문자 무시 비교 | 세션 식별 |
| sequence | 정수 | 0 – 9,007,199,254,740,991 | 명령·팔 명령·이벤트 공용 증가 번호 |
| control_active | 불리언 | | false면 세 축을 무시하고 목표 0 |
| forward | 숫자 | −1 – 1 유한값 | 단위 없음. + 앞 |
| strafe | 숫자 | −1 – 1 유한값 | 단위 없음. + **왼쪽**(REP‑103) |
| turn | 숫자 | −1 – 1 유한값 | 단위 없음. + 좌회전(반시계, 위에서 볼 때) |

차체 중립 = `control_active:false`, 또는 `control_active:true`이면서 세 축 모두 0.

### 2.2 `arm_command` — 팔·그리퍼 (정확히 10필드) **[v3 신규]**

```json
{"protocol":3,"type":"arm_command","client_id":"…","sequence":124,
 "control_active":true,"x":0.1047,"y":0.0,"z":0.0753,"pitch":-1.0263,"gripper":0.0}
```

| 필드 | JSON 타입 | 허용값 | 의미 |
|---|---|---|---|
| control_active | 불리언 | | false = 팔 중립(현재 목표 유지, 새 목표 무시) |
| x, y, z | 숫자 | 유한값, 설정 `arm.workspace`(m) 안 | `armpi_arm_base` 좌표계(REP‑103: x 앞, y 왼쪽, z 위)에서 손끝(그리퍼 끝점) **절대** 목표 위치, 미터 |
| pitch | 숫자 | −π – π 유한값 | 손끝 기울기, rad. 0 = 수평으로 앞을 향함, −π/2 = 아래를 향함 |
| gripper | 숫자 | 0 – 1 유한값 | 0 = 완전히 열림, 1 = 완전히 닫힘 |

- 팔 중립 = `control_active:false`.
- `armpi_arm_base` 원점과 IK 판정은 로봇 쪽 기구학 라이브러리가 정한다(링크 길이: `ros2_ws/src/armpi_bringup/config/armpi.yaml`, 관절 기준·범위: 하드웨어 프로필 `hardware_profile.yaml`).
- 로봇은 목표를 **즉시 점프시키지 않는다.** 목표까지 설정 속도(`arm.max_linear_speed_m_s`, `arm.max_pitch_rate_rad_s`, `arm.max_gripper_rate_per_s`)로 보간한다.
- 작업 공간 밖 → `workspace_limit`, IK 해 없음 → `ik_unreachable`. 둘 다 거부 시 목표를 바꾸지 않는다(이전 목표 유지). **거부도 팔 watchdog을 갱신하지 않는다.**

### 2.3 `event` (공통 4필드)

`{"protocol":3,"type":"emergency_stop"|"clear_emergency"|"disconnect","client_id":"…","sequence":N}`

### 2.4 `ack` (로봇 → 클라이언트, 정확히 8필드, 값은 문자열·불리언·정수만)

```json
{"protocol":3,"type":"ack","client_id":"…","sequence":124,"accepted":true,"reason":"ok","emergency_stop":false,"state":"active"}
```

- `state`는 **그 패킷이 속한 채널**의 상태다. `command`·`event` → 차체 채널, `arm_command` → 팔 채널. **[v3 변경]**
- 식별 불가 패킷의 ACK: `client_id:null`, `sequence:-1`.

| state | 의미 |
|---|---|
| `awaiting_neutral` | 시작·새 조종자·비상정지 해제·timeout 뒤. 해당 채널의 중립 패킷 대기 |
| `idle` | 중립 확인, 비활성 |
| `active` | 활성 명령 적용 중 |
| `timeout` | 해당 채널 watchdog 만료(차체 정지 / 팔 목표 고정). 새 중립 필요 |
| `emergency_stop` | latch |
| `fault` | 하드웨어 출력 실패 등. 재시작 필요 |

| reason | 의미 |
|---|---|
| `ok` | 규칙상 수락 (실제 움직임 성공을 뜻하지 않음) |
| `invalid_packet` | JSON·필드·타입·범위 오류 |
| `unsupported_protocol` | protocol ≠ 3 |
| `out_of_order` | 이 세션의 이전 번호 이하 |
| `not_owner` | 다른 조종자 소유, retire된 세션, 또는 로봇 제어기가 다른 경로에 제어권을 준 상태 |
| `emergency_stop` | 비상정지 중 활성 명령 |
| `neutral_required` | 중립 확인 전 활성 명령 |
| `fault` | fault 상태의 활성 명령 |
| `workspace_limit` **[v3 신규]** | 팔 목표가 작업 공간 상자 밖 |
| `ik_unreachable` **[v3 신규]** | 팔 목표에 IK 해가 없음 |

## 3. 상태 규칙

- 세션(소유권·sequence·비상정지·retire)은 **하나**이고, 채널 상태(arming·watchdog)는 **차체/팔 따로** 둔다. **[v3 변경]**
- 패킷을 적용하기 **전에** 두 채널의 watchdog과 lease 만료를 검사한다.
- 채널 watchdog은 그 채널에서 새로 **인정된** 패킷(중립 포함)만 갱신한다. 오류·중복·역순·비소유자·거부(workspace/IK)·이벤트는 갱신하지 않는다. 로봇 monotonic 시계.
  - 차체 watchdog 만료 → 차체 즉시 정지, 채널 `timeout`.
  - 팔 watchdog 만료 → 팔은 **현재 보간 위치에서 정지(자세 유지, 토크 유지)**, 채널 `timeout`. **토크 해제 금지**(팔이 떨어짐).
- lease: 세션 전체에 하나. 어느 채널이든 인정된 패킷이 lease를 갱신. 만료 시 두 채널 모두 정지·유지하고 소유권 해제.
- 단일 조종자 = UUID + 송신 주소(IP·포트). lease 동안 다른 UUID·주소는 `not_owner`.
- lease가 만료된 같은 UUID는 마지막 번호보다 큰 번호로 재획득 가능. 다른 UUID가 인수하면 이전 UUID는 retire.
- `disconnect`한 UUID는 프로세스 실행 동안 거부(최대 1,024개).
- 비상정지 latch는 조종자가 바뀌어도 유지. 비소유자 유효 클라이언트의 `emergency_stop`도 latch. `clear_emergency`는 소유자만, 해제 후 **두 채널 모두** 새 중립 필요.
- 비상정지 시: 차체 즉시 정지, 팔은 현재 위치 유지(토크 유지), 그리퍼 현재 위치 유지.
- 로봇 제어기(armpi_control)가 비상정지·fault를 보고하면 세션도 그 상태를 따른다.

## 4. 시간 값 (로봇 설정, 기본값)

| 값 | 기본 | 근거 |
|---|---|---|
| 차체 watchdog | 0.4 s | v2 기본값 |
| 팔 watchdog | 0.4 s | 차체와 같게 시작 |
| lease | 1.0 s | v2 기본값(클라이언트 재시도 1.1 s와 맞물림) |
| 제어기 2차 watchdog | 0.6 s | v2 기본값. 수신 노드가 죽어도 정지 |
| 클라이언트 ACK timeout | 0.30 s | v2 클라이언트 기본값 |

## 5. 물리량 변환 (로봇 설정 `config/armpi.yaml`, **초기 보수값 — 실측 후 조정**)

| 단계 | 식 | 기본 상한 |
|---|---|---|
| 앞뒤 | `vx = forward × chassis.max_vx_m_s` | 0.25 m/s |
| 좌우 | `vy = strafe × chassis.max_vy_m_s` | 0.25 m/s |
| 회전 | `wz = turn × chassis.max_wz_rad_s` (+ 좌회전) | 1.2 rad/s |
| ROS 2 | `/armpi/cmd_vel` `geometry_msgs/TwistStamped`, frame `base_link` | |
| 팔 보간 | 손끝 직선 속도 | 0.15 m/s |

기본 상한은 사용자 결정 2026-11-04, 바퀴 방향 실기 시험 전에 올렸다(이전 0.10 m/s·0.10 m/s·0.60 rad/s, 팔 0.10 m/s).
| | pitch 변화율 | 1.0 rad/s |
| | 그리퍼 0→1 | 1.0 /s |

## 6. 실시간이 아닌 통로 (rosbridge 9090, HTTP 8080)

| 이름 | 타입 | 방향 | 용도 |
|---|---|---|---|
| `/armpi/status` | std_msgs/String (JSON) | 로봇 → | 2 Hz + 변화 시. 세션·채널 상태, 비상정지, 배터리 V, 팔 목표/현재, `position_source` |
| `/joint_states` | sensor_msgs/JointState | 로봇 → | 20 Hz |
| `/armpi/battery` | sensor_msgs/BatteryState | 로봇 → | 1 Hz |
| `/armpi/emergency_stop` | std_srvs/Trigger | → 로봇 | 전체 비상정지 latch |
| `/armpi/reset_estop` | std_srvs/Trigger | → 로봇 | 해제 |
| `/armpi/arm/home` | std_srvs/Trigger | → 로봇 | 팔 기본 자세(비상정지·조종 중엔 거부) |
| `/armpi/arm/joint_debug` | sensor_msgs/JointState | → 로봇 | 디버그용 관절 직접 지정(조종 세션이 없을 때만) |
| 카메라 | HTTP MJPEG | 로봇 → | `http://<IP>:8080/stream?topic=/usb_cam/image_raw&type=ros_compressed&qos_profile=sensor_data` |
