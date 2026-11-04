# armpi_bringup — ArmPi Pro 실행 묶음 (ROS 2 Humble)

작성 2026-11-02. 같은 날 만든 C++ 패키지 `armpi_hardware`, `armpi_control`과 함께 쓴다.
공통 설정은 `config/armpi.yaml`, 로봇마다 다르거나 제조사 공개 자료로 확인되지 않은 장치 값은 **하드웨어 프로필**(4.1절)에서 넣는다. 토픽 규칙은 `docs/02_ROS2_INTERFACES.md`, 시간값·물리량은 `docs/01_PROTOCOL_V3_SPEC.md`를 따른다.

> **검증 범위**: 아래 시험은 모두 **개발 컨테이너의 가짜 하드웨어**에서 통과한 것이다. 실제 로봇에서 바퀴·서보·GPIO를 움직여 본 적은 **없다**. 실기 확인 목록(맨 아래)을 끝내기 전에는 실제 동작을 주장하지 않는다.

## 1. 패키지

| 패키지 | 내용 |
|---|---|
| `armpi_hardware` | 드라이버 라이브러리(노드 없음). I2C 0x34 바퀴 모터 보드, 확장보드(배터리 mV·팬), 반이중 버스 서보(`/dev/ttyS0` 115200 + libgpiod 방향 핀 2개), GPIO(부저, 버튼 2개). 확장보드 주소·레지스터와 핀 번호는 하드웨어 프로필에서 받는다. 장치마다 인터페이스 + 실제 구현 + 가짜 구현. 메카넘·cmd_vel·서보값·그리퍼 변환 순수 함수 |
| `armpi_control` | 노드 `armpi_control_node`. 비상정지 latch, 2차 watchdog, cmd_vel → 메카넘 바퀴, ArmTarget 보간 → IK(`armpi_kinematics`) → 서보, 상태 발행 |
| `armpi_bringup` | `config/armpi.yaml`(모든 노드 공통 매개변수), `config/hardware_profile.template.yaml`(하드웨어 프로필 본보기), `config/test_hardware_profile.yaml`(시험용 값), `launch/armpi.launch.py`(실기), `launch/armpi_mock.launch.py`(가짜 하드웨어), `test/`(시험) |

## 2. 빌드

```bash
sudo apt install libgpiod-dev          # 실행만 할 때는 libgpiod2. I2C 는 커널 헤더만 쓰므로 libi2c 불필요
cd ros2_ws && colcon build --merge-install --packages-up-to armpi_bringup armpi_vr_bridge
```

libgpiod 가 없으면 빌드는 되지만 경고가 나오고, 실제 GPIO(서보 방향 전환·부저·버튼)를 쓸 수 없어 실기 모드에서 fault 로 시작한다.

## 3. 실행

```bash
# 실기: 제어기 + UDP 수신기(5001/5002) + usb_cam + web_video_server(8080) + rosbridge(9090)
ros2 launch armpi_bringup armpi.launch.py
# 가짜 하드웨어(Mac/Docker, 카메라 없음): 제어기(mock) + UDP 수신기 + rosbridge
ros2 launch armpi_bringup armpi_mock.launch.py
# 제어기만
ros2 run armpi_control armpi_control_node --ros-args --params-file <armpi.yaml> -p hardware.mock:=true
```

`armpi.launch.py` 인자: `params_file`, `hardware_profile`(4.1절), `mock`, `vr_bridge`, `camera`, `video_device`(/dev/video0), `pixel_format`(yuyv2rgb), `image_width`/`image_height`(640/480), `framerate`(15.0), `web_video`, `web_video_port`(8080), `rosbridge`, `rosbridge_port`(9090). 설치되지 않은 선택 패키지는 경고만 남기고 건너뛴다.
카메라 스트림: `http://<IP>:8080/stream?topic=/usb_cam/image_raw&type=ros_compressed`

## 4. 매개변수 파일 `config/armpi.yaml`

| 구역 | 내용 |
|---|---|
| `/**` | **두 노드가 반드시 같은 값을 써야 하는 것**: `chassis.max_*`(속도 상한), `arm.workspace.*`(작업 공간 상자), `arm.kinematics.*`(기구학). 한 곳에만 둔다 |
| `armpi_control` | 서보 포트, 팬·부저·버튼 사용 여부, 차체 가속·watchdog, 팔 보간·home 손끝 목표, 발행 주기 |
| `armpi_vr_bridge` | `armpi_vr_bridge/config/vr_bridge.yaml`과 같은 키(2026-11-02 21:47 판). 그 파일이 바뀌면 이 구역도 맞춰야 한다 |

주의
- ROS 2 우선순위: **노드 이름 구역에 적힌 값이 명령행 `-p`나 launch 의 dict 매개변수보다 우선**한다. 그래서 `hardware.mock`은 파일에 두지 않고 launch 인자 `mock:=`로만 정한다. 다른 값을 명령행으로 바꾸려면 파일을 복사해 고친 뒤 `params_file:=`로 넘긴다.
- 실수에는 소수점, 정수에는 소수점 없이(자료형이 엄격).
- 작업 공간 상자 근거는 파일 주석에 있다(x ≥ 0.05 → 직선 보간 중 joint1 각속도 ≤ 2 rad/s, z ≥ −0.08 등). 상자 안이어도 IK 해가 없으면 거부된다.

### 4.1 하드웨어 프로필

아래 값은 코드와 `armpi.yaml`에 기본값을 두지 않는다. 로봇마다 다를 수 있거나 제조사 공개 자료로 확인되지 않은 값이기 때문이다.

| 키 | 내용 |
|---|---|
| `chassis.a_mm`, `chassis.b_mm`, `chassis.wheel_diameter_mm`, `chassis.pulse_per_cycle` | 차체 중심 ↔ 바퀴 거리, 바퀴 지름, 바퀴 1회전당 엔코더 펄스 |
| `arm.servo_ids`, `arm.joint_directions` | 관절 joint1..joint5·r_joint 를 움직이는 서보 ID(1..6 한 번씩)와 방향(±1) |
| `arm.home_servo_values`, `arm.gripper.*` | 켤 때 보낼 서보값(ID1..6), 그리퍼 열림·닫힘 값과 허용 범위 |
| `arm.kinematics.j1..j4_sign/offset`, `joint_min/max` | 기구학 관절 기준과 관절 범위(rad). `/**` 구역(제어기·VR 수신기 공통) |
| `hardware.expansion_board_address`, `battery_register`, `dc_motor_register`, `dc_motor_sign`, `fan.motor_index`, `fan.speed` | 확장보드(배터리·팬) |
| `hardware.servo_rx_con_bcm`, `servo_tx_con_bcm`, `buzzer.bcm`, `buttons.key1_bcm`, `key2_bcm` | GPIO 핀(BCM 번호) |
| `servo.pre_write_delay_us`, `post_write_delay_us`, `read_turnaround_delay_us`, `read_timeout_ms`, `read_attempts` | 버스 서보 반이중 타이밍 |

- **만드는 법**: `config/hardware_profile.template.yaml`을 `config/hardware_profile.yaml`로 복사하고 빈 값(`null`)을 직접 측정하거나 제조사 자료로 채운다. 채운 뒤 `colcon build`(설치 폴더로 복사됨). 다른 위치의 파일은 `hardware_profile:=<경로>`로 넘긴다.
- **실기 모드**에서 값이 하나라도 비어 있으면 제어 노드는 장치를 열지 않고 `fault`("하드웨어 프로필 값 없음 …: 빠진 키")로 시작한다. 바퀴·서보에 아무것도 보내지 않는다. 프로필 파일이 없으면 launch 가 경고를 남긴다.
- **가짜 하드웨어**(`mock:=true`, `armpi_mock.launch.py`)는 `config/test_hardware_profile.yaml`(시험용 값, 실제 로봇 값 아님)을 쓰고, 빈 칸은 같은 시험값으로 채운 뒤 경고를 남긴다. 시험값을 실제 로봇에 쓰면 팔·바퀴가 엉뚱하게 움직일 수 있다.
- 서보 진단 도구 `tools/servo_read_diag.py`도 같은 파일에서 핀·타이밍을 읽는다(`--hardware-profile`).

## 5. 토픽·서비스 (`armpi_control`)

| 이름 | 타입 | 방향·QoS | 내용 |
|---|---|---|---|
| `/armpi/cmd_vel` | `geometry_msgs/TwistStamped` | 구독, reliable depth 1 | stamp 가 0.5 s 넘게 차이 나면 버림. 상한으로 자름 |
| `/armpi/arm/target` | `armpi_interfaces/ArmTarget` | 구독, reliable depth 1 | 손끝 절대 목표. 작업 공간·IK 다시 확인 |
| `/armpi/estop_request` | `std_msgs/String` JSON | 구독 | `{"source":"udp","latch":true/false}`. 해제는 그 출처만(서비스·버튼 출처는 토픽으로 못 품) |
| `/armpi/arm/joint_debug` | `sensor_msgs/JointState` | 구독 | `joint1..5`, `r_joint`(rad). UDP 조종자(owner) 없을 때만 |
| `/armpi_vr_bridge/state` | `std_msgs/String` JSON | 구독, transient_local | owner·채널 상태를 `/armpi/status`에 넣고 joint_debug·home 허용 판단에 씀(2 s 넘으면 없음으로 봄) |
| `/armpi/control_state` | `std_msgs/String` JSON | 발행, transient_local, 2 Hz + 변화 | `{"estop","estop_sources","fault","fault_reason","udp_allowed"}`. home·joint_debug 이동 중에는 `udp_allowed:false` |
| `/armpi/status` | `std_msgs/String` JSON | 발행, 2 Hz + 변화 | Mac 조종기·가짜 로봇과 같은 키(`owner, owner_addr, base_state, arm_state, emergency_stop, estop_sources, fault, fault_reason, udp_allowed, battery_v, position_source, arm_target, arm_current, arm_moving, base_cmd`) + `arm_mode, arm_initialized, arm_ik_blocked, wheel_pulses, cmd_vel{…}, battery_low, buttons_ok, fan_ok …` |
| `/armpi/battery` | `sensor_msgs/BatteryState` | 1 Hz | voltage(V). percentage 는 6.6–8.4 V 선형 대략값. 측정 안 하는 값은 NaN |
| `/joint_states` | `sensor_msgs/JointState` | 20 Hz | `joint1..joint5, r_joint`, 서보값에서 환산(home 전에는 발행 안 함) |
| `/armpi/shutdown_requested` | `std_msgs/Empty` | 발행 | KEY2 2 s 누름마다 1회. **실제 종료는 시스템 스크립트가 구독해서 한다** |
| `/armpi_control/hw_state` | `std_msgs/String` JSON | 10 Hz | 진단용: 마지막 바퀴 펄스, 서보 명령값, 쓰기·실패 횟수(가짜면 가짜 서보 위치·토크도) |
| `/armpi_control/mock_cmd` | `std_msgs/String` | 구독(가짜만) | `gpio <key1_bcm> 0`(KEY1 누름), `gpio <key1_bcm> 1`, `gpio <key2_bcm> 0`(KEY2), `fail i2c 1`, `fail servo 1`, `battery 7000` |
| `/armpi/emergency_stop`, `/armpi/reset_estop`, `/armpi/arm/home` | `std_srvs/Trigger` | 서비스 | 비상정지 latch(출처 service) / 모든 출처 해제 / 기본 자세(비상정지·fault·팔 채널 active 중 거부) |

## 6. 동작 규칙

- **차체 50 Hz**: cmd_vel → 상한 → 축별 가속 제한(0.6 m/s², 4 rad/s²) → `direction = atan2(vx, −vy)`, `velocity = hypot·1000`, `angular = angular_sign·wz` → 메카넘 식 → 모터 1–4 펄스/10 ms(레지스터 51). **정지·감속은 항상 즉시**, 부호 반전은 0 을 거친다. 0.6 s 동안 cmd_vel 이 없으면 정지. 값이 같아도 0.5 s 마다 다시 씀.
- **팔 50 Hz**: 목표까지 손끝 0.15 m/s, pitch 1.0 rad/s, 그리퍼 1.0/s 로 보간 → IK → 관절 속도 제한(2 rad/s) → joint1–4 서보(배정은 프로필 `arm.servo_ids`)와 그리퍼에 `MOVE_TIME_WRITE`(40 ms, 바뀐 서보만). 손목 회전(joint5) 500 고정. IK 실패 → 직전 유효 자세 유지 + 경고. ArmTarget 0.6 s 없음 / `active:false` / 비상정지 → 현재 명령 자세 유지, **새 명령 안 보냄, 토크 해제 안 함**.
- **시작**: 바퀴 0 → 팬 켬(프로필 `fan.motor_index`·`fan.speed`) → `arm.home_on_start`면 서보 ID1..6 = `arm.home_servo_values` (1500 ms) → 보간 상태를 그 자세의 FK로 초기화(`armpi.yaml` `arm.home` 손끝 목표 x≈0.105, z≈0.075 m, pitch≈−58.8° 와 같게 맞춘다). home 을 끄면 `/armpi/arm/home` 전까지 팔 목표를 무시한다(위치를 모르므로).
- **비상정지**: 출처별 latch(`service`, `udp`, `button` 등). 걸리면 바퀴 즉시 0, 팔 유지, 부저 0.1 s. `/armpi/reset_estop`이 모두 해제, UDP `latch:false`는 udp 만. 해제 뒤에는 새 명령이 와야 움직인다.
- **버튼**: KEY1 짧게 누름 → 비상정지(출처 button). KEY2 2 s → `/armpi/shutdown_requested`. 시작할 때 이미 눌려(0) 있으면 한 번 떼질 때까지 무시(선로 고장 대비).
- **fault**: 하드웨어 초기화 실패, I2C 바퀴 쓰기 또는 서보 UART 쓰기 5회 연속 실패, 설정 오류 → `fault:true`(재시작 필요), 바퀴 정지 시도(0.5 s 마다), 활성 명령 무시. 노드는 죽지 않고 상태를 계속 발행한다.
- **위치 피드백**: 기본 `position_source:"commanded"`(명령한 서보값). `servo.read_position:=true`면 POS_READ 를 한 틱에 서보 1개씩(서보마다 5 Hz) 읽어 6개 모두 신선하면 `"measured"`, 아니면 commanded 로 되돌림. **실기 미검증이라 기본 끔.**

## 7. 시험

```bash
ros2_ws/src/armpi_bringup/test/dev_build_and_test.sh [unit|integration|all]
```

개발 컨테이너(`armpi-ctl-dev:humble`, Ubuntu 22.04 arm64, `--network none`)에서 빌드 → gtest → 설정 검사 → 통합 시험(노드 기본값, `armpi.yaml`) → 실기 모드 시작 확인(장치 없음 → fault) → `armpi_mock.launch.py` 종단 시험(UDP v3 클라이언트 → 수신기 → 제어기 → 가짜 바퀴·서보, rosbridge) → `armpi.launch.py mock:=true` 8 s 실행(포트 8080/9090/5001/5002). 결과는 `work/logs/armpi_control_<모드>_<시각>/`.

| 시험 | 파일 | 내용 |
|---|---|---|
| 변환 | `armpi_hardware/test/test_conversions.cpp` | 메카넘·cmd_vel→극좌표·서보값↔관절각·그리퍼(식을 시험 안에서 따로 구현해 비교) |
| 서보 패킷 | `armpi_hardware/test/test_servo_protocol.cpp` | MOVE_TIME_WRITE·읽기·STOP·LOAD 바이트열·체크섬, 응답 해석과 오류 |
| 장치 | `armpi_hardware/test/test_devices.cpp` | I2C 바이트열(0x34 레지스터 20/51, 확장보드 배터리·DC 모터는 시험용 주소·레지스터), 서보 반이중 순서(TX/RX 핀·쓰기·drain·flush·읽기), 재시도, 가짜 서보 선형 이동 |
| 제어 | `armpi_control/test/test_*.cpp` | 비상정지 latch, watchdog, 버튼, 가속 제한·즉시 정지, 보간·관절 속도 제한, home, 유지(새 명령 없음), IK·작업 공간 거부, IK 막힘, joint_debug, fault, 피드백 |
| 통합 | `armpi_control/test/integration/test_control_integration.py` | 노드(mock) 16단계 |
| 종단 | `armpi_bringup/test/test_mock_launch_e2e.py` | launch + UDP 클라이언트 |


## 10. 실기에서 반드시 확인할 것 (바퀴를 띄우고·팔 주변을 비우고·비상정지 손에 두고)

1. **바퀴 방향**: 각 모터 1–4 를 작은 펄스로 하나씩 → 어느 바퀴가 어느 쪽으로 도는지. 앞으로(vx>0) 네 바퀴가 같은 방향으로 앞으로 가는지.
2. **회전 부호**: `wz>0`이 위에서 볼 때 반시계(좌회전)인지. 아니면 `chassis.angular_sign: -1.0`.
3. **옆걸음**: `vy>0`이 왼쪽인지.
4. **I2C 0x34 쓰기 실패 시 모터 보드가 마지막 속도를 유지하는지**(제어기가 멈추면 계속 굴러갈 수 있음) → 별도 하드웨어 차단 필요 여부.
5. **서보 반이중 타이밍**: 쓰기만 하는 기본 모드에서 6개 서보가 모두 반응하는지. POS_READ 를 켜면 응답이 오는지(프로필 `servo.*` 타이밍), 끈 상태와 비교.
6. **관절 배정·방향·영점**: 프로필 `arm.servo_ids`·`arm.joint_directions`·`arm.kinematics.*`와 기본 자세(`arm.home_servo_values`)가 예상한 모양인지.
7. **그리퍼 끝점**: 프로필 `arm.gripper.open_value`·`close_value`가 기계 한계를 누르지 않는지, 물체 없이 닫을 때 과부하·발열.
8. **작업 공간**: z −0.08 이 바닥보다 위인지, 옆(y ±0.25)·아래에서 차체·바퀴·카메라와 부딪히지 않는지. 실제 손끝 위치와 FK 비교.
9. **팬** 동작, **배터리 mV**(실측 7358 mV 근처), 부저, KEY1·KEY2 의 실제 수준(누르면 0)과 풀업.
10. **비상정지**: 서비스·UDP·KEY1 각각에서 바퀴가 즉시 서는지, 팔이 처지지 않는지(토크 유지).
11. 기본 자세 이동 중(1.5 s) 비상정지를 걸면 서보는 그 이동을 끝까지 한다(중간 정지 명령을 보내지 않음). 허용 가능한지 판단.
