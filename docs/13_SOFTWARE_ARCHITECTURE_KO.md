# 13. 소프트웨어 구조 (v2, 2026-11-04)

로봇에서 도는 우리 코드를 모두 C++ 로 옮긴 판(v2)의 구조다. 폴더, 패키지 의존 관계, 실제로 적용한 설계, 파이썬 → C++ 대응을 정리한다.

증거 수준: **코드·빌드·모의 시험**(컨테이너, 가짜 하드웨어, 루프백)까지다. v2 이미지는 아직 빌드하지 않았고, 실제 로봇에서 v2 를 돌려 보지 않았다.
v1 이미지(2026-11-02)의 실기 B절 결과는 `11_KNOWN_LIMITATIONS_KO.md` 00절에 있다.

## 1. 폴더 구조

```
ArmPro_Cpp_ROS2/
├── README.md
├── ros2_ws/src/            로봇 소프트웨어(ROS 2 Humble 패키지, C++)
├── clients/                로봇 밖 조종기
│   ├── mac_controller/     Mac 조종기(파이썬, Mac 에서만), 아이콘 생성기, HTTP API 확인 시험
│   ├── mac_app/            Mac 앱(Swift 래퍼)
│   ├── android_app/        Android 앱
│   └── vr_handoff/         VR(Unity) 인계용 C# 예제와 자체 시험
├── protocol_v3/            Protocol v3 파이썬 참조 클라이언트·가짜 로봇·적합성 시험
├── deploy/
│   └── systemd/            로봇 서비스 유닛
├── scripts/                개발 스크립트(빌드·시험)
├── tools/                  실기 점검 도구(첫 부팅 점검, 서보 읽기 진단)
├── docs/                   문서
```

git 밖(무시): `output/`(앱 산출물), `work/`(빌드·시험 로그).

이미지 빌드 스크립트와 로봇 이미지는 공개본에 넣지 않았다.

ROS 2 패키지는 표준 구조를 따른다: `package.xml`(format 3), `CMakeLists.txt`, `include/<패키지>/`, `src/`, `test/`, 필요하면 `config/`·`launch/`·`www/`.

## 2. 패키지와 의존 관계

| 패키지 | 종류 | 하는 일 |
|---|---|---|
| `armpi_common` | 라이브러리(ROS 없음) | 엄격한 JSON 해석·작성(중복 키·정수/실수 구분·UTF-8 검사), 고정 소수 출력 |
| `armpi_protocol` | 라이브러리(ROS 없음) | Protocol v3 형식(패킷 해석·ACK 작성), 클라이언트 쪽 패킷 작성·ACK 해석, 클라이언트 세션 `V3Client`, UDP 연결 |
| `armpi_kinematics` | 라이브러리 | 팔 IK/FK |
| `armpi_hardware` | 라이브러리 | 바퀴 보드·확장 보드(I2C), 버스 서보(반이중 UART), GPIO. 실제/가짜 구현 |
| `armpi_interfaces` | 메시지 | `ArmTarget.msg` |
| `armpi_control` | 노드 `armpi_control_node` | 비상정지 latch, watchdog, 차체·팔 제어, 상태 발행, 부팅 점검 |
| `armpi_vr_bridge` | 노드 `armpi_vr_bridge_node` | Protocol v3 UDP 수신기(5001)·검색 응답(5002) |
| `armpi_web_gateway` | 노드 `armpi_web_gateway_node` | 로봇 안 웹 조종기(HTTP 18765). 조종은 v3 UDP 로만 |
| `armpi_power` | 노드 `armpi_power_node` | KEY2 종료 요청 → 시스템 종료 |
| `armpi_bringup` | launch·설정 | `armpi.launch.py`, `armpi.yaml`, `fastdds_armpi.xml` |

```
armpi_common ◀── armpi_protocol ◀── armpi_vr_bridge ──▶ armpi_kinematics, armpi_interfaces
      ▲                 ▲
      │                 └──────── armpi_web_gateway
      └── armpi_control ──▶ armpi_hardware, armpi_kinematics, armpi_interfaces
armpi_power (rclcpp, std_msgs 만)
```

## 3. 로봇에서 도는 프로그램

| 서비스 | v1 (2026-11-02 이미지) | v2 (이 저장소) |
|---|---|---|
| `armpi.service` | `ros2 launch` → 제어기·수신기(C++), usb_cam·web_video_server(C++), rosbridge(파이썬) | 같음. Fast DDS 설정 파일을 launch 가 적용 |
| `armpi-web.service` | 파이썬 `armpi_mac_controller.py`(+ rosbridge 로 상태) | **C++ `armpi_web_gateway_node`**(rclcpp 로 상태) |
| `armpi-shutdown.service` | `ros2 topic echo --once`(파이썬 CLI, 약 110 MB) + `systemctl poweroff` | **C++ `armpi_power_node`** |

- 영상: 게이트웨이가 화면에 알려 주는 주소는 `…&type=ros_compressed&qos_profile=sensor_data`(`camera_qos_profile`, 기본 `sensor_data`). 원시 영상은 반드시 `qos_profile=sensor_data`(best-effort)로 받는다. 기본(RELIABLE)으로 받으면(스냅샷·`type=mjpeg` 포함) 로봇 안 ROS 통신이 막힐 수 있다(2026-11-04 실기, `docs/11` 0절).
- `armpi.service`는 시작 전에 `fastdds shm clean`(쓰이지 않는 공유 메모리 흔적 정리)을 한다. 소프트웨어를 바꾼 뒤에는 서비스 재시작보다 재부팅을 권장한다.

로봇 안에 남는 파이썬은 ROS 2 표준 부품뿐이다: `ros2 launch`(시작 관리), rosbridge·rosapi(Mac 조종기·VR 이 쓰는 9090). 이것들은 바꾸면 통신 규격이 달라지므로 그대로 둔다.
v1 에서 잰 메모리(참고, 섞인 부팅 상태): 파이썬 웹 조종기 20 MB, 종료 대기 57 MB + ros2 데몬 53 MB, C++ 노드 각 22–23 MB. v2 의 메모리는 아직 재지 않았다.

## 4. 파이썬 → C++ 대응

| 파이썬 | C++ | 같게 맞춘 규칙 |
|---|---|---|
| `protocol_v3/armpi_v3_client.py` `parse_ack` | `armpi_protocol::parse_ack` (`client_codec.cpp`) | 정확히 8필드, 실수 금지, 중복 키 금지, 문자열 `[A-Za-z0-9_-]` |
| 같은 파일 `ArmPiV3Client` | `armpi_protocol::V3Client` (`client.cpp`) | 채널별 중립 확인 → 무장, 강한 차단 뒤 손을 떼야 재무장, ACK 0.30 s, 비상정지 재전송 50 ms, 팔 목표 거부 시 마지막 수락값으로 |
| `clients/mac_controller/armpi_mac_controller.py` `Controller` | `armpi_web_gateway::Controller` | 화면 토큰 조종권 1.5 s, on-demand 세션, 입력 0.3 s 끊기면 중립, 20 Hz, 비상정지는 어느 화면에서나, 조작 없음 60 s 뒤 놓기 |
| 같은 파일 `make_handler` | `armpi_web_gateway::WebApi` | 경로, Host 검사, Origin·Content-Type 검사, 본문 8 KiB, 응답 코드(409·400·403·503) |
| 같은 파일 `RosBridge`(rosbridge WebSocket) | `armpi_web_gateway::RosRobotLink`(rclcpp) | `/armpi/status`(2 s 신선도)·`/armpi/battery`, Trigger 서비스, joint_debug |
| 입력 변환 `float()`·`int()`·`bool()` | `armpi_web_gateway::pyconv` (`input.cpp`) | 숫자 문자열 허용, 정수 버림, 빈 값은 거짓 |
| `systemd/armpi-shutdown.service` 의 셸 명령 | `armpi_power_node` | 첫 요청 한 번만, 셸을 거치지 않고 `systemctl poweroff` |

`/api/state`·`/api/config` 의 JSON 키는 파이썬 판과 같다(화면 파일을 그대로 쓰기 위해). 상태 출처가 rclcpp 로 바뀌었지만 화면 호환을 위해 키 이름 `rosbridge` 는 유지했다. `/api/config` 에는 `backend` 키 하나가 더 있다. 두 판 모두 `/api/state` 의 `session.reason`(none·always·user·estop_only·closing)과 `/api/config` 의 `ui.default_speed`(휴대폰 화면 속도 버튼 기본값)를 낸다(2026-11-04 추가).

웹 화면 파일(`index.html`, `mobile.html`, 아이콘, manifest)의 기준 위치는 `ros2_ws/src/armpi_web_gateway/www/` 한 곳이다. Mac 조종기는 저장소에서는 그 폴더를 읽고, Mac 앱 빌드는 그 파일을 번들에 복사한다.

## 5. 실제로 적용한 설계

| 설계 | 쓴 곳 | 왜 |
|---|---|---|
| 인터페이스 + 실제/가짜 구현(Strategy) | `armpi_hardware`: `MotorBoard`·`ExpansionBoard`·`ServoBus`·`Gpio` ↔ `I2cMotorBoard`·`HalfDuplexServoBus`… / `Mock*` | 장치 없이 제어 논리를 시험 |
| 생성 함수(Factory) | `armpi_hardware::open_real_hardware`, `make_mock_hardware` | 제어 노드는 어느 쪽인지 몰라도 된다 |
| 송수신 인터페이스(Strategy) | `armpi_protocol::DatagramLink` ↔ `UdpLink` / 시험용 가짜 | 클라이언트 상태 기계를 소켓 없이, 시계도 바꿔 끼워 결정적으로 시험 |
| 어댑터(Adapter) | `armpi_web_gateway::RobotLink` ↔ `RosRobotLink`(rclcpp) / 시험용 가짜 | 웹 조종기 핵심을 ROS 없이 시험, rosbridge 자리를 rclcpp 로 |
| 명령 표(Command) | `Controller::register_commands`: 명령 이름 → {조종권 필요 여부, 처리 함수} | 조종권 규칙을 한 곳에서. 새 명령 추가가 쉬움 |
| 실행 방법 교체(Strategy) | `armpi_power::PowerAction` ↔ `CommandPowerAction` / `DryRunPowerAction` | 실제로 끄지 않고 종단 시험 |
| 상태 기계(열거형·플래그) | `armpi_protocol::V3Client` 채널, `armpi_vr_bridge::Session` | 프로토콜 상태 전이를 한 클래스에 모음(GoF State 클래스 분리는 하지 않음) |
| 계층 분리 | `HttpServer`(소켓) / `WebApi`(경로·보안) / `Controller`(정책) | 각 층을 따로 시험 |
| 조립 지점(Composition root) | 각 노드 `main.cpp` | 객체 생성·연결을 한 곳에서 |
| ROS 없는 핵심 라이브러리 | `armpi_common`, `armpi_protocol`, `*_core` | 단위 시험이 빠르고 ROS 버전에 덜 묶임 |
| 책임별 파일 분리 | `armpi_control/src/control_node*.cpp` 5개(준비·안전·입력·주기·상태) | 837줄 한 파일을 나눔. 클래스·동작은 같음 |
| 패턴 재생기(우선순위 있는 상태 기계) | `armpi_control::BuzzerSequencer`(v2.1): 시작 알림(1·2·3번)·비상정지 부저 | 시간 계산을 ROS·GPIO 없이 시험. 비상정지가 시작 알림을 덮어씀 |

중복 제거: JSON 구현 두 벌(제어기용·수신기용)을 `armpi_common::json` 하나로 합쳤다. 제어기는 `lenient_topic_options()`(64 KiB, 중복 키는 첫 값, 유한하지 않은 숫자 거부)로 예전 동작을 유지한다. 출력 숫자 형식(`fixed(x, 4)`)도 같다.

## 6. 시험

| 명령 | 내용 | 마지막 결과(2026-11-04) |
|---|---|---|
| `scripts/test_all.sh` | 전체 패키지 빌드 + gtest | 229개(v1 179 + v2 새 50) 실패 0, 컴파일 경고 0. v2.1(14시): 234개(부저 5 추가) 실패 0 |
| `ros2_ws/src/armpi_bringup/test/dev_build_and_test.sh all` | 하드웨어·제어기 gtest, 제어기 통합 16×2, 실기 모드 시작, 종단 3, **게이트웨이 종단 6**, **파이썬 조종기용 HTTP API 확인을 C++ 게이트웨이에(46/46, 카메라 제외)**, **KEY2 → 종료 dry-run 2** | 모두 통과 |
| `scripts/armpi_dev_build_and_test.sh all` | 수신기 gtest 73 + 통합 10 | 통과 |
| `scripts/selftest_mac.sh` | Mac 조종기 + 파이썬 가짜 로봇: 적합성, HTTP API 47 | 47/47 |
| `clients/vr_handoff/selftest/run_selftest.sh` | C# 예제 | 67/67 |

로그는 (시험 기록) 아래(실행마다 새 폴더).


## 7. 남은 한계

- v2 이미지 빌드·실기 미실시. 특히 Fast DDS 설정(`fastdds_armpi.xml`)·소켓 버퍼(sysctl)가 카메라 문제를 푸는지는 **실기에서 확인해야 한다**(컨테이너에서는 카메라가 없고, 합성 영상 921,600 B·15 Hz 로는 기본 설정에서도 문제가 재현되지 않았다).
- `HttpServer` 는 연결마다 요청 하나를 처리하는 작은 서버다(Keep-Alive·HTTPS 없음, 동시 연결 32). 인증이 없다(v1 과 같음).
- `/api/state` 의 `rosbridge.connected` 는 v2 에서 "최근 3 s 안에 `/armpi/status` 를 받았는가"다(v1 은 WebSocket 연결 여부).
- 웹 조종기는 비상정지 직후 채널 상태가 다음 ACK(최대 50 ms 뒤)부터 `emergency_stop` 이 된다. 파이썬 판과 같은 규칙이다.
