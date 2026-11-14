# ArmPro_Cpp_ROS2

로봇 팔과 메카넘 바퀴 차체를 가진 ArmPi Pro(Raspberry Pi 4B)를 **Ubuntu 22.04 + ROS 2 Humble**에서 돌리기 위한 C++ 소프트웨어다.
VR·휴대폰·PC 조종기가 같은 UDP 규격(Protocol v3)으로 로봇을 조종하게 하고, 안전 규칙(조종권, 중립 확인, watchdog, 비상정지)을 로봇 쪽에서 강제한다.

> 대상 하드웨어: Hiwonder ArmPi Pro. 이 저장소는 Hiwonder와 관계없는 독립 프로젝트다. "Hiwonder", "ArmPi" 등은 각 소유자의 상표이며 대상 하드웨어를 가리키는 데만 쓴다. <!-- trademark-notice -->

## 무엇이 들어 있나

| 위치 | 내용 |
|---|---|
| `ros2_ws/src/armpi_control` | 제어 노드: 차체(메카넘)·팔(IK·속도 제한)·그리퍼·부저·버튼·팬, 비상정지 latch, 시작 부저 |
| `ros2_ws/src/armpi_hardware` | 장치 계층: I2C 모터 보드, 확장 보드(배터리·팬), 반이중 버스 서보, GPIO. 실제/가짜 구현을 바꿔 끼운다 |
| `ros2_ws/src/armpi_kinematics` | 4자유도 팔 기구학(FK/IK, 관절 범위) |
| `ros2_ws/src/armpi_vr_bridge` | Protocol v3 UDP 수신기(5001)와 로봇 찾기(5002), 세션·watchdog 규칙 |
| `ros2_ws/src/armpi_protocol` | Protocol v3 메시지 인코딩·해석, C++ 클라이언트 |
| `ros2_ws/src/armpi_web_gateway` | 로봇 안 웹 조종기(HTTP 18765, 휴대폰 화면) |
| `ros2_ws/src/armpi_power` | KEY2 종료 요청 처리 |
| `ros2_ws/src/armpi_common` | 엄격한 JSON 파서 등 공용 코드 |
| `ros2_ws/src/armpi_interfaces` | 메시지 정의 |
| `ros2_ws/src/armpi_bringup` | launch, 설정(`config/armpi.yaml`), 하드웨어 프로필 본보기(`config/hardware_profile.template.yaml`)·시험용 프로필, Fast DDS 설정, 통합·종단 시험 |
| `clients/` | Mac 조종기(파이썬)·Mac 앱(Swift)·Android 앱·Unity용 C# 예제 |
| `protocol_v3/` | 파이썬 참조 클라이언트, 가짜 로봇, 규격 적합성 시험 |
| `deploy/` | systemd 유닛, sysctl(DDS 버퍼), udev 규칙, logind(SSH 종료 때 공유 메모리 유지) |
| `tools/` | 첫 부팅 읽기 전용 점검, 서보 읽기 전용 진단 |
| `docs/` | 규격·인터페이스·통신 구조·VR 연동·시험 계획·한계·구조 문서 |

로봇 이미지와 이미지 빌드 스크립트는 이 저장소에 없다.

## 빌드와 시험

개발 컨테이너(ROS 2 Humble)에서 빌드하고 시험한다. 로봇이 없어도 된다(가짜 하드웨어).

```bash
docker build -t armpi-ctl-dev:humble -f docker/Dockerfile.dev .
bash scripts/test_all.sh                                   # 전체 colcon build + gtest
bash ros2_ws/src/armpi_bringup/test/dev_build_and_test.sh all   # 제어기 통합·종단·웹 게이트웨이 시험
ARMPI_DEV_IMAGE=armpi-ctl-dev:humble bash scripts/armpi_dev_build_and_test.sh all   # 수신기 단위·통합 시험
```

로봇(Ubuntu 22.04 + ROS 2 Humble)에서 직접 빌드할 때(이 공개본으로는 아직 시험하지 않음):

```bash
cd ros2_ws
rosdep install --from-paths src -y --ignore-src
colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## 실행

```bash
source ros2_ws/install/setup.bash
ros2 launch armpi_bringup armpi_mock.launch.py   # 가짜 하드웨어(PC에서 시험, 시험용 프로필)
ros2 launch armpi_bringup armpi.launch.py        # 실제 로봇(하드웨어 프로필 필요, 아래)
```

## 하드웨어 프로필 만들기 (실제 로봇에 필요)

로봇마다 다를 수 있거나 제조사 공개 자료로 확인되지 않은 장치 값(차체 치수·엔코더 펄스, 관절 ↔ 서보 배정과 방향, 관절 기준·범위, 그리퍼 끝값, 기본 자세 서보값, 확장보드 주소·레지스터, GPIO 핀, 서보 반이중 타이밍)은 **이 저장소에 없다.** 실제 로봇에서는 이 값을 하드웨어 프로필 파일로 넣는다.

1. 본보기를 복사한다.
   ```bash
   cp ros2_ws/src/armpi_bringup/config/hardware_profile.template.yaml \
      ros2_ws/src/armpi_bringup/config/hardware_profile.yaml     # .gitignore 에 들어 있어 커밋되지 않는다
   ```
2. `null`로 비어 있는 값을 **직접 측정하거나 제조사 자료로** 채운다. 채워 둔 값(버스 서보 규격, 서보 ID 1·2·6, 관절 범위 일부)은 각 줄 주석의 Hiwonder 공개 자료에서 확인한 것이다. 키 설명은 `ros2_ws/src/armpi_bringup/README_KO.md` 4.1절.
3. `colcon build` 하면 설치 폴더(`share/armpi_bringup/config/`)로 복사되고 `armpi.launch.py`가 자동으로 읽는다. 다른 위치의 파일은 `ros2 launch armpi_bringup armpi.launch.py hardware_profile:=<경로>`.
4. 처음 움직일 때는 바퀴를 띄우고 팔 주변을 비운 뒤 한 관절씩 확인한다(`docs/10_HARDWARE_TEST_PLAN_KO.md`). 서보 읽기 진단 `tools/servo_read_diag.py --hardware-profile <파일>`로 위치만 먼저 읽어 볼 수 있다.

- 값이 하나라도 비어 있거나 형식이 틀리면 제어 노드는 **장치를 열지 않고** `fault`("하드웨어 프로필 값 없음 …: 빠진 키")로 시작한다. 바퀴·서보에 아무것도 보내지 않는다.
- `config/test_hardware_profile.yaml`은 가짜 하드웨어·시험 전용 값이다. **실제 로봇 값이 아니다**(일부러 다르게 골랐다). 실제 로봇에 쓰면 팔·바퀴가 엉뚱하게 움직일 수 있다.

- 로봇에서 자동 시작하려면 `deploy/systemd/`의 유닛을 쓴다. 카메라 영상(큰 메시지)에는 `deploy/sysctl/60-armpi-dds.conf`가 필요하다. 서비스를 로그인 사용자(예: `ubuntu`)로 돌리면 `deploy/logind/90-armpi-keep-ipc.conf`를 `/etc/systemd/logind.conf.d/`에 넣는다(넣지 않으면 SSH 를 닫을 때 노드 사이 통신이 끊긴다, `docs/11` 0000절).
- 카메라 영상: `http://<IP>:8080/stream?topic=/usb_cam/image_raw&type=ros_compressed&qos_profile=sensor_data`. **원시 영상은 반드시 `qos_profile=sensor_data`(best-effort)로 받는다.** 기본(RELIABLE)으로 받으면(스냅샷·`type=mjpeg` 포함) 로봇 안 ROS 통신이 막힐 수 있다(2026-11-04 실기, `docs/11` 0절).
- 소프트웨어를 바꾼 뒤에는 서비스 재시작보다 **로봇 재부팅**을 권장한다(같은 실기).
- 조종기: `clients/mac_controller/README_KO.md`, 휴대폰 화면 `http://<로봇 IP>:18765/mobile`.
- VR 클라이언트를 만들 때: `docs/08_VR_HANDOFF_KO.md`, `docs/01_PROTOCOL_V3_SPEC.md`, `clients/vr_handoff/ArmPiV3Messages.cs`.

## 안전

- **실제 로봇 구동은 사람이 옆에 있을 때만 한다.** 처음에는 차체를 받침대에 올려 바퀴를 띄우고, 팔이 닿는 범위를 비운다.
- 독립 하드웨어 비상정지가 없다. 모든 정지(UDP, 화면 버튼, KEY1, watchdog)는 같은 Pi의 소프트웨어다. 전원 스위치에 손을 둔다.
- 기본 속도 상한: 차체 0.25 m/s·1.2 rad/s, 팔 손끝 0.15 m/s(2026-11-04 결정, 바퀴 방향 실기 시험 전). 처음 시험할 때는 `armpi.yaml`에서 더 낮춰 쓴다.
- 제어 노드가 시작할 때 팔이 기본 자세로 움직인다(`arm.home_on_start`).
- 인증·암호화가 없다. 같은 망의 누구나 접속할 수 있다(`docs/11`, `docs/12`).

## 현재 상태 (2026-11-04)

| 항목 | 상태 |
|---|---|
| 빌드·단위 시험 | gtest 241개 통과, 컴파일 경고 0(개발 컨테이너, 시험용 하드웨어 프로필) |
| 통합·종단 시험(가짜 하드웨어) | 통과 |
| 실제 로봇: 부팅·네트워크·SSH·서비스·장치 통신(I2C 쓰기·서보 쓰기 실패 0)·카메라 웹 스트림 | 확인 |
| 실제 로봇: 비상정지 latch(휴대폰 화면) | 확인 |
| 실제 로봇: 바퀴 방향·회전 부호·팔 손끝 정확도·그리퍼 끝점 | **미완료** |
| VR(Unity/Quest) 앱 | 없음(규격과 C# 예제만) |

링크 길이는 `config/armpi.yaml`의 Hiwonder 공개 ArmPi FPV 기구학 값이다(ArmPi Pro 실측 확인 필요). 로봇마다 다른 장치 값은 위 하드웨어 프로필로 넣는다. 실제 로봇에서 확인한 뒤 조정한다(`docs/10_HARDWARE_TEST_PLAN_KO.md`).

## 문서

`docs/01` 규격 · `docs/02` ROS 2 인터페이스 · `docs/04` 앱 사용법 · `docs/07` 통신 구조 · `docs/08` VR 연동 · `docs/09` ROS 2 공부 자료 · `docs/10` 실기 시험 계획 · `docs/11` 알려진 한계 · `docs/12` 조종권 중재 제안 · `docs/13` 소프트웨어 구조

## 라이선스

Apache License 2.0(`LICENSE`, `NOTICE`). 있는 그대로 제공하며, 실제 로봇에서 쓰는 책임은 사용자에게 있다.
