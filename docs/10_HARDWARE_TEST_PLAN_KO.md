# 10. 첫 실기 시험 계획 (ArmPi Pro, Ubuntu 22.04 + ROS 2 Humble)

작성 2026-11-02 22:30 (Asia/Seoul).

**이 문서의 항목은 아직 하나도 실행되지 않았다.** 새 이미지로 실제 로봇을 움직여 본 기록은 0건이다.
결과는 9절 기록 양식으로 남긴다. PASS는 출력·로그·영상이 붙은 항목에만 쓴다.

증거 표기
- [코드]: 이 저장소 코드·설정 파일에서 확인
- [계산]: 코드의 식으로 직접 계산한 기대값(실측 아님)
- [시험 로그]: (시험 기록) 아래 기록(가짜 로봇·컨테이너·루프백 기준)
- [추정]: 근거가 부족함
- [미확인]: 아직 모름. 이 시험에서 확인할 것

관련 문서
- 하드웨어 설정: `ros2_ws/src/armpi_bringup/config/armpi.yaml`(공통)과 하드웨어 프로필(`config/hardware_profile.yaml`, 본보기 `hardware_profile.template.yaml`). 시험 전에 프로필을 채운다(README "하드웨어 프로필 만들기")
- 프로토콜: `01_PROTOCOL_V3_SPEC.md`, 토픽·서비스: `02_ROS2_INTERFACES.md`
- USB 굽기: 이미지 굽기 안내(공개본에 없음), 앱 사용법: `04_APPS_HOWTO_KO.md`(작성 중)
- 알려진 한계: `11_KNOWN_LIMITATIONS_KO.md`

## 0. 안전 준비 (모든 단계 공통)

이 시험은 **개발자가 장치를 확인하는 시험**이다. 사람 대상 실험이 아니다. 참가자를 부르지 않는다.

| # | 준비 | 이유 |
|---|---|---|
| 0-1 | 차체를 받침대에 올려 **네 바퀴가 바닥에 닿지 않게** 한다 (C·E·V 단계) | 메카넘은 앞뒤·옆·대각선·제자리 회전을 모두 한다 |
| 0-2 | **팔이 닿는 범위를 비운다.** 손·케이블·물건을 치운다 | 어깨 축에서 손끝까지 링크 합이 약 0.36 m다 [계산: 링크 길이 합] |
| 0-3 | 그리퍼·관절 사이에 **손가락을 넣지 않는다** | 끼임 위험. 그리퍼 닫힘 끝점은 아직 모른다 |
| 0-4 | 한 사람은 **배터리 스위치(또는 전원 분리 수단)에 손을 둔다** | 독립 하드웨어 비상정지가 없다 [코드]. 모든 정지는 같은 Pi의 소프트웨어다 |
| 0-5 | 전원을 넣으면 **팔이 바로 움직인다**는 것을 미리 알린다 | `arm.home_on_start: true`: 시작하자마자 기본 자세로 1.5 s 동안 이동 [코드] |
| 0-6 | 배터리 전압을 확인한다(멀티미터) | 낮으면 서보가 떨리거나 처질 수 있다 |
| 0-7 | 속도 상한은 **설정값 그대로** 둔다(사용자 결정 2026-11-04, 바퀴 방향 실기 시험 전. 이전 보수값 차체 0.10 m/s·0.60 rad/s, 팔 0.10 m/s) | 차체 0.25 m/s, 1.2 rad/s / 팔 손끝 0.15 m/s, pitch 1.0 rad/s, 그리퍼 1.0 /s [코드: `config/armpi.yaml`] |
| 0-8 | 조종기 속도는 **10–30 %** 로 낮춰 시작한다 | 휴대폰 화면 기본 60 %, Mac 조종기 슬라이더 기본 30 % [코드]. 상한이 올라갔으므로(0-7) 처음에는 반드시 낮춘다 |
| 0-9 | **한 단계라도 실패하면 다음 단계로 가지 않는다** | 원인을 먼저 찾는다(10절) |
| 0-10 | 조종기는 **한 번에 하나만** 켠다 | 두 번째 조종기는 `not_owner`가 된다 [코드] |

준비물
- 받침대, 마스킹 테이프, 줄자(또는 자), 모눈종이(A3 이상)
- 위에서 찍을 휴대전화(가능하면 120 fps 이상 슬로모션), 멀티미터
- Mac(조종기, `protocol_v3/`), 유선 LAN 케이블

권장 실행 순서: **B → S → A → C → E → V → W(바닥)**. M(측정)은 해당 단계와 함께 한다.

## 1. 부팅·네트워크 (B)

> **로봇에 SD 카드가 꽂혀 있으면 켜기 전에 반드시 뺀다.** 꽂힌 채 켜면 예전 시스템 커널로 섞여 부팅되고, SD 카드 내용이 바뀔 수 있다.
> v2 이미지: `/armpi/status` 의 `kernel` 이 이미지 커널이고 `boot_warnings` 가 빈 목록인지 B1 에서 같이 본다.
> 와이파이 우선순위: **1순위 와이파이 → 2순위 와이파이 → 대체 AP**(2순위는 v2 이미지를 비밀 파일과 함께 만들었을 때만).
> 2026-11-04 v1 이미지 B절 결과: `11_KNOWN_LIMITATIONS_KO.md` 00.2절(카메라 B15 부분 실패).

전제: 새 이미지를 새 USB에 구웠다(이미지 굽기 안내(공개본에 없음)). 예전 시스템이 든 SD 카드·USB는 꽂지 않는다.
작성 시점(22:30)에는 이미지 10·20단계만 끝났다. 30–60단계와 `output/` 이미지는 아직 없다 [빌드 기록].

| ID | 절차 | 합격 기준 | 기록 |
|---|---|---|---|
| B1 | 새 USB만 꽂고 전원을 켠다. 첫 부팅은 루트 확장·SSH 호스트 키 생성이 함께 돈다 | SSH로 접속된다 | 전원 → SSH 접속까지 시간(기준값은 첫 결과 뒤 정함) |
| B2 | 유선: Mac과 직결(인터넷 공유)한다. 로봇에서 `ip -4 addr show eth0`, `nmcli device status` | eth0가 DHCP 주소를 받는다. `armpi-eth0` 연결이 활성 | 주소 |
| B3 | 무선: `nmcli -t -f DEVICE,STATE device`, `nmcli connection show --active`, `journalctl -u armpi-net-fallback -b` | `wlan0:connected`, 연결 이름 `armpi-wifi`(SSID 1순위 와이파이). 로그에 "wlan0 연결됨" | 주소, 신호 |
| B4 | 대체 AP: 1순위 와이파이가 없는 곳에서 부팅한다 | 약 60 s 뒤 AP `ArmPi-ROS2`(2.4 GHz ch6)가 보인다. 대체 AP 비밀번호(문서에 적지 않음, 담당자에게 확인)로 접속하면 로봇이 `192.168.149.1` | 보일 때까지 시간 |
| B5 | mDNS: Mac에서 `ping armpi-ros2.local` | 응답 | 주소 |
| B6 | SSH: `ssh -i ~/.ssh/id_ed25519_armpi ubuntu@armpi-ros2.local` | 비밀번호 없이 접속 | |
| B7 | `systemctl status armpi.service NetworkManager avahi-daemon ssh systemd-timesyncd armpi-net-fallback` | 모두 active (`armpi-net-fallback`은 oneshot 완료) | 출력 |
| B8 | `journalctl -u armpi.service -b --no-pager \| tail -100` | "armpi_control 시작 (실제 하드웨어)" 줄이 있고 `FAULT`가 없다 | 로그 파일 |
| B9 | 소스 기준판: `cat /opt/armpi/manifests/stage30.env` | `ros2_ws_src_tree_sha256` 값이 있다 | 해시(9절 양식에 적음) |
| B10 | 시간: `timedatectl`, `timedatectl timesync-status` | 인터넷이 될 때 "System clock synchronized: yes" | 오프셋. **AP 모드(인터넷 없음)에서는 동기화가 안 된다** [추정] |
| B11 | 직렬 콘솔 없음: `cat /proc/cmdline`, `systemctl is-enabled serial-getty@ttyS0`, `ls -l /dev/ttyS0 /dev/serial*` | cmdline에 `console=serial0`·`console=ttyS0`가 없다. getty는 `masked`. `/dev/ttyS0`가 있고 그룹이 `dialout` | 출력. `serial0` 링크가 있으면 어디를 가리키는지 적는다. GPIO14/15 UART가 `ttyS0`가 아니면 중단(`hardware.servo_port` 확인) |
| B12 | I2C(쓰기 없이 먼저): `ls -l /dev/i2c-1`, `ros2 topic echo --once /armpi/status`, `ros2 topic echo --once /armpi_control/hw_state` | `battery_v`가 null이 아니다(확장보드 읽기 성공). `wheel_failures` 0, `wheel_writes`가 늘어난다(0x34 쓰기 성공) | 출력 |
| B12b | (선택) 주소 스캔: `sudo systemctl stop armpi.service` 뒤 `i2cdetect -y -r 1 0x34 0x34`, `i2cdetect -y -r 1 <확장보드 주소> <확장보드 주소>`(프로필 `hardware.expansion_board_address`) | 두 주소가 보인다 | `i2cdetect`는 장치에 탐지 신호를 보낸다. 확장보드에 영향이 없는지 [미확인]이므로 B12로 충분하면 생략 |
| B13 | 배터리: `ros2 topic echo --once /armpi/battery` | `voltage`가 멀티미터 값과 비슷하다 | 두 값. `percentage`는 6.6–8.4 V 선형 대략값이다 [코드] |
| B14 | 팬: 귀·손으로 확인, `/armpi/status`의 `fan_ok` | 팬이 돈다, `fan_ok: true` | |
| B14b | 부저(v2.1부터): 전원을 넣거나 `armpi.service`를 다시 시작한 직후 소리를 듣는다 | 짧게 **1번**(정상). 2번이면 경고(`boot_warnings`·`warnings`), 3번이면 fault | 들린 횟수. 재시작하면 팔이 기본 자세로 움직이므로 팔 주변을 비운다 |
| B15 | 카메라(원시 영상은 반드시 `qos_profile=sensor_data`(best-effort)로 받는다. 기본(RELIABLE)으로 받으면(스냅샷·`type=mjpeg` 포함) 로봇 안 ROS 통신이 막힐 수 있다(2026-11-04 실기, `docs/11` 0절).): `v4l2-ctl --list-devices`, `ros2 topic hz /usb_cam/image_raw`, 브라우저 `http://<IP>:8080/stream?topic=/usb_cam/image_raw&type=ros_compressed&qos_profile=sensor_data` | 영상이 보인다 | fps. 기본 640×480, 15 fps, `yuyv2rgb` [코드: `armpi.launch.py`]. v2.1 기본: 밝기 0(`camera_brightness`), 전원 주파수 60 Hz(`camera_power_line_hz`). `v4l2-ctl -d /dev/video0 --get-ctrl brightness,power_line_frequency`로 `0`, `2`인지 본다 |
| B16 | 재부팅 후 B7·B8·B12 반복 | 같은 결과 | |

### B절 자동 수집 도구 `tools/first_boot_check.sh` (2026-11-04 추가)

Mac에서 실행한다. B1–B3, B5, B7–B15의 **읽기 전용** 항목을 한 번에 모아 (시험 기록)에 저장한다. `summary.md`를 먼저 본다. PASS/FAIL 판정은 사람이 이 표의 기준으로 한다.

```bash
tools/first_boot_check.sh --dry-run                    # 보낼 명령만 확인(접속 안 함)
tools/first_boot_check.sh --host <로봇 IP 또는 이름>    # 기본 ubuntu@armpi-ros2.local, 키 ~/.ssh/id_ed25519_armpi
tools/first_boot_check.sh --host <로봇> --wait 300     # 전원을 넣는 순간 시작하면 SSH가 열린 시간(B1)도 적는다
```

- SSH는 공개키로만 접속한다. 비밀번호를 묻거나 저장하지 않는다. 저장 전에 비밀값처럼 보이는 문자열을 가린다.
- 로봇에 보내는 명령은 허용 목록으로 검사한다. 서비스 시작·정지, 서비스 호출, 토픽 발행, `i2cdetect`, GPIO 요청, 직렬 포트 읽기, 재부팅, 파일 쓰기는 실행 전에 막는다(`--guard-self-test`로 확인).
- 로봇에 설치된 소스 트리 해시를 다시 계산해 `output/IMAGE_BUILD_MANIFEST.json`과 대조한다(B9, USB 굽기 간접 확인).
- 자동으로 못 하는 것: B4(대체 AP), B12b, B13의 멀티미터 비교, B14 팬 소리, B15 브라우저 영상, B16 재부팅.
- 시험 상태: 컨테이너에서 완성 이미지를 부팅해 모의 시험만 했다((시험 기록)). 실제 로봇에서는 아직 쓰지 않았다.

## 2. 버스 서보 통신 (S)

서보 버스는 `/dev/ttyS0` 115200 bps 반이중이다(Hiwonder 버스 서보 통신 규격). 방향 핀 RX_CON·TX_CON 번호는 하드웨어 프로필 `hardware.servo_rx_con_bcm`·`servo_tx_con_bcm`에서 받는다(README의 "하드웨어 프로필 만들기").

**단독 읽기 도구는 아직 없다(도구 미구현).** 새 코드의 실행 파일은 `armpi_control_node`, `armpi_vr_bridge_node` 두 개뿐이다 [코드: `add_executable`]. POS_READ·VIN_READ·TEMP_READ를 따로 불러 보는 명령이 없다.
그래서 아래 S2는 제어기의 `servo.read_position`을 잠시 켜서 확인한다. VIN·TEMP 읽기는 제어기가 쓰지 않으므로 이번 계획에서는 확인할 수 없다.

| ID | 절차 | 합격 기준 | 기록 |
|---|---|---|---|
| S1 | 쓰기: 부팅 시 기본 자세 이동(A1과 같음), `/armpi_control/hw_state` | 팔이 움직인다. `servo_failures` 0, `servo_writes` 증가 | 출력 |
| S2 | 읽기: 설치된 `armpi_bringup/config/armpi.yaml`을 홈 폴더에 복사하고 `servo.read_position: true`로 바꾼다(타이밍을 바꿀 때는 하드웨어 프로필 사본을 `hardware_profile:=`로 넘긴다). `sudo systemctl stop armpi.service` 뒤 `ros2 launch armpi_bringup armpi.launch.py params_file:=/home/ubuntu/armpi_s2.yaml`. 팔은 정지 상태 | `/armpi/status`의 `position_source`가 `"measured"` | 각 관절 측정값과 명령값 차이 |
| S3 | S2에서 읽기가 실패하면(`position_source`가 `"commanded"`에 머묾) `servo.read_turnaround_delay_us`, `servo.read_timeout_ms`(하드웨어 프로필), `servo.drain_before_turnaround`(`armpi.yaml`, 기본 true) 순서로 하나씩 바꾼다 | 바꾼 값과 결과를 모두 적는다 | 값·결과 표. 읽기 실패 횟수는 토픽에 없다 → `journalctl` 경고로 본다 [코드] |
| S5 | 끝나면 수동 launch를 끝내고 `sudo systemctl start armpi.service`로 기본 설정(`servo.read_position: false`)에 돌아간다 | 기본값 복귀 | |

타이밍 값은 하드웨어 프로필(`servo.pre_write_delay_us`, `post_write_delay_us`, `read_turnaround_delay_us`, `read_timeout_ms`, `read_attempts`)에서 받는다. 이 저장소에는 실제 값이 없다. `drain_before_turnaround`는 `armpi.yaml`(기본 true).

### 서보 읽기 전용 진단 도구 `tools/servo_read_diag.py` (2026-11-04 추가)

위 "단독 읽기 도구 없음"을 보완한다. 로봇 안에서 위치(POS_READ)·전압(VIN_READ)·온도(TEMP_READ)·ID 응답(ID_READ)만 읽는다. 반이중 순서와 타이밍은 `armpi_hardware`의 드라이버와 같다.

- 이미지에는 들어 있지 않다. Mac에서 `scp tools/servo_read_diag.py ubuntu@<로봇>:~/`로 복사한다. 이미지에 있는 Python 3와 libgpiod만 쓴다.
- `armpi.service`가 켜져 있거나 다른 프로세스가 `/dev/ttyS0`를 쓰고 있으면 **실행을 거부한다**(종료 코드 4). 서비스를 멈출지는 담당자가 정한다. 도구가 서비스를 멈추지 않는다.
- 이동·토크·ID 변경 명령은 코드에 없다. 보내기 직전에 패킷이 읽기 요청인지 다시 검사한다.
- 방향 핀·타이밍·관절 배정은 하드웨어 프로필에서 읽는다(`--hardware-profile <파일>`, 없으면 설치 위치 `share/armpi_bringup/config/hardware_profile.yaml`을 찾는다). 값이 없으면 아무것도 보내지 않고 종료 코드 2로 끝난다. 프로필 파일도 함께 복사한다.

```bash
python3 ~/servo_read_diag.py --hardware-profile ~/hardware_profile.yaml   # ID 1–6, pos·vin·temp, 1회
python3 ~/servo_read_diag.py --repeat 5 --json ~/servo_read.json
python3 ~/servo_read_diag.py --turnaround-us 2000      # S3처럼 타이밍을 바꿔 볼 때(--pre-write-us, --post-write-us, --timeout-ms, --attempts, --no-drain)
```

- 위치 0–1000 옆에 관절각(참고, 오프셋 없음)을 함께 보여 준다(프로필의 관절 배정·방향 기준). ID 1은 그리퍼(프로필 열림·닫힘 기준 비율)다.
- 시험 상태: pty 가짜 서보와 시험용 프로필 값으로 모의 시험만 했다(`tools/servo_read_diag_selftest.py`). 실제 서보 버스에서는 아직 쓰지 않았다.

## 3. 팔·그리퍼 (A)

차체는 받침대 위. 팔 주변을 비운다. 먼저 조종 세션이 없는 상태(Mac 조종기를 켜지 않음)에서 한다.

| ID | 절차 | 합격 기준 | 기록 |
|---|---|---|---|
| A1 | 기본 자세: 부팅 시 자동, 또는 `ros2 service call /armpi/arm/home std_srvs/srv/Trigger` | 서보 ID 1–6 = 하드웨어 프로필 `arm.home_servo_values`로 1.5 s 동안 이동. 떨림·충돌음 없음 | 영상 |
| A2 | 기본 자세의 손끝 위치를 잰다 | 계산값 [계산]: x 0.1047 m, y 0, z 0.0753 m, pitch −1.0263 rad(−58.8°). 원점 `armpi_arm_base`는 어깨 관절축에서 0.064605 m 아래, 베이스 회전축 위의 점이다. 손끝은 어깨 축보다 약 0.011 m 높고 베이스 회전축에서 약 0.105 m 앞 | 실측값과 차이. 원점의 실제 위치는 [미확인] |
| A3 | 관절 하나씩 작게: `ros2 topic pub --once /armpi/arm/joint_debug sensor_msgs/msg/JointState "{name: [joint1], position: [0.1]}"` (joint2·3·4·5도 같은 방법, ±0.1 rad ≈ ±5.7°) | 기대 방향은 하드웨어 프로필의 관절 기준(`arm.kinematics.j*_sign/offset`)과 방향(`arm.joint_directions`)으로 미리 계산해 적어 둔다 | 관절별 실제 방향. 하나라도 반대면 IK 결과가 틀린다 → 중단 |
| A4 | 그리퍼 끝점: `r_joint`를 프로필 열림값(`arm.gripper.open_value`)에 해당하는 각도 → 닫힘값(`close_value`)에 해당하는 각도로(각도 = 방향·(서보값 − 500)·0.24°) | 열림·닫힘이 끝까지 가고, 닫힘에서 서보가 계속 힘을 쓰는 소리(윙윙)·발열이 없다. **손가락을 넣지 않는다** | 실제 끝점 서보값. 허용 범위는 프로필 `arm.gripper.servo_min/max` |
| A5 | 작업 공간·IK 거부: Mac 조종기에서 팔 조종을 켜고, 작업 공간 밖(예 x 0.40) 숫자 입력 | `workspace_limit` 표시, 팔은 움직이지 않는다 | 화면 |
| A6 | 도달 불가 목표(예 x 0.28, z 0.30, pitch 0) | `ik_unreachable` 표시, 팔은 그대로 | 화면 |
| A7 | 작은 IK 이동: 팔 조종 켜기 → I 키를 0.2 s | 손끝이 앞으로 약 3 cm 곧게 움직인다(키 속도 0.15 m/s) | 실측 이동량 |
| A8 | 토크 유지: 팔을 기본 자세가 아닌 곳에 둔 채 (1) Space 비상정지, (2) Mac 조종기 강제 종료, (3) `sudo systemctl stop armpi.service` | 세 경우 모두 팔이 **처지지 않는다** | 영상. (3)은 서보가 마지막 명령을 유지하는지 [추정] |
| A9 | 손끝 정확도: 모눈종이를 깔고 목표 5곳을 숫자로 입력(예: A2 자세 근처, x ±3 cm, y ±3 cm). 곳마다 5회 | 기준은 첫 결과 뒤 정함 | 목표·실측 x/y/z, 반복 차이. 그리퍼 끝점의 실제 위치 정의는 [미확인] |
| A10 | 손목 회전: joint5는 서보값 500 고정이다 [코드] | 조종 중 joint5가 움직이지 않는다 | |

참고: 실제 링크 길이로 계산하면 손끝이 **아래(pitch −90°)를 향하는 자세는 z ≈ 0 근처에서만** 풀린다 [계산]. A9 목표는 pitch −45°~0° 쪽에서 고른다. 도달 범위표는 `08_VR_HANDOFF_KO.md`에 있다.

## 4. 차체 (C, 바퀴를 띄운 상태)

Mac 조종기로 한다(`python3 clients/mac_controller/armpi_mac_controller.py --robot <IP>`). 속도 슬라이더 30 %에서 시작한다.
"앞"은 메카넘 식의 90° 방향이다. 팔이 향한 쪽과 같은지는 [미확인]이므로 C1에서 적는다.

기대 바퀴 펄스 [계산: `armpi_hardware` 메카넘 식, `angular_sign` 1.0, 100 % 기준]. 실제 값은 `/armpi_control/hw_state`의 `wheels`(모터 1–4)에서 본다.

| 입력 | 모터 1–4 (펄스/10 ms) |
|---|---|
| 전진 0.10 m/s | 25, −25, −25, 25 |
| 후진 | −25, 25, 25, −25 |
| 왼쪽 평행이동 0.10 m/s | 25, 25, 25, 25 |
| 오른쪽 평행이동 | −25, −25, −25, −25 |
| 좌회전 +0.60 rad/s | 32, −32, 32, −32 |
| 우회전 | −32, 32, −32, 32 |
| 전진+왼쪽+좌회전 모두 100 % | 83, −32, 32, 19 |

모터 번호 1–4가 실제 어느 바퀴인지는 [미확인]이다. C1에서 적는다.

| ID | 절차 | 합격 기준 | 기록 |
|---|---|---|---|
| C1 | W(전진)만 1 s | `hw_state.wheels`가 위 표와 부호가 같다. 네 바퀴가 모두 "앞으로 굴러가는" 방향으로 돈다 | 바퀴별(앞왼·앞오·뒤왼·뒤오) 회전 방향, 모터 번호 대응, 앞 = 팔 쪽인지 |
| C2 | S, A, D 각각 1 s | 표와 같은 부호. A(왼쪽)에서 메카넘 바퀴 패턴이 왼쪽 이동에 맞다 | 바퀴별 방향 |
| C3 | **회전 부호**: Q(좌회전, `turn>0`) 1 s | 오른쪽 바퀴는 앞으로, 왼쪽 바퀴는 뒤로 → 바닥에서는 위에서 볼 때 **반시계**가 된다 | 바퀴별 방향. 반대면 `chassis.angular_sign`이 −1이어야 할 수 있다. 바꾸기 전에 원인을 먼저 찾는다(10절) |
| C4 | 키를 뗀다 | 바퀴가 바로 멈춘다(정지 명령은 다음 틱을 기다리지 않음 [코드]) | 영상 프레임으로 정지 시간 |
| C5 | 2차 watchdog: W를 누른 채 다른 터미널에서 `pkill -KILL -f armpi_vr_bridge_node` | 0.6 s 안에 멈춘다. 로그 "cmd_vel 이 0.60 s 동안 없음 → 차체 정지(2차 watchdog)" | 정지 시간. **수신기는 launch에서 자동 재시작되지 않는다** [코드: respawn 없음] → `sudo systemctl restart armpi.service` |
| C6 | 오래된 명령 버림: `ros2 topic pub --once /armpi/cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.05}}}"` (stamp 0) | 바퀴가 돌지 않는다. `/armpi/status`의 `cmd_vel.dropped_stale`이 1 늘어난다 | 출력 |
| C7 | 펄스 상한: 슬라이더 100 %에서 W+A+Q 동시 | 로그에 "바퀴 펄스가 상한 100 를 넘어" 경고가 **없다**(계산상 최대 83) | `hw_state.wheels` |
| C8 | 가속 제한: 0 → 100 % 전진 | 약 0.17 s에 걸쳐 빨라진다(0.6 m/s² [코드]). 감속·정지는 즉시 | 영상(선택) |

## 5. 비상정지 (E, 바퀴를 띄운 상태)

정지 수단은 두 종류다. 섞어서 판정하지 않는다.
- **소프트웨어 정지**: UDP·서비스·KEY1·watchdog. 모두 같은 Pi의 프로그램이 처리한다. 프로그램이 멈추면 듣지 않는다.
- **물리 정지**: 배터리 스위치·전원 분리. 확실하지만 서보 토크도 사라져 **팔이 처진다** [추정].

| ID | 절차 | 합격 기준 | 기록 |
|---|---|---|---|
| E1 | 바퀴가 도는 중 Mac 조종기 Space(UDP `emergency_stop`) | 바퀴 즉시 정지, 팔 자세 유지, 부저 0.1 s. `/armpi/status`의 `estop_sources: ["udp"]` | 정지 시간 |
| E2 | 비상정지 중 키를 계속 누름 | 움직이지 않는다 | |
| E3 | 해제 버튼(UDP `clear_emergency`) → 키를 누른 채 | 해제돼도 자동 출발하지 않는다. 키를 뗐다 다시 눌러야 움직인다 [코드: Mac 조종기] | |
| E4 | `ros2 service call /armpi/emergency_stop std_srvs/srv/Trigger` | `estop_sources: ["service"]`. UDP 해제로는 풀리지 않는다 | |
| E5 | `ros2 service call /armpi/reset_estop std_srvs/srv/Trigger` | 모든 출처 해제. 해제 뒤에도 새 중립 전에는 움직이지 않는다 | |
| E6 | **KEY1**을 짧게 누른다 | 비상정지 latch, `estop_sources: ["button"]`. UDP 해제로는 안 풀리고 `/armpi/reset_estop`으로만 풀린다 [코드] | | |
| E7 | **KEY2**를 2 s 이상 누른다. 다른 터미널에서 `ros2 topic echo /armpi/shutdown_requested` | 메시지 1개가 나오고, `armpi-shutdown.service`가 `systemctl poweroff`로 로봇을 끈다 | 22:32에 `deploy/systemd/armpi-shutdown.service`(토픽 1회 수신 → poweroff)가 추가됐다 [코드]. 실기·이미지 포함 여부 미확인. **꺼지는 시험이므로 마지막 순서로 한다**. 끄기 전 로그를 먼저 옮긴다 |
| E8 | 조종기 강제 종료: 바퀴가 도는 중 Mac에서 조종기 프로세스를 `kill -9` | 0.4 s 안에 정지(수신기 차체 watchdog), 팔 유지 | 정지 시간 |
| E9 | Wi-Fi 끊기: 바퀴가 도는 중 Mac Wi-Fi를 끈다(또는 공유기 전원) | 0.4 s 안에 정지. 복구 뒤 **키를 누르고 있지 않으면** 출발하지 않는다 | 복구 뒤 동작. 키를 누른 채면 Mac 조종기가 자동 재무장한다(설계) [코드]. 이 정책은 사람이 정한다 |
| E10 | 수신 노드 강제 종료: C5와 같음 | 0.6 s 안에 정지 | |
| E11 | **제어 노드 강제 종료**: 바퀴가 도는 중 `pkill -KILL -f armpi_control_node` | 바퀴가 멈추는지 본다 | **[미확인] 위험 항목.** 강제 종료 때는 종료 정지 코드가 돌지 않는다. 모터 보드가 마지막 속도를 계속 유지하는지 모른다. 계속 돌면 즉시 배터리 스위치로 끊고, 결과를 `11_KNOWN_LIMITATIONS_KO.md`에 반영할 것 |
| E12 | 물리 정지: 배터리 스위치 끄기 | 모두 멈춘다 | 팔이 처지는 정도(손으로 받칠 준비) |

## 6. 조종기 연결 (V)

| ID | 절차 | 합격 기준 | 기록 |
|---|---|---|---|
| V1 | 검색: Mac에서 `python3 protocol_v3/armpi_v3_client.py --discover` | 목록에 로봇 이름(hostname `armpi-ros2`)과 IP, protocol 3 | 출력 |
| V2 | 수신기 단독 적합성: `sudo systemctl stop armpi.service` → 로봇에서 `ros2 launch armpi_vr_bridge vr_bridge.launch.py` → Mac에서 `python3 protocol_v3/conformance_test.py --host <IP> --condition control_missing -v` | 실패 0. 제어기가 없으므로 활성 명령은 `not_owner`, 로봇은 움직이지 않는다 | 결과 JSON(`--json-report`) |
| V3 | 전체 적합성(바퀴 띄움, 팔 주변 비움): `armpi.service` 실행 상태에서 `--condition normal --workspace 0.05,0.32,-0.25,0.25,-0.08,0.32` | 실패 0. 차체가 잠깐(전진 0.3 이하) 돌고 팔이 기본 자세 근처에서 움직인다 | 결과 JSON. 작업 공간 기본값이 로봇 설정과 달라 `--workspace`를 꼭 준다 [코드] |
| V4 | Mac 조종기 연결: `python3 clients/mac_controller/armpi_mac_controller.py --robot <IP>` | 차체·팔 채널 `idle`, rosbridge 연결, 영상 표시 | 화면 캡처 |
| V5 | 두 번째 조종기(V4를 켠 채 `armpi_v3_client.py --robot <IP> --smoke`) | 두 번째는 `not_owner` | 출력 |
| V6 | (선택) 주소 제한: `allowed_sources`에 Mac 주소만 넣고 다른 기기에서 검색 | 다른 기기에는 응답이 없다 | |
| V7 | VR 앱: 앱이 준비되면 `08_VR_HANDOFF_KO.md` 절차로 | — | 작성 시점 VR 앱 없음 |

## 7. 지연·통신 측정 (M)

먼저 **무엇에서 무엇까지**를 정한다. 아래 구간을 섞지 않는다.

| 구간 | 시작 | 끝 | 같은 시계인가 |
|---|---|---|---|
| T1 왕복(RTT) | 조종기 송신 | 조종기 ACK 수신 | 예(조종기 시계 하나) |
| T2 망 한 방향 | 조종기 송신 | 로봇 수신기 수신 | **아니오**(Mac·로봇 두 시계) |
| T3 로봇 내부 | 수신기 `cmd_vel` 발행(`header.stamp`) | 제어기 수신 | 예(로봇 시계) |
| T4 구동 | 제어기 I2C·UART 쓰기 | 바퀴·서보가 실제로 움직임 | 측정 장비 필요 |
| T5 종단 | 사람 입력(키·컨트롤러) | 물리 반응 | 카메라 한 대로 둘 다 찍기 |

| ID | 측정 | 방법 | 주의 |
|---|---|---|---|
| M0 | 시계 확인 | 로봇 `timedatectl timesync-status`(오프셋), Mac `sntp time.apple.com` | 로봇에 RTC가 없다. 인터넷이 없으면(AP 모드) 시계가 맞지 않는다. 두 시계 차이로 T2를 구하지 않는다 |
| M1 | 수신 주기·유실·역순 | 수신기 10 s 통계 로그(`stats_log_period_sec` 10.0): `journalctl -u armpi.service -f`, `/armpi_vr_bridge/state`의 `packets` | 조종 20 Hz 기준. 최대 간격·p99를 적는다 |
| M2 | ACK 왕복 | Mac 조종기 화면 ACK 지연(마지막·중앙·최대), `armpi_v3_client.py --smoke`의 `ack_ms_p50` | RTT/2를 한 방향 지연으로 쓰려면 "두 방향이 같다"는 가정이 필요하다. 보고할 때 가정을 적는다 |
| M3 | 로봇 내부 | `ros2 topic delay /armpi/cmd_vel` (header.stamp 기준) | 같은 기기 시계라 비교 가능. 이 명령이 이미지에서 되는지는 [미확인] |
| M4 | 종단 | 120 fps 이상 카메라로 Mac 키보드(또는 화면 표시)와 바퀴를 한 화면에 찍는다. 프레임 차이 × 프레임 간격 | 프레임 간격만큼 오차가 있다. 10회 이상 |
| M5 | 재연결 시간 | Wi-Fi 복구 시각 → 조종기 채널 `idle` | |
| M6 | CPU·메모리·온도 | 조종 중 `top -b -n 30 -d 1`, `free -m`, `cat /sys/class/thermal/thermal_zone0/temp` | 카메라 스트림을 켠 경우·끈 경우를 나눈다 |
| M7 | 영상 | `ros2 topic hz /usb_cam/image_raw`, 브라우저 표시 fps | 영상이 조종 패킷에 주는 영향은 M1과 함께 본다 |

참고 수치(실기 아님): 컨테이너 루프백에서 수신기 watchdog 정지는 0.403 s, 팔 유지 전환은 0.407 s였다 [시험 로그: (시험 기록)]. 가짜 로봇과 Mac 조종기의 ACK 왕복 중앙값은 2.62 ms였다 [시험 로그: (시험 기록)]. **실제 Wi-Fi 값이 아니다.**

watchdog(0.4 s)·ACK timeout(0.30 s)은 M1·M2의 p99와 E9 결과를 본 뒤 조정한다. 조정 전 값을 합격 기준으로 쓰지 않는다.

## 8. 바닥 저속 시험 (W)

C·E 단계를 모두 통과한 뒤에만 한다. 2 m 이상 빈 바닥, 주변 사람·물건 없음.

| ID | 절차 | 합격 기준 |
|---|---|---|
| W1 | 정지 30 s | 미끄러짐·떨림 없음 |
| W2 | 전진 20 % 3 s | C1에서 정한 "앞"으로 간다. 반대면 즉시 중단 |
| W3 | 후진, 왼쪽·오른쪽 평행이동 각 20 % 3 s | 해당 방향 |
| W4 | 좌회전(Q) 20 % 3 s | 위에서 볼 때 **반시계** |
| W5 | 대각선(W+A) | 대각선으로 간다 |
| W6 | 키 떼기 | 멈춘다. 미끄러진 거리 기록 |
| W7 | 실제 속도: 테이프로 1 m 표시, 전진 40 %(0.10 m/s)와 100 %(0.25 m/s) 각 5회 | 걸린 시간으로 속도 계산. 명령값과 차이 기록 |
| W8 | 팔을 움직이며 이동(팔 조종 켜고 차체 저속) | 넘어지거나 기울지 않는다 |
| W9 | 10분 반복 조작 | 서보·모터 과열, 로그 오류, 메모리 증가 없음 |

## 9. 기록 양식

시험마다 한 줄. 원자료(로그·영상)는 고치지 않고 보관한다.
원자료·요약표 보관 위치(제안): `work/logs/hardware_<날짜>/`(git 밖).

| 열 | 내용 | 예시 형식 |
|---|---|---|
| experiment_id | 시험 묶음 | `HW-20261103-01` |
| trial_id | 시험 ID + 반복 번호 | `C3-02` |
| date_timezone | 날짜·시각·시간대 | `2026-11-03 14:05 KST` |
| operator | 조작자 / 안전 담당자 | 이름 두 개 |
| image_sha256 | 구운 이미지 해시(`output/SHA256SUMS`) | |
| source_tree_sha256 | `/opt/armpi/manifests/stage30.env`의 `ros2_ws_src_tree_sha256` | |
| config_changes | 기본값에서 바꾼 매개변수 | `servo.read_position=true` |
| condition | 받침대/바닥, 유선/1순위 와이파이/AP, 배터리 V | |
| input | 보낸 입력(키, 슬라이더 %, 목표 좌표) | |
| expected | 기대 결과(이 문서의 합격 기준) | |
| observed | 실제로 본 것 | |
| result | PASS / FAIL / 중단 / 미실시 | |
| evidence_path | 로그·영상·캡처 경로 | |
| aborted_reason | 중단했다면 이유 | |
| notes | 기타 | |

Markdown 표 머리글(복사용):

```text
| experiment_id | trial_id | date_timezone | operator | image_sha256 | source_tree_sha256 | config_changes | condition | input | expected | observed | result | evidence_path | aborted_reason | notes |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
```

시험이 끝나면 로봇에서 가져올 것: `journalctl -u armpi.service -b --no-pager > armpi_<시각>.log`, `/opt/armpi/manifests/` 전체, 바꾼 설정 파일.

## 10. 판정 규칙

- **부호가 반대면 설정을 바로 뒤집지 않는다.** 원인을 먼저 찾는다.
  1. 다른 조종 수단으로 같은 입력의 방향을 확인한다.
  2. 로봇 쪽(모터 번호·식·`angular_sign`) 문제인지, 조종기 쪽(키 배치·부호) 문제인지 나눈다.
  3. 고친 뒤에는 바꾼 값과 새 소스 해시를 기록한다.
- PASS는 **출력·로그·영상이 붙은 항목에만** 쓴다. 사용자 구두 보고나 "된 것 같다"는 PASS가 아니다.
- 실패한 단계 뒤의 항목은 "미실시"로 남긴다.
- 모든 W 항목을 통과하기 전에는 속도 상한을 올리지 않는다.
- 이 시험 결과는 장치 확인 기록이다. 사람 대상 평가 결과가 아니다.
