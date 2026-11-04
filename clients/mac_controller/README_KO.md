# ArmPi Mac 조종기 (`clients/mac_controller/`)

맥 키보드로 ArmPi Pro의 메카넘 차체와 팔·그리퍼를 조종하고 상태와 카메라 영상을 보는 도구다.
같은 서버가 휴대폰용 터치 화면(`/mobile`)도 제공한다. 로봇 안에서 `--bind 0.0.0.0`으로 돌리면 같은 와이파이의 휴대폰 브라우저·Android 앱이 접속한다.
**설치할 패키지가 없다.** macOS 기본 `python3`(3.8 이상) 표준 라이브러리와 브라우저만 쓴다.
앱으로 쓰는 법(Mac 앱, Android 앱, 휴대폰 브라우저)은 `../docs/04_APPS_HOWTO_KO.md`에 있다.

| 파일 | 내용 |
|---|---|
| `armpi_mac_controller.py` | 조종기 서버. 브라우저 입력을 받아 Protocol v3 UDP로 보내고, 필요하면 rosbridge에 연결한다 |
| `index.html` | PC 조종 화면(서버가 `http://127.0.0.1:18765`로 제공). **기준 위치: `ros2_ws/src/armpi_web_gateway/www/`**(로봇의 C++ 웹 조종기와 같은 파일. 맥 앱 번들에는 빌드 때 이 폴더 옆에 복사된다) |
| `mobile.html` | 휴대폰 가로 화면용 터치 조종 화면(`/mobile`): 조이스틱·회전 슬라이더·팔 버튼·비상정지 |
| `manifest.webmanifest`, `icon-192.png`, `icon-512.png` | 홈 화면 추가용 manifest·아이콘(같은 `www/` 폴더) |
| `test/e2e_mac_controller_check.py` | HTTP API 종단 확인(브라우저 없이). 파이썬 조종기와 로봇의 C++ 웹 조종기를 같은 항목으로 확인한다 |
| `make_icon.py` | 아이콘 PNG 생성기(웹·맥 앱·Android 앱 공용, 표준 라이브러리) |
| `../vr_protocol_v3/armpi_v3_client.py` | v3 클라이언트 라이브러리. **같은 상대 위치에 있어야 한다**(`clients/mac_controller/`만 복사하면 import 실패) |
| `../vr_protocol_v3/mock_robot.py` | Mac에서 시험할 때 쓰는 가짜 로봇 |
| `../vr_protocol_v3/conformance_test.py` | 규격 적합성 시험(가짜 로봇·로봇 C++ 수신기 공용) |

통신 규칙은 `../docs/01_PROTOCOL_V3_SPEC.md`를 따른다.

## 1. 실행

```bash
cd ArmPro_Cpp_ROS2
python3 clients/mac_controller/armpi_mac_controller.py --robot auto           # 같은 망에서 UDP 5002 검색(브로드캐스트)
python3 clients/mac_controller/armpi_mac_controller.py --robot 192.168.0.50   # IP 직접 지정
```

브라우저가 `http://127.0.0.1:18765/`를 연다. 기본(`--bind 127.0.0.1`)에서는 이 맥에서만 열린다. 끝낼 때는 터미널에서 Ctrl+C를 누른다. 그러면 조종기가 두 채널의 중립과 `disconnect`를 보내고 끝난다.

자주 쓰는 옵션:

| 옵션 | 기본 | 설명 |
|---|---|---|
| `--robot auto\|<IP>` | auto | auto면 `--discover-target`(기본 255.255.255.255)으로 검색한다. 여러 대가 응답하면 첫 번째 로봇을 쓰고 목록을 출력한다 |
| `--discover-target <IP>` | | 검색 요청을 보낼 주소. 여러 번 줄 수 있고, 주면 이 주소에만 보낸다(예: `127.0.0.1`) |
| `--port` | 5001(auto면 검색 응답 값) | 로봇 제어 UDP 포트 |
| `--http-port` | 18765 | 조종 화면 포트. 이 맥에서는 8765를 다른 프로그램(`app.server`)이 쓰고 있어서(2026-11-02 확인) 기본값을 18765로 바꿨다 |
| `--bind` | 127.0.0.1 | 조종 화면 HTTP를 열 주소. `0.0.0.0`이면 같은 망의 다른 기기(휴대폰)도 접속한다(**인증 없음**) |
| `--session auto\|always\|on-demand` | auto | v3 세션 방식. always = 서버가 켜져 있는 동안 세션을 잡음(맥 기본), on-demand = 조종권을 잡은 화면이 있을 때만 세션을 연다. auto = `--bind`가 127.0.0.1이면 always, 아니면 on-demand |
| `--idle-release` | 60 | on-demand에서 조종권을 가진 화면이 이 시간(s) 동안 조작이 없으면 조종권·세션을 놓는다(0 = 놓지 않음) |
| `--allow-host` | | `--bind`가 공개 주소일 때 추가로 받을 Host 이름. IP 주소와 이 기기의 호스트 이름(`armpi-ros2`, `armpi-ros2.local`)은 기본으로 받는다 |
| `--exit-with-stdin` | | 표준 입력이 닫히면(부모 앱이 끝나면) 중립·`disconnect`를 보내고 끝낸다(맥 앱이 쓴다) |
| `--no-rosbridge` / `--rosbridge-port` | 9090 | rosbridge 사용 여부·포트 |
| `--camera-port` / `--camera-url` | 8080 | 영상 주소 `http://<로봇>:8080/stream?topic=/usb_cam/image_raw&type=ros_compressed&qos_profile=sensor_data` |
| `--ack-timeout` | 0.30 | ACK 허용 지연(s, 사양 값). Wi-Fi가 불안정할 때만 늘린다. |
| `--home x,y,z,pitch,gripper` | 0.1047,0,0.0753,−1.0263,0 | 팔 시작 목표(`armpi_v3_client.DEFAULT_HOME`, 로봇 부팅 자세에 해당하는 손끝 위치) |
| `--arm-speed` / `--pitch-speed` / `--gripper-speed` | 0.15 m/s, 0.6 rad/s, 0.8 /s | 키를 누르는 동안 목표가 바뀌는 속도. 로봇은 별도 상한(0.15 m/s, 1.0 rad/s, 1.0 /s)으로 보간한다 |
| `--ui-default-speed` | 0.6 | 휴대폰 화면 속도 버튼 기본값(화면에 저장된 값이 없을 때) |

## 2. 키 배치와 화면

| 입력 | 동작 |
|---|---|
| **W / S** | 앞 / 뒤 (누르는 동안만) |
| **A / D** | 왼쪽 / 오른쪽 **평행이동**(strafe, 메카넘) |
| **Q / E** | 좌회전 / 우회전 (제자리, 위에서 볼 때 Q = 반시계) |
| 속도 슬라이더 | 10~100 %(기본 30 %). 100 % = 로봇 설정 상한(기본 앞뒤·좌우 0.25 m/s, 회전 1.2 rad/s) |
| **팔 조종 켜기** 버튼 | 팔 채널 활성(`control_active:true`). 끄면 팔은 현재 위치에서 멈춘다 |
| **I / K** | 손끝 목표 x 앞 / 뒤 |
| **J / L** | 손끝 목표 y 왼쪽 / 오른쪽 |
| **U / O** | 손끝 목표 z 위 / 아래 |
| **R / F** | pitch 올림 / 내림 (0 = 수평 앞, −90° = 아래) |
| **G / H** | 그리퍼 닫기 / 열기. 슬라이더: 0 = 열림, 1 = 닫힘 |
| 숫자 입력 + "입력값을 목표로 적용" | x·y·z(m), pitch(°)를 직접 지정. 빈 칸은 현재 목표 유지 |
| **Space** | **비상정지**(UDP `emergency_stop`, ACK를 받을 때까지 50 ms마다 재전송). 입력 칸에 커서가 있어도 동작 |
| 비상정지 해제 | UDP `clear_emergency`(조종자만 가능) |
| 연결 끊기 / 연결(새 세션) | `disconnect`(이 UUID는 다시 못 씀) / 새 UUID로 다시 시작 |
| 로봇 전체 정지 / 전체 해제 | rosbridge `/armpi/emergency_stop`, `/armpi/reset_estop` |
| 기본 자세로 이동 | rosbridge `/armpi/arm/home`(비상정지 중이나 팔 조종 중에는 로봇이 거부) |
| 관절 디버그 | `/armpi/arm/joint_debug` 슬라이더(±120°). **UDP 조종 세션이 없을 때만** 쓸 수 있다("연결 끊기" 후) |

화면 위쪽에는 UDP 연결, 차체 채널 상태, 팔 채널 상태, 비상정지, rosbridge 상태가 표시된다. 상태 표에는 채널별 상태·마지막 reason·마지막 해제 이유, ACK 지연(마지막·중앙값·최대, ms), 배터리, 로봇 소유자, 패킷 수가 나온다. 팔 표에는 목표와 로봇 현재값(rosbridge `/armpi/status`)을 나란히 보여 준다.
로봇이 팔 목표를 거부하면(`workspace_limit` = 작업 공간 밖, `ik_unreachable` = IK 해 없음) 빨간 상자로 5초간 알린다. 이때 목표는 **마지막으로 수락된 값으로 되돌린다.**
rosbridge가 없으면 "rosbridge 미연결"로 표시된다. 그래도 UDP 조종은 계속 동작한다. 영상이 없으면 "영상 없음"으로 표시된다.

## 3. 안전 동작(조종기 쪽)

- 차체·팔 패킷을 채널마다 50 ms마다 보낸다. 다음 경우에는 **두 채널 모두 중립**을 보낸다: 키를 뗐을 때, 브라우저 창이 포커스를 잃었을 때, 브라우저 입력이 0.3 s 끊겼을 때(탭을 닫았거나 브라우저가 멈춘 경우).
- 로봇이 중립을 확인해야 활성 명령을 보낸다(ACK `state=idle`). 순서는 "중립 → 확인 → 활성"이다.
- **자동 재무장**: 키를 누른 채(또는 팔 조종을 켠 채) 통신이 잠깐 끊기면 조종기가 위 순서를 스스로 다시 밟아 이어 간다. 해당 경우는 ACK가 0.30 s 없거나, 로봇 watchdog이 timeout됐거나, lease가 만료된 경우다.
  **비상정지·해제·fault·not_owner 뒤에는 자동으로 다시 움직이지 않는다.** 차체는 키를 모두 뗐다가 다시 눌러야 하고, 팔은 "팔 조종 켜기"를 다시 눌러야 한다. 비상정지가 걸리면 팔 조종 토글은 자동으로 꺼진다.
- 팔 조종을 켜면 로봇 현재값(`/armpi/status`의 `arm_current`)으로 목표를 맞춰 갑자기 튀지 않게 한다. rosbridge가 없으면 마지막 목표를 그대로 쓴다.
- 조종기가 죽으면 로봇 수신기의 watchdog(차체·팔 각 0.4 s)이 차체를 멈추고 팔을 현재 위치에 고정한다. 로봇 쪽 규칙이다.
- 조종 화면 서버는 기본으로 127.0.0.1에서만 열린다(`--bind`로 바꿀 수 있음). 다른 웹사이트가 조종 명령을 보내지 못하도록 다음 요청은 403으로 막는다: Host 이름이 IP 주소·localhost·이 기기 이름이 아닌 요청(DNS rebinding 방지), Origin이 `http://<Host>`와 다른 POST, Content-Type이 JSON이 아닌 POST.
- **조종권(화면 토큰)**: 여러 화면이 같은 서버를 열어도 마지막 입력이 이기지 않는다. 화면마다 임의 토큰(`screen`)을 만들어 요청에 싣고, 조종권이 비어 있을 때 실제 조작(키 누름·팔 방향·명령)을 보낸 화면이 조종권을 잡는다. 다른 화면은 "다른 기기가 조종 중"으로 표시되고 입력이 무시된다(명령은 409). 조종권은 그 화면이 1.5 s 동안 요청이 없으면 풀리고, 다른 화면은 "조종권 가져오기"(`take_control`)로만 가져온다. 가져오면 입력은 중립이 되고 팔 조종 토글이 꺼진다. **비상정지(`estop`, `robot_estop`)는 어느 화면에서나 받는다.**
- 입력에 번호(`n`)가 있으면 그보다 작은 번호의 늦게 도착한 입력은 버린다(느린 와이파이에서 순서가 뒤바뀌는 경우).
- **on-demand 세션**(로봇 안에서 `--bind 0.0.0.0`): 조종권을 잡은 화면이 없으면 v3 패킷을 보내지 않고, 화면이 사라지면 `disconnect`를 보낸다. 그래서 웹 조종기가 상주해도 VR 등 다른 조종기를 막지 않는다. 세션이 닫혀 있을 때 비상정지를 누르면 새 세션으로 비상정지를 보낸다(비소유자의 비상정지도 latch).

## 4. 가짜 로봇으로 Mac에서 시험하기 (실제 로봇 없이)

터미널 두 개를 연다.

```bash
# 터미널 1: 가짜 로봇 (UDP 5001/5002, 가짜 rosbridge 9090, 가짜 카메라 8080 — 모두 127.0.0.1)
python3 protocol_v3/mock_robot.py --bind 127.0.0.1

# 터미널 2: 조종기
python3 clients/mac_controller/armpi_mac_controller.py --robot 127.0.0.1            # 화면 http://127.0.0.1:18765
#   또는 검색 경로 확인: --robot auto --discover-target 127.0.0.1
```

브라우저에서 키를 누르면 가짜 로봇 터미널에 `차체 명령 수신: vx=… vy=… wz=…`, `팔 목표: x=…`, `비상정지 latch`, `거부 workspace_limit` 같은 줄이 찍힌다.
가짜 카메라는 PNG 프레임 스트림이다. 위쪽 띠의 색은 상태를 나타낸다(초록 = 활성, 빨강 = 비상정지). 노란 점은 위에서 본 팔 손끝 위치다.

가짜 로봇 옵션:

| 옵션 | 용도 |
|---|---|
| `--control-state missing\|udp_blocked\|fault` | 제어기(armpi_control) 상태를 흉내 낸다. missing·udp_blocked면 활성 명령이 `not_owner`, fault면 `fault`로 거부된다 |
| `--allowed-sources 10.0.0.0/8` | 허용 주소 목록. 목록 밖 주소의 패킷은 응답 없이 버린다 |
| `--workspace`, `--links`, `--home`, `--config 파일.json` | 작업 공간·링크 길이·기본 자세(실측 후 바꿀 값) |
| `--base-watchdog`, `--arm-watchdog`, `--lease` | 시간 값(기본 0.4/0.4/1.0 s) |
| `--rosbridge-port 0`, `--camera-port 0` | 가짜 rosbridge·카메라 끄기 |

조종기와 적합성 시험은 **동시에 돌리지 않는다.** 조종기가 세션을 잡고 있으면 시험이 `not_owner`를 받는다.

```bash
python3 protocol_v3/conformance_test.py --host 127.0.0.1 -v                                  # 기본
python3 protocol_v3/conformance_test.py --host 127.0.0.1 --rosbridge-port 9090 --interpretation -v  # 해석 시험·외부 비상정지까지
```

전체 자체 시험(가짜 로봇, 적합성 시험, 조종기 HTTP 종단 확인, 조건별 시험)은 다음 스크립트 하나로 다시 돌릴 수 있다.
`scripts/selftest_mac.sh` (결과는 (시험 기록). 2026-11-02 기록은 (시험 기록)).

## 5. 실제 로봇에 연결하기 (아직 하지 않음)


1. 로봇과 맥을 같은 망에 둔다(목표 Wi-Fi: 1순위 와이파이, 또는 유선 인터넷 공유). 로봇의 `allowed_sources`가 비어 있지 않으면 맥 주소가 포함됐는지 확인한다.
2. 로봇에서 v3 수신기를 단독으로 띄웠다면 먼저 적합성 시험을 돌린다. 제어기 없이 수신기만 띄운 경우:
   `python3 protocol_v3/conformance_test.py --host <로봇IP> --condition control_missing -v`
   제어기까지 띄웠고 UDP가 허용된 상태라면 `--condition normal`을 쓴다. 이 경우 바퀴가 돌고 팔이 움직이므로 아래 안전 준비를 먼저 마친다. 링크 치수를 실측값으로 바꿨다면 `--workspace`, `--reachable`, `--unreachable`도 로봇 설정에 맞춘다.
3. `python3 clients/mac_controller/armpi_mac_controller.py --robot <로봇IP>`(또는 `--robot auto`)를 실행한다.
4. 처음에는 속도 10~30 %, 팔은 짧게 눌러 목표가 조금씩 움직이는지 본다. 화면의 "로봇 현재" 값이 목표를 따라가는지 확인한다.

## 6. 안전 주의 (실제 로봇)

- **처음에는 반드시 바퀴를 띄운다**(차체 밑에 받침을 두어 바퀴가 바닥에 닿지 않게). 메카넘은 옆·대각선으로도 움직인다.
- **팔이 닿는 범위를 비운다.** 손과 케이블이 그리퍼·관절 사이에 끼지 않게 한다. 링크 길이가 아직 placeholder라서 IK 판정과 실제 자세가 다를 수 있다.
- **물리적 비상정지 수단을 손에 둔다**(전원 스위치·배터리 분리). 화면의 Space 비상정지는 소프트웨어 정지라서 로봇 프로그램이 멈췄을 때는 듣지 않는다.
- 비상정지가 걸려도 팔은 토크를 유지한 채 그 자리에 선다(떨어지지 않게 하는 사양). 팔을 손으로 옮기려면 전원을 끈다.
- 속도 상한(차체 0.25 m/s, 1.2 rad/s, 팔 0.15 m/s)은 사용자 결정 2026-11-04, 바퀴 방향 실기 시험 전에 올린 값이다(이전 0.10/0.60/0.10). 처음에는 속도를 낮춰 쓴다.
- 배터리 전압이 낮으면(상태 표 확인) 서보 동작이 불안정할 수 있다. 시험을 멈추고 충전한다.
- 다른 조종기(VR 등)가 이미 조종 중이면 이쪽은 `not_owner`가 된다. 한 번에 한 조종기만 쓴다.

## 7. HTTP API (자동 시험용)

브라우저 대신 다른 프로그램으로도 조종할 수 있다. 요청에는 `Content-Type: application/json`이 필요하다.

| 요청 | 본문·응답 |
|---|---|
| `GET /`, `/index.html`, `/mobile` | PC 화면, 휴대폰 화면. 휴대폰 브라우저(User-Agent)로 `/`에 오면 `/mobile`로 보낸다(302) |
| `GET /manifest.webmanifest`, `/icon-192.png`, `/icon-512.png` | 홈 화면 추가용 |
| `GET /api/config` | 로봇 주소, 영상 URL, 기본 자세, 속도, 세션 방식(`session_mode`) |
| `GET /api/state?screen=<토큰>` | UDP 세션·채널 상태, 팔 목표·마지막 수락·마지막 거부, ACK 지연, rosbridge 상태(`/armpi/status` 원문 포함), `control`(조종권: `owner`, `you`, 최근 화면 목록), `session`(`mode`, `live`), 최근 사건 |
| `POST /api/input` | `{"f":-1~1,"s":-1~1,"t":-1~1,"held":bool,"focused":bool,"arm":{"x":-1/0/1,"y":…,"z":…,"p":…,"g":…},"screen":"토큰","screen_name":"이름","n":번호}`. 50 ms마다 보낸다(0.3 s 끊기면 중립). 응답 `control:false` = 다른 화면이 조종 중이라 무시됨 |
| `POST /api/cmd` | `{"cmd": "estop"\|"clear"\|"disconnect"\|"connect"\|"arm_enable"(+on)\|"arm_target"(+x,y,z,pitch[rad],gripper)\|"gripper"(+value)\|"arm_sync"\|"arm_home_target"\|"robot_estop"\|"robot_reset"\|"arm_home"\|"joint_debug"(+positions 6개 rad)\|"take_control"\|"release_control", "screen":…}`. 조종권이 없으면 409 |

`screen`을 빼면 모두 `"api"` 한 화면으로 본다(예전 스크립트도 그대로 동작).

```bash
curl -s -H 'Content-Type: application/json' -d '{"cmd":"estop"}' http://127.0.0.1:18765/api/cmd
```

## 8. 알려진 한계

- `/armpi/status`의 JSON 키는 문서(02)에 항목만 있고 이름이 정해지지 않았다. 조종기와 가짜 로봇은 다음 키를 쓴다: `owner`, `owner_addr`, `base_state`, `arm_state`, `emergency_stop`, `estop_sources`, `fault`, `battery_v`, `arm_target{x,y,z,pitch,gripper}`, `arm_current{…}`, `position_source`. C++ 제어기와 키 이름을 맞춰야 한다. 키가 없으면 해당 칸이 "-"로 나오고 조종은 계속된다.
- 가짜 카메라는 PNG multipart 스트림이다. 실제 web_video_server(JPEG)와 형식이 다르다. 브라우저 표시는 이번에 눈으로 확인하지 않았다(HTTP 헤더·첫 프레임만 확인).
- 여기서 통과한 시험은 가짜 로봇 기준이다. 실제 로봇·실제 Wi-Fi 지연·실제 서보 동작은 확인하지 않았다.
- 휴대폰 화면(`/mobile`)은 헤드리스 Chrome의 휴대폰 크기 흉내와 터치 이벤트로만 확인했다. 실제 휴대폰·Android WebView·iPhone Safari는 시험하지 않았다((시험 기록)).
- `--bind 0.0.0.0`에는 인증이 없다. 같은 망의 누구나 화면을 열고 조종권을 가져올 수 있다(로봇 제어 UDP와 같은 수준).
