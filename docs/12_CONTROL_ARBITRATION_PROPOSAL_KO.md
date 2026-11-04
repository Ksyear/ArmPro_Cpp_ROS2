# 12. 제어권 중재·네트워크 노출 정리 제안

> **제안 문서다. 사람이 결정한다.** 이 문서를 쓰면서 소스·설정은 하나도 바꾸지 않았다.
> 이미 구운 USB 이미지(소스 트리 `1a7800bd…`)가 첫 실기의 기준판이다. 바꾸는 것은 첫 실기 뒤에, 새 해시와 재시험 기록을 남기고 한다.

작성 2026-11-03 (Asia/Seoul). 대상: `NEXT_ACTIONS.md` P0 #4, `11_KNOWN_LIMITATIONS_KO.md` 2절 7번·4절 2–3번·0.4절 2번, `07_COMMUNICATION_ARCHITECTURE_KO.md` 5·9절.
증거 표기는 `11_KNOWN_LIMITATIONS_KO.md`와 같다([코드], [시험 로그], [추정], [미확인]).

## 1. 지금 로봇을 움직이거나 끌 수 있는 경로

| 경로 | 할 수 있는 일 | 지금 막는 것 | 근거 |
|---|---|---|---|
| UDP 5001 (Protocol v3) | 차체·팔 조종, 비상정지 latch·해제(소유자) | 세션 규칙(소유권·lease 1.0 s·중립 확인·watchdog 0.4 s), `allowed_sources`(기본 비어 있음 = 모두 허용, IPv4 출처 위조 가능) | [코드] `armpi.yaml`, `armpi_vr_bridge` |
| 로봇 안 웹 조종기 TCP 18765 → UDP `127.0.0.1:5001` | 위와 같음(휴대폰 브라우저·Android 앱) | v3 세션 규칙은 거친다. **화면 인증 없음.** 출처가 항상 127.0.0.1이라 `allowed_sources`로 못 막는다 | [코드] `deploy/systemd/armpi-web.service` |
| rosbridge TCP 9090 | `/armpi/cmd_vel`·`/armpi/arm/target`·`/armpi/arm/joint_debug` 발행, `/armpi/estop_request`(`latch:false`로 일부 출처 해제), `/armpi/reset_estop`(전체 해제)·`/armpi/arm/home`·`/armpi/emergency_stop` 호출, `/armpi_vr_bridge/state` 위조(제어기의 소유자 판단에 쓰임), rosapi로 매개변수 변경 | 제어기 2차 watchdog 0.6 s, `header.stamp` 나이 0.5 s 검사뿐. 로봇 시각은 rosapi `GetTime` 등으로 알 수 있으므로 stamp 검사는 보안 장치가 아니다 [추정] | [코드] `control_node.cpp`, `armpi.launch.py`, rosbridge_server 2.0.8 |
| DDS (UDP 7400+, `ROS_LOCALHOST_ONLY=0`, `ROS_DOMAIN_ID=0`) | rosbridge와 같은 일 전부 + `/armpi/shutdown_requested` 발행 → `armpi-shutdown.service`(root)가 `systemctl poweroff` | 없음 | [코드] `deploy/systemd/armpi.service`, `deploy/systemd/armpi-shutdown.service` |
| SSH 22 | 전부(비밀번호 로그인, NOPASSWD sudo) | 비밀번호 | [코드] 이 문서 범위 밖. `11` 6절 |

참고: 매개변수는 노드가 시작할 때만 읽는다(`get_parameter` 재조회·변경 콜백 없음) [코드]. 그래서 실행 중 매개변수를 바꿔도 동작은 바뀌지 않는다 [추정]. 다만 `ros2 param get`으로 보이는 값이 실제 동작값과 달라질 수 있다.

### 정상 사용자가 쓰는 경로

| 사용자 | UDP 5001 | rosbridge 9090 | DDS | 18765 |
|---|---|---|---|---|
| Mac 조종기(원격) | 조종 | 상태·배터리 구독, `joint_debug` 발행, 비상정지·해제·기본 자세 서비스 | — | (자기 Mac 안) |
| 로봇 안 웹 조종기 | 127.0.0.1 | 127.0.0.1 | — | 화면 제공 |
| 휴대폰 브라우저·Android 앱 | — | — | — | 접속 |
| VR(Unity) 앱 (`docs/08`) | 조종 | 상태 표시·서비스(선택) | — | — |
| 개발자 `ros2` 명령 | — | — | 원격 디버깅 | — |

## 2. 판단 기준 (제안)

- G1. 바퀴·팔을 움직이는 명령은 v3 세션 한 길로만 들어온다(소유권·중립 확인 우회 불가).
- G2. 비상정지는 여러 길로 걸 수 있게 둔다(안전 쪽). 해제는 좁힌다.
- G3. 로봇 전원 끄기는 망에서 할 수 없다(KEY2 또는 SSH).
- G4. 지금 조종기(Mac, 웹·Android, VR)의 기능을 되도록 잃지 않는다.
- G5. 바꾼 뒤에는 새 소스 해시와 재시험 기록을 남긴다. `ros2_ws/src` 안을 고치면 이미지 트리 해시가 바뀐다.

## 3. 선택지

### A. 운영 규칙만 (코드·설정 변경 없음)

- 실험은 전용 공유기나 로봇 대체 AP에서 한다. 실험 중 그 망에 연구 장비 말고는 붙이지 않는다.
- 필요하면 로봇에서 그날만 `sudo systemctl stop armpi-shutdown.service`(KEY2 종료 끔), `sudo systemctl stop armpi-web.service`(웹 조종기 끔)를 한다. 시험 기록에 적는다.
- 장점: 지금 바로 된다. 실기 기준판을 건드리지 않는다.
- 단점: 사람이 지켜야만 효과가 있다. 1순위 와이파이 같은 공용 망에서는 1절 위험이 그대로다. G1·G3를 코드로 보장하지 않는다.
- 바꿀 파일: 없음. 시험 기록 양식(`docs/10` 9절)에 "망 종류·끈 서비스" 칸을 추가하는 정도.

### B. 망 노출 줄이기 (노드 코드는 그대로, systemd·launch 설정만)

| 항목 | 내용 | 효과 | 잃는 것 | 바꿀 파일 |
|---|---|---|---|---|
| B1 DDS를 로봇 안으로 | `armpi.service`·`armpi-shutdown.service` 둘 다 `Environment=ROS_LOCALHOST_ONLY=1` | 같은 망의 ROS 2 기기가 토픽·서비스에 못 닿는다. KEY2 종료 토픽도 로봇 안에서만 | 원격 `ros2` 디버깅(SSH로 들어가서 하면 됨) | `deploy/systemd/armpi.service`, `deploy/systemd/armpi-shutdown.service` |
| B2 rosbridge 허용 목록 | rosbridge를 따로 띄우고 `topics_pub_glob:="['/armpi/arm/joint_debug']"`, `services_glob:="['/armpi/emergency_stop', '/armpi/reset_estop', '/armpi/arm/home']"`(rosapi는 rosbridge가 자동 추가), 구독은 `/armpi/status`·`/armpi/battery`·`/joint_states`만 | rosbridge로 `cmd_vel`·`arm/target`·`estop_request`·`vr_bridge/state`·`shutdown_requested` 발행이 막힌다. Mac 조종기 기능은 그대로 | 다른 토픽을 쓰는 새 도구는 목록에 추가해야 함. `joint_debug`·`reset_estop`은 여전히 열림 | `armpi.service` ExecStart에 `rosbridge:=false` + 새 `deploy/systemd/armpi-rosbridge.service`(→ `ros2_ws/src` 해시 그대로). 또는 `armpi.launch.py`에 인자 추가(해시 바뀜) |
| B3 rosbridge를 로봇 안으로 | `address:=127.0.0.1` | 원격에서 9090 접속 불가. 로봇 안 웹 조종기는 그대로 | **원격 Mac 조종기·VR 앱의 상태 표시·서비스 버튼**(조종은 됨). 필요하면 `ssh -L 9090:127.0.0.1:9090`으로 터널 | B2와 같음 |
| B4 출처 주소 제한 | `allowed_sources: ["<Mac IP>/32", "<VR PC IP>/32", "127.0.0.1/32"]` | 그 밖 주소의 UDP 무시 | 위조 가능. DHCP 주소가 바뀌면 조종 불가. 실험 망 주소는 실기 날에 정해짐 | `armpi.yaml` 또는 `/etc/armpi/` 덮어쓰기 파일 + `armpi.service`의 `params_file:=` |

- `rosbridge_server` 2.0.8 launch에 `address`·`topics_pub_glob`·`topics_sub_glob`·`services_glob` 인자가 있음을 개발 컨테이너에서 확인했다 [코드: `/opt/ros/humble/share/rosbridge_server/launch/rosbridge_websocket_launch.xml`]. 지금 `armpi.launch.py`는 `port`만 넘긴다.
- B1은 Humble에서 유효하다. 같은 기기 안에서 한쪽만 `ROS_LOCALHOST_ONLY=1`이면 통신이 안 될 수 있다 [미확인]. 그래서 두 서비스를 같은 값으로 바꾼다.

### C. 제어기가 명령 출처를 확인 (노드 코드 변경)

| 항목 | 내용 | 장점 | 단점 |
|---|---|---|---|
| C1 발행자 확인 | 제어기가 `cmd_vel`·`arm/target` 메시지의 발행자 GID를 보고, 노드 이름 `armpi_vr_bridge`의 발행자만 받는다 | rosbridge·다른 노드 경로를 코드로 막음(G1) | DDS가 열려 있으면 같은 이름 노드를 만들어 흉내 낼 수 있다. **B1과 함께 써야 의미가 있다.** 구현·시험 비용 중간 |
| C2 소유자 없을 때 거부 | `/armpi_vr_bridge/state`에 소유자가 없으면 `cmd_vel`·`arm/target`을 버린다 | 구현이 작다 | `/armpi_vr_bridge/state`도 위조 가능. 세션이 있는 동안의 끼어들기는 못 막음 |
| C3 한 프로세스로 합치기 | 수신기와 제어기를 한 프로세스(컴포넌트 + intra-process)로 띄우고 명령을 토픽 대신 함수 호출로 전달 | 망·다른 노드에서 명령 주입 불가(가장 강함) | 구조 변경이 크다. 지금 시험(통합 16×2, 종단 3, 적합성)을 다시 짜야 한다 |

- 바꿀 파일: `armpi_control/src/control_node.cpp`(+ 시험), C3은 `armpi_vr_bridge`·`armpi_bringup`까지.

### D. 웹 조종기(18765) 접근 제한

| 항목 | 내용 | 잃는 것 | 바꿀 파일 |
|---|---|---|---|
| D1 끄기 | 실험 때 `armpi-web.service`를 끈다(또는 `ENABLED`에서 뺌) | 휴대폰·Android 앱 조종 | `deploy/systemd/ENABLED` 또는 운영 규칙(A) |
| D2 로봇 안으로만 | `--bind 127.0.0.1` + SSH 터널 | 휴대폰에서 바로 접속 | `deploy/systemd/armpi-web.service` |
| D3 연결 코드 | 첫 접속 때 짧은 코드(예: 로봇 로그·부저 패턴으로 확인)를 넣어야 조종권을 잡게 함 | 구현 필요 | `clients/mac_controller/armpi_mac_controller.py`, `mobile.html`(이미지의 `/opt/armpi/mac_controller` 트리 해시 바뀜) |

### E. KEY2 종료를 망에서 떼기

| 항목 | 내용 | 장점 | 단점 | 바꿀 파일 |
|---|---|---|---|---|
| E1 | B1 적용(두 서비스 `ROS_LOCALHOST_ONLY=1`) | 설정만 | 로봇 안 프로세스는 여전히 토픽으로 끌 수 있음(로봇 안 rosbridge 포함 → B2와 같이) | systemd 2개 |
| E2 | 제어기가 토픽 대신 `/run/armpi/shutdown_requested` 파일을 만들고, systemd `.path` 유닛이 그 파일을 보고 끈다 | 망·ROS 경로와 완전히 분리 | 제어기 코드 변경 | `control_node.cpp`, 새 `deploy/systemd/armpi-shutdown.path`, `armpi-shutdown.service` |
| E3 | KEY2 종료를 쓰지 않는다(`armpi-shutdown.service` 비활성) | 바로 됨 | 버튼으로 안전 종료 불가. 전원 스위치로 끄면 저장장치 손상 위험 → SSH `sudo poweroff` 습관 필요 | `deploy/systemd/ENABLED` 또는 운영 규칙(A) |

## 4. 조합 제안 (결정은 사람)

| 시점 | 제안 | 이유 |
|---|---|---|
| 첫 실기 날 | **A** + 필요하면 그날만 E3·D1(서비스 정지, 기록) | 기준판을 바꾸지 않는다. 첫날 목적은 부팅·부호·비상정지 확인이다 |
| 첫 실기 뒤 다음 이미지 | **B1 + B2** (+ E1) | 노드 코드를 바꾸지 않고 G1·G3 대부분을 얻는다. Mac 조종기·VR 앱 기능을 잃지 않는다 |
| 사람 대상 실험 전 | B4는 실험 망 주소가 정해진 뒤. D2 또는 D3로 웹 조종기 정리. C1 또는 E2는 실험 설계가 정해진 뒤 필요성 판단 | 공용 망에서 실험하면 C·D가 필요해진다 |

사람이 정할 것:
1. 실험 망: 1순위 와이파이, 대체 AP, 전용 공유기 중 무엇인가().
2. 원격 Mac 조종기·VR 앱이 rosbridge 상태 표시를 꼭 써야 하나(B3 가능 여부).
3. 휴대폰 조종을 실험에 쓰나(D1 가능 여부).
4. KEY2 버튼 종료를 남기나(E2 대 E3).

## 5. 시험 방법 (로봇 없이 먼저, 그 다음 실기)

| 선택지 | 로봇 없이(컨테이너·가짜 하드웨어) | 실기 |
|---|---|---|
| B1 | 컨테이너 두 개를 같은 docker 망에 둔다. A: `armpi_mock.launch.py` + `ROS_LOCALHOST_ONLY=1`. B: `ros2 topic pub --once /armpi/shutdown_requested std_msgs/msg/Empty`, `ros2 topic pub /armpi/cmd_vel …` → A에서 받지 않아야 한다. A 안에서 같은 명령 → 받아야 한다 | Mac에서 `ros2 topic list`가 로봇 토픽을 못 봐야 한다. KEY2 2 s → 종료되는지 |
| B2·B3 | 가짜 로봇 스택 + 별도 rosbridge(새 인자)에 Python WebSocket 클라이언트로 `advertise`/`publish /armpi/cmd_vel` → 거부 응답, `call_service /armpi/emergency_stop` → 성공, `/armpi/status` 구독 → 수신. 기존 종단 시험(`armpi_bringup/test/test_mock_launch_e2e.py`) 3/3 유지 | Mac 조종기 상태 줄 "rosbridge 연결", 비상정지·해제 버튼 동작 |
| B4 | 기존 gtest `test_source_filter`(7개) + `conformance_test.py`를 허용·비허용 주소에서 실행 | 허용 안 된 기기의 조종이 ACK 없이 무시되는지 |
| C1·C2 | 제어기 통합 시험에 "다른 노드가 `cmd_vel` 발행 → 무시" 경우 추가. 기존 16×2 유지 | V절 반복 |
| D1–D3 | `apps` 시험(가짜 로봇)으로 화면 접속·조종권 확인 | 휴대폰 접속 |
| E2 | 가짜 하드웨어 `/armpi_control/mock_cmd`에 `gpio <key2_bcm> 0`(KEY2 누름, 시험용 프로필은 26) → 2 s 뒤 `gpio <key2_bcm> 1` → 파일 생성 확인(실제 poweroff 대신 `.path` 유닛을 시험용 서비스에 연결) | KEY2 2 s → 종료 |

바꾼 뒤 공통: 소스 트리 해시와 systemd 파일 해시를 다시 적고, (시험 기록)처럼 전체 재시험을 남긴다. 부호·속도 같은 실기 결과는 바꾸기 전 기준판에서 먼저 얻는다.
