#!/usr/bin/env bash
# armpi_hardware / armpi_control / armpi_bringup 을 개발 컨테이너(Ubuntu 22.04 arm64, ROS 2 Humble)에서 빌드·시험한다.
#   ros2_ws/src/armpi_bringup/test/dev_build_and_test.sh [unit|integration|all]   (기본 all)
#
# - 이미지: armpi-ctl-dev:humble (없으면 armpi-dev:humble 위에 libgpiod-dev·usb_cam·web_video_server·rosbridge 를 더해 만든다)
# - 소스(외장 exFAT·virtiofs 공유 폴더)는 읽기 전용으로 붙이고 컨테이너 /tmp/ws 로 복사해 빌드한다.
# - --network none: 루프백만 쓴다. 실제 로봇에 접속하지 않는다. 실제 장치(/dev/i2c-1 등)는 컨테이너에 없다.
# - 결과: work/logs/armpi_control_<모드>_<시각>/ (run.log, 빌드 로그, gtest XML, 통합 시험 로그)
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
PROJ="$(cd "$HERE/../../../.." && pwd)"   # 저장소 루트
MODE="${1:-all}"
case "$MODE" in unit|integration|all) ;; *) echo "사용법: $0 [unit|integration|all]" >&2; exit 2 ;; esac
IMAGE="${ARMPI_CTL_DEV_IMAGE:-armpi-ctl-dev:humble}"
STAMP="$(date +%Y%m%d_%H%M%S)"
RES="$PROJ/work/logs/armpi_control_${MODE}_${STAMP}"
mkdir -p "$RES"

if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
  BASE="armpi-dev:humble"
  docker image inspect "$BASE" >/dev/null 2>&1 || BASE="ros:humble-ros-base-jammy"
  echo "== 개발 이미지 $IMAGE 생성 (기반 $BASE)"
  docker build -t "$IMAGE" - <<DOCKER
FROM $BASE
RUN apt-get update && apt-get install -y --no-install-recommends \
      libgpiod-dev gpiod linux-libc-dev python3-pytest procps psmisc iproute2 \
      ros-humble-rosbridge-server ros-humble-usb-cam ros-humble-image-transport-plugins \
    && (apt-get install -y --no-install-recommends ros-humble-web-video-server || echo "web_video_server 없음") \
    && rm -rf /var/lib/apt/lists/*
DOCKER
fi

NAME="armpi-ctl-test-${STAMP}-$$"
set +e
docker run --name "$NAME" --network none \
  -v "$PROJ/ros2_ws/src:/src:ro" -v "$PROJ/protocol_v3:/v3:ro" -v "$PROJ/clients/mac_controller:/mac:ro" \
  -e MODE="$MODE" -e ROS_LOCALHOST_ONLY=1 -e ROS_DOMAIN_ID=79 -e PYTHONDONTWRITEBYTECODE=1 \
  "$IMAGE" bash -c '
set -o pipefail
source /opt/ros/humble/setup.bash
mkdir -p /results /tmp/ws/src && cd /tmp/ws
cp -r /src/. src/   # 워크스페이스 전체(의존 패키지는 colcon 이 --packages-up-to 로 고른다)
echo "== 환경: $(uname -m), $(. /etc/os-release; echo $PRETTY_NAME), $(g++ --version | head -1), ROS $ROS_DISTRO, libgpiod $(pkg-config --modversion libgpiodcxx 2>/dev/null || echo 없음)"
(cd src && find armpi_common armpi_hardware armpi_control armpi_bringup -type f | sort | xargs sha256sum) > /results/source_sha256.txt
echo "== 소스 해시 $(wc -l < /results/source_sha256.txt)개 → source_sha256.txt"

echo "== colcon build (armpi_kinematics, armpi_interfaces, armpi_hardware, armpi_control, armpi_bringup)"
colcon build --merge-install --packages-up-to armpi_control armpi_bringup \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo --event-handlers console_direct- > /results/colcon_build.txt 2>&1
rc_build=$?
tail -8 /results/colcon_build.txt
for p in armpi_hardware armpi_control; do cp log/latest_build/$p/stdout_stderr.log /results/build_$p.log 2>/dev/null; done
nwarn=$(cat /results/build_armpi_hardware.log /results/build_armpi_control.log 2>/dev/null | grep -c "warning:")
echo "   컴파일 경고 수(armpi_hardware + armpi_control): ${nwarn:-0}"
if [ $rc_build -ne 0 ]; then echo "빌드 실패"; tail -40 /results/colcon_build.txt; exit 10; fi
echo "== colcon build armpi_vr_bridge·armpi_web_gateway·armpi_power (종단 시험용)"
colcon build --merge-install --packages-up-to armpi_vr_bridge armpi_web_gateway armpi_power --event-handlers console_direct- > /results/colcon_build_vr_bridge.txt 2>&1
have_bridge=$?; [ $have_bridge -eq 0 ] && echo "   armpi_vr_bridge 빌드 성공" || echo "   armpi_vr_bridge 빌드 실패 → 종단 시험 건너뜀"
source install/setup.bash
rc=0

if [ "$MODE" = unit ] || [ "$MODE" = all ]; then
  echo "== 단위 시험 (gtest)"
  colcon test --merge-install --packages-select armpi_hardware armpi_control --event-handlers console_direct- \
    > /results/colcon_test.txt 2>&1
  mkdir -p /results/unit
  cp build/armpi_hardware/test_results/armpi_hardware/*.xml build/armpi_control/test_results/armpi_control/*.xml /results/unit/ 2>/dev/null
  python3 - <<PY | tee /results/unit_summary.txt
import glob, xml.etree.ElementTree as ET
tot = fail = err = 0
for f in sorted(glob.glob("/results/unit/*.gtest.xml")):
    r = ET.parse(f).getroot()
    t, fa, e = (int(r.get(k, 0)) for k in ("tests", "failures", "errors"))
    tot += t; fail += fa; err += e
    print("   %-36s 시험 %3d, 실패 %d, 오류 %d" % (f.split("/")[-1], t, fa, e))
    for tc in r.iter("testcase"):
        for fl in tc.findall("failure"):
            print("     실패: %s.%s: %s" % (tc.get("classname"), tc.get("name"), (fl.get("message") or "")[:300]))
print("   gtest 합계: %d 개, 실패 %d, 오류 %d" % (tot, fail, err))
PY
  colcon test-result --test-result-base build/armpi_control > /dev/null 2>&1 || rc=20
  colcon test-result --test-result-base build/armpi_hardware > /dev/null 2>&1 || rc=20
fi

if [ "$MODE" = integration ] || [ "$MODE" = all ]; then
  CFG=install/share/armpi_bringup/config/armpi.yaml
  echo "== 설정 파일 검사 ($CFG)"
  python3 - "$CFG" <<PY 2>&1 | tee /results/config_check.txt || rc=$((rc + 25))
import sys, yaml
d = yaml.safe_load(open(sys.argv[1]))
shared = d["/**"]["ros__parameters"]; ctl = d["armpi_control"]["ros__parameters"]; br = d["armpi_vr_bridge"]["ros__parameters"]
dup = sorted((set(shared) & set(ctl)) | (set(shared) & set(br)))
assert not dup, "공유(/**) 키가 노드 구역에 다시 있음: %s" % dup
assert "hardware.mock" not in ctl, "hardware.mock 이 노드 구역에 있으면 launch mock 인자가 무시됨"
for k in ("arm.kinematics.upper_arm_m", "arm.workspace.x_min", "chassis.max_vx_m_s"):
    assert k in shared, k
import os
cfgdir = os.path.dirname(sys.argv[1])
def keys(f):
    dd = yaml.safe_load(open(os.path.join(cfgdir, f)))
    return {k for sec in dd.values() for k in sec["ros__parameters"]}
tk, pk = keys("test_hardware_profile.yaml"), keys("hardware_profile.template.yaml")
assert tk == pk, "시험용 프로필과 본보기의 키가 다름: %s" % sorted(tk ^ pk)
assert not ((set(shared) | set(ctl) | set(br)) & pk), "하드웨어 프로필 키가 armpi.yaml 에 있음: %s" % sorted((set(shared) | set(ctl) | set(br)) & pk)
if os.path.isfile(os.path.join(cfgdir, "hardware_profile.yaml")):
    rk = keys("hardware_profile.yaml")
    assert rk == pk, "hardware_profile.yaml 과 본보기의 키가 다름: %s" % sorted(rk ^ pk)
    print("   OK: hardware_profile.yaml 키 %d개 = 본보기" % len(rk))
print("   OK: 공유 키 %d개, armpi_control %d개, armpi_vr_bridge %d개, 프로필 키 %d개(armpi.yaml 에 없음)" % (len(shared), len(ctl), len(br), len(pk)))
PY
  mkdir -p /results/itest_default /results/itest_yaml
  cd /tmp/ws/src/armpi_control/test/integration
  echo "== 통합 시험 1: armpi_control_node(mock), 노드 기본값"
  ARMPI_ITEST_LOG_DIR=/results/itest_default timeout 300 python3 -m pytest -q -s -p no:cacheprovider \
    --junitxml=/results/itest_default/junit.xml test_control_integration.py 2>&1 | tee /results/itest_default/pytest.txt | tail -4
  [ ${PIPESTATUS[0]} -eq 0 ] || rc=$((rc + 30))
  echo "== 통합 시험 2: armpi_control_node(mock), config/armpi.yaml"
  ARMPI_PARAMS_FILE=/tmp/ws/$CFG ARMPI_ITEST_LOG_DIR=/results/itest_yaml timeout 300 python3 -m pytest -q -s -p no:cacheprovider \
    --junitxml=/results/itest_yaml/junit.xml test_control_integration.py 2>&1 | tee /results/itest_yaml/pytest.txt | tail -4
  [ ${PIPESTATUS[0]} -eq 0 ] || rc=$((rc + 30))
  cd /tmp/ws

  echo "== 실제 하드웨어 모드 시작 확인: (1) 하드웨어 프로필 없음 → 장치를 열지 않고 fault, (2) 시험용 프로필 + 장치 없음 → fault, (3) 본보기 그대로 → 장치를 열지 않고 fault"
  python3 - <<PY 2>&1 | tee /results/real_mode_check.txt || rc=$((rc + 40))
import json, os, signal, subprocess, time, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
share = "/tmp/ws/install/share/armpi_bringup/config/"
rclpy.init(); got = {}
def run(tag, files):
    got.clear()
    n = Node("real_mode_probe_" + tag)  # 실행마다 새 구독(앞 실행의 메시지를 읽지 않게)
    n.create_subscription(String, "/armpi/control_state", lambda m: got.update(cs=json.loads(m.data)),
                          QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    log = open("/results/real_mode_node_%s.log" % tag, "w")
    cmd = ["ros2", "run", "armpi_control", "armpi_control_node", "--ros-args"]
    for f in files: cmd += ["--params-file", share + f]
    p = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    t0 = time.time()
    while time.time() - t0 < 10 and "cs" not in got: rclpy.spin_once(n, timeout_sec=0.1)
    time.sleep(1.0); alive = p.poll() is None
    os.killpg(p.pid, signal.SIGINT); p.wait(timeout=10)
    n.destroy_node()
    cs = got.get("cs"); print("   [%s] control_state:" % tag, cs); print("   [%s] 프로세스 유지:" % tag, alive, "종료 코드:", p.returncode)
    return cs, alive
cs, alive = run("no_profile", ["armpi.yaml"])
assert cs and cs["fault"] is True and "하드웨어 프로필 값 없음" in cs["fault_reason"] and alive
assert "/dev/i2c-1" not in cs["fault_reason"]  # 프로필이 없으면 장치를 열지 않는다
cs, alive = run("test_profile", ["armpi.yaml", "test_hardware_profile.yaml"])
assert cs and cs["fault"] is True and "/dev/i2c-1" in cs["fault_reason"] and alive
cs, alive = run("template", ["armpi.yaml", "hardware_profile.template.yaml"])  # 본보기(빈 값)를 그대로 넣었을 때
assert cs and cs["fault"] is True and "하드웨어 프로필 값 없음" in cs["fault_reason"] and alive
assert "/dev/i2c-1" not in cs["fault_reason"] and "chassis.a_mm" in cs["fault_reason"]
if os.path.isdir("/tmp/ws/install/lib/armpi_vr_bridge"):
    # VR 수신기도 본보기(빈 값)로 죽지 않아야 한다(경고 후 제한 없는 기본값으로 사전 확인)
    blog = "/results/real_mode_bridge_template.log"
    with open(blog, "w") as lf:
        b = subprocess.Popen(["ros2", "run", "armpi_vr_bridge", "armpi_vr_bridge_node", "--ros-args",
                              "--params-file", share + "armpi.yaml", "--params-file", share + "hardware_profile.template.yaml"],
                             stdout=lf, stderr=subprocess.STDOUT, start_new_session=True)
        time.sleep(4.0); b_alive = b.poll() is None
        os.killpg(b.pid, signal.SIGINT); b.wait(timeout=10)
    btxt = open(blog).read()
    print("   [bridge_template] 프로세스 유지:", b_alive)
    assert b_alive and "하드웨어 프로필 값 없음" in btxt, btxt[-600:]
print("   OK: 프로필이 없거나 본보기 그대로면 장치를 열지 않고 fault, 프로필이 있고 장치가 없으면 fault 로 보고하고 계속 실행")
PY

  if [ $have_bridge -eq 0 ]; then
    echo "== 종단 시험: armpi_mock.launch.py (UDP v3 클라이언트 → 수신기 → 제어기 → 가짜 바퀴·서보, rosbridge)"
    mkdir -p /results/e2e
    cd /tmp/ws/src/armpi_bringup/test
    ARMPI_V3_CLIENT_DIR=/v3 ARMPI_ITEST_LOG_DIR=/results/e2e timeout 300 python3 -m pytest -q -s -p no:cacheprovider \
      --junitxml=/results/e2e/junit.xml test_mock_launch_e2e.py 2>&1 | tee /results/e2e/pytest.txt | tail -4
    [ ${PIPESTATUS[0]} -eq 0 ] || rc=$((rc + 50))
    cd /tmp/ws
    echo "== 종단 시험: 웹 게이트웨이(HTTP → v3 UDP → 수신기 → 제어기 → 가짜 바퀴), 파이썬 조종기와 JSON 모양 비교"
    mkdir -p /results/e2e_gateway
    cd /tmp/ws/src/armpi_web_gateway/test/integration
    ARMPI_MAC_CONTROLLER_DIR=/mac ARMPI_V3_CLIENT_DIR=/v3 ARMPI_ITEST_LOG_DIR=/results/e2e_gateway timeout 300 \
      python3 -m pytest -q -s -p no:cacheprovider --junitxml=/results/e2e_gateway/junit.xml test_gateway_e2e.py 2>&1 \
      | tee /results/e2e_gateway/pytest.txt | tail -4
    [ ${PIPESTATUS[0]} -eq 0 ] || rc=$((rc + 70))
    cd /tmp/ws
    echo "== 종단 시험: 파이썬 조종기용 HTTP API 확인 항목을 C++ 게이트웨이에(세션 always, 카메라 제외)"
    mkdir -p /results/e2e_gateway_check
    setsid ros2 launch armpi_bringup armpi_mock.launch.py rosbridge:=false > /results/e2e_gateway_check/mock_stack.log 2>&1 &
    STACKPID=$!
    setsid ros2 run armpi_web_gateway armpi_web_gateway_node --ros-args -p http_bind:=127.0.0.1 -p http_port:=18765 \
      -p session:=always > /results/e2e_gateway_check/gateway_node.log 2>&1 &
    GWPID=$!
    sleep 7
    timeout 120 python3 -B /mac/test/e2e_mac_controller_check.py --http 127.0.0.1:18765 --camera "" \
      --json /results/e2e_gateway_check/result.json > /results/e2e_gateway_check/check.txt 2>&1
    rc_chk=$?
    # on-demand 게이트웨이(18766): 연결 없이 비상정지 → 세션 이유 estop_only, 조종권 없음, 2 s 뒤 닫힘
    setsid ros2 run armpi_web_gateway armpi_web_gateway_node --ros-args -r __node:=armpi_web_gateway_ondemand \
      -p http_bind:=127.0.0.1 -p http_port:=18766 -p session:=on-demand \
      > /results/e2e_gateway_check/gateway_ondemand.log 2>&1 &
    GW2PID=$!
    sleep 4
    timeout 60 python3 -B /mac/test/e2e_mac_controller_check.py --http 127.0.0.1:18766 --only estop-idle \
      --json /results/e2e_gateway_check/estop_idle.json > /results/e2e_gateway_check/estop_idle.txt 2>&1
    rc_idle=$?
    kill -INT -- -"$GW2PID" 2>/dev/null
    for _ in $(seq 1 50); do kill -0 "$GW2PID" 2>/dev/null || break; sleep 0.2; done
    kill -KILL -- -"$GW2PID" 2>/dev/null
    tail -2 /results/e2e_gateway_check/estop_idle.txt
    [ $rc_idle -eq 0 ] || rc_chk=1
    # 띄운 두 프로세스 그룹만 PID 로 지정해 끈다(패턴으로 찾아 끄지 않는다)
    kill -INT -- -"$GWPID" -"$STACKPID" 2>/dev/null
    for _ in $(seq 1 50); do kill -0 "$GWPID" 2>/dev/null || kill -0 "$STACKPID" 2>/dev/null || break; sleep 0.2; done
    kill -KILL -- -"$GWPID" -"$STACKPID" 2>/dev/null
    tail -2 /results/e2e_gateway_check/check.txt
    [ $rc_chk -eq 0 ] || rc=$((rc + 80))
    echo "== 종단 시험: KEY2 2 s → /armpi/shutdown_requested → armpi_power(dry-run, 실제로 끄지 않음)"
    mkdir -p /results/e2e_power
    cd /tmp/ws/src/armpi_power/test/integration
    ARMPI_ITEST_LOG_DIR=/results/e2e_power timeout 180 python3 -m pytest -q -s -p no:cacheprovider \
      --junitxml=/results/e2e_power/junit.xml test_power_e2e.py 2>&1 | tee /results/e2e_power/pytest.txt | tail -3
    [ ${PIPESTATUS[0]} -eq 0 ] || rc=$((rc + 90))
    cd /tmp/ws
    echo "== 실기 launch(armpi.launch.py mock:=true, 카메라·MJPEG·rosbridge 포함) 8 s 실행 확인"
    ( timeout -s INT 8 ros2 launch armpi_bringup armpi.launch.py mock:=true > /results/armpi_launch_mock_full.log 2>&1 ) &
    sleep 6
    ss -ltnu 2>/dev/null | grep -E ":(8080|9090|5001|5002) " | tee /results/armpi_launch_ports.txt
    wait
    grep -E "process has died|경고|ERROR" /results/armpi_launch_mock_full.log | cut -c1-200 | head -10
    for port in 8080 9090 5001 5002; do grep -q ":$port " /results/armpi_launch_ports.txt || { echo "   포트 $port 없음"; rc=$((rc + 60)); }; done
  fi
fi
exit $rc
' 2>&1 | tee "$RES/run.log"
RC=${PIPESTATUS[0]}
docker cp "$NAME:/results/." "$RES/" >/dev/null 2>&1 || echo "경고: 결과 파일 복사 실패" >&2
docker rm -f "$NAME" >/dev/null 2>&1
set -e
echo "결과 폴더: $RES (종료 코드 $RC)"
exit "$RC"
