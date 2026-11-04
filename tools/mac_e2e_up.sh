#!/bin/bash
# Mac 에서 로봇 없이 전체 시험: 컨테이너에 ArmPi ROS 2 전체(가짜 하드웨어)를 띄운다.
#   컨테이너: colcon build -> armpi_mock.launch.py(제어기 mock + UDP 수신기 + rosbridge) + UDP<->TCP 중계 서버
#   Mac 쪽:  python3 tools/udp_tcp_relay.py client --udp 127.0.0.1:25001 --tcp 127.0.0.1:15001
#            그 뒤 조종기·시험은 로봇 주소 127.0.0.1, 포트 25001, rosbridge ws://127.0.0.1:9090
# 끄기: docker rm -f armpi-mac-e2e
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${ARMPI_DEV_IMAGE:-armpi-ctl-dev:humble}"
docker rm -f armpi-mac-e2e >/dev/null 2>&1 || true
docker run -d --name armpi-mac-e2e -p 15001:15001 -p 9090:9090 \
  -v "$ROOT/ros2_ws/src:/ws_src:ro" -v "$ROOT/tools:/tools:ro" "$IMAGE" bash -c '
set -e
source /opt/ros/humble/setup.bash
mkdir -p /tmp/ws && cp -r /ws_src /tmp/ws/src && cd /tmp/ws
colcon build --event-handlers console_cohesion- --cmake-args -DBUILD_TESTING=OFF > /tmp/build.log 2>&1 || { tail -40 /tmp/build.log; exit 1; }
echo "[e2e] 빌드 완료"
source install/setup.bash
python3 /tools/udp_tcp_relay.py server --tcp 0.0.0.0:15001 --udp 127.0.0.1:5001 &
exec ros2 launch armpi_bringup armpi_mock.launch.py
' >/dev/null
echo "컨테이너 armpi-mac-e2e 시작. 빌드·실행 대기: docker logs -f armpi-mac-e2e"
