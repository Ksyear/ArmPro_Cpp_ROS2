#!/usr/bin/env bash
# ArmPi ROS 2 (Humble, arm64) 개발 컨테이너에서 armpi_vr_bridge 와 의존 패키지를 빌드·시험한다.
#   scripts/armpi_dev_build_and_test.sh [unit|integration|all]     (기본 all)
#
# - 대상 이미지와 같은 Ubuntu 22.04 arm64 사용자 공간(Apple Silicon 에서 에뮬레이션 없이 실행).
# - 소스(외장 exFAT·virtiofs 공유 폴더)는 읽기 전용으로 붙이고 컨테이너 /tmp/ws 로 복사해 빌드한다
#   (exFAT 에서 덮어쓰기 직후 ENOENT 가 나는 문제 회피).
# - --network none: 루프백만 쓴다. 실제 로봇에 접속하거나 패킷을 보내지 않는다.
# - 결과는 work/logs/vr_bridge_<모드>_<시각>/ 에 남긴다(run.log, 빌드 로그, gtest XML, 통합 시험 XML·노드 로그).
set -euo pipefail
PROJ="$(cd "$(dirname "$0")/.." && pwd)"
MODE="${1:-all}"
case "$MODE" in unit|integration|all) ;; *) echo "사용법: $0 [unit|integration|all]" >&2; exit 2 ;; esac
IMAGE="${ARMPI_DEV_IMAGE:-armpi-dev:humble}"
STAMP="$(date +%Y%m%d_%H%M%S)"
RES="$PROJ/work/logs/vr_bridge_${MODE}_${STAMP}"
mkdir -p "$RES"

if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
  echo "== 개발 이미지 $IMAGE 생성"
  docker build -t "$IMAGE" - <<'DOCKER'
FROM ros:humble-ros-base-jammy
RUN apt-get update && apt-get install -y --no-install-recommends \
      procps iproute2 python3-pytest psmisc && rm -rf /var/lib/apt/lists/*
DOCKER
fi

# 다른 docker 작업과 겹치지 않는 컨테이너 이름(이미지 제작용 이름은 쓰지 않는다)
NAME="armpi-vrbridge-test-${STAMP}-$$"
# 결과는 컨테이너 안 /results 에 쓰고 끝난 뒤 docker cp 로 꺼낸다.
# (exFAT 공유 폴더에 방금 만든 하위 폴더가 컨테이너에서 ENOENT 로 보이는 문제 회피)
set +e
docker run --name "$NAME" --network none \
  -v "$PROJ/ros2_ws/src:/src:ro" \
  -e MODE="$MODE" -e ROS_LOCALHOST_ONLY=1 -e ROS_DOMAIN_ID=77 -e PYTHONDONTWRITEBYTECODE=1 \
  "$IMAGE" bash -c '
set -o pipefail
source /opt/ros/humble/setup.bash
mkdir -p /results /tmp/ws && cp -r /src /tmp/ws/src && cd /tmp/ws
echo "== 환경: $(uname -m), $(. /etc/os-release; echo $PRETTY_NAME), $(g++ --version | head -1), ROS $ROS_DISTRO"
(cd src && find armpi_common armpi_protocol armpi_interfaces armpi_vr_bridge armpi_kinematics -type f | sort | xargs sha256sum) > /results/source_sha256.txt
echo "== 소스 해시 $(wc -l < /results/source_sha256.txt)개 → source_sha256.txt"

echo "== colcon build (armpi_vr_bridge 와 의존 패키지: common·protocol·kinematics·interfaces)"
colcon build --merge-install --packages-up-to armpi_vr_bridge \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo --event-handlers console_direct- > /results/colcon_build.txt 2>&1
rc_build=$?
tail -6 /results/colcon_build.txt
cp log/latest_build/armpi_vr_bridge/stdout_stderr.log /results/build_armpi_vr_bridge.log 2>/dev/null
nwarn=$(grep -c "warning:" /results/build_armpi_vr_bridge.log 2>/dev/null)
echo "   컴파일 경고 수(armpi_vr_bridge): ${nwarn:-0}"
if [ $rc_build -ne 0 ]; then echo "빌드 실패"; cat /results/build_armpi_vr_bridge.log | tail -40; exit 10; fi
rc=0

if [ "$MODE" = unit ] || [ "$MODE" = all ]; then
  echo "== 단위 시험 (gtest, colcon test)"
  colcon test --merge-install --packages-select armpi_vr_bridge --event-handlers console_direct- \
    > /results/colcon_test.txt 2>&1
  mkdir -p /results/unit && cp build/armpi_vr_bridge/test_results/armpi_vr_bridge/*.xml /results/unit/ 2>/dev/null
  cp -r log/latest_test /results/colcon_test_log 2>/dev/null
  colcon test-result --test-result-base build/armpi_vr_bridge --verbose 2>&1 | tee /results/unit_summary.txt | tail -25
  python3 - <<PY | tee -a /results/unit_summary.txt
import glob, xml.etree.ElementTree as ET
tot = fail = err = skip = 0
for f in sorted(glob.glob("/results/unit/*.gtest.xml")):
    r = ET.parse(f).getroot()
    t, fa, e, s = (int(r.get(k, 0)) for k in ("tests", "failures", "errors", "disabled"))
    tot += t; fail += fa; err += e; skip += s
    print("   %-28s 시험 %3d, 실패 %d, 오류 %d" % (f.split("/")[-1], t, fa, e))
print("   gtest 합계: %d 개, 실패 %d, 오류 %d, 비활성 %d" % (tot, fail, err, skip))
PY
  colcon test-result --test-result-base build/armpi_vr_bridge > /dev/null 2>&1 || rc=20
fi

if [ "$MODE" = integration ] || [ "$MODE" = all ]; then
  echo "== 통합 시험 (노드 실행 + UDP ↔ ROS 토픽, 가짜 control_state)"
  source install/setup.bash
  cd /tmp/ws/src/armpi_vr_bridge/test/integration
  ARMPI_ITEST_LOG_DIR=/results timeout 300 python3 -m pytest -q -s -p no:cacheprovider \
    --junitxml=/results/integration_junit.xml test_bridge_integration.py 2>&1 | tee /results/integration.txt | tail -40
  [ ${PIPESTATUS[0]} -eq 0 ] || rc=$((rc + 30))
  cd /tmp/ws
fi
exit $rc
' 2>&1 | tee "$RES/run.log"
RC=${PIPESTATUS[0]}
docker cp "$NAME:/results/." "$RES/" >/dev/null 2>&1 || echo "경고: 결과 파일 복사 실패" >&2
docker rm -f "$NAME" >/dev/null 2>&1
set -e
echo "결과 폴더: $RES (종료 코드 $RC)"
exit "$RC"
