#!/usr/bin/env bash
# 워크스페이스 전체 colcon build + colcon test(gtest). 2026-11-03 기록용 스크립트를 저장소로 옮김.
#   scripts/test_all.sh            결과: work/logs/all_packages_<시각>/
# - 소스는 읽기 전용으로 붙이고 컨테이너 /tmp/ws 로 복사해 빌드한다. --network none(루프백만).
# - 이미지 빌드(30_workspace.sh)는 Release + BUILD_TESTING=OFF 였다. 여기서는 시험을 위해 RelWithDebInfo + BUILD_TESTING=ON.
# - 통합·종단 시험은 기존 스크립트 두 개(scripts/armpi_dev_build_and_test.sh all,
#   ros2_ws/src/armpi_bringup/test/dev_build_and_test.sh all)를 그대로 따로 돌린다(결과 폴더는 각 스크립트가 만든다).
set -euo pipefail
PROJ="$(cd "$(dirname "$0")/.." && pwd)"
RES="$PROJ/work/logs/all_packages_$(date +%Y%m%d_%H%M%S)"; mkdir -p "$RES"
IMAGE="${ARMPI_CTL_DEV_IMAGE:-armpi-ctl-dev:humble}"
NAME="armpi-allpkg-test-$(basename "$RES")-$$"
docker image inspect "$IMAGE" --format '이미지 {{.RepoTags}} ID {{.Id}} 생성 {{.Created}} 아키텍처 {{.Architecture}}' | tee "$RES/image_id.txt"
{ date '+시작 %F %T %Z'; uptime; } | tee "$RES/host_load_start.txt"
set +e
docker run --name "$NAME" --network none \
  -v "$PROJ/ros2_ws/src:/src:ro" \
  -e ROS_LOCALHOST_ONLY=1 -e ROS_DOMAIN_ID=83 -e PYTHONDONTWRITEBYTECODE=1 \
  "$IMAGE" bash -c '
set -o pipefail
source /opt/ros/humble/setup.bash
mkdir -p /results /tmp/ws && cp -r /src /tmp/ws/src && cd /tmp/ws
echo "== 환경: $(uname -m), $(. /etc/os-release; echo $PRETTY_NAME), $(g++ --version | head -1), ROS $ROS_DISTRO"
(cd src && find . -type f | LC_ALL=C sort | xargs sha256sum) > /results/source_sha256.txt
echo "== 소스 파일 $(wc -l < /results/source_sha256.txt)개 → source_sha256.txt"
colcon list | tee /results/colcon_list.txt
echo "== colcon build (전체 패키지, BUILD_TESTING=ON)"
colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo --event-handlers console_direct- > /results/colcon_build.txt 2>&1
rc_build=$?; tail -10 /results/colcon_build.txt
mkdir -p /results/build_logs
for p in $(colcon list -n); do
  cp log/latest_build/$p/stdout_stderr.log /results/build_logs/$p.log 2>/dev/null
  printf "   %-18s 컴파일 경고 %s\n" "$p" "$(grep -c "warning:" /results/build_logs/$p.log 2>/dev/null)"
done | tee /results/warnings.txt
[ $rc_build -eq 0 ] || { echo "빌드 실패"; tail -40 /results/colcon_build.txt; exit 10; }
echo "== colcon test (전체 패키지)"
colcon test --merge-install --event-handlers console_direct- > /results/colcon_test.txt 2>&1
tail -15 /results/colcon_test.txt
mkdir -p /results/unit
for p in $(colcon list -n); do mkdir -p /results/unit/$p; cp build/$p/test_results/$p/*.xml /results/unit/$p/ 2>/dev/null; done
colcon test-result --test-result-base build --verbose 2>&1 | tee /results/test_result.txt | tail -6
python3 - <<PY | tee /results/unit_summary.txt
import glob, os, xml.etree.ElementTree as ET
grand = [0, 0, 0, 0]
for pkg in sorted(os.listdir("/results/unit")):
    files = sorted(glob.glob("/results/unit/%s/*.xml" % pkg))
    s = [0, 0, 0, 0]
    for f in files:
        r = ET.parse(f).getroot()
        suites = [r] if r.tag == "testsuite" else list(r.iter("testsuite"))[:1] or [r]
        t, fa, e = (int(r.get(k, 0)) for k in ("tests", "failures", "errors"))
        sk = int(r.get("skipped", r.get("disabled", 0)) or 0)
        for i, v in enumerate((t, fa, e, sk)): s[i] += v
        for tc in r.iter("testcase"):
            for fl in tc.findall("failure"):
                print("     실패: %s.%s: %s" % (tc.get("classname"), tc.get("name"), (fl.get("message") or "")[:300]))
    for i in range(4): grand[i] += s[i]
    print("   %-18s XML %2d개  시험 %4d, 실패 %d, 오류 %d, 건너뜀 %d" % (pkg, len(files), *s))
print("   합계                         시험 %4d, 실패 %d, 오류 %d, 건너뜀 %d" % tuple(grand))
PY
colcon test-result --test-result-base build > /dev/null 2>&1; rc_test=$?
echo "== 종료: build=$rc_build test=$rc_test"
exit $(( rc_test != 0 ? 20 : 0 ))
' 2>&1 | tee "$RES/run.log"
RC=${PIPESTATUS[0]}
docker cp "$NAME:/results/." "$RES/" >/dev/null 2>&1 || echo "경고: 결과 복사 실패" >&2
docker rm -f "$NAME" >/dev/null 2>&1
set -e
{ date '+끝 %F %T %Z'; uptime; } | tee "$RES/host_load_end.txt"
echo "결과 폴더: $RES (종료 코드 $RC)" | tee -a "$RES/run.log"
exit "$RC"
