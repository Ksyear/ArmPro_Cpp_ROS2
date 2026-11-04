#!/bin/bash
# Mac 단독 자체 시험(실제 로봇에 접속하지 않음, 모든 소켓 127.0.0.1).
# 1) 가짜 로봇(기본 포트 5001/5002/9090/8080) 2) 적합성 시험(기본·해석+rosbridge) 3) 클라이언트 smoke·검색
# 4) 조종기(HTTP 18765 — 이 맥의 8765 는 다른 프로그램이 사용 중) + HTTP API 종단 확인
# 4b) on-demand 조종기에서 연결 없이 비상정지 5) --robot auto(127.0.0.1 유니캐스트)
# 6) 대상 조건별 적합성 시험(제어기 상태 미수신·udp 차단·fault·주소 차단 가짜 로봇)
# 끝나면 띄운 프로세스를 모두 종료한다. 2026-11-02 기록 폴더의 run_selftest.sh 를 저장소로 옮겼다.
#   scripts/selftest_mac.sh        결과: work/logs/selftest_<시각>/
set -u
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="$ROOT/work/logs/selftest_$(date +%Y%m%d_%H%M%S)"; mkdir -p "$OUT"
CHECK="$ROOT/clients/mac_controller/test/e2e_mac_controller_check.py"
PY="${PY:-python3}"
cd "$ROOT" || exit 1
{
  echo "date: $(date '+%Y-%m-%dT%H:%M:%S%z')"
  echo "python: $($PY --version 2>&1) ($(command -v $PY))"
  echo "os: $(uname -srm)"
  echo "root: $ROOT"
  shasum -a 256 protocol_v3/*.py clients/mac_controller/armpi_mac_controller.py ros2_ws/src/armpi_web_gateway/www/index.html "$CHECK" "$0"
} > "$OUT/environment.txt"

PIDS=()
cleanup() { for p in "${PIDS[@]}"; do kill -TERM "$p" 2>/dev/null; done; wait 2>/dev/null; }
trap cleanup EXIT

echo "== 1. 가짜 로봇"
$PY protocol_v3/mock_robot.py --bind 127.0.0.1 > "$OUT/mock_robot.log" 2>&1 &
MOCK=$!; PIDS+=($MOCK)
sleep 1

echo "== 2. 적합성 시험(기본)"
$PY protocol_v3/conformance_test.py --host 127.0.0.1 -v --json-report "$OUT/conformance_default.json" \
  > "$OUT/conformance_default.txt" 2>&1; echo "exit=$?" >> "$OUT/conformance_default.txt"
tail -3 "$OUT/conformance_default.txt"
echo "== 2b. 적합성 시험(해석 시험 + rosbridge 외부 비상정지)"
$PY protocol_v3/conformance_test.py --host 127.0.0.1 --rosbridge-port 9090 --interpretation -v \
  --json-report "$OUT/conformance_full.json" > "$OUT/conformance_full.txt" 2>&1; echo "exit=$?" >> "$OUT/conformance_full.txt"
tail -3 "$OUT/conformance_full.txt"

echo "== 3. 클라이언트 라이브러리 검색·smoke"
$PY protocol_v3/armpi_v3_client.py --discover --discover-target 127.0.0.1 > "$OUT/client_discover.txt" 2>&1
echo "exit=$?" >> "$OUT/client_discover.txt"
$PY protocol_v3/armpi_v3_client.py --robot 127.0.0.1 --smoke > "$OUT/client_smoke.txt" 2>&1
echo "exit=$?" >> "$OUT/client_smoke.txt"
tail -2 "$OUT/client_smoke.txt"

echo "== 4. Mac 조종기 + HTTP API 종단 확인"
$PY clients/mac_controller/armpi_mac_controller.py --robot 127.0.0.1 --http-port 18765 --no-browser \
  > "$OUT/mac_controller.log" 2>&1 &
CTL=$!; PIDS+=($CTL)
sleep 1.5
$PY "$CHECK" --http 127.0.0.1:18765 --json "$OUT/e2e_mac_controller.json" \
  > "$OUT/e2e_mac_controller.txt" 2>&1; echo "exit=$?" >> "$OUT/e2e_mac_controller.txt"
tail -3 "$OUT/e2e_mac_controller.txt"
kill -TERM $CTL; wait $CTL 2>/dev/null
sleep 1.2

echo "== 4b. on-demand 조종기: 연결 없이 비상정지(연결·조종권이 생기지 않고 임시 세션은 2 s 뒤 닫힘)"
$PY clients/mac_controller/armpi_mac_controller.py --robot 127.0.0.1 --http-port 18767 --session on-demand --no-browser \
  > "$OUT/mac_controller_ondemand.log" 2>&1 &
CTL3=$!; PIDS+=($CTL3)
sleep 1.5
$PY "$CHECK" --http 127.0.0.1:18767 --only estop-idle --json "$OUT/e2e_estop_idle.json" \
  > "$OUT/e2e_estop_idle.txt" 2>&1; echo "exit=$?" >> "$OUT/e2e_estop_idle.txt"
tail -3 "$OUT/e2e_estop_idle.txt"
kill -TERM $CTL3; wait $CTL3 2>/dev/null
sleep 1.2

echo "== 5. --robot auto (127.0.0.1 유니캐스트 검색, 브로드캐스트 안 함)"
$PY clients/mac_controller/armpi_mac_controller.py --robot auto --discover-target 127.0.0.1 --http-port 18766 --no-browser \
  --no-rosbridge > "$OUT/mac_controller_auto.log" 2>&1 &
CTL2=$!; PIDS+=($CTL2)
sleep 3.5
curl -s -H 'Host: 127.0.0.1:18766' http://127.0.0.1:18766/api/state > "$OUT/mac_controller_auto_state.json"
kill -TERM $CTL2; wait $CTL2 2>/dev/null
grep -E "발견|조종기" "$OUT/mac_controller_auto.log"

kill -TERM $MOCK; wait $MOCK 2>/dev/null

echo "== 6. 대상 조건별 적합성 시험"
: > "$OUT/conformance_conditions.txt"
for spec in "15011 15012 control_missing --control-state=missing" "15021 15022 udp_blocked --control-state=udp_blocked" \
            "15031 15032 fault --control-state=fault" "15041 15042 source_blocked --allowed-sources=10.0.0.0/8"; do
  set -- $spec
  $PY protocol_v3/mock_robot.py --bind 127.0.0.1 --port $1 --discovery-port $2 --rosbridge-port 0 --camera-port 0 $4 \
    > "$OUT/mock_robot_$3.log" 2>&1 &
  M=$!; PIDS+=($M)
  sleep 0.8
  echo "### 조건 $3 (가짜 로봇 $4)" >> "$OUT/conformance_conditions.txt"
  $PY protocol_v3/conformance_test.py --host 127.0.0.1 --port $1 --discovery-port $2 --condition $3 --interpretation -v \
    --json-report "$OUT/conformance_$3.json" >> "$OUT/conformance_conditions.txt" 2>&1
  echo "exit=$?" >> "$OUT/conformance_conditions.txt"
  kill -TERM $M; wait $M 2>/dev/null
done
grep -E "^###|^요약" "$OUT/conformance_conditions.txt"
echo "== 끝"
