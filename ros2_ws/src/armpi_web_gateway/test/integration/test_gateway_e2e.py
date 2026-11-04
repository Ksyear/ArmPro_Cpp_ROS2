#!/usr/bin/env python3
"""armpi_web_gateway 종단 시험(가짜 하드웨어, 루프백만).

armpi_mock.launch.py(제어기 mock + UDP 수신기)와 armpi_web_gateway_node(HTTP 127.0.0.1:18766, on-demand)를 띄우고
브라우저 대신 HTTP 로
  1) 화면 파일·/api/config
  2) /api/input 누름 → Protocol v3 UDP → 수신기 → 제어기 → 가짜 바퀴, 놓으면 0
  3) 조종권: 다른 화면은 거부(409), 비상정지는 어느 화면에서나, 소유 화면의 해제
  4) 화면이 1.5 s 조용하면 조종권·세션이 풀리고 수신기 owner 도 비는지
  5) /api/state·/api/config 키가 파이썬 조종기와 같은지(ARMPI_MAC_CONTROLLER_DIR 가 있으면 실제 파이썬 판을 띄워 비교)
  6) Host·출처·형식 검사
를 확인한다.

실행: ARMPI_ITEST_LOG_DIR=<결과> [ARMPI_MAC_CONTROLLER_DIR=<clients/mac_controller> ARMPI_V3_CLIENT_DIR=<protocol_v3>]
      python3 -m pytest -q -s test_gateway_e2e.py
"""
import http.client
import json
import os
import signal
import subprocess
import threading
import time

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

LOG_DIR = os.environ.get("ARMPI_ITEST_LOG_DIR", "/tmp")
PORT = 18766
PY_PORT = 18767
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
REL1 = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)


class Probe(Node):
    def __init__(self):
        super().__init__("armpi_gateway_e2e_probe")
        self.lock = threading.Lock()
        self.data = {}
        for topic, qos in (("/armpi/control_state", LATCHED), ("/armpi_vr_bridge/state", LATCHED),
                           ("/armpi/status", REL1), ("/armpi_control/hw_state", REL1)):
            self.create_subscription(String, topic, lambda m, t=topic: self._on(t, m), qos)

    def _on(self, topic, m):
        with self.lock:
            self.data[topic] = json.loads(m.data)

    def get(self, topic):
        with self.lock:
            return self.data.get(topic)


def wait_for(pred, timeout=3.0, period=0.02):
    t0 = time.monotonic()
    last = None
    while time.monotonic() - t0 < timeout:
        try:
            last = pred()
            if last:
                return last
        except (TypeError, KeyError, AttributeError, OSError, ValueError):
            pass
        time.sleep(period)
    return last


def request(method, path, body=None, port=PORT, headers=None):
    c = http.client.HTTPConnection("127.0.0.1", port, timeout=3)
    h = {"Host": "127.0.0.1:%d" % port}
    data = None
    if body is not None:
        data = json.dumps(body).encode()
        h.update({"Content-Type": "application/json", "Origin": "http://127.0.0.1:%d" % port})
    h.update(headers or {})
    c.request(method, path, body=data, headers=h)
    r = c.getresponse()
    raw = r.read()
    c.close()
    try:
        return r.status, json.loads(raw.decode()), r
    except ValueError:
        return r.status, raw, r


def hold(screen, seconds, f=0.0, held=True, arm=None):
    end = time.monotonic() + seconds
    n = 0
    while time.monotonic() < end:
        n += 1
        request("POST", "/api/input", {"screen": screen, "screen_name": "시험 " + screen, "f": f, "held": held,
                                       "focused": True, "arm": arm or {}, "n": int(time.time() * 1000) + n})
        time.sleep(0.05)


def spawn(cmd, name, env=None):
    log = open(os.path.join(LOG_DIR, name + ".log"), "w")
    return subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
                            env=dict(os.environ, **(env or {}))), log


def stop(proc):
    try:
        os.killpg(proc.pid, signal.SIGINT)
        proc.wait(timeout=10)
    except Exception:  # noqa: BLE001
        os.killpg(proc.pid, signal.SIGKILL)
        proc.wait()
    return proc.returncode


@pytest.fixture(scope="module")
def ctx():
    procs = []
    stack, l1 = spawn(["ros2", "launch", "armpi_bringup", "armpi_mock.launch.py", "rosbridge:=false"], "mock_stack")
    procs.append((stack, l1))
    gw, l2 = spawn(["ros2", "run", "armpi_web_gateway", "armpi_web_gateway_node", "--ros-args",
                    "-p", "http_bind:=127.0.0.1", "-p", "http_port:=%d" % PORT, "-p", "session:=on-demand"],
                   "gateway_node")
    procs.append((gw, l2))
    py = None
    mac_dir = os.environ.get("ARMPI_MAC_CONTROLLER_DIR")
    if mac_dir and os.path.isfile(os.path.join(mac_dir, "armpi_mac_controller.py")):
        py, l3 = spawn(["python3", "-B", os.path.join(mac_dir, "armpi_mac_controller.py"), "--robot", "127.0.0.1",
                        "--bind", "127.0.0.1", "--http-port", str(PY_PORT), "--no-browser", "--session",
                        "on-demand", "--no-rosbridge"], "python_controller",
                       env={"PYTHONPATH": os.environ.get("ARMPI_V3_CLIENT_DIR", "")})
        procs.append((py, l3))
    rclpy.init()
    probe = Probe()
    ex = SingleThreadedExecutor()
    ex.add_node(probe)
    threading.Thread(target=ex.spin, daemon=True).start()
    assert wait_for(lambda: request("GET", "/api/config")[0] == 200, 20.0), "게이트웨이 HTTP 응답 없음"
    assert wait_for(lambda: p_ready(probe), 20.0), "제어기 상태 없음"
    yield probe, py is not None
    for p, log in reversed(procs):
        print("종료", p.args[:3], "코드", stop(p))
        log.close()
    ex.shutdown()
    probe.destroy_node()
    rclpy.shutdown()


def p_ready(probe):
    # 제어기가 켜지자마자 팔을 기본 자세로 옮기는 동안(1.5 s)은 UDP 활성 명령을 받지 않는다(강한 차단 → 손을 떼야 재무장).
    st = probe.get("/armpi/status")
    return st and st.get("arm_initialized") and st.get("udp_allowed") and probe.get("/armpi_control/hw_state") is not None


def test_01_pages_and_config(ctx):
    st, cfg, _ = request("GET", "/api/config")
    assert st == 200
    assert cfg["session_mode"] == "on-demand" and cfg["port"] == 5001 and cfg["control_timeout_s"] == 1.5
    st, body, _ = request("GET", "/mobile")
    assert st == 200 and b"<html" in body.lower()
    st, _, r = request("GET", "/", headers={"User-Agent": "Mozilla/5.0 (Linux; Android 13)"})
    assert st == 302 and r.getheader("Location") == "/mobile"


def test_02_drive_via_http_and_neutral_on_release(ctx):
    p, _ = ctx
    t = threading.Thread(target=hold, args=("A", 2.0, 0.5))
    t.start()
    moved = wait_for(lambda: any(p.get("/armpi_control/hw_state")["wheels"]), 2.0)
    forward = wait_for(lambda: p.get("/armpi/status")["base_cmd"]["vx"] > 0, 1.2)  # 상태는 2 Hz
    t.join()
    assert moved, p.get("/armpi_control/hw_state")
    assert forward, p.get("/armpi/status")["base_cmd"]
    # 화면 입력이 끊기면(0.3 s) 중립 → 바퀴 0
    assert wait_for(lambda: p.get("/armpi_control/hw_state")["wheels"] == [0, 0, 0, 0], 1.5), \
        p.get("/armpi_control/hw_state")


def test_03_ownership_estop_and_clear(ctx):
    p, _ = ctx
    t = threading.Thread(target=hold, args=("A", 1.5, 0.0, True))
    t.start()
    time.sleep(0.3)
    st, res, _ = request("POST", "/api/input", {"screen": "B", "held": True, "focused": True})
    assert st == 200 and res == {"ok": True, "control": False}
    st, res, _ = request("POST", "/api/cmd", {"screen": "B", "cmd": "clear"})
    assert st == 409 and res["control"] is False
    st, res, _ = request("POST", "/api/cmd", {"screen": "B", "cmd": "estop"})  # 조종권 없는 화면의 비상정지
    assert st == 200 and res["ok"]
    cs = wait_for(lambda: p.get("/armpi/control_state")["estop"] and p.get("/armpi/control_state"), 1.5)
    assert cs and "udp" in cs["estop_sources"], p.get("/armpi/control_state")
    st, res, _ = request("POST", "/api/cmd", {"screen": "A", "cmd": "clear"})
    assert st == 200, res
    assert wait_for(lambda: p.get("/armpi/control_state")["estop"] is False, 1.5), p.get("/armpi/control_state")
    t.join()


def test_04_quiet_screen_releases_control_and_session(ctx):
    p, _ = ctx
    hold("A", 0.5, 0.0, True)
    assert wait_for(lambda: p.get("/armpi_vr_bridge/state")["owner"] is not None, 1.0)
    time.sleep(2.0)  # 화면이 조용함(조종권 1.5 s)
    st, s, _ = request("GET", "/api/state?screen=A")
    assert s["control"]["owner"] is None and s["session"]["live"] is False, s["control"]
    assert wait_for(lambda: p.get("/armpi_vr_bridge/state")["owner"] is None, 1.5), p.get("/armpi_vr_bridge/state")
    assert s["udp"]["retired"] is True


PY_STATE_KEYS = {"udp", "arm_enabled", "input", "rosbridge", "joint_debug_allowed", "loop_overruns", "control",
                 "session", "events"}
PY_UDP_KEYS = {"robot", "client_id", "sequence", "connected", "emergency", "estop_pending", "faulted", "retired",
               "base", "arm", "arm_target", "arm_last_accepted", "arm_last_reject", "ack_ms_last", "ack_ms_p50",
               "ack_ms_max", "packets_sent", "acks", "bad_acks", "late_acks", "null_acks", "send_errors",
               "link_drops", "last_error", "ack_timeout_s"}
PY_CONFIG_KEYS = {"robot", "port", "video_url", "home", "rosbridge", "ack_timeout_s", "input_timeout_s", "rates",
                  "joint_names", "joint_limit_rad", "session_mode", "control_timeout_s", "idle_release_s", "ui"}


def test_05_json_shape_matches_python_controller(ctx):
    _, have_py = ctx
    _, s, _ = request("GET", "/api/state?screen=A")
    _, cfg, _ = request("GET", "/api/config")
    assert set(s) == PY_STATE_KEYS
    assert set(s["udp"]) == PY_UDP_KEYS
    assert set(cfg) == PY_CONFIG_KEYS | {"backend"}
    assert set(s["control"]) == {"owner", "you", "timeout_s", "screens", "idle_release_s", "changes"}
    assert set(s["session"]) == {"mode", "live", "reason"}
    assert s["session"]["reason"] in ("none", "always", "user", "estop_only", "closing"), s["session"]
    assert set(cfg["ui"]) == {"default_speed"} and 0 < cfg["ui"]["default_speed"] <= 1, cfg["ui"]
    if not have_py:
        pytest.skip("파이썬 조종기 경로 없음(ARMPI_MAC_CONTROLLER_DIR) — 고정 키 목록으로만 비교")
    assert wait_for(lambda: request("GET", "/api/config", port=PY_PORT)[0] == 200, 10.0)
    _, ps, _ = request("GET", "/api/state?screen=A", port=PY_PORT)
    _, pc, _ = request("GET", "/api/config", port=PY_PORT)
    assert set(ps) == set(s)
    for k in ("udp", "control", "session", "input"):
        assert set(ps[k]) == set(s[k]), k
    assert set(ps["udp"]["base"]) == set(s["udp"]["base"])
    assert set(pc) | {"backend"} == set(cfg)
    for k in ("home", "rates", "joint_names", "joint_limit_rad", "input_timeout_s", "control_timeout_s"):
        assert pc[k] == cfg[k], k


def test_06_host_origin_and_type_checks(ctx):
    st, _, _ = request("GET", "/api/config", headers={"Host": "evil.example:%d" % PORT})
    assert st == 403
    st, _, _ = request("POST", "/api/input", {"screen": "Z"}, headers={"Origin": "http://evil.example"})
    assert st == 403
    st, _, _ = request("POST", "/api/input", {"screen": "Z"}, headers={"Content-Type": "text/plain"})
    assert st == 403
