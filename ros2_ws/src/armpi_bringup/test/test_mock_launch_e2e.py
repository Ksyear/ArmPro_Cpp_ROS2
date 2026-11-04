#!/usr/bin/env python3
"""armpi_mock.launch.py 종단 시험(가짜 하드웨어, 루프백만 사용).

launch 로 armpi_control(mock) + armpi_vr_bridge + rosbridge 를 띄우고,
  1) 두 노드가 config/armpi.yaml 의 공유 값(/**: 기구학·작업 공간·속도 상한)을 받았는지
  2) Protocol v3 UDP 클라이언트 → 수신기 → /armpi/cmd_vel·/armpi/arm/target → 제어기 → 가짜 바퀴·서보
  3) UDP 비상정지/해제 → /armpi/estop_request → 제어기 latch(출처 udp) → control_state → 수신기
  4) rosbridge(9090) 접속 가능
를 확인한다. 클라이언트는 protocol_v3/armpi_v3_client.py(읽기 전용)를 경로로 불러 쓴다.

실행: ARMPI_V3_CLIENT_DIR=<protocol_v3 폴더> ARMPI_ITEST_LOG_DIR=<결과> python3 -m pytest -q -s test_mock_launch_e2e.py
"""
import json
import math
import os
import re
import signal
import socket
import subprocess
import sys
import threading
import time

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

sys.path.insert(0, os.environ.get("ARMPI_V3_CLIENT_DIR", "/tmp/vr_protocol_v3"))
from armpi_v3_client import ArmPiV3Client  # noqa: E402

LOG_DIR = os.environ.get("ARMPI_ITEST_LOG_DIR", "/tmp")
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
REL1 = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)


class Probe(Node):
    def __init__(self):
        super().__init__("armpi_e2e_probe")
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
        except (TypeError, KeyError, AttributeError):
            pass
        time.sleep(period)
    return last


def run_client(c, seconds, arm_dx=0.0, **intent):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if arm_dx and c.arm.armed:
            c.move_arm_target(dx=arm_dx)
        c.tick(**intent)
        t0 = time.monotonic()
        while time.monotonic() - t0 < 0.05:
            c.poll()
            time.sleep(0.002)


def param(node, name):
    out = subprocess.run(["ros2", "param", "get", node, name], capture_output=True, text=True, timeout=20)
    return out.stdout.strip()


def param_float(node, name):
    m = re.search(r"value is:\s*([-+0-9.eE]+)", param(node, name))
    assert m, (node, name, param(node, name))
    return float(m.group(1))


def expected_wheels(vx, vy, wz, node="/armpi_control"):
    """제어기 매개변수로 바퀴 펄스 기대값을 계산한다(armpi_hardware 변환과 같은 식, 상한·상수를 시험에 박지 않음)."""
    a, b = param_float(node, "chassis.a_mm"), param_float(node, "chassis.b_mm")
    d, ppc = param_float(node, "chassis.wheel_diameter_mm"), param_float(node, "chassis.pulse_per_cycle")
    sign, cap = param_float(node, "chassis.angular_sign"), int(param_float(node, "chassis.max_wheel_pulse"))
    v = math.hypot(vx, vy) * 1000.0
    direction = math.degrees(math.atan2(vx, -vy)) if v > 0 else 90.0
    velocity, angular = -v, -(sign * wz)
    X, Y = velocity * math.cos(math.radians(direction)), velocity * math.sin(math.radians(direction))
    vp = angular * (a + b)
    v1, v2, v3, v4 = Y - X + vp, Y + X - vp, Y - X - vp, Y + X + vp
    pulse = lambda s: int(s / (math.pi * d) * ppc * 0.01)
    raw = [pulse(-v1), pulse(v4), pulse(v2), pulse(-v3)]
    peak = max(abs(x) for x in raw)
    return [int(x * cap / peak) for x in raw] if peak > cap else raw


@pytest.fixture(scope="module")
def ctx():
    log_path = os.path.join(LOG_DIR, "armpi_mock_launch_e2e.log")
    log = open(log_path, "w")
    proc = subprocess.Popen(["ros2", "launch", "armpi_bringup", "armpi_mock.launch.py"], stdout=log,
                            stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    probe = Probe()
    ex = SingleThreadedExecutor()
    ex.add_node(probe)
    threading.Thread(target=ex.spin, daemon=True).start()
    yield probe, proc
    try:
        os.killpg(proc.pid, signal.SIGINT)
        proc.wait(timeout=20)
    except Exception:
        os.killpg(proc.pid, signal.SIGKILL)
    ex.shutdown()
    probe.destroy_node()
    rclpy.shutdown()
    log.close()
    print("launch 로그:", log_path, "종료 코드:", proc.returncode)


def test_01_nodes_up_with_shared_params(ctx):
    p, proc = ctx
    assert wait_for(lambda: p.get("/armpi/control_state"), 15.0), "control_state 없음"
    assert wait_for(lambda: p.get("/armpi_vr_bridge/state"), 10.0), "vr_bridge state 없음"
    assert wait_for(lambda: p.get("/armpi/status")["hardware"] == "mock", 5.0)
    # /** 공유 값이 두 노드에 같이 들어갔는지
    for node in ("/armpi_vr_bridge", "/armpi_control"):
        assert "0.10048" in param(node, "arm.kinematics.upper_arm_m"), node
        assert "0.05" in param(node, "arm.workspace.x_min"), node
        assert "-1.4" in param(node, "arm.kinematics.j2_offset"), node  # 시험용 프로필(실제 로봇 값 아님)
    # 차체 상한도 두 노드가 같은 값을 받았는지(값 자체는 config/armpi.yaml 이 정한다)
    for k in ("chassis.max_vx_m_s", "chassis.max_vy_m_s", "chassis.max_wz_rad_s"):
        assert param_float("/armpi_vr_bridge", k) == param_float("/armpi_control", k), k
    assert "True" in param("/armpi_control", "hardware.mock")
    # 수신기가 제어기 상태를 받아 게이트가 열렸는지(home 1.5 s 뒤)
    assert wait_for(lambda: p.get("/armpi_vr_bridge/state")["udp_permitted"] is True, 6.0), \
        p.get("/armpi_vr_bridge/state")
    assert proc.poll() is None


def test_02_udp_drive_arm_and_estop_end_to_end(ctx):
    p, _ = ctx
    cur = wait_for(lambda: p.get("/armpi/status")["arm_current"], 3.0)
    home = {k: cur[k] for k in ("x", "y", "z", "pitch", "gripper")}
    c = ArmPiV3Client("127.0.0.1", home=home)
    try:
        run_client(c, 0.5)  # 중립 확인
        assert c.snapshot()["base"]["state"] == "idle", c.snapshot()["base"]
        # 차체: forward 1.0 → vx = chassis.max_vx_m_s → 바퀴 펄스(매개변수로 계산)
        want = expected_wheels(param_float("/armpi_vr_bridge", "chassis.max_vx_m_s"), 0.0, 0.0)
        run_client(c, 1.2, base_intent=True, forward=1.0)
        assert c.snapshot()["base"]["state"] == "active"
        assert wait_for(lambda: p.get("/armpi_control/hw_state")["wheels"] == want, 1.0), \
            (p.get("/armpi_control/hw_state")["wheels"], want)
        run_client(c, 0.4)  # 놓기 → 정지
        assert wait_for(lambda: p.get("/armpi_control/hw_state")["wheels"] == [0, 0, 0, 0], 1.0)
        # 팔: +x 로 0.02 m/s × 1.5 s
        w0 = p.get("/armpi_control/hw_state")["servo_writes"]
        x0 = p.get("/armpi/status")["arm_current"]["x"]
        run_client(c, 1.5, arm_dx=0.02 * 0.05, arm_intent=True)
        assert c.snapshot()["arm"]["state"] == "active", c.snapshot()["arm"]
        assert wait_for(lambda: p.get("/armpi_control/hw_state")["servo_writes"] > w0, 1.0)
        assert wait_for(lambda: p.get("/armpi/status")["arm_current"]["x"] > x0 + 0.015, 1.5), \
            (x0, p.get("/armpi/status")["arm_current"])
        st = p.get("/armpi/status")
        assert st["owner"] == c.snapshot()["client_id"] and st["arm_state"] in ("active", "idle", "timeout")
        # 비상정지(UDP) → 제어기 latch(출처 udp) → 해제
        c.emergency_stop()
        run_client(c, 0.3)
        assert wait_for(lambda: p.get("/armpi/control_state")["estop_sources"] == ["udp"], 1.5), \
            p.get("/armpi/control_state")
        assert p.get("/armpi_control/hw_state")["wheels"] == [0, 0, 0, 0]
        c.clear_emergency()
        run_client(c, 0.3)
        assert wait_for(lambda: p.get("/armpi/control_state")["estop"] is False, 1.5), p.get("/armpi/control_state")
        c.quit()
        assert wait_for(lambda: p.get("/armpi/status")["owner"] is None, 3.0)
    finally:
        c.close()
    assert p.get("/armpi_control/hw_state")["mock_unload_commands"] == 0


def test_03_rosbridge_port_open(ctx):
    ok = False
    for _ in range(50):
        try:
            with socket.create_connection(("127.0.0.1", 9090), timeout=0.5):
                ok = True
                break
        except OSError:
            time.sleep(0.2)
    assert ok, "rosbridge 9090 접속 불가"
