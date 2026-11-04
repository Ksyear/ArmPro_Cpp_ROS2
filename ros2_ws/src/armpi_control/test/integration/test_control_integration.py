#!/usr/bin/env python3
"""armpi_control_node 통합 시험(가짜 하드웨어).

노드를 hardware.mock:=true 로 띄우고 ROS 토픽·서비스로 명령을 넣은 뒤, 가짜 하드웨어 상태
(/armpi_control/hw_state: 바퀴 펄스, 서보 명령값, 가짜 서보 위치)와 발행 토픽을 확인한다.

실행(설치 공간을 source 한 뒤):
  ARMPI_PARAMS_FILE=<armpi.yaml> ARMPI_ITEST_LOG_DIR=<결과 폴더> python3 -m pytest -q -s test_control_integration.py
시험은 파일 순서대로 같은 노드 하나에 이어서 실행된다(상태가 이어짐).
"""
import json
import math
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

from armpi_interfaces.msg import ArmTarget
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import BatteryState, JointState
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger

def _load_test_profile():
    """설치된 armpi_bringup 의 시험용 하드웨어 프로필(실제 로봇 값 아님). 가짜 하드웨어는 이 값으로 빈 칸을 채운다."""
    import yaml
    from ament_index_python.packages import get_package_share_directory
    path = os.environ.get("ARMPI_TEST_PROFILE") or os.path.join(
        get_package_share_directory("armpi_bringup"), "config", "test_hardware_profile.yaml")
    with open(path, encoding="utf-8") as f:
        doc = yaml.safe_load(f)
    vals = {}
    for sec in ("/**", "armpi_control"):
        vals.update((doc.get(sec) or {}).get("ros__parameters", {}))
    return vals


PROF = _load_test_profile()
HOME = {i + 1: int(v) for i, v in enumerate(PROF["arm.home_servo_values"])}  # 서보 ID → 켤 때 값
SERVO_IDS = [int(v) for v in PROF["arm.servo_ids"]]                         # 관절 순서 → 서보 ID
DIRS = [int(v) for v in PROF["arm.joint_directions"]]
KEY1, KEY2 = int(PROF["buttons.key1_bcm"]), int(PROF["buttons.key2_bcm"])
FAN_REG = max(-100, min(100, int(PROF["fan.speed"]))) * int(PROF["hardware.dc_motor_sign"])
LOG_DIR = os.environ.get("ARMPI_ITEST_LOG_DIR", "/tmp")
PARAMS_FILE = os.environ.get("ARMPI_PARAMS_FILE", "")


# ── 메카넘 바퀴 속도 식(기대값 계산용, 시험 안에서 독립 계산) ──
def reference_pulses(velocity, direction, angular, a=PROF["chassis.a_mm"], b=PROF["chassis.b_mm"],
                     d=PROF["chassis.wheel_diameter_mm"], ppc=PROF["chassis.pulse_per_cycle"]):
    velocity, angular = -velocity, -angular
    r = math.pi / 180
    vx = velocity * math.cos(direction * r)
    vy = velocity * math.sin(direction * r)
    vp = angular * (a + b)
    v1, v2, v3, v4 = vy - vx + vp, vy + vx - vp, vy - vx - vp, vy + vx + vp
    return [int(v / (math.pi * d) * ppc * 0.01) for v in (-v1, v4, v2, -v3)]


def cmd_pulses(vx, vy, wz, sign=1.0):
    v = math.hypot(vx, vy) * 1000.0
    direction = math.degrees(math.atan2(vx, -vy)) if v > 0 else 90.0
    return reference_pulses(v, direction, sign * wz)


LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
REL1 = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
REL10 = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)


class Probe(Node):
    def __init__(self):
        super().__init__("armpi_control_itest_probe")
        self.lock = threading.Lock()
        self.control_state = None
        self.control_states = []
        self.status = None
        self.hw = None
        self.joint_state = None
        self.battery = None
        self.shutdown_count = 0
        self.create_subscription(String, "/armpi/control_state", self._cs, LATCHED)
        self.create_subscription(String, "/armpi/status", self._st, REL1)
        self.create_subscription(String, "/armpi_control/hw_state", self._hw, REL1)
        self.create_subscription(JointState, "/joint_states", self._js, REL10)
        self.create_subscription(BatteryState, "/armpi/battery", self._bat, REL10)
        self.create_subscription(Empty, "/armpi/shutdown_requested", self._sd, REL1)
        self.cmd_pub = self.create_publisher(TwistStamped, "/armpi/cmd_vel", REL1)
        self.arm_pub = self.create_publisher(ArmTarget, "/armpi/arm/target", REL1)
        self.estop_pub = self.create_publisher(String, "/armpi/estop_request", REL10)
        self.jd_pub = self.create_publisher(JointState, "/armpi/arm/joint_debug", REL10)
        self.bridge_pub = self.create_publisher(String, "/armpi_vr_bridge/state", LATCHED)
        self.mock_pub = self.create_publisher(String, "/armpi_control/mock_cmd", REL10)
        self.cli = {n: self.create_client(Trigger, n) for n in
                    ("/armpi/emergency_stop", "/armpi/reset_estop", "/armpi/arm/home")}

    def _cs(self, m):
        with self.lock:
            self.control_state = json.loads(m.data)
            self.control_states.append(self.control_state)

    def _st(self, m):
        with self.lock:
            self.status = json.loads(m.data)

    def _hw(self, m):
        with self.lock:
            self.hw = json.loads(m.data)

    def _js(self, m):
        with self.lock:
            self.joint_state = m

    def _bat(self, m):
        with self.lock:
            self.battery = m

    def _sd(self, _m):
        with self.lock:
            self.shutdown_count += 1

    # ── 보내기 ──
    def twist(self, vx=0.0, vy=0.0, wz=0.0, age=0.0):
        m = TwistStamped()
        t = self.get_clock().now().nanoseconds - int(age * 1e9)
        m.header.stamp.sec, m.header.stamp.nanosec = t // 10**9, t % 10**9
        m.header.frame_id = "base_link"
        m.twist.linear.x, m.twist.linear.y, m.twist.angular.z = float(vx), float(vy), float(wz)
        self.cmd_pub.publish(m)

    def arm(self, x, y, z, pitch, gripper, active=True):
        m = ArmTarget()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "armpi_arm_base"
        m.active = active
        m.x, m.y, m.z, m.pitch, m.gripper = float(x), float(y), float(z), float(pitch), float(gripper)
        self.arm_pub.publish(m)

    def estop_request(self, source, latch):
        self.estop_pub.publish(String(data=json.dumps({"source": source, "latch": latch})))

    def mock(self, text):
        self.mock_pub.publish(String(data=text))

    def bridge_state(self, owner=None, arm_state="idle"):
        self.bridge_pub.publish(String(data=json.dumps(
            {"owner": owner, "owner_addr": "10.0.0.5:40000" if owner else None, "base_state": "idle",
             "arm_state": arm_state, "emergency_stop": False, "packets": {}})))

    def joint_debug(self, names, positions):
        m = JointState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.name = list(names)
        m.position = [float(p) for p in positions]
        self.jd_pub.publish(m)

    def call(self, name, timeout=5.0):
        cli = self.cli[name]
        assert cli.wait_for_service(timeout_sec=timeout), name
        fut = cli.call_async(Trigger.Request())
        t0 = time.monotonic()
        while not fut.done() and time.monotonic() - t0 < timeout:
            time.sleep(0.01)
        assert fut.done(), name + " 응답 없음"
        return fut.result()

    def get(self, attr):
        with self.lock:
            return getattr(self, attr)


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


def stream(fn, duration, hz=20.0):
    t_end = time.monotonic() + duration
    while time.monotonic() < t_end:
        fn()
        time.sleep(1.0 / hz)


@pytest.fixture(scope="module")
def ctx():
    log_path = os.path.join(LOG_DIR, "armpi_control_node_itest.log")
    cmd = ["ros2", "run", "armpi_control", "armpi_control_node", "--ros-args"]
    if PARAMS_FILE:
        cmd += ["--params-file", PARAMS_FILE]
    cmd += ["-p", "hardware.mock:=true", "-p", "buzzer.estop_beep_sec:=0.5", "-p", "hardware.fault_threshold:=5"]
    log = open(log_path, "w")
    proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    probe = Probe()
    ex = SingleThreadedExecutor()
    ex.add_node(probe)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()
    yield probe, proc
    probe.twist()
    try:
        os.killpg(proc.pid, signal.SIGINT)
        proc.wait(timeout=10)
    except Exception:
        os.killpg(proc.pid, signal.SIGKILL)
    ex.shutdown()
    probe.destroy_node()
    rclpy.shutdown()
    log.close()
    print("노드 로그:", log_path)


def hw_wheels(p):
    hw = p.get("hw")
    return hw["wheels"] if hw else None


def servo_cmd(p):
    hw = p.get("hw")
    return {int(k): v for k, v in hw["servo_cmd"].items()} if hw else None


# ───────────────────────── 시험 ─────────────────────────

def test_01_startup_state_and_topics(ctx):
    p, proc = ctx
    cs = wait_for(lambda: p.get("control_state"), 10.0)
    assert cs is not None, "control_state 미수신"
    assert cs["estop"] is False and cs["fault"] is False and cs["estop_sources"] == []
    assert set(cs) == {"estop", "estop_sources", "fault", "fault_reason", "udp_allowed"}
    st = wait_for(lambda: p.get("status"), 5.0)
    assert st["hardware"] == "mock" and st["arm_initialized"] is True
    # Mac 조종기가 읽는 키
    for k in ("owner", "owner_addr", "base_state", "arm_state", "emergency_stop", "estop_sources", "battery_v",
              "arm_current", "arm_target", "position_source", "udp_allowed", "fault"):
        assert k in st, k
    assert st["position_source"] == "commanded"
    assert st["owner"] is None and st["base_state"] == "unknown"
    bat = wait_for(lambda: p.get("battery"), 3.0)
    assert bat is not None and abs(bat.voltage - 7.4) < 1e-3
    assert wait_for(lambda: p.get("status")["battery_v"] == 7.4, 3.0)
    js = wait_for(lambda: p.get("joint_state"), 3.0)
    assert list(js.name) == ["joint1", "joint2", "joint3", "joint4", "joint5", "r_joint"]
    assert wait_for(lambda: servo_cmd(p) == HOME, 3.0), servo_cmd(p)
    hw = p.get("hw")
    assert hw["mock_fan_register"] == FAN_REG  # 팬: fan.speed × dc_motor_sign(시험용 프로필)
    assert hw["wheels"] == [0, 0, 0, 0]
    assert proc.poll() is None


def test_02_home_pose_reached_and_joint_states(ctx):
    p, _ = ctx
    ok = wait_for(lambda: all(abs(p.get("hw")["mock_servo_pos"][str(i)] - v) < 0.6 for i, v in HOME.items()), 4.0)
    assert ok, p.get("hw")["mock_servo_pos"]
    js = p.get("joint_state")
    d = math.radians(0.24)
    want = [DIRS[i] * (HOME[SERVO_IDS[i]] - 500) * d for i in range(6)]
    for a, b in zip(js.position, want):
        assert abs(a - b) < 1e-9
    cur = p.get("status")["arm_current"]
    # 켤 때 자세의 FK: 작업 공간 안의 유한값, 그리퍼 0(열림)
    assert all(math.isfinite(cur[k]) for k in ("x", "y", "z", "pitch"))
    assert 0.05 <= cur["x"] <= 0.32 and -0.08 <= cur["z"] <= 0.32 and cur["gripper"] == 0.0
    assert wait_for(lambda: p.get("control_state")["udp_allowed"] is True, 2.0)
    assert all(p.get("hw")["mock_torque"].values())


def test_03_cmd_vel_forward_then_watchdog(ctx):
    p, _ = ctx
    want = cmd_pulses(0.1, 0.0, 0.0)
    assert want[0] > 0 and want == [want[0], -want[0], -want[0], want[0]]  # 앞으로: [+, −, −, +]
    t_end = time.monotonic() + 1.2
    saw_partial = False
    while time.monotonic() < t_end:
        p.twist(vx=0.1)
        w = hw_wheels(p)
        if w and w != [0, 0, 0, 0] and w != want:
            saw_partial = True  # 가속 제한 중간값
        time.sleep(0.05)
    assert hw_wheels(p) == want, hw_wheels(p)
    assert saw_partial, "가속 제한 중간값을 보지 못함"
    assert abs(p.get("status")["base_cmd"]["vx"] - 0.1) < 1e-6 or wait_for(
        lambda: abs(p.get("status")["base_cmd"]["vx"] - 0.1) < 1e-6, 1.0)
    # 보내기 중단: 0.3 s 뒤엔 아직 움직이고, 1.0 s 뒤엔 정지(2차 watchdog 0.6 s)
    t_stop = time.monotonic()
    time.sleep(0.3)
    assert hw_wheels(p) == want
    assert wait_for(lambda: hw_wheels(p) == [0, 0, 0, 0], 1.0)
    dt = time.monotonic() - t_stop
    assert 0.55 <= dt <= 1.0, dt


def test_04_stop_command_is_immediate(ctx):
    p, _ = ctx
    stream(lambda: p.twist(vx=0.1), 0.6)
    assert wait_for(lambda: hw_wheels(p) == cmd_pulses(0.1, 0.0, 0.0), 1.0)
    t0 = time.monotonic()
    p.twist()
    assert wait_for(lambda: hw_wheels(p) == [0, 0, 0, 0], 0.5, period=0.005)
    assert time.monotonic() - t0 < 0.25


def test_05_stale_stamp_dropped(ctx):
    p, _ = ctx
    before = wait_for(lambda: p.get("status"), 1.0)["cmd_vel"]["dropped_stale"]
    stream(lambda: p.twist(vx=0.1, age=1.0), 0.6)
    assert hw_wheels(p) == [0, 0, 0, 0]
    assert wait_for(lambda: p.get("status")["cmd_vel"]["dropped_stale"] >= before + 5, 1.5)


def test_06_strafe_and_rotate_match_reference_formula(ctx):
    p, _ = ctx
    for vx, vy, wz in ((0.0, 0.1, 0.0), (0.0, -0.1, 0.0), (0.0, 0.0, 0.6), (0.07, 0.05, -0.3)):
        want = cmd_pulses(vx, vy, wz)
        stream(lambda: p.twist(vx, vy, wz), 0.6)
        assert wait_for(lambda: hw_wheels(p) == want, 0.5), (vx, vy, wz, hw_wheels(p), want)
        p.twist()
        assert wait_for(lambda: hw_wheels(p) == [0, 0, 0, 0], 0.5)
    s1 = cmd_pulses(0.0, 0.1, 0.0)
    assert s1[0] > 0 and s1 == [s1[0]] * 4                      # 왼쪽 옆걸음: 네 바퀴 같은 부호
    r = cmd_pulses(0.0, 0.0, 0.6)
    assert r[0] > 0 and r == [r[0], -r[0], r[0], -r[0]]          # 제자리 회전


def test_07_arm_target_tracking_then_watchdog_hold(ctx):
    p, _ = ctx
    goal = dict(x=0.145, y=0.03, z=0.095, pitch=-0.8, gripper=0.5)
    w0 = p.get("hw")["servo_writes"]
    stream(lambda: p.arm(**goal), 2.5)
    st = p.get("status")
    cur = st["arm_current"]
    err = math.dist((cur["x"], cur["y"], cur["z"]), (goal["x"], goal["y"], goal["z"]))
    assert err < 0.003, (cur, goal)
    assert abs(cur["pitch"] - goal["pitch"]) < 0.01 and abs(cur["gripper"] - 0.5) < 0.005
    assert st["arm_target"]["x"] == goal["x"]
    hw = p.get("hw")
    assert hw["servo_writes"] > w0
    assert hw["servo_cmd"][str(SERVO_IDS[4])] == 500  # 손목 회전 고정
    # joint_states 와 서보 명령값 일치(관절 방향은 프로필)
    js = p.get("joint_state")
    d = math.radians(0.24)
    for i in range(4):
        assert abs(js.position[i] - DIRS[i] * (hw["servo_cmd"][str(SERVO_IDS[i])] - 500) * d) < 1e-9, i
    # 목표 중단 → 0.6 s 뒤 유지, 그 뒤 서보 명령 없음
    time.sleep(0.8)
    w1 = p.get("hw")["servo_writes"]
    time.sleep(1.0)
    assert p.get("hw")["servo_writes"] == w1
    assert p.get("hw")["mock_unload_commands"] == 0


def test_08_arm_inactive_holds_immediately(ctx):
    p, _ = ctx
    goal = dict(x=0.20, y=-0.05, z=0.05, pitch=-1.0, gripper=0.0)
    stream(lambda: p.arm(**goal), 0.3)
    assert wait_for(lambda: p.get("status")["arm_moving"] is True, 1.0)
    p.arm(active=False, **goal)
    time.sleep(0.15)
    w = p.get("hw")["servo_writes"]
    time.sleep(0.5)
    assert p.get("hw")["servo_writes"] == w
    cur = p.get("status")["arm_current"]
    assert math.dist((cur["x"], cur["y"], cur["z"]), (goal["x"], goal["y"], goal["z"])) > 0.01


def test_09_workspace_and_ik_rejects(ctx):
    p, _ = ctx
    p.arm(0.45, 0.0, 0.1, 0.0, 0.0)
    assert wait_for(lambda: p.get("status")["arm_last_target_result"] == "workspace_limit", 1.5)
    p.arm(0.10, 0.0, 0.30, -math.pi / 2, 0.0)
    assert wait_for(lambda: p.get("status")["arm_last_target_result"] == "ik_unreachable", 1.5)


def test_10_estop_service_stops_and_blocks(ctx):
    p, _ = ctx
    stream(lambda: p.twist(vx=0.1), 0.5)
    assert wait_for(lambda: hw_wheels(p) == cmd_pulses(0.1, 0.0, 0.0), 1.0)
    t0 = time.monotonic()
    r = p.call("/armpi/emergency_stop")
    assert r.success
    assert wait_for(lambda: hw_wheels(p) == [0, 0, 0, 0], 0.5, period=0.005)
    assert time.monotonic() - t0 < 0.4
    assert wait_for(lambda: p.get("control_state")["estop_sources"] == ["service"], 1.0)
    assert wait_for(lambda: p.get("hw")["buzzer"] is True or p.get("hw")["mock_buzzer_gpio"] == 1, 0.5)
    assert wait_for(lambda: p.get("hw")["mock_buzzer_gpio"] == 0, 1.5)  # 0.5 s 뒤 꺼짐
    st = wait_for(lambda: p.get("status") if p.get("status")["emergency_stop"] else None, 1.0)
    assert st["estop_sources"] == ["service"]
    # 비상정지 중: cmd_vel 무시, 팔 목표 무시, home 거부
    w = p.get("hw")["servo_writes"]
    stream(lambda: (p.twist(vx=0.1), p.arm(0.15, 0.0, 0.1, -0.5, 0.0)), 0.6)
    assert hw_wheels(p) == [0, 0, 0, 0]
    assert p.get("hw")["servo_writes"] == w
    r = p.call("/armpi/arm/home")
    assert not r.success and "비상정지" in r.message


def test_11_estop_sources_latch_release_reset(ctx):
    p, _ = ctx
    p.estop_request("udp", True)
    assert wait_for(lambda: p.get("control_state")["estop_sources"] == ["service", "udp"], 1.0)
    p.estop_request("udp", False)  # UDP 출처만 해제
    assert wait_for(lambda: p.get("control_state")["estop_sources"] == ["service"], 1.0)
    p.estop_request("service", False)  # 서비스 latch 는 토픽으로 못 푼다
    time.sleep(0.3)
    assert p.get("control_state")["estop_sources"] == ["service"]
    p.estop_pub.publish(String(data="{bad json"))
    r = p.call("/armpi/reset_estop")
    assert r.success and "service" in r.message
    assert wait_for(lambda: p.get("control_state")["estop"] is False, 1.0)
    # 해제 뒤 예전 명령으로 다시 움직이지 않음
    time.sleep(0.3)
    assert hw_wheels(p) == [0, 0, 0, 0]
    stream(lambda: p.twist(vx=0.1), 0.6)
    assert wait_for(lambda: hw_wheels(p) == cmd_pulses(0.1, 0.0, 0.0), 1.0)
    p.twist()
    assert wait_for(lambda: hw_wheels(p) == [0, 0, 0, 0], 0.5)


def test_12_key1_button_latches_estop(ctx):
    p, _ = ctx
    p.mock(f"gpio {KEY1} 0")
    assert wait_for(lambda: p.get("control_state")["estop_sources"] == ["button"], 1.0)
    p.mock(f"gpio {KEY1} 1")
    time.sleep(0.2)
    p.estop_request("button", False)  # 버튼 latch 도 토픽으로 못 푼다
    time.sleep(0.2)
    assert p.get("control_state")["estop_sources"] == ["button"]
    assert p.call("/armpi/reset_estop").success
    assert wait_for(lambda: p.get("control_state")["estop"] is False, 1.0)


def test_13_key2_hold_requests_shutdown(ctx):
    p, _ = ctx
    p.mock(f"gpio {KEY2} 0")
    time.sleep(1.0)
    assert p.get("shutdown_count") == 0
    assert wait_for(lambda: p.get("shutdown_count") == 1, 2.0)
    time.sleep(0.5)
    p.mock(f"gpio {KEY2} 1")
    time.sleep(0.3)
    assert p.get("shutdown_count") == 1  # 한 번 누름에 한 번
    assert p.get("control_state")["estop"] is False


def test_14_joint_debug_only_without_udp_owner(ctx):
    p, _ = ctx
    p.joint_debug(["joint1", "joint2", "joint3", "joint4", "joint5", "r_joint"], [0.3, 0.2, -1.2, -1.0, 0.0, -2.0])
    d = 0.24
    want_j1 = round(500 + DIRS[0] * math.degrees(0.3) / d)
    assert wait_for(lambda: servo_cmd(p)[SERVO_IDS[0]] == want_j1, 1.0), servo_cmd(p)
    sc = servo_cmd(p)
    assert sc[SERVO_IDS[1]] == round(500 + DIRS[1] * math.degrees(0.2) / d)
    assert sc[SERVO_IDS[2]] == round(500 + DIRS[2] * math.degrees(-1.2) / d)
    assert sc[SERVO_IDS[5]] == int(PROF["arm.gripper.servo_min"])  # r_joint −2.0 rad → 그리퍼 하한으로 자름
    assert wait_for(lambda: p.get("status")["arm_mode"] == "joint_debug", 0.6)
    time.sleep(3.2)
    # UDP 조종자가 있으면 무시
    p.bridge_state(owner="d8b27f10-09a8-4c21-b649-1e7058d46b72", arm_state="idle")
    assert wait_for(lambda: p.get("status")["owner"] == "d8b27f10-09a8-4c21-b649-1e7058d46b72", 1.5)
    before = servo_cmd(p)
    p.joint_debug(["joint1"], [-0.3])
    time.sleep(0.5)
    assert servo_cmd(p) == before
    # 팔 채널 active 면 home 거부
    p.bridge_state(owner="d8b27f10-09a8-4c21-b649-1e7058d46b72", arm_state="active")
    assert wait_for(lambda: p.get("status")["arm_state"] == "active", 1.5)
    r = p.call("/armpi/arm/home")
    assert not r.success and "active" in r.message
    p.bridge_state(owner=None, arm_state="idle")
    assert wait_for(lambda: p.get("status")["owner"] is None, 1.5)


def test_15_home_service_and_udp_gate(ctx):
    p, _ = ctx
    r = p.call("/armpi/arm/home")
    assert r.success, r.message
    assert wait_for(lambda: p.get("control_state")["udp_allowed"] is False, 0.5)
    assert wait_for(lambda: servo_cmd(p) == HOME, 1.0)
    assert wait_for(lambda: p.get("control_state")["udp_allowed"] is True, 3.0)
    assert wait_for(lambda: all(abs(p.get("hw")["mock_servo_pos"][str(i)] - v) < 0.6 for i, v in HOME.items()), 3.0)


def test_16_hardware_write_failure_becomes_fault(ctx):
    p, _ = ctx
    p.mock("fail i2c 1")
    stream(lambda: p.twist(vx=0.1), 0.8)
    cs = wait_for(lambda: p.get("control_state") if p.get("control_state")["fault"] else None, 2.0)
    assert cs is not None, "fault 보고 없음"
    assert "0x34" in cs["fault_reason"]
    st = wait_for(lambda: p.get("status") if p.get("status")["fault"] else None, 2.0)
    assert st["fault"] is True
    r = p.call("/armpi/arm/home")
    assert not r.success and "fault" in r.message
    assert p.get("hw")["mock_unload_commands"] == 0
    assert all(p.get("hw")["mock_torque"].values())
