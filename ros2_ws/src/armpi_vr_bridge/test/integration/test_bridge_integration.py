#!/usr/bin/env python3
"""armpi_vr_bridge 통합 시험: 실제 노드를 띄우고 UDP 로 Protocol v3 패킷을 보내 ACK 와 ROS 토픽을 확인한다.

- 가짜 armpi_control: /armpi/control_state (transient_local) 를 2 Hz 로 발행하고 값을 바꿀 수 있다.
- 관찰: /armpi/cmd_vel, /armpi/arm/target, /armpi/estop_request, /armpi_vr_bridge/state
- 실제 로봇·하드웨어 없이 127.0.0.1 에서만 동작한다(ROS_LOCALHOST_ONLY=1 권장).

실행(설치 공간을 source 한 셸에서):
  python3 -m pytest -q -s test_bridge_integration.py
환경변수:
  ARMPI_BRIDGE_PARAMS   매개변수 YAML (기본: 설치된 share/armpi_vr_bridge/config/vr_bridge.yaml)
  ARMPI_ITEST_LOG_DIR   노드 출력 로그 저장 폴더 (기본: /tmp)
"""

import json
import os
import signal
import socket
import subprocess
import threading
import time
import uuid

import pytest
import rclpy
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from armpi_interfaces.msg import ArmTarget
from geometry_msgs.msg import TwistStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

PORT = 15001
DPORT = 15002
FILTERED_PORT = 15011
FILTERED_DPORT = 15012
LOG_DIR = os.environ.get("ARMPI_ITEST_LOG_DIR", "/tmp")
REL = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
LATCHED_SUB = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)


# ───────────────────────── 관찰 노드 + 가짜 제어기 ─────────────────────────

class Probe(Node):
    def __init__(self):
        super().__init__("armpi_bridge_itest_probe")
        self.lock = threading.Lock()
        self.cmd = []    # (t, vx, vy, wz, frame)
        self.arm = []    # (t, active, x, y, z, pitch, gripper, frame)
        self.estop = []  # (t, dict)
        self.state = {}  # topic -> [(t, dict)]
        self.create_subscription(TwistStamped, "/armpi/cmd_vel", self._on_cmd, REL)
        self.create_subscription(ArmTarget, "/armpi/arm/target", self._on_arm, REL)
        self.create_subscription(String, "/armpi/estop_request", self._on_estop, REL)
        self.watch_state("/armpi_vr_bridge/state")
        self.cs_pub = self.create_publisher(String, "/armpi/control_state", LATCHED)
        self.cs = None  # None 이면 발행하지 않음(일시 정지)
        self.create_timer(0.5, self._publish_cs)

    def watch_state(self, topic):
        self.state.setdefault(topic, [])
        self.create_subscription(String, topic, lambda m, t=topic: self._on_state(t, m), LATCHED_SUB)

    def _on_cmd(self, m):
        with self.lock:
            self.cmd.append((time.monotonic(), m.twist.linear.x, m.twist.linear.y, m.twist.angular.z,
                             m.header.frame_id))

    def _on_arm(self, m):
        with self.lock:
            self.arm.append((time.monotonic(), m.active, m.x, m.y, m.z, m.pitch, m.gripper, m.header.frame_id))

    def _on_estop(self, m):
        with self.lock:
            self.estop.append((time.monotonic(), json.loads(m.data)))

    def _on_state(self, topic, m):
        with self.lock:
            self.state[topic].append((time.monotonic(), json.loads(m.data)))

    def _publish_cs(self):
        cs = self.cs
        if cs is not None:
            self.cs_pub.publish(String(data=json.dumps(cs)))

    def control(self, estop=False, fault=False, udp_allowed=True, fault_reason=""):
        self.cs = {"estop": estop, "estop_sources": ["itest"] if estop else [], "fault": fault,
                   "fault_reason": fault_reason, "udp_allowed": udp_allowed}
        self._publish_cs()

    def pause_control(self):
        self.cs = None

    def snapshot(self, name):
        with self.lock:
            return list(getattr(self, name))

    def states(self, topic="/armpi_vr_bridge/state"):
        with self.lock:
            return list(self.state.get(topic, []))


def wait_for(getter, pred, timeout=1.0, since=0.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        for item in getter():
            if item[0] >= since and pred(item):
                return item
        time.sleep(0.005)
    return None


def latest_state(probe, topic="/armpi_vr_bridge/state"):
    s = probe.states(topic)
    return s[-1][1] if s else None


# ───────────────────────── 노드 프로세스 ─────────────────────────

def bridge_params_path():
    return os.environ.get("ARMPI_BRIDGE_PARAMS") or os.path.join(
        get_package_share_directory("armpi_vr_bridge"), "config", "vr_bridge.yaml")


def chassis_limits():
    """수신기에 준 매개변수 YAML 의 차체 상한(파일에 없으면 노드 기본값). 상한 값을 시험에 박지 않는다."""
    import yaml
    d = yaml.safe_load(open(bridge_params_path(), encoding="utf-8")) or {}
    vals = {}
    for sect in ("/**", "armpi_vr_bridge"):
        vals.update((d.get(sect) or {}).get("ros__parameters") or {})
    return (float(vals.get("chassis.max_vx_m_s", 0.25)), float(vals.get("chassis.max_vy_m_s", 0.25)),
            float(vals.get("chassis.max_wz_rad_s", 1.2)))


def start_bridge(tag, args):
    exe = os.path.join(get_package_prefix("armpi_vr_bridge"), "lib", "armpi_vr_bridge", "armpi_vr_bridge_node")
    params = bridge_params_path()
    log_path = os.path.join(LOG_DIR, "itest_bridge_%s.log" % tag)
    log = open(log_path, "w")
    proc = subprocess.Popen([exe, "--ros-args", "--params-file", params] + args,
                            stdout=log, stderr=subprocess.STDOUT)
    proc.log_path = log_path
    proc.log_file = log
    return proc


def stop_bridge(proc, timeout=5.0):
    if proc.poll() is None:
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()
    proc.log_file.close()
    return proc.returncode


# ───────────────────────── UDP 클라이언트 ─────────────────────────

class Client:
    def __init__(self, port=PORT, cid=None):
        self.cid = cid or str(uuid.uuid4())
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 0))
        self.seq = 0
        self.last_send = None

    def close(self):
        self.sock.close()

    def send(self, obj, timeout=0.5):
        self.sock.sendto(json.dumps(obj).encode(), ("127.0.0.1", self.port))
        self.last_send = time.monotonic()
        return self.recv_ack(obj["sequence"], timeout)

    def recv_ack(self, seq, timeout=0.5):
        deadline = time.monotonic() + timeout
        while True:
            left = deadline - time.monotonic()
            if left <= 0:
                return None
            self.sock.settimeout(left)
            try:
                data, addr = self.sock.recvfrom(4096)
            except socket.timeout:
                return None
            ack = json.loads(data)
            assert addr[1] == self.port, "ACK 출발 포트가 다름: %r" % (addr,)
            assert list(ack) == ["protocol", "type", "client_id", "sequence", "accepted", "reason",
                                 "emergency_stop", "state"], ack
            if ack["client_id"] == self.cid and ack["sequence"] == seq:
                return ack

    def cmd(self, active, forward=0.0, strafe=0.0, turn=0.0):
        self.seq += 1
        return self.send({"protocol": 3, "type": "command", "client_id": self.cid, "sequence": self.seq,
                          "control_active": active, "forward": forward, "strafe": strafe, "turn": turn})

    # 기본 목표 = 기본 자세 손끝 목표(armpi.yaml arm.home, 링크 길이와 시험용 관절 기준에서 IK 해 있음)
    def arm(self, active, x=0.1047, y=0.0, z=0.0753, pitch=-1.0263, gripper=0.5):
        self.seq += 1
        return self.send({"protocol": 3, "type": "arm_command", "client_id": self.cid, "sequence": self.seq,
                          "control_active": active, "x": x, "y": y, "z": z, "pitch": pitch, "gripper": gripper})

    def event(self, kind):
        self.seq += 1
        return self.send({"protocol": 3, "type": kind, "client_id": self.cid, "sequence": self.seq})


def expect(ack, accepted, reason, state=None, estop=None):
    assert ack is not None, "ACK 없음"
    assert ack["accepted"] is accepted and ack["reason"] == reason, ack
    if state is not None:
        assert ack["state"] == state, ack
    if estop is not None:
        assert ack["emergency_stop"] is estop, ack


def near(a, b, tol=1e-9):
    return abs(a - b) <= tol


# ───────────────────────── 픽스처 ─────────────────────────

class Env:
    pass


@pytest.fixture(scope="module")
def env():
    rclpy.init()
    probe = Probe()
    executor = SingleThreadedExecutor()
    executor.add_node(probe)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    bridge = start_bridge("main", ["-p", "port:=%d" % PORT, "-p", "discovery.port:=%d" % DPORT,
                                   "-p", "discovery.name:=armpi-itest"])
    ok = wait_for(probe.states, lambda s: True, timeout=15.0)
    if ok is None:
        stop_bridge(bridge)
        pytest.fail("노드 상태 토픽을 받지 못함 (로그 %s)" % bridge.log_path)
    e = Env()
    e.probe, e.bridge = probe, bridge
    yield e
    stop_bridge(bridge)
    executor.shutdown()
    probe.destroy_node()
    rclpy.shutdown()


# ───────────────────────── 시험 (파일 순서대로 실행) ─────────────────────────

def test_01_gate_before_control_state(env):
    """control_state 를 받기 전: 중립은 수락, 활성 명령은 not_owner, 출력은 0."""
    t0 = time.monotonic()
    c = Client()
    expect(c.cmd(False), True, "ok", "idle")
    expect(c.cmd(True, 0.5), False, "not_owner", "idle")
    expect(c.arm(False), True, "ok", "idle")
    expect(c.arm(True), False, "not_owner", "idle")
    time.sleep(0.2)
    cmds = [m for m in env.probe.snapshot("cmd") if m[0] >= t0]
    assert cmds, "cmd_vel 이 20 Hz 로 발행되어야 함"
    assert all(m[1] == 0.0 and m[2] == 0.0 and m[3] == 0.0 for m in cmds)
    st = latest_state(env.probe)
    assert st["udp_permitted"] is False and "control_state" in st["blocked_reason"], st
    expect(c.event("disconnect"), True, "ok")
    c.close()


def test_02_base_drive_mapping_rate_and_watchdog(env):
    env.probe.control()
    assert wait_for(env.probe.states, lambda s: s[1]["udp_permitted"], timeout=2.0)
    c = Client()
    expect(c.cmd(False), True, "ok", "idle")
    expect(c.cmd(True, 0.5, -0.5, 0.25), True, "ok", "active")
    t_cmd = c.last_send
    mx, my, mw = chassis_limits()
    want = (0.5 * mx, -0.5 * my, 0.25 * mw)
    m = wait_for(lambda: env.probe.snapshot("cmd"),
                 lambda m: near(m[1], want[0]) and near(m[2], want[1]) and near(m[3], want[2]), timeout=0.5, since=t_cmd)
    assert m is not None, "vx=%.3f, vy=%.3f(+왼쪽 반대), wz=%.3f cmd_vel 을 받지 못함" % want
    assert m[4] == "base_link"
    assert m[0] - t_cmd < 0.1, "값 변경 시 즉시 발행되어야 함 (%.3fs)" % (m[0] - t_cmd)
    # 1.0 s 동안 20 Hz 로 같은 명령 → 주기 발행 확인
    w0 = time.monotonic()
    while time.monotonic() - w0 < 1.0:
        expect(c.cmd(True, 0.5, -0.5, 0.25), True, "ok", "active")
        time.sleep(0.05)
    t_last = c.last_send
    n = len([m for m in env.probe.snapshot("cmd") if w0 <= m[0] <= t_last])
    print("\n[02] 1.0 s 동안 cmd_vel %d 개" % n)
    assert 15 <= n <= 45, n
    # 송신을 멈추면 watchdog(0.4 s) 뒤 0
    z = wait_for(lambda: env.probe.snapshot("cmd"), lambda m: m[1] == 0.0 and m[2] == 0.0 and m[3] == 0.0,
                 timeout=1.5, since=t_last)
    assert z is not None
    latency = z[0] - t_last
    print("[02] 마지막 활성 명령 → cmd_vel 0: %.3f s" % latency)
    assert 0.35 <= latency <= 0.65, latency
    expect(c.cmd(True, 0.5), False, "neutral_required", "timeout")
    expect(c.cmd(False), True, "ok", "idle")
    expect(c.event("disconnect"), True, "ok")
    c.close()


def test_03_arm_target_reject_rules_republish_and_watchdog(env):
    c = Client()
    expect(c.cmd(False), True, "ok", "idle")
    # 앞 시험이 같은 기본 좌표로 중립을 보냈으면 출력이 같아 즉시 재발행되지 않으므로(변화 시에만 발행) 다른 좌표를 쓴다
    expect(c.arm(False, 0.11, 0.0, 0.075, -1.0, 0.2), True, "ok", "idle")
    t_neutral = c.last_send
    assert wait_for(lambda: env.probe.snapshot("arm"), lambda m: m[1] is False, timeout=0.5, since=t_neutral)
    expect(c.arm(True, 0.12, 0.02, 0.07, -1.0, 0.7), True, "ok", "active")
    t_acc = c.last_send
    m = wait_for(lambda: env.probe.snapshot("arm"), lambda m: m[1] is True, timeout=0.5, since=t_acc)
    assert m is not None
    assert near(m[2], 0.12) and near(m[3], 0.02) and near(m[4], 0.07) and near(m[5], -1.0) and near(m[6], 0.7)
    assert m[7] == "armpi_arm_base"
    # 작업 공간 밖 / IK 해 없음 → 거부, 이전 목표 유지
    expect(c.arm(True, 0.5, 0.0, 0.05), False, "workspace_limit", "active")
    expect(c.arm(True, 0.28, 0.0, 0.30, 0.0), False, "ik_unreachable", "active")  # 작업 공간 안, 실제 치수로 닿지 않음
    time.sleep(0.25)
    window = [m for m in env.probe.snapshot("arm") if t_acc <= m[0] <= t_acc + 0.3]
    print("\n[03] 수락 후 0.3 s 동안 ArmTarget %d 개" % len(window))
    assert len(window) >= 4, "팔 active 동안 20 Hz 재발행이어야 함"
    assert all(mm[1] is True and near(mm[2], 0.12) for mm in window), window
    # 거부는 watchdog 을 갱신하지 않으므로 수락 0.4 s 뒤 active:false
    off = wait_for(lambda: env.probe.snapshot("arm"), lambda m: m[1] is False, timeout=1.0, since=t_acc)
    assert off is not None
    print("[03] 마지막 수락 → ArmTarget active:false: %.3f s" % (off[0] - t_acc))
    assert 0.35 <= off[0] - t_acc <= 0.65
    assert near(off[2], 0.12), "정지 시에도 마지막 목표 값을 담아 보냄"
    time.sleep(0.3)
    after = [m for m in env.probe.snapshot("arm") if m[0] > off[0]]
    assert after == [], "비활성 뒤에는 재발행하지 않음: %r" % after[:3]
    expect(c.arm(True), False, "neutral_required", "timeout")
    expect(c.event("disconnect"), True, "ok")
    c.close()


def test_04_emergency_stop_roundtrip(env):
    c = Client()
    expect(c.cmd(False), True, "ok", "idle")
    expect(c.arm(False), True, "ok", "idle")
    expect(c.cmd(True, 0.5), True, "ok", "active")
    t_e = time.monotonic()
    expect(c.event("emergency_stop"), True, "ok", "emergency_stop", estop=True)
    r = wait_for(lambda: env.probe.snapshot("estop"), lambda m: m[1] == {"source": "udp", "latch": True},
                 timeout=0.5, since=t_e)
    assert r is not None, "estop_request latch:true 미발행"
    assert wait_for(lambda: env.probe.snapshot("cmd"), lambda m: m[1] == 0.0, timeout=0.3, since=t_e)
    env.probe.control(estop=True)  # 가짜 제어기 latch
    time.sleep(0.2)
    expect(c.cmd(True, 0.5), False, "emergency_stop", "emergency_stop", estop=True)
    expect(c.arm(True), False, "emergency_stop", "emergency_stop")
    other = Client()
    expect(other.event("emergency_stop"), True, "ok", "emergency_stop")  # 비소유자 비상정지도 수락
    expect(other.event("clear_emergency"), False, "not_owner")            # 해제는 소유자만
    other.close()
    t_c = time.monotonic()
    expect(c.event("clear_emergency"), True, "ok", "emergency_stop", estop=True)  # 제어기가 풀기 전까지 유지
    r = wait_for(lambda: env.probe.snapshot("estop"), lambda m: m[1] == {"source": "udp", "latch": False},
                 timeout=0.5, since=t_c)
    assert r is not None, "estop_request latch:false 미발행"
    env.probe.control(estop=False)
    time.sleep(0.2)
    expect(c.cmd(True, 0.5), False, "neutral_required", "awaiting_neutral", estop=False)
    expect(c.arm(True), False, "neutral_required", "awaiting_neutral")
    expect(c.cmd(False), True, "ok", "idle")
    expect(c.cmd(True, 0.5), True, "ok", "active")
    expect(c.event("disconnect"), True, "ok")
    c.close()


def test_05_controller_fault_and_udp_allowed(env):
    c = Client()
    expect(c.cmd(False), True, "ok", "idle")
    env.probe.control(fault=True, fault_reason="itest fault")
    time.sleep(0.2)
    expect(c.cmd(True, 0.5), False, "fault", "fault")
    expect(c.arm(True), False, "fault", "fault")
    assert "itest fault" in (latest_state(env.probe).get("fault") or "")
    env.probe.control(udp_allowed=False)
    time.sleep(0.2)
    expect(c.cmd(False), True, "ok", "idle")
    expect(c.cmd(True, 0.5), False, "not_owner", "idle")
    env.probe.control()
    time.sleep(0.2)
    expect(c.cmd(False), True, "ok", "idle")
    expect(c.cmd(True, 0.5), True, "ok", "active")
    expect(c.event("disconnect"), True, "ok")
    c.close()


def test_06_stale_control_state_blocks_active(env):
    env.probe.pause_control()
    c = Client()
    expect(c.cmd(False), True, "ok", "idle")
    t0 = time.monotonic()
    while time.monotonic() - t0 < 2.4:  # 중립으로 watchdog 은 유지하며 control_state 만 오래되게
        expect(c.cmd(False), True, "ok")
        time.sleep(0.1)
    expect(c.cmd(True, 0.5), False, "not_owner", "idle")
    st = latest_state(env.probe)
    assert st["udp_permitted"] is False and "미수신" in st["blocked_reason"], st
    env.probe.control()
    time.sleep(0.2)
    expect(c.cmd(False), True, "ok", "idle")
    expect(c.cmd(True, 0.5), True, "ok", "active")
    expect(c.event("disconnect"), True, "ok")
    c.close()


def test_07_discovery_reply(env):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    s.settimeout(1.0)
    nonce = uuid.uuid4().hex
    s.sendto(("ARMPI_DISCOVER/1|" + nonce).encode(), ("127.0.0.1", DPORT))
    data, addr = s.recvfrom(1024)
    fields = data.decode("ascii").split("|")
    assert addr[1] == DPORT, "응답 출발 포트는 검색 포트여야 함"
    assert fields[0] == "ARMPI_HERE/1" and fields[1] == nonce
    assert len(fields[2]) == 32 and all(ch in "0123456789abcdef" for ch in fields[2])
    assert fields[3:] == ["armpi-itest", str(PORT), "3"], fields
    assert 80 <= len(data) <= 640
    s.sendto(b"OTHERBOT_DISCOVER/1|" + nonce.encode(), ("127.0.0.1", DPORT))
    with pytest.raises(socket.timeout):
        s.settimeout(0.3)
        s.recvfrom(1024)
    s.close()


def test_08_state_topic_is_transient_local(env):
    late = rclpy.create_node("armpi_bridge_itest_late")
    got = []
    late.create_subscription(String, "/armpi_vr_bridge/state", lambda m: got.append(json.loads(m.data)),
                             LATCHED_SUB)
    deadline = time.monotonic() + 2.0
    while not got and time.monotonic() < deadline:
        rclpy.spin_once(late, timeout_sec=0.1)
    late.destroy_node()
    assert got, "늦게 붙은 구독자도 마지막 상태를 받아야 함(transient_local)"
    for key in ("owner", "owner_addr", "base_state", "arm_state", "emergency_stop", "packets"):
        assert key in got[0], key
    assert got[0]["packets"]["received"] > 0


def test_09_allowed_sources_filter(env):
    topic = "/armpi_vr_bridge_filtered/state"
    env.probe.watch_state(topic)
    proc = start_bridge("filtered", [
        "-r", "__node:=armpi_vr_bridge_filtered",
        "-p", "port:=%d" % FILTERED_PORT, "-p", "discovery.port:=%d" % FILTERED_DPORT,
        "-p", "allowed_sources:=[\"10.0.0.0/8\"]",
        "-p", "cmd_vel_topic:=/itest_filtered/cmd_vel", "-p", "arm_target_topic:=/itest_filtered/arm/target",
        "-p", "estop_request_topic:=/itest_filtered/estop_request",
        "-p", "control_state_topic:=/itest_filtered/control_state"])
    try:
        assert wait_for(lambda: env.probe.states(topic), lambda s: True, timeout=15.0), proc.log_path
        c = Client(port=FILTERED_PORT)
        assert c.cmd(False) is None, "allowed_sources 밖 주소에는 응답하지 않아야 함"
        c.close()
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind(("127.0.0.1", 0))
        s.settimeout(0.5)
        s.sendto(("ARMPI_DISCOVER/1|" + uuid.uuid4().hex).encode(), ("127.0.0.1", FILTERED_DPORT))
        with pytest.raises(socket.timeout):
            s.recvfrom(1024)
        s.close()
        st = wait_for(lambda: env.probe.states(topic),
                      lambda s: s[1]["packets"]["filtered"] >= 1 and s[1]["discovery"]["filtered"] >= 1, timeout=2.0)
        assert st is not None
        assert st[1]["packets"]["received"] == 0
        assert st[1]["discovery"]["filtered"] >= 1
    finally:
        assert stop_bridge(proc) == 0


def test_10_shutdown_publishes_stop_three_times(env):
    c = Client()
    expect(c.cmd(False), True, "ok", "idle")
    expect(c.arm(False), True, "ok", "idle")
    expect(c.cmd(True, 0.5), True, "ok", "active")
    expect(c.arm(True), True, "ok", "active")
    t_sig = time.monotonic()
    env.bridge.send_signal(signal.SIGINT)
    assert env.bridge.wait(5.0) == 0
    time.sleep(0.5)
    cmds = [m for m in env.probe.snapshot("cmd") if m[0] >= t_sig]
    arms = [m for m in env.probe.snapshot("arm") if m[0] >= t_sig]
    print("\n[10] SIGINT 뒤 cmd_vel %d 개, ArmTarget %d 개" % (len(cmds), len(arms)))
    assert len(cmds) >= 3 and all(m[1] == 0.0 and m[2] == 0.0 and m[3] == 0.0 for m in cmds[-3:]), cmds[-4:]
    assert len(arms) >= 3 and all(m[1] is False for m in arms[-3:]), arms[-4:]
    with open(env.bridge.log_path) as f:
        assert "정지 명령(cmd_vel 0, ArmTarget active:false) 3회 발행" in f.read()
    c.close()
