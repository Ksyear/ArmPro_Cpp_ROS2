#!/usr/bin/env python3
"""armpi_power 종단 시험(가짜 하드웨어, dry-run — 실제로 끄지 않음).

armpi_mock.launch.py(제어기 mock)와 armpi_power_node(dry_run:=true)를 띄우고
  1) KEY2(시험용 프로필의 buttons.key2_bcm)를 2 s 넘게 누르면 제어기가 /armpi/shutdown_requested 를 발행하고 power 노드가 표시 파일을 쓰는지
  2) 짧게(0.5 s) 누르면 종료 요청이 없는지
  3) 실행한 뒤 노드가 스스로 끝나는지(종료 코드 0)
를 확인한다. 가짜 버튼은 /armpi_control/mock_cmd "gpio <KEY2> 0"(누름)·"gpio <KEY2> 1"(뗌).

실행: ARMPI_ITEST_LOG_DIR=<결과> python3 -m pytest -q -s test_power_e2e.py
"""
import os
import signal
import subprocess
import time

import pytest
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

LOG_DIR = os.environ.get("ARMPI_ITEST_LOG_DIR", "/tmp")
MARKER = "/tmp/armpi_power_dry_run_marker"


def stop(p):
    try:
        os.killpg(p.pid, signal.SIGINT)
        p.wait(timeout=10)
    except Exception:  # noqa: BLE001
        try:
            os.killpg(p.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        p.wait()


@pytest.fixture(scope="module")
def ctx():
    if os.path.exists(MARKER):
        os.unlink(MARKER)
    log1 = open(os.path.join(LOG_DIR, "power_mock_stack.log"), "w")
    stack = subprocess.Popen(["ros2", "launch", "armpi_bringup", "armpi_mock.launch.py", "rosbridge:=false",
                              "vr_bridge:=false"], stdout=log1, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    node = Node("armpi_power_e2e")
    pub = node.create_publisher(String, "/armpi_control/mock_cmd", QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
    time.sleep(4.0)
    yield node, pub
    stop(stack)
    log1.close()
    node.destroy_node()
    rclpy.shutdown()



def _test_key2():
    """설치된 시험용 하드웨어 프로필(실제 로봇 값 아님)의 KEY2 핀."""
    import yaml
    from ament_index_python.packages import get_package_share_directory
    path = os.environ.get("ARMPI_TEST_PROFILE") or os.path.join(
        get_package_share_directory("armpi_bringup"), "config", "test_hardware_profile.yaml")
    with open(path, encoding="utf-8") as f:
        doc = yaml.safe_load(f)
    return int(doc["armpi_control"]["ros__parameters"]["buttons.key2_bcm"])


KEY2 = _test_key2()

def start_power(name):
    log = open(os.path.join(LOG_DIR, name + ".log"), "w")
    p = subprocess.Popen(["ros2", "run", "armpi_power", "armpi_power_node", "--ros-args", "-p", "dry_run:=true",
                          "-p", "dry_run_marker:=" + MARKER], stdout=log, stderr=subprocess.STDOUT,
                         start_new_session=True)
    time.sleep(1.5)
    return p, log


def press(node, pub, seconds):
    for cmd, wait in ((f"gpio {KEY2} 0", seconds), (f"gpio {KEY2} 1", 0.5)):
        end = time.monotonic() + wait
        while time.monotonic() < end:
            pub.publish(String(data=cmd))
            rclpy.spin_once(node, timeout_sec=0.05)


def test_01_short_press_does_not_shut_down(ctx):
    node, pub = ctx
    p, log = start_power("power_short")
    press(node, pub, 0.5)
    time.sleep(1.0)
    assert not os.path.exists(MARKER)
    assert p.poll() is None  # 계속 대기
    stop(p)
    log.close()


def test_02_long_press_requests_poweroff_once(ctx):
    node, pub = ctx
    p, log = start_power("power_long")
    press(node, pub, 2.6)
    t0 = time.monotonic()
    while time.monotonic() - t0 < 5 and p.poll() is None:
        time.sleep(0.1)
    assert os.path.exists(MARKER), "표시 파일 없음"
    with open(MARKER) as f:
        assert f.read().startswith("poweroff (dry-run) 토픽 /armpi/shutdown_requested")
    assert p.poll() == 0, "실행 뒤 노드가 끝나야 함"
    log.close()
