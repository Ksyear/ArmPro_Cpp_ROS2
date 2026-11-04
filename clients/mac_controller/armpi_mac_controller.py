#!/usr/bin/env python3
"""맥 ArmPi 조종기 (표준 라이브러리만 사용, 화면은 브라우저).

  python3 armpi_mac_controller.py --robot auto            # UDP 5002 검색으로 로봇 찾기(같은 망)
  python3 armpi_mac_controller.py --robot 192.168.0.50
  python3 armpi_mac_controller.py --robot 127.0.0.1       # Mac 에서 가짜 로봇(mock_robot.py)과 시험

브라우저: http://127.0.0.1:18765 (기본은 이 맥에서만 열리는 화면), 휴대폰용 터치 화면은 /mobile

  # 로봇 안에서 돌려 같은 와이파이의 휴대폰 브라우저가 접속하게 할 때(systemd armpi-web.service)
  python3 armpi_mac_controller.py --robot 127.0.0.1 --bind 0.0.0.0 --http-port 18765 --no-browser

- 조종: Protocol v3 (UDP 5001, ../vr_protocol_v3/armpi_v3_client.py) — VR 과 같은 안전 경로.
  차체·팔 채널을 각각 50 ms 마다 보낸다. 키를 떼거나 창 포커스를 잃거나 브라우저 입력이 0.3 s 끊기면 두 채널 모두 중립.
  로봇은 중립 확인 전 활성 명령 거부, 채널별 0.4 s watchdog, 비상정지 latch 를 그대로 적용한다.
- 상태·서비스(선택): rosbridge (TCP 9090). 서버가 대신 연결해 /api/state 로 중계한다(브라우저는 9090 에 직접 붙지 않음).
  없으면 "rosbridge 미연결"로 표시하고 UDP 조종은 계속 동작.
- 영상: 브라우저가 로봇의 web_video_server(8080)에서 직접 받는다. 로봇 주소가 127.0.0.1 인데 화면을 다른 기기에서
  열었으면 화면(JS)이 주소창의 호스트(window.location.hostname)로 바꿔 받는다.
- 조종권: 여러 화면(브라우저·앱)이 같은 서버를 열어도 한 화면만 조종한다(화면 토큰). 나머지 화면은
  "다른 기기가 조종 중"으로 표시되고 입력이 무시된다. 비상정지는 어느 화면에서나 된다.
- 세션 방식(--session): always = 서버가 켜져 있는 동안 v3 세션을 계속 잡음(맥 기본),
  on-demand = 조종권을 잡은 화면이 있을 때만 세션을 연다(--bind 가 127.0.0.1 이 아니면 기본). 그래야 로봇 안에서 상주해도
  VR 등 다른 조종기를 막지 않는다.

../vr_protocol_v3/ 폴더가 이 폴더 옆에 있어야 한다(상대 경로 import). 로봇 배치 /opt/armpi/mac_controller/ +
/opt/armpi/vr_protocol_v3/ 과 맥 앱 번들 Resources/mac_controller/ + Resources/vr_protocol_v3/ 모두 이 배치다.
"""

import argparse
import base64
import ipaddress
import json
import math
import os
import re
import signal
import socket
import socketserver
import struct
import sys
import threading
import time
import webbrowser
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs

HERE = os.path.dirname(os.path.abspath(__file__))
# 웹 화면 파일(index.html·mobile.html·아이콘)의 기준 위치는 ros2_ws/src/armpi_web_gateway/www/ 한 곳에 있다.
# 맥 앱 번들은 빌드 때 이 폴더 옆에 복사한다. 있는 곳을 쓴다.
WWW = HERE if os.path.isfile(os.path.join(HERE, "mobile.html")) else \
    os.path.join(HERE, "..", "..", "ros2_ws", "src", "armpi_web_gateway", "www")
sys.path.insert(0, HERE)  # armpi_v3_client.py 를 이 폴더에 같이 둔 배치도 허용(아래 경로가 우선)
# 배치별 v3 클라이언트 위치: 저장소(clients/mac_controller → ../../protocol_v3),
# 맥 앱 번들·로봇 v1 이미지(mac_controller 옆 ../vr_protocol_v3). 있는 곳이 우선.
for _rel in (("..", "vr_protocol_v3"), ("..", "..", "protocol_v3")):
    _p = os.path.join(HERE, *_rel)
    if os.path.isdir(_p):
        sys.path.insert(0, _p)
from armpi_v3_client import (ACK_TIMEOUT, CONTROL_PORT, DEFAULT_HOME, DISCOVERY_PORT,  # noqa: E402
                             ArmPiV3Client, discover)

INPUT_TIMEOUT = 0.3       # 브라우저 입력이 이 시간 끊기면 중립
LOOP_PERIOD = 0.05        # 채널마다 20 Hz
STATUS_FRESH_S = 2.0      # /armpi/status 가 이보다 오래되면 '오래됨'
JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "r_joint"]
JOINT_LIMIT_RAD = 2.0944  # ±120° (서보 0~1000 × 0.24°)
OWNER_TIMEOUT = 1.5       # 조종권을 가진 화면이 이 시간 동안 아무 요청도 없으면 조종권이 풀린다
ESTOP_LINGER_S = 2.0      # on-demand 에서 비상정지를 보내려고 연 세션을 유지하는 최소 시간
SCREEN_ID_RE = re.compile(r"\A[A-Za-z0-9_-]{1,64}\Z")
# 조종권 없이도 받는 명령(안전·조종권 관리). 나머지 명령은 조종권을 가진 화면만(없으면 잡고) 실행한다.
ANY_SCREEN_CMDS = {"estop", "robot_estop", "take_control", "release_control"}
OWNER_CMDS = {"clear", "disconnect", "connect", "arm_enable", "arm_target", "gripper", "arm_sync", "arm_home_target",
              "robot_reset", "arm_home", "joint_debug"}
STATIC_FILES = {  # 경로 → (파일, Content-Type, 캐시)
    "/": ("index.html", "text/html; charset=utf-8", False),
    "/index.html": ("index.html", "text/html; charset=utf-8", False),
    "/mobile": ("mobile.html", "text/html; charset=utf-8", False),
    "/mobile.html": ("mobile.html", "text/html; charset=utf-8", False),
    "/manifest.webmanifest": ("manifest.webmanifest", "application/manifest+json; charset=utf-8", False),
    "/icon-192.png": ("icon-192.png", "image/png", True),
    "/icon-512.png": ("icon-512.png", "image/png", True),
    "/apple-touch-icon.png": ("icon-192.png", "image/png", True),
    "/favicon.ico": ("icon-192.png", "image/png", True),
}
MOBILE_UA_RE = re.compile(r"Android|iPhone|iPad|iPod|Mobile", re.I)
LOOPBACK_NAMES = {"127.0.0.1", "localhost", "::1"}


def _print(text):
    try:
        print(text, flush=True)
    except (OSError, ValueError):
        pass


def _ts():
    t = time.time()
    return time.strftime("%H:%M:%S", time.localtime(t)) + ".%03d" % int((t % 1) * 1000)


class RosBridge:
    """최소 rosbridge v2 WebSocket 클라이언트(표준 라이브러리). 끊기면 2 s 뒤 다시 연결."""

    def __init__(self, host, port=9090, log=print):
        self.host, self.port = host, port
        self.log = log
        self.connected = False
        self.status = None
        self.status_time = None
        self.battery = None
        self.battery_time = None
        self.last_result = None
        self.connects = 0
        self._sock = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._ids = 0
        threading.Thread(target=self._loop, name="rosbridge", daemon=True).start()

    def close(self):
        self._stop.set()
        s = self._sock
        if s:
            try:
                s.close()
            except OSError:
                pass

    def _handshake(self):
        s = socket.create_connection((self.host, self.port), timeout=3)
        key = base64.b64encode(os.urandom(16)).decode()
        s.sendall(("GET / HTTP/1.1\r\nHost: %s:%d\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
                   "Sec-WebSocket-Key: %s\r\nSec-WebSocket-Version: 13\r\n\r\n" % (self.host, self.port, key)).encode())
        resp = b""
        while b"\r\n\r\n" not in resp:
            chunk = s.recv(1024)
            if not chunk:
                raise OSError("handshake 중 연결 종료")
            resp += chunk
        if b" 101 " not in resp.split(b"\r\n")[0]:
            raise OSError("handshake 거부")
        self._pending = resp.split(b"\r\n\r\n", 1)[1]
        s.settimeout(None)
        return s

    def _recv_exact(self, n):
        buf = self._pending[:n]
        self._pending = self._pending[n:]
        while len(buf) < n:
            chunk = self._sock.recv(n - len(buf))
            if not chunk:
                raise OSError("연결 종료")
            buf += chunk
        return buf

    def _recv_message(self):
        parts, first = [], None
        while True:
            b0, b1 = self._recv_exact(2)
            op, length = b0 & 0x0F, b1 & 0x7F
            if length == 126:
                length = struct.unpack("!H", self._recv_exact(2))[0]
            elif length == 127:
                length = struct.unpack("!Q", self._recv_exact(8))[0]
            mask = self._recv_exact(4) if b1 & 0x80 else None
            data = self._recv_exact(length)
            if mask:
                data = bytes(b ^ mask[i % 4] for i, b in enumerate(data))
            if op >= 0x8:
                return op, data
            first = op if first is None else first
            parts.append(data)
            if b0 & 0x80:
                return first, b"".join(parts)

    def _send_frame(self, data, op=0x1):
        mask = os.urandom(4)
        n = len(data)
        hdr = bytearray([0x80 | op])
        if n < 126:
            hdr.append(0x80 | n)
        elif n < 65536:
            hdr.append(0x80 | 126)
            hdr += struct.pack("!H", n)
        else:
            hdr.append(0x80 | 127)
            hdr += struct.pack("!Q", n)
        payload = bytes(b ^ mask[i % 4] for i, b in enumerate(data))
        with self._lock:
            self._sock.sendall(bytes(hdr) + mask + payload)

    def send(self, obj):
        if not self.connected:
            raise OSError("rosbridge 미연결")
        self._send_frame(json.dumps(obj).encode("utf-8"))

    def _loop(self):
        while not self._stop.is_set():
            try:
                self._sock = self._handshake()
                self.connected = True
                self.connects += 1
                self.log("rosbridge 연결: %s:%d" % (self.host, self.port))
                self.send({"op": "subscribe", "topic": "/armpi/status", "type": "std_msgs/String"})
                self.send({"op": "subscribe", "topic": "/armpi/battery", "type": "sensor_msgs/BatteryState"})
                self.send({"op": "advertise", "topic": "/armpi/arm/joint_debug", "type": "sensor_msgs/JointState"})
                while not self._stop.is_set():
                    op, data = self._recv_message()
                    if op == 0x8:
                        raise OSError("로봇이 연결을 닫음")
                    if op == 0x9:
                        self._send_frame(data, op=0xA)
                        continue
                    if op != 0x1:
                        continue
                    self._on_message(json.loads(data.decode("utf-8")))
            except (OSError, ValueError) as exc:
                if self.connected and not self._stop.is_set():  # 종료 중에 닫은 것은 기록하지 않음
                    self.log("rosbridge 끊김: %s" % exc)
            self.connected = False
            if self._sock:
                try:
                    self._sock.close()
                except OSError:
                    pass
            self._stop.wait(2.0)

    def _on_message(self, msg):
        op = msg.get("op")
        if op == "publish":
            topic, body = msg.get("topic"), msg.get("msg") or {}
            if topic == "/armpi/status":
                try:
                    self.status = json.loads(body.get("data", ""))
                    self.status_time = time.monotonic()
                except (ValueError, TypeError):
                    pass
            elif topic == "/armpi/battery":
                self.battery = {"voltage": body.get("voltage"), "percentage": body.get("percentage")}
                self.battery_time = time.monotonic()
        elif op == "service_response":
            vals = msg.get("values")
            if isinstance(vals, dict):
                ok, text = bool(msg.get("result")) and bool(vals.get("success", True)), vals.get("message", "")
            else:
                ok, text = False, str(vals)
            self.last_result = {"service": msg.get("service"), "ok": ok, "message": text, "time": time.time()}
            self.log("서비스 응답 %s: %s %s" % (msg.get("service"), "성공" if ok else "실패", text))

    def call(self, service, srv_type="std_srvs/Trigger", args=None):
        self._ids += 1
        self.send({"op": "call_service", "service": service, "type": srv_type, "args": args or {},
                   "id": "mac-%d-%d" % (int(time.time()), self._ids)})

    def publish(self, topic, msg):
        self.send({"op": "publish", "topic": topic, "msg": msg})

    def fresh_status(self):
        if self.status is None or self.status_time is None or time.monotonic() - self.status_time > STATUS_FRESH_S:
            return None
        return self.status

    def snapshot(self):
        now = time.monotonic()
        return {"enabled": True, "connected": self.connected, "status": self.status,
                "status_age_s": round(now - self.status_time, 2) if self.status_time else None,
                "battery": self.battery,
                "battery_age_s": round(now - self.battery_time, 2) if self.battery_time else None,
                "last_result": self.last_result}


class _Owner:
    """조종권을 가진 화면 하나."""
    __slots__ = ("id", "name", "addr", "since", "last", "last_intent", "last_n")

    def __init__(self, sid, name, addr, now):
        self.id, self.name, self.addr = sid, name, addr
        self.since = self.last = self.last_intent = now
        self.last_n = None

    def label(self):
        return "%s(%s)" % (self.name or "API", self.addr or "?")


def _screen_name(value):
    text = "".join(ch for ch in str(value or "") if ch.isprintable())
    return text.strip()[:40]


NEUTRAL_INPUT = {"f": 0.0, "s": 0.0, "t": 0.0, "held": False, "focused": False, "arm": [0, 0, 0, 0, 0]}


class Controller:
    """브라우저 입력(/api/input)을 받아 50 ms 마다 v3 패킷을 보내는 루프.

    조종권(화면 토큰): 입력·명령에 실린 ``screen`` 값으로 화면을 구분한다(없으면 "api" 한 화면으로 본다).
    조종권이 비어 있을 때 실제 조작(누름·팔 방향·명령)을 보낸 화면이 조종권을 잡고, 그 화면이 OWNER_TIMEOUT 동안
    아무 요청도 보내지 않으면 풀린다. 다른 화면은 ``take_control`` 로만 가져올 수 있다(이때 입력은 중립으로 되돌리고
    팔 조종 토글을 끈다). 비상정지(estop·robot_estop)는 조종권과 관계없이 받는다.
    """

    def __init__(self, robot, port, ack_timeout, home, rates, rb, on_demand=False, idle_release=0.0):
        self.events = deque(maxlen=60)
        self.client = ArmPiV3Client(robot, port, ack_timeout=ack_timeout, home=home, log=self.log)
        self.home = dict(home)
        self.rates = rates  # (m/s, rad/s, /s)
        self.rb = rb
        self.lock = threading.Lock()
        self.inp = dict(NEUTRAL_INPUT)
        self.last_input = 0.0
        self.arm_enabled = False
        self.running = True
        self.loop_overruns = 0
        self.on_demand = bool(on_demand)
        self.idle_release = float(idle_release or 0.0)
        self.session_live = not self.on_demand   # always 모드는 시작부터 세션을 잡는다(기존 동작)
        self._live_until = 0.0
        self.owner = None
        self._last_owner_id = None
        self.control_changes = 0
        self.screens = {}   # 최근 요청한 화면: sid → [이름, 주소, 마지막 시각]
        self._thread = threading.Thread(target=self._loop, name="v3-loop", daemon=True)
        self._thread.start()

    def log(self, msg):
        line = "[%s] %s" % (_ts(), msg)
        self.events.append(line)
        try:
            print(line, flush=True)
        except (OSError, ValueError):  # 표준 출력을 읽던 부모(맥 앱)가 사라진 경우 등 — 조종 루프는 계속
            pass

    # ── 조종권 ──

    def _neutral_locked(self):
        self.inp = dict(NEUTRAL_INPUT)
        self.last_input = 0.0

    def _expire_owner_locked(self, now):
        o = self.owner
        if o is None:
            return
        if now - o.last > OWNER_TIMEOUT:
            why = "화면 응답 없음 %.1f s" % OWNER_TIMEOUT
        elif self.on_demand and self.idle_release > 0 and now - o.last_intent > self.idle_release:
            why = "%.0f s 동안 조작 없음" % self.idle_release
            self._last_owner_id = None  # 조작 없이 다시 잡지 못하게(그래야 다른 조종기가 로봇을 쓸 수 있음)
        else:
            return
        self.owner = None
        self._neutral_locked()
        self.control_changes += 1
        self.log("조종권 풀림: %s — %s" % (o.label(), why))

    def _seen_locked(self, sid, name, addr, now):
        self.screens[sid] = [name or "API", addr, now]
        if len(self.screens) > 32:
            for k in [k for k, v in self.screens.items() if now - v[2] > 10.0]:
                del self.screens[k]

    def _claim_locked(self, sid, name, addr, intent, focused, force=False):
        """sid 화면이 조종권을 가지면(또는 지금 잡으면) True."""
        now = time.monotonic()
        self._seen_locked(sid, name, addr, now)
        self._expire_owner_locked(now)
        o = self.owner
        if o is not None and o.id == sid:
            o.last = now
            o.addr = addr or o.addr
            if name:
                o.name = name
            if intent:
                o.last_intent = now
            return True
        if o is not None and not force:
            return False
        if o is None and not force and not intent and not (focused and sid == self._last_owner_id):
            return False
        if sid != self._last_owner_id or o is not None:
            # 다른 화면으로 넘어감: 이전 화면의 입력·팔 조종 토글을 버린다.
            self._neutral_locked()
            if self.arm_enabled:
                self.arm_enabled = False
            self.control_changes += 1
            self.log("조종권: %s%s" % ("%s → " % o.label() if o is not None else "",
                                      "%s(%s)" % (name or "API", addr or "?")))
        self.owner = _Owner(sid, name, addr, now)
        self._last_owner_id = sid
        return True

    def release_control(self, sid):
        with self.lock:
            o = self.owner
            if o is None or o.id != sid:
                return False
            self.owner = None
            self._last_owner_id = None
            self._neutral_locked()
            self.arm_enabled = False
            self._live_until = 0.0
            self.control_changes += 1
        self.log("조종권 놓음: %s" % o.label())
        return True

    def control_snapshot(self, sid):
        now = time.monotonic()
        with self.lock:
            self._expire_owner_locked(now)
            o = self.owner
            owner = None
            if o is not None:
                owner = {"name": o.name or "API", "addr": o.addr, "short": o.id[:4],
                         "held_s": round(now - o.since, 1), "idle_s": round(now - o.last_intent, 1)}
            screens = [{"name": v[0], "addr": v[1], "short": k[:4], "age_s": round(now - v[2], 1), "you": k == sid}
                       for k, v in self.screens.items() if now - v[2] <= 3.0]
            return {"owner": owner, "you": o is not None and o.id == sid, "timeout_s": OWNER_TIMEOUT, "screens": screens,
                    "idle_release_s": self.idle_release if self.on_demand else 0, "changes": self.control_changes}

    # ── 입력 ──

    def set_input(self, req, sid="api", name="", addr=""):
        def axis(key):
            try:
                v = float(req.get(key, 0.0))
            except (TypeError, ValueError):
                return 0.0
            return max(-1.0, min(1.0, v)) if math.isfinite(v) else 0.0
        arm = req.get("arm") or {}
        if not isinstance(arm, dict):
            arm = {}
        dirs = []
        for k in ("x", "y", "z", "p", "g"):
            try:
                v = int(arm.get(k, 0))
            except (TypeError, ValueError):
                v = 0
            dirs.append(max(-1, min(1, v)))
        held, focused = bool(req.get("held")), bool(req.get("focused"))
        n = req.get("n")
        n = n if type(n) is int else None
        with self.lock:
            if not self._claim_locked(sid, name, addr, focused and (held or any(dirs)), focused):
                return {"ok": True, "control": False}
            o = self.owner
            if n is not None:
                if o.last_n is not None and n <= o.last_n:
                    return {"ok": True, "control": True, "stale": True}  # 늦게 도착한 옛 입력은 버림
                o.last_n = n
            self.inp = {"f": axis("f"), "s": axis("s"), "t": axis("t"), "held": held, "focused": focused, "arm": dirs}
            self.last_input = time.monotonic()
        return {"ok": True, "control": True}

    # ── 주기 루프 ──

    def _session_reason_locked(self, now):
        """세션이 열린 이유: none·always·user·estop_only(조종권 없이 비상정지만 보내려고 잠깐 연 세션)·closing."""
        if not self.session_live:
            return "none"
        if not self.on_demand:
            return "always"
        if self.owner is not None:
            return "user"
        if now < self._live_until or self.client.pending_estop:
            return "estop_only"
        return "closing"

    def _session_step_locked(self, now):
        """on-demand 세션 열기/닫기 판단. (start, stop) 을 돌려준다."""
        self._expire_owner_locked(now)
        want = (not self.on_demand) or self.owner is not None or now < self._live_until or self.client.pending_estop
        start = want and not self.session_live
        stop = self.session_live and not want
        if start or stop:
            self.session_live = want
        if stop:
            self.arm_enabled = False
        return start, stop

    def _start_session(self, why):
        c = self.client
        with c.lock:
            if c.retired:
                c.new_session()
        self.log("조종 세션 시작(%s)" % why)

    def _loop(self):
        nxt = time.monotonic()
        while self.running:
            now = time.monotonic()
            with self.lock:
                start, stop = self._session_step_locked(now)
                live = self.session_live
                inp = dict(self.inp)
                fresh = now - self.last_input < INPUT_TIMEOUT
            c = self.client
            if start:
                self._start_session("화면이 조종권을 잡음")
            if stop:
                c.disconnect()
                self.log("조종하는 화면 없음 → 세션 종료(disconnect). 다른 조종기(VR 등)가 로봇을 쓸 수 있음")
            valid = live and fresh and inp["focused"]
            base_intent = valid and inp["held"]
            arm_intent = valid and self.arm_enabled
            with c.lock:
                if live:
                    if arm_intent and c.arm.armed and any(inp["arm"]):
                        lin, prate, grate = self.rates
                        d = inp["arm"]
                        c.move_arm_target(d[0] * lin * LOOP_PERIOD, d[1] * lin * LOOP_PERIOD, d[2] * lin * LOOP_PERIOD,
                                          d[3] * prate * LOOP_PERIOD, d[4] * grate * LOOP_PERIOD)
                    try:
                        c.tick(base_intent, inp["f"], inp["s"], inp["t"], arm_intent)
                    except OSError as exc:
                        c.last_error = str(exc)
                if self.arm_enabled and (c.arm.needs_release or c.retired):
                    # 비상정지·not_owner·fault·연결 끊기 같은 강한 차단 뒤에는 사용자가 다시 켜야 한다.
                    self.arm_enabled = False
                    self.log("팔 조종 꺼짐(%s) — 다시 켜야 함" % (c.arm.last_block_reason or "연결 끊김"))
            nxt += LOOP_PERIOD
            if nxt < time.monotonic():
                self.loop_overruns += 1
                nxt = time.monotonic() + LOOP_PERIOD
            while time.monotonic() < nxt:
                c.poll()
                time.sleep(0.002)

    # ── 명령 ──

    def arm_enable(self, on):
        c = self.client
        if on and not self.arm_enabled:
            st = self.rb.fresh_status() if self.rb else None
            cur = (st or {}).get("arm_current")
            if isinstance(cur, dict) and all(isinstance(cur.get(k), (int, float)) for k in ("x", "y", "z", "pitch")):
                g = cur.get("gripper", c.arm_target["gripper"])
                c.set_arm_target(cur["x"], cur["y"], cur["z"], cur["pitch"], g, accepted=True)
                self.log("팔 조종 켬: 목표를 로봇 현재값으로 맞춤 x=%.3f y=%.3f z=%.3f" % (cur["x"], cur["y"], cur["z"]))
            else:
                self.log("팔 조종 켬: 목표 유지(로봇 현재값 모름 — rosbridge 상태 없음)")
        elif not on and self.arm_enabled:
            self.log("팔 조종 끔")
        self.arm_enabled = bool(on)

    def command(self, req, sid="api", name="", addr=""):
        cmd = req.get("cmd")
        c, rb = self.client, self.rb
        if cmd not in ANY_SCREEN_CMDS and cmd not in OWNER_CMDS:
            return {"ok": False, "error": "알 수 없는 cmd"}
        if cmd in OWNER_CMDS:
            with self.lock:
                mine = self._claim_locked(sid, name, addr, True, True)
                o = self.owner
            if not mine:
                return {"ok": False, "control": False,
                        "error": "다른 기기가 조종 중입니다: %s" % (o.label() if o else "?")}
        if cmd == "take_control":
            with self.lock:
                self._claim_locked(sid, name, addr, True, True, force=True)
            return {"ok": True, "control": True}
        if cmd == "release_control":
            return {"ok": True, "released": self.release_control(sid)}
        if cmd == "estop":
            start = False
            with self.lock:
                if self.on_demand:
                    # 세션이 닫혀 있어도 비상정지는 보낸다: 새 UUID 로 세션을 열고(비소유자의 비상정지도 latch 됨)
                    # ACK 를 받을 때까지 재전송한 뒤 조종권이 없으면 다시 닫는다.
                    self._live_until = max(self._live_until, time.monotonic() + ESTOP_LINGER_S)
                    start = not self.session_live
                    self.session_live = True
            if start:
                self._start_session("비상정지 송신")
            c.emergency_stop()
            self.arm_enabled = False
        elif cmd == "clear":
            c.clear_emergency()
        elif cmd == "disconnect":
            self.arm_enabled = False
            if self.on_demand:
                self.release_control(sid)  # 세션은 루프가 닫는다(disconnect)
            else:
                c.disconnect()
        elif cmd == "connect":
            if self.on_demand:
                self.log("연결 요청: 조종권을 잡았으므로 세션을 연다")
            else:
                c.new_session()
                self.log("새 세션으로 연결 시도")
        elif cmd == "arm_enable":
            self.arm_enable(bool(req.get("on")))
        elif cmd == "arm_target":
            vals = {}
            for k in ("x", "y", "z", "pitch", "gripper"):
                if req.get(k) is None:
                    continue
                try:
                    v = float(req[k])
                except (TypeError, ValueError):
                    return {"ok": False, "error": "%s 숫자 아님" % k}
                if not math.isfinite(v):
                    return {"ok": False, "error": "%s 유한값 아님" % k}
                vals[k] = v
            t = c.set_arm_target(**vals)
            self.log("팔 목표 입력: x=%.3f y=%.3f z=%.3f pitch=%.3f gripper=%.2f"
                     % (t["x"], t["y"], t["z"], t["pitch"], t["gripper"]))
        elif cmd == "gripper":
            try:
                c.set_arm_target(gripper=float(req.get("value")))
            except (TypeError, ValueError):
                return {"ok": False, "error": "value 숫자 아님"}
        elif cmd == "arm_sync":
            st = rb.fresh_status() if rb else None
            cur = (st or {}).get("arm_current")
            if not isinstance(cur, dict):
                return {"ok": False, "error": "로봇 현재값 없음(rosbridge 상태 필요)"}
            c.set_arm_target(cur.get("x"), cur.get("y"), cur.get("z"), cur.get("pitch"), cur.get("gripper"),
                             accepted=True)
        elif cmd == "arm_home_target":
            c.set_arm_target(**self.home)
        elif cmd in ("robot_estop", "robot_reset", "arm_home"):
            if not rb:
                return {"ok": False, "error": "rosbridge 사용 안 함(--no-rosbridge)"}
            service = {"robot_estop": "/armpi/emergency_stop", "robot_reset": "/armpi/reset_estop",
                       "arm_home": "/armpi/arm/home"}[cmd]
            rb.call(service)
            self.log("서비스 호출 %s" % service)
            if cmd == "arm_home":
                c.set_arm_target(**self.home)
        elif cmd == "joint_debug":
            if not self.joint_debug_allowed():
                return {"ok": False, "error": "조종 세션이 있으면 관절 디버그 불가(먼저 '연결 끊기')"}
            pos = req.get("positions")
            if not isinstance(pos, list) or len(pos) != len(JOINT_NAMES):
                return {"ok": False, "error": "positions 6개 필요"}
            try:
                pos = [max(-JOINT_LIMIT_RAD, min(JOINT_LIMIT_RAD, float(v))) for v in pos]
            except (TypeError, ValueError):
                return {"ok": False, "error": "positions 숫자 아님"}
            if not all(math.isfinite(v) for v in pos):
                return {"ok": False, "error": "positions 유한값 아님"}
            now = time.time()
            rb.publish("/armpi/arm/joint_debug", {
                "header": {"stamp": {"sec": int(now), "nanosec": int((now % 1) * 1e9)}, "frame_id": ""},
                "name": JOINT_NAMES, "position": pos, "velocity": [], "effort": []})
        return {"ok": True}

    def joint_debug_allowed(self):
        if not self.rb or not self.rb.connected:
            return False
        st = self.rb.fresh_status()
        no_session = self.client.retired or (self.on_demand and not self.session_live)
        return bool(no_session and st is not None and st.get("owner") is None)

    def state(self, sid=None):
        control = self.control_snapshot(sid)  # 조종권 만료를 먼저 반영
        with self.lock:
            now = time.monotonic()
            inp = dict(self.inp)
            fresh = now - self.last_input < INPUT_TIMEOUT
            live = self.session_live
            reason = self._session_reason_locked(now)
        return {"udp": self.client.snapshot(), "arm_enabled": self.arm_enabled,
                "input": {"fresh": fresh, "focused": inp["focused"], "held": inp["held"]},
                "rosbridge": self.rb.snapshot() if self.rb else {"enabled": False, "connected": False},
                "joint_debug_allowed": self.joint_debug_allowed(), "loop_overruns": self.loop_overruns,
                "control": control,
                "session": {"mode": "on-demand" if self.on_demand else "always", "live": live, "reason": reason},
                "events": list(self.events)[-10:]}

    def quit(self):
        self.running = False
        self._thread.join(timeout=0.5)
        try:
            if self.session_live or not self.on_demand:
                self.client.quit()
        finally:
            self.client.close()


def _split_host(value):
    """Host 헤더 → (이름 소문자, 포트 또는 None). 형식이 틀리면 (None, None)."""
    h = (value or "").strip().lower()
    if not h:
        return None, None
    if h.startswith("["):  # [IPv6]:port
        end = h.find("]")
        if end < 0:
            return None, None
        name, rest = h[1:end], h[end + 1:]
        if rest and not rest.startswith(":"):
            return None, None
        port = rest[1:] if rest else None
    else:
        name, sep, port = h.rpartition(":")
        if not sep:
            name, port = h, None
    if port is not None and not port.isdigit():
        return None, None
    return name, (int(port) if port is not None else None)


def _is_ip_literal(name):
    try:
        ipaddress.ip_address(name)
        return True
    except ValueError:
        return False


def make_handler(ctx):
    http_port = ctx["http_port"]
    host_names = set(LOOPBACK_NAMES)
    if ctx.get("public"):
        # 0.0.0.0 등으로 열었을 때: IP 주소로 연 화면과 이 기기의 호스트 이름(armpi.local 등)으로 연 화면을 받는다.
        hn = socket.gethostname().lower()
        host_names |= {hn, hn.split(".")[0], hn.split(".")[0] + ".local"}
        host_names |= {h.lower() for h in ctx.get("extra_hosts") or ()}

    def host_ok(value):
        # DNS rebinding 방지: 이름이 IP 주소이거나 이 기기의 이름일 때만 받는다(공격자 도메인 이름은 거부).
        name, port = _split_host(value)
        if name is None or port not in (http_port, None if http_port == 80 else -1):
            return False
        return name in host_names or _is_ip_literal(name)

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _send(self, body, ctype, code=200, cache=False, extra=None):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "max-age=86400" if cache else "no-store")
            for k, v in (extra or {}).items():
                self.send_header(k, v)
            self.end_headers()
            if self.command != "HEAD":
                self.wfile.write(body)

        def _json(self, obj, code=200):
            body = json.dumps(obj, ensure_ascii=False, default=str).encode("utf-8")
            self._send(body, "application/json; charset=utf-8", code)

        def _host_ok(self):
            return host_ok(self.headers.get("Host", ""))

        def _screen(self, value):
            sid = str(value) if value is not None else "api"
            return sid if SCREEN_ID_RE.match(sid) else "api"

        def do_HEAD(self):
            self.do_GET()

        def do_GET(self):
            if not self._host_ok():
                return self._json({"ok": False, "error": "host 거부"}, 403)
            path, _, query = self.path.partition("?")
            if path == "/" and MOBILE_UA_RE.search(self.headers.get("User-Agent", "")):
                # 휴대폰 브라우저로 주소만 치고 들어오면 터치 화면으로 보낸다(PC 화면은 /index.html).
                return self._send(b"", "text/plain; charset=utf-8", 302, extra={"Location": "/mobile"})
            if path in STATIC_FILES:
                fname, ctype, cache = STATIC_FILES[path]
                try:
                    with open(os.path.join(WWW, fname), "rb") as fp:
                        body = fp.read()
                except OSError:
                    return self.send_error(404)
                return self._send(body, ctype, cache=cache)
            if path == "/api/config":
                return self._json(ctx["config"])
            if path == "/api/state":
                sid = self._screen((parse_qs(query).get("screen") or [None])[0])
                return self._json(ctx["ctl"].state(sid))
            self.send_error(404)

        def do_POST(self):
            # 다른 웹사이트가 이 조종기로 명령을 보내지 못하게(CSRF): JSON 본문 + 같은 출처(Origin = http://<Host>)만.
            origin = self.headers.get("Origin")
            ctype = (self.headers.get("Content-Type") or "").split(";")[0].strip().lower()
            same_origin = origin is None or origin.lower() == "http://" + self.headers.get("Host", "").strip().lower()
            if not self._host_ok() or not same_origin or ctype != "application/json":
                return self._json({"ok": False, "error": "요청 출처·형식 거부"}, 403)
            try:
                n = int(self.headers.get("Content-Length") or 0)
                if n > 8192:
                    raise ValueError("본문이 너무 큼")
                req = json.loads(self.rfile.read(n) or b"{}")
                if not isinstance(req, dict):
                    raise ValueError("객체 아님")
            except ValueError as exc:
                return self._json({"ok": False, "error": "bad json: %s" % exc}, 400)
            ctl = ctx["ctl"]
            sid, name, addr = self._screen(req.get("screen")), _screen_name(req.get("screen_name")), self.client_address[0]
            path = self.path.split("?", 1)[0]
            try:
                if path == "/api/input":
                    return self._json(ctl.set_input(req, sid, name, addr))
                if path == "/api/cmd":
                    res = ctl.command(req, sid, name, addr)
                    code = 200 if res.get("ok") else (409 if res.get("control") is False else 400)
                    return self._json(res, code)
                self.send_error(404)
            except OSError as exc:
                return self._json({"ok": False, "error": str(exc)}, 503)

    return Handler


class _HTTPServer(ThreadingHTTPServer):
    daemon_threads = True

    def server_bind(self):
        # HTTPServer.server_bind 의 getfqdn(역방향 DNS)을 건너뛴다: 오프라인 로봇에서 시작이 늦어지지 않게.
        socketserver.TCPServer.server_bind(self)
        host, port = self.server_address[:2]
        self.server_name, self.server_port = host, port


def _local_ipv4():
    """화면 안내용 이 기기의 주 IPv4(UDP connect 는 패킷을 보내지 않는다). 모르면 빈 목록."""
    out = []
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.connect(("192.0.2.1", 9))  # TEST-NET-1, 경로 선택만 함
            out.append(s.getsockname()[0])
        finally:
            s.close()
    except OSError:
        pass
    return [a for a in out if a and not a.startswith("0.")]


def _floats(text, n):
    vals = [float(v) for v in text.split(",")]
    if len(vals) != n or not all(math.isfinite(v) for v in vals):
        raise argparse.ArgumentTypeError("숫자 %d개를 쉼표로" % n)
    return vals


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--robot", default="auto", help="로봇 IP 또는 auto(UDP 검색)")
    ap.add_argument("--port", type=int, default=None, help="로봇 제어 UDP 포트(기본 5001, auto 면 검색 응답 값)")
    ap.add_argument("--discovery-port", type=int, default=DISCOVERY_PORT)
    ap.add_argument("--discover-target", action="append",
                    help="검색 요청 보낼 주소(여러 번 가능). 주면 이것만 쓴다. 기본 255.255.255.255")
    ap.add_argument("--discover-timeout", type=float, default=2.0)
    ap.add_argument("--http-port", type=int, default=18765)  # 이 Mac 의 8765 는 다른 프로그램이 사용 중
    ap.add_argument("--bind", default="127.0.0.1",
                    help="조종 화면 HTTP 를 열 주소. 기본 127.0.0.1(이 기기만). 0.0.0.0 = 같은 망의 휴대폰도 접속(인증 없음)")
    ap.add_argument("--allow-host", action="append", default=[],
                    help="--bind 가 공개 주소일 때 추가로 받을 Host 이름(예: armpi.local). IP 주소는 항상 받는다")
    ap.add_argument("--session", choices=["auto", "always", "on-demand"], default="auto",
                    help="v3 세션: always = 계속 잡음, on-demand = 조종하는 화면이 있을 때만. "
                         "auto = --bind 가 127.0.0.1 이면 always, 아니면 on-demand")
    ap.add_argument("--idle-release", type=float, default=60.0,
                    help="on-demand 에서 조종권을 가진 화면이 이 시간(s) 동안 조작이 없으면 조종권·세션을 놓는다(0 = 안 놓음)")
    ap.add_argument("--exit-with-stdin", action="store_true",
                    help="표준 입력이 닫히면(부모 앱이 끝나면) 중립·disconnect 를 보내고 끝낸다(맥 앱용)")
    ap.add_argument("--rosbridge-port", type=int, default=9090)
    ap.add_argument("--no-rosbridge", action="store_true", help="rosbridge 연결을 아예 시도하지 않음")
    ap.add_argument("--camera-port", type=int, default=8080)
    ap.add_argument("--camera-url", help="카메라 URL 을 직접 지정(기본 http://<로봇>:8080/stream?...)")
    ap.add_argument("--ack-timeout", type=float, default=ACK_TIMEOUT,
                    help="ACK 허용 지연(s). 기본 0.30(사양). 불안정한 Wi-Fi 에서만 늘릴 것")
    ap.add_argument("--home", type=lambda t: _floats(t, 5),
                    help="팔 시작 목표 x,y,z,pitch,gripper (기본 %s)" % ",".join(str(v) for v in DEFAULT_HOME.values()))
    ap.add_argument("--arm-speed", type=float, default=0.15, help="키를 누르는 동안 손끝 목표 이동 속도 m/s")
    ap.add_argument("--pitch-speed", type=float, default=0.6, help="pitch 목표 변화율 rad/s")
    ap.add_argument("--gripper-speed", type=float, default=0.8, help="그리퍼 목표 변화율 /s")
    ap.add_argument("--ui-default-speed", type=float, default=0.6,
                    help="휴대폰 화면 속도 버튼 기본값(0–1, 화면에 저장된 값이 없을 때)")
    ap.add_argument("--no-browser", action="store_true")
    args = ap.parse_args(argv)
    if not 0 < args.ui_default_speed <= 1:
        ap.error("--ui-default-speed 는 0 초과 1 이하")

    robot, port = args.robot, args.port
    if robot == "auto":
        targets = args.discover_target or ["255.255.255.255"]
        print("로봇 검색 중: %s UDP %d …" % (", ".join(targets), args.discovery_port), flush=True)
        found = discover(timeout=args.discover_timeout, targets=targets, port=args.discovery_port)
        if not found:
            print("로봇을 찾지 못했습니다. --robot <IP> 로 지정하십시오(같은 망, UDP 5002).", file=sys.stderr)
            return 1
        for f in found:
            print("  발견: %s (%s) 제어 포트 %d, id %s" % (f["address"], f["name"], f["port"], f["device_id"]))
        if len(found) > 1:
            print("  여러 대가 응답했습니다. 첫 번째를 씁니다. 다른 로봇은 --robot <IP> 로 지정.")
        robot = found[0]["address"]
        port = port or found[0]["port"]
    port = port or CONTROL_PORT
    home = dict(zip(("x", "y", "z", "pitch", "gripper"), args.home)) if args.home else dict(DEFAULT_HOME)
    video = args.camera_url or ("http://%s:%d/stream?topic=/usb_cam/image_raw&type=ros_compressed"
                                % (robot, args.camera_port))
    public = args.bind not in LOOPBACK_NAMES and not args.bind.startswith("127.")
    on_demand = args.session == "on-demand" or (args.session == "auto" and public)
    rb = None
    if not args.no_rosbridge:
        rb = RosBridge(robot, args.rosbridge_port, log=lambda m: _print("[%s] %s" % (_ts(), m)))
    ctl = Controller(robot, port, args.ack_timeout, home, (args.arm_speed, args.pitch_speed, args.gripper_speed), rb,
                     on_demand=on_demand, idle_release=args.idle_release)
    ctx = {"ctl": ctl, "http_port": args.http_port, "public": public, "extra_hosts": args.allow_host,
           "config": {"robot": robot, "port": port, "video_url": video, "home": home,
                      "rosbridge": None if args.no_rosbridge else "%s:%d" % (robot, args.rosbridge_port),
                      "ack_timeout_s": args.ack_timeout, "input_timeout_s": INPUT_TIMEOUT,
                      "rates": {"arm_m_s": args.arm_speed, "pitch_rad_s": args.pitch_speed,
                                "gripper_per_s": args.gripper_speed},
                      "joint_names": JOINT_NAMES, "joint_limit_rad": JOINT_LIMIT_RAD,
                      "session_mode": "on-demand" if on_demand else "always", "control_timeout_s": OWNER_TIMEOUT,
                      "idle_release_s": args.idle_release if on_demand else 0,
                      "ui": {"default_speed": args.ui_default_speed}}}
    try:
        server = _HTTPServer((args.bind, args.http_port), make_handler(ctx))
    except OSError as exc:
        print("HTTP %s:%d 열기 실패: %s" % (args.bind, args.http_port, exc), file=sys.stderr)
        ctl.quit()
        if rb:
            rb.close()
        return 1
    url = "http://127.0.0.1:%d/" % args.http_port
    print("ArmPi 조종기: 로봇 %s UDP %d, rosbridge %s, 세션 %s"
          % (robot, port, ctx["config"]["rosbridge"] or "사용 안 함", ctx["config"]["session_mode"]))
    print("조종 화면: %s   (종료: Ctrl+C — 중립·disconnect 를 보내고 끝냄)" % url, flush=True)
    if public:
        for ip in _local_ipv4():
            print("휴대폰 화면: http://%s:%d/mobile   (같은 망의 누구나 열 수 있음 — 인증 없음)" % (ip, args.http_port),
                  flush=True)
        print("HTTP 수신 주소: %s:%d" % (args.bind, args.http_port), flush=True)
    if not args.no_browser:
        webbrowser.open(url)

    def _term(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _term)
    signal.signal(signal.SIGINT, signal.default_int_handler)  # 부모(앱)가 SIGINT 를 무시한 채 띄워도 Ctrl+C·SIGINT 로 정리

    if args.exit_with_stdin:
        def _watch_stdin():
            try:
                fd = sys.stdin.fileno()
                while os.read(fd, 4096):
                    pass
            except (OSError, ValueError):
                pass
            if ctl.running:
                # 부모가 사라졌으면 표준 출력 파이프도 읽는 쪽이 없다: 이후 출력은 버리고(BrokenPipe 방지) 정리 시작.
                try:
                    sys.stdout = sys.stderr = open(os.devnull, "w")
                except OSError:
                    pass
                ctl.log("표준 입력 닫힘(부모 앱 종료) → 중립·disconnect 후 종료")
                os.kill(os.getpid(), signal.SIGINT)
        threading.Thread(target=_watch_stdin, name="stdin-watch", daemon=True).start()
    threading.Thread(target=server.serve_forever, name="http", daemon=True).start()
    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        server.server_close()
        ctl.quit()
        if rb:
            rb.close()
        _print("[%s] 조종기 종료(중립·disconnect 송신)" % _ts())
    return 0


if __name__ == "__main__":
    sys.exit(main())
