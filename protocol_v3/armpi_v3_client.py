#!/usr/bin/env python3
"""ArmPi Protocol v3 Python 클라이언트 라이브러리 (표준 라이브러리만 사용).

기준 문서: ../docs/01_PROTOCOL_V3_SPEC.md (구현 기준). 이전 프로젝트 Protocol v2 의 참조 클라이언트(VrClient,
Unity V2Session.cs 이식본)를 v3 로 확장했다. 바뀐 점:

- 채널 두 개(차체 ``command`` 8필드, 팔 ``arm_command`` 10필드)를 같은 소켓·같은 UUID·공용 sequence 로 보낸다.
- 채널마다 중립 확인·무장(armed)·ACK 시간 초과를 따로 관리한다. ACK 의 ``state`` 는 그 패킷이 속한 채널 상태다.
- 자동 재무장: 사용자가 계속 조종 의도(키를 누름·팔 조종 켬)를 유지하는 동안 통신 끊김·watchdog·lease 때문에
  풀린 채널은 "중립 송신 → 로봇의 중립 확인(ACK state=idle) → 활성"을 스스로 다시 밟는다.
  비상정지·해제·fault·not_owner·형식 오류처럼 '강한' 차단 뒤에는 의도를 한 번 내려야(키를 떼거나 팔 조종을 껐다 켜야)
  다시 무장한다.
- 팔 목표 거부(``workspace_limit``·``ik_unreachable``)는 채널을 풀지 않고, 목표를 마지막으로 수락된 값으로 되돌린다.
- ``emergency_stop`` 이벤트는 ACK 를 받을 때까지 50 ms 마다(매번 새 sequence) 재전송한다.

이 파일은 실제 VR 앱이 아니다. 여기서 통과한 시험은 실제 로봇·Quest 실기 통과가 아니다.

사용 예:
  python3 armpi_v3_client.py --discover --discover-target 127.0.0.1
  python3 armpi_v3_client.py --robot 127.0.0.1 --smoke
"""

import argparse
import json
import math
import re
import socket
import sys
import threading
import time
import uuid
from collections import OrderedDict, deque
from urllib.parse import unquote

PROTOCOL = 3
MAX_SEQUENCE = 9007199254740991  # 2**53 - 1
MAX_PACKET_BYTES = 2048
CONTROL_PORT = 5001
DISCOVERY_PORT = 5002
ACK_TIMEOUT = 0.30  # 사양 4절: 클라이언트 ACK timeout
SEND_PERIOD = 0.05  # 채널마다 20 Hz
ESTOP_RETRY_PERIOD = 0.05

STATES = ("awaiting_neutral", "idle", "active", "timeout", "emergency_stop", "fault")
REASONS = ("ok", "invalid_packet", "unsupported_protocol", "out_of_order", "not_owner", "emergency_stop",
           "neutral_required", "fault", "workspace_limit", "ik_unreachable")
ARM_TARGET_REJECTS = ("workspace_limit", "ik_unreachable")
EVENT_TYPES = ("emergency_stop", "clear_emergency", "disconnect")
ACK_KEYS = ("protocol", "type", "client_id", "sequence", "accepted", "reason", "emergency_stop", "state")

# 팔 기본 자세(설정으로 바꿀 수 있음). 링크 길이가 placeholder 인 지금은 IK 가 풀리는 보수적 위치를 골랐다.
DEFAULT_HOME = {"x": 0.1047, "y": 0.0, "z": 0.0753, "pitch": -1.0263, "gripper": 0.0}  # 기본 자세의 FK

_SAFE_STR = re.compile(r"\A[A-Za-z0-9_-]*\Z")  # Unity V2AckParser 와 같은 문자 집합
_HEX32 = re.compile(r"\A[0-9a-f]{32}\Z")
_NAME = re.compile(r"\A(?:[A-Za-z0-9_.~-]|%[0-9A-Fa-f]{2}){1,384}\Z")


# ───────────────────────── ACK 해석 ─────────────────────────

def _no_dup(pairs):
    out = {}
    for key, value in pairs:
        if key in out:
            raise ValueError("중복 키 %s" % key)
        out[key] = value
    return out


def _no_float(text):
    raise ValueError("ACK 에 실수 값 %s" % text)


def _no_const(text):
    raise ValueError("JSON 상수 %s" % text)


def parse_ack(data):
    """로봇 ACK 1개를 엄격하게 해석한다. 형식이 틀리면 None.

    정확히 8필드, 값은 문자열·불리언·정수만(실수 금지), 중복 키 금지, 문자열은 [A-Za-z0-9_-] 만.
    client_id 는 식별 불가 패킷의 ACK 이면 null 이다(그 경우에도 dict 를 돌려주되 client_id=None).
    """
    if not isinstance(data, (bytes, bytearray)) or not data or len(data) > MAX_PACKET_BYTES:
        return None
    try:
        obj = json.loads(bytes(data).decode("utf-8"), object_pairs_hook=_no_dup,
                         parse_float=_no_float, parse_constant=_no_const)
    except (UnicodeDecodeError, ValueError, RecursionError):
        return None
    if not isinstance(obj, dict) or set(obj) != set(ACK_KEYS):
        return None
    if type(obj["protocol"]) is not int or obj["protocol"] != PROTOCOL or obj["type"] != "ack":
        return None
    cid = obj["client_id"]
    if cid is not None and (type(cid) is not str or not _SAFE_STR.match(cid)):
        return None
    seq = obj["sequence"]
    if type(seq) is not int or seq < -1 or seq > MAX_SEQUENCE:
        return None
    if type(obj["accepted"]) is not bool or type(obj["emergency_stop"]) is not bool:
        return None
    if obj["reason"] not in REASONS or obj["state"] not in STATES:
        return None
    return obj


# ───────────────────────── 검색 ─────────────────────────

def discover(timeout=2.0, targets=("255.255.255.255",), port=DISCOVERY_PORT, probe_period=0.5):
    """UDP 검색. ``ARMPI_DISCOVER/1|<nonce>`` 를 targets 로 보내고 응답을 모은다.

    응답 조건(사양 1절): 출발 포트 = port, ``ARMPI_HERE/1|<nonce>|<32 hex id>|<URL 인코딩 이름>|<제어 포트>|3``.
    targets 에 브로드캐스트 주소를 넣으면 같은 망의 로봇에게 모두 간다(시험할 때는 127.0.0.1 만 쓰기).
    """
    nonce = uuid.uuid4().hex
    request = ("ARMPI_DISCOVER/1|" + nonce).encode("ascii")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    found = OrderedDict()
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.bind(("0.0.0.0", 0))
        sock.settimeout(0.05)
        end = time.monotonic() + timeout
        next_probe = 0.0
        while time.monotonic() < end:
            if time.monotonic() >= next_probe:
                next_probe = time.monotonic() + probe_period
                for target in targets:
                    try:
                        sock.sendto(request, (target, port))
                    except OSError:
                        pass
            try:
                packet, source = sock.recvfrom(1024)
            except socket.timeout:
                continue
            except OSError:
                continue
            info = parse_discovery_reply(packet, nonce)
            if info is None or source[1] != port:
                continue
            info["address"] = source[0]
            found[(info["device_id"], source[0])] = info
    finally:
        sock.close()
    return list(found.values())


def parse_discovery_reply(packet, nonce):
    """검색 응답 1개 해석. 틀리면 None."""
    if not (80 <= len(packet) <= 640) or any(b < 32 or b > 126 for b in packet):
        return None
    f = packet.decode("ascii").split("|")
    if (len(f) != 6 or f[0] != "ARMPI_HERE/1" or f[1] != nonce or not _HEX32.match(f[2])
            or not _NAME.match(f[3]) or f[5] != "3" or not f[4].isdigit()):
        return None
    control_port = int(f[4])
    if not 1 <= control_port <= 65535:
        return None
    return {"device_id": f[2], "name": unquote(f[3]), "port": control_port, "protocol": 3}


# ───────────────────────── 세션·채널 ─────────────────────────

class Channel:
    """채널 하나(차체 또는 팔)의 클라이언트 쪽 무장 상태."""

    def __init__(self, name):
        self.name = name
        self.intent = False          # 사용자가 지금 조종하려 함(키 누름·팔 조종 켬)
        self.armed = False           # 활성 패킷을 보내도 됨
        self.neutral_confirmed = False
        self.neutral_after = 0       # 이 번호보다 뒤에 보낸 중립만 확인으로 인정
        self.needs_release = False   # 강한 차단 뒤: 의도를 한 번 내려야 재무장
        self.server_state = "no_ack"
        self.server_reason = "no_ack"
        self.last_ack_time = None
        self.blocks = 0
        self.auto_rearms = 0
        self.last_block_reason = ""

    def snapshot(self):
        return {"state": self.server_state, "reason": self.server_reason, "armed": self.armed,
                "neutral_confirmed": self.neutral_confirmed, "needs_release": self.needs_release,
                "intent": self.intent, "blocks": self.blocks, "auto_rearms": self.auto_rearms,
                "last_block_reason": self.last_block_reason}


class _Sent:
    __slots__ = ("time", "kind", "channel", "neutral", "target")

    def __init__(self, t, kind, channel, neutral, target):
        self.time, self.kind, self.channel, self.neutral, self.target = t, kind, channel, neutral, target


def _clip(v, lo, hi):
    v = float(v)
    if not math.isfinite(v):
        return 0.0 if lo <= 0.0 <= hi else lo
    return max(lo, min(hi, v))


def _resolve(host):
    return socket.gethostbyname(host)


class ArmPiV3Client:
    """Protocol v3 송수신기. ``tick()`` 을 50 ms 마다 호출하고, 그 사이에 ``poll()`` 로 ACK 를 비운다.

    모든 공개 메서드는 내부 lock 으로 보호되므로 다른 스레드(예: HTTP 처리기)에서 ``emergency_stop()`` 을 바로 불러도 된다.
    """

    def __init__(self, host, port=CONTROL_PORT, ack_timeout=ACK_TIMEOUT, ack_max_age=None,
                 home=None, send_arm=True, clock=time.monotonic, log=None):
        self.host = host
        self.addr = (_resolve(host), int(port))
        self.ack_timeout = float(ack_timeout)
        self.ack_max_age = float(ack_max_age) if ack_max_age is not None else float(ack_timeout)
        self.home = dict(DEFAULT_HOME if home is None else home)
        self.send_arm = bool(send_arm)
        self.clock = clock
        self._log = log or (lambda msg: None)
        self.lock = threading.RLock()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # connect: 운영체제가 로봇 주소·포트에서 온 데이터그램만 넘겨준다(다른 출발지 ACK 무시, 사양 1절).
        self.sock.connect(self.addr)
        self.sock.setblocking(False)
        self.rtts = deque(maxlen=200)
        self.last_rtt = None
        self.packets_sent = 0
        self.acks_received = 0
        self.bad_acks = 0
        self.late_acks = 0
        self.null_acks = 0
        self.send_errors = 0
        self.link_drops = 0
        self.last_error = ""
        self.sessions_started = 0
        self.emergency = False
        self._new_session_locked(keep_emergency=False)

    # ── 세션 ──

    def _new_session_locked(self, keep_emergency=True):
        self.client_id = str(uuid.uuid4())
        self.sequence = 0
        self.last_ack_sequence = -1
        self.last_ack_time = None
        self.connected = False
        self.faulted = False
        self.retired = False
        if not keep_emergency:
            self.emergency = False
        self.pending_estop = False
        self._last_estop_send = -math.inf
        self._estop_acked_seq = -1
        self._sent = OrderedDict()
        self.base = Channel("base")
        self.arm = Channel("arm")
        self.arm_target = dict(self.home)
        self.arm_last_accepted = None
        self.arm_last_reject = None   # {"reason", "target", "time"}
        self.sessions_started += 1
        self._log("새 세션 %s" % self.client_id)

    def new_session(self):
        """새 UUID 로 다시 시작한다(disconnect·retire 뒤 '연결' 버튼). 비상정지 표시는 로봇 ACK 가 다시 알려 준다."""
        with self.lock:
            self._new_session_locked(keep_emergency=True)

    # ── 팔 목표 ──

    def set_arm_target(self, x=None, y=None, z=None, pitch=None, gripper=None, accepted=False):
        """팔 목표를 바꾼다(부분 지정 가능). pitch 는 −π..π, gripper 는 0..1 로 자른다(범위 밖이면 로봇이 invalid_packet).

        accepted=True 이면 '마지막 수락값'도 같이 바꾼다(로봇 현재값으로 맞출 때).
        """
        with self.lock:
            t = dict(self.arm_target)
            for key, value in (("x", x), ("y", y), ("z", z)):
                if value is not None and math.isfinite(float(value)):
                    t[key] = float(value)
            if pitch is not None:
                t["pitch"] = _clip(pitch, -math.pi, math.pi)
            if gripper is not None:
                t["gripper"] = _clip(gripper, 0.0, 1.0)
            self.arm_target = t
            if accepted:
                self.arm_last_accepted = dict(t)
            return dict(t)

    def move_arm_target(self, dx=0.0, dy=0.0, dz=0.0, dpitch=0.0, dgripper=0.0):
        with self.lock:
            t = self.arm_target
            return self.set_arm_target(t["x"] + dx, t["y"] + dy, t["z"] + dz, t["pitch"] + dpitch,
                                       t["gripper"] + dgripper)

    # ── 송신 ──

    def _next_seq(self):
        if self.sequence >= MAX_SEQUENCE:
            raise OverflowError("sequence 소진: 새 세션이 필요합니다")
        self.sequence += 1
        return self.sequence

    def _send_locked(self, payload, kind, channel, neutral, target=None):
        now = self.clock()
        seq = payload["sequence"]
        self._sent[seq] = _Sent(now, kind, channel, neutral, target)
        while len(self._sent) > 512:
            self._sent.popitem(last=False)
        data = json.dumps(payload, separators=(",", ":"), allow_nan=False).encode("utf-8")
        try:
            self.sock.send(data)
            self.packets_sent += 1
        except OSError as exc:
            self.send_errors += 1
            self.last_error = "송신 실패: %s" % exc

    def _send_command_locked(self, forward, strafe, turn):
        active = self.base.armed and self.base.intent and self._can_drive_locked()
        payload = OrderedDict([
            ("protocol", PROTOCOL), ("type", "command"), ("client_id", self.client_id),
            ("sequence", self._next_seq()), ("control_active", bool(active)),
            ("forward", _clip(forward, -1, 1) if active else 0.0),
            ("strafe", _clip(strafe, -1, 1) if active else 0.0),
            ("turn", _clip(turn, -1, 1) if active else 0.0),
        ])
        self._send_locked(payload, "command", "base", not active)

    def _send_arm_locked(self):
        active = self.arm.armed and self.arm.intent and self._can_drive_locked()
        # 중립 패킷에도 유효한 숫자가 필요하다. 마지막 수락값(없으면 현재 목표)을 담는다(로봇은 무시).
        src = self.arm_target if active else (self.arm_last_accepted or self.arm_target)
        t = {"x": float(src["x"]), "y": float(src["y"]), "z": float(src["z"]),
             "pitch": _clip(src["pitch"], -math.pi, math.pi), "gripper": _clip(src["gripper"], 0.0, 1.0)}
        payload = OrderedDict([
            ("protocol", PROTOCOL), ("type", "arm_command"), ("client_id", self.client_id),
            ("sequence", self._next_seq()), ("control_active", bool(active)),
            ("x", t["x"]), ("y", t["y"]), ("z", t["z"]), ("pitch", t["pitch"]), ("gripper", t["gripper"]),
        ])
        self._send_locked(payload, "arm_command", "arm", not active, t)

    def _send_event_locked(self, kind):
        payload = OrderedDict([("protocol", PROTOCOL), ("type", kind), ("client_id", self.client_id),
                               ("sequence", self._next_seq())])
        self._send_locked(payload, kind, "base", True)
        if kind == "emergency_stop":
            self._last_estop_send = self.clock()

    def _can_drive_locked(self):
        return self.connected and not self.emergency and not self.faulted and not self.retired

    # ── 차단·무장 ──

    def _block_locked(self, ch, hard, why):
        if ch.armed or ch.neutral_confirmed:
            ch.blocks += 1
        ch.armed = False
        ch.neutral_confirmed = False
        ch.neutral_after = self.sequence
        ch.last_block_reason = why
        if hard and ch.intent:
            ch.needs_release = True

    def _block_all_locked(self, hard, why):
        self._block_locked(self.base, hard, why)
        self._block_locked(self.arm, hard, why)

    def _update_intent_locked(self, ch, intent):
        if not intent:
            if ch.intent:
                # 놓기 = v2 Block(): 다시 무장하려면 놓은 뒤 보낸 중립이 확인돼야 한다.
                self._block_locked(ch, False, "의도 해제")
            ch.needs_release = False
            ch.armed = False
        elif (not ch.armed and ch.neutral_confirmed and not ch.needs_release and self._can_drive_locked()):
            ch.armed = True
            if ch.blocks:
                ch.auto_rearms += 1
        ch.intent = bool(intent)

    # ── 수신 ──

    def poll(self):
        with self.lock:
            self._poll_locked()

    def _poll_locked(self):
        while True:
            try:
                data = self.sock.recv(4096)
            except (BlockingIOError, InterruptedError):
                break
            except OSError as exc:  # ICMP port unreachable 등
                self.last_error = "수신 오류: %s" % exc
                break
            now = self.clock()
            ack = parse_ack(data)
            if ack is None:
                self.bad_acks += 1
                continue
            self._on_ack_locked(ack, now)

    def _on_ack_locked(self, ack, now):
        if ack["client_id"] is None:
            self.null_acks += 1  # 식별 불가 패킷의 ACK(우리 패킷은 늘 식별 가능해야 함)
            return
        if ack["client_id"].lower() != self.client_id:
            return
        seq = ack["sequence"]
        info = self._sent.get(seq)
        if info is None or seq <= self.last_ack_sequence:
            return
        age = now - info.time
        if age < 0 or age > self.ack_max_age:
            self.late_acks += 1
            return
        self.acks_received += 1
        self.last_ack_sequence = seq
        self.last_ack_time = now
        self.connected = True
        self.last_rtt = age
        self.rtts.append(age)
        ch = self.arm if info.channel == "arm" else self.base
        ch.server_state = ack["state"]
        ch.server_reason = ack["reason"]
        ch.last_ack_time = now

        if ack["reason"] == "fault" or ack["state"] == "fault":
            if not self.faulted:
                self._log("로봇 fault 보고")
            self.faulted = True
            self._block_all_locked(True, "fault")

        robot_estop = ack["emergency_stop"] or ack["state"] == "emergency_stop"
        if robot_estop:
            if info.kind == "emergency_stop" or self.pending_estop:
                if self.pending_estop:
                    self._log("비상정지 ACK 수신(seq %d)" % seq)
                self.pending_estop = False
                self._estop_acked_seq = max(self._estop_acked_seq, seq)
            if not self.emergency:
                self._log("로봇 비상정지 상태")
            self.emergency = True
            self._block_all_locked(True, "비상정지")
        elif self.emergency and not self.pending_estop and seq > self._estop_acked_seq:
            # 로봇이 latch 가 풀렸다고 보고(우리 clear 또는 다른 경로의 해제). 무장은 의도를 새로 내야 한다.
            self.emergency = False
            self._log("비상정지 해제 확인")
            self._block_all_locked(True, "비상정지 해제")

        if info.kind in EVENT_TYPES:
            if info.kind == "disconnect" and ack["accepted"]:
                self.retired = True
            if info.kind == "clear_emergency" and ack["accepted"]:
                self._block_all_locked(True, "clear_emergency")
            return

        if not ack["accepted"]:
            reason = ack["reason"]
            if ch is self.arm and reason in ARM_TARGET_REJECTS:
                self.arm_last_reject = {"reason": reason, "target": info.target, "time": now, "sequence": seq}
                self.arm_target = dict(self.arm_last_accepted) if self.arm_last_accepted else dict(self.home)
                return
            if reason == "neutral_required" and ack["state"] in ("timeout", "awaiting_neutral"):
                self._block_locked(ch, False, "로봇 %s" % ack["state"])
            elif reason == "not_owner":
                self._block_all_locked(True, "not_owner")
            else:
                self._block_locked(ch, True, reason)
            return

        if ack["state"] in ("timeout", "awaiting_neutral"):
            self._block_locked(ch, False, "로봇 %s" % ack["state"])
        elif info.neutral and seq > ch.neutral_after and ack["state"] == "idle":
            ch.neutral_confirmed = True
        if ch is self.arm and not info.neutral:
            self.arm_last_accepted = dict(info.target)

    def _check_link_locked(self, now):
        if self.connected and self.last_ack_time is not None and now - self.last_ack_time > self.ack_timeout:
            self.connected = False
            self.link_drops += 1
            self._block_all_locked(False, "ACK %.0f ms 없음" % (self.ack_timeout * 1e3))
        for ch in (self.base, self.arm):
            if ch.armed and ch.last_ack_time is not None and now - ch.last_ack_time > self.ack_timeout:
                self._block_locked(ch, False, "채널 ACK 없음")

    # ── 주기 처리 ──

    def tick(self, base_intent=False, forward=0.0, strafe=0.0, turn=0.0, arm_intent=False):
        """50 ms 마다 1회: ACK 비우기 → 링크 검사 → 의도 반영 → (비상정지 재전송) → 차체·팔 패킷 1개씩."""
        with self.lock:
            self._poll_locked()
            now = self.clock()
            self._check_link_locked(now)
            if self.pending_estop and now - self._last_estop_send >= ESTOP_RETRY_PERIOD - 0.005:
                self._send_event_locked("emergency_stop")
            if self.retired:
                return
            self._update_intent_locked(self.base, bool(base_intent))
            self._update_intent_locked(self.arm, bool(arm_intent) and self.send_arm)
            self._send_command_locked(forward, strafe, turn)
            if self.send_arm:
                self._send_arm_locked()

    # ── 이벤트 ──

    def emergency_stop(self):
        """즉시 송신하고, ACK 를 받을 때까지 tick() 마다(50 ms) 새 sequence 로 재전송한다."""
        with self.lock:
            self.emergency = True
            self.pending_estop = True
            self._block_all_locked(True, "비상정지 요청")
            self._send_event_locked("emergency_stop")
            self._log("비상정지 송신")

    def clear_emergency(self):
        with self.lock:
            if self.retired:
                return False
            self._send_event_locked("clear_emergency")
            self._log("비상정지 해제 요청 송신")
            return True

    def disconnect(self):
        """중립 2개(차체·팔) + disconnect. 이 UUID 는 로봇 프로세스가 도는 동안 다시 쓸 수 없다."""
        with self.lock:
            if self.retired:
                return
            self._update_intent_locked(self.base, False)
            self._update_intent_locked(self.arm, False)
            self._send_command_locked(0.0, 0.0, 0.0)
            if self.send_arm:
                self._send_arm_locked()
            self._send_event_locked("disconnect")
            self.retired = True
            self._log("disconnect 송신")

    def quit(self, wait=0.05):
        self.disconnect()
        time.sleep(wait)
        self.poll()

    def close(self):
        with self.lock:
            if self.sock is not None:
                self.sock.close()
                self.sock = None

    # ── 조회 ──

    def snapshot(self):
        with self.lock:
            r = sorted(self.rtts)
            now = self.clock()
            rej = None
            if self.arm_last_reject:
                rej = dict(self.arm_last_reject)
                rej["age_s"] = round(now - rej.pop("time"), 3)
            return {
                "robot": "%s:%d" % self.addr, "client_id": self.client_id, "sequence": self.sequence,
                "connected": self.connected, "emergency": self.emergency, "estop_pending": self.pending_estop,
                "faulted": self.faulted, "retired": self.retired,
                "base": self.base.snapshot(), "arm": self.arm.snapshot(),
                "arm_target": dict(self.arm_target),
                "arm_last_accepted": dict(self.arm_last_accepted) if self.arm_last_accepted else None,
                "arm_last_reject": rej,
                "ack_ms_last": round(self.last_rtt * 1e3, 2) if self.last_rtt is not None else None,
                "ack_ms_p50": round(r[len(r) // 2] * 1e3, 2) if r else None,
                "ack_ms_max": round(r[-1] * 1e3, 2) if r else None,
                "packets_sent": self.packets_sent, "acks": self.acks_received, "bad_acks": self.bad_acks,
                "late_acks": self.late_acks, "null_acks": self.null_acks, "send_errors": self.send_errors,
                "link_drops": self.link_drops, "last_error": self.last_error,
                "ack_timeout_s": self.ack_timeout,
            }


# ───────────────────────── 명령행(간단 확인용) ─────────────────────────

def _run_for(client, seconds, **intent):
    end = time.monotonic() + seconds
    nxt = time.monotonic()
    while time.monotonic() < end:
        if time.monotonic() >= nxt:
            client.tick(**intent)
            nxt += SEND_PERIOD
        client.poll()
        time.sleep(0.002)


def smoke(host, port, ack_timeout):
    """중립 0.5 s → 차체 전진 0.3 1 s → 놓기 → 팔 +x 이동 1 s → 종료. 결과 dict."""
    lines = []
    c = ArmPiV3Client(host, port, ack_timeout=ack_timeout, log=lambda m: lines.append(m))
    try:
        _run_for(c, 0.5)
        s0 = c.snapshot()
        _run_for(c, 1.0, base_intent=True, forward=0.3)
        s1 = c.snapshot()
        _run_for(c, 0.3)
        start_x = c.arm_target["x"]
        end = time.monotonic() + 1.0
        while time.monotonic() < end:
            if c.arm.armed:
                c.move_arm_target(dx=0.02 * SEND_PERIOD)
            c.tick(arm_intent=True)
            t0 = time.monotonic()
            while time.monotonic() - t0 < SEND_PERIOD:
                c.poll()
                time.sleep(0.002)
        s2 = c.snapshot()
        c.quit()
        ok = (s0["base"]["state"] == "idle" and s1["base"]["state"] == "active" and s2["arm"]["state"] == "active"
              and c.arm_target["x"] > start_x)
        return ok, {"after_neutral": s0["base"], "while_driving": s1["base"], "arm": s2["arm"],
                    "arm_target": s2["arm_target"], "ack_ms_p50": s2["ack_ms_p50"], "log": lines}
    finally:
        c.close()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--discover", action="store_true", help="검색만 하고 결과 출력")
    ap.add_argument("--discover-target", action="append",
                    help="검색 요청을 보낼 주소(여러 번 가능). 기본 255.255.255.255")
    ap.add_argument("--discovery-port", type=int, default=DISCOVERY_PORT)
    ap.add_argument("--robot", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=CONTROL_PORT)
    ap.add_argument("--ack-timeout", type=float, default=ACK_TIMEOUT)
    ap.add_argument("--smoke", action="store_true", help="짧은 송수신 확인 시나리오 실행")
    args = ap.parse_args(argv)
    if args.discover:
        targets = args.discover_target or ["255.255.255.255"]
        found = discover(targets=targets, port=args.discovery_port)
        print(json.dumps(found, ensure_ascii=False, indent=2))
        return 0 if found else 1
    if args.smoke:
        ok, detail = smoke(args.robot, args.port, args.ack_timeout)
        print(json.dumps(detail, ensure_ascii=False, indent=2))
        print("결과: %s" % ("PASS" if ok else "FAIL"))
        return 0 if ok else 1
    ap.print_help()
    return 0


if __name__ == "__main__":
    sys.exit(main())
