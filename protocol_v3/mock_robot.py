#!/usr/bin/env python3
"""ArmPi Protocol v3 참조 가짜 로봇 (표준 라이브러리만 사용, Mac 에서 실제 로봇 없이 시험용).

구현 기준: ../docs/01_PROTOCOL_V3_SPEC.md, ../docs/02_ROS2_INTERFACES.md.
상태기계는 이전 프로젝트 Protocol v2 의 세션 규칙(V2Session)을 바탕으로 하며, v3 변경(채널 2개, strafe,
arm_command, workspace_limit/ik_unreachable, 채널별 watchdog, 세션 1개)을 더했다.
팔 IK 판정은 ``ros2_ws/src/armpi_kinematics/src/arm_kinematics.cpp`` 를 같은 식으로 옮겼다.

제공하는 것
- UDP 제어 수신(기본 5001): 엄격한 파서, 소유권·sequence·lease·retire·비상정지 latch, 차체/팔 채널별 상태와 watchdog
- UDP 검색 응답(기본 5002): ``ARMPI_DISCOVER/1|nonce`` → ``ARMPI_HERE/1|nonce|id|name|port|3`` (초당 최대 20회)
- 간단한 시뮬레이션: 차체 속도·위치 적분, 팔 손끝 목표 보간(직선 속도·pitch 변화율·그리퍼 속도 제한)
- 가짜 rosbridge(WebSocket, 기본 9090): /armpi/status(2 Hz+변화), /armpi/battery(1 Hz), /joint_states(20 Hz) 발행,
  /armpi/emergency_stop·/armpi/reset_estop·/armpi/arm/home 서비스, /armpi/arm/joint_debug 구독
- 가짜 카메라(HTTP, 기본 8080): /stream → PNG 프레임 multipart 스트림(실제 ros_compressed JPEG 아님)

이 프로그램은 실제 로봇이 아니다. 여기서 통과한 시험은 실제 하드웨어·C++ 수신기 통과가 아니다.

  python3 mock_robot.py --bind 127.0.0.1
  python3 mock_robot.py --bind 127.0.0.1 --port 15001 --discovery-port 15002 --rosbridge-port 0 --camera-port 0
"""

import argparse
import base64
import hashlib
import ipaddress
import json
import math
import signal
import socket
import struct
import sys
import threading
import time
import uuid
import zlib
from collections import Counter, OrderedDict
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import quote

PROTOCOL = 3
MAX_PACKET_BYTES = 2048
MAX_SEQUENCE = 9007199254740991
NIL_UUID = "00000000-0000-0000-0000-000000000000"
COMMON_FIELDS = frozenset({"protocol", "type", "client_id", "sequence"})
COMMAND_FIELDS = COMMON_FIELDS | {"control_active", "forward", "strafe", "turn"}
ARM_FIELDS = COMMON_FIELDS | {"control_active", "x", "y", "z", "pitch", "gripper"}
EVENT_TYPES = frozenset({"emergency_stop", "clear_emergency", "disconnect"})
DISCOVER_PREFIX = b"ARMPI_DISCOVER/1|"

# 기본 설정. 사양 4·5절 기본값과 02 문서의 placeholder 치수. --config JSON 으로 덮어쓸 수 있다.
DEFAULT_CONFIG = {
    "name": "armpi-mock",
    "allowed_sources": [],            # CIDR 목록, 빈 목록 = 모두 허용
    "base_watchdog_s": 0.4,
    "arm_watchdog_s": 0.4,
    "lease_s": 1.0,
    "accept_estop_from_any_client": True,
    "retired_limit": 1024,
    "chassis": {"max_vx_m_s": 0.25, "max_vy_m_s": 0.25, "max_wz_rad_s": 1.2},  # 로봇 armpi.yaml 과 같게(사용자 결정 2026-11-04, 바퀴 방향 실기 시험 전)
    "arm": {
        "max_linear_speed_m_s": 0.15,
        "max_pitch_rate_rad_s": 1.0,
        "max_gripper_rate_per_s": 1.0,
        "workspace": {"x_min": 0.05, "x_max": 0.32, "y_min": -0.25, "y_max": 0.25, "z_min": -0.08, "z_max": 0.32},  # armpi.yaml 과 같은 값
        "kinematics": {
            # 링크 길이: Hiwonder 공개 ArmPi FPV 기구학 값(ArmPi Pro 실측 확인 필요).
            # 관절 기준·범위: 시험용(armpi_bringup config/test_hardware_profile.yaml 과 같음, 실제 로봇 값 아님).
            # 실제 로봇과 같은 판정이 필요하면 --hardware-profile <hardware_profile.yaml> 로 덮어쓴다.
            "base_height_m": 0.064605, "upper_arm_m": 0.10048, "forearm_m": 0.094714, "hand_m": 0.16331,
            "j1_sign": 1.0, "j1_offset": 0.0, "j2_sign": 1.0, "j2_offset": -1.4,
            "j3_sign": -1.0, "j3_offset": 0.0, "j4_sign": 1.0, "j4_offset": 0.1,
            "min_wrist_height_m": 0.0, "min_elbow_bend_rad": 0.05,
            "joint_min": [-2.0943951023931953, -1.4, -2.0943951023931953, -1.9],
            "joint_max": [2.0943951023931953, 1.4, 2.0943951023931953, 1.9],
        },
        "home": {"x": 0.1047, "y": 0.0, "z": 0.0753, "pitch": -1.0263, "gripper": 0.0},  # 기본 자세 손끝 목표(armpi.yaml 과 같음)
    },
    "battery_v": 7.6,
}


# ───────────────────────── 기구학 (arm_kinematics.cpp 이식) ─────────────────────────

class ArmKinematics:
    """C++ ``armpi_kinematics::ArmKinematics`` 와 같은 식. 팔꿈치 위(elbow-up) 해."""

    OK, OUT_OF_REACH, JOINT_LIMIT, INVALID = "ok", "out_of_reach", "joint_limit", "invalid"

    def __init__(self, p):
        self.p = dict(p)

    def valid(self):
        p = self.p
        if not (p["upper_arm_m"] > 0 and p["forearm_m"] > 0 and p["hand_m"] >= 0):
            return False
        for i in range(4):
            if not (p["joint_min"][i] < p["joint_max"][i]):
                return False
        return all(p["j%d_sign" % i] != 0 for i in range(1, 5))

    def inverse(self, x, y, z, pitch):
        """(status, [j1, j2, j3, j4] 또는 None)"""
        p = self.p
        if not self.valid() or not all(math.isfinite(v) for v in (x, y, z, pitch)):
            return self.INVALID, None
        r = math.hypot(x, y)
        yaw = 0.0 if r < 1e-9 else math.atan2(y, x)
        rw = r - p["hand_m"] * math.cos(pitch)
        zw = (z - p["base_height_m"]) - p["hand_m"] * math.sin(pitch)
        if zw + p["base_height_m"] < p.get("min_wrist_height_m", -1e9) - 1e-12:
            return self.OUT_OF_REACH, None
        l1, l2 = p["upper_arm_m"], p["forearm_m"]
        d2 = rw * rw + zw * zw
        c3 = (d2 - l1 * l1 - l2 * l2) / (2.0 * l1 * l2)
        if c3 > 1.0 + 1e-9 or c3 < -1.0 - 1e-9:
            return self.OUT_OF_REACH, None
        c3 = max(-1.0, min(1.0, c3))
        q3 = -math.acos(c3)
        if -q3 < p.get("min_elbow_bend_rad", 0.0):
            return self.OUT_OF_REACH, None
        q2 = math.atan2(zw, rw) - math.atan2(l2 * math.sin(q3), l1 + l2 * math.cos(q3))
        q4 = pitch - q2 - q3
        j = [p["j1_sign"] * yaw + p["j1_offset"], p["j2_sign"] * q2 + p["j2_offset"],
             p["j3_sign"] * q3 + p["j3_offset"], p["j4_sign"] * q4 + p["j4_offset"]]
        for i in range(4):
            if not math.isfinite(j[i]) or j[i] < p["joint_min"][i] - 1e-9 or j[i] > p["joint_max"][i] + 1e-9:
                return self.JOINT_LIMIT, j
        return self.OK, j

    def forward(self, j):
        p = self.p
        yaw = (j[0] - p["j1_offset"]) / p["j1_sign"]
        q2 = (j[1] - p["j2_offset"]) / p["j2_sign"]
        q3 = (j[2] - p["j3_offset"]) / p["j3_sign"]
        q4 = (j[3] - p["j4_offset"]) / p["j4_sign"]
        a2, a3, a4 = q2, q2 + q3, q2 + q3 + q4
        r = p["upper_arm_m"] * math.cos(a2) + p["forearm_m"] * math.cos(a3) + p["hand_m"] * math.cos(a4)
        z = (p["base_height_m"] + p["upper_arm_m"] * math.sin(a2) + p["forearm_m"] * math.sin(a3)
             + p["hand_m"] * math.sin(a4))
        return {"x": r * math.cos(yaw), "y": r * math.sin(yaw), "z": z, "pitch": a4}


def inside_workspace(w, x, y, z):
    """C++ ``inside()`` 와 같이 경계 포함."""
    return w["x_min"] <= x <= w["x_max"] and w["y_min"] <= y <= w["y_max"] and w["z_min"] <= z <= w["z_max"]


# ───────────────────────── 엄격한 파서 ─────────────────────────

class PacketError(ValueError):
    def __init__(self, reason, message, client_id=None, sequence=-1, channel="base"):
        super().__init__(message)
        self.reason, self.client_id, self.sequence, self.channel = reason, client_id, sequence, channel


class Packet:
    __slots__ = ("type", "client_id", "sid", "sequence", "control_active", "forward", "strafe", "turn",
                 "x", "y", "z", "pitch", "gripper")

    def __init__(self, **kw):
        for k in self.__slots__:
            setattr(self, k, kw.get(k))

    @property
    def channel(self):
        return "arm" if self.type == "arm_command" else "base"

    @property
    def is_neutral(self):
        if self.type == "command":
            return (not self.control_active) or (self.forward == 0.0 and self.strafe == 0.0 and self.turn == 0.0)
        if self.type == "arm_command":
            return not self.control_active
        return False


def _reject_const(value):
    raise ValueError("JSON 상수 %s 금지" % value)


def _reject_dup(pairs):
    out = {}
    for k, v in pairs:
        if k in out:
            raise ValueError("중복 키 %s" % k)
        out[k] = v
    return out


def _canonical_uuid(value):
    if type(value) is not str or len(value) != 36:
        return None
    try:
        text = str(uuid.UUID(value))
    except ValueError:
        return None
    if text != value.lower() or text == NIL_UUID:
        return None
    return text


def _is_seq(v):
    return type(v) is int and 0 <= v <= MAX_SEQUENCE


def _number(v):
    """bool 이 아닌 JSON 숫자 → 유한 float, 아니면 None."""
    if type(v) is int:
        try:
            f = float(v)
        except OverflowError:
            return None
        return f if math.isfinite(f) else None
    if type(v) is float and math.isfinite(v):
        return v
    return None


def _ranged(v, lo, hi):
    f = _number(v)
    if f is None or f < lo or f > hi:
        return None
    return f


def parse_packet(data):
    if not data or len(data) > MAX_PACKET_BYTES:
        raise PacketError("invalid_packet", "패킷 크기 %d byte" % len(data or b""))
    try:
        payload = json.loads(data.decode("utf-8"), parse_constant=_reject_const, object_pairs_hook=_reject_dup)
    except (UnicodeDecodeError, ValueError, RecursionError) as exc:
        raise PacketError("invalid_packet", "JSON 해석 실패: %s" % exc)
    if not isinstance(payload, dict):
        raise PacketError("invalid_packet", "JSON 객체가 아님")

    sid = _canonical_uuid(payload.get("client_id"))
    echo_id = payload["client_id"] if sid is not None else None
    raw_seq = payload.get("sequence")
    echo_seq = raw_seq if _is_seq(raw_seq) else -1
    channel = "arm" if payload.get("type") == "arm_command" else "base"

    def reject(reason, message):
        return PacketError(reason, message, echo_id, echo_seq, channel)

    protocol = payload.get("protocol")
    if type(protocol) is not int:
        raise reject("invalid_packet", "protocol 은 정수여야 함")
    if protocol != PROTOCOL:
        raise reject("unsupported_protocol", "protocol %d 미지원" % protocol)
    kind = payload.get("type")
    if type(kind) is not str:
        raise reject("invalid_packet", "type 은 문자열이어야 함")
    if kind == "command":
        required = COMMAND_FIELDS
    elif kind == "arm_command":
        required = ARM_FIELDS
    elif kind in EVENT_TYPES:
        required = COMMON_FIELDS
    else:
        raise reject("invalid_packet", "알 수 없는 type")
    fields = set(payload)
    if fields != required:
        raise reject("invalid_packet", "필드 불일치: 누락=%s 추가=%s"
                     % (sorted(required - fields), sorted(fields - required)))
    if sid is None:
        raise reject("invalid_packet", "client_id 는 nil 이 아닌 36자 UUID 여야 함")
    if echo_seq == -1:
        raise reject("invalid_packet", "sequence 는 0~2^53-1 정수여야 함")
    if kind in EVENT_TYPES:
        return Packet(type=kind, client_id=echo_id, sid=sid, sequence=raw_seq)
    if type(payload["control_active"]) is not bool:
        raise reject("invalid_packet", "control_active 는 불리언이어야 함")
    if kind == "command":
        axes = [_ranged(payload[k], -1.0, 1.0) for k in ("forward", "strafe", "turn")]
        if any(a is None for a in axes):
            raise reject("invalid_packet", "forward/strafe/turn 은 -1~1 유한 숫자")
        return Packet(type=kind, client_id=echo_id, sid=sid, sequence=raw_seq,
                      control_active=payload["control_active"], forward=axes[0], strafe=axes[1], turn=axes[2])
    xyz = [_number(payload[k]) for k in ("x", "y", "z")]
    if any(v is None for v in xyz):
        raise reject("invalid_packet", "x/y/z 는 유한 숫자")
    pitch = _ranged(payload["pitch"], -math.pi, math.pi)
    gripper = _ranged(payload["gripper"], 0.0, 1.0)
    if pitch is None or gripper is None:
        raise reject("invalid_packet", "pitch 는 -π~π, gripper 는 0~1 유한 숫자")
    return Packet(type=kind, client_id=echo_id, sid=sid, sequence=raw_seq,
                  control_active=payload["control_active"], x=xyz[0], y=xyz[1], z=xyz[2], pitch=pitch, gripper=gripper)


# ───────────────────────── 시뮬레이션 ─────────────────────────

class BaseSim:
    def __init__(self):
        self.cmd = (0.0, 0.0, 0.0)  # vx, vy, wz
        self.odom = [0.0, 0.0, 0.0]

    def stop(self):
        self.cmd = (0.0, 0.0, 0.0)

    def step(self, dt):
        vx, vy, wz = self.cmd
        th = self.odom[2]
        self.odom[0] += (vx * math.cos(th) - vy * math.sin(th)) * dt
        self.odom[1] += (vx * math.sin(th) + vy * math.cos(th)) * dt
        self.odom[2] = math.atan2(math.sin(th + wz * dt), math.cos(th + wz * dt))


class ArmSim:
    """손끝 목표를 속도 제한으로 따라간다(직선 보간). 정지·유지 = 목표를 현재값으로."""

    def __init__(self, home, lin, pitch_rate, grip_rate):
        self.current = dict(home)
        self.target = dict(home)
        self.lin, self.pitch_rate, self.grip_rate = lin, pitch_rate, grip_rate

    def hold(self):
        self.target = dict(self.current)

    def step(self, dt):
        c, t = self.current, self.target
        d = [t["x"] - c["x"], t["y"] - c["y"], t["z"] - c["z"]]
        dist = math.sqrt(sum(v * v for v in d))
        lim = self.lin * dt
        if dist <= lim:
            c["x"], c["y"], c["z"] = t["x"], t["y"], t["z"]
        else:
            for k, v in zip(("x", "y", "z"), d):
                c[k] += v * lim / dist
        for k, rate in (("pitch", self.pitch_rate), ("gripper", self.grip_rate)):
            delta = t[k] - c[k]
            step = rate * dt
            c[k] = t[k] if abs(delta) <= step else c[k] + math.copysign(step, delta)

    def moving(self):
        return any(abs(self.target[k] - self.current[k]) > 1e-6 for k in self.target)


# ───────────────────────── v3 세션 (상태기계) ─────────────────────────

class Channel:
    def __init__(self, name):
        self.name = name
        self.neutral_confirmed = False
        self.timed_out = False
        self.active = False
        self.last_rx = None

    def reset(self):
        self.neutral_confirmed = False
        self.timed_out = False
        self.active = False
        self.last_rx = None


class V3Session:
    """세션 1개(소유권·sequence·비상정지·retire) + 채널 2개(차체·팔, arming·watchdog).

    모든 메서드는 호출자가 lock 을 잡고 monotonic 시각을 넘긴다. 출력(차체 속도·팔 목표)은 sim 객체에 바로 쓴다.
    """

    def __init__(self, cfg, kin, base_sim, arm_sim, log):
        self.cfg = cfg
        self.kin = kin
        self.base_sim = base_sim
        self.arm_sim = arm_sim
        self.log = log
        self.base = Channel("base")
        self.arm = Channel("arm")
        self.owner = None
        self.owner_addr = None
        self.last_sequence = -1
        self.last_owner_activity = 0.0
        self.previous_owner = None  # (sid, last_sequence)
        self.retired = OrderedDict()
        self.estop_sources = set()   # "udp", "service" — 제어기(armpi_control) 흉내
        self.fault = None
        self.control_state = "ok"    # ok | missing | udp_blocked | fault (제어기 상태 게이트 흉내)
        self.last_reason = "ok"
        self.detail = ""
        self.counters = Counter()

    # ── 조회 ──

    @property
    def emergency(self):
        return bool(self.estop_sources)

    def faulted(self):
        return self.fault is not None or self.control_state == "fault"

    def state_of(self, ch):
        if self.faulted():
            return "fault"
        if self.emergency:
            return "emergency_stop"
        if ch.timed_out:
            return "timeout"
        if not ch.neutral_confirmed:
            return "awaiting_neutral"
        return "active" if ch.active else "idle"

    def gate_reason(self):
        """제어기 게이트(02 문서 '수신기 규칙'). None = 허용."""
        if self.control_state == "missing":
            return "armpi_control 상태 미수신"
        if self.control_state == "udp_blocked":
            return "udp_allowed:false"
        return None

    # ── 출력 ──

    def _stop_base(self):
        self.base_sim.stop()

    def _hold_arm(self):
        self.arm_sim.hold()  # 현재 보간 위치에서 정지, 토크 유지(가짜)

    def _reset_base(self):
        self.base.reset()
        self._stop_base()

    def _reset_arm(self):
        self.arm.reset()
        self._hold_arm()

    def _reset_all(self):
        self._reset_base()
        self._reset_arm()

    # ── 외부(제어기) 입력 ──

    def set_estop_source(self, source, latched):
        before = self.emergency
        if latched:
            self.estop_sources.add(source)
        else:
            self.estop_sources.discard(source)
        if before != self.emergency or latched:
            self._reset_all()
        return before != self.emergency

    def clear_all_estop(self):
        before = self.emergency
        self.estop_sources.clear()
        self._reset_all()
        return before

    # ── 시간 ──

    def check_timeouts(self, now):
        events = []
        for ch, wd in ((self.base, self.cfg["base_watchdog_s"]), (self.arm, self.cfg["arm_watchdog_s"])):
            if ch.last_rx is not None and now - ch.last_rx >= wd:
                if ch is self.base:
                    self._reset_base()
                else:
                    self._reset_arm()
                ch.timed_out = True
                events.append("%s watchdog %.3f s 만료" % ("차체" if ch is self.base else "팔", wd))
        if self.owner is not None and now - self.last_owner_activity >= self.cfg["lease_s"]:
            self.previous_owner = (self.owner, self.last_sequence)
            self._release_owner()
            events.append("lease %.3f s 만료, 소유권 해제" % self.cfg["lease_s"])
        return events

    # ── 패킷 ──

    def handle(self, data, addr, now):
        """(ack dict, 상태 변화 문자열 목록, 거부 상세). 거부 상세는 호출자가 사유별로 묶어 드물게 기록한다."""
        self.detail = ""
        events = self.check_timeouts(now)  # 패킷 적용 전에 두 채널 watchdog·lease 검사(사양 3절)
        try:
            pkt = parse_packet(data)
        except PacketError as exc:
            ch = self.arm if exc.channel == "arm" else self.base
            self.counters["rx_invalid"] += 1
            return self._ack(exc.client_id, exc.sequence, False, exc.reason, ch), events, str(exc)
        self.counters["rx_" + pkt.type] += 1
        ack, more = self._handle_packet(pkt, addr, now)
        return ack, events + more, self.detail

    def _handle_packet(self, pkt, addr, now):
        sid = pkt.sid
        is_owner = self.owner is not None and sid == self.owner and addr == self.owner_addr
        base = self.base

        # 안전 우선: 비소유 유효 클라이언트의 비상정지도 latch. 소유권·sequence 는 건드리지 않는다.
        if (pkt.type == "emergency_stop" and self.cfg["accept_estop_from_any_client"] and not is_owner
                and (self.owner is not None or sid in self.retired)):
            was = self.emergency
            self.set_estop_source("udp", True)
            return self._ack(pkt.client_id, pkt.sequence, True, "ok", base), \
                ([] if was else ["비상정지 latch (비소유자 %s)" % sid])

        if sid in self.retired:
            self.detail = "retire 된 세션 %s" % sid
            return self._ack(pkt.client_id, pkt.sequence, False, "not_owner", pkt), []

        claimed = []
        if self.owner is None:
            if pkt.type == "disconnect":
                self._retire(sid)
                if self.previous_owner is not None and self.previous_owner[0] == sid:
                    self.previous_owner = None
                return self._ack(pkt.client_id, pkt.sequence, True, "ok", base), ["소유자 없음 상태의 disconnect: %s retire" % sid]
            baseline = -1
            if self.previous_owner is not None and self.previous_owner[0] == sid:
                baseline = self.previous_owner[1]
            if pkt.sequence <= baseline:
                self.detail = "lease 만료 뒤 재획득 sequence 가 이전 번호 이하"
                return self._ack(pkt.client_id, pkt.sequence, False, "out_of_order", pkt), []
            if self.previous_owner is not None and self.previous_owner[0] != sid:
                self._retire(self.previous_owner[0])
                claimed.append("이전 조종자 %s retire" % self.previous_owner[0])
            self.previous_owner = None
            self.owner, self.owner_addr = sid, addr
            self.last_sequence = baseline
            self._reset_all()
            claimed.append("조종자 획득 %s %s:%d" % (sid, addr[0], addr[1]))
        elif not is_owner:
            self.detail = "다른 조종자 %s 소유" % self.owner
            return self._ack(pkt.client_id, pkt.sequence, False, "not_owner", pkt), []

        if pkt.sequence <= self.last_sequence:
            return self._ack(pkt.client_id, pkt.sequence, False, "out_of_order", pkt), claimed
        self.last_sequence = pkt.sequence
        self.last_owner_activity = now  # lease: 소유자의 순서가 맞는 패킷이면 갱신(v2 와 같음)
        ack, more = self._apply(pkt, now)
        return ack, claimed + more

    def _apply(self, pkt, now):
        base, arm = self.base, self.arm
        if pkt.type == "emergency_stop":
            was = self.emergency
            self.set_estop_source("udp", True)
            return self._ack(pkt.client_id, pkt.sequence, True, "ok", base), ([] if was else ["비상정지 latch"])
        if pkt.type == "clear_emergency":
            was = "udp" in self.estop_sources
            self.estop_sources.discard("udp")  # 소유자의 해제 요청: UDP latch 만 푼다(서비스 latch 는 reset_estop 으로)
            self._reset_all()                   # v2 와 같이 해제 뒤 두 채널 모두 새 중립 필요
            msg = []
            if was:
                msg.append("비상정지 해제(udp)" + ("; 다른 latch 남음: %s" % sorted(self.estop_sources)
                                                  if self.emergency else "; 두 채널 새 중립 대기"))
            return self._ack(pkt.client_id, pkt.sequence, True, "ok", base), msg
        if pkt.type == "disconnect":
            who = self.owner
            self._retire(who)
            self._release_owner()
            return self._ack(pkt.client_id, pkt.sequence, True, "ok", base), ["disconnect: %s retire, 소유권 해제" % who]

        ch = base if pkt.type == "command" else arm
        name = "차체" if ch is base else "팔"
        if pkt.is_neutral:
            msg = []
            if not self.faulted() and not self.emergency:
                if not ch.neutral_confirmed or ch.timed_out:
                    msg.append("%s 중립 확인" % name)
                ch.neutral_confirmed = True
                ch.timed_out = False
                ch.active = bool(pkt.control_active) if ch is base else False
                ch.last_rx = now
            if ch is base:
                self._stop_base()
            else:
                self._hold_arm()  # 팔 중립 = 새 목표 무시, 현재 위치 유지(ArmTarget active:false)
            return self._ack(pkt.client_id, pkt.sequence, True, "ok", ch), msg

        if self.faulted():
            return self._ack(pkt.client_id, pkt.sequence, False, "fault", ch), []
        if self.emergency:
            return self._ack(pkt.client_id, pkt.sequence, False, "emergency_stop", ch), []
        if not ch.neutral_confirmed:
            return self._ack(pkt.client_id, pkt.sequence, False, "neutral_required", ch), []
        gate = self.gate_reason()
        if gate is not None:
            # v2 receiver 와 같이: 다른 경로가 제어권 보유 → not_owner. 출력은 정지·유지.
            ch.active = False
            ch.last_rx = now
            if ch is base:
                self._stop_base()
            else:
                self._hold_arm()
            self.detail = "제어기 게이트: %s" % gate
            return self._ack(pkt.client_id, pkt.sequence, False, "not_owner", ch), []

        if ch is base:
            c = self.cfg["chassis"]
            was = ch.active
            ch.active = True
            ch.last_rx = now
            self.base_sim.cmd = (pkt.forward * c["max_vx_m_s"], pkt.strafe * c["max_vy_m_s"],
                                 pkt.turn * c["max_wz_rad_s"])
            return self._ack(pkt.client_id, pkt.sequence, True, "ok", ch), ([] if was else ["차체 활성"])

        # 팔: 작업 공간 → IK 순서로 판정. 거부는 목표·watchdog 을 바꾸지 않는다.
        ws = self.cfg["arm"]["workspace"]
        if not inside_workspace(ws, pkt.x, pkt.y, pkt.z):
            return self._ack(pkt.client_id, pkt.sequence, False, "workspace_limit", ch), []
        status, _ = self.kin.inverse(pkt.x, pkt.y, pkt.z, pkt.pitch)
        if status != ArmKinematics.OK:
            return self._ack(pkt.client_id, pkt.sequence, False, "ik_unreachable", ch), []
        was = ch.active
        ch.active = True
        ch.last_rx = now
        self.arm_sim.target = {"x": pkt.x, "y": pkt.y, "z": pkt.z, "pitch": pkt.pitch, "gripper": pkt.gripper}
        return self._ack(pkt.client_id, pkt.sequence, True, "ok", ch), ([] if was else ["팔 활성"])

    # ── 내부 ──

    def _release_owner(self):
        self.owner = None
        self.owner_addr = None
        self.last_sequence = -1
        self.last_owner_activity = 0.0
        self._reset_all()

    def _retire(self, sid):
        if sid is None:
            return
        self.retired[sid] = True
        self.retired.move_to_end(sid)
        while len(self.retired) > self.cfg["retired_limit"]:
            self.retired.popitem(last=False)

    def _ack(self, client_id, sequence, accepted, reason, ch_or_pkt):
        if isinstance(ch_or_pkt, Packet):
            ch = self.arm if ch_or_pkt.channel == "arm" else self.base
        else:
            ch = ch_or_pkt
        self.last_reason = reason
        self.counters["reason_" + reason] += 1
        return OrderedDict([
            ("protocol", PROTOCOL), ("type", "ack"), ("client_id", client_id), ("sequence", sequence),
            ("accepted", bool(accepted)), ("reason", reason), ("emergency_stop", self.emergency),
            ("state", self.state_of(ch)),
        ])


def encode_ack(ack):
    return json.dumps(ack, separators=(",", ":"), allow_nan=False).encode("utf-8")


# ───────────────────────── 최소 WebSocket (가짜 rosbridge) ─────────────────────────

WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"


class WsConn:
    def __init__(self, sock, addr):
        self.sock = sock
        self.addr = addr
        self.lock = threading.Lock()
        self.subs = set()
        self.alive = True

    def handshake(self):
        self.sock.settimeout(5.0)
        buf = b""
        while b"\r\n\r\n" not in buf:
            chunk = self.sock.recv(1024)
            if not chunk or len(buf) > 16384:
                raise OSError("handshake 실패")
            buf += chunk
        head = buf.split(b"\r\n\r\n", 1)[0].decode("latin-1").split("\r\n")
        headers = {}
        for line in head[1:]:
            if ":" in line:
                k, v = line.split(":", 1)
                headers[k.strip().lower()] = v.strip()
        key = headers.get("sec-websocket-key")
        if "websocket" not in headers.get("upgrade", "").lower() or not key:
            self.sock.sendall(b"HTTP/1.1 400 Bad Request\r\nContent-Length: 0\r\n\r\n")
            raise OSError("WebSocket 요청 아님")
        accept = base64.b64encode(hashlib.sha1((key + WS_GUID).encode()).digest()).decode()
        self.sock.sendall(("HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
                           "Sec-WebSocket-Accept: %s\r\n\r\n" % accept).encode())
        self.sock.settimeout(None)

    def _recv_exact(self, n):
        buf = b""
        while len(buf) < n:
            chunk = self.sock.recv(n - len(buf))
            if not chunk:
                raise OSError("연결 종료")
            buf += chunk
        return buf

    def recv_message(self):
        """(opcode, payload). 조각난 프레임은 이어 붙인다."""
        parts, first_op = [], None
        while True:
            b0, b1 = self._recv_exact(2)
            fin, op, length = b0 & 0x80, b0 & 0x0F, b1 & 0x7F
            if length == 126:
                length = struct.unpack("!H", self._recv_exact(2))[0]
            elif length == 127:
                length = struct.unpack("!Q", self._recv_exact(8))[0]
            if length > 1 << 20:
                raise OSError("프레임이 너무 큼")
            mask = self._recv_exact(4) if b1 & 0x80 else None
            data = self._recv_exact(length)
            if mask:
                data = bytes(b ^ mask[i % 4] for i, b in enumerate(data))
            if op >= 0x8:  # 제어 프레임은 조각 사이에도 올 수 있음
                return op, data
            if first_op is None:
                first_op = op
            parts.append(data)
            if fin:
                return first_op, b"".join(parts)

    def send_frame(self, data, op=0x1):
        n = len(data)
        hdr = bytearray([0x80 | op])
        if n < 126:
            hdr.append(n)
        elif n < 65536:
            hdr.append(126)
            hdr += struct.pack("!H", n)
        else:
            hdr.append(127)
            hdr += struct.pack("!Q", n)
        with self.lock:
            self.sock.sendall(bytes(hdr) + data)

    def send_json(self, obj):
        try:
            self.send_frame(json.dumps(obj, ensure_ascii=False, allow_nan=False).encode("utf-8"))
            return True
        except (OSError, ValueError):
            self.alive = False
            return False

    def close(self):
        self.alive = False
        try:
            self.sock.close()
        except OSError:
            pass


# ───────────────────────── 가짜 카메라 (PNG multipart) ─────────────────────────

def make_png(width, height, rows):
    def chunk(tag, data):
        return (struct.pack("!I", len(data)) + tag + data
                + struct.pack("!I", zlib.crc32(tag + data) & 0xFFFFFFFF))
    raw = b"".join(b"\x00" + bytes(r) for r in rows)
    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack("!IIBBBBB", width, height, 8, 2, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))


# ───────────────────────── 가짜 로봇 본체 ─────────────────────────

def _deep_update(dst, src):
    for k, v in src.items():
        if isinstance(v, dict) and isinstance(dst.get(k), dict):
            _deep_update(dst[k], v)
        else:
            dst[k] = v


def _ts():
    t = time.time()
    return time.strftime("%H:%M:%S", time.localtime(t)) + ".%03d" % int((t % 1) * 1000)


class MockRobot:
    def __init__(self, cfg, bind="0.0.0.0", port=5001, discovery_port=5002, rosbridge_port=9090,
                 camera_port=8080, stats_period=5.0, identity=None):
        self.cfg = cfg
        self.bind, self.port, self.discovery_port = bind, port, discovery_port
        self.rosbridge_port, self.camera_port = rosbridge_port, camera_port
        self.stats_period = stats_period
        self.identity = identity or hashlib.sha256(("armpi-mock:" + socket.gethostname()).encode()).hexdigest()[:32]
        self.lock = threading.RLock()
        self.running = threading.Event()
        self.kin = ArmKinematics(cfg["arm"]["kinematics"])
        if not self.kin.valid():
            raise ValueError("기구학 설정이 올바르지 않습니다(링크 길이·관절 한계)")
        a = cfg["arm"]
        self.base_sim = BaseSim()
        self.arm_sim = ArmSim(a["home"], a["max_linear_speed_m_s"], a["max_pitch_rate_rad_s"],
                              a["max_gripper_rate_per_s"])
        hs, _ = self.kin.inverse(a["home"]["x"], a["home"]["y"], a["home"]["z"], a["home"]["pitch"])
        if hs != ArmKinematics.OK or not inside_workspace(a["workspace"], a["home"]["x"], a["home"]["y"], a["home"]["z"]):
            raise ValueError("기본 자세(home)가 작업 공간 밖이거나 IK 해가 없습니다: %s" % hs)
        self.session = V3Session(cfg, self.kin, self.base_sim, self.arm_sim, self.log)
        self.session.control_state = cfg.get("control_state", "ok")
        self.allowed = [ipaddress.ip_network(c, strict=False) for c in cfg.get("allowed_sources", [])]
        self.dropped_sources = 0
        self.discovery_replies = 0
        self.joints = [0.0, 0.0, 0.0, 0.0]
        self.ws_conns = []
        self._socks = []
        self._threads = []
        self._http = None
        self._last_key = None
        self._status_dirty = True
        self._reject_log = {}
        self._last_cmd_log = 0.0
        self._last_arm_log = 0.0
        self._battery_v = float(cfg.get("battery_v", 7.6))
        self._packets = 0

    # ── 로그 ──

    def log(self, msg):
        sys.stdout.write("[%s] %s\n" % (_ts(), msg))
        sys.stdout.flush()

    # ── 수명 ──

    def start(self):
        self.running.set()
        ctl = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        ctl.bind((self.bind, self.port))  # SO_REUSEADDR 금지(두 서버가 같은 포트를 나눠 받지 않도록)
        ctl.settimeout(0.02)
        self.ctl = ctl
        self.port = ctl.getsockname()[1]
        self._socks.append(ctl)
        self._spawn(self._rx_loop, "v3-rx")
        if self.discovery_port:
            dsc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            dsc.bind((self.bind, self.discovery_port))
            dsc.settimeout(0.1)
            self.dsc = dsc
            self.discovery_port = dsc.getsockname()[1]
            self._socks.append(dsc)
            self._spawn(self._discovery_loop, "v3-discovery")
        self._spawn(self._sim_loop, "sim")
        if self.rosbridge_port:
            ws = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            ws.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            ws.bind((self.bind, self.rosbridge_port))
            ws.listen(8)
            ws.settimeout(0.2)
            self.ws_listen = ws
            self._socks.append(ws)
            self._spawn(self._ws_accept_loop, "rosbridge-accept")
            self._spawn(self._ws_publish_loop, "rosbridge-pub")
        if self.camera_port:
            self._http = ThreadingHTTPServer((self.bind, self.camera_port), self._camera_handler())
            self._http.daemon_threads = True
            self._spawn(self._http.serve_forever, "camera")
        w = self.cfg["arm"]["workspace"]
        k = self.cfg["arm"]["kinematics"]
        self.log("가짜 ArmPi 시작: 제어 UDP %s:%d, 검색 %s, rosbridge %s, 카메라 %s"
                 % (self.bind, self.port, self.discovery_port or "끔", self.rosbridge_port or "끔",
                    self.camera_port or "끔"))
        self.log("설정: watchdog 차체 %.2f s / 팔 %.2f s, lease %.2f s, 제어기 상태=%s, allowed_sources=%s"
                 % (self.cfg["base_watchdog_s"], self.cfg["arm_watchdog_s"], self.cfg["lease_s"],
                    self.session.control_state, self.cfg.get("allowed_sources") or "모두 허용"))
        self.log("팔: 링크 base %.3f upper %.3f forearm %.3f hand %.3f m (placeholder), 작업공간 x %.2f~%.2f y %.2f~%.2f z %.2f~%.2f"
                 % (k["base_height_m"], k["upper_arm_m"], k["forearm_m"], k["hand_m"],
                    w["x_min"], w["x_max"], w["y_min"], w["y_max"], w["z_min"], w["z_max"]))

    def _spawn(self, fn, name):
        t = threading.Thread(target=fn, name=name, daemon=True)
        t.start()
        self._threads.append(t)

    def shutdown(self):
        self.running.clear()
        if self._http is not None:
            self._http.shutdown()
            self._http.server_close()
        for c in list(self.ws_conns):
            c.close()
        for s in self._socks:
            try:
                s.close()
            except OSError:
                pass
        for t in self._threads:
            t.join(timeout=1.0)
        with self.lock:
            self.base_sim.stop()
            self.arm_sim.hold()
        self.log("가짜 ArmPi 종료 (수신 패킷 %d)" % self._packets)

    def _allowed(self, ip):
        if not self.allowed:
            return True
        try:
            a = ipaddress.ip_address(ip)
        except ValueError:
            return False
        return any(a in n for n in self.allowed)

    # ── 제어 수신 ──

    def _rx_loop(self):
        while self.running.is_set():
            try:
                data, addr = self.ctl.recvfrom(65535)
            except socket.timeout:
                self._check_timeouts()
                continue
            except OSError:
                if not self.running.is_set():
                    break
                time.sleep(0.01)
                continue
            if not self._allowed(addr[0]):
                self.dropped_sources += 1
                continue
            try:
                self._handle(data, addr)
            except Exception as exc:  # 한 패킷 오류가 수신 스레드를 끝내지 않게
                self.log("패킷 처리 오류: %r — 안전 정지" % exc)
                with self.lock:
                    self.session._reset_all()
            self._check_timeouts()

    def _handle(self, data, addr):
        with self.lock:
            now = time.monotonic()
            ack, events, detail = self.session.handle(data, addr, now)
            self._packets += 1
            vx, vy, wz = self.base_sim.cmd
            arm_t = dict(self.arm_sim.target)
        try:
            self.ctl.sendto(encode_ack(ack), addr)
        except OSError:
            pass
        for e in events:
            self.log(e)
        if not ack["accepted"]:
            self._log_reject(ack["reason"], addr, detail)
        elif ack["state"] == "active":
            now = time.monotonic()
            if now - self._last_cmd_log >= 0.5 and (vx or vy or wz):
                self._last_cmd_log = now
                self.log("차체 명령 수신: vx=%.3f m/s vy=%.3f m/s wz=%.3f rad/s (seq %d)" % (vx, vy, wz, ack["sequence"]))
            if now - self._last_arm_log >= 0.5 and self.session.arm.active:
                self._last_arm_log = now
                self.log("팔 목표: x=%.3f y=%.3f z=%.3f pitch=%.3f gripper=%.2f" %
                         (arm_t["x"], arm_t["y"], arm_t["z"], arm_t["pitch"], arm_t["gripper"]))
        self._note_change()

    def _log_reject(self, reason, addr, detail):
        """같은 사유는 1초에 한 줄로 묶는다(×N = 그동안의 건수)."""
        now = time.monotonic()
        last, count = self._reject_log.get(reason, (0.0, 0))
        count += 1
        if now - last >= 1.0:
            self.log("거부 %s ×%d (%s:%d) %s" % (reason, count, addr[0], addr[1], detail))
            self._reject_log[reason] = (now, 0)
        else:
            self._reject_log[reason] = (last, count)

    def _check_timeouts(self):
        with self.lock:
            events = self.session.check_timeouts(time.monotonic())
        for e in events:
            self.log(e + " → 차체 정지·팔 자세 유지")
        if events:
            self._note_change()

    def _note_change(self):
        with self.lock:
            s = self.session
            key = (s.owner, s.state_of(s.base), s.state_of(s.arm), s.emergency, tuple(sorted(s.estop_sources)))
        if key != self._last_key:
            if self._last_key is not None:
                self.log("상태: 소유자=%s 차체=%s 팔=%s 비상정지=%s%s" % (
                    key[0] or "없음", key[1], key[2], "ON" if key[3] else "off",
                    " (%s)" % ",".join(key[4]) if key[4] else ""))
            self._last_key = key
            self._status_dirty = True

    # ── 검색 ──

    def _discovery_loop(self):
        window, replies = time.monotonic(), 0
        name = quote(self.cfg["name"], safe="")
        while self.running.is_set():
            try:
                packet, peer = self.dsc.recvfrom(1024)
            except socket.timeout:
                continue
            except OSError:
                if not self.running.is_set():
                    break
                continue
            if not self._allowed(peer[0]):
                continue
            if len(packet) != len(DISCOVER_PREFIX) + 32 or not packet.startswith(DISCOVER_PREFIX):
                continue
            nonce = packet[len(DISCOVER_PREFIX):]
            if any(c not in b"0123456789abcdef" for c in nonce):
                continue
            now = time.monotonic()
            if now - window >= 1.0:
                window, replies = now, 0
            if replies >= 20:
                continue
            replies += 1
            reply = "ARMPI_HERE/1|%s|%s|%s|%d|3" % (nonce.decode("ascii"), self.identity, name, self.port)
            try:
                self.dsc.sendto(reply.encode("ascii"), peer)
                self.discovery_replies += 1
            except OSError:
                pass

    # ── 시뮬레이션 ──

    def _sim_loop(self):
        period = 0.02
        last = time.monotonic()
        last_stats = last
        while self.running.is_set():
            time.sleep(period)
            now = time.monotonic()
            dt = min(0.1, now - last)
            last = now
            self._check_timeouts()
            with self.lock:
                self.base_sim.step(dt)
                self.arm_sim.step(dt)
                c = self.arm_sim.current
                st, j = self.kin.inverse(c["x"], c["y"], c["z"], c["pitch"])
                if st == ArmKinematics.OK:
                    self.joints = j
                self._battery_v = max(6.6, self._battery_v - dt * 1e-4)
            if now - last_stats >= self.stats_period:
                last_stats = now
                self._log_stats()

    def _log_stats(self):
        with self.lock:
            s = self.session
            reasons = {k[7:]: v for k, v in s.counters.items() if k.startswith("reason_")}
            rx = {k[3:]: v for k, v in s.counters.items() if k.startswith("rx_")}
            vx, vy, wz = self.base_sim.cmd
            c = dict(self.arm_sim.current)
        if not self._packets:
            return
        self.log("통계: 수신 %d %s, 사유 %s | 차체 vx=%.3f vy=%.3f wz=%.3f | 팔 현재 x=%.3f y=%.3f z=%.3f p=%.3f g=%.2f"
                 % (self._packets, rx, reasons, vx, vy, wz, c["x"], c["y"], c["z"], c["pitch"], c["gripper"]))

    # ── 상태 (rosbridge /armpi/status) ──

    def status(self):
        with self.lock:
            s = self.session
            vx, vy, wz = self.base_sim.cmd
            return {
                "source": "mock_robot",
                "owner": s.owner,
                "owner_addr": "%s:%d" % s.owner_addr if s.owner_addr else None,
                "base_state": s.state_of(s.base),
                "arm_state": s.state_of(s.arm),
                "emergency_stop": s.emergency,
                "estop_sources": sorted(s.estop_sources),
                "fault": s.faulted(),
                "fault_reason": s.fault or ("control_state fault" if s.control_state == "fault" else ""),
                "udp_allowed": s.gate_reason() is None,
                "battery_v": round(self._battery_v, 3),
                "position_source": "commanded",
                "arm_target": {k: round(v, 4) for k, v in self.arm_sim.target.items()},
                "arm_current": {k: round(v, 4) for k, v in self.arm_sim.current.items()},
                "arm_moving": self.arm_sim.moving(),
                "base_cmd": {"vx": round(vx, 4), "vy": round(vy, 4), "wz": round(wz, 4)},
                "odom": {"x": round(self.base_sim.odom[0], 4), "y": round(self.base_sim.odom[1], 4),
                         "yaw": round(self.base_sim.odom[2], 4)},
                "last_sequence": s.last_sequence,
                "last_reason": s.last_reason,
                "packets": self._packets,
            }

    # ── 서비스 (제어기 흉내) ──

    def service(self, name, args):
        with self.lock:
            s = self.session
            if name == "/armpi/emergency_stop":
                s.set_estop_source("service", True)
                res = (True, "비상정지 latch(service)")
            elif name == "/armpi/reset_estop":
                was = s.clear_all_estop()
                res = (True, "비상정지 해제" if was else "이미 해제 상태")
            elif name == "/armpi/arm/home":
                if s.emergency:
                    res = (False, "비상정지 중에는 home 거부")
                elif s.faulted():
                    res = (False, "fault 상태")
                elif s.arm.active:
                    res = (False, "팔 조종 중(UDP 팔 채널 active)에는 home 거부")
                else:
                    self.arm_sim.target = dict(self.cfg["arm"]["home"])
                    res = (True, "기본 자세로 이동")
            else:
                return None
        self.log("서비스 %s → %s %s" % (name, "성공" if res[0] else "거부", res[1]))
        self._note_change()
        return res

    def joint_debug(self, msg):
        names = msg.get("name") or []
        pos = msg.get("position") or []
        with self.lock:
            if self.session.owner is not None:
                self.log("joint_debug 무시: 조종 세션 있음")
                return
            j = list(self.joints)
            for n, p in zip(names, pos):
                if n in ("joint1", "joint2", "joint3", "joint4") and isinstance(p, (int, float)) and math.isfinite(p):
                    j[int(n[-1]) - 1] = float(p)
            pose = self.kin.forward(j)
            gripper = self.arm_sim.target["gripper"]
            for n, p in zip(names, pos):
                if n == "r_joint" and isinstance(p, (int, float)) and math.isfinite(p):
                    gripper = max(0.0, min(1.0, float(p)))  # 가짜: r_joint 값을 그리퍼 0~1 로 그대로 사용
            pose["gripper"] = gripper
            self.arm_sim.target = pose
        self.log("joint_debug 적용: %s → 손끝 x=%.3f y=%.3f z=%.3f pitch=%.3f"
                 % (["%.2f" % v for v in j], pose["x"], pose["y"], pose["z"], pose["pitch"]))

    # ── rosbridge 서버 ──

    def _ws_accept_loop(self):
        while self.running.is_set():
            try:
                sock, addr = self.ws_listen.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            if not self._allowed(addr[0]):
                sock.close()
                continue
            conn = WsConn(sock, addr)
            threading.Thread(target=self._ws_serve, args=(conn,), name="rosbridge-conn", daemon=True).start()

    def _ws_serve(self, conn):
        try:
            conn.handshake()
        except OSError:
            conn.close()
            return
        self.ws_conns.append(conn)
        self.log("rosbridge 연결: %s:%d" % conn.addr)
        try:
            while self.running.is_set() and conn.alive:
                op, data = conn.recv_message()
                if op == 0x8:
                    break
                if op == 0x9:
                    conn.send_frame(data, 0xA)
                    continue
                if op != 0x1:
                    continue
                try:
                    msg = json.loads(data.decode("utf-8"))
                except ValueError:
                    continue
                self._ws_dispatch(conn, msg)
        except OSError:
            pass
        finally:
            conn.close()
            if conn in self.ws_conns:
                self.ws_conns.remove(conn)
            self.log("rosbridge 연결 끊김: %s:%d" % conn.addr)

    def _ws_dispatch(self, conn, msg):
        op = msg.get("op")
        if op == "subscribe":
            conn.subs.add(msg.get("topic"))
            if msg.get("topic") == "/armpi/status":
                self._status_dirty = True
        elif op == "unsubscribe":
            conn.subs.discard(msg.get("topic"))
        elif op == "advertise":
            pass
        elif op == "publish":
            if msg.get("topic") == "/armpi/arm/joint_debug" and isinstance(msg.get("msg"), dict):
                self.joint_debug(msg["msg"])
        elif op == "call_service":
            res = self.service(msg.get("service"), msg.get("args") or {})
            reply = {"op": "service_response", "service": msg.get("service"), "id": msg.get("id")}
            if res is None:
                reply.update(result=False, values="서비스 없음(가짜 rosbridge): %s" % msg.get("service"))
            else:
                reply.update(result=True, values={"success": res[0], "message": res[1]})
            conn.send_json(reply)

    def _ws_publish_loop(self):
        last_status = last_batt = 0.0
        while self.running.is_set():
            time.sleep(0.05)
            conns = [c for c in self.ws_conns if c.alive]
            if not conns:
                continue
            now = time.monotonic()
            sec = time.time()
            stamp = {"sec": int(sec), "nanosec": int((sec % 1) * 1e9)}
            if self._status_dirty or now - last_status >= 0.5:
                self._status_dirty = False
                last_status = now
                body = {"op": "publish", "topic": "/armpi/status",
                        "msg": {"data": json.dumps(self.status(), ensure_ascii=False)}}
                for c in conns:
                    if "/armpi/status" in c.subs:
                        c.send_json(body)
            if now - last_batt >= 1.0:
                last_batt = now
                v = round(self._battery_v, 3)
                body = {"op": "publish", "topic": "/armpi/battery",
                        "msg": {"header": {"stamp": stamp, "frame_id": ""}, "voltage": v, "current": 0.0,
                                "charge": 0.0, "capacity": 0.0, "design_capacity": 0.0,
                                "percentage": round(max(0.0, min(1.0, (v - 6.6) / (8.4 - 6.6))), 3),
                                "power_supply_status": 2, "power_supply_health": 1, "power_supply_technology": 2,
                                "present": True, "cell_voltage": [], "cell_temperature": [],
                                "location": "mock", "serial_number": ""}}
                for c in conns:
                    if "/armpi/battery" in c.subs:
                        c.send_json(body)
            if any("/joint_states" in c.subs for c in conns):
                with self.lock:
                    j = list(self.joints)
                    g = self.arm_sim.current["gripper"]
                body = {"op": "publish", "topic": "/joint_states",
                        "msg": {"header": {"stamp": stamp, "frame_id": "base_link"},
                                "name": ["joint1", "joint2", "joint3", "joint4", "joint5", "r_joint"],
                                "position": [round(v, 5) for v in j] + [0.0, round(g, 4)],
                                "velocity": [], "effort": []}}
                for c in conns:
                    if "/joint_states" in c.subs:
                        c.send_json(body)

    # ── 카메라 ──

    def _frame(self, t):
        w, h = 160, 120
        st = self.status()
        top = (200, 40, 40) if st["emergency_stop"] else ((40, 180, 90) if st["base_state"] == "active"
                                                          or st["arm_state"] == "active" else (90, 90, 100))
        bar = int((t * 40) % w)
        ac = st["arm_current"]
        ax = int(w / 2 - ac["y"] / 0.25 * (w / 2 - 6))
        ay = int(h - 8 - ac["x"] / 0.30 * (h - 30))
        rows = []
        for yy in range(h):
            row = bytearray()
            for xx in range(w):
                if yy < 10:
                    row += bytes(top)
                elif abs(xx - bar) < 2:
                    row += b"\x50\x80\xd0"
                elif abs(xx - ax) <= 3 and abs(yy - ay) <= 3:
                    row += b"\xff\xd0\x30"
                else:
                    row += b"\x18\x1c\x22" if (xx // 20 + yy // 20) % 2 else b"\x22\x27\x2f"
            rows.append(row)
        return make_png(w, h, rows)

    def _camera_handler(self):
        robot = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def do_GET(self):
                if self.path.startswith("/stream"):
                    robot.log("카메라 스트림 시작: %s (%s)" % (self.client_address[0], self.path))
                    self.send_response(200)
                    self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                    self.send_header("Cache-Control", "no-store")
                    self.end_headers()
                    try:
                        while robot.running.is_set():
                            img = robot._frame(time.monotonic())
                            self.wfile.write(b"--frame\r\nContent-Type: image/png\r\nContent-Length: %d\r\n\r\n"
                                             % len(img) + img + b"\r\n")
                            self.wfile.flush()
                            time.sleep(0.2)
                    except OSError:
                        pass
                    return
                body = "가짜 ArmPi 카메라: /stream?topic=/usb_cam/image_raw&type=ros_compressed&qos_profile=sensor_data\n".encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/plain; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

        return Handler


# ───────────────────────── 명령행 ─────────────────────────

def _floats(text, n):
    vals = [float(v) for v in text.split(",")]
    if len(vals) != n or not all(math.isfinite(v) for v in vals):
        raise argparse.ArgumentTypeError("숫자 %d개를 쉼표로" % n)
    return vals


def load_profile_kinematics(path):
    """하드웨어 프로필 YAML 에서 arm.kinematics.* (관절 기준·범위)만 읽는다. 표준 라이브러리만 쓰는 간단한 읽기:
    'arm.kinematics.<키>: <숫자>' 또는 '[숫자, ...]' 줄만 본다."""
    out = {}
    with open(path, "r", encoding="utf-8") as fp:
        for line in fp:
            line = line.split("#", 1)[0].strip()
            if not line.startswith("arm.kinematics.") or ":" not in line:
                continue
            key, val = line.split(":", 1)
            key, val = key[len("arm.kinematics."):].strip(), val.strip()
            if not val or val in ("null", "~"):
                raise SystemExit("하드웨어 프로필의 arm.kinematics.%s 가 비어 있습니다: %s" % (key, path))
            out[key] = json.loads(val)
    return out


def build_config(args):
    cfg = json.loads(json.dumps(DEFAULT_CONFIG))
    if args.hardware_profile:
        cfg["arm"]["kinematics"].update(load_profile_kinematics(args.hardware_profile))
    if args.config:
        with open(args.config, "r", encoding="utf-8") as fp:
            _deep_update(cfg, json.load(fp))
    if args.name:
        cfg["name"] = args.name
    if args.allowed_sources is not None:
        cfg["allowed_sources"] = [c for c in args.allowed_sources.split(",") if c.strip()]
    for key in ("base_watchdog_s", "arm_watchdog_s", "lease_s"):
        v = getattr(args, key)
        if v is not None:
            cfg[key] = v
    if args.workspace:
        w = _floats(args.workspace, 6)
        cfg["arm"]["workspace"] = dict(zip(("x_min", "x_max", "y_min", "y_max", "z_min", "z_max"), w))
    if args.links:
        l = _floats(args.links, 4)
        cfg["arm"]["kinematics"].update(zip(("base_height_m", "upper_arm_m", "forearm_m", "hand_m"), l))
    if args.home:
        h = _floats(args.home, 5)
        cfg["arm"]["home"] = dict(zip(("x", "y", "z", "pitch", "gripper"), h))
    cfg["control_state"] = args.control_state
    if not (0 < cfg["base_watchdog_s"] < cfg["lease_s"] and 0 < cfg["arm_watchdog_s"] < cfg["lease_s"]):
        raise SystemExit("lease 는 두 watchdog 보다 길어야 합니다")
    name = cfg["name"]
    if not name.strip() or len(name) > 32 or any(ord(ch) < 32 or 127 <= ord(ch) <= 159 for ch in name):
        raise SystemExit("이름은 출력 가능한 1~32자")
    return cfg


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bind", default="0.0.0.0", help="받을 주소(Mac 단독 시험은 127.0.0.1 권장)")
    ap.add_argument("--port", type=int, default=5001, help="제어 UDP 포트")
    ap.add_argument("--discovery-port", type=int, default=5002, help="검색 UDP 포트(0 = 끔)")
    ap.add_argument("--rosbridge-port", type=int, default=9090, help="가짜 rosbridge WebSocket 포트(0 = 끔)")
    ap.add_argument("--camera-port", type=int, default=8080, help="가짜 카메라 HTTP 포트(0 = 끔)")
    ap.add_argument("--name", help="검색 응답 이름(기본 armpi-mock)")
    ap.add_argument("--config", help="설정 JSON(DEFAULT_CONFIG 와 같은 구조, 일부만 써도 됨)")
    ap.add_argument("--hardware-profile", help="하드웨어 프로필 YAML(arm.kinematics.* 관절 기준·범위를 이 값으로)")
    ap.add_argument("--allowed-sources", help="허용 CIDR 쉼표 목록(빈 문자열 = 모두 허용)")
    ap.add_argument("--base-watchdog", dest="base_watchdog_s", type=float)
    ap.add_argument("--arm-watchdog", dest="arm_watchdog_s", type=float)
    ap.add_argument("--lease", dest="lease_s", type=float)
    ap.add_argument("--workspace", help="x_min,x_max,y_min,y_max,z_min,z_max (m)")
    ap.add_argument("--links", help="base_height,upper_arm,forearm,hand (m)")
    ap.add_argument("--home", help="x,y,z,pitch,gripper")
    ap.add_argument("--control-state", default="ok", choices=["ok", "missing", "udp_blocked", "fault"],
                    help="제어기(armpi_control) 상태 흉내: ok=허용, missing=상태 미수신, udp_blocked, fault")
    ap.add_argument("--stats-period", type=float, default=5.0)
    args = ap.parse_args(argv)
    cfg = build_config(args)
    robot = MockRobot(cfg, args.bind, args.port, args.discovery_port, args.rosbridge_port, args.camera_port,
                      args.stats_period)
    try:
        robot.start()
    except OSError as exc:
        print("시작 실패(포트 사용 중?): %s" % exc, file=sys.stderr)
        robot.shutdown()
        return 1
    def _term(signum, frame):  # SIGTERM(kill)도 Ctrl+C 처럼 정리하고 끝낸다
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _term)
    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        robot.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
