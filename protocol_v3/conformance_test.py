#!/usr/bin/env python3
"""ArmPi Protocol v3 규격 적합성 시험 (unittest, 표준 라이브러리만).

기준: ../docs/01_PROTOCOL_V3_SPEC.md. 대상은 가짜 로봇(mock_robot.py)이든 로봇의 C++ 수신기(armpi_vr_bridge)든
UDP 로 같은 방식으로 시험한다. 이 파일은 mock_robot.py·armpi_v3_client.py 의 코드를 쓰지 않는다(원시 JSON 을 직접
만들어 보내고 ACK 를 독립적으로 검사한다).

실행 예:
  python3 conformance_test.py --host 127.0.0.1                       # 가짜 로봇(기본 포트 5001/5002)
  python3 conformance_test.py --host 127.0.0.1 --interpretation -v   # 사양이 애매한 부분(해석 시험)까지
  python3 conformance_test.py --host <IP> --condition control_missing  # 제어기(armpi_control) 없이 수신기만 띄운 경우
  python3 conformance_test.py --host 127.0.0.1 --json-report out.json -k T06

대상 조건(--condition)
  normal          제어기가 UDP 를 허용(udp_allowed:true), 비상정지·fault 없음. 대부분의 시험은 이 조건에서만 실행
  control_missing 수신기가 /armpi/control_state 를 받지 못함 → 활성 명령 not_owner (02 문서 '수신기 규칙')
  udp_blocked     제어기가 udp_allowed:false 를 보냄 → 활성 명령 not_owner
  fault           제어기가 fault:true 를 보고 → 활성 명령 fault
  source_blocked  대상의 allowed_sources 가 이 컴퓨터 주소를 막음 → 응답이 전혀 없어야 함
C++ 수신기를 단독으로 시험할 때는 /armpi/control_state 에 {"udp_allowed":true,...} 를 발행해 normal 조건을 만들면
더 많은 시험이 돈다. 조건이 맞지 않는 시험은 "건너뜀"으로 표시된다.

주의: 실제 로봇에 돌리면 차체·팔이 움직일 수 있다(전진 0.3 이하, 팔은 --reachable 점 근처).
반드시 바퀴를 띄우고 팔 주변을 비운 뒤, 물리 비상정지를 손에 두고 실행할 것.
"""

import argparse
import base64
import functools
import json
import math
import os
import re
import socket
import struct
import sys
import time
import unittest
import uuid
from collections import OrderedDict

PROTOCOL = 3
MAX_SEQUENCE = 9007199254740991
ACK_KEYS = ("protocol", "type", "client_id", "sequence", "accepted", "reason", "emergency_stop", "state")
STATES = ("awaiting_neutral", "idle", "active", "timeout", "emergency_stop", "fault")
REASONS = ("ok", "invalid_packet", "unsupported_protocol", "out_of_order", "not_owner", "emergency_stop",
           "neutral_required", "fault", "workspace_limit", "ik_unreachable")
SAFE_STR = re.compile(r"\A[A-Za-z0-9_-]+\Z")
HEX32 = re.compile(r"\A[0-9a-f]{32}\Z")
NAME_RE = re.compile(r"\A(?:[A-Za-z0-9_.~-]|%[0-9A-Fa-f]{2}){1,384}\Z")

CFG = argparse.Namespace()  # main() 에서 채운다


# ───────────────────────── ACK 형식 검사 ─────────────────────────

def _dup(pairs):
    out = {}
    for k, v in pairs:
        if k in out:
            raise AssertionError("ACK 에 중복 키 %r" % k)
        out[k] = v
    return out


def _nofloat(text):
    raise AssertionError("ACK 값에 실수 %s (문자열·불리언·정수만 허용)" % text)


def _noconst(text):
    raise AssertionError("ACK 에 JSON 상수 %s" % text)


def check_ack(raw):
    """사양 2.4: 정확히 8필드, 값은 문자열·불리언·정수만. 위반이면 AssertionError."""
    if len(raw) > 2048:
        raise AssertionError("ACK 가 2048 byte 초과: %d" % len(raw))
    try:
        text = raw.decode("utf-8")
    except UnicodeDecodeError:
        raise AssertionError("ACK 가 UTF-8 이 아님: %r" % raw[:80])
    try:
        obj = json.loads(text, object_pairs_hook=_dup, parse_float=_nofloat, parse_constant=_noconst)
    except ValueError as exc:
        raise AssertionError("ACK JSON 해석 실패: %s (%r)" % (exc, text[:120]))
    if not isinstance(obj, dict):
        raise AssertionError("ACK 가 객체가 아님: %r" % text[:120])
    if set(obj) != set(ACK_KEYS):
        raise AssertionError("ACK 필드 불일치: 누락=%s 추가=%s" % (sorted(set(ACK_KEYS) - set(obj)),
                                                             sorted(set(obj) - set(ACK_KEYS))))
    if type(obj["protocol"]) is not int or obj["protocol"] != PROTOCOL:
        raise AssertionError("ACK protocol 은 정수 3 이어야 함: %r" % obj["protocol"])
    if obj["type"] != "ack":
        raise AssertionError("ACK type 은 'ack': %r" % obj["type"])
    cid = obj["client_id"]
    if cid is not None and (type(cid) is not str or not SAFE_STR.match(cid)):
        raise AssertionError("ACK client_id 는 null 또는 [A-Za-z0-9_-] 문자열: %r" % cid)
    if type(obj["sequence"]) is not int or not -1 <= obj["sequence"] <= MAX_SEQUENCE:
        raise AssertionError("ACK sequence 는 -1~2^53-1 정수: %r" % obj["sequence"])
    for key in ("accepted", "emergency_stop"):
        if type(obj[key]) is not bool:
            raise AssertionError("ACK %s 는 불리언: %r" % (key, obj[key]))
    if obj["reason"] not in REASONS:
        raise AssertionError("ACK reason 이 목록에 없음: %r" % obj["reason"])
    if obj["state"] not in STATES:
        raise AssertionError("ACK state 가 목록에 없음: %r" % obj["state"])
    if obj["accepted"] != (obj["reason"] == "ok"):
        raise AssertionError("accepted 와 reason 불일치: accepted=%r reason=%r" % (obj["accepted"], obj["reason"]))
    if obj["state"] == "emergency_stop" and not obj["emergency_stop"]:
        raise AssertionError("state=emergency_stop 인데 emergency_stop=false")
    return obj


# ───────────────────────── 원시 패킷 ─────────────────────────

def raw_obj(items):
    """[(키, JSON 토큰 문자열)] → bytes. 중복 키·NaN 같은 비정상 토큰을 그대로 넣을 수 있다."""
    return ("{" + ",".join('"%s":%s' % (k, v) for k, v in items) + "}").encode("utf-8")


def fnum(v):
    return repr(float(v))


def cmd_items(cid, seq, active=False, f=0.0, s=0.0, t=0.0):
    return [("protocol", "3"), ("type", '"command"'), ("client_id", '"%s"' % cid), ("sequence", str(seq)),
            ("control_active", "true" if active else "false"), ("forward", fnum(f)), ("strafe", fnum(s)),
            ("turn", fnum(t))]


def arm_items(cid, seq, active=False, pose=None, gripper=0.0):
    x, y, z, p = pose if pose is not None else CFG.reachable
    return [("protocol", "3"), ("type", '"arm_command"'), ("client_id", '"%s"' % cid), ("sequence", str(seq)),
            ("control_active", "true" if active else "false"), ("x", fnum(x)), ("y", fnum(y)), ("z", fnum(z)),
            ("pitch", fnum(p)), ("gripper", fnum(gripper))]


def ev_items(cid, seq, kind):
    return [("protocol", "3"), ("type", '"%s"' % kind), ("client_id", '"%s"' % cid), ("sequence", str(seq))]


def replace(items, key, value):
    return [(k, value if k == key else v) for k, v in items]


def drop(items, key):
    return [(k, v) for k, v in items if k != key]


class Peer:
    """시험용 송신기 1개 = UDP 소켓 1개(= 주소·포트 1개) + UUID 1개 + 자기 sequence."""

    def __init__(self, cid=None):
        self.cid = cid or str(uuid.uuid4())
        self.seq = 0
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", 0))
        self.target = (CFG.ip, CFG.port)
        self.last_raw = None

    def close(self):
        if self.sock is not None:
            self.sock.close()
            self.sock = None

    def next_seq(self):
        self.seq += 1
        return self.seq

    def drain(self):
        self.sock.setblocking(False)
        try:
            while True:
                self.sock.recvfrom(65535)
        except (BlockingIOError, InterruptedError, OSError):
            pass
        finally:
            self.sock.setblocking(True)

    def send_raw(self, data):
        self.drain()
        self.sock.sendto(data, self.target)

    def recv_ack(self, timeout=None):
        """다음 ACK 1개(형식 검사 통과)를 돌려준다. 없으면 AssertionError."""
        timeout = CFG.ack_wait if timeout is None else timeout
        deadline = time.monotonic() + timeout
        while True:
            remain = deadline - time.monotonic()
            if remain <= 0:
                raise AssertionError("%.0f ms 안에 ACK 없음(대상 %s:%d)" % (timeout * 1e3, self.target[0], self.target[1]))
            self.sock.settimeout(remain)
            try:
                data, src = self.sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError as exc:  # ICMP port unreachable 등
                raise AssertionError("수신 오류: %s" % exc)
            if src != self.target:
                raise AssertionError("ACK 출발지 %s:%d ≠ 보낸 곳 %s:%d (사양 1절: 같은 소켓으로 응답)"
                                     % (src[0], src[1], self.target[0], self.target[1]))
            self.last_raw = data
            return check_ack(data)

    def expect_silence(self, wait):
        self.sock.settimeout(wait)
        try:
            data, src = self.sock.recvfrom(65535)
        except socket.timeout:
            return None
        return data

    def request(self, data, seq=None):
        self.send_raw(data)
        ack = self.recv_ack()
        if seq is not None and ack["sequence"] != seq:
            raise AssertionError("ACK sequence %r ≠ 보낸 번호 %r" % (ack["sequence"], seq))
        return ack

    def cmd(self, active=False, f=0.0, s=0.0, t=0.0, seq=None):
        seq = self.next_seq() if seq is None else seq
        return self.request(raw_obj(cmd_items(self.cid, seq, active, f, s, t)), seq)

    def arm(self, active=False, pose=None, gripper=0.0, seq=None):
        seq = self.next_seq() if seq is None else seq
        return self.request(raw_obj(arm_items(self.cid, seq, active, pose, gripper)), seq)

    def event(self, kind, seq=None):
        seq = self.next_seq() if seq is None else seq
        return self.request(raw_obj(ev_items(self.cid, seq, kind)), seq)

    def try_event(self, kind, wait=0.3):
        try:
            seq = self.next_seq()
            self.send_raw(raw_obj(ev_items(self.cid, seq, kind)))
            return self.recv_ack(wait)
        except (AssertionError, OSError):
            return None


# ───────────────────────── 최소 rosbridge 클라이언트 (선택 시험용) ─────────────────────────

class MiniRosbridge:
    def __init__(self, host, port):
        self.sock = socket.create_connection((host, port), timeout=3)
        key = base64.b64encode(os.urandom(16)).decode()
        self.sock.sendall(("GET / HTTP/1.1\r\nHost: %s:%d\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
                           "Sec-WebSocket-Key: %s\r\nSec-WebSocket-Version: 13\r\n\r\n" % (host, port, key)).encode())
        buf = b""
        while b"\r\n\r\n" not in buf:
            chunk = self.sock.recv(1024)
            if not chunk:
                raise OSError("rosbridge handshake 실패")
            buf += chunk
        if b" 101 " not in buf.split(b"\r\n")[0]:
            raise OSError("rosbridge handshake 거부")
        self.rest = buf.split(b"\r\n\r\n", 1)[1]

    def _exact(self, n):
        while len(self.rest) < n:
            chunk = self.sock.recv(4096)
            if not chunk:
                raise OSError("rosbridge 연결 종료")
            self.rest += chunk
        out, self.rest = self.rest[:n], self.rest[n:]
        return out

    def recv(self):
        b0, b1 = self._exact(2)
        n = b1 & 0x7F
        if n == 126:
            n = struct.unpack("!H", self._exact(2))[0]
        elif n == 127:
            n = struct.unpack("!Q", self._exact(8))[0]
        mask = self._exact(4) if b1 & 0x80 else None
        data = self._exact(n)
        if mask:
            data = bytes(b ^ mask[i % 4] for i, b in enumerate(data))
        return b0 & 0x0F, data

    def send(self, obj):
        data = json.dumps(obj).encode()
        mask = os.urandom(4)
        n = len(data)
        hdr = bytearray([0x81])
        if n < 126:
            hdr.append(0x80 | n)
        elif n < 65536:
            hdr.append(0x80 | 126)
            hdr += struct.pack("!H", n)
        else:
            hdr.append(0x80 | 127)
            hdr += struct.pack("!Q", n)
        self.sock.sendall(bytes(hdr) + mask + bytes(b ^ mask[i % 4] for i, b in enumerate(data)))

    def call(self, service, timeout=3.0):
        cid = "conf-%d" % int(time.time() * 1e6)
        self.send({"op": "call_service", "service": service, "type": "std_srvs/Trigger", "args": {}, "id": cid})
        self.sock.settimeout(timeout)
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            op, data = self.recv()
            if op != 0x1:
                continue
            msg = json.loads(data.decode("utf-8"))
            if msg.get("op") == "service_response" and msg.get("id") == cid:
                return msg
        raise AssertionError("서비스 응답 없음: %s" % service)

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass


# ───────────────────────── 시험 기반 ─────────────────────────

def require(*conditions):
    def deco(fn):
        @functools.wraps(fn)
        def wrapper(self, *a, **k):
            if CFG.condition not in conditions:
                self.skipTest("대상 조건 %s 에서만 실행(현재 --condition %s)" % ("/".join(conditions), CFG.condition))
            return fn(self, *a, **k)
        return wrapper
    return deco


ANY = require("normal", "control_missing", "udp_blocked", "fault")   # 응답이 오는 모든 조건
NORMAL = require("normal")


def interpretation(fn):
    """사양 문구가 애매해 이 구현이 고른 해석을 확인하는 시험. --interpretation 일 때만 실행."""
    @functools.wraps(fn)
    def wrapper(self, *a, **k):
        if not CFG.interpretation:
            self.skipTest("[해석] 사양이 명시하지 않은 동작 — --interpretation 으로 실행")
        return fn(self, *a, **k)
    wrapper.__doc__ = "[해석] " + (fn.__doc__ or "")
    return wrapper


class Base(unittest.TestCase):
    maxDiff = None

    def setUp(self):
        self.peers = []

    def tearDown(self):
        # 이 시험이 만든 송신기를 정리: 비상정지 해제 + disconnect(소유권 즉시 반환). 실패는 무시.
        for p in reversed(self.peers):
            p.try_event("clear_emergency")
            p.try_event("disconnect")
            p.close()
        if self.peers and CFG.condition != "source_blocked":
            # 소유자가 없는데 latch 만 남은 경우(소유자가 disconnect 한 뒤)를 치운다.
            j = Peer()
            j.try_event("clear_emergency")
            j.try_event("disconnect")
            j.close()

    # ── 도우미 ──

    def peer(self, cid=None):
        p = Peer(cid)
        self.peers.append(p)
        return p

    def neutral_state(self):
        return "fault" if CFG.condition == "fault" else "idle"

    def assertAck(self, ack, accepted, reason, state=None, estop=None, note=""):
        states = (state,) if isinstance(state, str) else state
        ok = (ack["accepted"] is accepted and ack["reason"] == reason
              and (states is None or ack["state"] in states) and (estop is None or ack["emergency_stop"] is estop))
        if not ok:
            self.fail("%s기대 accepted=%s reason=%s state=%s estop=%s / 실제 accepted=%s reason=%s state=%s estop=%s"
                      % (note + ": " if note else "", accepted, reason, "/".join(states) if states else "*",
                         "*" if estop is None else estop, ack["accepted"], ack["reason"], ack["state"],
                         ack["emergency_stop"]))

    def claim(self, p):
        """차체 중립 1개로 소유권을 잡고 차체 중립을 확인한다. 남은 소유자가 있으면 lease 만큼 기다려 한 번 더."""
        ack = p.cmd(False)
        if ack["reason"] == "not_owner":
            time.sleep(CFG.lease + CFG.margin)
            ack = p.cmd(False)
        if ack["emergency_stop"] and CFG.condition == "normal":
            self.fail("대상이 비상정지 latch 상태(이전 시험 잔여물 또는 외부 비상정지)")
        self.assertAck(ack, True, "ok", self.neutral_state(), note="소유권 획득용 중립")
        return ack

    def ready(self, p):
        """차체·팔 모두 중립 확인."""
        self.claim(p)
        self.assertAck(p.arm(False), True, "ok", self.neutral_state(), note="팔 중립")

    def feed(self, seconds, fn, period=0.05):
        """seconds 동안 period 마다 fn() 을 부르고 ACK 목록을 돌려준다."""
        acks = []
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            t0 = time.monotonic()
            acks.append(fn())
            time.sleep(max(0.0, period - (time.monotonic() - t0)))
        return acks


# ───────────────────────── T01 검색 ─────────────────────────

class T01Discovery(Base):
    def _probe(self, payloads, wait=0.4):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind(("0.0.0.0", 0))
        try:
            for data in payloads:
                s.sendto(data, (CFG.ip, CFG.discovery_port))
            out = []
            end = time.monotonic() + wait
            while True:
                remain = end - time.monotonic()
                if remain <= 0:
                    break
                s.settimeout(remain)
                try:
                    out.append(s.recvfrom(4096))
                except socket.timeout:
                    break
            return out
        finally:
            s.close()

    @ANY
    def test_01_reply_format(self):
        """검색 요청 → ARMPI_HERE/1|nonce|32hex|이름|제어포트|3, 출발 포트 = 검색 포트 (사양 1절)"""
        nonce = uuid.uuid4().hex
        replies = self._probe([("ARMPI_DISCOVER/1|" + nonce).encode()])
        self.assertTrue(replies, "검색 응답 없음")
        data, src = replies[0]
        self.assertEqual(src[1], CFG.discovery_port, "응답 출발 포트가 검색 포트가 아님")
        self.assertTrue(all(32 <= b <= 126 for b in data), "출력 가능한 ASCII 가 아님: %r" % data)
        self.assertTrue(80 <= len(data) <= 640, "응답 길이 %d (v2 Unity 파서 조건 80~640)" % len(data))
        f = data.decode("ascii").split("|")
        self.assertEqual(len(f), 6, "필드 6개가 아님: %r" % data)
        self.assertEqual(f[0], "ARMPI_HERE/1")
        self.assertEqual(f[1], nonce, "nonce 를 그대로 돌려주지 않음")
        self.assertRegex(f[2], HEX32, "기기 id 는 소문자 32 hex")
        self.assertRegex(f[3], NAME_RE, "이름은 URL 인코딩 문자열")
        self.assertEqual(f[4], str(CFG.port), "제어 포트가 --port 와 다름")
        self.assertEqual(f[5], "3", "protocol 표시가 3 이 아님")

    @ANY
    def test_02_bad_requests_ignored(self):
        """형식이 틀린 검색 요청에는 응답하지 않는다(v2 규칙 유지)"""
        h = uuid.uuid4().hex
        bad = [("OTHERBOT_DISCOVER/1|" + h).encode(), ("ARMPI_DISCOVER/2|" + h).encode(),
               ("ARMPI_DISCOVER/1|" + h[:31]).encode(), ("ARMPI_DISCOVER/1|" + h + "0").encode(),
               ("ARMPI_DISCOVER/1|" + "g" * 32).encode(), ("ARMPI_DISCOVER/1|" + h + "\n").encode(),
               b"ARMPI_DISCOVER/1|", b""]
        for data in bad:
            with self.subTest(request=data[:60]):
                self.assertEqual(self._probe([data], wait=0.25), [], "잘못된 요청에 응답함")

    @ANY
    @interpretation
    def test_03_uppercase_nonce_ignored(self):
        """대문자 hex nonce 거부(v2 discovery.py 와 같이 소문자만). 사양은 '32 hex' 라고만 적음"""
        h = uuid.uuid4().hex.upper()
        self.assertEqual(self._probe([("ARMPI_DISCOVER/1|" + h).encode()], wait=0.25), [])

    @ANY
    def test_04_rate_limit(self):
        """응답은 초당 최대 20회 (사양 1절)"""
        time.sleep(1.1)  # 이전 시험의 1초 창을 비운다
        replies = self._probe([("ARMPI_DISCOVER/1|" + uuid.uuid4().hex).encode() for _ in range(30)], wait=0.5)
        self.assertGreaterEqual(len(replies), 1, "응답이 하나도 없음")
        self.assertLessEqual(len(replies), 20, "1초 안에 응답 %d회 (상한 20)" % len(replies))


# ───────────────────────── T02 엄격한 파서 ─────────────────────────

class T02Parser(Base):
    def expect_invalid(self, p, data, ident, seq=None, note=""):
        """ident: True=client_id·sequence 되돌림, False=null/-1, 'id'=client_id 만 되돌림(sequence -1),
        'null_id'=client_id null(sequence 는 보지 않음), None=식별 검사 안 함"""
        p.send_raw(data)
        ack = p.recv_ack()
        self.assertAck(ack, False, "invalid_packet", note=note or data[:80].decode("utf-8", "replace"))
        if ident is True:
            self.assertTrue(isinstance(ack["client_id"], str) and ack["client_id"].lower() == p.cid.lower(),
                            "%s: client_id 를 되돌려야 함, 실제 %r" % (note, ack["client_id"]))
            self.assertEqual(ack["sequence"], seq, "%s: sequence 를 되돌려야 함" % note)
        elif ident is False:
            self.assertIsNone(ack["client_id"], "%s: 식별 불가 → client_id null" % note)
            self.assertEqual(ack["sequence"], -1, "%s: 식별 불가 → sequence -1" % note)
        elif ident == "id":
            self.assertEqual(ack["sequence"], -1, "%s: sequence 가 잘못됐으면 -1" % note)
            self.assertIn(ack["client_id"], (p.cid, None), "%s: client_id 는 되돌리거나 null" % note)
        elif ident == "null_id":
            self.assertIsNone(ack["client_id"], "%s: client_id 가 잘못됐으면 null" % note)
        return ack

    @ANY
    def test_01_unidentifiable_packets(self):
        """JSON 이 아니거나 객체가 아니면 invalid_packet, client_id null, sequence -1 (사양 2.4)"""
        p = self.peer()
        good = raw_obj(cmd_items(p.cid, 1)).decode()
        for data in (b"hello", b"[]", b'"str"', b"3", b"null", b"{}", b"\xff\xfe{}", good[:-1].encode(),
                     (good + "x").encode(), (good + good).encode(), b'{"protocol":3,'):
            with self.subTest(packet=data[:50]):
                self.expect_invalid(p, data, False)

    @ANY
    def test_02_oversize(self):
        """2048 byte 를 넘는 데이터그램은 invalid_packet (사양 1절)"""
        p = self.peer()
        body = raw_obj(cmd_items(p.cid, p.next_seq()))
        data = b"{" + b" " * (2049 - len(body)) + body[1:]
        self.assertEqual(len(data), 2049)
        self.expect_invalid(p, data, None, note="2049 byte")

    @ANY
    def test_03_max_size_accepted(self):
        """정확히 2048 byte 인 올바른 중립 명령은 수락"""
        p = self.peer()
        seq = p.next_seq()
        body = raw_obj(cmd_items(p.cid, seq))
        data = b"{" + b" " * (2048 - len(body)) + body[1:]
        self.assertEqual(len(data), 2048)
        ack = p.request(data, seq)
        self.assertAck(ack, True, "ok", note="2048 byte 중립")

    @ANY
    def test_04_command_field_errors(self):
        """command: 필드 누락·추가·중복, 타입·범위 오류는 invalid_packet (해석 가능한 JSON 이면 client_id·sequence 되돌림)"""
        p = self.peer()
        cases = [
            ("turn 누락", lambda s: drop(cmd_items(p.cid, s), "turn")),
            ("strafe 없음(v2 모양, protocol 3)", lambda s: drop(cmd_items(p.cid, s), "strafe")),
            ("추가 필드", lambda s: cmd_items(p.cid, s) + [("foo", "1")]),
            ("중복 키 forward", lambda s: cmd_items(p.cid, s) + [("forward", "0.0")], None),
            ("공통 4필드만", lambda s: cmd_items(p.cid, s)[:4]),
            ("protocol 문자열", lambda s: replace(cmd_items(p.cid, s), "protocol", '"3"')),
            ("protocol 3.0", lambda s: replace(cmd_items(p.cid, s), "protocol", "3.0")),
            ("protocol true", lambda s: replace(cmd_items(p.cid, s), "protocol", "true")),
            ("protocol null", lambda s: replace(cmd_items(p.cid, s), "protocol", "null")),
            ("type 대문자", lambda s: replace(cmd_items(p.cid, s), "type", '"Command"')),
            ("type 미지원", lambda s: replace(cmd_items(p.cid, s), "type", '"ping"')),
            ("type 숫자", lambda s: replace(cmd_items(p.cid, s), "type", "1")),
            ("control_active 1", lambda s: replace(cmd_items(p.cid, s), "control_active", "1")),
            ("control_active 문자열", lambda s: replace(cmd_items(p.cid, s), "control_active", '"true"')),
            ("control_active null", lambda s: replace(cmd_items(p.cid, s), "control_active", "null")),
            ("forward 1.5", lambda s: replace(cmd_items(p.cid, s), "forward", "1.5")),
            ("forward -1.0001", lambda s: replace(cmd_items(p.cid, s), "forward", "-1.0001")),
            ("strafe 2", lambda s: replace(cmd_items(p.cid, s), "strafe", "2")),
            ("turn 문자열", lambda s: replace(cmd_items(p.cid, s), "turn", '"0.1"')),
            ("forward true", lambda s: replace(cmd_items(p.cid, s), "forward", "true")),
            ("forward NaN", lambda s: replace(cmd_items(p.cid, s), "forward", "NaN"), None),
            ("forward Infinity", lambda s: replace(cmd_items(p.cid, s), "forward", "Infinity"), None),
            ("strafe -Infinity", lambda s: replace(cmd_items(p.cid, s), "strafe", "-Infinity"), None),
            ("turn 1e400", lambda s: replace(cmd_items(p.cid, s), "turn", "1e400"), None),
            ("forward null", lambda s: replace(cmd_items(p.cid, s), "forward", "null")),
            ("forward 배열", lambda s: replace(cmd_items(p.cid, s), "forward", "[0.1]")),
            ("forward 객체", lambda s: replace(cmd_items(p.cid, s), "forward", '{"v":0.1}')),
            ("disconnect + 추가 필드", lambda s: ev_items(p.cid, s, "disconnect") + [("control_active", "false")]),
            ("clear_emergency + 추가 필드", lambda s: ev_items(p.cid, s, "clear_emergency") + [("x", "0")]),
            ("emergency_stop + 추가 필드", lambda s: ev_items(p.cid, s, "emergency_stop") + [("forward", "0.0")]),
            ("arm_command 에 command 필드", lambda s: replace(cmd_items(p.cid, s), "type", '"arm_command"')),
        ]
        self._run_cases(p, cases)

    def _run_cases(self, p, cases):
        # 세 번째 값 None = JSON 자체가 표준이 아니거나(NaN·Infinity·중복 키) double 로 표현 못 하는 수(1e400)라
        # 파서가 식별하지 못할 수 있는 사례: invalid_packet 만 확인하고 client_id·sequence 되돌림은 보지 않는다.
        for case in cases:
            name, make = case[0], case[1]
            ident = case[2] if len(case) > 2 else True
            with self.subTest(case=name):
                seq = p.next_seq()
                self.expect_invalid(p, raw_obj(make(seq)), ident, seq, note=name)

    @ANY
    def test_05_arm_field_errors(self):
        """arm_command: 10필드 엄격, pitch −π~π, gripper 0~1, x/y/z 유한 숫자 (사양 2.2)"""
        p = self.peer()
        cases = [
            ("gripper 누락", lambda s: drop(arm_items(p.cid, s), "gripper")),
            ("추가 필드", lambda s: arm_items(p.cid, s) + [("roll", "0.0")]),
            ("중복 x", lambda s: arm_items(p.cid, s) + [("x", "0.1")], None),
            ("x 문자열", lambda s: replace(arm_items(p.cid, s), "x", '"0.1"')),
            ("y NaN", lambda s: replace(arm_items(p.cid, s), "y", "NaN"), None),
            ("z true", lambda s: replace(arm_items(p.cid, s), "z", "true")),
            ("x 1e400", lambda s: replace(arm_items(p.cid, s), "x", "1e400"), None),
            ("z null", lambda s: replace(arm_items(p.cid, s), "z", "null")),
            ("pitch 3.2", lambda s: replace(arm_items(p.cid, s), "pitch", "3.2")),
            ("pitch -3.2", lambda s: replace(arm_items(p.cid, s), "pitch", "-3.2")),
            ("pitch Infinity", lambda s: replace(arm_items(p.cid, s), "pitch", "Infinity"), None),
            ("gripper 1.01", lambda s: replace(arm_items(p.cid, s), "gripper", "1.01")),
            ("gripper -0.01", lambda s: replace(arm_items(p.cid, s), "gripper", "-0.01")),
            ("gripper 문자열", lambda s: replace(arm_items(p.cid, s), "gripper", '"0"')),
            ("control_active 문자열", lambda s: replace(arm_items(p.cid, s), "control_active", '"false"')),
            ("command 에 arm 필드", lambda s: replace(arm_items(p.cid, s), "type", '"command"')),
        ]
        self._run_cases(p, cases)

    @ANY
    def test_06_sequence_errors(self):
        """sequence 는 0~2^53-1 정수만(2.0·문자열·불리언 불가). 잘못되면 sequence -1"""
        p = self.peer()
        for token in ("5.0", "-1", "9007199254740992", '"5"', "true", "null", "1e3"):
            with self.subTest(sequence=token):
                self.expect_invalid(p, raw_obj(replace(cmd_items(p.cid, 1), "sequence", token)), "id", note=token)

    @ANY
    def test_07_uuid_errors(self):
        """client_id: 하이픈 포함 36자 UUID, nil 불가. 잘못되면 client_id null"""
        p = self.peer()
        u = str(uuid.uuid4())
        bad = [u.replace("-", ""), "00000000-0000-0000-0000-000000000000", "{%s}" % u, "urn:uuid:" + u,
               "g" + u[1:], u[:8] + u[9:13] + "-" + u[8] + u[14:], u + "0", u[:-1], ""]
        for cid in bad:
            with self.subTest(client_id=cid):
                self.expect_invalid(p, raw_obj(cmd_items(cid, 1)), "null_id", note=cid)
        self.expect_invalid(p, raw_obj(replace(cmd_items(u, 1), "client_id", "123")), "null_id", note="숫자")

    @ANY
    def test_08_valid_number_forms(self):
        """정수·지수 표기·-0.0·경계값(±1, ±π, 0/1)은 올바른 숫자로 수락"""
        p = self.peer()
        self.claim(p)
        for f, s, t in (("1", "-1", "0"), ("1e0", "-1E0", "-0.0"), ("0.25e-1", "0", "1.0")):
            with self.subTest(axes=(f, s, t)):
                seq = p.next_seq()
                items = cmd_items(p.cid, seq)
                items = replace(replace(replace(items, "forward", f), "strafe", s), "turn", t)
                self.assertAck(p.request(raw_obj(items), seq), True, "ok", note="중립(control_active false) %s" % ((f, s, t),))
        for pitch, grip in ((repr(math.pi), "0"), (repr(-math.pi), "1"), (repr(-math.pi / 2), "0.5")):
            with self.subTest(pitch=pitch, gripper=grip):
                seq = p.next_seq()
                items = replace(replace(arm_items(p.cid, seq), "pitch", pitch), "gripper", grip)
                self.assertAck(p.request(raw_obj(items), seq), True, "ok", note="팔 중립 pitch=%s" % pitch)

    @ANY
    def test_09_uuid_case_insensitive(self):
        """client_id 는 대소문자 무시 비교: 대문자로 시작한 세션을 소문자로 이어서 쓸 수 있다"""
        upper = str(uuid.uuid4()).upper()
        p = self.peer(upper)
        ack = self.claim(p)
        self.assertEqual(ack["client_id"].lower(), upper.lower())
        p.cid = upper.lower()
        self.assertAck(p.cmd(False), True, "ok", note="같은 소켓·소문자 UUID")

    @ANY
    def test_10_invalid_from_other_client_does_not_disturb_owner(self):
        """다른 클라이언트의 잘못된 패킷은 invalid_packet 일 뿐 소유권을 바꾸지 않는다"""
        a, b = self.peer(), self.peer()
        self.claim(a)
        seq = b.next_seq()
        self.expect_invalid(b, raw_obj(replace(cmd_items(b.cid, seq), "forward", "9")), True, seq)
        self.assertAck(a.cmd(False), True, "ok", note="소유자 A 계속")
        self.assertAck(b.cmd(False), False, "not_owner", note="B 는 여전히 비소유자")


# ───────────────────────── T03 protocol 번호 ─────────────────────────

class T03Protocol(Base):
    @ANY
    def test_01_unsupported_protocol(self):
        """protocol 이 3 이 아닌 정수 → unsupported_protocol, client_id·sequence 되돌림, ACK protocol 은 3"""
        p = self.peer()
        for proto in ("2", "4", "0", "-1", "1000"):
            with self.subTest(protocol=proto):
                seq = p.next_seq()
                ack = p.request(raw_obj(replace(cmd_items(p.cid, seq), "protocol", proto)), seq)
                self.assertAck(ack, False, "unsupported_protocol")
                self.assertEqual(ack["client_id"].lower(), p.cid)
        seq = p.next_seq()
        self.assertAck(p.request(raw_obj(replace(arm_items(p.cid, seq), "protocol", "4")), seq),
                       False, "unsupported_protocol", note="arm_command protocol 4")

    @ANY
    @interpretation
    def test_02_v2_packet(self):
        """v2 모양(7필드, protocol 2) 패킷 → unsupported_protocol (v2 코드처럼 protocol 을 필드 검사보다 먼저 판정)"""
        p = self.peer()
        seq = p.next_seq()
        items = replace(drop(cmd_items(p.cid, seq), "strafe"), "protocol", "2")
        self.assertAck(p.request(raw_obj(items), seq), False, "unsupported_protocol")


# ───────────────────────── T04 세션·소유권·lease ─────────────────────────

class T04Session(Base):
    @NORMAL
    def test_01_new_owner_needs_neutral(self):
        """새 조종자의 첫 활성 명령은 neutral_required(awaiting_neutral). 중립 확인 뒤 활성 (사양 2.4·3절)"""
        p = self.peer()
        ack = p.cmd(True, 0.3)
        if ack["reason"] == "not_owner":
            time.sleep(CFG.lease + CFG.margin)
            ack = p.cmd(True, 0.3)
        self.assertAck(ack, False, "neutral_required", "awaiting_neutral", estop=False)
        self.assertAck(p.cmd(False), True, "ok", "idle")
        self.assertAck(p.cmd(True, 0.3), True, "ok", "active")
        self.assertAck(p.arm(True), False, "neutral_required", "awaiting_neutral", note="팔 채널은 아직 중립 미확인")

    @ANY
    def test_02_neutral_forms(self):
        """차체 중립 = control_active:false(축 값 무관) 또는 true 이면서 세 축 0"""
        p = self.peer()
        self.claim(p)
        self.assertAck(p.cmd(False, 0.5, -0.5, 0.2), True, "ok", self.neutral_state(), note="false + 축 값")
        states = ("fault",) if CFG.condition == "fault" else ("idle", "active")
        self.assertAck(p.cmd(True, 0.0, 0.0, 0.0), True, "ok", states, note="true + 세 축 0")

    @ANY
    def test_03_out_of_order_shared_sequence(self):
        """sequence 는 command·arm_command·event 공용. 이전 번호 이하 → out_of_order"""
        p = self.peer()
        p.seq = 9
        self.claim(p)  # seq 10
        self.assertAck(p.cmd(False, seq=10), False, "out_of_order", note="같은 번호")
        self.assertAck(p.cmd(False, seq=9), False, "out_of_order", note="작은 번호")
        self.assertAck(p.arm(False, seq=10), False, "out_of_order", note="팔도 같은 번호 공간")
        self.assertAck(p.event("clear_emergency", seq=5), False, "out_of_order", note="이벤트도 같은 번호 공간")
        self.assertAck(p.cmd(False, seq=11), True, "ok")
        self.assertAck(p.arm(False, seq=12), True, "ok")
        self.assertAck(p.cmd(False, seq=12), False, "out_of_order", note="팔이 쓴 12")
        self.assertAck(p.arm(False, seq=13), True, "ok")
        self.assertAck(p.cmd(False, seq=1000), True, "ok", note="건너뛴 번호는 허용")
        ack = p.cmd(False, seq=999)
        self.assertAck(ack, False, "out_of_order")
        self.assertEqual(ack["sequence"], 999, "거부 ACK 도 sequence 를 되돌림")

    @ANY
    def test_04_not_owner(self):
        """lease 동안 다른 UUID, 또는 같은 UUID 의 다른 주소·포트는 not_owner"""
        a, b = self.peer(), self.peer()
        self.claim(a)
        ack = b.cmd(False)
        self.assertAck(ack, False, "not_owner", note="다른 UUID 중립")
        self.assertEqual(ack["client_id"].lower(), b.cid)
        self.assertAck(b.arm(False), False, "not_owner", note="다른 UUID 팔 중립")
        self.assertAck(b.cmd(True, 0.2), False, "not_owner", note="다른 UUID 활성")
        a2 = self.peer(a.cid)
        a2.seq = a.seq + 100
        self.assertAck(a2.cmd(False), False, "not_owner", note="같은 UUID, 다른 포트")
        self.assertAck(a.cmd(False), True, "ok", note="원래 소유자")

    @ANY
    def test_05_disconnect_retires(self):
        """disconnect 한 UUID 는 이후 거부(다른 포트에서도). 소유권은 즉시 풀림"""
        a, b = self.peer(), self.peer()
        self.claim(a)
        self.assertAck(a.event("disconnect"), True, "ok")
        self.assertAck(a.cmd(False), False, "not_owner", note="disconnect 한 UUID")
        a2 = self.peer(a.cid)
        a2.seq = a.seq + 10
        self.assertAck(a2.cmd(False), False, "not_owner", note="disconnect 한 UUID, 새 포트")
        self.assertAck(b.cmd(False), True, "ok", self.neutral_state(), note="다른 UUID 즉시 획득")

    @ANY
    def test_06_disconnect_without_owner(self):
        """소유자가 없을 때의 disconnect 도 그 UUID 를 retire 한다(v2 와 같음)"""
        c = self.peer()
        self.assertAck(c.event("disconnect"), True, "ok")
        self.assertAck(c.cmd(False), False, "not_owner")

    @ANY
    def test_07_non_owner_disconnect_rejected(self):
        """비소유자의 disconnect 는 not_owner 이고 소유자에게 영향 없음, 그 UUID 도 retire 되지 않음"""
        a, b = self.peer(), self.peer()
        self.claim(a)
        self.assertAck(b.event("disconnect"), False, "not_owner")
        self.assertAck(a.cmd(False), True, "ok")
        self.assertAck(a.event("disconnect"), True, "ok")
        self.assertAck(b.cmd(False), True, "ok", note="B 는 retire 되지 않았으므로 획득 가능")

    @ANY
    def test_08_lease_holds(self):
        """lease(기본 1.0 s) 안에서는 소유권 유지: watchdog 보다 길게 쉬어도 다른 UUID 는 not_owner"""
        a, b = self.peer(), self.peer()
        self.claim(a)
        time.sleep(CFG.lease * 0.6)
        self.assertAck(b.cmd(False), False, "not_owner")
        self.assertAck(a.cmd(False), True, "ok", self.neutral_state())

    @ANY
    def test_09_lease_reacquire_same_uuid(self):
        """lease 만료 뒤 같은 UUID 는 마지막 번호보다 큰 번호로 재획득(같은 포트·새 포트 모두). 이하 번호는 out_of_order"""
        a = self.peer()
        self.claim(a)
        last = a.seq
        time.sleep(CFG.lease + CFG.margin)
        self.assertAck(a.cmd(False, seq=last), False, "out_of_order", note="만료 뒤 같은 번호")
        self.assertAck(a.cmd(False, seq=last + 1), True, "ok", self.neutral_state(), note="만료 뒤 큰 번호")
        a.seq = last + 1
        time.sleep(CFG.lease + CFG.margin)
        a2 = self.peer(a.cid)
        a2.seq = a.seq
        self.assertAck(a2.cmd(False), True, "ok", self.neutral_state(), note="만료 뒤 새 포트에서 같은 UUID")
        a.seq = a2.seq + 10  # 정리용

    @ANY
    def test_10_lease_takeover_retires_previous(self):
        """lease 만료 뒤 다른 UUID 가 인수하면 이전 UUID 는 retire(소유자가 없어져도 거부)"""
        a, b = self.peer(), self.peer()
        self.claim(a)
        time.sleep(CFG.lease + CFG.margin)
        self.assertAck(b.cmd(False), True, "ok", self.neutral_state(), note="B 인수")
        self.assertAck(a.cmd(False), False, "not_owner", note="A retire")
        self.assertAck(b.event("disconnect"), True, "ok")
        self.assertAck(a.cmd(False), False, "not_owner", note="소유자 없어도 A 는 retire 상태")


# ───────────────────────── T05 watchdog ─────────────────────────

class T05Watchdog(Base):
    @NORMAL
    def test_01_base_watchdog(self):
        """차체 watchdog(기본 0.4 s): 그 안에서는 active 유지, 넘으면 timeout → 새 중립 필요"""
        p = self.peer()
        self.claim(p)
        self.assertAck(p.cmd(True, 0.2), True, "ok", "active")
        time.sleep(CFG.base_wd * 0.6)
        self.assertAck(p.cmd(True, 0.2), True, "ok", "active", note="watchdog 전")
        time.sleep(CFG.base_wd + CFG.margin)
        self.assertAck(p.cmd(True, 0.2), False, "neutral_required", "timeout", note="watchdog 뒤 활성")
        self.assertAck(p.cmd(False), True, "ok", "idle")
        self.assertAck(p.cmd(True, 0.2), True, "ok", "active")

    @NORMAL
    def test_02_arm_watchdog(self):
        """팔 watchdog(기본 0.4 s): 넘으면 팔 채널 timeout → 새 중립 필요"""
        p = self.peer()
        self.ready(p)
        self.assertAck(p.arm(True), True, "ok", "active")
        time.sleep(CFG.arm_wd * 0.6)
        self.assertAck(p.arm(True), True, "ok", "active", note="watchdog 전")
        time.sleep(CFG.arm_wd + CFG.margin)
        self.assertAck(p.arm(True), False, "neutral_required", "timeout", note="watchdog 뒤 활성")
        self.assertAck(p.arm(False), True, "ok", "idle")
        self.assertAck(p.arm(True), True, "ok", "active")

    @NORMAL
    def test_03_arm_times_out_while_base_fed(self):
        """채널별 watchdog: 차체 패킷만 계속 와도 팔 채널은 timeout (차체는 active 유지)"""
        p = self.peer()
        self.ready(p)
        self.assertAck(p.arm(True), True, "ok", "active")
        acks = self.feed(CFG.arm_wd + CFG.margin, lambda: p.cmd(True, 0.1))
        for a in acks:
            self.assertAck(a, True, "ok", "active", note="차체 계속")
        self.assertAck(p.arm(True), False, "neutral_required", "timeout", note="팔 timeout")
        self.assertAck(p.cmd(True, 0.1), True, "ok", "active", note="차체는 영향 없음")

    @NORMAL
    def test_04_base_times_out_while_arm_fed(self):
        """채널별 watchdog: 팔 패킷만 계속 와도 차체 채널은 timeout (팔은 active 유지)"""
        p = self.peer()
        self.ready(p)
        self.assertAck(p.cmd(True, 0.1), True, "ok", "active")
        acks = self.feed(CFG.base_wd + CFG.margin, lambda: p.arm(True))
        for a in acks:
            self.assertAck(a, True, "ok", "active", note="팔 계속")
        self.assertAck(p.cmd(True, 0.1), False, "neutral_required", "timeout", note="차체 timeout")
        self.assertAck(p.arm(True), True, "ok", "active", note="팔은 영향 없음")

    @NORMAL
    def test_05_arm_rejects_do_not_refresh(self):
        """reject: workspace_limit·ik_unreachable 거부는 팔 watchdog 을 갱신하지 않는다 (사양 2.2·3절).
        nearest: 범위 밖·닿지 않는 목표도 가까운 자세로 수락되므로 watchdog 이 갱신되고 팔 채널은 active 를 유지한다."""
        p = self.peer()
        self.ready(p)
        self.assertAck(p.arm(True), True, "ok", "active")
        outside = (CFG.workspace[1] + 0.05, 0.0, CFG.reachable[2], CFG.reachable[3])
        toggle = [0]

        def bad():
            toggle[0] ^= 1
            return p.arm(True, outside if toggle[0] else CFG.unreachable)
        acks = self.feed(CFG.arm_wd + CFG.margin, bad)
        if CFG.arm_policy == "nearest":
            for a in acks:
                self.assertAck(a, True, "ok", "active", note="nearest: 보정해서 수락")
            self.assertAck(p.arm(True), True, "ok", "active", note="보정 목표만 온 뒤 정상 목표")
            return
        self.assertIn(acks[0]["reason"], ("workspace_limit",), "첫 거부 사유")
        for a in acks:
            self.assertFalse(a["accepted"], "범위 밖 목표가 수락됨: %r" % a)
        self.assertAck(p.arm(True), False, "neutral_required", "timeout", note="거부만 온 뒤 정상 목표")

    @NORMAL
    def test_06_invalid_does_not_refresh(self):
        """같은 UUID 의 형식 오류 패킷은 watchdog 을 갱신하지 않는다"""
        p = self.peer()
        self.claim(p)
        self.assertAck(p.cmd(True, 0.2), True, "ok", "active")

        def bad():
            seq = p.next_seq()
            return p.request(raw_obj(replace(cmd_items(p.cid, seq, True, 0.2), "forward", "2.0")), seq)
        for a in self.feed(CFG.base_wd + CFG.margin, bad):
            self.assertAck(a, False, "invalid_packet")
        self.assertAck(p.cmd(True, 0.2), False, "neutral_required", "timeout")

    @NORMAL
    def test_07_non_owner_does_not_refresh(self):
        """다른 클라이언트의 패킷(not_owner)은 소유자의 watchdog 을 갱신하지 않는다"""
        a, b = self.peer(), self.peer()
        self.claim(a)
        self.assertAck(a.cmd(True, 0.2), True, "ok", "active")
        for ack in self.feed(CFG.base_wd + CFG.margin, lambda: b.cmd(False)):
            self.assertAck(ack, False, "not_owner")
        self.assertAck(a.cmd(True, 0.2), False, "neutral_required", "timeout")


# ───────────────────────── T06 비상정지 ─────────────────────────

class T06Estop(Base):
    @NORMAL
    def test_01_latch_and_clear(self):
        """비상정지 latch: 활성 명령 거부, 중립은 받되 확인하지 않음. 해제 뒤 두 채널 모두 새 중립 필요"""
        p = self.peer()
        self.ready(p)
        self.assertAck(p.cmd(True, 0.2), True, "ok", "active")
        self.assertAck(p.arm(True), True, "ok", "active")
        self.assertAck(p.event("emergency_stop"), True, "ok", "emergency_stop", estop=True)
        self.assertAck(p.cmd(True, 0.2), False, "emergency_stop", "emergency_stop", estop=True)
        self.assertAck(p.arm(True), False, "emergency_stop", "emergency_stop", estop=True)
        self.assertAck(p.cmd(False), True, "ok", "emergency_stop", estop=True, note="latch 중 중립")
        self.assertAck(p.arm(False), True, "ok", "emergency_stop", estop=True, note="latch 중 팔 중립")
        self.assertAck(p.event("clear_emergency"), True, "ok", "awaiting_neutral", estop=False)
        self.assertAck(p.cmd(True, 0.2), False, "neutral_required", "awaiting_neutral", estop=False)
        self.assertAck(p.arm(True), False, "neutral_required", "awaiting_neutral", estop=False)
        self.assertAck(p.cmd(False), True, "ok", "idle")
        self.assertAck(p.arm(False), True, "ok", "idle")
        self.assertAck(p.cmd(True, 0.2), True, "ok", "active")
        self.assertAck(p.arm(True), True, "ok", "active")

    @NORMAL
    def test_02_non_owner_estop(self):
        """비소유자의 emergency_stop 도 latch. 해제는 소유자만(비소유자 clear → not_owner)"""
        a, b = self.peer(), self.peer()
        self.claim(a)
        self.assertAck(a.cmd(True, 0.2), True, "ok", "active")
        self.assertAck(b.event("emergency_stop"), True, "ok", estop=True, note="비소유자 비상정지")
        self.assertAck(a.cmd(True, 0.2), False, "emergency_stop", "emergency_stop", estop=True)
        self.assertAck(b.event("clear_emergency"), False, "not_owner", estop=True, note="비소유자 해제")
        self.assertAck(a.cmd(True, 0.2), False, "emergency_stop", estop=True, note="여전히 latch")
        self.assertAck(b.cmd(False), False, "not_owner", note="비상정지로 소유권을 얻지 않음")
        self.assertAck(a.event("clear_emergency"), True, "ok", estop=False)

    @NORMAL
    def test_03_estop_survives_owner_change(self):
        """latch 는 조종자가 바뀌어도 유지. 새 소유자가 해제할 수 있다"""
        a, b = self.peer(), self.peer()
        self.claim(a)
        self.assertAck(a.event("emergency_stop"), True, "ok", estop=True)
        self.assertAck(a.event("disconnect"), True, "ok", estop=True)
        self.assertAck(b.cmd(False), True, "ok", "emergency_stop", estop=True, note="새 소유자에게도 latch")
        self.assertAck(b.cmd(True, 0.2), False, "emergency_stop", estop=True)
        self.assertAck(b.event("clear_emergency"), True, "ok", estop=False)
        self.assertAck(b.cmd(False), True, "ok", "idle")
        self.assertAck(b.cmd(True, 0.2), True, "ok", "active")

    @NORMAL
    def test_04_repeated_estop(self):
        """비상정지 재전송(매번 새 번호)은 모두 수락"""
        p = self.peer()
        self.claim(p)
        for i in range(5):
            self.assertAck(p.event("emergency_stop"), True, "ok", estop=True, note="%d번째" % (i + 1))
        self.assertAck(p.event("clear_emergency"), True, "ok", estop=False)

    @NORMAL
    def test_05_retired_client_estop(self):
        """retire 된 UUID 의 emergency_stop 도 latch(유효 클라이언트의 비상정지는 항상 받는다)"""
        a, c = self.peer(), self.peer()
        self.claim(a)
        self.assertAck(a.event("disconnect"), True, "ok")
        self.claim(c)
        self.assertAck(a.event("emergency_stop"), True, "ok", estop=True)
        self.assertAck(c.cmd(True, 0.2), False, "emergency_stop", estop=True)
        self.assertAck(c.event("clear_emergency"), True, "ok", estop=False)


# ───────────────────────── T07 채널별 ACK state ─────────────────────────

class T07ChannelState(Base):
    @NORMAL
    def test_01_ack_state_per_channel(self):
        """ACK state 는 그 패킷의 채널 상태: command → 차체, arm_command → 팔 (사양 2.4)"""
        p = self.peer()
        self.claim(p)
        self.assertAck(p.cmd(True, 0.2), True, "ok", "active")
        self.assertAck(p.arm(True), False, "neutral_required", "awaiting_neutral", note="팔 채널 상태")
        self.assertAck(p.cmd(True, 0.2), True, "ok", "active", note="팔 거부가 차체에 영향 없음")
        self.assertAck(p.arm(False), True, "ok", "idle")
        self.assertAck(p.cmd(True, 0.2), True, "ok", "active")
        self.assertAck(p.arm(True), True, "ok", "active")
        self.assertAck(p.cmd(False), True, "ok", "idle")
        self.assertAck(p.arm(True), True, "ok", "active", note="차체 중립이 팔에 영향 없음")

    @NORMAL
    def test_02_event_ack_uses_base_state(self):
        """event ACK 의 state 는 차체 채널 상태"""
        p = self.peer()
        self.ready(p)
        self.assertAck(p.arm(True), True, "ok", "active")
        self.assertAck(p.cmd(False), True, "ok", "idle")
        ack = p.event("clear_emergency", seq=1)  # 순서 오류로 거부(상태는 안 바뀜)
        self.assertAck(ack, False, "out_of_order", "idle", note="event → 차체(idle), 팔(active) 아님")
        self.assertAck(p.cmd(True, 0.2), True, "ok", "active")
        self.assertAck(p.arm(False), True, "ok", "idle")
        self.assertAck(p.event("clear_emergency", seq=2), False, "out_of_order", "active", note="차체 active")

    @NORMAL
    def test_03_arm_reject_reports_arm_state(self):
        """팔 목표 거부 ACK 의 state 는 팔 채널 상태(차체가 active 여도 팔은 idle)"""
        p = self.peer()
        self.ready(p)
        self.assertAck(p.cmd(True, 0.2), True, "ok", "active")
        if CFG.arm_policy == "nearest":
            # nearest 는 범위 밖 목표를 보정해서 받으므로, 형식 오류(그리퍼 2.0)로 거부되는 팔 패킷으로 확인한다
            seq = p.next_seq()
            ack = p.request(raw_obj(replace(arm_items(p.cid, seq, True), "gripper", "2.0")), seq)
        else:
            outside = (CFG.workspace[1] + 0.05, 0.0, CFG.reachable[2], CFG.reachable[3])
            ack = p.arm(True, outside)
        self.assertEqual(ack["state"], "idle", "팔 채널은 idle 이어야 함: %r" % ack)
        self.assertFalse(ack["accepted"])


# ───────────────────────── T08 팔 목표 판정 ─────────────────────────

class T08Arm(Base):
    def setUp(self):
        super().setUp()
        w = CFG.workspace
        for name, pt in (("reachable", CFG.reachable), ("unreachable", CFG.unreachable)):
            if not (w[0] < pt[0] < w[1] and w[2] < pt[1] < w[3] and w[4] < pt[2] < w[5]):
                self.skipTest("시험점 --%s %s 가 --workspace 안에 있지 않음" % (name, pt))

    @NORMAL
    def test_01_workspace_limit(self):
        """작업 공간 상자 밖(6면 각각) → reject: workspace_limit / nearest: 상자 면으로 보정해 수락. 채널은 active 유지 (사양 2.2)"""
        p = self.peer()
        self.ready(p)
        self.assertAck(p.arm(True), True, "ok", "active")
        w, (x, y, z, pitch) = CFG.workspace, CFG.reachable
        faces = {"x>x_max": (w[1] + 0.05, y, z), "x<x_min": (w[0] - 0.05, y, z), "y>y_max": (x, w[3] + 0.05, z),
                 "y<y_min": (x, w[2] - 0.05, z), "z>z_max": (x, y, w[5] + 0.05), "z<z_min": (x, y, w[4] - 0.05)}
        for name, (xx, yy, zz) in faces.items():
            with self.subTest(face=name):
                if CFG.arm_policy == "nearest":
                    self.assertAck(p.arm(True, (xx, yy, zz, pitch)), True, "ok", "active", estop=False)
                else:
                    self.assertAck(p.arm(True, (xx, yy, zz, pitch)), False, "workspace_limit", "active", estop=False)
                self.assertAck(p.arm(True), True, "ok", "active", note="다시 정상 목표")

    @NORMAL
    def test_02_ik_unreachable(self):
        """작업 공간 안이지만 IK 해가 없는 목표 → reject: ik_unreachable / nearest: 가까운 자세로 수락. 채널은 active 유지"""
        p = self.peer()
        self.ready(p)
        self.assertAck(p.arm(True), True, "ok", "active")
        if CFG.arm_policy == "nearest":
            self.assertAck(p.arm(True, CFG.unreachable), True, "ok", "active")
        else:
            self.assertAck(p.arm(True, CFG.unreachable), False, "ik_unreachable", "active")
        self.assertAck(p.arm(True), True, "ok", "active")

    @NORMAL
    def test_03_gripper_and_pitch(self):
        """그리퍼 0→1·pitch 변경은 수락(도달 가능한 점)"""
        p = self.peer()
        self.ready(p)
        self.assertAck(p.arm(True, gripper=0.0), True, "ok", "active")
        self.assertAck(p.arm(True, gripper=1.0), True, "ok", "active")
        self.assertAck(p.arm(True, gripper=0.5), True, "ok", "active")

    @NORMAL
    @interpretation
    def test_04_box_boundary_inclusive(self):
        """작업 공간 경계값은 안쪽(C++ inside() 는 <=). x=x_max 는 workspace_limit 이 아니어야 함(IK 는 별개)"""
        p = self.peer()
        self.ready(p)
        x, y, z, pitch = CFG.reachable
        ack = p.arm(True, (CFG.workspace[1], y, z, pitch))
        self.assertIn(ack["reason"], ("ok", "ik_unreachable"), "경계값을 작업 공간 밖으로 판정함: %r" % ack)

    @NORMAL
    @interpretation
    def test_05_neutral_arm_ignores_workspace(self):
        """팔 중립(control_active:false)의 x/y/z 는 무시되므로 작업 공간 밖이어도 수락"""
        p = self.peer()
        self.ready(p)
        x, y, z, pitch = CFG.reachable
        self.assertAck(p.arm(False, (CFG.workspace[1] + 0.2, y, z, pitch)), True, "ok", "idle")

    @NORMAL
    @interpretation
    def test_06_state_rejection_precedes_workspace(self):
        """중립 미확인 상태의 범위 밖 목표 → neutral_required (상태 판정이 목표 판정보다 먼저)"""
        p = self.peer()
        self.claim(p)
        x, y, z, pitch = CFG.reachable
        self.assertAck(p.arm(True, (CFG.workspace[1] + 0.2, y, z, pitch)), False, "neutral_required",
                       "awaiting_neutral")


# ───────────────────────── T09 ACK 형식 ─────────────────────────

class T09AckFormat(Base):
    @ANY
    def test_01_fields_and_types(self):
        """ACK: 정확히 8필드, 값은 문자열·불리언·정수, 문자열은 [A-Za-z0-9_-] (모든 ACK 에 같은 검사 적용)"""
        p = self.peer()
        self.claim(p)
        obj = json.loads(p.last_raw.decode("utf-8"), object_pairs_hook=OrderedDict)
        self.assertEqual(len(obj), 8)
        for key, value in obj.items():
            self.assertIn(type(value), (str, bool, int, type(None)), "%s 값 타입 %s" % (key, type(value).__name__))
        self.assertEqual(obj["client_id"].lower(), p.cid)
        p.send_raw(b"not json")
        ack = p.recv_ack()
        self.assertIsNone(ack["client_id"])
        self.assertEqual(ack["sequence"], -1)

    @ANY
    def test_02_one_ack_per_packet(self):
        """패킷마다 ACK 정확히 1개(기다리지 않고 10개 연속 송신)"""
        p = self.peer()
        self.claim(p)
        seqs = [p.next_seq() for _ in range(10)]
        p.drain()
        for s in seqs:
            p.sock.sendto(raw_obj(cmd_items(p.cid, s)), p.target)
        got = []
        for _ in seqs:
            got.append(p.recv_ack()["sequence"])
        self.assertEqual(sorted(got), seqs)
        self.assertIsNone(p.expect_silence(0.2), "ACK 가 더 옴")


# ───────────────────────── T10 외부 조건 ─────────────────────────

class T10Conditions(Base):
    @require("control_missing", "udp_blocked")
    def test_01_gate_blocks_active(self):
        """제어기 상태 미수신/udp_allowed:false → 활성 명령 not_owner, 중립은 수락 (02 문서 수신기 규칙)"""
        p = self.peer()
        self.ready(p)
        self.assertAck(p.cmd(True, 0.2), False, "not_owner", note="차체 활성")
        self.assertAck(p.arm(True), False, "not_owner", note="팔 활성")
        self.assertAck(p.cmd(False), True, "ok", note="차체 중립")
        self.assertAck(p.arm(False), True, "ok", note="팔 중립")

    @require("fault")
    def test_02_fault(self):
        """제어기 fault → 중립은 수락(state fault), 활성 명령 fault"""
        p = self.peer()
        self.assertAck(p.cmd(False), True, "ok", "fault")
        self.assertAck(p.cmd(True, 0.2), False, "fault", "fault")
        self.assertAck(p.arm(False), True, "ok", "fault")
        self.assertAck(p.arm(True), False, "fault", "fault")

    @require("source_blocked")
    def test_03_source_blocked(self):
        """allowed_sources 밖의 주소: 제어·검색 모두 응답 없이 버림 (사양 1절)"""
        p = Peer()
        try:
            p.send_raw(raw_obj(cmd_items(p.cid, 1)))
            self.assertIsNone(p.expect_silence(0.5), "막힌 주소에 ACK 를 보냄")
        finally:
            p.close()
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.sendto(("ARMPI_DISCOVER/1|" + uuid.uuid4().hex).encode(), (CFG.ip, CFG.discovery_port))
            s.settimeout(0.5)
            with self.assertRaises(socket.timeout, msg="막힌 주소에 검색 응답"):
                s.recvfrom(1024)
        finally:
            s.close()

    @NORMAL
    def test_04_external_estop_via_rosbridge(self):
        """제어기 비상정지(/armpi/emergency_stop 서비스)를 세션이 따른다. /armpi/reset_estop 뒤 새 중립 필요"""
        if not CFG.rosbridge_port:
            self.skipTest("--rosbridge-port 를 주면 실행(대상의 rosbridge 필요)")
        rb = MiniRosbridge(CFG.ip, CFG.rosbridge_port)
        p = self.peer()
        try:
            self.ready(p)
            self.assertAck(p.cmd(True, 0.2), True, "ok", "active")
            res = rb.call("/armpi/emergency_stop")
            self.assertTrue(res.get("result") and (res.get("values") or {}).get("success"), "서비스 실패: %r" % res)
            ack = None
            for _ in range(20):
                ack = p.cmd(False)
                if ack["emergency_stop"]:
                    break
                time.sleep(0.05)
            self.assertTrue(ack["emergency_stop"], "1 s 안에 세션이 비상정지를 따르지 않음")
            self.assertAck(p.cmd(True, 0.2), False, "emergency_stop", estop=True)
            res = rb.call("/armpi/reset_estop")
            self.assertTrue(res.get("result"), "reset 실패: %r" % res)
            for _ in range(20):
                ack = p.cmd(True, 0.2)
                if not ack["emergency_stop"]:
                    break
                time.sleep(0.05)
            self.assertAck(ack, False, "neutral_required", "awaiting_neutral", estop=False, note="해제 직후 활성")
            self.assertAck(p.arm(True), False, "neutral_required", "awaiting_neutral", note="팔도 새 중립 필요")
            self.assertAck(p.cmd(False), True, "ok", "idle")
            self.assertAck(p.cmd(True, 0.2), True, "ok", "active")
        finally:
            rb.close()


# ───────────────────────── 실행 ─────────────────────────

class _Result(unittest.TextTestResult):
    def __init__(self, *a, **k):
        super().__init__(*a, **k)
        self.records = []
        self._t0 = {}

    def startTest(self, test):
        self._t0[test.id()] = time.monotonic()
        super().startTest(test)

    def _rec(self, test, status, detail=""):
        doc = (test._testMethodDoc or "").strip().splitlines()[0] if getattr(test, "_testMethodDoc", None) else ""
        self.records.append({"id": test.id().split(".", 1)[-1], "status": status, "doc": doc,
                             "detail": detail[-1500:], "seconds": round(time.monotonic() - self._t0.get(test.id(), time.monotonic()), 3)})

    def addSuccess(self, test):
        super().addSuccess(test)
        self._rec(test, "pass")

    def addFailure(self, test, err):
        super().addFailure(test, err)
        self._rec(test, "fail", self._exc_info_to_string(err, test))

    def addError(self, test, err):
        super().addError(test, err)
        self._rec(test, "error", self._exc_info_to_string(err, test))

    def addSkip(self, test, reason):
        super().addSkip(test, reason)
        self._rec(test, "skip", reason)

    def addSubTest(self, test, subtest, err):
        super().addSubTest(test, subtest, err)
        if err is not None:
            status = "fail" if issubclass(err[0], test.failureException) else "error"
            self.records.append({"id": subtest.id().split(".", 1)[-1], "status": status, "doc": "하위 시험",
                                 "detail": self._exc_info_to_string(err, test)[-1500:], "seconds": 0})


def preflight():
    """대상이 응답하는지, 비상정지 잔여물이 없는지 확인. 문제가 있으면 메시지."""
    if CFG.condition == "source_blocked":
        return None
    p = Peer()
    try:
        try:
            ack = p.cmd(False)
        except AssertionError as exc:
            return "대상 %s:%d 응답 없음 또는 ACK 형식 오류: %s" % (CFG.ip, CFG.port, exc)
        if ack["reason"] == "not_owner":
            time.sleep(CFG.lease + CFG.margin)
            ack = p.cmd(False)
            if ack["reason"] == "not_owner":
                return "다른 조종자가 계속 제어 중(not_owner). 조종기를 끄고 다시 실행"
        if ack["emergency_stop"]:
            p.try_event("clear_emergency")
            ack = p.cmd(False)
            if ack["emergency_stop"] and CFG.condition == "normal":
                return "대상이 비상정지 latch 상태(UDP 로 풀리지 않음: 서비스/물리 비상정지 확인)"
        return None
    finally:
        p.try_event("disconnect")
        p.close()


def _floats(text, n):
    vals = [float(v) for v in text.split(",")]
    if len(vals) != n:
        raise argparse.ArgumentTypeError("숫자 %d개" % n)
    return tuple(vals)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1", help="대상 주소(가짜 로봇 또는 C++ 수신기)")
    ap.add_argument("--port", type=int, default=5001, help="제어 UDP 포트")
    ap.add_argument("--discovery-port", type=int, default=5002, help="검색 UDP 포트")
    ap.add_argument("--rosbridge-port", type=int, default=0, help="주면 rosbridge 외부 비상정지 시험 실행(예: 9090)")
    ap.add_argument("--condition", default="normal",
                    choices=["normal", "control_missing", "udp_blocked", "fault", "source_blocked"])
    ap.add_argument("--interpretation", action="store_true", help="사양이 애매한 부분의 [해석] 시험도 실행")
    ap.add_argument("--base-watchdog", type=float, default=0.4)
    ap.add_argument("--arm-watchdog", type=float, default=0.4)
    ap.add_argument("--lease", type=float, default=1.0)
    ap.add_argument("--margin", type=float, default=0.15, help="시간 판정 여유(s)")
    ap.add_argument("--ack-wait", type=float, default=0.5, help="ACK 대기 시간(s)")
    ap.add_argument("--workspace", type=lambda t: _floats(t, 6), default=(0.05, 0.32, -0.25, 0.25, -0.08, 0.32),
                    help="x_min,x_max,y_min,y_max,z_min,z_max (대상 설정과 같게)")
    ap.add_argument("--reachable", type=lambda t: _floats(t, 4), default=(0.1047, 0.0, 0.0753, -1.0263),
                    help="IK 해가 있는 점 x,y,z,pitch (기본 자세의 FK, 기본 IK 치수 기준)")
    ap.add_argument("--unreachable", type=lambda t: _floats(t, 4), default=(0.28, 0.0, 0.30, 0.0),
                    help="작업 공간 안이지만 IK 해가 없는 점 x,y,z,pitch (기본 IK 치수 기준)")
    ap.add_argument("--arm-policy", choices=["nearest", "reject"], default="nearest",
                    help="대상의 arm.target_policy(로봇 armpi.yaml 기본 nearest, 규격 v3 원래 동작 reject)")
    ap.add_argument("--json-report", help="결과 JSON 저장 경로")
    ap.add_argument("--no-preflight", action="store_true")
    args, rest = ap.parse_known_args(argv)
    CFG.ip = socket.gethostbyname(args.host)
    CFG.host, CFG.port, CFG.discovery_port = args.host, args.port, args.discovery_port
    CFG.rosbridge_port = args.rosbridge_port
    CFG.condition, CFG.interpretation = args.condition, args.interpretation
    CFG.base_wd, CFG.arm_wd, CFG.lease, CFG.margin = args.base_watchdog, args.arm_watchdog, args.lease, args.margin
    CFG.ack_wait = args.ack_wait
    CFG.workspace, CFG.reachable, CFG.unreachable = args.workspace, args.reachable, args.unreachable
    CFG.arm_policy = args.arm_policy

    print("ArmPi Protocol v3 적합성 시험: 대상 %s:%d (검색 %d), 조건 %s, 팔 목표 처리 %s, 해석 시험 %s"
          % (CFG.ip, CFG.port, CFG.discovery_port, CFG.condition, CFG.arm_policy,
             "포함" if CFG.interpretation else "제외"))
    if not args.no_preflight:
        problem = preflight()
        if problem:
            print("사전 확인 실패: " + problem, file=sys.stderr)
            return 2
    started = time.time()
    prog = [sys.argv[0]] + rest
    runner = unittest.TextTestRunner(verbosity=2 if "-v" in rest else 1, resultclass=_Result, stream=sys.stdout)
    program = unittest.main(module=__name__, argv=prog, testRunner=runner, exit=False)
    res = program.result
    counts = {k: sum(1 for r in res.records if r["status"] == k) for k in ("pass", "fail", "error", "skip")}
    print("\n요약: 통과 %d / 실패 %d / 오류 %d / 건너뜀 %d (실행 %d, %.1f s)"
          % (counts["pass"], counts["fail"], counts["error"], counts["skip"], res.testsRun, time.time() - started))
    for r in res.records:
        if r["status"] in ("fail", "error"):
            print("  [%s] %s — %s" % (r["status"].upper(), r["id"], r["detail"].strip().splitlines()[-1]))
    if args.json_report:
        report = {"target": "%s:%d" % (CFG.ip, CFG.port), "discovery_port": CFG.discovery_port,
                  "condition": CFG.condition, "arm_policy": CFG.arm_policy, "interpretation": CFG.interpretation,
                  "timing": {"base_watchdog": CFG.base_wd, "arm_watchdog": CFG.arm_wd, "lease": CFG.lease,
                             "margin": CFG.margin},
                  "arm_points": {"workspace": CFG.workspace, "reachable": CFG.reachable, "unreachable": CFG.unreachable},
                  "started": time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime(started)),
                  "seconds": round(time.time() - started, 2), "tests_run": res.testsRun, "counts": counts,
                  "records": res.records}
        with open(args.json_report, "w", encoding="utf-8") as fp:
            json.dump(report, fp, ensure_ascii=False, indent=2)
    return 0 if res.wasSuccessful() else 1


if __name__ == "__main__":
    sys.exit(main())
