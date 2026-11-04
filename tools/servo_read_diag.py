#!/usr/bin/env python3
"""ArmPi Pro 버스 서보 읽기 전용 진단 도구 (로봇 안에서 실행).

목적: docs/10 S절. 제어 노드 없이 서보의 위치(POS_READ)·전압(VIN_READ)·온도(TEMP_READ)·ID(ID_READ)를 읽는다.

안전 규칙
- 읽기 명령 4개(ID_READ 14, TEMP_READ 26, VIN_READ 27, POS_READ 28)만 보낸다.
  이동·정지·토크·ID 변경 같은 쓰기 명령 코드는 이 파일에 아예 없다.
- 보내기 직전에 패킷을 다시 검사한다(길이 6, 매개변수 없음, 허용 명령, 방송 ID 254 금지).
  직렬 포트에 쓰는 곳은 _write_packet() 한 곳뿐이다.
- armpi.service 가 돌고 있거나 다른 프로세스가 포트를 잡고 있으면 실행하지 않는다.
  서비스를 멈추거나 다시 켜지 않는다(사람이 판단한다).

반이중 처리는 ros2_ws/src/armpi_hardware 의 HalfDuplexServoBus 와 같다.
  - 근거: armpi_hardware/src/devices.cpp, src/linux_serial_port.cpp, include/armpi_hardware/servo_protocol.hpp
  - 시작: RX_CON 출력 0, TX_CON 출력 1, 보내기 모드
  - 보내기 모드: TX_CON=1 다음 RX_CON=0 / 받기 모드: RX_CON=1 다음 TX_CON=0
  - 한 번 읽기: 보내기 모드 → pre_write_delay_us → 쓰기 → tcdrain → post_write_delay_us → 받기 모드 → 입력 비우기
               → read_turnaround_delay_us → 4 B(read_timeout_ms 안) → 머리 0x55 0x55·길이 확인
               → 나머지 LEN−1 B(read_timeout_ms 안) → 보내기 모드 → 해석
  - 실패하면 같은 순서로 다시(모두 read_attempts 번)
  - 직렬: termios raw 8N1, 흐름 제어 없음, VMIN=0 VTIME=0, 115200

핀 번호(RX_CON·TX_CON)와 타이밍은 코드에 기본값이 없다. 하드웨어 프로필(armpi_bringup 의 hardware_profile.yaml,
본보기 hardware_profile.template.yaml)에서 읽거나 명령행으로 준다. 없으면 실행하지 않는다(종료 코드 2).
프로필 찾는 순서: --hardware-profile → 환경 변수 ARMPI_HARDWARE_PROFILE → AMENT_PREFIX_PATH 의
share/armpi_bringup/config/hardware_profile.yaml → /opt/armpi/ros2_ws/install/share/armpi_bringup/config/

이미지에 있는 것만 쓴다: Python 3 표준 모듈 + libgpiod.so.2(libgpiod 1.6, gpiod 패키지 의존성).
"""

from __future__ import annotations

import argparse
import ctypes
import ctypes.util
import errno
import fcntl
import json
import os
import select
import shutil
import subprocess
import sys
import termios
import time

TOOL_VERSION = "1.1 (2026-11-04)"

HEADER = 0x55
BROADCAST_ID = 254

# 읽기 명령만. 값은 servo_protocol.hpp 의 kIdRead / kTempRead / kVinRead / kPosRead 와 같다.
READ_COMMANDS = {
    "id": 14,
    "temp": 26,
    "vin": 27,
    "pos": 28,
}
ALLOWED_CMDS = frozenset(READ_COMMANDS.values())
CMD_NAMES = {v: k for k, v in READ_COMMANDS.items()}
# 응답 매개변수 길이(바이트). servo_protocol.cpp 의 decode_* 와 같다.
RESPONSE_PARAM_LEN = {14: 1, 26: 1, 27: 2, 28: 2}

# 반이중 타이밍(하드웨어 프로필 servo.*). 값은 프로필이나 명령행에서만 온다.
TIMING_KEYS = ("pre_write_delay_us", "post_write_delay_us", "read_turnaround_delay_us", "read_timeout_ms", "read_attempts")

# 관절 표시(참고)에 쓰는 프로필 값: 관절 ↔ 서보 배정, 방향, 그리퍼 열림·닫힘
JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "r_joint(그리퍼)"]
SERVO_CENTER = 500          # Hiwonder 버스 서보 규격: 0..1000 ↔ 0..240°, 중앙 500
SERVO_DEG_PER_UNIT = 0.24

PROFILE_NAME = os.path.join("share", "armpi_bringup", "config", "hardware_profile.yaml")
PROFILE_FALLBACK = os.path.join("/opt/armpi/ros2_ws/install", PROFILE_NAME)  # 로봇 설치(--merge-install)

EXIT_OK, EXIT_READ_FAIL, EXIT_USAGE, EXIT_REFUSED, EXIT_SETUP = 0, 1, 2, 4, 5


class ReadOnlyViolation(Exception):
    """읽기 전용 규칙 위반(보내지 않고 멈춘다)."""


# ───────────── 패킷 ─────────────

def checksum(body: bytes) -> int:
    # 체크섬 = 255 − (sum(ID, LEN, CMD, P…) mod 256)
    return 255 - (sum(body) % 256)


def encode_read(servo_id: int, cmd: int) -> bytes:
    """읽기 요청 0x55 0x55 ID 3 CMD CHK. 허용 명령 밖이면 예외."""
    if cmd not in ALLOWED_CMDS:
        raise ReadOnlyViolation(f"허용되지 않은 명령 코드 {cmd} (읽기 명령 {sorted(ALLOWED_CMDS)}만 허용)")
    if not isinstance(servo_id, int) or not 0 <= servo_id <= 253:
        raise ReadOnlyViolation(f"서보 ID {servo_id!r} 는 0..253 이어야 함(방송 ID 254 금지)")
    body = bytes([servo_id, 3, cmd])
    return bytes([HEADER, HEADER]) + body + bytes([checksum(body)])


def assert_read_only_packet(pkt: bytes) -> None:
    """보내기 직전 최종 검사."""
    if len(pkt) != 6:
        raise ReadOnlyViolation(f"패킷 길이 {len(pkt)} != 6 (매개변수 있는 패킷 금지)")
    if pkt[0] != HEADER or pkt[1] != HEADER:
        raise ReadOnlyViolation("패킷 머리 오류")
    if pkt[2] == BROADCAST_ID or pkt[2] > 253:
        raise ReadOnlyViolation(f"ID {pkt[2]} 금지")
    if pkt[3] != 3:
        raise ReadOnlyViolation(f"LEN {pkt[3]} != 3")
    if pkt[4] not in ALLOWED_CMDS:
        raise ReadOnlyViolation(f"명령 {pkt[4]} 는 읽기 명령이 아님")
    if checksum(pkt[2:5]) != pkt[5]:
        raise ReadOnlyViolation("체크섬 오류")


def parse_packet(b: bytes):
    """servo_proto::parse_packet 과 같은 판정. (오류 문자열 | None, (id, cmd, params) | None)"""
    if len(b) < 6:
        return "too_short", None
    if b[0] != HEADER or b[1] != HEADER:
        return "bad_header", None
    ln = b[3]
    if ln < 3:
        return "bad_length", None
    if len(b) != ln + 3:
        return ("too_short" if len(b) < ln + 3 else "bad_length"), None
    if checksum(b[2:-1]) != b[-1]:
        return "bad_checksum", None
    return None, (b[2], b[4], bytes(b[5:-1]))


def decode_value(cmd: int, params: bytes):
    if len(params) != RESPONSE_PARAM_LEN[cmd]:
        return None
    if cmd == READ_COMMANDS["pos"]:
        raw = params[0] | (params[1] << 8)
        return raw - 0x10000 if raw & 0x8000 else raw  # signed int16 (servo_protocol.cpp)
    if cmd == READ_COMMANDS["vin"]:
        return params[0] | (params[1] << 8)  # mV
    return params[0]  # temp °C, id


# ───────────── 직렬 포트 (linux_serial_port.cpp 와 같은 설정) ─────────────

class SerialPort:
    def __init__(self, path: str, baud: int):
        self.path = path
        speed = {9600: termios.B9600, 19200: termios.B19200, 38400: termios.B38400,
                 57600: termios.B57600, 115200: termios.B115200}.get(baud)
        if speed is None:
            raise OSError(errno.EINVAL, f"지원하지 않는 baud {baud}")
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY | getattr(os, "O_CLOEXEC", 0))
        try:
            iflag, oflag, cflag, lflag, _isp, _osp, cc = termios.tcgetattr(self.fd)
            # cfmakeraw
            iflag &= ~(termios.IGNBRK | termios.BRKINT | termios.PARMRK | termios.ISTRIP
                       | termios.INLCR | termios.IGNCR | termios.ICRNL | termios.IXON)
            oflag &= ~termios.OPOST
            lflag &= ~(termios.ECHO | termios.ECHONL | termios.ICANON | termios.ISIG | termios.IEXTEN)
            cflag &= ~(termios.CSIZE | termios.PARENB)
            cflag |= termios.CS8
            # armpi_hardware 와 같은 설정: 1 정지 비트, 패리티 없음, 하드웨어 흐름 제어 없음, CLOCAL|CREAD, 소프트웨어 흐름 제어 없음
            cflag &= ~(termios.CSTOPB | termios.PARENB | getattr(termios, "CRTSCTS", 0) | termios.CSIZE)
            cflag |= termios.CS8 | termios.CLOCAL | termios.CREAD
            iflag &= ~(termios.IXON | termios.IXOFF | getattr(termios, "IXANY", 0))
            cc = list(cc)
            cc[termios.VMIN] = 0
            cc[termios.VTIME] = 0
            termios.tcsetattr(self.fd, termios.TCSANOW, [iflag, oflag, cflag, lflag, speed, speed, cc])
            termios.tcflush(self.fd, termios.TCIOFLUSH)
            # 이 도구가 도는 동안 다른 일반 사용자 프로세스가 포트를 열지 못하게 한다(root 는 막지 못함).
            tiocexcl = getattr(termios, "TIOCEXCL", None)
            self.exclusive = False
            if tiocexcl is not None:
                try:
                    fcntl.ioctl(self.fd, tiocexcl)
                    self.exclusive = True
                except OSError:
                    pass
        except Exception:
            os.close(self.fd)
            raise

    def write_all(self, data: bytes) -> None:
        off = 0
        while off < len(data):
            try:
                n = os.write(self.fd, data[off:])
            except InterruptedError:
                continue
            except BlockingIOError:
                select.select([], [self.fd], [], 0.01)
                continue
            off += n

    def drain(self) -> None:
        termios.tcdrain(self.fd)

    def flush_input(self) -> None:
        termios.tcflush(self.fd, termios.TCIFLUSH)

    def read_exact(self, n: int, timeout_ms: int) -> bytes:
        """n 바이트를 timeout_ms 안에 읽는다. 모자라면 TimeoutError(받은 바이트 포함)."""
        deadline = time.monotonic() + timeout_ms / 1000.0
        out = bytearray()
        while len(out) < n:
            left = deadline - time.monotonic()
            if left < 0:
                break
            r, _, _ = select.select([self.fd], [], [], left)
            if not r:
                break
            try:
                chunk = os.read(self.fd, min(n - len(out), 64))
            except (InterruptedError, BlockingIOError):
                continue
            if not chunk:
                continue
            out += chunk
        if len(out) < n:
            err = TimeoutError(f"응답 시간 초과({len(out)}/{n} B)")
            err.partial = bytes(out)
            raise err
        return bytes(out)

    def close(self) -> None:
        if self.fd >= 0:
            try:
                termios.tcflush(self.fd, termios.TCIOFLUSH)
            except termios.error:
                pass
            # 독점 모드를 풀고 닫는다. pty 처럼 tty 가 남아 있는 장치에서는 풀지 않으면
            # 다음 열기(제어 노드 포함)가 EBUSY 로 막힌다(2026-11-03 Linux 자체 시험에서 확인).
            tiocnxcl = getattr(termios, "TIOCNXCL", None)
            if self.exclusive and tiocnxcl is not None:
                try:
                    fcntl.ioctl(self.fd, tiocnxcl)
                except OSError:
                    pass
            os.close(self.fd)
            self.fd = -1


# ───────────── GPIO (libgpiod 1.6 C API, ctypes) ─────────────

class LibgpiodGpio:
    """gpiod_chip_open_lookup / gpiod_line_request_output / gpiod_line_set_value (libgpiod v1)."""

    CONSUMER = b"servo_read_diag"

    def __init__(self, chip_name: str):
        libname = ctypes.util.find_library("gpiod") or "libgpiod.so.2"
        try:
            self.lib = ctypes.CDLL(libname, use_errno=True)
        except OSError as e:
            raise OSError(errno.ENOENT, f"libgpiod 라이브러리({libname})를 열지 못함: {e}") from e
        L = self.lib
        L.gpiod_chip_open_lookup.restype = ctypes.c_void_p
        L.gpiod_chip_open_lookup.argtypes = [ctypes.c_char_p]
        L.gpiod_chip_get_line.restype = ctypes.c_void_p
        L.gpiod_chip_get_line.argtypes = [ctypes.c_void_p, ctypes.c_uint]
        L.gpiod_line_request_output.restype = ctypes.c_int
        L.gpiod_line_request_output.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int]
        L.gpiod_line_set_value.restype = ctypes.c_int
        L.gpiod_line_set_value.argtypes = [ctypes.c_void_p, ctypes.c_int]
        L.gpiod_line_release.restype = None
        L.gpiod_line_release.argtypes = [ctypes.c_void_p]
        L.gpiod_chip_close.restype = None
        L.gpiod_chip_close.argtypes = [ctypes.c_void_p]
        self.chip = L.gpiod_chip_open_lookup(chip_name.encode())
        if not self.chip:
            e = ctypes.get_errno()
            raise OSError(e, f"GPIO 칩 {chip_name} 열기 실패: {os.strerror(e)}")
        self.lines = {}

    def request_output(self, bcm: int, initial: int) -> None:
        line = self.lib.gpiod_chip_get_line(self.chip, bcm)
        if not line:
            e = ctypes.get_errno()
            raise OSError(e, f"GPIO{bcm} 찾기 실패: {os.strerror(e)}")
        if self.lib.gpiod_line_request_output(line, self.CONSUMER, 1 if initial else 0) != 0:
            e = ctypes.get_errno()
            hint = " (다른 프로세스가 사용 중: armpi.service 등)" if e == errno.EBUSY else ""
            raise OSError(e, f"GPIO{bcm} 출력 요청 실패: {os.strerror(e)}{hint}")
        self.lines[bcm] = line

    def write(self, bcm: int, value: int) -> None:
        if self.lib.gpiod_line_set_value(self.lines[bcm], 1 if value else 0) != 0:
            e = ctypes.get_errno()
            raise OSError(e, f"GPIO{bcm} 쓰기 실패: {os.strerror(e)}")

    def close(self) -> None:
        for line in self.lines.values():
            self.lib.gpiod_line_release(line)
        self.lines.clear()
        if self.chip:
            self.lib.gpiod_chip_close(self.chip)
            self.chip = None


class NoGpio:
    """--no-gpio: 방향 핀을 건드리지 않는다(가짜 서보·USB 어댑터 시험용)."""

    def request_output(self, bcm: int, initial: int) -> None:
        pass

    def write(self, bcm: int, value: int) -> None:
        pass

    def close(self) -> None:
        pass


# ───────────── 반이중 버스 (HalfDuplexServoBus 와 같은 순서) ─────────────

class ServoReader:
    def __init__(self, port: SerialPort, gpio, rx_con: int, tx_con: int, timing: dict, sleep_us=None):
        self.port = port
        self.gpio = gpio
        self.rx_con = rx_con
        self.tx_con = tx_con
        self.t = dict(timing)
        self.sleep_us = sleep_us or (lambda us: time.sleep(us / 1e6) if us > 0 else None)
        self.sent_packets = []  # 실제로 보낸 패킷(기록용)

    def init(self) -> None:
        # 초기화: RX_CON 출력 0, TX_CON 출력 1, 이어서 보내기 모드
        self.gpio.request_output(self.rx_con, 0)
        self.gpio.request_output(self.tx_con, 1)
        self.set_tx()

    def set_tx(self) -> None:
        self.gpio.write(self.tx_con, 1)
        self.gpio.write(self.rx_con, 0)

    def set_rx(self) -> None:
        self.gpio.write(self.rx_con, 1)
        self.gpio.write(self.tx_con, 0)

    def _write_packet(self, pkt: bytes) -> None:
        assert_read_only_packet(pkt)  # 최종 검사: 통과하지 못하면 아무것도 보내지 않는다
        self.set_tx()
        self.sleep_us(self.t["pre_write_delay_us"])
        self.port.write_all(pkt)
        if self.t["drain_before_turnaround"]:
            self.port.drain()
        self.sleep_us(self.t["post_write_delay_us"])
        self.sent_packets.append(pkt)

    def read(self, servo_id: int, cmd: int) -> dict:
        pkt = encode_read(servo_id, cmd)  # 허용 명령·ID 검사
        attempts = max(1, int(self.t["read_attempts"]))
        tries = []
        t0 = time.monotonic()
        for _ in range(attempts):
            data = b""
            err = None
            try:
                self._write_packet(pkt)
                self.set_rx()
                try:
                    self.port.flush_input()
                    self.sleep_us(self.t["read_turnaround_delay_us"])
                    data = self.port.read_exact(4, self.t["read_timeout_ms"])
                    if not (data[0] == HEADER and data[1] == HEADER):
                        err = "응답 머리 오류"
                    elif data[3] < 3:
                        err = "응답 길이 오류"
                    else:
                        data += self.port.read_exact(data[3] - 1, self.t["read_timeout_ms"])
                except TimeoutError as e:
                    data += getattr(e, "partial", b"")
                    err = str(e)
                finally:
                    self.set_tx()  # 끝나면 항상 보내기 모드로
            except ReadOnlyViolation:
                raise
            except (OSError, termios.error) as e:
                err = f"입출력 오류: {e}"
                try:
                    self.set_tx()
                except OSError:
                    pass
            if err is None:
                perr, parsed = parse_packet(data)
                if perr is None:
                    rid, rcmd, params = parsed
                    if rid != servo_id:
                        perr = "unexpected_id"
                    elif rcmd != cmd:
                        perr = "unexpected_cmd"
                if perr is not None:
                    err = f"응답 해석 실패: {perr}"
            tries.append({"error": err, "rx_hex": data.hex()})
            if err is None:
                value = decode_value(cmd, params)
                return {"ok": value is not None, "value": value,
                        "error": None if value is not None else f"값 길이 오류({len(params)} B)",
                        "attempts": len(tries), "tries": tries,
                        "elapsed_ms": round((time.monotonic() - t0) * 1000, 2)}
        return {"ok": False, "value": None, "error": tries[-1]["error"], "attempts": len(tries),
                "tries": tries, "elapsed_ms": round((time.monotonic() - t0) * 1000, 2)}


# ───────────── 실행 전 점검: 다른 사용자가 버스를 쓰는지 ─────────────

def preflight(port_path: str) -> tuple:
    """(거부 이유 목록, 참고 메시지 목록). 상태를 바꾸는 명령은 쓰지 않는다."""
    refuse, notes = [], []
    systemctl = shutil.which("systemctl")
    if systemctl:
        try:
            r = subprocess.run([systemctl, "is-active", "armpi.service"], capture_output=True, text=True, timeout=5)
            state = (r.stdout or "").strip().splitlines()[0] if r.stdout.strip() else "(빈 출력)"
            notes.append(f"systemctl is-active armpi.service = {state}")
            if state in ("active", "activating", "reloading", "deactivating", "refreshing"):
                refuse.append(
                    f"armpi.service 가 '{state}' 상태다. 제어 노드가 같은 서보 버스를 쓰므로 함께 쓰면 안 된다. "
                    "이 도구는 서비스를 멈추지 않는다. 담당자가 판단해 서비스를 멈춘 뒤(docs/10 S절) 다시 실행한다.")
        except (OSError, subprocess.TimeoutExpired) as e:
            notes.append(f"systemctl 확인 실패: {e}")
    else:
        notes.append("systemctl 없음: 서비스 상태 확인 생략(로봇 밖 시험 환경)")

    if os.path.isdir("/proc/self/fd"):
        real = os.path.realpath(port_path)
        me = os.getpid()
        holders, unreadable = [], 0
        for name in os.listdir("/proc"):
            if not name.isdigit() or int(name) == me:
                continue
            comm = ""
            try:
                with open(f"/proc/{name}/comm") as f:
                    comm = f.read().strip()
            except OSError:
                pass
            if comm.startswith("armpi_control"):
                holders.append(f"pid {name} ({comm}: 제어 노드 실행 중)")
                continue
            try:
                fds = os.listdir(f"/proc/{name}/fd")
            except PermissionError:
                unreadable += 1
                continue
            except OSError:
                continue
            for fd in fds:
                try:
                    if os.readlink(f"/proc/{name}/fd/{fd}") == real:
                        holders.append(f"pid {name} ({comm or '?'})가 {real} 를 열고 있음")
                        break
                except OSError:
                    continue
        if holders:
            refuse.append("다른 프로세스가 서보 포트를 쓰고 있다: " + "; ".join(holders))
        if unreadable:
            notes.append(f"권한이 없어 열린 파일을 확인하지 못한 프로세스 {unreadable}개(다른 사용자 소유)")
    else:
        notes.append("/proc 없음: 포트 점유 확인 생략(Linux 가 아님)")
    return refuse, notes


# ───────────── 하드웨어 프로필 ─────────────

def load_profile(path: str) -> dict:
    """ROS 2 매개변수 YAML 에서 '키: 값' 줄만 읽는다(표준 라이브러리만). 값은 숫자·true/false·[숫자, ...].
    null·~·빈 값은 None 으로 둔다(비어 있음)."""
    out = {}
    with open(path, "r", encoding="utf-8") as fp:
        for line in fp:
            line = line.split("#", 1)[0].strip()
            if ":" not in line or line.endswith(":"):
                continue
            key, val = (x.strip() for x in line.split(":", 1))
            if not key or key in ("ros__parameters",) or key.startswith(("/", "-")):
                continue
            if val in ("", "null", "~"):
                out[key] = None
                continue
            try:
                out[key] = json.loads(val)
            except ValueError:
                out[key] = val
    return out


def find_profile(explicit):
    if explicit:
        return explicit
    env = os.environ.get("ARMPI_HARDWARE_PROFILE")
    if env:
        return env
    for prefix in filter(None, os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep)):
        cand = os.path.join(prefix, PROFILE_NAME)
        if os.path.isfile(cand):
            return cand
    return PROFILE_FALLBACK if os.path.isfile(PROFILE_FALLBACK) else None


def _int_or_none(v):
    return v if isinstance(v, int) and not isinstance(v, bool) else None


def joint_map(prof: dict):
    """(servo_ids, directions, (open, close)) 또는 값이 없으면 None"""
    ids, dirs = prof.get("arm.servo_ids"), prof.get("arm.joint_directions")
    if not (isinstance(ids, list) and isinstance(dirs, list) and len(ids) == 6 and len(dirs) == 6):
        return None
    if sorted(ids) != [1, 2, 3, 4, 5, 6] or any(d not in (1, -1) for d in dirs):
        return None
    grip = (_int_or_none(prof.get("arm.gripper.open_value")), _int_or_none(prof.get("arm.gripper.close_value")))
    return ids, dirs, grip if None not in grip and grip[0] != grip[1] else None


# ───────────── 표시 ─────────────

def joint_info(servo_id: int, pos, jmap=None):
    """서보값을 관절각(참고, 오프셋 없음)으로. 프로필에 관절 배정이 없으면 빈 문자열."""
    if jmap is None or pos is None or servo_id not in jmap[0]:
        return ""
    ids, dirs, grip = jmap
    i = ids.index(servo_id)
    if i == 5:
        if grip is None:
            return JOINT_NAMES[i]
        g = (pos - grip[0]) / (grip[1] - grip[0])
        return f"{JOINT_NAMES[i]} 닫힘 비율 {max(0.0, min(1.0, g)):.2f}(프로필 열림·닫힘 기준)"
    deg = dirs[i] * (pos - SERVO_CENTER) * SERVO_DEG_PER_UNIT
    return f"{JOINT_NAMES[i]} {deg:+.1f}°"


def parse_ids(text: str):
    ids = []
    for part in text.split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part:
            a, b = part.split("-", 1)
            ids.extend(range(int(a), int(b) + 1))
        else:
            ids.append(int(part))
    for i in ids:
        if not 0 <= i <= 253:
            raise ValueError(f"ID {i} 는 0..253 이어야 한다(254 방송 금지)")
    return ids


def build_arg_parser():
    p = argparse.ArgumentParser(
        description="ArmPi Pro 버스 서보 읽기 전용 진단(POS/VIN/TEMP/ID 읽기만). docs/10 S절.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p.add_argument("--port", default="/dev/ttyS0", help="서보 직렬 포트(armpi.yaml hardware.servo_port)")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--ids", default="1-6", help="읽을 서보 ID. 예: 1-6 또는 1,3,5")
    p.add_argument("--read", default="pos,vin,temp", help="읽을 항목: pos,vin,temp,id 중 쉼표로")
    p.add_argument("--repeat", type=int, default=1, help="전체를 몇 번 반복할지")
    p.add_argument("--interval", type=float, default=0.5, help="반복 사이 간격(초)")
    p.add_argument("--gpio-chip", default="gpiochip0")
    p.add_argument("--hardware-profile", metavar="YAML",
                   help="하드웨어 프로필(핀·타이밍·관절 배정). 없으면 ARMPI_HARDWARE_PROFILE·설치 위치에서 찾는다")
    p.add_argument("--rx-con", type=int, help="RX_CON BCM 번호(프로필 hardware.servo_rx_con_bcm 대신)")
    p.add_argument("--tx-con", type=int, help="TX_CON BCM 번호(프로필 hardware.servo_tx_con_bcm 대신)")
    p.add_argument("--no-gpio", action="store_true", help="방향 핀을 쓰지 않음(가짜 서보·USB 어댑터 시험용)")
    p.add_argument("--pre-write-us", type=int, help="프로필 servo.pre_write_delay_us 대신")
    p.add_argument("--post-write-us", type=int, help="프로필 servo.post_write_delay_us 대신")
    p.add_argument("--no-drain", action="store_true", help="쓰기 뒤 tcdrain 생략(servo.drain_before_turnaround: false)")
    p.add_argument("--turnaround-us", type=int, help="프로필 servo.read_turnaround_delay_us 대신")
    p.add_argument("--timeout-ms", type=int, help="프로필 servo.read_timeout_ms 대신")
    p.add_argument("--attempts", type=int, help="프로필 servo.read_attempts 대신")
    p.add_argument("--json", metavar="PATH", help="결과를 JSON 파일로도 저장")
    p.add_argument("--skip-preflight", action="store_true",
                   help="서비스·포트 점유 확인 생략(가짜 서보 시험 전용. 로봇에서 쓰지 말 것)")
    return p


def main(argv=None) -> int:
    args = build_arg_parser().parse_args(argv)
    try:
        ids = parse_ids(args.ids)
        what = [w.strip() for w in args.read.split(",") if w.strip()]
        for w in what:
            if w not in READ_COMMANDS:
                raise ValueError(f"알 수 없는 항목 '{w}' (pos, vin, temp, id 만 가능)")
    except ValueError as e:
        print(f"인자 오류: {e}", file=sys.stderr)
        return EXIT_USAGE

    prof_path = find_profile(args.hardware_profile)
    prof = {}
    if prof_path:
        try:
            prof = load_profile(prof_path)
        except OSError as e:
            print(f"하드웨어 프로필을 읽지 못함: {prof_path}: {e}", file=sys.stderr)
            return EXIT_USAGE
    given = {"pre_write_delay_us": args.pre_write_us, "post_write_delay_us": args.post_write_us,
             "read_turnaround_delay_us": args.turnaround_us, "read_timeout_ms": args.timeout_ms,
             "read_attempts": args.attempts}
    timing = {"drain_before_turnaround": not args.no_drain}
    missing = []
    for k in TIMING_KEYS:
        v = given[k] if given[k] is not None else _int_or_none(prof.get("servo." + k))
        if v is None or v < 0 or (k in ("read_timeout_ms", "read_attempts") and v < 1):
            missing.append("servo." + k)
        timing[k] = v
    rx = args.rx_con if args.rx_con is not None else _int_or_none(prof.get("hardware.servo_rx_con_bcm"))
    tx = args.tx_con if args.tx_con is not None else _int_or_none(prof.get("hardware.servo_tx_con_bcm"))
    if not args.no_gpio:
        if rx is None or rx < 0:
            missing.append("hardware.servo_rx_con_bcm(--rx-con)")
        if tx is None or tx < 0:
            missing.append("hardware.servo_tx_con_bcm(--tx-con)")
    if missing:
        where = prof_path or "찾지 못함"
        print(f"하드웨어 프로필 값 없음(프로필: {where}): {', '.join(missing)}. "
              "--hardware-profile 이나 명령행 인자로 주세요. 아무것도 보내지 않았다.", file=sys.stderr)
        return EXIT_USAGE
    jmap = joint_map(prof)
    report = {"tool": "servo_read_diag.py", "version": TOOL_VERSION,
              "started": time.strftime("%Y-%m-%dT%H:%M:%S%z"), "port": args.port, "baud": args.baud,
              "hardware_profile": prof_path,
              "gpio": None if args.no_gpio else {"chip": args.gpio_chip, "rx_con": rx, "tx_con": tx},
              "timing": timing, "ids": ids, "read": what, "rounds": []}

    print(f"servo_read_diag {TOOL_VERSION}: 읽기 명령만 보낸다(팔을 움직이지 않음). 포트 {args.port}, ID {ids}")
    if args.skip_preflight:
        print("주의: 실행 전 점검을 건너뜀(--skip-preflight)")
        report["preflight"] = "skipped"
    else:
        refuse, notes = preflight(args.port)
        for n in notes:
            print(f"  점검: {n}")
        report["preflight"] = {"refuse": refuse, "notes": notes}
        if refuse:
            for r in refuse:
                print(f"실행 거부: {r}", file=sys.stderr)
            _save_json(args.json, report)
            return EXIT_REFUSED

    port = gpio = None
    try:
        try:
            port = SerialPort(args.port, args.baud)
        except OSError as e:
            print(f"직렬 포트 열기 실패: {e}", file=sys.stderr)
            report["setup_error"] = str(e)
            _save_json(args.json, report)
            return EXIT_SETUP
        try:
            gpio = NoGpio() if args.no_gpio else LibgpiodGpio(args.gpio_chip)
            if args.no_gpio:
                print("주의: --no-gpio. 방향 핀을 바꾸지 않는다. 실제 로봇 버스에서는 응답을 받을 수 없다.")
            reader = ServoReader(port, gpio, rx, tx, timing)
            reader.init()
        except OSError as e:
            print(f"GPIO 준비 실패: {e}", file=sys.stderr)
            report["setup_error"] = str(e)
            _save_json(args.json, report)
            return EXIT_SETUP

        failures = 0
        for rnd in range(max(1, args.repeat)):
            rows = []
            for sid in ids:
                row = {"id": sid}
                for w in what:
                    res = reader.read(sid, READ_COMMANDS[w])
                    row[w] = res
                    if not res["ok"]:
                        failures += 1
                rows.append(row)
            report["rounds"].append({"round": rnd + 1, "time": time.strftime("%H:%M:%S"), "rows": rows})
            _print_round(rnd + 1, rows, what, jmap)
            if rnd + 1 < args.repeat:
                time.sleep(max(0.0, args.interval))
        report["failures"] = failures
        report["sent_packets"] = len(reader.sent_packets)
        report["sent_commands"] = sorted({p[4] for p in reader.sent_packets})
        print(f"보낸 패킷 {len(reader.sent_packets)}개, 명령 코드 {report['sent_commands']}(모두 읽기), 실패 {failures}건")
        _save_json(args.json, report)
        return EXIT_OK if failures == 0 else EXIT_READ_FAIL
    finally:
        if gpio is not None:
            try:
                gpio.close()
            except Exception:
                pass
        if port is not None:
            port.close()


def _print_round(rnd: int, rows, what, jmap=None):
    print(f"\n[{rnd}회차] {time.strftime('%H:%M:%S')}")
    head = ["ID"] + [{"pos": "위치(0-1000)", "vin": "전압(mV)", "temp": "온도(°C)", "id": "ID응답"}[w] for w in what]
    print("  " + " | ".join(head) + " | 관절(참고, 오프셋 없음)")
    for row in rows:
        cells = [str(row["id"])]
        for w in what:
            r = row[w]
            if r["ok"]:
                cells.append(str(r["value"]) + (f" (재시도 {r['attempts'] - 1})" if r["attempts"] > 1 else ""))
            else:
                cells.append(f"실패: {r['error']}")
        pos = row.get("pos", {}).get("value") if "pos" in row else None
        print("  " + " | ".join(cells) + " | " + joint_info(row["id"], pos, jmap))


def _save_json(path, report):
    if not path:
        return
    with open(path, "w", encoding="utf-8") as f:
        json.dump(report, f, ensure_ascii=False, indent=1)
    print(f"JSON 저장: {path}")


if __name__ == "__main__":
    sys.exit(main())
