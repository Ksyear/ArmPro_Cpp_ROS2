#!/usr/bin/env python3
"""servo_read_diag.py 자체 시험: pty 위의 가짜 서보로 확인한다(로봇·실제 서보 없음 = 모의 시험).

가짜 서보 규칙
- 요청이 선에 다 나간 시각(도구가 쓰기를 마친 시각 + 6 B 전송 시간 0.52 ms) 뒤 정해진 지연(기본 1.5 ms)에 응답한다.
- 그 시각에 가짜 GPIO 가 받기 모드(RX_CON=1, TX_CON=0)가 아니었으면 응답은 버스에서 사라진다(반이중 흉내).
  판정은 시각이 찍힌 GPIO 기록으로 한다. Linux pty 는 데이터를 커널 작업 큐로 늦게 넘기므로
  가짜 서보 스레드가 요청을 읽은 시각을 쓰면 결과가 흔들린다(2026-11-03 컨테이너 시험에서 확인).
- 받은 모든 바이트를 기록해 읽기 요청 외의 패킷이 있었는지 확인한다.

사용: python3 tools/servo_read_diag_selftest.py --log-dir work/logs/servo_diag_selftest_<시각>/mac
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import platform
import select
import stat
import subprocess
import sys
import tempfile
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import servo_read_diag as D  # noqa: E402

TOOL = os.path.join(HERE, "servo_read_diag.py")

POS = {1: 75, 2: -5, 3: 512, 4: 488, 5: 640, 6: 500}

# 시험용 하드웨어 프로필 값(실제 로봇 값 아님. armpi_bringup config/test_hardware_profile.yaml 과 같음)
RX, TX = 17, 22
TEST_TIMING = {
    "pre_write_delay_us": 400,
    "post_write_delay_us": 300,
    "drain_before_turnaround": True,
    "read_turnaround_delay_us": 2500,
    "read_timeout_ms": 12,
    "read_attempts": 3,
}
TEST_PROFILE_YAML = """# 시험용(실제 로봇 값 아님)
armpi_control:
  ros__parameters:
    arm.servo_ids: [6, 3, 4, 5, 2, 1]
    arm.joint_directions: [1, -1, 1, 1, 1, 1]
    arm.gripper.open_value: 100
    arm.gripper.close_value: 400
    hardware.servo_rx_con_bcm: 17
    hardware.servo_tx_con_bcm: 22
    servo.pre_write_delay_us: 400
    servo.post_write_delay_us: 300
    servo.read_turnaround_delay_us: 2500
    servo.read_timeout_ms: 12
    servo.read_attempts: 3
"""
VIN = {i: 7400 + i for i in range(1, 7)}
TEMP = {i: 30 + i for i in range(1, 7)}


class FakeGpio:
    def __init__(self):
        self.state = {}
        self.events = []  # (t_ns, kind, bcm, value)
        self.lock = threading.Lock()

    def request_output(self, bcm, initial):
        with self.lock:
            self.state[bcm] = 1 if initial else 0
            self.events.append((time.perf_counter_ns(), "request", bcm, self.state[bcm]))

    def write(self, bcm, value):
        with self.lock:
            if bcm not in self.state:
                raise OSError(22, f"GPIO{bcm} 요청되지 않음")
            self.state[bcm] = 1 if value else 0
            self.events.append((time.perf_counter_ns(), "write", bcm, self.state[bcm]))

    def is_rx(self, rx=RX, tx=TX):
        return self.state.get(rx) == 1 and self.state.get(tx) == 0

    def is_tx(self, rx=RX, tx=TX):
        return self.state.get(tx) == 1 and self.state.get(rx) == 0

    def state_at(self, t_ns, rx=RX, tx=TX):
        """기록된 사건으로 t_ns 시각의 (rx, tx) 값을 재현한다."""
        st = {}
        with self.lock:
            for e in self.events:
                if e[0] > t_ns:
                    break
                st[e[2]] = e[3]
        return st.get(rx), st.get(tx)

    def rx_at(self, t_ns):
        return self.state_at(t_ns) == (1, 0)

    def tx_at(self, t_ns):
        return self.state_at(t_ns) == (0, 1)

    def close(self):
        pass


def make_response(sid, cmd, params, *, rid=None, rcmd=None, header=(0x55, 0x55), length=None, bad_chk=False):
    rid = sid if rid is None else rid
    rcmd = cmd if rcmd is None else rcmd
    ln = 3 + len(params) if length is None else length
    body = bytes([rid, ln, rcmd]) + bytes(params)
    chk = D.checksum(body)
    if bad_chk:
        chk ^= 0xFF
    return bytes(header) + body + bytes([chk])


def value_params(sid, cmd):
    if cmd == 28:
        v = POS[sid] & 0xFFFF
        return [v & 0xFF, (v >> 8) & 0xFF]
    if cmd == 27:
        v = VIN[sid]
        return [v & 0xFF, (v >> 8) & 0xFF]
    if cmd == 26:
        return [TEMP[sid]]
    return [sid]  # 14


class FakeServoBus(threading.Thread):
    def __init__(self, gpio=None, delay_ms=1.5):
        super().__init__(daemon=True)
        self.master, slave = os.openpty()
        self.slave_path = os.ttyname(slave)
        os.close(slave)  # 시험 프로세스가 슬레이브를 잡고 있지 않게(포트 점유 검사에 걸리지 않도록)
        self.gpio = gpio  # None 이면 방향 핀 무시(--no-gpio CLI 시험)
        self.delay_ms = delay_ms
        self.behaviors = {}  # (id, cmd) 또는 id → 동작 이름
        self.seen = {}
        self.wire = []  # (t, hex)
        self.requests = []  # 받은 패킷(bytes)
        self.violations = []  # 읽기 요청이 아닌 패킷
        self.tx_state_at_request = []  # 요청을 다 받았을 때 GPIO 가 보내기 모드였는지
        self.responses = []  # (id, cmd, 동작, 'sent'|'lost'|'none')
        self.stop_ev = threading.Event()
        self.buf = bytearray()
        self.write_times = []  # 도구가 쓰기를 마친 시각(perf_counter_ns), 같은 프로세스 시험에서만 채움

    def behavior(self, sid, cmd):
        return self.behaviors.get((sid, cmd), self.behaviors.get(sid, "ok"))

    def run(self):
        while not self.stop_ev.is_set():
            r, _, _ = select.select([self.master], [], [], 0.02)
            if not r:
                continue
            try:
                data = os.read(self.master, 256)
            except OSError:
                time.sleep(0.005)  # 슬레이브가 닫혀 있으면 EIO
                continue
            if not data:
                continue
            self.wire.append((time.time(), data.hex()))
            self.buf += data
            self._drain_requests()

    def _drain_requests(self):
        while True:
            while self.buf and self.buf[0] != 0x55:
                self.violations.append(("stray", bytes([self.buf[0]]).hex()))
                del self.buf[0]
            if len(self.buf) < 4:
                return
            if self.buf[1] != 0x55:
                self.violations.append(("bad_header", bytes(self.buf[:2]).hex()))
                del self.buf[0]
                continue
            n = self.buf[3] + 3
            if len(self.buf) < n:
                return
            pkt = bytes(self.buf[:n])
            del self.buf[:n]
            k = len(self.requests)
            if k < len(self.write_times):
                t_write = self.write_times[k]
            else:
                t_write = time.perf_counter_ns()
            t_end = t_write + int(len(pkt) * 10 / 115200 * 1e9)  # 선에 다 나간 시각
            self.requests.append(pkt)
            if self.gpio is not None:
                self.tx_state_at_request.append(self.gpio.tx_at(t_write))
            try:
                D.assert_read_only_packet(pkt)
            except D.ReadOnlyViolation as e:
                self.violations.append(("not_read", pkt.hex(), str(e)))
                continue
            self._respond(pkt[2], pkt[4], t_end)

    def _respond(self, sid, cmd, t_end):
        beh = self.behavior(sid, cmd)
        key = (sid, cmd)
        count = self.seen.get(key, 0)
        self.seen[key] = count + 1
        if beh == "silent" or sid not in POS:
            self.responses.append((sid, cmd, beh, "none"))
            return
        params = value_params(sid, cmd)
        if beh == "ok" or (beh == "bad_checksum_once" and count >= 1):
            resp = make_response(sid, cmd, params)
        elif beh in ("bad_checksum", "bad_checksum_once"):
            resp = make_response(sid, cmd, params, bad_chk=True)
        elif beh == "wrong_id":
            resp = make_response(sid, cmd, params, rid=(sid % 6) + 1)
        elif beh == "wrong_cmd":
            other = 27 if cmd != 27 else 26
            resp = make_response(sid, cmd, value_params(sid, other), rcmd=other)
        elif beh == "bad_header":
            resp = make_response(sid, cmd, params, header=(0xAA, 0x55))
        elif beh == "bad_length":
            resp = make_response(sid, cmd, params, length=2)
        elif beh == "short":
            resp = make_response(sid, cmd, params)[:5]
        elif beh == "wrong_param_len":
            resp = make_response(sid, cmd, params[:1])
        else:
            raise ValueError(beh)
        deadline = t_end + int(self.delay_ms * 1e6)
        while time.perf_counter_ns() < deadline:
            time.sleep(0.0001)
        if self.gpio is not None and not self.gpio.rx_at(deadline):
            self.responses.append((sid, cmd, beh, "lost"))  # 호스트가 아직 보내기 모드: 버스에서 사라짐
            return
        os.write(self.master, resp)
        self.responses.append((sid, cmd, beh, "sent"))

    def stop(self):
        self.stop_ev.set()
        self.join(timeout=2)
        os.close(self.master)


class Results:
    def __init__(self):
        self.items = []

    def check(self, tid, title, ok, detail=""):
        self.items.append({"id": tid, "title": title, "result": "PASS" if ok else "FAIL", "detail": detail})
        print(f"{'PASS' if ok else 'FAIL'} {tid} {title}" + (f" — {detail}" if detail else ""))

    def info(self, tid, title, detail):
        self.items.append({"id": tid, "title": title, "result": "INFO", "detail": detail})
        print(f"INFO {tid} {title} — {detail}")


def new_reader(fake, timing=None):
    port = D.SerialPort(fake.slave_path, 115200)
    gpio = fake.gpio
    reader = D.ServoReader(port, gpio, RX, TX, timing or TEST_TIMING)
    reader.init()
    return reader, port


def run_inprocess(res: Results, log):
    gpio = FakeGpio()
    fake = FakeServoBus(gpio=gpio, delay_ms=1.5)
    fake.start()
    write_states = []
    orig_write_all = D.SerialPort.write_all

    def spy_write_all(self, data):
        write_states.append((gpio.is_tx(), bytes(data).hex()))
        r = orig_write_all(self, data)
        fake.write_times.append(time.perf_counter_ns())
        return r

    D.SerialPort.write_all = spy_write_all
    try:
        reader, port = new_reader(fake)

        # T1 정상 읽기
        bad = []
        for sid in range(1, 7):
            for name, cmd in D.READ_COMMANDS.items():
                r = reader.read(sid, cmd)
                exp = {"pos": POS, "vin": VIN, "temp": TEMP, "id": {i: i for i in POS}}[name][sid]
                if not (r["ok"] and r["value"] == exp and r["attempts"] == 1):
                    bad.append((sid, name, r))
        res.check("T01", "정상 응답 6개 서보 × 4항목(pos/vin/temp/id) 값 일치, 재시도 없음", not bad, f"불일치 {len(bad)}")
        r = reader.read(2, 28)
        res.check("T02", "부호 있는 위치(−5) 해석", r["ok"] and r["value"] == -5, f"value={r['value']}")

        cases = [
            ("T03", "체크섬 오류 1회 → 두 번째 시도 성공", 3, 28, "bad_checksum_once",
             lambda r: r["ok"] and r["attempts"] == 2 and r["value"] == POS[3]),
            ("T04", "체크섬 오류 계속 → 실패(bad_checksum), read_attempts(시험값 3)회 시도", 4, 28, "bad_checksum",
             lambda r: (not r["ok"]) and r["attempts"] == TEST_TIMING["read_attempts"] and "bad_checksum" in r["error"]),
            ("T05", "응답 없음 → 시간 초과 실패", 5, 27, "silent",
             lambda r: (not r["ok"]) and "시간 초과" in r["error"] and r["elapsed_ms"] < 150),
            ("T06", "다른 ID 의 응답 → 실패(unexpected_id)", 2, 28, "wrong_id",
             lambda r: (not r["ok"]) and "unexpected_id" in r["error"]),
            ("T07", "다른 명령의 응답 → 실패(unexpected_cmd)", 6, 26, "wrong_cmd",
             lambda r: (not r["ok"]) and "unexpected_cmd" in r["error"]),
            ("T08", "응답 머리 오류 → 실패", 1, 26, "bad_header",
             lambda r: (not r["ok"]) and "머리" in r["error"]),
            ("T09", "응답 길이 바이트 오류(<3) → 실패", 1, 27, "bad_length",
             lambda r: (not r["ok"]) and "길이" in r["error"]),
            ("T10", "응답이 중간에 끊김 → 시간 초과 실패", 3, 26, "short",
             lambda r: (not r["ok"]) and "시간 초과" in r["error"]),
            ("T11", "값 길이 오류(POS 1 B) → 실패", 6, 28, "wrong_param_len",
             lambda r: (not r["ok"]) and "값 길이" in r["error"]),
        ]
        for tid, title, sid, cmd, beh, pred in cases:
            fake.behaviors = {(sid, cmd): beh}
            fake.seen = {}
            r = reader.read(sid, cmd)
            res.check(tid, title, pred(r), f"error={r['error']}, attempts={r['attempts']}, {r['elapsed_ms']} ms")
        fake.behaviors = {}
        r = reader.read(7, 28)
        res.check("T12", "없는 ID(7) → 시간 초과 실패", (not r["ok"]) and "시간 초과" in r["error"], f"error={r['error']}")

        # T13 방향 전환이 늦으면 응답 유실(S3 의 의미)
        port.close()
        slow = dict(TEST_TIMING, post_write_delay_us=5000)
        reader_slow, port_slow = new_reader(fake, slow)
        n_lost0 = sum(1 for x in fake.responses if x[3] == "lost")
        r_slow = reader_slow.read(3, 28)
        n_lost = sum(1 for x in fake.responses if x[3] == "lost") - n_lost0
        port_slow.close()
        reader, port = new_reader(fake)
        r_ok = reader.read(3, 28)
        res.check("T13", "쓰기 뒤 대기 5000 µs(서보 응답 1.5 ms보다 늦음) → 응답 유실·실패, 시험 설정 300 µs → 성공",
                  (not r_slow["ok"]) and n_lost == TEST_TIMING["read_attempts"] and r_ok["ok"],
                  f"느린 설정: {r_slow['error']}, 유실 {n_lost}건 / 기본: value={r_ok['value']}")

        # T14 GPIO 순서
        req = [e for e in gpio.events if e[1] == "request"]
        init_ok = [(e[2], e[3]) for e in req[:2]] == [(RX, 0), (TX, 1)]
        all_tx = all(s for s, _ in write_states)
        final_tx = gpio.is_tx()
        res.check("T14", "GPIO: 시작 RX_CON=0·TX_CON=1, 모든 쓰기가 보내기 모드에서, 끝나면 보내기 모드",
                  init_ok and all_tx and final_tx and len(write_states) > 0,
                  f"쓰기 {len(write_states)}회, 보내기 모드 아님 {sum(1 for s, _ in write_states if not s)}회")
        # 방향 전환 순서: 보내기 = TX 먼저 1, RX 0 / 받기 = RX 먼저 1, TX 0
        seq = [(e[2], e[3]) for e in gpio.events if e[1] == "write"]
        pairs_ok = True
        for i in range(0, len(seq) - 1, 2):
            if seq[i:i + 2] not in ([(TX, 1), (RX, 0)], [(RX, 1), (TX, 0)]):
                pairs_ok = False
                break
        res.check("T15", "방향 전환 순서가 armpi_hardware 와 같음(보내기: TX→RX, 받기: RX→TX)", pairs_ok and len(seq) % 2 == 0,
                  f"GPIO 쓰기 {len(seq)}회")

        # T16 읽기 전용 차단
        blocked = []
        for fn in (lambda: D.encode_read(1, 1), lambda: D.encode_read(1, 12), lambda: D.encode_read(1, 31),
                   lambda: D.encode_read(1, 13), lambda: D.encode_read(254, 28), lambda: D.encode_read(-1, 28)):
            try:
                fn()
                blocked.append(False)
            except D.ReadOnlyViolation:
                blocked.append(True)
        n_req_before = len(fake.requests)
        move_pkt = bytes([0x55, 0x55, 1, 7, 1, 0xF4, 0x01, 0xE8, 0x03])
        move_pkt += bytes([D.checksum(move_pkt[2:])])
        try:
            reader._write_packet(move_pkt)
            sent_blocked = False
        except D.ReadOnlyViolation:
            sent_blocked = True
        time.sleep(0.05)
        nothing_sent = len(fake.requests) == n_req_before
        res.check("T16", "쓰기 명령(1·12·13·31)·방송 ID 254·음수 ID 차단, 이동 패킷은 보내기 전에 막힘",
                  all(blocked) and sent_blocked and nothing_sent,
                  f"차단 {sum(blocked)}/{len(blocked)}, 이동 패킷 차단={sent_blocked}, 선에 나감={not nothing_sent}")

        # T27 독점 열기: 열려 있는 동안 다른 프로세스는 못 열고, 닫은 뒤에는 다시 열 수 있어야 한다
        probe = ("import os,sys\ntry:\n fd=os.open(sys.argv[1], os.O_RDWR|os.O_NOCTTY); os.close(fd); print('OPENED')\n"
                 "except OSError as e:\n print('ERR', e.errno)")
        while_open = subprocess.run([sys.executable, "-I", "-c", probe, fake.slave_path],
                                    capture_output=True, text=True, timeout=10).stdout.strip()
        port.close()
        after_close = subprocess.run([sys.executable, "-I", "-c", probe, fake.slave_path],
                                     capture_output=True, text=True, timeout=10).stdout.strip()
        reader, port = new_reader(fake)
        ok_reopen = after_close == "OPENED"
        excl_note = "다른 프로세스 열기 차단됨" if while_open.startswith("ERR 16") else f"다른 프로세스 열기 결과: {while_open}"
        res.check("T27", "포트 독점: 닫은 뒤 다시 열 수 있음(독점 해제 확인)", ok_reopen,
                  f"열려 있는 동안: {excl_note} / 닫은 뒤: {after_close}")

        src = open(TOOL, encoding="utf-8").read()
        res.check("T17", "도구 소스: 허용 명령 = {14,26,27,28}, 직렬 쓰기 호출 os.write( 1곳",
                  D.ALLOWED_CMDS == frozenset({14, 26, 27, 28}) and src.count("os.write(") == 1,
                  f"허용={sorted(D.ALLOWED_CMDS)}, os.write 호출 {src.count('os.write(')}곳")
        port.close()
    finally:
        D.SerialPort.write_all = orig_write_all
        fake.stop()

    # T18 선 위 바이트 전체가 읽기 요청뿐
    cmds = sorted({p[4] for p in fake.requests})
    res.check("T18", "가짜 서보가 받은 모든 패킷이 6 B 읽기 요청(쓰기 명령 0건)",
              not fake.violations and all(len(p) == 6 for p in fake.requests) and set(cmds) <= {14, 26, 27, 28},
              f"패킷 {len(fake.requests)}개, 명령 {cmds}, 위반 {len(fake.violations)}")
    res.check("T19", "쓰기를 마친 시각의 GPIO 기록이 보내기 모드(전부)",
              all(fake.tx_state_at_request) and len(fake.tx_state_at_request) == len(fake.requests),
              f"{sum(fake.tx_state_at_request)}/{len(fake.tx_state_at_request)}")
    with open(os.path.join(log, "wire_inprocess.txt"), "w") as f:
        f.write("# 가짜 서보가 받은 패킷(hex)\n")
        for p in fake.requests:
            f.write(p.hex() + "\n")
        f.write("# 응답 (id, cmd, 동작, 결과)\n")
        for x in fake.responses:
            f.write(f"{x}\n")
    with open(os.path.join(log, "gpio_events.csv"), "w") as f:
        f.write("t_ns,kind,bcm,value\n")
        for e in gpio.events:
            f.write(f"{e[0]},{e[1]},{e[2]},{e[3]}\n")


_PROFILE_PATH = None


def test_profile_path():
    """시험용 프로필을 임시 파일로 한 번 쓴다."""
    global _PROFILE_PATH
    if _PROFILE_PATH is None:
        fd, _PROFILE_PATH = tempfile.mkstemp(prefix="servo_diag_test_profile_", suffix=".yaml")
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            f.write(TEST_PROFILE_YAML)
    return _PROFILE_PATH


def run_cli(args, env=None, timeout=30, profile=True):
    # 설치된 프로필을 자동으로 찾지 않게 환경을 비운다. profile=True 면 시험용 프로필을 준다.
    env = dict(os.environ if env is None else env)
    env.pop("ARMPI_HARDWARE_PROFILE", None)
    env.pop("AMENT_PREFIX_PATH", None)
    extra = ["--hardware-profile", test_profile_path()] if profile else []
    p = subprocess.run([sys.executable, "-I", TOOL] + extra + args, capture_output=True, text=True, timeout=timeout,
                       env=env)
    return p.returncode, p.stdout + p.stderr


def fake_systemctl_dir(state, code):
    d = tempfile.mkdtemp(prefix="fake_systemctl_")
    path = os.path.join(d, "systemctl")
    with open(path, "w") as f:
        f.write(f"#!/bin/sh\n# 시험용 가짜 systemctl: 상태만 출력\necho {state}\nexit {code}\n")
    os.chmod(path, stat.S_IRWXU)
    return d


def run_cli_tests(res: Results, log):
    out_lines = []

    fake = FakeServoBus(gpio=None, delay_ms=1.5)
    fake.start()
    try:
        js = os.path.join(log, "cli_ok.json")
        code, out = run_cli(["--port", fake.slave_path, "--no-gpio", "--skip-preflight", "--ids", "1-3",
                             "--read", "pos,vin,temp", "--json", js])
        out_lines.append(("cli_ok", code, out))
        ok = code == 0
        if ok:
            rep = json.load(open(js, encoding="utf-8"))
            row = {r["id"]: r for r in rep["rounds"][0]["rows"]}
            ok = all(row[i]["pos"]["value"] == POS[i] and row[i]["vin"]["value"] == VIN[i]
                     and row[i]["temp"]["value"] == TEMP[i] for i in (1, 2, 3)) and rep["sent_commands"] == [26, 27, 28]
        res.check("T20", "명령행 끝까지 실행(--no-gpio, 가짜 서보): 종료 코드 0, JSON 값 일치", ok, f"exit={code}")

        env = dict(os.environ)
        env["PATH"] = fake_systemctl_dir("active", 0) + os.pathsep + env.get("PATH", "")
        code, out = run_cli(["--port", fake.slave_path, "--no-gpio", "--ids", "1"], env=env)
        out_lines.append(("cli_refuse_active", code, out))
        res.check("T21", "armpi.service 가 active 이면 실행 거부(종료 코드 4), 패킷 0개",
                  code == 4 and "armpi.service" in out and "보낸 패킷" not in out, f"exit={code}")

        env["PATH"] = fake_systemctl_dir("inactive", 3) + os.pathsep + os.environ.get("PATH", "")
        n0 = len(fake.requests)
        code, out = run_cli(["--port", fake.slave_path, "--no-gpio", "--ids", "1", "--read", "pos"], env=env)
        out_lines.append(("cli_inactive", code, out))
        res.check("T22", "armpi.service 가 inactive 면 진행(종료 코드 0)", code == 0 and len(fake.requests) > n0,
                  f"exit={code}")

        if os.path.isdir("/proc/self/fd"):
            holder = subprocess.Popen([sys.executable, "-I", "-c",
                                       "import os,sys,time; fd=os.open(sys.argv[1], os.O_RDWR|os.O_NOCTTY); time.sleep(20)",
                                       fake.slave_path])
            time.sleep(0.5)
            env["PATH"] = fake_systemctl_dir("inactive", 3) + os.pathsep + os.environ.get("PATH", "")
            n0 = len(fake.requests)
            code, out = run_cli(["--port", fake.slave_path, "--no-gpio", "--ids", "1"], env=env)
            holder.kill()
            holder.wait()
            out_lines.append(("cli_refuse_busy", code, out))
            res.check("T23", "다른 프로세스가 포트를 열고 있으면 실행 거부(종료 코드 4), 패킷 0개",
                      code == 4 and f"pid {holder.pid}" in out and len(fake.requests) == n0, f"exit={code}")
        else:
            code, out = run_cli(["--port", fake.slave_path, "--no-gpio", "--ids", "1", "--read", "pos"], env=env)
            out_lines.append(("cli_no_proc", code, out))
            res.info("T23", "포트 점유 검사", "이 OS 에는 /proc 가 없어 건너뜀(Linux 시험에서 확인). 도구는 '/proc 없음' 안내를 출력: "
                     + str("/proc 없음" in out))
    finally:
        fake.stop()

    code, out = run_cli(["--ids", "254", "--skip-preflight"])
    out_lines.append(("cli_bad_id", code, out))
    code2, out2 = run_cli(["--read", "move", "--skip-preflight"])
    out_lines.append(("cli_bad_read", code2, out2))
    res.check("T24", "인자 오류(ID 254, 항목 move) → 종료 코드 2", code == 2 and code2 == 2, f"exit={code},{code2}")

    code, out = run_cli(["--port", "/dev/nonexistent_servo_port", "--no-gpio", "--skip-preflight"])
    out_lines.append(("cli_no_port", code, out))
    res.check("T25", "없는 포트 → 종료 코드 5(준비 실패)", code == 5, f"exit={code}")

    # T28 프로필 없음: 핀·타이밍이 비어 있으면 포트를 열기 전에 멈춘다(종료 코드 2, 5 가 아님)
    code, out = run_cli(["--port", "/dev/nonexistent_servo_port", "--skip-preflight"], profile=False)
    out_lines.append(("cli_no_profile", code, out))
    code2, out2 = run_cli(["--port", "/dev/nonexistent_servo_port", "--no-gpio", "--skip-preflight",
                           "--pre-write-us", "400"], profile=False)
    out_lines.append(("cli_partial_timing", code2, out2))
    res.check("T28", "하드웨어 프로필 없음·타이밍 일부만 → 종료 코드 2, 빠진 항목 이름 표시, 포트 안 엶",
              code == 2 and "하드웨어 프로필 값 없음" in out and "servo_rx_con_bcm" in out and "read_attempts" in out
              and code2 == 2 and "servo.post_write_delay_us" in out2 and "servo.pre_write_delay_us" not in out2
              and "servo_rx_con_bcm" not in out2,
              f"exit={code},{code2}")

    # T29 관절 표시: 프로필의 관절 배정·방향·그리퍼 끝값으로만 계산, 프로필 없으면 표시 안 함
    jm = D.joint_map(D.load_profile(test_profile_path()))
    shown = (D.joint_info(3, 512, jm), D.joint_info(6, 500, jm), D.joint_info(1, 250, jm), D.joint_info(3, 512, None))
    res.check("T29", "관절 표시: 시험 프로필 기준(반대 방향 관절) 512 → −2.9°, 그리퍼 250 → 0.50, 프로필 없으면 빈칸",
              shown[0] == "joint2 -2.9°" and shown[1] == "joint1 +0.0°" and "0.50" in shown[2] and shown[3] == "",
              repr(shown))

    # libgpiod (정보): 이 환경에서 라이브러리·칩을 열 수 있는지
    try:
        g = D.LibgpiodGpio("gpiochip0")
        g.close()
        res.info("T26", "libgpiod", "라이브러리 로드·gpiochip0 열기 성공(이 환경에 GPIO 칩이 있음)")
    except OSError as e:
        res.info("T26", "libgpiod", f"열기 실패(GPIO 칩 또는 라이브러리 없음 = 로봇 밖에서는 예상됨): {e}")
    except (AttributeError, TypeError) as e:
        res.info("T26", "libgpiod", f"라이브러리는 있으나 v1 API 아님: {e}")
    except Exception as e:  # 라이브러리 없음(Mac 등)
        res.info("T26", "libgpiod", f"라이브러리 없음: {type(e).__name__}: {e}")

    with open(os.path.join(log, "cli_outputs.txt"), "w", encoding="utf-8") as f:
        for name, code, out in out_lines:
            f.write(f"===== {name} (exit {code})\n{out}\n")


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        h.update(f.read())
    return h.hexdigest()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--log-dir", required=True)
    a = ap.parse_args()
    os.makedirs(a.log_dir, exist_ok=True)
    res = Results()
    t0 = time.time()
    print(f"servo_read_diag 자체 시험 — {platform.platform()}, Python {platform.python_version()}")
    run_inprocess(res, a.log_dir)
    run_cli_tests(res, a.log_dir)
    n_pass = sum(1 for i in res.items if i["result"] == "PASS")
    n_fail = sum(1 for i in res.items if i["result"] == "FAIL")
    n_info = sum(1 for i in res.items if i["result"] == "INFO")
    summary = {
        "when": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "platform": platform.platform(), "python": platform.python_version(),
        "evidence_level": "모의 시험(pty 가짜 서보·가짜 GPIO). 실제 서보·Pi 아님",
        "tool_sha256": sha256(TOOL), "selftest_sha256": sha256(os.path.abspath(__file__)),
        "pass": n_pass, "fail": n_fail, "info": n_info, "seconds": round(time.time() - t0, 2),
        "items": res.items,
    }
    with open(os.path.join(a.log_dir, "results.json"), "w", encoding="utf-8") as f:
        json.dump(summary, f, ensure_ascii=False, indent=1)
    print(f"\n합계: PASS {n_pass} / FAIL {n_fail} / INFO {n_info} ({summary['seconds']} s)")
    return 0 if n_fail == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
