#!/usr/bin/env python3
"""웹 조종기 종단 확인(브라우저 없이 HTTP API 로). 가짜 로봇과 조종기가 떠 있어야 한다.
파이썬 조종기(+ mock_robot.py)와 C++ armpi_web_gateway(+ ROS 가짜 스택) 둘 다 같은 항목으로 확인한다.

브라우저가 하는 일(50 ms 마다 /api/input)을 스레드로 흉내 내고, 조종기 /api/state 와 가짜 로봇의
/armpi/status(조종기가 rosbridge 로 받아 /api/state 에 실어 줌)를 보고 판정한다.

  python3 e2e_mac_controller_check.py --http 127.0.0.1:18765 --camera http://127.0.0.1:8080 --json result.json
  python3 e2e_mac_controller_check.py --http 127.0.0.1:18766 --only estop-idle   # on-demand 조종기: 연결 없이 비상정지
차체 기대값은 --limits(로봇 설정 상한 vx,vy,wz; 기본은 armpi.yaml·mock_robot.py 와 같은 0.25,0.25,1.2)로 계산한다.
"""

import argparse
import json
import sys
import threading
import time
import urllib.error
import urllib.request

RESULTS = []


def check(name, cond, detail=""):
    RESULTS.append({"check": name, "pass": bool(cond), "detail": detail})
    print("[%s] %s %s" % ("PASS" if cond else "FAIL", name, detail), flush=True)
    return cond


class Api:
    def __init__(self, base):
        self.base = "http://" + base

    def req(self, path, body=None, headers=None, method=None):
        data = None if body is None else json.dumps(body).encode()
        h = {"Content-Type": "application/json"} if body is not None else {}
        h.update(headers or {})
        r = urllib.request.Request(self.base + path, data=data, headers=h, method=method)
        try:
            with urllib.request.urlopen(r, timeout=3) as resp:
                return resp.status, json.loads(resp.read() or b"{}")
        except urllib.error.HTTPError as exc:
            try:
                return exc.code, json.loads(exc.read() or b"{}")
            except ValueError:
                return exc.code, {}

    def state(self):
        return self.req("/api/state")[1]

    def cmd(self, c, **kw):
        kw["cmd"] = c
        return self.req("/api/cmd", kw)


class Browser(threading.Thread):
    """브라우저의 50 ms 입력 루프 흉내."""

    def __init__(self, api):
        super().__init__(daemon=True)
        self.api = api
        self.lock = threading.Lock()
        self.inp = {"f": 0.0, "s": 0.0, "t": 0.0, "held": False, "focused": True,
                    "arm": {"x": 0, "y": 0, "z": 0, "p": 0, "g": 0}}
        self.paused = False
        self.running = True
        self.start()

    def set(self, **kw):
        with self.lock:
            arm = kw.pop("arm", None)
            self.inp.update(kw)
            self.inp["arm"] = dict({"x": 0, "y": 0, "z": 0, "p": 0, "g": 0}, **(arm or {}))

    def release(self):
        self.set(f=0.0, s=0.0, t=0.0, held=False, focused=True)

    def run(self):
        while self.running:
            if not self.paused:
                with self.lock:
                    body = json.loads(json.dumps(self.inp))
                try:
                    self.api.req("/api/input", body)
                except OSError:
                    pass
            time.sleep(0.05)


def wait_for(fn, timeout=2.0, period=0.05):
    end = time.monotonic() + timeout
    last = None
    while time.monotonic() < end:
        last = fn()
        if last:
            return last
        time.sleep(period)
    return last


def robot_status(api):
    st = api.state()
    rb = st.get("rosbridge") or {}
    return st, (rb.get("status") or {})


def estop_idle_checks(api):
    """on-demand 조종기에서 조종권 없이 비상정지: 비상정지는 가지만 '연결'·조종권은 생기지 않고, 임시 세션은 2 s 뒤 닫힌다."""
    code, cfg = api.req("/api/config")
    check("[대기 비상정지] on-demand 조종기", code == 200 and cfg.get("session_mode") == "on-demand", str(cfg.get("session_mode")))
    st = api.state()
    ses = st.get("session") or {}
    check("[대기 비상정지] 처음: 세션 닫힘(reason none)·조종권 없음",
          ses.get("live") is False and ses.get("reason") == "none" and st["control"]["owner"] is None, json.dumps(ses))
    t0 = time.monotonic()
    code, res = api.req("/api/cmd", {"cmd": "estop"})
    st = api.state()
    ses = st.get("session") or {}
    check("[대기 비상정지] 비상정지 명령 수락", code == 200 and res.get("ok") is True, str(res))
    check("[대기 비상정지] 세션 이유 = estop_only(연결로 보이지 않음)", ses.get("live") is True and ses.get("reason") == "estop_only",
          json.dumps(ses))
    check("[대기 비상정지] 조종권은 생기지 않음", st["control"]["owner"] is None and not st["control"]["you"],
          json.dumps(st["control"].get("owner")))
    got = wait_for(lambda: (lambda r: r if r.get("emergency_stop") is True else None)(robot_status(api)[1]), 1.5)
    check("[대기 비상정지] 로봇 비상정지 latch", bool(got), "sources=%s" % (got or {}).get("estop_sources"))
    closed = wait_for(lambda: (lambda s: s if not (s.get("session") or {}).get("live") else None)(api.state()), 4.0)
    dt = time.monotonic() - t0
    check("[대기 비상정지] 약 2 s 뒤 임시 세션 닫힘", bool(closed) and 1.8 <= dt <= 4.0,
          "%.2f s, session=%s" % (dt, json.dumps((closed or api.state()).get("session"))))
    time.sleep(0.6)
    st, rs = robot_status(api)
    check("[대기 비상정지] 세션이 닫힌 뒤에도 로봇 비상정지 표시 유지(로봇 상태 경로)",
          rs.get("emergency_stop") is True and not (st.get("session") or {}).get("live"),
          "emergency_stop=%s live=%s" % (rs.get("emergency_stop"), (st.get("session") or {}).get("live")))
    check("[대기 비상정지] 끝까지 조종권 없음", st["control"]["owner"] is None, json.dumps(st["control"].get("owner")))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--http", default="127.0.0.1:18765")
    ap.add_argument("--camera", default="http://127.0.0.1:8080")
    ap.add_argument("--json")
    ap.add_argument("--limits", default="0.25,0.25,1.2",
                    help="로봇 차체 상한 vx,vy,wz(m/s, m/s, rad/s). 기본 = armpi.yaml·mock_robot.py 기본값")
    ap.add_argument("--only", choices=["estop-idle"], help="estop-idle: on-demand 조종기에서 연결 없이 비상정지만 확인")
    args = ap.parse_args()
    api = Api(args.http)
    t_start = time.time()
    lim_vx, lim_vy, lim_wz = (float(v) for v in args.limits.split(","))
    half_vx = 0.5 * lim_vx  # 50 % 입력일 때 기대 속도
    if args.only == "estop-idle":
        estop_idle_checks(api)
        summary = {"pass": sum(r["pass"] for r in RESULTS), "fail": sum(not r["pass"] for r in RESULTS),
                   "seconds": round(time.time() - t_start, 1)}
        print("\n요약: 통과 %d / 실패 %d (%.1f s)" % (summary["pass"], summary["fail"], summary["seconds"]))
        if args.json:
            with open(args.json, "w", encoding="utf-8") as fp:
                json.dump({"summary": summary, "checks": RESULTS}, fp, ensure_ascii=False, indent=2)
        return 0 if summary["fail"] == 0 else 1

    code, cfg = api.req("/api/config")
    check("config 응답", code == 200 and cfg.get("robot") == "127.0.0.1", json.dumps(cfg, ensure_ascii=False)[:200])
    br = Browser(api)

    # 1. 연결·중립 확인
    ok = wait_for(lambda: (lambda s: s["udp"]["connected"] and s["udp"]["base"]["state"] == "idle"
                           and s["udp"]["arm"]["state"] == "idle" and s["rosbridge"].get("connected")
                           and s["rosbridge"].get("status"))(api.state()), 4.0)
    st = api.state()
    check("UDP 연결·두 채널 중립 확인·rosbridge 연결", bool(ok),
          "base=%s arm=%s rb=%s" % (st["udp"]["base"]["state"], st["udp"]["arm"]["state"], st["rosbridge"].get("connected")))
    check("가짜 로봇 소유자 = 조종기 세션", (st["rosbridge"]["status"] or {}).get("owner") == st["udp"]["client_id"])

    # 2. 차체 W/S/A/D/Q/E
    for name, kw, key, expect in (("전진 W 50%", dict(f=0.5), "vx", half_vx), ("후진 S 50%", dict(f=-0.5), "vx", -half_vx),
                                  ("왼쪽 평행이동 A 50%", dict(s=0.5), "vy", 0.5 * lim_vy),
                                  ("오른쪽 평행이동 D 50%", dict(s=-0.5), "vy", -0.5 * lim_vy),
                                  ("좌회전 Q 50%", dict(t=0.5), "wz", 0.5 * lim_wz),
                                  ("우회전 E 50%", dict(t=-0.5), "wz", -0.5 * lim_wz)):
        br.set(held=True, focused=True, **kw)
        t0 = time.monotonic()
        got = wait_for(lambda: (lambda r: r if abs(r.get("base_cmd", {}).get(key, 0) - expect) < 1e-6 else None)(
            robot_status(api)[1]), 2.0)
        dt = time.monotonic() - t0
        st, rs = robot_status(api)
        check("%s → 가짜 로봇 %s=%.2f" % (name, key, expect), bool(got),
              "로봇 base_cmd=%s, 조종기 차체=%s, 반영 %.0f ms(상태 토픽 경유)" % (rs.get("base_cmd"), st["udp"]["base"]["state"], dt * 1e3))
        br.release()
        got = wait_for(lambda: (lambda r: r if r.get("base_cmd") == {"vx": 0.0, "vy": 0.0, "wz": 0.0} else None)(
            robot_status(api)[1]), 2.0)
        check("%s 뗌 → 정지" % name, bool(got), "base_cmd=%s" % robot_status(api)[1].get("base_cmd"))

    # 3. 포커스 잃음·입력 끊김
    br.set(f=0.5, held=True, focused=False)
    time.sleep(0.4)
    st, rs = robot_status(api)
    check("키를 누른 채 창 포커스 잃음 → 중립", rs.get("base_cmd", {}).get("vx") == 0.0 and st["udp"]["base"]["state"] == "idle",
          "base_cmd=%s base=%s" % (rs.get("base_cmd"), st["udp"]["base"]["state"]))
    br.set(f=0.5, held=True, focused=True)
    wait_for(lambda: robot_status(api)[1].get("base_cmd", {}).get("vx") == half_vx, 2.0)
    br.paused = True
    t0 = time.monotonic()
    got = wait_for(lambda: robot_status(api)[1].get("base_cmd", {}).get("vx") == 0.0, 2.0)
    check("이동 중 브라우저 입력 끊김 → 0.3 s 뒤 중립", bool(got), "정지 확인까지 %.0f ms(입력 timeout 300 ms + 상태 지연)"
          % ((time.monotonic() - t0) * 1e3))
    br.release()
    br.paused = False
    time.sleep(0.3)

    # 4. 팔
    st, rs = robot_status(api)
    cur0 = dict(rs.get("arm_current") or {})
    api.cmd("arm_enable", on=True)
    ok = wait_for(lambda: api.state()["udp"]["arm"]["state"] == "active", 2.0)
    st = api.state()
    check("팔 조종 켜기 → 팔 채널 active", bool(ok), "arm=%s armed=%s" % (st["udp"]["arm"]["state"], st["udp"]["arm"]["armed"]))
    check("팔 켤 때 목표 = 로봇 현재값", all(abs(st["udp"]["arm_target"][k] - cur0.get(k, 99)) < 1e-3 for k in ("x", "y", "z")),
          "target=%s current=%s" % (st["udp"]["arm_target"], cur0))
    x0 = st["udp"]["arm_target"]["x"]
    rate = float((cfg.get("rates") or {}).get("arm_m_s") or 0.05)  # 조종기 설정 속도(m/s)
    hold = 0.05 / rate                                              # 약 5 cm 움직일 만큼 누른다
    br.set(arm={"x": 1})
    time.sleep(hold)
    br.set(arm={})
    time.sleep(0.2)
    st, rs = robot_status(api)
    x1 = st["udp"]["arm_target"]["x"]
    check("I 키 %.2f s → 목표 x 약 +0.05 m(%.2f m/s)" % (hold, rate), 0.035 <= x1 - x0 <= 0.065,
          "x %.4f → %.4f (Δ %.4f)" % (x0, x1, x1 - x0))
    # 로봇 상태는 2 Hz 라 바로 읽으면 옛 값일 수 있다 → 1.5 s 안에 따라오는지 본다
    got = wait_for(lambda: (lambda r: r if abs(r.get("arm_target", {}).get("x", 0) - x1) < 0.003 else None)(
        robot_status(api)[1]), 1.5)
    rs = got or robot_status(api)[1]
    check("가짜 로봇 팔 목표가 조종기 목표를 따라옴", bool(got), "robot arm_target.x=%s" % rs.get("arm_target", {}).get("x"))
    time.sleep(0.6)
    rs = robot_status(api)[1]
    check("가짜 로봇 팔 현재값이 목표로 보간됨(로봇 손끝 속도 상한)", abs(rs.get("arm_current", {}).get("x", 0) - x1) < 0.003,
          "arm_current=%s" % rs.get("arm_current"))
    for z_dir, key in ((1, "z"),):
        z0 = api.state()["udp"]["arm_target"]["z"]
        br.set(arm={key: z_dir})
        time.sleep(0.4)
        br.set(arm={})
        time.sleep(0.2)
        z1 = api.state()["udp"]["arm_target"]["z"]
        check("U 키 0.4 s → 목표 z 증가", z1 - z0 > 0.01, "z %.4f → %.4f" % (z0, z1))

    acc = api.state()["udp"]["arm_target"]
    api.cmd("arm_target", x=0.45)
    got = wait_for(lambda: (lambda s: s if (s["udp"]["arm_last_reject"] or {}).get("reason") == "workspace_limit" else None)(api.state()), 1.0)
    st = got or api.state()
    check("작업 공간 밖 목표(x=0.45) → workspace_limit 표시", bool(got), "reject=%s" % st["udp"]["arm_last_reject"])
    check("거부 뒤 목표를 마지막 수락값으로 되돌림", abs(st["udp"]["arm_target"]["x"] - acc["x"]) < 0.003,
          "target.x=%.4f 마지막 수락=%.4f" % (st["udp"]["arm_target"]["x"], acc["x"]))
    check("거부 뒤에도 팔 채널 active 유지", st["udp"]["arm"]["state"] == "active", st["udp"]["arm"]["state"])
    time.sleep(0.2)
    api.cmd("arm_target", x=0.28, y=0.20, z=0.30, pitch=0.0)
    got = wait_for(lambda: (lambda s: s if (s["udp"]["arm_last_reject"] or {}).get("reason") == "ik_unreachable" else None)(api.state()), 1.0)
    check("도달 불가 목표 → ik_unreachable 표시", bool(got), "reject=%s" % (got or api.state())["udp"]["arm_last_reject"])
    api.cmd("gripper", value=1.0)
    got = wait_for(lambda: robot_status(api)[1].get("arm_target", {}).get("gripper") == 1.0, 1.5)
    check("그리퍼 1.0(닫힘) → 가짜 로봇 목표 gripper 1.0", bool(got))
    api.cmd("arm_home")
    got = wait_for(lambda: (api.state()["rosbridge"].get("last_result") or {}).get("service") == "/armpi/arm/home", 1.5)
    lr = api.state()["rosbridge"].get("last_result") or {}
    check("팔 조종 중 home 서비스 → 거부", bool(got) and lr.get("ok") is False, "%s" % lr)

    # 5. 비상정지(Space)
    br.set(f=0.5, held=True, focused=True)
    wait_for(lambda: robot_status(api)[1].get("base_cmd", {}).get("vx") == half_vx, 2.0)
    t0 = time.monotonic()
    api.cmd("estop")
    got = wait_for(lambda: robot_status(api)[1].get("emergency_stop") is True, 1.0, period=0.01)
    dt = time.monotonic() - t0
    st, rs = robot_status(api)
    check("비상정지 → 가짜 로봇 latch·차체 정지", bool(got) and rs.get("base_cmd", {}).get("vx") == 0.0,
          "상태 토픽 반영 %.0f ms, base_cmd=%s, sources=%s" % (dt * 1e3, rs.get("base_cmd"), rs.get("estop_sources")))
    # 채널 state 는 다음 패킷의 ACK(최대 50 ms 뒤)부터 emergency_stop 이 된다. 한 번만 읽지 않고 0.5 s 안에 되는지 본다.
    est = wait_for(lambda: (lambda s: s if s["udp"]["emergency"] and not s["arm_enabled"]
                            and s["udp"]["base"]["state"] == "emergency_stop" else None)(api.state()), 0.5, period=0.01)
    st = est or api.state()
    check("비상정지 → 조종기 emergency·팔 조종 꺼짐", bool(est),
          "base=%s arm_enabled=%s" % (st["udp"]["base"]["state"], st["arm_enabled"]))
    time.sleep(0.3)
    check("비상정지 중 키를 계속 눌러도 정지 유지", robot_status(api)[1].get("base_cmd", {}).get("vx") == 0.0)
    api.cmd("clear")
    got = wait_for(lambda: not api.state()["udp"]["emergency"], 1.0)
    time.sleep(0.4)
    st, rs = robot_status(api)
    check("해제(키를 누른 채) → 해제되지만 자동 출발하지 않음", bool(got) and rs.get("base_cmd", {}).get("vx") == 0.0
          and st["udp"]["base"]["needs_release"], "needs_release=%s base=%s" % (st["udp"]["base"]["needs_release"], st["udp"]["base"]["state"]))
    br.release()
    time.sleep(0.3)
    br.set(f=0.5, held=True)
    got = wait_for(lambda: robot_status(api)[1].get("base_cmd", {}).get("vx") == half_vx, 2.0)
    check("키를 뗐다 다시 누르면 출발", bool(got))
    br.release()
    time.sleep(0.3)

    # 6. rosbridge 전체 정지·해제
    api.cmd("robot_estop")
    got = wait_for(lambda: api.state()["udp"]["emergency"], 1.5)
    rs = robot_status(api)[1]
    check("로봇 전체 정지(service) → UDP ACK 로도 비상정지 보고", bool(got) and "service" in rs.get("estop_sources", []),
          "sources=%s" % rs.get("estop_sources"))
    api.cmd("clear")
    time.sleep(0.4)
    check("UDP 해제로는 service latch 가 풀리지 않음", api.state()["udp"]["emergency"] and robot_status(api)[1].get("emergency_stop"))
    api.cmd("robot_reset")
    got = wait_for(lambda: not api.state()["udp"]["emergency"] and not robot_status(api)[1].get("emergency_stop"), 1.5)
    check("전체 해제(reset_estop) → 비상정지 풀림", bool(got))

    # 7. 연결 끊기 → 관절 디버그 → home → 새 세션
    cid_old = api.state()["udp"]["client_id"]
    api.cmd("disconnect")
    got = wait_for(lambda: (lambda s: s if s["udp"]["retired"] and (s["rosbridge"]["status"] or {}).get("owner") is None
                            and s["joint_debug_allowed"] else None)(api.state()), 2.0)
    check("연결 끊기 → 로봇 소유자 없음·관절 디버그 허용", bool(got))
    before = dict(robot_status(api)[1].get("arm_target") or {})
    code, res = api.cmd("joint_debug", positions=[0.3, 0.5, -1.0, -0.5, 0.0, 0.5])
    got = wait_for(lambda: abs(robot_status(api)[1].get("arm_target", {}).get("y", 0) - before.get("y", 0)) > 0.01, 1.5)
    rs = robot_status(api)[1]
    check("관절 디버그 발행 → 가짜 로봇 팔 목표 변경", code == 200 and bool(got), "응답=%s target=%s" % (res, rs.get("arm_target")))
    api.cmd("arm_home")
    time.sleep(0.4)
    lr = api.state()["rosbridge"].get("last_result") or {}
    check("세션 없을 때 home 서비스 → 성공", lr.get("service") == "/armpi/arm/home" and lr.get("ok") is True, "%s" % lr)
    api.cmd("connect")
    got = wait_for(lambda: (lambda s: s if s["udp"]["connected"] and not s["udp"]["retired"]
                            and (s["rosbridge"]["status"] or {}).get("owner") == s["udp"]["client_id"] else None)(api.state()), 2.0)
    st = api.state()
    check("연결(새 세션) → 새 UUID 로 소유권 획득", bool(got) and st["udp"]["client_id"] != cid_old,
          "%s → %s" % (cid_old[:8], st["udp"]["client_id"][:8]))
    code, res = api.cmd("joint_debug", positions=[0, 0, 0, 0, 0, 0])
    check("세션이 있으면 관절 디버그 거부", code == 400, "%s %s" % (code, res))

    # 8. 카메라(--camera "" 이면 건너뜀: 카메라 없이 띄운 ROS 가짜 스택에서 C++ 게이트웨이를 시험할 때)
    if not args.camera:
        print("[SKIP] 카메라 스트림(--camera 빈 값)", flush=True)
    else:
        try:
            with urllib.request.urlopen(args.camera + "/stream?topic=/usb_cam/image_raw&type=ros_compressed&qos_profile=sensor_data",
                                        timeout=3) as r:
                head = r.read(4096)
                ctype = r.headers.get("Content-Type", "")
            check("카메라 스트림(가짜, PNG multipart)", "multipart/x-mixed-replace" in ctype and b"\x89PNG" in head,
                  "Content-Type=%s" % ctype)
        except OSError as exc:
            check("카메라 스트림(가짜, PNG multipart)", False, str(exc))

    # 9. 요청 출처 검사
    code, _ = api.req("/api/cmd", {"cmd": "estop"}, headers={"Origin": "http://evil.example"})
    check("다른 Origin 의 POST 거부(403)", code == 403, str(code))
    r = urllib.request.Request(api.base + "/api/cmd", data=b'{"cmd":"estop"}', headers={"Content-Type": "text/plain"})
    try:
        urllib.request.urlopen(r, timeout=3)
        code = 200
    except urllib.error.HTTPError as exc:
        code = exc.code
    check("JSON 이 아닌 Content-Type POST 거부(403)", code == 403, str(code))
    code, _ = api.req("/api/state", headers={"Host": "evil.example:18765"})
    check("다른 Host 헤더 GET 거부(403)", code == 403, str(code))
    check("위 거부 요청으로 비상정지가 걸리지 않음", not api.state()["udp"]["emergency"])

    br.running = False
    st = api.state()
    summary = {"pass": sum(r["pass"] for r in RESULTS), "fail": sum(not r["pass"] for r in RESULTS),
               "seconds": round(time.time() - t_start, 1), "final_udp": st["udp"], "loop_overruns": st["loop_overruns"]}
    print("\n요약: 통과 %d / 실패 %d (%.1f s), ACK 지연 중앙 %s ms 최대 %s ms, 루프 지연 %d회"
          % (summary["pass"], summary["fail"], summary["seconds"], st["udp"]["ack_ms_p50"], st["udp"]["ack_ms_max"],
             st["loop_overruns"]))
    if args.json:
        with open(args.json, "w", encoding="utf-8") as fp:
            json.dump({"summary": summary, "checks": RESULTS}, fp, ensure_ascii=False, indent=2)
    return 0 if summary["fail"] == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
