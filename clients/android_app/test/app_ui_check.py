#!/usr/bin/env python3
"""Android 앱 안 조종 화면(assets/app_ui/control.html) 시험 — 실제 로봇·휴대폰에 접속하지 않는다.

구성
  - 도커 컨테이너(armpi-ctl-dev:humble): 지금 저장소의 C++ armpi_web_gateway(로봇 v2.3 과 같은 API) +
    가짜 로봇(protocol_v3/mock_robot.py: UDP 5001, 가짜 카메라 8080, 가짜 rosbridge 9090).
    포트는 이 맥의 127.0.0.1 에만 연다(18765·8080·9090).
  - 헤드리스 Chrome(DevTools 프로토콜): 앱과 같은 방식으로 화면을 띄운다.
      http://127.0.0.1:18765/app-ui/control.html?video=<앱이 만드는 영상 주소>
    이 경로는 Fetch 가로채기로 assets 파일을 내준다(Android 의 shouldInterceptRequest 와 같은 규칙,
    AppUiPolicy 를 파이썬으로 옮긴 decide()). 나머지 요청은 그대로 게이트웨이·가짜 카메라로 간다.
  - 확인: 배치(가로 800×360·세로 360×800, DPR 3 → 캡처 2400×1080·1080×2400, 그리고 915×412·412×915),
    이동 입력 → /api/input → 가짜 로봇 base_cmd, 손 떼면 중립, 팔 패널 열고 닫을 때 중립,
    화면 돌리면 중립, JS 가 멈추면 게이트웨이가 입력 끊김으로 처리, 비상정지 송신·latch·길게 눌러 해제,
    메뉴·카메라 크게 보기 중에도 비상정지 버튼이 보이는지, 다른 기기 조종 중 안내, 허용 밖 요청 0건,
    file:// 화면(Origin: null)이면 게이트웨이가 POST 를 거부한다는 근거.

실행: /usr/bin/python3 -B clients/android_app/test/app_ui_check.py [--out <폴더>] [--keep]
결과: work/logs/app_ui_v1_1_<시각>/ (results.json, summary.txt, 캡처 PNG, chrome·gateway·mock 로그)
"""

import argparse
import base64
import json
import os
import queue
import re
import shutil
import signal
import socket
import struct
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.parse
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
APP = os.path.abspath(os.path.join(HERE, ".."))
ROOT = os.path.abspath(os.path.join(APP, "..", ".."))
ASSETS = os.path.join(APP, "assets")
CHROME = "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome"
IMAGE = os.environ.get("ARMPI_CTL_DEV_IMAGE", "armpi-ctl-dev:humble")
HOST = "127.0.0.1"
WEB_PORT, CAM_PORT, RB_PORT = 18765, 8080, 9090
UA = ("Mozilla/5.0 (Linux; Android 14; SM-G781N) AppleWebKit/537.36 (KHTML, like Gecko) "
      "Chrome/130.0.0.0 Mobile Safari/537.36 ArmPiController-Android/1.1")
VIDEO_URL = "http://%s:%d/stream?topic=/usb_cam/image_raw&type=ros_compressed&qos_profile=sensor_data" % (HOST, CAM_PORT)
PAGE_URL = "http://%s:%d/app-ui/control.html?video=%s" % (HOST, WEB_PORT, urllib.parse.quote(VIDEO_URL, safe=""))
RESULTS = []
# 데스크톱 Chrome 의 로컬 네트워크 접근(LNA) 검사: 가로채서 내준 화면은 주소 공간이 '공용'으로 잡혀
# 다른 포트(8080)의 영상 요청을 막는다. Android WebView 처럼 이 검사를 끄고 시험한다(--lna-check 로 켠 결과도 기록).
LNA_FEATURES = "LocalNetworkAccessChecks,LocalNetworkAccessChecksWebRTC"


def check(phase, name, cond, detail=""):
    RESULTS.append({"phase": phase, "check": name, "pass": bool(cond), "detail": str(detail)[:700]})
    print("[%s] %s | %s  %s" % ("PASS" if cond else "FAIL", phase, name, str(detail)[:240]), flush=True)
    return bool(cond)


def wait_for(fn, timeout=3.0, period=0.05):
    end = time.monotonic() + timeout
    last = None
    while time.monotonic() < end:
        try:
            last = fn()
        except Exception:  # noqa: BLE001  (시험 중 일시 오류는 다시 시도)
            last = None
        if last:
            return last
        time.sleep(period)
    return last


# ───────────────────────── 앱의 요청 허용 규칙(AppUiPolicy.java 와 같은 규칙) ─────────────────────────
ASSET_NAME = re.compile(r"^[a-z0-9_-]{1,64}\.(html|js|css|png|svg)$")
MIME = {"html": "text/html", "js": "text/javascript", "css": "text/css", "png": "image/png", "svg": "image/svg+xml"}


def decide(url, host=HOST, web_port=WEB_PORT):
    u = urllib.parse.urlsplit(url)
    s = (u.scheme or "").lower()
    if s in ("about", "data", "blob"):
        return ("NETWORK", None)
    if s != "http" or (u.hostname or "").lower() != host:
        return ("BLOCK", None)
    port = u.port or 80
    path = u.path or "/"
    if ".." in path or "\\" in path or "%" in path:
        return ("BLOCK", None)
    if port == web_port:
        if path.startswith("/app-ui/"):
            name = path[len("/app-ui/"):]
            return ("ASSET", "app_ui/" + name) if ASSET_NAME.match(name) else ("BLOCK", None)
        if path.startswith("/api/"):
            return ("NETWORK", None)
        return ("BLOCK", None)
    if port == CAM_PORT and path == "/stream":
        return ("NETWORK", None)
    return ("BLOCK", None)


# ───────────────────────── HTTP(게이트웨이) ─────────────────────────
def http(method, path, body=None, headers=None, port=WEB_PORT):
    h = {"Host": "%s:%d" % (HOST, port)}
    data = None
    if body is not None:
        data = json.dumps(body).encode()
        h["Content-Type"] = "application/json"
    h.update(headers or {})
    req = urllib.request.Request("http://%s:%d%s" % (HOST, port, path), data=data, headers=h, method=method)
    try:
        with urllib.request.urlopen(req, timeout=3) as r:
            raw = r.read()
            return r.status, raw, dict(r.headers)
    except urllib.error.HTTPError as e:
        return e.code, e.read(), dict(e.headers)


def state(screen="pytest"):
    st, raw, _ = http("GET", "/api/state?screen=" + screen)
    return json.loads(raw) if st == 200 else None


# ───────────────────────── 가짜 로봇 상태(가짜 rosbridge 9090, /armpi/status) ─────────────────────────
class WS:
    """아주 작은 WebSocket 클라이언트(마스킹 송신, 조각 수신)."""

    def __init__(self, host, port, path="/"):
        self.sock = socket.create_connection((host, port), timeout=10)
        key = base64.b64encode(os.urandom(16)).decode()
        self.sock.sendall(("GET %s HTTP/1.1\r\nHost: %s:%d\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
                           "Sec-WebSocket-Key: %s\r\nSec-WebSocket-Version: 13\r\n\r\n" % (path, host, port, key)).encode())
        resp = b""
        while b"\r\n\r\n" not in resp:
            chunk = self.sock.recv(4096)
            if not chunk:
                raise OSError("WebSocket 연결 종료")
            resp += chunk
        if b" 101 " not in resp.split(b"\r\n")[0]:
            raise OSError("WebSocket handshake 실패: %r" % resp[:80])
        self.buf = resp.split(b"\r\n\r\n", 1)[1]
        self.wlock = threading.Lock()

    def _exact(self, n):
        while len(self.buf) < n:
            chunk = self.sock.recv(65536)
            if not chunk:
                raise OSError("WebSocket 연결 종료")
            self.buf += chunk
        out, self.buf = self.buf[:n], self.buf[n:]
        return out

    def recv(self):
        parts = []
        while True:
            b0, b1 = self._exact(2)
            n = b1 & 0x7F
            if n == 126:
                n = struct.unpack("!H", self._exact(2))[0]
            elif n == 127:
                n = struct.unpack("!Q", self._exact(8))[0]
            data = self._exact(n)
            op = b0 & 0x0F
            if op == 8:
                raise OSError("WebSocket close")
            if op in (9, 10):
                continue
            parts.append(data)
            if b0 & 0x80:
                return b"".join(parts)

    def send(self, text):
        payload = text.encode() if isinstance(text, str) else text
        mask = os.urandom(4)
        n = len(payload)
        hdr = bytearray([0x81])
        if n < 126:
            hdr.append(0x80 | n)
        elif n < 65536:
            hdr.append(0x80 | 126)
            hdr += struct.pack("!H", n)
        else:
            hdr.append(0x80 | 127)
            hdr += struct.pack("!Q", n)
        with self.wlock:
            self.sock.sendall(bytes(hdr) + mask + bytes(b ^ mask[i % 4] for i, b in enumerate(payload)))

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass


class RobotStatus(threading.Thread):
    """가짜 로봇의 /armpi/status 를 계속 받아 최신값을 보관한다."""

    def __init__(self):
        super().__init__(daemon=True)
        self.ws = WS(HOST, RB_PORT)
        self.ws.send(json.dumps({"op": "subscribe", "topic": "/armpi/status", "type": "std_msgs/String"}))
        self.last = None
        self.lock = threading.Lock()
        self.running = True

    def run(self):
        while self.running:
            try:
                m = json.loads(self.ws.recv())
            except (OSError, ValueError):
                break
            if m.get("topic") == "/armpi/status":
                with self.lock:
                    self.last = json.loads(m["msg"]["data"])

    def get(self):
        with self.lock:
            return self.last

    def stop(self):
        self.running = False
        self.ws.close()


# ───────────────────────── Chrome DevTools(이벤트 처리 + 요청 가로채기) ─────────────────────────
class CDP:
    def __init__(self, ws_url):
        assert ws_url.startswith("ws://")
        hostport, path = ws_url[5:].split("/", 1)
        h, p = hostport.split(":")
        self.ws = WS(h, int(p), "/" + path)
        self.id = 0
        self.idlock = threading.Lock()
        self.pending = {}
        self.events = queue.Queue()
        self.handlers = {}
        self.alive = True
        threading.Thread(target=self._reader, daemon=True).start()
        threading.Thread(target=self._dispatch, daemon=True).start()

    def _reader(self):
        while self.alive:
            try:
                msg = json.loads(self.ws.recv())
            except (OSError, ValueError):
                self.alive = False
                break
            if "id" in msg:
                q = self.pending.pop(msg["id"], None)
                if q:
                    q.put(msg)
            else:
                self.events.put(msg)

    def _dispatch(self):
        while True:
            msg = self.events.get()
            if msg is None:
                return
            fn = self.handlers.get(msg.get("method"))
            if fn:
                try:
                    fn(msg.get("params", {}))
                except Exception as e:  # noqa: BLE001
                    print("CDP 이벤트 처리 오류:", msg.get("method"), e, flush=True)

    def _next(self):
        with self.idlock:
            self.id += 1
            return self.id

    def send(self, method, params=None, timeout=15):
        i = self._next()
        q = queue.Queue()
        self.pending[i] = q
        self.ws.send(json.dumps({"id": i, "method": method, "params": params or {}}))
        try:
            msg = q.get(timeout=timeout)
        except queue.Empty:
            raise OSError("CDP %s 응답 없음" % method)
        if "error" in msg:
            raise OSError("CDP %s 오류: %s" % (method, msg["error"]))
        return msg.get("result", {})

    def send_nowait(self, method, params=None):
        i = self._next()
        self.ws.send(json.dumps({"id": i, "method": method, "params": params or {}}))

    def on(self, method, fn):
        self.handlers[method] = fn

    def eval(self, expr):
        r = self.send("Runtime.evaluate", {"expression": expr, "returnByValue": True, "awaitPromise": True})
        if r.get("exceptionDetails"):
            raise OSError("JS 오류: %s" % json.dumps(r["exceptionDetails"])[:300])
        return (r.get("result") or {}).get("value")

    def shot(self, path):
        data = self.send("Page.captureScreenshot", {"format": "png", "captureBeyondViewport": False})["data"]
        raw = base64.b64decode(data)
        with open(path, "wb") as fp:
            fp.write(raw)
        w, h = struct.unpack("!II", raw[16:24])
        return w, h

    def touch(self, kind, points):
        pts = [{"x": x, "y": y, "id": i, "radiusX": 6, "radiusY": 6, "force": 1} for i, (x, y) in points]
        self.send("Input.dispatchTouchEvent", {"type": kind, "touchPoints": pts})

    def rect(self, sel):
        return self.eval("(() => { const e = document.querySelector(%s); if (!e) return null; const r = e.getBoundingClientRect();"
                         " return {x:r.left, y:r.top, w:r.width, h:r.height, r:r.right, b:r.bottom,"
                         " cx:r.left + r.width/2, cy:r.top + r.height/2}; })()" % json.dumps(sel))

    def metrics(self, width, height, dpr):
        self.send("Emulation.setDeviceMetricsOverride", {
            "width": width, "height": height, "deviceScaleFactor": dpr, "mobile": True,
            "screenOrientation": {"type": "landscapePrimary" if width > height else "portraitPrimary",
                                  "angle": 90 if width > height else 0}})

    def close(self):
        self.alive = False
        self.events.put(None)
        self.ws.close()


class Chrome:
    def __init__(self, out, port, lna=False):
        self.port = port
        self.prof = os.path.join(out, "chrome_profile")
        self.log = open(os.path.join(out, "chrome.log"), "w")
        self.p = subprocess.Popen([CHROME, "--headless=new", "--disable-gpu", "--hide-scrollbars", "--no-first-run",
                                   "--no-default-browser-check", "--disable-extensions", "--disable-background-networking",
                                   "--disable-component-update", "--disable-sync", "--metrics-recording-only",
                                   "--remote-debugging-address=127.0.0.1", "--remote-debugging-port=%d" % port,
                                   *(["--disable-features=" + LNA_FEATURES] if not lna else []),
                                   "--remote-allow-origins=http://127.0.0.1:%d" % port,
                                   "--user-data-dir=" + self.prof, "about:blank"],
                                  stdout=self.log, stderr=subprocess.STDOUT)
        ok = wait_for(lambda: urllib.request.urlopen("http://127.0.0.1:%d/json/version" % port, timeout=1).status == 200, 20)
        if not ok:
            raise OSError("헤드리스 Chrome 이 뜨지 않음")

    def page(self):
        req = urllib.request.Request("http://127.0.0.1:%d/json/new?about:blank" % self.port, method="PUT")
        info = json.loads(urllib.request.urlopen(req, timeout=5).read())
        return CDP(info["webSocketDebuggerUrl"])

    def quit(self):
        if self.p.poll() is None:
            self.p.send_signal(signal.SIGTERM)   # 이 시험이 띄운 PID 만 종료
            try:
                self.p.wait(8)
            except subprocess.TimeoutExpired:
                self.p.kill()
                self.p.wait()
        self.log.close()
        shutil.rmtree(self.prof, ignore_errors=True)


class Traffic:
    """Fetch 가로채기: 앱 규칙대로 assets 를 주고, 나머지는 통과/차단. API 본문을 기록한다."""

    def __init__(self, cdp):
        self.cdp = cdp
        self.lock = threading.Lock()
        self.log = []          # (t, kind, method, url, body_json)
        self.blocked = []
        self.assets = []
        cdp.on("Fetch.requestPaused", self._paused)

    def _paused(self, p):
        req = p["request"]
        url = req["url"]
        kind, asset = decide(url)
        body = None
        if req.get("postData"):
            try:
                body = json.loads(req["postData"])
            except ValueError:
                body = req["postData"]
        with self.lock:
            self.log.append((time.monotonic(), kind, req["method"], url, body))
        if kind == "ASSET":
            path = os.path.join(ASSETS, asset)
            try:
                with open(path, "rb") as fp:
                    data = fp.read()
                ext = asset.rsplit(".", 1)[-1]
                with self.lock:
                    self.assets.append(asset)
                self.cdp.send_nowait("Fetch.fulfillRequest", {
                    "requestId": p["requestId"], "responseCode": 200,
                    "responseHeaders": [{"name": "Content-Type", "value": MIME.get(ext, "application/octet-stream") + "; charset=utf-8"},
                                        {"name": "Cache-Control", "value": "no-store"}],
                    "body": base64.b64encode(data).decode()})
            except OSError:
                self.cdp.send_nowait("Fetch.fulfillRequest", {"requestId": p["requestId"], "responseCode": 404, "body": ""})
        elif kind == "NETWORK":
            self.cdp.send_nowait("Fetch.continueRequest", {"requestId": p["requestId"]})
        else:
            with self.lock:
                self.blocked.append(url)
            self.cdp.send_nowait("Fetch.fulfillRequest", {"requestId": p["requestId"], "responseCode": 403, "body": ""})

    def bodies(self, path, since=0.0):
        with self.lock:
            return [(t, b) for (t, k, m, u, b) in self.log
                    if t >= since and m == "POST" and urllib.parse.urlsplit(u).path == path and isinstance(b, dict)]

    def mark(self):
        return time.monotonic()


# ───────────────────────── 도커: C++ 게이트웨이 + 가짜 로봇 ─────────────────────────
def port_free(port):
    """이미 듣고 있는 프로그램이 있으면 False. 직전 시험의 TIME_WAIT 은 무시(SO_REUSEADDR)."""
    s = socket.socket()
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        s.bind((HOST, port))
        return True
    except OSError:
        return False
    finally:
        s.close()


def start_container(name, out):
    script = r"""
set -e
source /opt/ros/humble/setup.bash
mkdir -p /tmp/ws/src && cp -r /src/. /tmp/ws/src/ && cd /tmp/ws
colcon build --merge-install --packages-up-to armpi_web_gateway --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release \
  > /tmp/build.log 2>&1 || { tail -60 /tmp/build.log; exit 1; }
(cd /tmp/ws/src && find armpi_common armpi_protocol armpi_web_gateway -type f | sort | xargs sha256sum) > /tmp/source_sha256.txt
echo "[appui] 빌드 완료"
source install/setup.bash
python3 -B /v3/mock_robot.py --bind 0.0.0.0 --discovery-port 0 --stats-period 30 > /tmp/mock.log 2>&1 &
sleep 1
exec ros2 run armpi_web_gateway armpi_web_gateway_node --ros-args -p http_bind:=0.0.0.0 -p http_port:=18765 \
  -p robot:=127.0.0.1 -p session:=on-demand > /tmp/gateway.log 2>&1
"""
    cmd = ["docker", "run", "-d", "--name", name,
           "-p", "127.0.0.1:%d:%d" % (WEB_PORT, WEB_PORT), "-p", "127.0.0.1:%d:%d" % (CAM_PORT, CAM_PORT),
           "-p", "127.0.0.1:%d:%d" % (RB_PORT, RB_PORT),
           "-v", os.path.join(ROOT, "ros2_ws", "src") + ":/src:ro", "-v", os.path.join(ROOT, "protocol_v3") + ":/v3:ro",
           "-e", "ROS_LOCALHOST_ONLY=1", "-e", "ROS_DOMAIN_ID=83", "-e", "PYTHONDONTWRITEBYTECODE=1",
           IMAGE, "bash", "-c", script]
    subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL)
    t0 = time.monotonic()
    while time.monotonic() - t0 < 900:
        logs = subprocess.run(["docker", "logs", name], capture_output=True, text=True).stdout
        running = subprocess.run(["docker", "inspect", "-f", "{{.State.Running}}", name], capture_output=True, text=True).stdout.strip()
        if running != "true":
            with open(os.path.join(out, "container_failed.log"), "w") as fp:
                fp.write(logs)
            raise OSError("컨테이너가 멈춤:\n" + logs[-2000:])
        if "빌드 완료" in logs:
            try:
                st, raw, _ = http("GET", "/api/config")
                if st == 200:
                    return time.monotonic() - t0
            except OSError:
                pass
        time.sleep(1.0)
    raise OSError("게이트웨이가 15 분 안에 뜨지 않음")


def stop_container(name, out):
    for src, dst in (("/tmp/gateway.log", "gateway.log"), ("/tmp/mock.log", "mock_robot.log"),
                     ("/tmp/build.log", "gateway_build.log"), ("/tmp/source_sha256.txt", "gateway_source_sha256.txt")):
        subprocess.run(["docker", "cp", "%s:%s" % (name, src), os.path.join(out, dst)], capture_output=True)
    subprocess.run(["docker", "rm", "-f", name], capture_output=True)


# ───────────────────────── 화면 조작 도우미 ─────────────────────────
LAYOUT_JS = r"""
(() => {
  const vis = e => { if (!e) return false; const cs = getComputedStyle(e); if (cs.display === 'none' || cs.visibility === 'hidden') return false;
    const r = e.getBoundingClientRect(); return r.width > 0 && r.height > 0 && !e.closest('[hidden]') && e.offsetParent !== null; };
  const R = s => { const e = document.querySelector(s); if (!vis(e)) return null; const r = e.getBoundingClientRect();
    return {x:r.left, y:r.top, w:r.width, h:r.height, r:r.right, b:r.bottom}; };
  const W = innerWidth, H = innerHeight;
  const inside = r => r && r.x >= -0.5 && r.y >= -0.5 && r.r <= W + 0.5 && r.b <= H + 0.5;
  const small = [];
  document.querySelectorAll('button').forEach(b => { if (!vis(b)) return; const r = b.getBoundingClientRect();
    if (r.width < 44 - 0.5 || r.height < 44 - 0.5) small.push((b.id || b.textContent.trim().slice(0, 8)) + ' ' + Math.round(r.width) + 'x' + Math.round(r.height)); });
  const outside = [];
  document.querySelectorAll('#app button, #app .joy, #app .turn').forEach(b => { if (!vis(b)) return; const r = b.getBoundingClientRect();
    if (!inside({x:r.left, y:r.top, r:r.right, b:r.bottom})) outside.push(b.id || b.className); });
  const es = document.querySelector('#bStop').getBoundingClientRect();
  const hit = document.elementFromPoint(es.left + es.width / 2, es.top + es.height / 2);
  const vid = document.querySelector('#video');
  return {W, H, cam:R('#cam'), stop:R('#stop'), move:R('#move'), arm:R('#arm'), joy:R('#joy'), turn:R('#turn'), estop:R('#bStop'),
    clear:R('#bClear'), armBtn:R('#bArmPanel'), menu:R('#bMenu'), camBig:R('#bCamBig'), sheetBox:R('#sheet .box'),
    scrollW:document.documentElement.scrollWidth, scrollH:document.documentElement.scrollHeight,
    small, outside, estopOnTop: !!(hit && hit.closest('#bStop')),
    novideoHidden: document.querySelector('#novideo').hidden, videoNatural:[vid.naturalWidth, vid.naturalHeight], videoSrc: vid.getAttribute('src'),
    link: document.querySelector('#cLink').textContent, estopChip: !document.querySelector('#cEstop').hidden,
    speedOn: [...document.querySelectorAll('#speed button.on')].map(b => b.dataset.v),
    armOpen: document.body.classList.contains('arm-open'), camBigOn: document.body.classList.contains('cam-big'),
    locked: document.body.classList.contains('locked'), estopped: document.body.classList.contains('estopped'),
    msg: document.querySelector('#msg').textContent, armEnabledUi: !document.body.classList.contains('armoff')};
})()
"""


def overlap(a, b):
    return a and b and not (a["r"] <= b["x"] + 0.5 or b["r"] <= a["x"] + 0.5 or a["b"] <= b["y"] + 0.5 or b["b"] <= a["y"] + 0.5)


def check_layout(ph, cdp, portrait, label):
    L = cdp.eval(LAYOUT_JS)
    check(ph, "%s: 가로 스크롤·세로 넘침 없음" % label, L["scrollW"] <= L["W"] and L["scrollH"] <= L["H"],
          "scroll %sx%s / view %sx%s" % (L["scrollW"], L["scrollH"], L["W"], L["H"]))
    check(ph, "%s: 버튼·조작부가 화면 안" % label, not L["outside"], L["outside"])
    check(ph, "%s: 보이는 버튼 모두 44 px 이상" % label, not L["small"], L["small"])
    cam, stop, move = L["cam"], L["stop"], L["move"]
    check(ph, "%s: 카메라·비상정지·이동 칸이 겹치지 않음" % label,
          cam and stop and move and not overlap(cam, stop) and not overlap(cam, move) and not overlap(stop, move),
          {"cam": cam, "stop": stop, "move": move})
    if portrait:
        check(ph, "%s: 세로 = 카메라 / 비상정지 / 이동(아래)" % label, cam["y"] < stop["y"] < move["y"] and move["b"] > L["H"] * 0.85,
              "y %.0f < %.0f < %.0f, 이동 칸 아래 %.0f/%d" % (cam["y"], stop["y"], move["y"], move["b"], L["H"]))
        ratio = cam["w"] / cam["h"]
        check(ph, "%s: 카메라 칸 4:3" % label, abs(ratio - 4 / 3) < 0.03, "%.0fx%.0f (%.3f)" % (cam["w"], cam["h"], ratio))
        check(ph, "%s: '크게' 버튼은 세로에서 숨김" % label, L["camBig"] is None)
    else:
        check(ph, "%s: 가로 = 이동 | 카메라 | 비상정지(오른쪽 끝)" % label,
              move["x"] < cam["x"] < stop["x"] and L["W"] - stop["r"] < 20,
              "x %.0f < %.0f < %.0f, 오른쪽 여백 %.0f" % (move["x"], cam["x"], stop["x"], L["W"] - stop["r"]))
    j = L["joy"]
    check(ph, "%s: 조이스틱 지름 ≥ 120 px, 비상정지 지름 ≥ 88 px" % label, j and j["w"] >= 120 and L["estop"]["w"] >= 88,
          "조이스틱 %.0f, 비상정지 %.0f" % (j["w"] if j else -1, L["estop"]["w"]))
    check(ph, "%s: 비상정지 버튼이 맨 위(가려지지 않음)" % label, L["estopOnTop"])
    return L


def tap(cdp, sel, hold=0.08, finger=7):
    r = cdp.rect(sel)
    cdp.touch("touchStart", [(finger, (r["cx"], r["cy"]))])
    time.sleep(hold)
    cdp.touch("touchEnd", [])
    return r


def second_finger_tap(cdp, sel):
    """다른 손가락이 누르고 있는 동안 두 번째 손가락으로 버튼 탭.
    CDP 터치는 손가락 하나만 떼지 못해서(touchEnd = 모두 뗌) 두 번째 손가락은 JS 포인터 이벤트로 흉내 낸다."""
    return cdp.eval("""(() => { const b = document.querySelector(%s); const r = b.getBoundingClientRect();
      const o = {pointerId: 99, pointerType: 'touch', isPrimary: false, bubbles: true, cancelable: true,
                 clientX: r.left + r.width / 2, clientY: r.top + r.height / 2};
      b.dispatchEvent(new PointerEvent('pointerdown', o)); b.dispatchEvent(new PointerEvent('pointerup', o)); return true; })()""" % json.dumps(sel))


def robot_cmd(rs):
    s = rs.get() or {}
    return s.get("base_cmd") or {}


# ───────────────────────── 시험 단계 ─────────────────────────
def phase_http(out):
    ph = "HTTP"
    st, raw, hd = http("GET", "/api/config")
    cfg = json.loads(raw)
    with open(os.path.join(out, "api_config.json"), "w") as fp:
        json.dump(cfg, fp, ensure_ascii=False, indent=1)
    check(ph, "/api/config: on-demand, UDP 5001, ui.default_speed 0.6", st == 200 and cfg.get("session_mode") == "on-demand"
          and cfg.get("port") == 5001 and (cfg.get("ui") or {}).get("default_speed") == 0.6, {k: cfg.get(k) for k in ("session_mode", "port", "ui")})
    check(ph, "/api/config video_url 은 로봇 안 주소(127.0.0.1) → 앱은 쓰지 않고 IP 로 만든 주소 사용",
          "127.0.0.1:8080" in cfg.get("video_url", ""), cfg.get("video_url"))
    check(ph, "게이트웨이에 CORS 헤더 없음(그래서 file:// 화면은 쓸 수 없음)",
          not any(k.lower().startswith("access-control") for k in hd), list(hd))
    st, raw, _ = http("POST", "/api/input", {"screen": "originnull", "f": 0, "s": 0, "t": 0, "held": False, "focused": True},
                      headers={"Origin": "null"})
    check(ph, "Origin: null(file:// 화면) POST → 403 거부", st == 403, raw[:120])
    st, raw, _ = http("POST", "/api/input", {"screen": "sameorigin", "f": 0, "s": 0, "t": 0, "held": False, "focused": True},
                      headers={"Origin": "http://%s:%d" % (HOST, WEB_PORT)})
    check(ph, "Origin: http://<로봇>:18765(앱 화면과 같은 출처) POST → 200", st == 200, raw[:120])
    st, _, _ = http("OPTIONS", "/api/cmd")
    check(ph, "OPTIONS(사전 요청) 미지원 501 — 다른 출처에서 JSON POST 불가", st == 501, st)
    st, _, _ = http("GET", "/app-ui/control.html")
    check(ph, "로봇에는 /app-ui/ 경로가 없음(404) — 화면은 앱 assets 에서만", st == 404, st)
    req = urllib.request.Request(VIDEO_URL, headers={"Host": "%s:%d" % (HOST, CAM_PORT)})
    with urllib.request.urlopen(req, timeout=3) as r:
        ctype = r.headers.get("Content-Type", "")
        head = r.read(64)
    check(ph, "앱이 만드는 영상 주소(8080 /stream, sensor_data) → multipart 스트림", "multipart/x-mixed-replace" in ctype and head.startswith(b"--frame"), ctype)


def phase_ui(out, chrome, rs):
    ph = "화면"
    cdp = chrome.page()
    tr = Traffic(cdp)
    cdp.send("Page.enable")
    cdp.send("Runtime.enable")
    cdp.send("Fetch.enable", {"patterns": [{"urlPattern": "*", "requestStage": "Request"}]})
    cdp.send("Emulation.setUserAgentOverride", {"userAgent": UA})
    cdp.send("Emulation.setTouchEmulationEnabled", {"enabled": True, "maxTouchPoints": 5})
    cdp.metrics(800, 360, 3)
    cdp.send("Page.navigate", {"url": PAGE_URL})
    ok = wait_for(lambda: cdp.eval("document.readyState") == "complete" and cdp.eval("!!document.querySelector('#bStop')"), 10)
    check(ph, "앱 화면 주소 → assets/app_ui/control.html 이 열림", ok and "app_ui/control.html" in tr.assets, tr.assets)
    origin = cdp.eval("location.origin")
    check(ph, "화면 출처 = 로봇 웹 조종기와 같은 http://127.0.0.1:18765", origin == "http://%s:%d" % (HOST, WEB_PORT), origin)
    got = wait_for(lambda: "대기" in cdp.eval("document.querySelector('#cLink').textContent"), 6)
    check(ph, "/api/config·/api/state 받음 → '대기 · 조작하면 연결'", bool(got), cdp.eval("document.querySelector('#cLink').textContent"))
    vid = wait_for(lambda: (lambda v: v if v and v[0] > 0 else None)(cdp.eval("[document.querySelector('#video').naturalWidth, document.querySelector('#video').naturalHeight]")), 6)
    src = cdp.eval("document.querySelector('#video').getAttribute('src')") or ""
    check(ph, "영상: 앱이 넘긴 주소(IP:8080 sensor_data)로 받아 표시", bool(vid) and src.startswith(VIDEO_URL), "%s %s" % (vid, src))
    wait_for(lambda: cdp.eval("document.querySelector('#novideo').hidden"), 3)
    check(ph, "영상이 오면 '영상 없음' 숨김", cdp.eval("document.querySelector('#novideo').hidden"))
    check(ph, "속도 기본값 = 로봇 설정 60 %", cdp.eval("[...document.querySelectorAll('#speed button.on')].map(b => b.dataset.v).join()") == "0.6")
    check(ph, "앱 표시(UA) → '전체' 버튼 숨김", cdp.eval("document.querySelector('#bFull').hidden"))

    shots = {}

    def shot(name):
        time.sleep(0.45)
        wait_for(lambda: not cdp.eval("document.querySelector('#toast').classList.contains('show')"), 5)   # 앞 조작의 알림이 사라진 뒤
        path = os.path.join(out, name)
        w, h = cdp.shot(path)
        shots[name] = [w, h]
        return w, h

    # ── 가로 800×360 ──
    L = check_layout(ph, cdp, False, "가로 800×360")
    w, h = shot("shot_landscape_2400x1080_1_basic.png")
    check(ph, "가로 캡처 2400×1080", (w, h) == (2400, 1080), (w, h))

    # 이동: 조이스틱 위로 → /api/input f>0 → 가짜 로봇 vx>0 → 손 떼면 중립
    j = cdp.rect("#joy")
    t0 = tr.mark()
    cdp.touch("touchStart", [(0, (j["cx"], j["cy"]))])
    for k in range(1, 6):
        cdp.touch("touchMove", [(0, (j["cx"], j["cy"] - j["h"] * 0.08 * k))])
        time.sleep(0.03)
    moving = wait_for(lambda: robot_cmd(rs).get("vx", 0) > 0.05, 3)
    ins = tr.bodies("/api/input", t0)
    held = [b for _, b in ins if b.get("held") and b.get("f", 0) > 0.2]
    check(ph, "조이스틱 앞 → /api/input held·f>0", bool(held), held[-1] if held else ins[-3:])
    check(ph, "→ 게이트웨이 → 가짜 로봇 차체 명령 vx>0", bool(moving), robot_cmd(rs))
    t_rel = tr.mark()
    cdp.touch("touchEnd", [])
    stopped = wait_for(lambda: robot_cmd(rs) == {"vx": 0.0, "vy": 0.0, "wz": 0.0}, 2, 0.02)
    dt = (time.monotonic() - t_rel) * 1e3
    after = tr.bodies("/api/input", t_rel)
    check(ph, "손 뗌 → 바로 중립 입력(held=false, f=s=t=0)", bool(after) and not after[0][1]["held"]
          and after[0][1]["f"] == 0 and after[0][1]["s"] == 0 and after[0][1]["t"] == 0, after[:1])
    check(ph, "→ 가짜 로봇 차체 명령 0", bool(stopped), "%.0f ms(상태 토픽 경유)" % dt)

    # 회전 슬라이더
    t = cdp.rect("#turn")
    t0 = tr.mark()
    cdp.touch("touchStart", [(1, (t["cx"], t["cy"]))])
    cdp.touch("touchMove", [(1, (t["x"] + t["w"] * 0.1, t["cy"]))])
    turning = wait_for(lambda: robot_cmd(rs).get("wz", 0) > 0.05, 3)
    cdp.touch("touchEnd", [])
    check(ph, "회전 슬라이더 왼쪽 → 좌회전(wz>0), 손 떼면 0",
          bool(turning) and bool(wait_for(lambda: robot_cmd(rs).get("wz") == 0.0, 2)), robot_cmd(rs))

    # 팔 패널: 조이스틱을 누른 채 '팔 조종'을 다른 손가락으로 → 중립 + 패널 열림
    a = cdp.rect("#bArmPanel")
    cdp.touch("touchStart", [(0, (j["cx"], j["cy"] - j["h"] * 0.3))])
    wait_for(lambda: robot_cmd(rs).get("vx", 0) > 0.05, 3)
    t_open = tr.mark()
    second_finger_tap(cdp, "#bArmPanel")
    opened = wait_for(lambda: cdp.eval("document.body.classList.contains('arm-open')"), 2)
    neutral = wait_for(lambda: any(not b["held"] and b["f"] == 0 for _, b in tr.bodies("/api/input", t_open)), 1.5)
    check(ph, "조이스틱을 누른 채(터치) 다른 손가락으로 '팔 조종' → 패널 열림", bool(opened))
    check(ph, "패널 열 때 누르던 이동 입력 중립(held=false)", bool(neutral), tr.bodies("/api/input", t_open)[:2])
    check(ph, "→ 가짜 로봇 차체 명령 0", bool(wait_for(lambda: robot_cmd(rs) == {"vx": 0.0, "vy": 0.0, "wz": 0.0}, 2)), robot_cmd(rs))
    cdp.touch("touchEnd", [])
    L = cdp.eval(LAYOUT_JS)
    check(ph, "팔 패널은 이동 칸 자리, 이동 칸은 숨김, 카메라·비상정지 그대로",
          L["arm"] is not None and L["move"] is None and L["cam"] is not None and L["estopOnTop"], {"arm": L["arm"]})
    check_layout_arm = cdp.eval("(() => { const a = document.querySelector('#arm').getBoundingClientRect();"
                                " return a.bottom <= innerHeight + 0.5 && a.right <= innerWidth + 0.5; })()")
    check(ph, "팔 패널 전체가 화면 안(가로)", check_layout_arm)

    # 팔 켜기 → '앞' 누름 → arm.x=1 → 누른 채 '닫기' → 중립
    tap(cdp, "#bArm")
    on = wait_for(lambda: (state() or {}).get("arm_enabled") and cdp.eval("!document.body.classList.contains('armoff')"), 3)
    check(ph, "'팔 조종 켜기' → arm_enable(팔 켜짐)", bool(on))
    x0 = ((rs.get() or {}).get("arm_target") or {}).get("x")
    f = cdp.rect('.hb[data-ax="x"][data-d="1"]')
    t0 = tr.mark()
    cdp.touch("touchStart", [(2, (f["cx"], f["cy"]))])
    time.sleep(0.5)
    arm_in = [b for _, b in tr.bodies("/api/input", t0) if (b.get("arm") or {}).get("x") == 1]
    x1 = ((rs.get() or {}).get("arm_target") or {}).get("x")
    check(ph, "팔 '앞' 누름 → /api/input arm.x=1", bool(arm_in), arm_in[-1] if arm_in else None)
    check(ph, "→ 가짜 로봇 팔 목표 x 증가", x0 is not None and x1 is not None and x1 - x0 > 0.03, "x %s → %s" % (x0, x1))
    shot("shot_landscape_2400x1080_2_arm_panel.png")
    c = cdp.rect("#bArmClose")
    t_close = tr.mark()
    second_finger_tap(cdp, "#bArmClose")
    closed = wait_for(lambda: not cdp.eval("document.body.classList.contains('arm-open')"), 2)
    zero = wait_for(lambda: any(all(v == 0 for v in (b.get("arm") or {"x": 1}).values()) for _, b in tr.bodies("/api/input", t_close)), 1.5)
    check(ph, "팔 버튼을 누른 채 '닫기' → 패널 닫힘", bool(closed))
    check(ph, "패널 닫을 때 팔 입력 중립(arm 모두 0)", bool(zero), tr.bodies("/api/input", t_close)[:2])
    cdp.touch("touchEnd", [])
    xa = ((rs.get() or {}).get("arm_target") or {}).get("x")
    time.sleep(0.6)
    xb = ((rs.get() or {}).get("arm_target") or {}).get("x")
    check(ph, "→ 닫은 뒤 팔 목표가 더 움직이지 않음", xa is not None and abs((xb or 0) - xa) < 0.005, "%s → %s" % (xa, xb))

    # 알림: 위쪽, 비상정지 칸과 겹치지 않음, 터치 통과
    cdp.eval("toast('알림 위치 시험: 이 글은 조작부를 막지 않습니다', 'info'); 1")
    tb, sb = cdp.rect("#toast"), cdp.rect("#stop")
    pe = cdp.eval("getComputedStyle(document.querySelector('#toast')).pointerEvents")
    check(ph, "알림: 상태 줄 바로 아래, 비상정지 칸과 겹치지 않음, 터치 통과", tb["y"] < 80 and tb["r"] <= sb["x"] and pe == "none",
          "알림 %.0f..%.0f, 비상정지 칸 x %.0f, pointer-events %s" % (tb["x"], tb["r"], sb["x"], pe))
    cdp.eval("document.querySelector('#toast').className = ''; 1")

    # 메뉴: 비상정지 칸을 가리지 않는 자리에 열림
    tap(cdp, "#bMenu")
    wait_for(lambda: cdp.eval("document.querySelector('#sheet').classList.contains('open')"), 2)
    L = cdp.eval(LAYOUT_JS)
    check(ph, "메뉴 열림: 메뉴 상자가 비상정지 칸과 겹치지 않고 비상정지가 맨 위", L["sheetBox"] and not overlap(L["sheetBox"], L["stop"]) and L["estopOnTop"],
          {"box": L["sheetBox"], "stop": L["stop"]})
    shot("shot_landscape_2400x1080_extra_menu.png")
    tap(cdp, "#mClose")
    wait_for(lambda: not cdp.eval("document.querySelector('#sheet').classList.contains('open')"), 2)

    # 카메라 크게(가로): 이동 칸 자리까지, 비상정지 칸 위치 그대로
    s0 = cdp.rect("#stop")
    c0 = cdp.rect("#cam")
    tap(cdp, "#bCamBig")
    wait_for(lambda: cdp.eval("document.body.classList.contains('cam-big')"), 2)
    time.sleep(0.2)
    c1, s1 = cdp.rect("#cam"), cdp.rect("#stop")
    check(ph, "카메라 '크게' → 폭 1.6 배 이상, 비상정지 칸 위치 그대로", c1["w"] > c0["w"] * 1.6 and abs(s1["x"] - s0["x"]) < 0.5 and abs(s1["y"] - s0["y"]) < 0.5,
          "카메라 %.0f→%.0f, 비상정지 x %.0f→%.0f" % (c0["w"], c1["w"], s0["x"], s1["x"]))
    check(ph, "크게 보기 중 비상정지 맨 위", cdp.eval(LAYOUT_JS)["estopOnTop"])
    shot("shot_landscape_2400x1080_extra_camera_big.png")
    tap(cdp, "#cam", finger=4)   # 카메라를 누르면 원래대로
    check(ph, "카메라를 다시 누르면 원래대로", bool(wait_for(lambda: not cdp.eval("document.body.classList.contains('cam-big')"), 2)))

    # JS 가 멈추면(앱 멈춤·망 끊김 흉내) 게이트웨이가 0.3 s 입력 끊김으로 중립
    cdp.touch("touchStart", [(0, (j["cx"], j["cy"] - j["h"] * 0.3))])
    wait_for(lambda: robot_cmd(rs).get("vx", 0) > 0.05, 3)
    cdp.send("Debugger.enable")
    cdp.send_nowait("Debugger.pause")
    t_p = time.monotonic()
    stale = wait_for(lambda: not (state() or {}).get("input", {}).get("fresh", True), 2, 0.02)
    dt_stale = (time.monotonic() - t_p) * 1e3
    rzero = wait_for(lambda: robot_cmd(rs) == {"vx": 0.0, "vy": 0.0, "wz": 0.0}, 2, 0.02)
    check(ph, "화면 JS 멈춤(누른 채) → 게이트웨이 입력 끊김 처리(fresh=false)", bool(stale), "%.0f ms" % dt_stale)
    check(ph, "→ 가짜 로봇 차체 명령 0", bool(rzero), robot_cmd(rs))
    cdp.send("Debugger.resume")
    cdp.send("Debugger.disable")
    cdp.touch("touchEnd", [])
    time.sleep(0.3)

    # 화면을 돌리면 중립(누른 채 세로로)
    cdp.touch("touchStart", [(0, (j["cx"], j["cy"] - j["h"] * 0.3))])
    wait_for(lambda: robot_cmd(rs).get("vx", 0) > 0.05, 3)
    t_rot = tr.mark()
    cdp.metrics(360, 800, 3)
    rot = wait_for(lambda: any(not b["held"] for _, b in tr.bodies("/api/input", t_rot)), 1.5)
    check(ph, "누른 채 화면을 세로로 돌림 → 중립 입력", bool(rot), tr.bodies("/api/input", t_rot)[:1])
    check(ph, "→ 가짜 로봇 차체 명령 0", bool(wait_for(lambda: robot_cmd(rs) == {"vx": 0.0, "vy": 0.0, "wz": 0.0}, 2)), robot_cmd(rs))
    cdp.touch("touchEnd", [])
    cdp.metrics(800, 360, 3)
    time.sleep(0.4)

    # 비상정지
    t_es = tr.mark()
    tap(cdp, "#bStop")
    es_cmd = wait_for(lambda: [b for _, b in tr.bodies("/api/cmd", t_es) if b.get("cmd") == "estop"], 1.5)
    latched = wait_for(lambda: (rs.get() or {}).get("emergency_stop") is True, 2)
    check(ph, "비상정지 누름 → /api/cmd estop 송신", bool(es_cmd), es_cmd[:1] if es_cmd else None)
    check(ph, "→ 가짜 로봇 비상정지 latch", bool(latched))
    shown = wait_for(lambda: cdp.eval("!document.querySelector('#cEstop').hidden && document.body.classList.contains('estopped')"), 2)
    check(ph, "화면: '비상정지 ON' 표시·버튼 강조", bool(shown))
    shot("shot_landscape_2400x1080_3_estop.png")
    # 비상정지 중 이동은 로봇이 막음(차체 0 유지)
    cdp.touch("touchStart", [(0, (j["cx"], j["cy"] - j["h"] * 0.3))])
    time.sleep(0.5)
    check(ph, "비상정지 중 조이스틱 → 차체 0 유지", robot_cmd(rs) == {"vx": 0.0, "vy": 0.0, "wz": 0.0}, robot_cmd(rs))
    cdp.touch("touchEnd", [])
    # 짧게 누르면 해제 안 됨, 0.8 s 길게 누르면 해제
    t_cl = tr.mark()
    tap(cdp, "#bClear", hold=0.3)
    time.sleep(0.4)
    check(ph, "'해제'를 0.3 s 만 누르면 clear 를 보내지 않음", not [b for _, b in tr.bodies("/api/cmd", t_cl) if b.get("cmd") == "clear"])
    tap(cdp, "#bClear", hold=1.0)
    cl = wait_for(lambda: [b for _, b in tr.bodies("/api/cmd", t_cl) if b.get("cmd") == "clear"], 1.5)
    cleared = wait_for(lambda: (rs.get() or {}).get("emergency_stop") is False, 3)
    check(ph, "0.8 s 길게 누름 → /api/cmd clear → 가짜 로봇 해제", bool(cl) and bool(cleared), rs.get() and rs.get().get("estop_sources"))

    # 다른 기기가 조종 중이면 안내·조작 무시
    stop_other = threading.Event()

    def other():
        while not stop_other.is_set():
            http("POST", "/api/input", {"screen": "otherdevice", "screen_name": "다른 휴대폰", "f": 0, "s": 0, "t": 0,
                                        "held": False, "focused": True, "n": int(time.monotonic() * 1000)})
            time.sleep(0.1)
    th = threading.Thread(target=other, daemon=True)
    st_take, raw_take, _ = http("POST", "/api/cmd", {"screen": "otherdevice", "screen_name": "다른 휴대폰", "cmd": "take_control"})
    check(ph, "(준비) 다른 기기가 take_control", st_take == 200, raw_take[:100])
    th.start()
    locked = wait_for(lambda: cdp.eval("document.body.classList.contains('locked') && getComputedStyle(document.querySelector('#owner')).display !== 'none'"), 4)
    check(ph, "다른 기기가 조종 중 → '다른 기기가 조종 중' 안내", bool(locked), cdp.eval("document.querySelector('#ownerText').textContent"))
    shot("shot_landscape_2400x1080_extra_other_device.png")
    t_lk = tr.mark()
    tap(cdp, "#bStop")
    es2 = wait_for(lambda: (rs.get() or {}).get("emergency_stop") is True, 2)
    check(ph, "다른 기기 조종 중에도 이 화면의 비상정지는 동작", bool(es2) and bool([b for _, b in tr.bodies("/api/cmd", t_lk) if b.get("cmd") == "estop"]))
    stop_other.set()
    th.join()
    time.sleep(1.8)   # 다른 기기 조종권 풀림(1.5 s) → 세션 닫힘
    t_cl = tr.mark()
    tap(cdp, "#bClear", hold=1.0)
    ok_clear = wait_for(lambda: (rs.get() or {}).get("emergency_stop") is False, 4)
    sent = [b.get("cmd") for _, b in tr.bodies("/api/cmd", t_cl)]
    check(ph, "세션이 없을 때 해제 한 번 → connect 뒤 clear → 가짜 로봇 해제", bool(ok_clear) and sent[:2] == ["connect", "clear"], sent)
    check(ph, "다른 기기가 멈추면 안내가 사라짐", bool(wait_for(lambda: not cdp.eval("document.body.classList.contains('locked')"), 4)))

    # ── 세로 360×800 ──
    cdp.metrics(360, 800, 3)
    time.sleep(0.5)
    check_layout(ph, cdp, True, "세로 360×800")
    w, h = shot("shot_portrait_1080x2400_1_basic.png")
    check(ph, "세로 캡처 1080×2400", (w, h) == (1080, 2400), (w, h))
    j = cdp.rect("#joy")
    cdp.touch("touchStart", [(0, (j["cx"] - j["w"] * 0.3, j["cy"]))])
    side = wait_for(lambda: robot_cmd(rs).get("vy", 0) > 0.05, 3)
    cdp.touch("touchEnd", [])
    check(ph, "세로: 조이스틱 왼쪽 → 옆 이동 vy>0, 손 떼면 0",
          bool(side) and bool(wait_for(lambda: robot_cmd(rs) == {"vx": 0.0, "vy": 0.0, "wz": 0.0}, 2)), robot_cmd(rs))
    tap(cdp, "#bArmPanel")
    wait_for(lambda: cdp.eval("document.body.classList.contains('arm-open')"), 2)
    L = cdp.eval(LAYOUT_JS)
    check(ph, "세로: 팔 패널이 이동 칸 자리(아래)에 열리고 화면 안, 비상정지 맨 위",
          L["arm"] is not None and L["arm"]["b"] <= L["H"] + 0.5 and L["arm"]["y"] > L["stop"]["b"] and L["estopOnTop"], L["arm"])
    check(ph, "세로: 팔 패널 버튼 44 px 이상", not L["small"], L["small"])
    if not cdp.eval("!document.body.classList.contains('armoff')"):
        tap(cdp, "#bArm")
        wait_for(lambda: cdp.eval("!document.body.classList.contains('armoff')"), 3)
    shot("shot_portrait_1080x2400_2_arm_panel.png")
    tap(cdp, "#bMenu")
    wait_for(lambda: cdp.eval("document.querySelector('#sheet').classList.contains('open')"), 2)
    L = cdp.eval(LAYOUT_JS)
    check(ph, "세로: 메뉴 상자가 비상정지 칸과 겹치지 않음", L["sheetBox"] and not overlap(L["sheetBox"], L["stop"]) and L["estopOnTop"],
          {"box": L["sheetBox"], "stop": L["stop"]})
    shot("shot_portrait_1080x2400_extra_menu.png")
    tap(cdp, "#mClose")
    tap(cdp, "#bArmClose")
    t_es = tr.mark()
    tap(cdp, "#bStop")
    latched = wait_for(lambda: (rs.get() or {}).get("emergency_stop") is True, 2)
    wait_for(lambda: cdp.eval("!document.querySelector('#cEstop').hidden"), 2)
    check(ph, "세로: 비상정지 → estop 송신·가짜 로봇 latch", bool(latched) and bool([b for _, b in tr.bodies("/api/cmd", t_es) if b.get("cmd") == "estop"]))
    shot("shot_portrait_1080x2400_3_estop.png")
    tap(cdp, "#bClear", hold=1.0)
    check(ph, "세로: 길게 눌러 해제", bool(wait_for(lambda: (rs.get() or {}).get("emergency_stop") is False, 3)))

    # ── 실제 휴대폰 크기(Galaxy S20 FE CSS 약 915×412)도 배치만 확인 ──
    cdp.metrics(915, 412, 2.625)
    time.sleep(0.4)
    check_layout(ph, cdp, False, "가로 915×412")
    cdp.metrics(412, 915, 2.625)
    time.sleep(0.4)
    check_layout(ph, cdp, True, "세로 412×915")

    with tr.lock:
        blocked = [u for u in tr.blocked if not u.endswith("/favicon.ico")]
        total = len(tr.log)
        kinds = {}
        for _, k, m, u, _b in tr.log:
            key = "%s %s %s" % (k, m, urllib.parse.urlsplit(u).port or 80)
            kinds[key] = kinds.get(key, 0) + 1
    check(ph, "허용 밖 요청 0건(앱 규칙: 로봇 IP 18765 /app-ui/·/api/, 8080 /stream 만)", not blocked,
          {"blocked": blocked[:5], "요청 수": total, "종류": kinds})
    with open(os.path.join(out, "requests_summary.json"), "w") as fp:
        json.dump({"total": total, "by_kind": kinds, "blocked": tr.blocked}, fp, ensure_ascii=False, indent=1)
    cdp.close()
    return shots


def lna_note(out, port):
    """기록용: 데스크톱 Chrome 기본(LNA 검사 켬)에서는 가로챈 화면의 8080 영상이 막히는지."""
    ch = Chrome(out, port, lna=True)
    try:
        cdp = ch.page()
        Traffic(cdp)
        errs = []
        cdp.on("Network.loadingFailed", lambda p: errs.append((p.get("corsErrorStatus") or {}).get("corsError") or p.get("errorText")))
        cdp.send("Page.enable")
        cdp.send("Network.enable")
        cdp.send("Fetch.enable", {"patterns": [{"urlPattern": "*", "requestStage": "Request"}]})
        cdp.send("Emulation.setUserAgentOverride", {"userAgent": UA})
        cdp.send("Page.navigate", {"url": PAGE_URL})
        time.sleep(3.5)
        nw = cdp.eval("document.querySelector('#video').naturalWidth")
        api_ok = "대기" in (cdp.eval("document.querySelector('#cLink').textContent") or "")
        cdp.close()
        return {"video_natural_width": nw, "api_ok": api_ok, "errors": sorted(set(e for e in errs if e))}
    finally:
        ch.quit()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out")
    ap.add_argument("--keep", action="store_true", help="끝나도 컨테이너를 남김(디버그)")
    ap.add_argument("--chrome-port", type=int, default=19333)
    args = ap.parse_args()
    out = args.out or os.path.join(ROOT, "work", "logs", "app_ui_v1_1_" + time.strftime("%Y%m%d_%H%M%S"))
    os.makedirs(out, exist_ok=True)
    for p in (WEB_PORT, CAM_PORT, RB_PORT, args.chrome_port, args.chrome_port + 1):
        if not port_free(p):
            print("포트 %d 가 이미 쓰이고 있어 시작하지 않음" % p, file=sys.stderr)
            return 2
    name = "armpi-appui-test-%s-%d" % (time.strftime("%Y%m%d%H%M%S"), os.getpid())
    env_txt = {
        "date": time.strftime("%Y-%m-%dT%H:%M:%S%z"), "page_url": PAGE_URL, "video_url": VIDEO_URL, "ua": UA,
        "image": IMAGE, "container": name, "chrome": subprocess.run([CHROME, "--version"], capture_output=True, text=True).stdout.strip(),
        "git_head": subprocess.run(["git", "-C", ROOT, "rev-parse", "--short", "HEAD"], capture_output=True, text=True).stdout.strip(),
        "asset_sha256": subprocess.run(["shasum", "-a", "256", os.path.join(ASSETS, "app_ui", "control.html")],
                                       capture_output=True, text=True).stdout.split()[0],
    }
    chrome = None
    rs = None
    t0 = time.monotonic()
    try:
        print("== 컨테이너: C++ 게이트웨이 빌드·실행 + 가짜 로봇", flush=True)
        boot = start_container(name, out)
        env_txt["container_boot_s"] = round(boot, 1)
        print("   준비 %.0f s" % boot, flush=True)
        phase_http(out)
        rs = RobotStatus()
        rs.start()
        wait_for(lambda: rs.get() is not None, 5)
        env_txt["lna_desktop_chrome_default"] = lna_note(out, args.chrome_port + 1)
        print("   (기록) 데스크톱 Chrome 기본 LNA 검사:", env_txt["lna_desktop_chrome_default"], flush=True)
        chrome = Chrome(out, args.chrome_port)
        env_txt["chrome_flags"] = "--disable-features=" + LNA_FEATURES
        env_txt["shots"] = phase_ui(out, chrome, rs)
    except Exception as e:  # noqa: BLE001
        check("실행", "시험 도중 예외", False, repr(e))
    finally:
        if rs:
            rs.stop()
        if chrome:
            chrome.quit()
        if args.keep:
            print("컨테이너 남김:", name)
        else:
            stop_container(name, out)
    npass = sum(r["pass"] for r in RESULTS)
    nfail = len(RESULTS) - npass
    env_txt["elapsed_s"] = round(time.monotonic() - t0, 1)
    with open(os.path.join(out, "results.json"), "w") as fp:
        json.dump({"environment": env_txt, "results": RESULTS, "pass": npass, "fail": nfail}, fp, ensure_ascii=False, indent=1)
    with open(os.path.join(out, "summary.txt"), "w") as fp:
        for r in RESULTS:
            fp.write("[%s] %s | %s  %s\n" % ("PASS" if r["pass"] else "FAIL", r["phase"], r["check"], r["detail"][:200]))
        fp.write("요약: 통과 %d, 실패 %d (%s)\n" % (npass, nfail, out))
    print("요약: 통과 %d, 실패 %d → %s" % (npass, nfail, out), flush=True)
    return 0 if nfail == 0 and npass > 0 else 1


if __name__ == "__main__":
    sys.exit(main())
