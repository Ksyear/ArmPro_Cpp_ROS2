#!/usr/bin/env python3
"""Mac 시험용 UDP <-> TCP 중계기 (표준 라이브러리만).

colima(Docker) 는 TCP 포트만 Mac 으로 넘겨 주고 UDP 는 넘기지 않는다(2026-11-02 확인).
그래서 Mac 조종기의 UDP 를 TCP 로 감싸 컨테이너로 보내고, 컨테이너 안에서 다시 UDP 로 푼다.
실제 로봇에는 필요 없다(Mac 과 로봇이 같은 망이면 UDP 를 바로 쓴다).

  컨테이너 안:  python3 udp_tcp_relay.py server --tcp 0.0.0.0:15001 --udp 127.0.0.1:5001
  Mac:          python3 udp_tcp_relay.py client --udp 127.0.0.1:25001 --tcp 127.0.0.1:15001
  Mac 조종기는 127.0.0.1:25001 을 로봇으로 쓴다.

틀: TCP 위에 [2바이트 길이(빅엔디언)][데이터그램]. 클라이언트는 마지막으로 보낸 UDP 주소에 응답을 돌려준다.
"""
import argparse
import select
import socket
import struct
import sys
import time


def hostport(s):
    h, p = s.rsplit(":", 1)
    return h, int(p)


def read_exact(sock, n):
    b = b""
    while len(b) < n:
        c = sock.recv(n - len(b))
        if not c:
            raise ConnectionError("TCP 끊김")
        b += c
    return b


def pump(tcp, udp, udp_peer_getter, udp_peer_setter, log):
    """한 TCP 연결 동안 양방향 전달. udp_peer_getter() 가 None 이면 UDP 응답을 버린다."""
    buf = b""
    while True:
        r, _, _ = select.select([tcp, udp], [], [], 1.0)
        if udp in r:
            data, addr = udp.recvfrom(4096)
            udp_peer_setter(addr)
            tcp.sendall(struct.pack(">H", len(data)) + data)
        if tcp in r:
            c = tcp.recv(65536)
            if not c:
                log("TCP 끊김")
                return
            buf += c
            while len(buf) >= 2:
                n = struct.unpack(">H", buf[:2])[0]
                if len(buf) < 2 + n:
                    break
                data, buf = buf[2:2 + n], buf[2 + n:]
                peer = udp_peer_getter()
                if peer is not None:
                    udp.sendto(data, peer)


def run_server(args):
    """컨테이너 쪽: TCP 연결 하나당 UDP 소켓 하나로 수신기(--udp)와 통신."""
    th, tp = hostport(args.tcp)
    target = hostport(args.udp)
    ls = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    ls.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    ls.bind((th, tp))
    ls.listen(1)
    print(f"[relay-server] TCP {th}:{tp} -> UDP {target[0]}:{target[1]}", flush=True)
    while True:
        conn, addr = ls.accept()
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        u = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        u.bind(("127.0.0.1", 0))
        print(f"[relay-server] 연결 {addr}, UDP 출발 포트 {u.getsockname()[1]}", flush=True)
        try:
            pump(conn, u, lambda: target, lambda a: None, lambda m: print("[relay-server]", m, flush=True))
        except Exception as e:  # noqa
            print("[relay-server] 오류:", e, flush=True)
        finally:
            conn.close()
            u.close()


def run_client(args):
    """Mac 쪽: UDP 를 받아 TCP 로 컨테이너에 보낸다. 끊기면 1초 뒤 다시 연결."""
    uh, up = hostport(args.udp)
    th, tp = hostport(args.tcp)
    u = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    u.bind((uh, up))
    peer = {"addr": None}
    print(f"[relay-client] UDP {uh}:{up} -> TCP {th}:{tp}", flush=True)
    while True:
        try:
            t = socket.create_connection((th, tp), timeout=3)
            t.settimeout(None)
            t.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            print("[relay-client] TCP 연결됨", flush=True)
            # Mac 쪽에서는 UDP 로 받은 것을 TCP 로, TCP 로 받은 것을 마지막 UDP 발신자에게
            pump(t, u, lambda: peer["addr"], lambda a: peer.__setitem__("addr", a),
                 lambda m: print("[relay-client]", m, flush=True))
        except Exception as e:  # noqa
            print("[relay-client] 재연결 대기:", e, flush=True)
            time.sleep(1.0)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mode", choices=["server", "client"])
    ap.add_argument("--tcp", required=True)
    ap.add_argument("--udp", required=True)
    args = ap.parse_args()
    (run_server if args.mode == "server" else run_client)(args)


if __name__ == "__main__":
    sys.exit(main())
