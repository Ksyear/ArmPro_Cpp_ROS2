#!/usr/bin/env python3
"""ArmPi 조종기 아이콘(PNG) 생성기 — 표준 라이브러리만 사용(외부 그림 파일·패키지 없음).

모양: 파란 둥근 사각형 위에 메카넘 차체(바퀴 2개)와 주황색 2관절 팔·그리퍼.
웹 화면(icon-192/512.png), 맥 앱(.icns 용 iconset), Android 런처 아이콘(mipmap)이 모두 이 스크립트를 쓴다.

  python3 make_icon.py out.png 512                 # 크기 하나
  python3 make_icon.py --web .                     # icon-192.png, icon-512.png
  python3 make_icon.py --iconset AppIcon.iconset   # 맥 iconset(16~1024)
  python3 make_icon.py --android res               # res/mipmap-*/ic_launcher.png
"""

import math
import os
import struct
import sys
import zlib

# 정규화 좌표(0~1, 위가 0)로 그린 도형. (종류, 매개변수, 색 RGBA)
BG_TOP = (37, 99, 235)
BG_BOTTOM = (11, 42, 107)
WHITE = (236, 242, 250)
ORANGE = (255, 170, 40)
DARK = (14, 22, 38)
RIM = (150, 172, 205)

SHAPES = [
    ("rrect", (0.50, 0.725, 0.32, 0.065, 0.035), WHITE),          # 차체
    ("circle", (0.31, 0.80, 0.085), RIM),                         # 바퀴(테)
    ("circle", (0.31, 0.80, 0.058), DARK),
    ("circle", (0.69, 0.80, 0.085), RIM),
    ("circle", (0.69, 0.80, 0.058), DARK),
    ("rrect", (0.47, 0.625, 0.075, 0.05, 0.02), WHITE),            # 팔 받침
    ("seg", (0.47, 0.60, 0.35, 0.36, 0.075), ORANGE),             # 위팔
    ("seg", (0.35, 0.36, 0.63, 0.27, 0.065), ORANGE),             # 아래팔
    ("circle", (0.47, 0.60, 0.052), WHITE),                       # 관절
    ("circle", (0.47, 0.60, 0.022), DARK),
    ("circle", (0.35, 0.36, 0.047), WHITE),
    ("circle", (0.35, 0.36, 0.019), DARK),
    ("poly", ((0.64, 0.255), (0.74, 0.19), (0.795, 0.225)), 0.032, WHITE),   # 그리퍼 위 손가락
    ("poly", ((0.65, 0.295), (0.755, 0.34), (0.80, 0.30)), 0.032, WHITE),    # 그리퍼 아래 손가락
    ("circle", (0.63, 0.27, 0.042), WHITE),                       # 손목
]
BG_MARGIN = 0.035
BG_RADIUS = 0.22


def _d_rrect(x, y, cx, cy, hx, hy, r):
    qx = abs(x - cx) - (hx - r)
    qy = abs(y - cy) - (hy - r)
    return math.hypot(max(qx, 0.0), max(qy, 0.0)) + min(max(qx, qy), 0.0) - r


def _d_seg(x, y, ax, ay, bx, by):
    vx, vy = bx - ax, by - ay
    t = ((x - ax) * vx + (y - ay) * vy) / (vx * vx + vy * vy)
    t = 0.0 if t < 0 else (1.0 if t > 1 else t)
    return math.hypot(x - ax - t * vx, y - ay - t * vy)


def _shape_fn(shape):
    kind = shape[0]
    if kind == "circle":
        cx, cy, r = shape[1]
        return (lambda x, y: math.hypot(x - cx, y - cy) - r), (cx - r, cy - r, cx + r, cy + r)
    if kind == "rrect":
        cx, cy, hx, hy, r = shape[1]
        return (lambda x, y: _d_rrect(x, y, cx, cy, hx, hy, r)), (cx - hx, cy - hy, cx + hx, cy + hy)
    if kind == "seg":
        ax, ay, bx, by, w = shape[1]
        h = w / 2
        return (lambda x, y: _d_seg(x, y, ax, ay, bx, by) - h), (min(ax, bx) - h, min(ay, by) - h,
                                                                  max(ax, bx) + h, max(ay, by) + h)
    if kind == "poly":
        pts, w = shape[1], shape[2]
        h = w / 2
        segs = list(zip(pts[:-1], pts[1:]))
        xs, ys = [p[0] for p in pts], [p[1] for p in pts]
        return (lambda x, y: min(_d_seg(x, y, a[0], a[1], b[0], b[1]) for a, b in segs) - h), \
            (min(xs) - h, min(ys) - h, max(xs) + h, max(ys) + h)
    raise ValueError(kind)


def render(size):
    """RGBA bytearray(size×size×4)."""
    px = bytearray(size * size * 4)
    inv = 1.0 / size
    # 배경: 세로 그라데이션 둥근 사각형
    half = 0.5 - BG_MARGIN
    for j in range(size):
        y = (j + 0.5) * inv
        t = j / max(1, size - 1)
        col = [int(BG_TOP[k] + (BG_BOTTOM[k] - BG_TOP[k]) * t + 0.5) for k in range(3)]
        row = j * size * 4
        for i in range(size):
            x = (i + 0.5) * inv
            d = _d_rrect(x, y, 0.5, 0.5, half, half, BG_RADIUS)
            a = 0.5 - d * size
            if a <= 0:
                continue
            a = 1.0 if a >= 1 else a
            o = row + i * 4
            px[o], px[o + 1], px[o + 2], px[o + 3] = col[0], col[1], col[2], int(a * 255 + 0.5)
    for shape in SHAPES:
        fn, (x0, y0, x1, y1) = _shape_fn(shape)
        color = shape[-1]
        i0, i1 = max(0, int(x0 * size) - 2), min(size, int(x1 * size) + 3)
        j0, j1 = max(0, int(y0 * size) - 2), min(size, int(y1 * size) + 3)
        for j in range(j0, j1):
            y = (j + 0.5) * inv
            row = j * size * 4
            for i in range(i0, i1):
                a = 0.5 - fn((i + 0.5) * inv, y) * size
                if a <= 0:
                    continue
                a = 1.0 if a >= 1 else a
                o = row + i * 4
                for k in range(3):
                    px[o + k] = int(px[o + k] * (1 - a) + color[k] * a + 0.5)
                px[o + 3] = int(px[o + 3] * (1 - a) + 255 * a + 0.5)
    return px


def write_png(path, size, px):
    raw = bytearray()
    stride = size * 4
    for j in range(size):
        raw.append(0)
        raw += px[j * stride:(j + 1) * stride]

    def chunk(tag, data):
        c = struct.pack(">I", len(data)) + tag + data
        return c + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)
    data = b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", size, size, 8, 6, 0, 0, 0)) \
        + chunk(b"IDAT", zlib.compress(bytes(raw), 9)) + chunk(b"IEND", b"")
    d = os.path.dirname(os.path.abspath(path))
    os.makedirs(d, exist_ok=True)
    with open(path, "wb") as fp:
        fp.write(data)


def make(path, size, cache={}):
    if size not in cache:
        cache[size] = render(size)
    write_png(path, size, cache[size])
    return path


def main(argv):
    if len(argv) == 3 and argv[1] == "--web":
        for s in (192, 512):
            print(make(os.path.join(argv[2], "icon-%d.png" % s), s))
        return 0
    if len(argv) == 3 and argv[1] == "--iconset":
        out = argv[2]
        for base in (16, 32, 128, 256, 512):
            print(make(os.path.join(out, "icon_%dx%d.png" % (base, base)), base))
            print(make(os.path.join(out, "icon_%dx%d@2x.png" % (base, base)), base * 2))
        return 0
    if len(argv) == 3 and argv[1] == "--android":
        for name, s in (("mdpi", 48), ("hdpi", 72), ("xhdpi", 96), ("xxhdpi", 144), ("xxxhdpi", 192)):
            print(make(os.path.join(argv[2], "mipmap-%s" % name, "ic_launcher.png"), s))
        return 0
    if len(argv) == 3 and argv[2].isdigit():
        print(make(argv[1], int(argv[2])))
        return 0
    print(__doc__, file=sys.stderr)
    return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv))
