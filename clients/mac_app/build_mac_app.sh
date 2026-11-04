#!/bin/bash
# ArmPi Controller.app 빌드(Xcode 없이 Command Line Tools 의 swiftc 만 사용).
#
#   bash clients/mac_app/build_mac_app.sh             # 빌드 → ad-hoc 서명 → output/ArmPi_Controller_mac.zip
#   bash clients/mac_app/build_mac_app.sh --install   # 위 + ~/Applications/ArmPi Controller.app 로 복사
#
# 빌드는 로컬 디스크의 임시 폴더(BUILD_DIR, 기본 $TMPDIR/armpi_mac_app_build)에서 한다.
# 프로젝트가 있는 외장 디스크(exFAT)는 확장 속성을 ._ 파일로 저장해서 코드 서명이 깨질 수 있기 때문이다.
# 결과 .app 경로는 마지막에 출력한다.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"   # 저장소 루트(clients/의 상위)
OUT_DIR="$ROOT/output"
BUILD_DIR="${BUILD_DIR:-${TMPDIR:-/tmp}/armpi_mac_app_build}"
APP_NAME="ArmPi Controller"
EXE="ArmPiController"
BUNDLE_ID="kr.wisoft.armpi.controller.mac"
VERSION="1.0"
BUILD_NO="$(date +%Y%m%d%H%M)"
MIN_MACOS="12.0"
INSTALL=0
[ "${1:-}" = "--install" ] && INSTALL=1

APP="$BUILD_DIR/$APP_NAME.app"
echo "== 빌드 폴더: $BUILD_DIR"
rm -rf "$BUILD_DIR"
mkdir -p "$BUILD_DIR" "$APP/Contents/MacOS" "$APP/Contents/Resources/mac_controller" "$APP/Contents/Resources/vr_protocol_v3" "$OUT_DIR"

command -v swiftc >/dev/null || { echo "swiftc 가 없습니다. xcode-select --install 로 명령줄 도구를 설치하세요." >&2; exit 1; }
echo "== swiftc: $(swiftc --version 2>&1 | head -1)"

# 1) 실행 파일: arm64 + x86_64 (x86_64 가 실패하면 arm64 만)
SRC="$HERE/main.swift"
FLAGS=(-swift-version 5 -O -framework AppKit -framework WebKit)
swiftc "${FLAGS[@]}" -target "arm64-apple-macos$MIN_MACOS" -o "$BUILD_DIR/$EXE-arm64" "$SRC"
if swiftc "${FLAGS[@]}" -target "x86_64-apple-macos$MIN_MACOS" -o "$BUILD_DIR/$EXE-x86_64" "$SRC" 2> "$BUILD_DIR/x86_64.err"; then
  lipo -create -output "$APP/Contents/MacOS/$EXE" "$BUILD_DIR/$EXE-arm64" "$BUILD_DIR/$EXE-x86_64"
else
  echo "   (x86_64 빌드 실패 → arm64 만 넣음: $(head -1 "$BUILD_DIR/x86_64.err"))"
  cp "$BUILD_DIR/$EXE-arm64" "$APP/Contents/MacOS/$EXE"
fi
echo "== 실행 파일: $(lipo -archs "$APP/Contents/MacOS/$EXE")"

# 2) 리소스: 조종기(파이썬·화면)와 v3 클라이언트. 상대 배치(clients/mac_controller/ 옆 protocol_v3/)를 그대로 유지한다.
cp -X "$ROOT/clients/mac_controller/armpi_mac_controller.py" "$APP/Contents/Resources/mac_controller/"
# 웹 화면 기준 위치: ros2_ws/src/armpi_web_gateway/www/ (로봇의 C++ 게이트웨이와 같은 파일). 번들에서는 조종기 옆에 둔다.
for f in index.html mobile.html manifest.webmanifest icon-192.png icon-512.png; do
  cp -X "$ROOT/ros2_ws/src/armpi_web_gateway/www/$f" "$APP/Contents/Resources/mac_controller/$f"
done
cp -X "$ROOT/protocol_v3/armpi_v3_client.py" "$APP/Contents/Resources/vr_protocol_v3/armpi_v3_client.py"
find "$APP/Contents/Resources" -type f -exec chmod 644 {} +   # exFAT 에서 복사하면 rwx------ 가 되므로 정리

# 3) 아이콘(.icns): 표준 라이브러리 PNG 생성기 → iconutil
ICONSET="$BUILD_DIR/AppIcon.iconset"
/usr/bin/python3 -B "$ROOT/clients/mac_controller/make_icon.py" --iconset "$ICONSET" > /dev/null
iconutil -c icns -o "$APP/Contents/Resources/AppIcon.icns" "$ICONSET"

# 4) Info.plist
cat > "$APP/Contents/Info.plist" <<PLIST
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0">
<dict>
  <key>CFBundleName</key><string>$APP_NAME</string>
  <key>CFBundleDisplayName</key><string>$APP_NAME</string>
  <key>CFBundleIdentifier</key><string>$BUNDLE_ID</string>
  <key>CFBundleExecutable</key><string>$EXE</string>
  <key>CFBundleIconFile</key><string>AppIcon</string>
  <key>CFBundlePackageType</key><string>APPL</string>
  <key>CFBundleShortVersionString</key><string>$VERSION</string>
  <key>CFBundleVersion</key><string>$BUILD_NO</string>
  <key>CFBundleDevelopmentRegion</key><string>ko</string>
  <key>LSMinimumSystemVersion</key><string>$MIN_MACOS</string>
  <key>LSApplicationCategoryType</key><string>public.app-category.utilities</string>
  <key>NSHighResolutionCapable</key><true/>
  <key>NSPrincipalClass</key><string>NSApplication</string>
  <key>NSHumanReadableCopyright</key><string>ArmPi Pro 조종기</string>
  <key>NSLocalNetworkUsageDescription</key>
  <string>같은 와이파이의 ArmPi 로봇을 찾고(UDP 5002) 조종 명령(UDP 5001)·상태(9090)·카메라 영상(8080)을 주고받습니다.</string>
  <key>NSAppTransportSecurity</key>
  <dict><key>NSAllowsArbitraryLoads</key><true/></dict>
</dict>
</plist>
PLIST
printf 'APPL????' > "$APP/Contents/PkgInfo"
plutil -lint "$APP/Contents/Info.plist"

# 5) ad-hoc 서명(개발자 인증서 없음). 확장 속성을 먼저 지운다.
xattr -cr "$APP"
codesign --force --sign - --timestamp=none "$APP"
codesign --verify --deep --strict --verbose=2 "$APP"
codesign -dv "$APP" 2>&1 | grep -E "Identifier|Signature|CDHash" || true

# 6) zip (Finder 에서 더블클릭하면 풀림)
ZIP="$OUT_DIR/ArmPi_Controller_mac.zip"
rm -f "$ZIP"
(cd "$BUILD_DIR" && ditto -c -k --norsrc --keepParent "$APP_NAME.app" "$ZIP")
echo "== zip: $ZIP ($(du -h "$ZIP" | cut -f1))"
shasum -a 256 "$ZIP"

if [ "$INSTALL" = 1 ]; then
  mkdir -p "$HOME/Applications"
  rm -rf "$HOME/Applications/$APP_NAME.app"
  ditto "$APP" "$HOME/Applications/$APP_NAME.app"
  echo "== 설치: $HOME/Applications/$APP_NAME.app"
fi
echo "== 앱: $APP"
