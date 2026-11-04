#!/bin/bash
# ArmPi 조종기 Android APK 빌드 — Gradle·Android Studio 없이 SDK 명령줄 도구만 사용.
#   javac → d8 → aapt2 compile/link → zipalign → apksigner → (검증) apksigner verify, aapt2 dump badging
#
#   bash clients/android_app/build_apk.sh
#
# 필요: Android SDK(build-tools, platforms) — ANDROID_HOME / ANDROID_SDK_ROOT / ~/Library/Android/sdk 순서로 찾음,
#       JDK 17 이상(JAVA_HOME, PATH, mise, /usr/libexec/java_home 순서로 찾음).
# 서명: clients/android_app/debug.keystore(없으면 keytool 로 만듦, 비밀번호 android). 디버그 키라 배포용이 아니다.
#       같은 키로 서명해야 휴대폰에서 '업데이트'로 설치된다(키가 바뀌면 먼저 지우고 설치).
# 결과: output/ArmPi_Controller_android.apk
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"   # 저장소 루트(clients/의 상위)
OUT_DIR="$ROOT/output"
APK_OUT="$OUT_DIR/ArmPi_Controller_android.apk"
BUILD="${BUILD_DIR:-${TMPDIR:-/tmp}/armpi_android_build}"
MIN_SDK=26
VERSION_NAME="1.0"
VERSION_CODE="$(date +%y%m%d%H)"      # 예: 26110222 (int 범위 안)
KEYSTORE="${KEYSTORE:-$HERE/debug.keystore}"
KS_PASS="${KS_PASS:-android}"
KS_ALIAS="${KS_ALIAS:-androiddebugkey}"

# ── SDK·도구 찾기 ──
SDK="${ANDROID_HOME:-${ANDROID_SDK_ROOT:-$HOME/Library/Android/sdk}}"
[ -d "$SDK/build-tools" ] || { echo "Android SDK 를 찾지 못했습니다: $SDK (ANDROID_HOME 지정)" >&2; exit 1; }
BT="$(ls -d "$SDK"/build-tools/*/ 2>/dev/null | sed 's:/$::' | sort -V | tail -1)"
PLATFORM="$(ls -d "$SDK"/platforms/android-*/ 2>/dev/null | sed 's:/$::' | sort -V | tail -1)"
[ -n "$BT" ] && [ -n "$PLATFORM" ] || { echo "build-tools 또는 platforms 가 없습니다($SDK)" >&2; exit 1; }
ANDROID_JAR="$PLATFORM/android.jar"
COMPILE_SDK="${PLATFORM##*android-}"
TARGET_SDK=34                          # 34 = 화면 가장자리 강제(edge-to-edge, 35) 이전 동작
[ "$COMPILE_SDK" -lt 34 ] 2>/dev/null && TARGET_SDK="$COMPILE_SDK"
for t in aapt2 d8 zipalign apksigner; do [ -x "$BT/$t" ] || { echo "$BT/$t 없음" >&2; exit 1; }; done
[ -f "$BT/core-lambda-stubs.jar" ] || { echo "$BT/core-lambda-stubs.jar 없음" >&2; exit 1; }

if [ -z "${JAVA_HOME:-}" ] || [ ! -x "${JAVA_HOME:-}/bin/javac" ]; then
  if command -v javac >/dev/null 2>&1; then
    JAVA_HOME="$(cd "$(dirname "$(command -v javac)")/.." && pwd)"
  elif command -v mise >/dev/null 2>&1 && mise which javac >/dev/null 2>&1; then
    JAVA_HOME="$(cd "$(dirname "$(mise which javac)")/.." && pwd)"
  elif /usr/libexec/java_home >/dev/null 2>&1; then
    JAVA_HOME="$(/usr/libexec/java_home)"
  else
    echo "JDK(javac)를 찾지 못했습니다. JAVA_HOME 을 지정하세요." >&2; exit 1
  fi
fi
export JAVA_HOME PATH="$JAVA_HOME/bin:$PATH"

echo "== SDK: $SDK"
echo "   build-tools $(basename "$BT"), compileSdk $COMPILE_SDK, targetSdk $TARGET_SDK, minSdk $MIN_SDK"
echo "   JDK: $("$JAVA_HOME/bin/javac" -version 2>&1)"
echo "   빌드 폴더: $BUILD"

rm -rf "$BUILD"
mkdir -p "$BUILD/res" "$BUILD/gen" "$BUILD/classes" "$BUILD/dex" "$OUT_DIR"

# ── 1) 리소스: 소스 res + 아이콘(mipmap) 생성 → aapt2 compile/link ──
cp -R "$HERE/res/." "$BUILD/res/"
find "$BUILD/res" -name '._*' -delete 2>/dev/null || true   # exFAT 의 AppleDouble 파일 제거
/usr/bin/python3 -B "$ROOT/clients/mac_controller/make_icon.py" --android "$BUILD/res" > /dev/null
"$BT/aapt2" compile --dir "$BUILD/res" -o "$BUILD/res.zip"
"$BT/aapt2" link -o "$BUILD/base.apk" -I "$ANDROID_JAR" --manifest "$HERE/AndroidManifest.xml" \
  --java "$BUILD/gen" --min-sdk-version "$MIN_SDK" --target-sdk-version "$TARGET_SDK" \
  --version-code "$VERSION_CODE" --version-name "$VERSION_NAME" --auto-add-overlay "$BUILD/res.zip"

# ── 2) 자바 → 클래스 → DEX ──
find "$HERE/src" "$BUILD/gen" -name '*.java' ! -name '._*' > "$BUILD/sources.txt"
"$JAVA_HOME/bin/javac" -encoding UTF-8 -source 8 -target 8 -Xlint:-options -Xlint:deprecation \
  -bootclasspath "$ANDROID_JAR:$BT/core-lambda-stubs.jar" -d "$BUILD/classes" @"$BUILD/sources.txt"   # 람다용 stub(d8 이 desugar)
find "$BUILD/classes" -name '*.class' > "$BUILD/classes.txt"
"$BT/d8" --release --min-api "$MIN_SDK" --lib "$ANDROID_JAR" --output "$BUILD/dex" @"$BUILD/classes.txt"

# ── 3) APK 조립 → 정렬 → 서명 ──
cp "$BUILD/base.apk" "$BUILD/unsigned.apk"
(cd "$BUILD/dex" && zip -q -X "$BUILD/unsigned.apk" classes.dex)
"$BT/zipalign" -f -p 4 "$BUILD/unsigned.apk" "$BUILD/aligned.apk"

if [ ! -f "$KEYSTORE" ]; then
  echo "== 디버그 키스토어 생성: $KEYSTORE"
  "$JAVA_HOME/bin/keytool" -genkeypair -keystore "$KEYSTORE" -storetype PKCS12 -storepass "$KS_PASS" -keypass "$KS_PASS" \
    -alias "$KS_ALIAS" -keyalg RSA -keysize 2048 -validity 10000 -dname "CN=ArmPi Controller Debug, O=ArmPi Controller, C=KR" >/dev/null
fi
"$BT/apksigner" sign --ks "$KEYSTORE" --ks-pass "pass:$KS_PASS" --key-pass "pass:$KS_PASS" --ks-key-alias "$KS_ALIAS" \
  --out "$APK_OUT" "$BUILD/aligned.apk"
rm -f "$APK_OUT.idsig"

# ── 4) 검증 ──
echo "== apksigner verify"
"$BT/apksigner" verify --verbose --print-certs "$APK_OUT" | grep -E "Verifie|Signer #1 certificate (DN|SHA-256)|Number of signers"
echo "== aapt2 dump badging"
"$BT/aapt2" dump badging "$APK_OUT" | grep -E "^package:|^sdkVersion|^targetSdkVersion|^uses-permission|^application-label|^launchable-activity|^native-code|^densities" || true
echo "== zipalign 확인"
"$BT/zipalign" -c -p 4 "$APK_OUT" && echo "정렬 OK"
echo "== 결과: $APK_OUT ($(du -h "$APK_OUT" | cut -f1))"
shasum -a 256 "$APK_OUT"
