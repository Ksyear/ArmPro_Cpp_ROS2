#!/bin/bash
# AppUiPolicy(앱 안 조종 화면 주소·허용 범위) 단위 시험 — Android SDK 없이 맥 JDK 로 컴파일·실행한다.
#   bash clients/android_app/test/run_policy_test.sh
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
APP="$(cd "$HERE/.." && pwd)"
if [ -z "${JAVA_HOME:-}" ] || [ ! -x "${JAVA_HOME:-}/bin/javac" ]; then
  if command -v javac >/dev/null 2>&1; then JAVA_HOME="$(cd "$(dirname "$(command -v javac)")/.." && pwd)"
  elif command -v mise >/dev/null 2>&1 && mise which javac >/dev/null 2>&1; then JAVA_HOME="$(cd "$(dirname "$(mise which javac)")/.." && pwd)"
  else JAVA_HOME="$(/usr/libexec/java_home)"; fi
fi
OUT="$(mktemp -d)"; trap 'rm -rf "$OUT"' EXIT
"$JAVA_HOME/bin/javac" -encoding UTF-8 -d "$OUT" \
  "$APP/src/kr/wisoft/armpi/controller/AppUiPolicy.java" "$HERE/kr/wisoft/armpi/controller/AppUiPolicyTest.java"
"$JAVA_HOME/bin/java" -cp "$OUT" kr.wisoft.armpi.controller.AppUiPolicyTest
