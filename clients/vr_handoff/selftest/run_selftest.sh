#!/usr/bin/env bash
# ArmPiV3Messages.cs 자체 시험을 Mac(colima/docker)에서 다시 돌린다. 실제 로봇·외부 네트워크에 접속하지 않는다.
#   1) armpi-dev:humble 컨테이너에서 가짜 로봇(protocol_v3/mock_robot.py)을 127.0.0.1 로 띄운다(--network none).
#      작업 공간은 로봇 armpi.yaml 값(0.05–0.32, ±0.25, −0.08–0.32)으로 맞춘다.
#   2) mcr.microsoft.com/dotnet/sdk:8.0 컨테이너가 같은 네트워크 이름공간을 공유해
#      - net8.0 콘솔(C# 9, 경고=오류)로 빌드하고 1부(오프라인)·2부(UDP 왕복) 시험을 실행
#      - netstandard2.1 라이브러리(Unity 와 같은 API 수준)로도 컴파일
#   결과: work/logs/vr_handoff_csharp_<시각>/
# 소스 폴더는 읽기 전용으로 붙이고 컨테이너 /tmp 로 복사해 빌드한다(exFAT 공유 폴더 ENOENT 회피).
set -euo pipefail
PROJ="$(cd "$(dirname "$0")/../../.." && pwd)"   # 저장소 루트
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="$PROJ/work/logs/vr_handoff_csharp_$STAMP"
mkdir -p "$OUT"
NAME="armpi-vrhandoff-mock-$STAMP-$$"
cleanup() { docker logs "$NAME" > "$OUT/mock_robot.log" 2>&1 || true; docker stop "$NAME" >/dev/null 2>&1 || true; }
trap cleanup EXIT

( cd "$PROJ" && shasum -a 256 clients/vr_handoff/ArmPiV3Messages.cs clients/vr_handoff/selftest/Program.cs \
    clients/vr_handoff/selftest/ArmPiV3SelfTest.csproj protocol_v3/mock_robot.py ) > "$OUT/source_sha256.txt"

docker run -d --rm --name "$NAME" --network none -e PYTHONDONTWRITEBYTECODE=1 \
  -v "$PROJ/protocol_v3:/v3:ro" armpi-dev:humble \
  python3 -B /v3/mock_robot.py --bind 127.0.0.1 --rosbridge-port 0 --camera-port 0 \
  --workspace 0.05,0.32,-0.25,0.25,-0.08,0.32 >/dev/null   # 로봇 armpi.yaml 작업 공간과 맞춤
sleep 2

docker run --rm --network "container:$NAME" -e DOTNET_CLI_TELEMETRY_OPTOUT=1 -e DOTNET_NOLOGO=1 \
  -v "$PROJ/clients/vr_handoff:/src:ro" mcr.microsoft.com/dotnet/sdk:8.0 bash -c '
set -o pipefail
mkdir -p /tmp/w /tmp/r && cp -r /src/. /tmp/w/
{ echo "dotnet $(dotnet --version), $(uname -m)"; } > /tmp/r/environment.txt
cd /tmp/w/selftest
dotnet build -c Release --nologo > /tmp/r/build.txt 2>&1; echo "build exit=$?" >> /tmp/r/build.txt
dotnet bin/Release/net8.0/ArmPiV3SelfTest.dll --robot 127.0.0.1 > /tmp/r/selftest.txt 2>&1; echo "exit=$?" >> /tmp/r/selftest.txt
mkdir -p /tmp/u && cd /tmp/u
cat > ArmPiV3Unity.csproj <<EOF
<Project Sdk="Microsoft.NET.Sdk">
  <PropertyGroup><TargetFramework>netstandard2.1</TargetFramework><LangVersion>9.0</LangVersion>
  <Nullable>disable</Nullable><TreatWarningsAsErrors>true</TreatWarningsAsErrors></PropertyGroup>
  <ItemGroup><Compile Include="/tmp/w/ArmPiV3Messages.cs" /></ItemGroup>
</Project>
EOF
dotnet build -c Release --nologo > /tmp/r/build_netstandard21.txt 2>&1; echo "build exit=$?" >> /tmp/r/build_netstandard21.txt
tar -C /tmp/r -cf - .' | tar -C "$OUT" -xf -

tail -n 3 "$OUT/selftest.txt"
tail -n 4 "$OUT/build_netstandard21.txt"
echo "결과: $OUT"
