#!/usr/bin/env bash
# first_boot_check.sh — ArmPi Pro 첫 부팅 점검(docs/10 B절)을 읽기 전용으로 자동 수집한다. Mac 에서 실행.
#
# 하는 일
#   - Mac 쪽: 이름 확인, ping, TCP 포트 연결 확인(연결만 하고 아무것도 보내지 않음), SSH 호스트 키 지문
#   - SSH(공개키 인증만)로 로봇 상태 읽기: OS·부팅 시간·네트워크·서비스·로그·기준판 해시·시간 동기·직렬 콘솔·
#     I2C/카메라/직렬 장치 목록·ROS 토픽 1회 읽기·포트·온도/전압 경고
#   - 이미지 매니페스트(output/IMAGE_BUILD_MANIFEST.json)와 로봇 안 파일 해시를 대조(USB 굽기 간접 확인)
#   - Mac ↔ 로봇 시계 차이 추정
# 하지 않는 일 (스크립트가 명령마다 검사해서 막는다)
#   - 서비스 시작·정지·재시작·활성화, ROS 서비스 호출·토픽 발행·노드 실행
#   - i2cdetect 같은 버스 탐지, GPIO 요청(gpioset/gpioget), 직렬 포트 읽기·설정
#   - 재부팅·종료, 파일 쓰기·삭제, 패키지 설치, 네트워크 설정 변경, sudo
#   - 비밀번호 입력·저장, 비밀값 출력(nmcli --show-secrets 등). 저장 전에 비밀값처럼 보이는 문자열을 가린다
#
# 사용
#   tools/first_boot_check.sh                         # 기본: ubuntu@armpi-ros2.local, 키 ~/.ssh/id_ed25519_armpi
#   tools/first_boot_check.sh --host 192.168.149.1    # 대체 AP
#   tools/first_boot_check.sh --wait 300              # 전원을 넣는 순간 시작: SSH 가 열릴 때까지 최대 300 s 기다리며 시간 기록(B1)
#   tools/first_boot_check.sh --dry-run               # 실행할 명령만 보여 줌(네트워크 사용 안 함)
#   tools/first_boot_check.sh --guard-self-test       # 금지 명령 검사기가 제대로 막는지 자체 확인
# 결과: work/logs/first_boot_<시각>/ (summary.md 를 먼저 본다). PASS/FAIL 판정은 사람이 docs/10 기준으로 한다.
# ros2 명령은 제한 시간이 지나면 SIGINT(Ctrl+C 와 같음)로 끝내 정상 정리되게 한다(5 s 안에 안 끝나면 강제 종료).

TOOL_VERSION="1.1 (2026-11-04)"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"
SELF="$HERE/$(basename "${BASH_SOURCE[0]}")"

HOST="armpi-ros2.local"
RUSER="ubuntu"
KEY="$HOME/.ssh/id_ed25519_armpi"
SSH_PORT=22
WEB_PORT=18765
MJPEG_PORT=8080
ROSBRIDGE_PORT=9090
WAIT=0
OUT=""
LABEL=""
KNOWN_HOSTS=""
SKIP_ROS=0
TOPIC_TIMEOUT=10
DRY=0
GUARD_TEST=0
MANIFEST="$REPO/output/IMAGE_BUILD_MANIFEST.json"

usage() { sed -n '2,26p' "$SELF" | sed 's/^# \{0,1\}//'; cat <<'EOF'

옵션
  --host H            로봇 주소(기본 armpi-ros2.local)
  --user U            SSH 사용자(기본 ubuntu)
  --key PATH          SSH 개인키(기본 ~/.ssh/id_ed25519_armpi). 비밀번호 로그인은 쓰지 않는다
  --ssh-port N        SSH 포트(기본 22)
  --web-port N        로봇 웹 조종기 포트(기본 18765)  --mjpeg-port N  카메라 스트림(기본 8080)
  --rosbridge-port N  rosbridge(기본 9090)
  --wait SEC          SSH 가 열릴 때까지 최대 SEC 초 기다림(부팅 시간 기록)
  --out DIR           결과 폴더(기본 work/logs/first_boot_<시각>)
  --label TEXT        결과에 남길 메모(예: "1차 부팅, eth0 직결")
  --known-hosts FILE  호스트 키 파일(기본: 결과 폴더 안 새 파일 = 이번 실행에서 처음 본 키를 기록)
  --skip-ros          ros2 명령 생략
  --topic-timeout S   토픽 1회 읽기 제한 시간(기본 10)
  --manifest FILE     대조할 이미지 매니페스트(기본 output/IMAGE_BUILD_MANIFEST.json)
  --dry-run           실행할 명령만 출력
  --guard-self-test   금지 명령 검사기 자체 시험
EOF
}

while [ $# -gt 0 ]; do
  case "$1" in
    --host) HOST="$2"; shift 2;;
    --user) RUSER="$2"; shift 2;;
    --key) KEY="$2"; shift 2;;
    --ssh-port) SSH_PORT="$2"; shift 2;;
    --web-port) WEB_PORT="$2"; shift 2;;
    --mjpeg-port) MJPEG_PORT="$2"; shift 2;;
    --rosbridge-port) ROSBRIDGE_PORT="$2"; shift 2;;
    --wait) WAIT="$2"; shift 2;;
    --out) OUT="$2"; shift 2;;
    --label) LABEL="$2"; shift 2;;
    --known-hosts) KNOWN_HOSTS="$2"; shift 2;;
    --skip-ros) SKIP_ROS=1; shift;;
    --topic-timeout) TOPIC_TIMEOUT="$2"; shift 2;;
    --manifest) MANIFEST="$2"; shift 2;;
    --dry-run) DRY=1; shift;;
    --guard-self-test) GUARD_TEST=1; shift;;
    -h|--help) usage; exit 0;;
    *) echo "알 수 없는 인자: $1 (--help 참고)" >&2; exit 2;;
  esac
done
case "$SSH_PORT$WEB_PORT$MJPEG_PORT$ROSBRIDGE_PORT$WAIT$TOPIC_TIMEOUT" in
  *[!0-9]*) echo "포트·시간 인자는 숫자여야 한다" >&2; exit 2;;
esac
case "$HOST$RUSER" in
  *[!A-Za-z0-9._:-]*) echo "호스트·사용자 이름에 쓸 수 없는 문자가 있다" >&2; exit 2;;
esac

# ───────────── 금지 명령 검사기 (perl) ─────────────
# 로봇에 보내는 명령은 "허용 명령 이름 목록 + 명령별 금지 인자"로 검사한다. 목록에 없는 명령은 모두 막는다.
GUARD_PL="$(cat <<'PERL'
use strict; use warnings;
my $cmd = join(' ', @ARGV);
my %ALLOW = map { $_ => 1 } qw(
  ls stat readlink realpath uname id whoami hostname uptime date cat head tail grep sort uniq cut wc tr sed
  find sha256sum free df lsblk findmnt ss ip iw nmcli rfkill lsusb lsmod gpioinfo gpiodetect systemctl
  journalctl timedatectl systemd-analyze v4l2-ctl ros2 echo printf true false test [ source . export cd
  timeout dpkg-query diff cmp sleep ps pgrep nproc getent basename dirname read);
my %KW_NEXT = map { $_ => 1 } qw(if then else elif while until do ! { time);
my %KW_SKIP = map { $_ => 1 } qw(fi done esac } for case in);
my @why;

sub split_top {   # 따옴표 밖의 ; & | 줄바꿈으로 나눈다. $( ) 안은 따로 모은다
  my ($s) = @_;
  my (@seg, @subs); my $cur = ''; my ($sq, $dq) = (0, 0); my $i = 0; my $n = length $s;
  while ($i < $n) {
    my $c = substr($s, $i, 1);
    if (!$sq && $c eq '\\' && $i + 1 < $n) { $cur .= substr($s, $i, 2); $i += 2; next; }
    if (!$dq && $c eq "'") { $sq = !$sq; $cur .= $c; $i++; next; }
    if (!$sq && $c eq '"') { $dq = !$dq; $cur .= $c; $i++; next; }
    if (!$sq && $c eq '$' && substr($s, $i + 1, 1) eq '(') {
      my $depth = 0; my $j = $i + 1; my $start = $i + 2; my ($q1, $q2) = (0, 0);
      for (; $j < $n; $j++) {
        my $d = substr($s, $j, 1);
        if (!$q2 && $d eq "'") { $q1 = !$q1; next; }
        if (!$q1 && $d eq '"') { $q2 = !$q2; next; }
        next if $q1;
        $depth++ if $d eq '('; $depth-- if $d eq ')';
        last if $depth == 0;
      }
      push @subs, substr($s, $start, $j - $start);
      $cur .= ' __SUBST__ '; $i = $j + 1; next;
    }
    if (!$sq && $c eq '`') { push @why, "백틱 명령 치환 금지"; $i++; next; }
    if (!$sq && !$dq && $c eq '>') { push @why, "출력 리다이렉션(파일 쓰기) 금지"; }
    if (!$sq && !$dq && ($c eq ';' || $c eq '&' || $c eq '|' || $c eq "\n")) { push @seg, $cur; $cur = ''; $i++; next; }
    $cur .= $c; $i++;
  }
  push @seg, $cur;
  return (\@seg, \@subs);
}

sub tokens {
  my ($s) = @_; my @t; my $cur = ''; my ($sq, $dq) = (0, 0); my $has = 0;
  for my $c (split //, $s) {
    if (!$dq && $c eq "'") { $sq = !$sq; $has = 1; next; }
    if (!$sq && $c eq '"') { $dq = !$dq; $has = 1; next; }
    if (!$sq && !$dq && $c =~ /\s/) { push @t, $cur if $has || length $cur; $cur = ''; $has = 0; next; }
    $cur .= $c;
  }
  push @t, $cur if $has || length $cur;
  return @t;
}

sub check_cmd {
  my ($all) = @_;
  # 허용된 리다이렉션만 지운 뒤 남은 > 는 파일 쓰기로 보고 막는다
  (my $s = $all) =~ s{\d?>&\d|\d?>\s*/dev/null}{ }g;
  my ($segs, $subs) = split_top($s);
  check_cmd($_) for @$subs;
  for my $seg (@$segs) {
    my @t = tokens($seg);
    while (@t) {
      my $w = $t[0];
      if ($KW_SKIP{$w}) { @t = (); last; }
      if ($KW_NEXT{$w} || $w =~ /^\w+=/ || $w eq '(' || $w eq ')' ) { shift @t; next; }
      last;
    }
    next unless @t;
    if ($t[0] eq 'timeout') {
      shift @t;
      while (@t) {
        if ($t[0] eq '-s' || $t[0] eq '-k' || $t[0] eq '--signal' || $t[0] eq '--kill-after') { splice(@t, 0, 2); next; }
        if ($t[0] =~ /^-/) { shift @t; next; }
        if ($t[0] =~ /^\d+(\.\d+)?[smhd]?$/) { shift @t; last; }
        last;
      }
    }
    next unless @t;
    my $name = $t[0]; $name =~ s{.*/}{};
    next if $name eq '__SUBST__';
    my @a = @t[1 .. $#t];
    my $args = join(' ', @a);
    unless ($ALLOW{$name}) { push @why, "허용 목록에 없는 명령 '$name'"; next; }
    if ($name eq 'systemctl') {
      my @v = grep { !/^-/ } @a;
      my %ok = map { $_ => 1 } qw(status is-active is-enabled is-failed show list-units list-unit-files list-jobs is-system-running cat list-dependencies list-timers);
      if (@v && !$ok{$v[0]}) { push @why, "systemctl $v[0] 금지(읽기 하위 명령만 허용)"; }
    } elsif ($name eq 'journalctl') {
      push @why, "journalctl 쓰기 옵션 금지" if grep { /^--(vacuum|rotate|flush|sync|relinquish|smart-relinquish|setup-keys|update-catalog)/ } @a;
    } elsif ($name eq 'ros2') {
      my @v = grep { !/^-/ } @a;
      my %ok = (topic => {map { $_ => 1 } qw(list echo hz info type bw find)}, node => {list => 1, info => 1},
                service => {list => 1, type => 1, find => 1}, interface => {show => 1, list => 1});
      if (!@v || !$ok{$v[0]} || !defined $v[1] || !$ok{$v[0]}{$v[1]}) {
        push @why, "ros2 " . join(' ', @v[0 .. ($#v < 1 ? $#v : 1)]) . " 금지(목록·읽기만 허용, 서비스 호출·발행·실행 금지)";
      }
    } elsif ($name eq 'nmcli') {
      my %bad = map { $_ => 1 } qw(up down connect disconnect modify mod delete del add reload load import export clone edit monitor radio networking hostname set reapply autoconnect -s --show-secrets -a --ask secrets);
      push @why, "nmcli 변경·비밀값 인자 금지" if grep { $bad{$_} } @a;
      for my $k (0 .. $#a - 1) { push @why, "nmcli --rescan 은 no 만 허용" if $a[$k] eq '--rescan' && $a[$k + 1] ne 'no'; }
    } elsif ($name eq 'ip') {
      push @why, "ip 변경 인자 금지" if grep { /^(set|add|del|delete|flush|change|replace|append|prepend|save|restore|exec|netns)$/ } @a;
    } elsif ($name eq 'iw') {
      push @why, "iw 변경·스캔 인자 금지" if grep { /^(set|connect|disconnect|scan|del|add|join|leave|switch|ibss|mesh|ocb|offchannel|vendor|wowlan|coalesce|cqm|reg)$/ } @a;
    } elsif ($name eq 'rfkill') {
      push @why, "rfkill block/unblock 금지" if grep { /^(block|unblock|toggle)$/ } @a;
    } elsif ($name eq 'timedatectl') {
      push @why, "timedatectl 설정 금지" if grep { /^set/ } @a;
    } elsif ($name eq 'v4l2-ctl') {
      push @why, "v4l2-ctl 설정·스트리밍 금지" if grep { /^--(set|stream|try|overlay|fbuf|silence|wait|sleep|poll|epoll|sliced)/ || /^-(c|s|p|v|i|o|f|x|e)$/ } @a;
    } elsif ($name eq 'date') {
      push @why, "date 설정 금지" if grep { /^(-s|--set)/ } @a;
    } elsif ($name eq 'hostname') {
      push @why, "hostname 설정 금지" if grep { !/^-/ } @a;
    } elsif ($name eq 'sed') {
      push @why, "sed -i 금지" if grep { /^-[a-zA-Z]*i|^--in-place/ } @a;
    } elsif ($name eq 'find') {
      push @why, "find 실행·삭제 동작 금지" if grep { /^-(delete|exec|execdir|ok|okdir|fprint|fprint0|fprintf|fls)$/ } @a;
    } elsif ($name eq 'source' || $name eq '.') {
      push @why, "source 는 /opt/ros, /opt/armpi 아래 setup.bash 만" unless @a == 1 && $a[0] =~ m{^/opt/(ros/humble|armpi/ros2_ws/install)/setup\.bash$};
    } elsif ($name eq 'export') {
      push @why, "export 는 ROS_DOMAIN_ID·ROS_LOCALHOST_ONLY 만" if grep { !/^(ROS_DOMAIN_ID|ROS_LOCALHOST_ONLY)=\d+$/ } @a;
    }
    if ($name =~ /^(cat|head|tail|grep|sort|cut|wc|diff|cmp|sha256sum|tr|uniq)$/) {
      push @why, "$name 로 장치 파일 읽기 금지(직렬·I2C·GPIO·카메라)" if grep { m{^/dev/} && $_ ne '/dev/null' } @a;
    }
  }
}
check_cmd($cmd);
if (@why) { my %u; print "DENY: ", join('; ', grep { !$u{$_}++ } @why), "\n"; exit 1; }
print "OK\n"; exit 0;
PERL
)"

guard() { perl -e "$GUARD_PL" -- "$1"; }

# Mac 쪽 명령은 첫 낱말만 허용 목록으로 확인한다(로봇에 닿는 것은 연결 확인뿐)
LOCAL_ALLOW=" ping nc ssh ssh-keyscan ssh-keygen dscacheutil getent perl "
guard_local() {
  local first="${1%% *}"
  case "$LOCAL_ALLOW" in *" $first "*) echo OK; return 0;; esac
  echo "DENY: Mac 쪽 허용 목록에 없는 명령 '$first'"; return 1
}

# 저장 전에 비밀값처럼 보이는 문자열을 가린다
MASK_PL='
  if (/-----BEGIN [A-Z ]*PRIVATE KEY-----/) { $in = 1; }
  if ($in) { $_ = "***(개인키 줄 가림)\n"; $in = 0 if /-----END [A-Z ]*PRIVATE KEY-----/; next; }
  s/((?:psk|password|passwd|passphrase|secret|token|private[_-]?key|pre-shared-key)[\w.-]*\s*[=:]\s*)\S+/$1***(가림)/gi;
  s/\$(?:1|2[aby]|5|6|y|gy|7)\$[^\s:]+/***(해시 가림)/g;
'
mask() { perl -pe "$MASK_PL"; }

# ───────────── 점검 목록 ─────────────
R_ID=(); R_REF=(); R_TITLE=(); R_CMD=()
add_remote() {  # add_remote ID 참조 제목  (명령은 표준 입력)
  local c; c="$(cat)"
  R_ID[${#R_ID[@]}]="$1"; R_REF[${#R_REF[@]}]="$2"; R_TITLE[${#R_TITLE[@]}]="$3"; R_CMD[${#R_CMD[@]}]="$c"
}
ROSENV='source /opt/ros/humble/setup.bash >/dev/null 2>&1; source /opt/armpi/ros2_ws/install/setup.bash >/dev/null 2>&1; export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0;'
TT="$TOPIC_TIMEOUT"

add_remote R01 B1 "기본 정보(사용자·호스트·커널·OS·가동 시간)" <<'CMD'
id; hostname; uname -a; head -4 /etc/os-release; uptime; date '+%Y-%m-%d %H:%M:%S %Z (%s)'; cat /proc/uptime
CMD
add_remote R02 B1 "부팅 소요 시간" <<'CMD'
systemd-analyze 2>&1; systemd-analyze blame 2>&1 | head -15; systemd-analyze critical-chain armpi.service 2>&1 | head -20
CMD
add_remote R03 B2,B3 "네트워크(주소·경로·NetworkManager 연결 이름·무선 신호)" <<'CMD'
ip -br addr; ip route; nmcli -t -f DEVICE,TYPE,STATE,CONNECTION device 2>&1; nmcli -t -f NAME,TYPE,DEVICE,STATE connection show --active 2>&1; nmcli -t -f ACTIVE,SSID,CHAN,FREQ,SIGNAL,SECURITY device wifi list --rescan no 2>&1 | grep '^yes:'; iw dev wlan0 link 2>&1 | grep -E 'Connected|SSID|freq|signal|bitrate'; rfkill list 2>&1
CMD
add_remote R04 B3,B4 "대체 AP 판단 서비스(armpi-net-fallback) 상태·로그" <<'CMD'
systemctl status armpi-net-fallback --no-pager -l 2>&1 | head -20; journalctl -u armpi-net-fallback -b --no-pager 2>&1 | tail -40
CMD
add_remote R05 B5 "mDNS(avahi) 상태·자기 이름 확인" <<'CMD'
systemctl is-active avahi-daemon 2>&1; hostname; getent hosts "$(hostname).local" 2>&1
CMD
add_remote R06 B7 "서비스 상태 표(active·enabled)" <<'CMD'
for u in armpi.service armpi-web.service armpi-shutdown.service NetworkManager.service avahi-daemon.service ssh.service systemd-timesyncd.service armpi-net-fallback.service armpi-ssh-hostkeys.service armpi-growroot.service; do printf 'UNIT %s %s %s\n' "$u" "$(systemctl is-active "$u" 2>&1)" "$(systemctl is-enabled "$u" 2>&1)"; done; echo "system: $(systemctl is-system-running 2>&1)"; systemctl --failed --no-pager 2>&1
CMD
add_remote R07 B7 "armpi 서비스 자세히(재시작 횟수 포함)" <<'CMD'
systemctl status armpi.service armpi-web.service armpi-shutdown.service --no-pager -l 2>&1 | head -80; systemctl show armpi.service -p NRestarts -p ActiveEnterTimestamp -p ExecMainStartTimestamp -p ExecMainStatus -p Result 2>&1
CMD
add_remote R08 B8 "armpi.service 로그(이번 부팅, 끝 150줄)" <<'CMD'
journalctl -u armpi.service -b --no-pager 2>&1 | tail -150
CMD
add_remote R09 B8 "armpi.service 로그 요약(시작 줄·FAULT·ERROR 개수)" <<'CMD'
echo "COUNT 실제하드웨어_시작줄 $(journalctl -u armpi.service -b --no-pager 2>/dev/null | grep -c '실제 하드웨어')"; echo "COUNT FAULT줄 $(journalctl -u armpi.service -b --no-pager 2>/dev/null | grep -ciw fault)"; echo "COUNT ERROR줄 $(journalctl -u armpi.service -b --no-pager 2>/dev/null | grep -c 'ERROR')"
CMD
add_remote R10 B9 "기준판 매니페스트(stage20.env, stage30.env)" <<'CMD'
ls -l /opt/armpi/manifests 2>&1; cat /opt/armpi/manifests/stage30.env 2>&1; cat /opt/armpi/manifests/stage20.env 2>&1
CMD
add_remote R11 B9 "설치된 소스 트리 해시 재계산(빌드 때와 같은 방법)" <<'CMD'
for d in /opt/armpi/ros2_ws/src /etc/armpi /opt/armpi/vr_protocol_v3 /opt/armpi/mac_controller; do n="$(echo "${d#/}" | tr '/' '_')"; if [ -d "$d" ]; then cur="$(cd "$d" && find . -type f ! -name '._*' ! -name '.DS_Store' ! -path '*/.git/*' ! -path '*/__pycache__/*' ! -name '*.pyc' ! -path '*/.pytest_cache/*' ! -path '*/.mypy_cache/*' ! -path '*/.idea/*' ! -path '*/.vscode/*' -print | LC_ALL=C sort | while IFS= read -r f; do printf '%s  %s\n' "$(sha256sum "$f" | cut -d' ' -f1)" "$f"; done | sha256sum | cut -d' ' -f1)"; else cur=absent; fi; stored="$(sha256sum "/opt/armpi/manifests/tree_$n.sha256" 2>/dev/null | cut -d' ' -f1)"; echo "TREE $n current=$cur stored=$stored"; done
CMD
add_remote R12 B10 "시간 동기" <<'CMD'
timedatectl 2>&1; timeout 8 timedatectl timesync-status 2>&1 | head -20; echo "timesyncd: $(systemctl is-active systemd-timesyncd 2>&1)"
CMD
add_remote R13 B11 "직렬 콘솔이 꺼져 있는지(cmdline·getty·장치)" <<'CMD'
cat /proc/cmdline; echo "CONSOLE_SERIAL_IN_CMDLINE $(grep -c -E 'console=(serial0|ttyS0|ttyAMA0)' /proc/cmdline)"; for u in serial-getty@ttyS0.service serial-getty@serial0.service serial-getty@ttyAMA0.service; do printf 'GETTY %s %s\n' "$u" "$(systemctl is-enabled "$u" 2>&1)"; done; ls -l /dev/ttyS0 /dev/ttyAMA0 /dev/serial0 /dev/serial1 2>&1
CMD
add_remote R14 B11,S "서보 포트를 연 프로세스·GPIO 사용 현황(읽기만)" <<'CMD'
find /proc/[0-9]*/fd -maxdepth 1 \( -lname /dev/ttyS0 -o -lname /dev/ttyAMA0 \) -print 2>/dev/null; echo "(위 목록은 ubuntu 사용자 프로세스만 보인다)"; gpiodetect 2>&1; gpioinfo gpiochip0 2>&1
CMD
add_remote R15 B12 "I2C 장치 파일(탐지 신호 없이 목록만)" <<'CMD'
ls -l /dev/i2c* 2>&1; lsmod 2>/dev/null | grep -E '^i2c'
CMD
add_remote R16 B15 "카메라 장치·형식" <<'CMD'
ls -l /dev/video* 2>&1; v4l2-ctl --list-devices 2>&1; v4l2-ctl -d /dev/video0 --list-formats-ext 2>&1 | head -40
CMD
add_remote R17 B7 "열린 포트(SSH 22, MJPEG 8080, rosbridge 9090, 웹 18765, UDP 5001·5002)" <<'CMD'
ss -ltnup 2>&1
CMD
add_remote R18 B1 "자원·온도·전원 경고" <<'CMD'
free -m; df -h / /boot/firmware 2>&1; nproc; cat /proc/loadavg; echo "SOC_TEMP_mC $(cat /sys/class/thermal/thermal_zone0/temp 2>&1)"; echo "THROTTLED $(cat /sys/devices/platform/soc/soc:firmware/get_throttled 2>&1)"; echo "UNDERVOLTAGE_LINES $(journalctl -k -b --no-pager 2>/dev/null | grep -ci -E 'under-?voltage')"; journalctl -k -b -p warning --no-pager 2>&1 | tail -60
CMD
add_remote R19 B1 "디스크·파티션(PARTUUID 대조용)·첫 부팅 서비스 로그" <<'CMD'
lsblk -o NAME,SIZE,TYPE,FSTYPE,LABEL,PARTUUID,MOUNTPOINT 2>&1; echo "ROOT $(findmnt -n -o SOURCE,FSTYPE,OPTIONS / 2>&1)"; echo "BOOTFW $(findmnt -n -o SOURCE,FSTYPE /boot/firmware 2>&1)"; journalctl -u armpi-growroot -u armpi-ssh-hostkeys -b --no-pager 2>&1 | tail -30
CMD
add_remote R20 B9 "패키지 목록이 빌드 기록과 다른 줄 수(참고)" <<'CMD'
echo "DPKG_DIFF_LINES $(dpkg-query -W -f '${Package}\t${Version}\t${Architecture}\n' 2>/dev/null | diff /opt/armpi/manifests/stage20_dpkg.tsv - 2>/dev/null | grep -c '^[<>]')"
CMD
if [ "$SKIP_ROS" = 0 ]; then
add_remote R21 B12-B14 "ROS 노드·토픽 목록" <<CMD
$ROSENV timeout -s INT -k 5 20 ros2 node list --no-daemon --spin-time 5 2>&1; timeout -s INT -k 5 20 ros2 topic list -t --no-daemon --spin-time 5 2>&1
CMD
add_remote R22 B12,B14 "/armpi/status 1회(배터리 V·fan_ok·position_source)" <<CMD
$ROSENV timeout -s INT -k 5 $TT ros2 topic echo --once --full-length /armpi/status 2>&1 | head -40
CMD
add_remote R23 B12 "/armpi_control/hw_state 1회(바퀴·서보 쓰기 실패 수)" <<CMD
$ROSENV timeout -s INT -k 5 $TT ros2 topic echo --once --full-length /armpi_control/hw_state 2>&1 | head -40
CMD
add_remote R24 B13 "/armpi/battery 1회" <<CMD
$ROSENV timeout -s INT -k 5 $TT ros2 topic echo --once --full-length /armpi/battery 2>&1 | head -30
CMD
add_remote R25 B7,E "/armpi/control_state·/armpi_vr_bridge/state 1회" <<CMD
$ROSENV timeout -s INT -k 5 $TT ros2 topic echo --once --full-length /armpi/control_state 2>&1 | head -20; timeout -s INT -k 5 $TT ros2 topic echo --once --full-length /armpi_vr_bridge/state 2>&1 | head -20
CMD
add_remote R26 B15 "카메라 토픽 주기(8 s)" <<CMD
$ROSENV timeout -s INT -k 5 8 ros2 topic hz /usb_cam/image_raw 2>&1 | tail -8
CMD
fi

LOCAL_CMDS=()
local_cmd_list() {
  LOCAL_CMDS=()
  case "$HOST" in
    *[!0-9.]*) if [ "$(uname)" = Darwin ]; then LOCAL_CMDS[${#LOCAL_CMDS[@]}]="dscacheutil -q host -a name $HOST"; else LOCAL_CMDS[${#LOCAL_CMDS[@]}]="getent hosts $HOST"; fi;;
  esac
  if [ "$(uname)" = Darwin ]; then LOCAL_CMDS[${#LOCAL_CMDS[@]}]="ping -c 5 -t 10 $HOST"; else LOCAL_CMDS[${#LOCAL_CMDS[@]}]="ping -c 5 -w 10 $HOST"; fi
  for p in "$SSH_PORT" "$MJPEG_PORT" "$ROSBRIDGE_PORT" "$WEB_PORT"; do
    if [ "$(uname)" = Darwin ]; then LOCAL_CMDS[${#LOCAL_CMDS[@]}]="nc -z -G 3 -w 3 $HOST $p"; else LOCAL_CMDS[${#LOCAL_CMDS[@]}]="nc -z -w 3 $HOST $p"; fi
  done
  LOCAL_CMDS[${#LOCAL_CMDS[@]}]="ssh-keyscan -p $SSH_PORT -T 5 $HOST"
}
local_cmd_list

# ───────────── 검사기 자체 시험 ─────────────
if [ "$GUARD_TEST" = 1 ]; then
  fails=0; n=0
  echo "== 1) 이 스크립트의 원격 명령 ${#R_CMD[@]}개는 모두 통과해야 한다"
  i=0
  while [ $i -lt ${#R_CMD[@]} ]; do
    n=$((n+1)); r="$(guard "${R_CMD[$i]}")"
    if [ "$r" = OK ]; then echo "PASS 허용 ${R_ID[$i]}"; else echo "FAIL 허용 ${R_ID[$i]}: $r"; fails=$((fails+1)); fi
    i=$((i+1))
  done
  echo "== 2) Mac 쪽 명령 ${#LOCAL_CMDS[@]}개는 모두 통과해야 한다"
  for c in "${LOCAL_CMDS[@]}"; do n=$((n+1)); r="$(guard_local "$c")"; if [ "$r" = OK ]; then echo "PASS 허용 $c"; else echo "FAIL 허용 $c: $r"; fails=$((fails+1)); fi; done
  echo "== 3) 아래 명령은 모두 막혀야 한다"
  while IFS= read -r bad; do
    [ -n "$bad" ] || continue
    n=$((n+1)); r="$(guard "$bad")"
    case "$r" in DENY*) echo "PASS 차단 [$bad] → ${r#DENY: }";; *) echo "FAIL 통과됨 [$bad]"; fails=$((fails+1));; esac
  done <<'BAD'
sudo systemctl stop armpi.service
systemctl restart armpi.service
systemctl stop armpi.service
systemctl --no-pager start armpi.service
systemctl daemon-reload
systemctl reboot
reboot
sudo reboot
shutdown -h now
poweroff
halt
i2cdetect -y -r 1
i2cset -y 1 0x34 51 0 0 0 0 i
i2cget -y 1 0x40 0
gpioset gpiochip0 5=1
gpioget gpiochip0 5
ros2 service call /armpi/reset_estop std_srvs/srv/Trigger
ros2 topic pub --once /armpi/cmd_vel geometry_msgs/msg/TwistStamped '{}'
ros2 run armpi_control armpi_control_node
ros2 launch armpi_bringup armpi.launch.py
ros2 param set /armpi_control servo.read_position true
ros2 param get /armpi_control servo.read_position
ros2 lifecycle set /x configure
ros2 action send_goal /a b c
nmcli -s connection show armpi-wifi
nmcli --show-secrets connection show armpi-wifi
nmcli connection up armpi-wifi
nmcli connection down armpi-wifi
nmcli device wifi list --rescan yes
nmcli radio wifi off
ip link set wlan0 down
ip addr flush dev eth0
iw dev wlan0 scan
rfkill block wifi
timedatectl set-time 2020-01-01
date -s 2020-01-01
hostname newname
cat /dev/ttyS0
head -c 10 /dev/ttyS0
grep x /dev/i2c-1
stty -F /dev/ttyS0 115200
v4l2-ctl -d /dev/video0 --set-fmt-video=width=320
v4l2-ctl -c brightness=10
echo 1 > /sys/class/gpio/export
echo x | tee /tmp/a
rm -rf /tmp/x
sed -i s/a/b/ /etc/hosts
find /tmp -delete
find / -exec rm {} \;
journalctl --vacuum-time=1s
kill 1
pkill armpi_control
python3 -c 'print(1)'
bash -c 'reboot'
sh -c reboot
echo $(reboot)
echo $(i2cdetect -y 1)
ls `reboot`
dd if=/dev/zero of=/dev/sda
apt-get install foo
source /tmp/evil.sh
export PATH=/tmp
curl http://127.0.0.1:18765/api
xargs reboot
timeout -s INT -k 5 3 reboot
timeout 5 ros2 topic pub /armpi/cmd_vel geometry_msgs/msg/TwistStamped '{}'
timeout --signal=INT 3 i2cdetect -y 1
timeout -s KILL 3 systemctl stop armpi.service
BAD
  echo "합계: 시험 $n, 실패 $fails"
  [ "$fails" = 0 ] && exit 0 || exit 1
fi

# 실행 전에 모든 명령을 검사한다. 하나라도 걸리면 아무것도 실행하지 않는다.
i=0; bad_any=0
while [ $i -lt ${#R_CMD[@]} ]; do
  r="$(guard "${R_CMD[$i]}")"
  if [ "$r" != OK ]; then echo "금지 명령 검사 실패 ${R_ID[$i]}: $r" >&2; bad_any=1; fi
  i=$((i+1))
done
for c in "${LOCAL_CMDS[@]}"; do r="$(guard_local "$c")"; [ "$r" = OK ] || { echo "금지 명령 검사 실패(Mac): $r" >&2; bad_any=1; }; done
[ "$bad_any" = 0 ] || { echo "중단: 금지 명령이 들어 있어 아무것도 실행하지 않았다." >&2; exit 3; }

SSH_OPTS_DESC="ssh -p $SSH_PORT -i $KEY -o BatchMode=yes -o PasswordAuthentication=no -o KbdInteractiveAuthentication=no -o PreferredAuthentications=publickey -o IdentitiesOnly=yes -o StrictHostKeyChecking=accept-new -o UserKnownHostsFile=<결과폴더>/known_hosts $RUSER@$HOST bash -s"

if [ "$DRY" = 1 ]; then
  echo "first_boot_check $TOOL_VERSION — 실행할 명령(--dry-run, 네트워크 사용 안 함). 금지 명령 검사: 모두 통과"
  echo "대상: $RUSER@$HOST (SSH $SSH_PORT), 키 $KEY"
  echo; echo "[Mac 쪽]"
  for c in "${LOCAL_CMDS[@]}"; do echo "  $c"; done
  echo "  $SSH_OPTS_DESC <<< true   # 로그인 시험"
  echo "  (시계 차이) 위 SSH 로 date +%s.%N 을 5번 실행하고 Mac 시각과 비교"
  echo; echo "[로봇 쪽: 각 명령을 위 SSH 로 bash -s 표준 입력에 넣어 실행]"
  i=0
  while [ $i -lt ${#R_CMD[@]} ]; do
    echo "--- ${R_ID[$i]} (docs/10 ${R_REF[$i]}) ${R_TITLE[$i]}"
    echo "${R_CMD[$i]}" | sed 's/^/    /'
    i=$((i+1))
  done
  exit 0
fi

# ───────────── 실행 ─────────────
TS="$(date +%Y%m%d_%H%M%S)"
[ -n "$OUT" ] || OUT="$REPO/work/logs/first_boot_$TS"
mkdir -p "$OUT/local" "$OUT/remote" || exit 2
[ -n "$KNOWN_HOSTS" ] || KNOWN_HOSTS="$OUT/known_hosts"
LOG="$OUT/run.log"
say() { echo "$*" | tee -a "$LOG"; }
now() { perl -MTime::HiRes=time -e 'printf "%.3f\n", time'; }
T_START="$(now)"

CTL="$(mktemp -d /tmp/afbc.XXXXXX)"
SSH=(ssh -p "$SSH_PORT" -i "$KEY" -T -o BatchMode=yes -o PasswordAuthentication=no -o KbdInteractiveAuthentication=no
     -o PreferredAuthentications=publickey -o IdentitiesOnly=yes -o ConnectTimeout=8 -o ServerAliveInterval=5
     -o ServerAliveCountMax=3 -o StrictHostKeyChecking=accept-new -o "UserKnownHostsFile=$KNOWN_HOSTS" -o LogLevel=ERROR
     -o ControlMaster=auto -o "ControlPath=$CTL/%C" -o ControlPersist=60 "$RUSER@$HOST")
cleanup() { ssh -o "ControlPath=$CTL/%C" -O exit "$RUSER@$HOST" >/dev/null 2>&1; rm -rf "$CTL"; }
trap cleanup EXIT

{
  echo "first_boot_check $TOOL_VERSION"
  echo "시작: $(date '+%Y-%m-%d %H:%M:%S %Z')"
  echo "대상: $RUSER@$HOST (SSH $SSH_PORT), 웹 $WEB_PORT, MJPEG $MJPEG_PORT, rosbridge $ROSBRIDGE_PORT"
  echo "메모: ${LABEL:-(없음)}"
  echo "키: $KEY (내용은 읽거나 저장하지 않음)"
  echo "도구 SHA-256: $( (command -v shasum >/dev/null 2>&1 && shasum -a 256 "$SELF" || sha256sum "$SELF") | cut -d' ' -f1)"
  echo "Mac: $(uname -sr), $(sw_vers -productVersion 2>/dev/null)"
} > "$OUT/run_info.txt"
cat "$OUT/run_info.txt" | tee -a "$LOG"

run_local() {  # run_local 파일이름 명령
  local name="$1" c="$2" t0 t1 rc
  t0="$(now)"
  { echo "# 명령: $c"; echo "# 시작: $(date '+%H:%M:%S')"; } > "$OUT/local/$name.txt"
  # shellcheck disable=SC2086
  $c >> "$OUT/local/$name.txt" 2>&1; rc=$?
  t1="$(now)"
  echo "# 종료 코드: $rc, 소요 $(perl -e "printf '%.2f', $t1-$t0") s" >> "$OUT/local/$name.txt"
  return $rc
}

# B1: SSH 가 열릴 때까지 기다림
WAIT_NOTE=""
if [ "$WAIT" -gt 0 ]; then
  say "SSH 포트를 최대 ${WAIT}s 기다린다(스크립트 시작 = 0 s)..."
  port_open_at=""; login_at=""
  while :; do
    el="$(perl -e "printf '%.1f', $(now)-$T_START")"
    if [ -z "$port_open_at" ]; then
      if nc -z -G 2 -w 2 "$HOST" "$SSH_PORT" >/dev/null 2>&1 || nc -z -w 2 "$HOST" "$SSH_PORT" >/dev/null 2>&1; then port_open_at="$el"; say "  SSH 포트 열림: ${el}s"; fi
    fi
    if [ -n "$port_open_at" ] && "${SSH[@]}" bash -s <<< 'true' >/dev/null 2>&1; then login_at="$el"; say "  SSH 로그인 성공: ${el}s"; break; fi
    if perl -e "exit !($el > $WAIT)"; then say "  ${WAIT}s 안에 SSH 로그인 실패"; break; fi
    sleep 2
  done
  WAIT_NOTE="SSH 포트 열림 ${port_open_at:-없음} s, 로그인 ${login_at:-실패} s (스크립트 시작 기준. 전원을 넣는 순간 시작했다면 B1 값)"
  echo "$WAIT_NOTE" > "$OUT/local/L00_wait.txt"
fi

say "Mac 쪽 확인..."
li=0
for c in "${LOCAL_CMDS[@]}"; do
  li=$((li+1)); name="$(printf 'L%02d_%s' "$li" "$(echo "${c%% *}" | tr -c 'A-Za-z0-9\n' '_')")"
  case "$c" in nc\ *) name="${name}_${c##* }";; esac
  if [ "${c%% *}" = ssh-keyscan ]; then
    { echo "# 명령: $c | ssh-keygen -lf -"; $c 2>/dev/null | ssh-keygen -lf - 2>&1; echo "# 종료 코드: $?"; } > "$OUT/local/$name.txt"
  else
    run_local "$name" "$c"
  fi
done

say "SSH 로그인 시험(공개키만, 비밀번호 사용 안 함)..."
t0="$(now)"
login_out="$("${SSH[@]}" bash -s <<< 'echo LOGIN_OK; id -un' 2>&1)"; login_rc=$?
t1="$(now)"
{ echo "# $SSH_OPTS_DESC <<< 'echo LOGIN_OK; id -un'"; echo "$login_out" | mask; echo "# 종료 코드: $login_rc, 소요 $(perl -e "printf '%.2f', $t1-$t0") s"; } > "$OUT/local/L90_ssh_login.txt"
if [ $login_rc -ne 0 ] || ! echo "$login_out" | grep -q LOGIN_OK; then
  say "SSH 로그인 실패(종료 코드 $login_rc). 원격 점검을 건너뛴다. 출력: $(echo "$login_out" | mask | head -3 | tr '\n' ' ')"
  SSH_OK=0
else
  say "SSH 로그인 성공 ($(perl -e "printf '%.2f', $t1-$t0") s)"
  SSH_OK=1
fi

CLOCK_NOTE=""
if [ "$SSH_OK" = 1 ]; then
  say "시계 차이 추정(5회)..."
  {
    echo "# 시계 차이 = 로봇 시각 − (Mac 보내기 시각 + Mac 받기 시각)/2, 오차 범위 ±왕복/2. 왕복이 가장 짧은 표본을 쓴다"
    best=""
    for k in 1 2 3 4 5; do
      a="$(now)"; r="$("${SSH[@]}" bash -s <<< 'date +%s.%N' 2>/dev/null)"; b="$(now)"
      case "$r" in ""|*[!0-9.]*) echo "표본 $k: 로봇 시각 읽기 실패"; continue;; esac
      perl -e 'my ($k, $a, $b, $r) = @ARGV; printf "표본 %d: 왕복 %.1f ms, 차이 %+.3f s\n", $k, ($b - $a) * 1000, $r - ($a + $b) / 2' "$k" "$a" "$b" "$r"
    done
  } > "$OUT/local/L91_clock_offset.txt" 2>&1
  CLOCK_NOTE="$(perl -ne 'if(/왕복 ([0-9.]+) ms, 차이 ([+-][0-9.]+) s/){ if(!defined $m || $1<$m){$m=$1;$o=$2} } END{ print defined $m ? "로봇 − Mac = ${o} s (±" . sprintf("%.1f",$m/2) . " ms, 왕복 최소 ${m} ms)" : "측정 실패" }' "$OUT/local/L91_clock_offset.txt")"
  echo "# 결과: $CLOCK_NOTE" >> "$OUT/local/L91_clock_offset.txt"
  say "  $CLOCK_NOTE"

  say "로봇 쪽 확인 ${#R_CMD[@]}개..."
  i=0
  while [ $i -lt ${#R_CMD[@]} ]; do
    id="${R_ID[$i]}"; f="$OUT/remote/${id}.txt"
    t0="$(now)"
    out="$(printf '%s\n' "${R_CMD[$i]}" | "${SSH[@]}" bash -s 2>&1)"; rc=$?
    t1="$(now)"
    dur="$(perl -e "printf '%.2f', $t1-$t0")"
    {
      echo "# ${id} (docs/10 ${R_REF[$i]}) ${R_TITLE[$i]}"
      echo "# 명령:"; printf '%s\n' "${R_CMD[$i]}" | sed 's/^/#   /'
      echo "# 종료 코드: $rc, 소요 ${dur} s"
      echo "$out"
    } | mask > "$f"
    printf '%s\t%s\t%s\t%s\t%s\n' "$id" "${R_REF[$i]}" "${R_TITLE[$i]}" "$rc" "$dur" >> "$OUT/remote/_index.tsv"
    say "  ${id} 종료 ${rc} (${dur}s) ${R_TITLE[$i]}"
    i=$((i+1))
  done
fi

# ───────────── 요약 ─────────────
S="$OUT/summary.md"
# ros2 topic echo 의 "data: '{...}'" 한 줄을 JSON 으로 읽어 표 행을 만든다
json_rows() {  # json_rows 파일 이름 docs10 키...
  local f="$1" title="$2" ref="$3"; shift 3
  perl -MJSON::PP -e '
    my ($f, $title, $ref, @keys) = @ARGV; my $j;
    if (open my $h, "<", $f) { while (<$h>) { if (/^data: \x27(.*)\x27\s*$/) { (my $t = $1) =~ s/\x27\x27/\x27/g; $j = eval { JSON::PP->new->utf8->decode($t) }; last; } } }
    for my $k (@keys) {
      my $v;
      if (!$j) { $v = "(못 읽음)"; } elsif (!exists $j->{$k}) { $v = "(키 없음)"; }
      else { $v = JSON::PP->new->canonical->allow_nonref->encode($j->{$k}); $v =~ s/\|/\//g;
             $v = substr($v, 0, 300) . "..." if length($v) > 300; utf8::encode($v); }
      print "| $title $k | $ref | $v |\n";
    }' "$f" "$title" "$ref" "$@"
}
val() { grep -h "$2" "$OUT/remote/$1.txt" 2>/dev/null | head -${3:-1}; }
{
  echo "# 첫 부팅 점검 요약 (자동 수집, 읽기 전용)"
  echo
  cat "$OUT/run_info.txt" | sed 's/^/- /'
  echo "- 끝: $(date '+%Y-%m-%d %H:%M:%S %Z')"
  echo
  echo "**PASS/FAIL 판정은 사람이 docs/10 기준과 아래 원자료로 한다. 아래 '관찰'은 자동으로 뽑은 값이다.**"
  echo
  echo "## Mac 쪽"
  echo
  [ -n "$WAIT_NOTE" ] && echo "- 부팅 대기(B1): $WAIT_NOTE"
  for f in "$OUT"/local/L0*_ping*.txt; do [ -f "$f" ] && echo "- ping: $(grep -E 'packet loss|packets transmitted' "$f" | head -1) / $(grep -E 'round-trip|rtt' "$f" | head -1)"; done
  for f in "$OUT"/local/L0*_nc_*.txt; do [ -f "$f" ] || continue; p="${f##*_}"; p="${p%.txt}"; rc="$(sed -n 's/^# 종료 코드: \([0-9]*\).*/\1/p' "$f")"; echo "- TCP $p: $([ "$rc" = 0 ] && echo 열림 || echo "닫힘/응답 없음(코드 $rc)")"; done
  for f in "$OUT"/local/L0*_ssh_keyscan*.txt; do [ -f "$f" ] && { echo "- SSH 호스트 키 지문:"; grep -E '^[0-9]+ SHA256' "$f" | sed 's/^/  - /'; }; done
  echo "- SSH 로그인: $([ "$SSH_OK" = 1 ] && echo "성공" || echo "실패 → 원격 점검 안 함")"
  [ -n "$CLOCK_NOTE" ] && echo "- 시계 차이: $CLOCK_NOTE"
  echo
  if [ "$SSH_OK" = 1 ]; then
    echo "## 관찰 (자동 추출)"
    echo
    echo "| 항목 | docs/10 | 관찰 |"
    echo "|---|---|---|"
    while read -r _ u a e; do echo "| 서비스 $u | B7 | $a / $e |"; done < <(grep '^UNIT ' "$OUT/remote/R06.txt" 2>/dev/null)
    echo "| 시스템 상태 | B7 | $(val R06 '^system:' | sed 's/^system: //') |"
    grep '^COUNT ' "$OUT/remote/R09.txt" 2>/dev/null | while read -r _ k v; do echo "| armpi 로그 $k | B8 | $v |"; done
    echo "| armpi.service 재시작 횟수(NRestarts) | B8 | $(val R07 '^NRestarts=' | cut -d= -f2) |"
    echo "| 시간 동기 | B10 | $(val R12 'synchronized' | sed 's/^ *//') |"
    echo "| cmdline 의 직렬 콘솔 | B11 | $(val R13 '^CONSOLE_SERIAL_IN_CMDLINE' | awk '{print ($2=="0")?"없음(정상 기대값)":"있음("$2")"}') |"
    grep '^GETTY ' "$OUT/remote/R13.txt" 2>/dev/null | while read -r _ u s; do echo "| $u | B11 | $s |"; done
    echo "| /dev/i2c-1 | B12 | $(grep -q 'i2c-1' "$OUT/remote/R15.txt" 2>/dev/null && echo 있음 || echo 없음) |"
    echo "| /dev/video* | B15 | $(grep -c '/dev/video' "$OUT/remote/R16.txt" 2>/dev/null)개 줄 |"
    echo "| SoC 온도 | — | $(val R18 '^SOC_TEMP_mC' | awk '{ if ($2 ~ /^[0-9]+$/) printf "%.1f °C", $2/1000; else { $1=""; print "읽지 못함:" $0 } }') |"
    echo "| get_throttled | — | $(val R18 '^THROTTLED' | cut -d' ' -f2-) (0x0 이 아니면 전압·온도 제한 이력) |"
    echo "| 커널 저전압 경고 줄 | — | $(val R18 '^UNDERVOLTAGE_LINES' | cut -d' ' -f2) |"
    echo "| 패키지 차이 줄(빌드 20단계 기록 대비) | B9 | $(val R20 '^DPKG_DIFF_LINES' | cut -d' ' -f2) (30단계 이후 설치분이 있으면 0이 아닐 수 있음) |"
    if [ -f "$OUT/remote/R22.txt" ]; then
      json_rows "$OUT/remote/R22.txt" "/armpi/status" "B12-B14" hardware battery_v battery_low fan_ok position_source fault fault_reason emergency_stop estop_sources base_state arm_state warnings
      json_rows "$OUT/remote/R23.txt" "hw_state" "B12, S1" mock wheel_writes wheel_failures servo_writes servo_failures fan_ok
      echo "| 카메라 주기 | B15 | $(grep -E 'average rate' "$OUT/remote/R26.txt" 2>/dev/null | tail -1 | sed 's/^ *//')$(grep -q 'average rate' "$OUT/remote/R26.txt" 2>/dev/null || echo '(메시지 없음)') |"
    fi
    echo
    echo "## 설치 파일 대조 (USB 굽기 간접 확인)"
    echo
    echo "로봇에서 다시 계산한 트리 해시(current), 로봇에 저장된 빌드 기록(stored), Mac 의 이미지 매니페스트(manifest)를 비교한다."
    echo
    if [ -f "$MANIFEST" ]; then
      grep '^TREE ' "$OUT/remote/R11.txt" 2>/dev/null | perl -MJSON::PP -e '
        local $/; my $j = do { open my $f, "<", $ARGV[0] or die; local $/; decode_json(<$f>) };
        my $t = $j->{workspace}{trees}; my $all = 1; my $n = 0;
        print "| 트리 | current = stored | stored = manifest | 판정(자동) |\n|---|---|---|---|\n";
        for (split /\n/, <STDIN>) {
          my ($name, $cur, $sto) = /^TREE (\S+) current=(\S*) stored=(\S*)/ or next; $n++;
          my $man = $t->{$name} ? $t->{$name}{tree_sha256} : "";
          my $a = ($cur ne "" && $cur eq $sto) ? "같음" : "다름"; my $b = ($sto ne "" && $sto eq $man) ? "같음" : "다름";
          $all = 0 unless $a eq "같음" && $b eq "같음";
          printf "| %s | %s | %s | %s |\n", $name, $a, $b, ($a eq "같음" && $b eq "같음") ? "일치" : "확인 필요";
        }
        print "\n전체: ", ($n && $all) ? "모든 트리 일치($n개)" : "확인 필요($n개 중 불일치 있음)", "\n";
      ' "$MANIFEST"
      pr="$(perl -MJSON::PP -e 'local $/; open my $f,"<",$ARGV[0]; my $j=decode_json(<$f>); my $id=$j->{image}{partition_table}{id} // ""; $id =~ s/^0x//; my $n=scalar @{$j->{image}{partition_table}{partitions}||[]}; print join(" ", map { sprintf "%s-%02d", $id, $_ } 1..$n)' "$MANIFEST" 2>/dev/null)"
      echo
      echo "- 매니페스트로 계산한 PARTUUID(디스크 ID-번호): ${pr:-(읽지 못함)}"
      echo "- 로봇의 파티션 UUID(lsblk): $(grep -oE '[0-9a-f]{8}-0[12]' "$OUT/remote/R19.txt" 2>/dev/null | sort -u | tr '\n' ' ')"
      echo "- 로봇 커널 명령행의 root: $(grep -oE 'root=[^ ]+' "$OUT/remote/R13.txt" 2>/dev/null | head -1)"
    else
      echo "(매니페스트 $MANIFEST 없음)"
    fi
    echo
    echo "## 원격 명령 결과 목록"
    echo
    echo "| ID | docs/10 | 항목 | 종료 코드 | 소요(s) |"
    echo "|---|---|---|---|---|"
    while IFS="$(printf '\t')" read -r a b c d e; do echo "| $a | $b | $c | $d | $e |"; done < "$OUT/remote/_index.tsv"
    echo
    echo "종료 코드가 0 이 아니어도 실패라고 단정하지 않는다(예: grep 에 맞는 줄이 없으면 1, timeout 으로 끝낸 ros2 topic hz 는 124). 원자료 파일을 본다."
  fi
  echo
  echo "## 사람이 따로 확인할 것 (자동으로 할 수 없음)"
  echo
  echo "- B4 대체 AP(등록된 와이파이가 없는 곳에서 부팅), B13 멀티미터 전압과 비교, B14 팬 소리, B15 브라우저로 영상 보기, B16 재부팅 뒤 반복"
  echo "- S·A·C·E·V·W 단계는 이 도구가 하지 않는다(로봇을 움직이지 않음)."
} > "$S" 2>>"$LOG"

say "요약: $S"
[ "$SSH_OK" = 1 ] && exit 0 || exit 4
