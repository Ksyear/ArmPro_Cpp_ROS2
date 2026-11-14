// ArmPi Protocol v3 — VR(Unity) 쪽 참고용 C# 코드 (Unity 의존 없음).
//
// 기준 문서: docs/01_PROTOCOL_V3_SPEC.md, docs/08_VR_HANDOFF_KO.md
// 참조 구현: protocol_v3/armpi_v3_client.py (Python), 가짜 로봇: protocol_v3/mock_robot.py
//
// 이 파일에 있는 것
//   1. ArmPiV3            상수(포트·주기·시간값), 상태·사유 문자열
//   2. ArmPiV3Json        command(8필드)·arm_command(10필드)·event(4필드) JSON 만들기
//   3. ArmPiV3Ack         ACK(정확히 8필드) 엄격 해석 — 실수·중복 키·추가 필드 거부
//   4. ArmPiV3Discovery   검색 요청·응답(ARMPI_DISCOVER/1, ARMPI_HERE/1)
//   5. ArmPiFrames        Unity(왼손, Y 위) ↔ ROS REP-103(오른손, Z 위) 좌표 변환
//   6. ArmPiKinematics    로봇 armpi_kinematics 와 같은 식의 IK/FK(보내기 전 미리 확인용)
//      ArmPiTargetProjector 로봇 arm.target_policy: nearest 와 같은 보정(닿지 않는 목표 → 닿는 가장 가까운 자세)
//   7. ArmTeleopMapper    컨트롤러 6자유도 자세 → 손끝 목표(상대 이동 + 작업 공간 클램프 + 속도 제한) 제안
//
// 이 파일에 없는 것: 소켓·스레드·세션 상태기계(중립 확인, 무장, ACK 시간 초과, 비상정지 재전송).
//   그 규칙은 docs/08_VR_HANDOFF_KO.md 의 "세션 규칙"과 Python 참조 구현 ArmPiV3Client 를 따른다.
//
// 언어 수준: C# 9 이하 문법만 쓴다(Unity 2021+ 와 맞춤). System.* 만 쓴다.
// 증거 수준: .NET 8 콘솔에서 컴파일하고 Python 가짜 로봇과 루프백 왕복 시험만 했다(clients/vr_handoff/selftest/).
//   Unity 에디터·Quest 실기·실제 로봇에서는 실행하지 않았다.
// 작성 2026-11-02.

using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text;

namespace ArmPi.V3
{
    // ───────────────────────── 1. 상수 ─────────────────────────

    public static class ArmPiV3
    {
        public const int Protocol = 3;
        public const int ControlPort = 5001;           // UDP 조종 + ACK(같은 소켓)
        public const int DiscoveryPort = 5002;         // UDP 검색(응답 출발 포트도 5002)
        public const int MaxPacketBytes = 2048;
        public const long MaxSequence = 9007199254740991L;  // 2^53 − 1
        public const double SendPeriodSec = 0.05;      // 채널(차체·팔)마다 20 Hz
        public const double AckTimeoutSec = 0.30;      // 이 시간 동안 ACK 가 없으면 중립으로
        public const double EstopRetrySec = 0.05;      // 비상정지는 ACK 를 받을 때까지 50 ms 마다 재전송
        public const double RobotWatchdogSec = 0.4;    // 로봇 쪽 채널 watchdog(참고용, 로봇 설정값)
        public const double RobotLeaseSec = 1.0;       // 로봇 쪽 소유권 lease(참고용)
        public const double ReconnectDelaySec = 1.1;   // 같은 UUID 로 다시 붙을 때 기다림(lease 1.0 s 보다 길게)

        // ACK state (그 패킷이 속한 채널의 상태)
        public const string StateAwaitingNeutral = "awaiting_neutral";
        public const string StateIdle = "idle";
        public const string StateActive = "active";
        public const string StateTimeout = "timeout";
        public const string StateEmergencyStop = "emergency_stop";
        public const string StateFault = "fault";

        // ACK reason
        public const string ReasonOk = "ok";
        public const string ReasonInvalidPacket = "invalid_packet";
        public const string ReasonUnsupportedProtocol = "unsupported_protocol";
        public const string ReasonOutOfOrder = "out_of_order";
        public const string ReasonNotOwner = "not_owner";
        public const string ReasonEmergencyStop = "emergency_stop";
        public const string ReasonNeutralRequired = "neutral_required";
        public const string ReasonFault = "fault";
        public const string ReasonWorkspaceLimit = "workspace_limit";   // [v3 신규] 팔 목표가 작업 공간 상자 밖
        public const string ReasonIkUnreachable = "ik_unreachable";     // [v3 신규] 팔 목표에 IK 해 없음

        public static readonly string[] States =
        {
            StateAwaitingNeutral, StateIdle, StateActive, StateTimeout, StateEmergencyStop, StateFault
        };

        public static readonly string[] Reasons =
        {
            ReasonOk, ReasonInvalidPacket, ReasonUnsupportedProtocol, ReasonOutOfOrder, ReasonNotOwner,
            ReasonEmergencyStop, ReasonNeutralRequired, ReasonFault, ReasonWorkspaceLimit, ReasonIkUnreachable
        };

        // 이벤트 type
        public const string EventEmergencyStop = "emergency_stop";
        public const string EventClearEmergency = "clear_emergency";
        public const string EventDisconnect = "disconnect";

        internal static bool IsFinite(double v)
        {
            return !double.IsNaN(v) && !double.IsInfinity(v);
        }

        internal static double Clamp(double v, double lo, double hi)
        {
            if (v < lo) return lo;
            if (v > hi) return hi;
            return v;
        }

        internal static bool Contains(string[] set, string value)
        {
            for (int i = 0; i < set.Length; i++)
            {
                if (string.Equals(set[i], value, StringComparison.Ordinal)) return true;
            }
            return false;
        }
    }

    // ───────────────────────── 2. 보낼 JSON 만들기 ─────────────────────────

    /// <summary>
    /// 로봇 파서는 엄격하다: 필드 수·이름이 정확해야 하고, protocol·sequence 는 정수(소수점 금지),
    /// 숫자 자리에 NaN·Infinity·문자열·불리언 금지. Unity JsonUtility 대신 이 함수로 만들면 순서·형식이 고정된다.
    /// </summary>
    public static class ArmPiV3Json
    {
        /// <summary>차체 command (정확히 8필드). controlActive=false 이면 세 축을 0 으로 보낸다(차체 중립).</summary>
        public static string Command(string clientId, long sequence, bool controlActive,
                                     double forward, double strafe, double turn)
        {
            CheckIdentity(clientId, sequence);
            double f = controlActive ? Axis(forward) : 0.0;
            double s = controlActive ? Axis(strafe) : 0.0;
            double t = controlActive ? Axis(turn) : 0.0;
            var sb = new StringBuilder(192);
            sb.Append("{\"protocol\":3,\"type\":\"command\",\"client_id\":\"").Append(clientId)
              .Append("\",\"sequence\":").Append(sequence.ToString(CultureInfo.InvariantCulture))
              .Append(",\"control_active\":").Append(controlActive ? "true" : "false")
              .Append(",\"forward\":").Append(Num(f))
              .Append(",\"strafe\":").Append(Num(s))
              .Append(",\"turn\":").Append(Num(t))
              .Append('}');
            return sb.ToString();
        }

        /// <summary>
        /// 팔 arm_command (정확히 10필드). x,y,z: armpi_arm_base(REP-103) 손끝 절대 목표[m],
        /// pitch: rad(0 = 수평 앞, −π/2 = 아래), gripper: 0 열림 … 1 닫힘.
        /// controlActive=false(팔 중립)여도 숫자 5개는 유효한 값이어야 한다(로봇은 무시). 마지막 수락값을 넣는 것을 권장.
        /// </summary>
        public static string ArmCommand(string clientId, long sequence, bool controlActive,
                                        double x, double y, double z, double pitch, double gripper)
        {
            CheckIdentity(clientId, sequence);
            if (!ArmPiV3.IsFinite(x) || !ArmPiV3.IsFinite(y) || !ArmPiV3.IsFinite(z) ||
                !ArmPiV3.IsFinite(pitch) || !ArmPiV3.IsFinite(gripper))
            {
                throw new ArgumentException("arm_command 값은 모두 유한값이어야 한다");
            }
            double p = ArmPiV3.Clamp(pitch, -Math.PI, Math.PI);
            double g = ArmPiV3.Clamp(gripper, 0.0, 1.0);
            var sb = new StringBuilder(256);
            sb.Append("{\"protocol\":3,\"type\":\"arm_command\",\"client_id\":\"").Append(clientId)
              .Append("\",\"sequence\":").Append(sequence.ToString(CultureInfo.InvariantCulture))
              .Append(",\"control_active\":").Append(controlActive ? "true" : "false")
              .Append(",\"x\":").Append(Num(x))
              .Append(",\"y\":").Append(Num(y))
              .Append(",\"z\":").Append(Num(z))
              .Append(",\"pitch\":").Append(Num(p))
              .Append(",\"gripper\":").Append(Num(g))
              .Append('}');
            return sb.ToString();
        }

        /// <summary>이벤트(정확히 4필드): emergency_stop / clear_emergency / disconnect.</summary>
        public static string Event(string type, string clientId, long sequence)
        {
            if (type != ArmPiV3.EventEmergencyStop && type != ArmPiV3.EventClearEmergency &&
                type != ArmPiV3.EventDisconnect)
            {
                throw new ArgumentException("알 수 없는 이벤트 type: " + type);
            }
            CheckIdentity(clientId, sequence);
            return "{\"protocol\":3,\"type\":\"" + type + "\",\"client_id\":\"" + clientId +
                   "\",\"sequence\":" + sequence.ToString(CultureInfo.InvariantCulture) + "}";
        }

        /// <summary>새 세션 UUID. 하이픈 포함 36자 소문자(Guid "D" 형식). nil UUID 는 나오지 않는다.</summary>
        public static string NewClientId()
        {
            return Guid.NewGuid().ToString("D");
        }

        /// <summary>
        /// JSON 숫자. 소수 6자리로 반올림(0.001 mm)하고, 정수처럼 보이면 ".0" 을 붙인다.
        /// 지수 표기(예: 1E-05)는 로봇 파서가 받는다.
        /// </summary>
        public static string Num(double v)
        {
            if (!ArmPiV3.IsFinite(v)) throw new ArgumentException("JSON 숫자에 NaN/Infinity 불가");
            double r = Math.Round(v, 6, MidpointRounding.AwayFromZero);
            if (r == 0.0) r = 0.0;  // −0 을 0 으로
            string s = r.ToString("R", CultureInfo.InvariantCulture);
            if (s.IndexOf('.') < 0 && s.IndexOf('E') < 0 && s.IndexOf('e') < 0) s += ".0";
            return s;
        }

        static double Axis(double v)
        {
            if (!ArmPiV3.IsFinite(v)) return 0.0;
            return ArmPiV3.Clamp(v, -1.0, 1.0);
        }

        static void CheckIdentity(string clientId, long sequence)
        {
            if (!IsUuid(clientId)) throw new ArgumentException("client_id 는 하이픈 포함 36자 UUID 여야 한다");
            if (sequence < 0 || sequence > ArmPiV3.MaxSequence) throw new ArgumentOutOfRangeException("sequence");
        }

        /// <summary>8-4-4-4-12 hex, nil UUID 아님.</summary>
        public static bool IsUuid(string s)
        {
            if (s == null || s.Length != 36) return false;
            bool allZero = true;
            for (int i = 0; i < 36; i++)
            {
                char c = s[i];
                bool dash = i == 8 || i == 13 || i == 18 || i == 23;
                if (dash)
                {
                    if (c != '-') return false;
                    continue;
                }
                bool hex = (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F');
                if (!hex) return false;
                if (c != '0') allZero = false;
            }
            return !allZero;
        }
    }

    // ───────────────────────── 3. ACK 해석 ─────────────────────────

    public sealed class ArmPiAck
    {
        public string ClientId;      // 식별 불가 패킷의 ACK 이면 null
        public long Sequence;        // 식별 불가 패킷의 ACK 이면 −1
        public bool Accepted;
        public string Reason;
        public bool EmergencyStop;
        public string State;         // 그 패킷이 속한 채널(command·이벤트 → 차체, arm_command → 팔)의 상태

        public override string ToString()
        {
            return "ack(seq=" + Sequence + ", accepted=" + Accepted + ", reason=" + Reason + ", estop=" +
                   EmergencyStop + ", state=" + State + ")";
        }
    }

    /// <summary>
    /// ACK 엄격 해석. 기존 Unity V2AckParser 와 같은 정신: 정확히 8필드, 값은 문자열·불리언·정수·null 만,
    /// 실수(소수점·지수) 금지, 중복 키 금지, 문자열은 [A-Za-z0-9_-] 만, protocol = 3.
    /// 로봇 ACK 에는 escape 가 나오지 않으므로 역슬래시가 있으면 거부한다.
    /// </summary>
    public static class ArmPiV3Ack
    {
        static readonly string[] Keys =
        {
            "protocol", "type", "client_id", "sequence", "accepted", "reason", "emergency_stop", "state"
        };

        static readonly UTF8Encoding StrictUtf8 = new UTF8Encoding(false, true);

        public static bool TryParse(byte[] data, int length, out ArmPiAck ack, out string error)
        {
            ack = null;
            if (data == null || length <= 0 || length > data.Length || length > ArmPiV3.MaxPacketBytes)
            {
                error = "길이 오류";
                return false;
            }
            string text;
            try
            {
                text = StrictUtf8.GetString(data, 0, length);
            }
            catch (ArgumentException)
            {
                error = "UTF-8 오류";
                return false;
            }
            return TryParse(text, out ack, out error);
        }

        public static bool TryParse(string text, out ArmPiAck ack, out string error)
        {
            ack = null;
            Dictionary<string, object> obj;
            if (!FlatJson.TryParseObject(text, out obj, out error)) return false;
            if (obj.Count != Keys.Length)
            {
                error = "필드 수 " + obj.Count + " (8 이어야 함)";
                return false;
            }
            for (int i = 0; i < Keys.Length; i++)
            {
                if (!obj.ContainsKey(Keys[i]))
                {
                    error = "필드 없음: " + Keys[i];
                    return false;
                }
            }
            object v;
            v = obj["protocol"];
            if (!(v is long) || (long)v != ArmPiV3.Protocol) { error = "protocol 이 정수 3 이 아님"; return false; }
            v = obj["type"];
            if (!(v is string) || (string)v != "ack") { error = "type 이 ack 아님"; return false; }
            v = obj["client_id"];
            if (v != null && !(v is string)) { error = "client_id 형식"; return false; }
            string cid = (string)v;
            v = obj["sequence"];
            if (!(v is long)) { error = "sequence 가 정수 아님"; return false; }
            long seq = (long)v;
            if (seq < -1 || seq > ArmPiV3.MaxSequence) { error = "sequence 범위"; return false; }
            v = obj["accepted"];
            if (!(v is bool)) { error = "accepted 가 불리언 아님"; return false; }
            bool accepted = (bool)v;
            v = obj["emergency_stop"];
            if (!(v is bool)) { error = "emergency_stop 이 불리언 아님"; return false; }
            bool estop = (bool)v;
            v = obj["reason"];
            if (!(v is string) || !ArmPiV3.Contains(ArmPiV3.Reasons, (string)v)) { error = "reason 값"; return false; }
            string reason = (string)v;
            v = obj["state"];
            if (!(v is string) || !ArmPiV3.Contains(ArmPiV3.States, (string)v)) { error = "state 값"; return false; }
            string state = (string)v;

            ack = new ArmPiAck
            {
                ClientId = cid,
                Sequence = seq,
                Accepted = accepted,
                Reason = reason,
                EmergencyStop = estop,
                State = state
            };
            error = null;
            return true;
        }

        /// <summary>평평한 JSON 객체 1개(중첩 없음)만 읽는 최소 파서.</summary>
        static class FlatJson
        {
            public static bool TryParseObject(string s, out Dictionary<string, object> obj, out string error)
            {
                obj = new Dictionary<string, object>(StringComparer.Ordinal);
                int i = 0;
                SkipWs(s, ref i);
                if (i >= s.Length || s[i] != '{') { error = "객체가 아님"; return false; }
                i++;
                SkipWs(s, ref i);
                if (i < s.Length && s[i] == '}')
                {
                    i++;
                    return End(s, i, out error);
                }
                while (true)
                {
                    string key;
                    if (!ReadString(s, ref i, out key, out error)) return false;
                    if (obj.ContainsKey(key)) { error = "중복 키 " + key; return false; }
                    SkipWs(s, ref i);
                    if (i >= s.Length || s[i] != ':') { error = "':' 없음"; return false; }
                    i++;
                    SkipWs(s, ref i);
                    object value;
                    if (!ReadValue(s, ref i, out value, out error)) return false;
                    obj[key] = value;
                    SkipWs(s, ref i);
                    if (i >= s.Length) { error = "객체가 끝나지 않음"; return false; }
                    if (s[i] == ',') { i++; SkipWs(s, ref i); continue; }
                    if (s[i] == '}') { i++; return End(s, i, out error); }
                    error = "',' 또는 '}' 없음";
                    return false;
                }
            }

            static bool End(string s, int i, out string error)
            {
                SkipWs(s, ref i);
                if (i != s.Length) { error = "객체 뒤에 남은 글자"; return false; }
                error = null;
                return true;
            }

            static void SkipWs(string s, ref int i)
            {
                while (i < s.Length && (s[i] == ' ' || s[i] == '\t' || s[i] == '\n' || s[i] == '\r')) i++;
            }

            static bool ReadString(string s, ref int i, out string value, out string error)
            {
                value = null;
                if (i >= s.Length || s[i] != '"') { error = "문자열 아님"; return false; }
                int start = ++i;
                while (i < s.Length && s[i] != '"')
                {
                    char c = s[i];
                    if (c == '\\') { error = "escape 미지원(ACK 에는 나오지 않음)"; return false; }
                    bool ok = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') ||
                              c == '_' || c == '-';
                    if (!ok) { error = "허용되지 않은 문자"; return false; }
                    i++;
                }
                if (i >= s.Length) { error = "문자열이 끝나지 않음"; return false; }
                value = s.Substring(start, i - start);
                i++;
                error = null;
                return true;
            }

            static bool ReadValue(string s, ref int i, out object value, out string error)
            {
                value = null;
                if (i >= s.Length) { error = "값 없음"; return false; }
                char c = s[i];
                if (c == '"')
                {
                    string str;
                    if (!ReadString(s, ref i, out str, out error)) return false;
                    value = str;
                    return true;
                }
                if (Lit(s, ref i, "true")) { value = true; error = null; return true; }
                if (Lit(s, ref i, "false")) { value = false; error = null; return true; }
                if (Lit(s, ref i, "null")) { value = null; error = null; return true; }
                if (c == '-' || (c >= '0' && c <= '9'))
                {
                    int start = i;
                    if (s[i] == '-') i++;
                    if (i >= s.Length || s[i] < '0' || s[i] > '9') { error = "숫자 형식"; return false; }
                    if (s[i] == '0' && i + 1 < s.Length && s[i + 1] >= '0' && s[i + 1] <= '9')
                    {
                        error = "앞자리 0";
                        return false;
                    }
                    while (i < s.Length && s[i] >= '0' && s[i] <= '9') i++;
                    if (i < s.Length && (s[i] == '.' || s[i] == 'e' || s[i] == 'E'))
                    {
                        error = "ACK 에 실수 값";
                        return false;
                    }
                    long n;
                    if (!long.TryParse(s.Substring(start, i - start), NumberStyles.AllowLeadingSign,
                                       CultureInfo.InvariantCulture, out n))
                    {
                        error = "정수 범위 초과";
                        return false;
                    }
                    value = n;
                    error = null;
                    return true;
                }
                error = "지원하지 않는 값(객체·배열 등)";
                return false;
            }

            static bool Lit(string s, ref int i, string lit)
            {
                if (string.CompareOrdinal(s, i, lit, 0, lit.Length) != 0) return false;
                int end = i + lit.Length;
                if (end < s.Length && char.IsLetterOrDigit(s[end])) return false;
                i = end;
                return true;
            }
        }
    }

    // ───────────────────────── 4. 검색 ─────────────────────────

    public sealed class ArmPiRobotInfo
    {
        public string DeviceId;   // 32 hex
        public string Name;       // URL 디코딩한 이름(보통 hostname, 예: armpi-ros2)
        public int ControlPort;   // 보통 5001
        public string Address;    // 응답을 보낸 IPv4(호출자가 채움)
    }

    public static class ArmPiV3Discovery
    {
        /// <summary>요청 nonce: 32자 소문자 hex.</summary>
        public static string NewNonce()
        {
            return Guid.NewGuid().ToString("N");
        }

        /// <summary>UDP 5002 로 보낼 요청(브로드캐스트 255.255.255.255 또는 로봇 IP).</summary>
        public static string Request(string nonce)
        {
            return "ARMPI_DISCOVER/1|" + nonce;
        }

        /// <summary>
        /// 응답 해석: ARMPI_HERE/1|nonce|32hex id|URL 인코딩 이름|제어 포트|3.
        /// 호출자는 응답 출발 포트가 5002 인지도 확인해야 한다(사양 1절).
        /// </summary>
        public static bool TryParseReply(string text, string nonce, out ArmPiRobotInfo info)
        {
            info = null;
            if (text == null || text.Length < 80 || text.Length > 640) return false;
            for (int i = 0; i < text.Length; i++)
            {
                if (text[i] < 32 || text[i] > 126) return false;
            }
            string[] f = text.Split('|');
            if (f.Length != 6 || f[0] != "ARMPI_HERE/1" || f[1] != nonce || f[5] != "3") return false;
            if (!IsHex32(f[2])) return false;
            int port;
            if (!int.TryParse(f[4], NumberStyles.None, CultureInfo.InvariantCulture, out port) || port < 1 ||
                port > 65535)
            {
                return false;
            }
            string name;
            try
            {
                name = Uri.UnescapeDataString(f[3]);
            }
            catch (Exception)
            {
                return false;
            }
            if (name.Length == 0) return false;
            info = new ArmPiRobotInfo { DeviceId = f[2], Name = name, ControlPort = port };
            return true;
        }

        static bool IsHex32(string s)
        {
            if (s.Length != 32) return false;
            for (int i = 0; i < s.Length; i++)
            {
                char c = s[i];
                if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'))) return false;
            }
            return true;
        }
    }

    // ───────────────────────── 5. 좌표 변환 ─────────────────────────

    public struct Vec3
    {
        public double X, Y, Z;

        public Vec3(double x, double y, double z)
        {
            X = x;
            Y = y;
            Z = z;
        }

        public static Vec3 operator +(Vec3 a, Vec3 b) { return new Vec3(a.X + b.X, a.Y + b.Y, a.Z + b.Z); }
        public static Vec3 operator -(Vec3 a, Vec3 b) { return new Vec3(a.X - b.X, a.Y - b.Y, a.Z - b.Z); }
        public static Vec3 operator *(Vec3 a, double k) { return new Vec3(a.X * k, a.Y * k, a.Z * k); }
        public double Length { get { return Math.Sqrt(X * X + Y * Y + Z * Z); } }

        public override string ToString()
        {
            return string.Format(CultureInfo.InvariantCulture, "({0:0.####}, {1:0.####}, {2:0.####})", X, Y, Z);
        }
    }

    /// <summary>쿼터니언 (x, y, z, w). Unity Quaternion 과 같은 성분 순서.</summary>
    public struct Quat
    {
        public double X, Y, Z, W;

        public Quat(double x, double y, double z, double w)
        {
            X = x;
            Y = y;
            Z = z;
            W = w;
        }

        public static Quat Identity { get { return new Quat(0, 0, 0, 1); } }

        public static Quat AxisAngle(Vec3 axis, double angleRad)
        {
            double n = axis.Length;
            if (n < 1e-12) return Identity;
            double s = Math.Sin(angleRad / 2) / n;
            return new Quat(axis.X * s, axis.Y * s, axis.Z * s, Math.Cos(angleRad / 2));
        }

        /// <summary>벡터 회전 v' = q v q*. (Unity 의 q * v 와 같은 결과)</summary>
        public Vec3 Rotate(Vec3 v)
        {
            // t = 2 (q.xyz × v);  v' = v + w t + q.xyz × t
            double tx = 2 * (Y * v.Z - Z * v.Y);
            double ty = 2 * (Z * v.X - X * v.Z);
            double tz = 2 * (X * v.Y - Y * v.X);
            return new Vec3(v.X + W * tx + (Y * tz - Z * ty),
                            v.Y + W * ty + (Z * tx - X * tz),
                            v.Z + W * tz + (X * ty - Y * tx));
        }
    }

    /// <summary>
    /// Unity: 왼손 좌표계, +X 오른쪽, +Y 위, +Z 앞.  ROS REP-103: 오른손 좌표계, +x 앞, +y 왼쪽, +z 위.
    ///   위치:   ros = ( u.z, −u.x,  u.y )      unity = ( −r.y, r.z, r.x )
    ///   회전:   q_ros = ( −q.z, q.x, −q.y, q.w )   (손잡이 방향이 바뀌므로 벡터부 부호가 바뀐다)
    /// 단위는 둘 다 m·rad 이다(Unity 1 unit = 1 m 로 가정).
    /// </summary>
    public static class ArmPiFrames
    {
        public static Vec3 RosFromUnity(Vec3 u) { return new Vec3(u.Z, -u.X, u.Y); }
        public static Vec3 UnityFromRos(Vec3 r) { return new Vec3(-r.Y, r.Z, r.X); }
        public static Quat RosFromUnity(Quat q) { return new Quat(-q.Z, q.X, -q.Y, q.W); }
        public static Quat UnityFromRos(Quat q) { return new Quat(q.Y, -q.Z, -q.X, q.W); }

        /// <summary>ROS 좌표의 방향 벡터 → pitch(rad). 위가 +, 수평 0, 아래 −π/2.</summary>
        public static double PitchOf(Vec3 rosDirection)
        {
            return Math.Atan2(rosDirection.Z, Math.Sqrt(rosDirection.X * rosDirection.X +
                                                        rosDirection.Y * rosDirection.Y));
        }

        /// <summary>Unity 회전(컨트롤러) → 그 컨트롤러 앞(+Z)이 가리키는 방향의 ROS pitch.</summary>
        public static double ControllerPitch(Quat unityRotation)
        {
            Vec3 fwdUnity = unityRotation.Rotate(new Vec3(0, 0, 1));
            return PitchOf(RosFromUnity(fwdUnity));
        }
    }

    // ───────────────────────── 6. 기구학 (로봇과 같은 식) ─────────────────────────

    public enum IkStatus { Ok, OutOfReach, JointLimit, Invalid }

    /// <summary>
    /// ros2_ws/src/armpi_kinematics 와 같은 식(팔꿈치 위 해). 링크 길이 기본값은 armpi_bringup/config/armpi.yaml 의
    /// arm.kinematics.*(Hiwonder 공개 ArmPi FPV 값, ArmPi Pro 실측 확인 필요). 관절 기준(JointSign·JointOffset)과
    /// 관절 범위는 로봇의 하드웨어 프로필(hardware_profile.yaml)과 같은 값으로 채운다. 아래 기본값은 시험용
    /// (test_hardware_profile.yaml 과 같음, 실제 로봇 값 아님). 최종 판정은 항상 로봇 ACK 다.
    /// </summary>
    public sealed class ArmPiKinematics
    {
        public double BaseHeight = 0.064605;   // armpi_arm_base 원점 → 어깨 축 높이 [m]
        public double UpperArm = 0.10048;      // 어깨 → 팔꿈치
        public double Forearm = 0.094714;      // 팔꿈치 → 손목
        public double Hand = 0.16331;          // 손목 → 그리퍼 끝점
        public double MinWristHeight = 0.0;    // 손목 관절축이 z=0 아래면 거부
        public double MinElbowBend = 0.05;     // 거의 일자로 편 자세 거부 [rad]
        // joint_i = JointSign[i] · 기하 각도 + JointOffset[i] (rad). 시험용 기본값 — 로봇 프로필 값으로 바꾼다
        public double[] JointSign = { 1.0, 1.0, -1.0, 1.0 };
        public double[] JointOffset = { 0.0, -1.4, 0.0, 0.1 };
        // 관절 범위(rad). joint1·joint3 의 ±120° 는 공개 값, joint2·joint4 는 시험용
        public double[] JointMin = { -120 * Deg, -1.4, -120 * Deg, -1.9 };
        public double[] JointMax = { 120 * Deg, 1.4, 120 * Deg, 1.9 };

        public const double Deg = Math.PI / 180.0;

        /// <summary>joints[0..3] = joint1(베이스) … joint4(손목 pitch), rad.</summary>
        public IkStatus Inverse(double x, double y, double z, double pitch, double[] joints)
        {
            if (!ArmPiV3.IsFinite(x) || !ArmPiV3.IsFinite(y) || !ArmPiV3.IsFinite(z) || !ArmPiV3.IsFinite(pitch))
            {
                return IkStatus.Invalid;
            }
            double r = Math.Sqrt(x * x + y * y);
            double yaw = r < 1e-9 ? 0.0 : Math.Atan2(y, x);
            double rw = r - Hand * Math.Cos(pitch);
            double zw = (z - BaseHeight) - Hand * Math.Sin(pitch);
            if (zw + BaseHeight < MinWristHeight - 1e-12) return IkStatus.OutOfReach;
            double l1 = UpperArm, l2 = Forearm;
            double c3 = (rw * rw + zw * zw - l1 * l1 - l2 * l2) / (2.0 * l1 * l2);
            if (c3 > 1.0 + 1e-9 || c3 < -1.0 - 1e-9) return IkStatus.OutOfReach;
            c3 = ArmPiV3.Clamp(c3, -1.0, 1.0);
            double q3 = -Math.Acos(c3);  // elbow-up
            if (-q3 < MinElbowBend) return IkStatus.OutOfReach;
            double q2 = Math.Atan2(zw, rw) - Math.Atan2(l2 * Math.Sin(q3), l1 + l2 * Math.Cos(q3));
            double q4 = pitch - q2 - q3;
            double[] q = { yaw, q2, q3, q4 };
            double[] j = new double[4];
            for (int i = 0; i < 4; i++) j[i] = JointSign[i] * q[i] + JointOffset[i];
            if (joints != null)
            {
                for (int i = 0; i < 4 && i < joints.Length; i++) joints[i] = j[i];
            }
            for (int i = 0; i < 4; i++)
            {
                if (!ArmPiV3.IsFinite(j[i]) || j[i] < JointMin[i] - 1e-9 || j[i] > JointMax[i] + 1e-9)
                {
                    return IkStatus.JointLimit;
                }
            }
            return IkStatus.Ok;
        }

        /// <summary>joint1..joint4(rad) → 손끝 (x, y, z), pitch.</summary>
        public Vec3 Forward(double[] joints, out double pitch)
        {
            double yaw = (joints[0] - JointOffset[0]) / JointSign[0];
            double a2 = (joints[1] - JointOffset[1]) / JointSign[1];
            double a3 = a2 + (joints[2] - JointOffset[2]) / JointSign[2];
            double a4 = a3 + (joints[3] - JointOffset[3]) / JointSign[3];
            double r = UpperArm * Math.Cos(a2) + Forearm * Math.Cos(a3) + Hand * Math.Cos(a4);
            double z = BaseHeight + UpperArm * Math.Sin(a2) + Forearm * Math.Sin(a3) + Hand * Math.Sin(a4);
            pitch = a4;
            return new Vec3(r * Math.Cos(yaw), r * Math.Sin(yaw), z);
        }
    }

    /// <summary>
    /// 로봇 armpi_kinematics::TargetProjector(arm.target_policy: nearest)와 같은 규칙(2026-11-11).
    /// 닿을 수 없는 목표 → ① 상자 면으로 자름 ② 요청에 가장 가까운 pitch(± MaxPitchChange, [PitchMin, PitchMax] 안,
    /// 같은 거리면 anchor pitch 쪽 먼저) ③ anchor → 목표 직선에서 닿는 가장 먼 점. 결과는 항상 상자 안·IK 해 있음.
    /// 로봇이 같은 보정을 하므로 VR 쪽에서 꼭 쓸 필요는 없다. "닿는 곳까지만 간다"를 화면에 보여 줄 때 쓴다.
    /// Kinematics 의 관절 기준·범위를 로봇 프로필 값으로 바꾸지 않으면 로봇 판정과 다를 수 있다.
    /// </summary>
    public sealed class ArmPiTargetProjector
    {
        public const int AdjustBox = 1, AdjustPitch = 2, AdjustPosition = 4;

        public ArmPiKinematics Kinematics;
        public Workspace Box;
        public double MaxPitchChange = Math.PI / 2;   // arm.nearest.max_pitch_change_rad
        public double PitchStep = Math.PI / 180.0;    // arm.nearest.pitch_step_rad
        public double PitchMin = -Math.PI / 2;        // arm.nearest.pitch_min_rad
        public double PitchMax = Math.PI / 6;         // arm.nearest.pitch_max_rad
        public bool AllowPositionChange = true;       // arm.nearest.allow_position_change
        public int BisectIterations = 14;             // arm.nearest.bisect_iterations

        public ArmPiTargetProjector(ArmPiKinematics kinematics, Workspace box)
        {
            Kinematics = kinematics;
            Box = box;
        }

        bool IkOk(double x, double y, double z, double pitch)
        {
            return Kinematics.Inverse(x, y, z, pitch, null) == IkStatus.Ok;
        }

        public bool Reachable(ArmTarget t)
        {
            return ArmPiV3.IsFinite(t.X) && ArmPiV3.IsFinite(t.Y) && ArmPiV3.IsFinite(t.Z) && ArmPiV3.IsFinite(t.Pitch) &&
                   Box.Contains(t.X, t.Y, t.Z) && IkOk(t.X, t.Y, t.Z, t.Pitch);
        }

        bool FindPitch(double x, double y, double z, double want, double? prefer, double? extra, out double result)
        {
            result = want;
            if (IkOk(x, y, z, want)) return true;
            if (MaxPitchChange > 0)
            {
                int n = (int)Math.Floor(MaxPitchChange / PitchStep + 1e-9);
                for (int i = 1; i <= n + 1; i++)
                {
                    double d = i <= n ? i * PitchStep : MaxPitchChange;
                    if (i == n + 1 && Math.Abs(d - n * PitchStep) < 1e-9) break;
                    double first = want - d, second = want + d;
                    if (prefer.HasValue && Math.Abs(second - prefer.Value) < Math.Abs(first - prefer.Value))
                    {
                        double tmp = first; first = second; second = tmp;
                    }
                    foreach (double c in new[] { first, second })
                    {
                        if (c >= PitchMin - 1e-9 && c <= PitchMax + 1e-9 && IkOk(x, y, z, c))
                        {
                            result = c;
                            return true;
                        }
                    }
                }
            }
            if (extra.HasValue && ArmPiV3.IsFinite(extra.Value) && IkOk(x, y, z, extra.Value))
            {
                result = extra.Value;
                return true;
            }
            return false;
        }

        /// <summary>
        /// ok=false 면 보정할 수 없음(anchor 없음 등). adjust 는 AdjustBox|AdjustPitch|AdjustPosition 비트. 그리퍼는 그대로 둔다.
        /// anchor: 닿는다고 알려진 자세(마지막으로 보낸 목표 등). 없으면 null.
        /// </summary>
        public bool Project(ArmTarget target, ArmTarget? anchor, out ArmTarget result, out int adjust)
        {
            result = target;
            adjust = 0;
            if (!ArmPiV3.IsFinite(target.X) || !ArmPiV3.IsFinite(target.Y) || !ArmPiV3.IsFinite(target.Z) ||
                !ArmPiV3.IsFinite(target.Pitch)) return false;
            Vec3 c = Box.Clamp(new Vec3(target.X, target.Y, target.Z));
            if (c.X != target.X || c.Y != target.Y || c.Z != target.Z) adjust |= AdjustBox;
            if (IkOk(c.X, c.Y, c.Z, target.Pitch))
            {
                result = new ArmTarget(c.X, c.Y, c.Z, target.Pitch, target.Gripper);
                return true;
            }
            double? prefer = anchor.HasValue ? anchor.Value.Pitch : (double?)null;
            double pc;
            if (FindPitch(c.X, c.Y, c.Z, target.Pitch, prefer, null, out pc))
            {
                result = new ArmTarget(c.X, c.Y, c.Z, pc, target.Gripper);
                adjust |= AdjustPitch;
                return true;
            }
            if (!AllowPositionChange || !anchor.HasValue || !Reachable(anchor.Value)) return false;
            ArmTarget a = anchor.Value;
            ArmTarget best = new ArmTarget(a.X, a.Y, a.Z, a.Pitch, target.Gripper);
            double lo = 0, hi = 1;
            for (int it = 0; it < BisectIterations; it++)
            {
                double s = 0.5 * (lo + hi);
                double x = a.X + s * (c.X - a.X), y = a.Y + s * (c.Y - a.Y), z = a.Z + s * (c.Z - a.Z);
                if (FindPitch(x, y, z, target.Pitch, prefer, a.Pitch, out pc))
                {
                    lo = s;
                    best = new ArmTarget(x, y, z, pc, target.Gripper);
                }
                else
                {
                    hi = s;
                }
            }
            result = best;
            adjust |= AdjustPosition;
            if (best.Pitch != target.Pitch) adjust |= AdjustPitch;
            return true;
        }
    }

    // ───────────────────────── 7. 컨트롤러 → 팔 목표 (제안) ─────────────────────────

    public struct ArmTarget
    {
        public double X, Y, Z, Pitch, Gripper;

        public ArmTarget(double x, double y, double z, double pitch, double gripper)
        {
            X = x;
            Y = y;
            Z = z;
            Pitch = pitch;
            Gripper = gripper;
        }

        /// <summary>기본 자세 손끝 목표(로봇 armpi.yaml·웹 조종기 home 과 같은 값).</summary>
        public static ArmTarget Home { get { return new ArmTarget(0.1047, 0.0, 0.0753, -1.0263, 0.0); } }

        public override string ToString()
        {
            return string.Format(CultureInfo.InvariantCulture,
                                 "x={0:0.0000} y={1:0.0000} z={2:0.0000} pitch={3:0.0000} g={4:0.00}",
                                 X, Y, Z, Pitch, Gripper);
        }
    }

    /// <summary>작업 공간 상자(armpi_arm_base, m). 로봇 설정 arm.workspace.* 와 같은 값을 넣는다.</summary>
    public struct Workspace
    {
        public double XMin, XMax, YMin, YMax, ZMin, ZMax;

        public Workspace(double xMin, double xMax, double yMin, double yMax, double zMin, double zMax)
        {
            XMin = xMin;
            XMax = xMax;
            YMin = yMin;
            YMax = yMax;
            ZMin = zMin;
            ZMax = zMax;
        }

        /// <summary>
        /// 로봇 공통 설정 armpi_bringup/config/armpi.yaml 의 arm.workspace.* (2026-11-02 22:25 판).
        /// 로봇에서 이 값이 바뀌면 여기도 같이 바꾼다. 가짜 로봇은 기본값이 다르므로
        /// mock_robot.py --workspace 0.05,0.32,-0.25,0.25,-0.08,0.32 로 맞춰 띄운다.
        /// </summary>
        public static Workspace Robot { get { return new Workspace(0.05, 0.32, -0.25, 0.25, -0.08, 0.32); } }

        /// <summary>
        /// Robot ∩ 가짜 로봇 기본값(0–0.30, ±0.25, −0.05–0.35). 가짜 로봇을 기본값으로 띄워 시험할 때 거부를 피하려면 이것을 쓴다.
        /// </summary>
        public static Workspace Conservative { get { return new Workspace(0.05, 0.30, -0.25, 0.25, -0.05, 0.32); } }

        public bool Contains(double x, double y, double z)
        {
            return x >= XMin && x <= XMax && y >= YMin && y <= YMax && z >= ZMin && z <= ZMax;
        }

        public Vec3 Clamp(Vec3 p)
        {
            return new Vec3(ArmPiV3.Clamp(p.X, XMin, XMax), ArmPiV3.Clamp(p.Y, YMin, YMax),
                            ArmPiV3.Clamp(p.Z, ZMin, ZMax));
        }
    }

    /// <summary>
    /// 제안: 상대 이동(clutch) 방식.
    ///   - 그립을 누르는 순간(Engage) 컨트롤러 자세와 로봇 손끝 현재값을 기준으로 잡는다.
    ///   - 누르는 동안(Update) "컨트롤러가 움직인 만큼 × Scale" 을 손끝 목표에 더한다.
    ///   - 결과를 작업 공간 상자로 자르고, 속도 상한으로 한 걸음씩만 움직인다.
    ///   - 닿지 않는 목표: 기본(Projector·Kinematics 둘 다 null)은 그대로 보낸다. 로봇(arm.target_policy: nearest, 2026-11-11 기본)이
    ///     닿는 가장 가까운 자세로 받는다. Projector 를 넣으면 같은 보정을 VR 쪽에서 미리 해서 Output 이 로봇 목표와 같아진다.
    ///     Kinematics 만 넣으면 예전 동작(IK 가 안 풀리면 직전 목표 유지 — pitch 를 고정한 채 막혀 "자꾸 멈춤"의 원인이 된다).
    ///   - 놓으면(Release) 목표를 그대로 둔다(로봇은 control_active:false 를 받으면 현재 위치를 유지).
    /// 입력 자세는 "조작 기준 좌표계"(Unity 에서 로봇 앞 방향을 +Z 로 맞춘 앵커의 로컬 좌표)로 넘긴다.
    ///   예) handPos = anchor.InverseTransformPoint(controller.position)
    ///       handRot = Quaternion.Inverse(anchor.rotation) * controller.rotation
    /// 손끝 yaw 는 목표 위치(atan2(y, x))로 정해지고, 손목 회전(joint5)은 프로토콜에 없다(로봇이 고정).
    /// </summary>
    public sealed class ArmTeleopMapper
    {
        public double Scale = 1.0;             // 손 1 m → 손끝 Scale m (작은 팔이므로 0.5 도 검토)
        public double MaxSpeed = 0.15;         // m/s. 로봇 보간 상한(arm.max_linear_speed_m_s)과 같게(사용자 결정 2026-11-04, 바퀴 방향 실기 시험 전)
        public double MaxPitchRate = 1.0;      // rad/s. 로봇 arm.max_pitch_rate_rad_s 와 같게
        public double PitchMin = -Math.PI / 2; // 사용할 pitch 범위(제안)
        public double PitchMax = 0.0;
        public bool UseControllerPitch = true; // false 면 pitch 를 잡을 때 값으로 고정
        public Workspace Box = Workspace.Robot;
        // 둘 다 null(기본): 미리 확인하지 않는다(로봇이 보정). Projector: 로봇과 같은 보정. Kinematics: IK 안 되면 직전 목표 유지(예전 기본).
        // 어느 쪽이든 관절 기준·범위를 로봇 프로필 값으로 바꾸지 않으면 로봇 판정과 다를 수 있다(ArmPiKinematics 기본값은 시험용).
        public ArmPiTargetProjector Projector = null;
        public ArmPiKinematics Kinematics = null;

        public bool Engaged { get; private set; }
        public bool LastIkBlocked { get; private set; }
        public bool LastAdjusted { get; private set; }  // Projector 가 목표를 바꿨는지(화면 표시·진동용)
        public ArmTarget Output { get { return _out; } }

        Vec3 _hand0;
        double _ctrlPitch0;
        ArmTarget _target0;
        ArmTarget _out;

        /// <summary>그립을 누른 순간. robotCurrent 는 로봇 /armpi/status 의 arm_current(없으면 마지막 수락 목표).</summary>
        public void Engage(Vec3 handPosUnity, Quat handRotUnity, ArmTarget robotCurrent)
        {
            _hand0 = handPosUnity;
            _ctrlPitch0 = ArmPiFrames.ControllerPitch(handRotUnity);
            _target0 = robotCurrent;
            _out = robotCurrent;
            Engaged = true;
            LastIkBlocked = false;
        }

        public void Release()
        {
            Engaged = false;
        }

        /// <summary>매 프레임(또는 50 ms 마다). gripper01: 트리거 0..1. dt: 지난 호출 이후 초.</summary>
        public ArmTarget Update(Vec3 handPosUnity, Quat handRotUnity, double gripper01, double dt)
        {
            if (!Engaged) return _out;
            if (!(dt > 0)) dt = 0;
            if (dt > 0.2) dt = 0.2;

            // 1) 원하는 손끝 위치(ROS) = 기준 + 손 이동량(Unity → ROS) × Scale, 작업 공간으로 자르기
            Vec3 deltaRos = ArmPiFrames.RosFromUnity(handPosUnity - _hand0) * Scale;
            Vec3 want = Box.Clamp(new Vec3(_target0.X, _target0.Y, _target0.Z) + deltaRos);

            // 2) 원하는 pitch
            double wantPitch = _target0.Pitch;
            if (UseControllerPitch)
            {
                wantPitch = _target0.Pitch + (ArmPiFrames.ControllerPitch(handRotUnity) - _ctrlPitch0);
            }
            wantPitch = ArmPiV3.Clamp(wantPitch, PitchMin, PitchMax);

            // 3) 속도 제한(직선 거리, pitch 변화율)
            Vec3 cur = new Vec3(_out.X, _out.Y, _out.Z);
            Vec3 d = want - cur;
            double maxStep = MaxSpeed * dt;
            Vec3 next = d.Length <= maxStep ? want : cur + d * (maxStep / d.Length);
            double maxP = MaxPitchRate * dt;
            double dp = wantPitch - _out.Pitch;
            double nextPitch = Math.Abs(dp) <= maxP ? wantPitch : _out.Pitch + Math.Sign(dp) * maxP;

            // 4) 닿지 않는 목표 처리(그리퍼는 항상 반영)
            double g = ArmPiV3.IsFinite(gripper01) ? ArmPiV3.Clamp(gripper01, 0.0, 1.0) : _out.Gripper;
            LastAdjusted = false;
            if (Projector != null)
            {
                ArmTarget adj;
                int how;
                if (Projector.Project(new ArmTarget(next.X, next.Y, next.Z, nextPitch, g), _out, out adj, out how))
                {
                    LastIkBlocked = false;
                    LastAdjusted = how != 0;
                    _out = adj;
                    return _out;
                }
                LastIkBlocked = true;
                _out.Gripper = g;
                return _out;
            }
            if (Kinematics != null && Kinematics.Inverse(next.X, next.Y, next.Z, nextPitch, null) != IkStatus.Ok)
            {
                LastIkBlocked = true;
                _out.Gripper = g;
                return _out;
            }
            LastIkBlocked = false;
            _out = new ArmTarget(next.X, next.Y, next.Z, nextPitch, g);
            return _out;
        }
    }
}
