// ArmPiV3Messages.cs 자체 시험.
//   1부(네트워크 없음): JSON 만들기, ACK 해석, 검색 응답 해석, 좌표 변환, 기구학, 컨트롤러 매핑
//   2부(--robot <IP>): 가짜 로봇(protocol_v3/mock_robot.py) 또는 C++ 수신기와 UDP 왕복
// 실행: dotnet run -- [--robot 127.0.0.1] [--port 5001] [--discovery-port 5002]
// 주의: 실제 로봇에 돌리면 차체·팔이 움직인다(전진 0.3, 팔 기본 자세 근처). 이 시험은 가짜 로봇용이다.

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.Net;
using System.Net.Sockets;
using System.Text;
using ArmPi.V3;

internal static class Program
{
    static int _pass, _fail;

    static void Check(bool ok, string name, string detail = "")
    {
        if (ok) _pass++; else _fail++;
        Console.WriteLine((ok ? "[PASS] " : "[FAIL] ") + name + (detail.Length > 0 ? "  " + detail : ""));
    }

    static bool Near(double a, double b, double tol) { return Math.Abs(a - b) <= tol; }

    static int Main(string[] args)
    {
        string robot = null;
        int port = ArmPiV3.ControlPort, dport = ArmPiV3.DiscoveryPort;
        for (int i = 0; i < args.Length; i++)
        {
            if (args[i] == "--robot" && i + 1 < args.Length) robot = args[++i];
            else if (args[i] == "--port" && i + 1 < args.Length) port = int.Parse(args[++i], CultureInfo.InvariantCulture);
            else if (args[i] == "--discovery-port" && i + 1 < args.Length) dport = int.Parse(args[++i], CultureInfo.InvariantCulture);
        }
        Console.WriteLine("ArmPiV3Messages 자체 시험 — " + DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss zzz", CultureInfo.InvariantCulture) +
                          ", .NET " + Environment.Version);
        Console.WriteLine("== 1부: 네트워크 없는 시험");
        OfflineTests();
        if (robot != null)
        {
            Console.WriteLine("== 2부: UDP 왕복 (" + robot + ":" + port + ", 검색 " + dport + ")");
            try
            {
                NetworkTests(robot, port, dport);
            }
            catch (Exception e)
            {
                Check(false, "네트워크 시험 예외", e.GetType().Name + ": " + e.Message);
            }
        }
        Console.WriteLine();
        Console.WriteLine("요약: 통과 " + _pass + " / 실패 " + _fail);
        return _fail == 0 ? 0 : 1;
    }

    // ───────────────────────── 1부 ─────────────────────────

    static void OfflineTests()
    {
        const string cid = "d8b27f10-09a8-4c21-b649-1e7058d46b72";

        // JSON 만들기
        string cmd = ArmPiV3Json.Command(cid, 123, true, 0.6, 0.0, -0.3);
        Check(cmd == "{\"protocol\":3,\"type\":\"command\",\"client_id\":\"" + cid +
                     "\",\"sequence\":123,\"control_active\":true,\"forward\":0.6,\"strafe\":0.0,\"turn\":-0.3}",
              "command 8필드·순서·숫자 형식", cmd);
        string neutral = ArmPiV3Json.Command(cid, 124, false, 0.9, 0.9, 0.9);
        Check(neutral.Contains("\"control_active\":false,\"forward\":0.0,\"strafe\":0.0,\"turn\":0.0"),
              "control_active:false 면 세 축 0", neutral);
        string clipped = ArmPiV3Json.Command(cid, 125, true, 2.0, double.NaN, -5.0);
        Check(clipped.Contains("\"forward\":1.0,\"strafe\":0.0,\"turn\":-1.0"), "축 값 −1..1 자르기, NaN → 0", clipped);
        string arm = ArmPiV3Json.ArmCommand(cid, 126, true, 0.1047, 0.0, 0.0753, -1.0263, 0.5);
        Check(arm == "{\"protocol\":3,\"type\":\"arm_command\",\"client_id\":\"" + cid +
                     "\",\"sequence\":126,\"control_active\":true,\"x\":0.1047,\"y\":0.0,\"z\":0.0753,\"pitch\":-1.0263,\"gripper\":0.5}",
              "arm_command 10필드·순서", arm);
        string ev = ArmPiV3Json.Event(ArmPiV3.EventEmergencyStop, cid, 127);
        Check(ev == "{\"protocol\":3,\"type\":\"emergency_stop\",\"client_id\":\"" + cid + "\",\"sequence\":127}",
              "event 4필드", ev);
        Check(ArmPiV3Json.Num(1e-5) == "1E-05" && ArmPiV3Json.Num(-0.0) == "0.0" && ArmPiV3Json.Num(2) == "2.0",
              "숫자 형식(지수·−0·정수처럼 보이는 실수)", ArmPiV3Json.Num(1e-5) + " " + ArmPiV3Json.Num(-0.0) + " " + ArmPiV3Json.Num(2));
        bool threw = false;
        try { ArmPiV3Json.ArmCommand(cid, 1, true, double.NaN, 0, 0, 0, 0); } catch (ArgumentException) { threw = true; }
        Check(threw, "arm_command NaN 거부(예외)");
        threw = false;
        try { ArmPiV3Json.Command("00000000-0000-0000-0000-000000000000", 1, false, 0, 0, 0); } catch (ArgumentException) { threw = true; }
        Check(threw, "nil UUID 거부");
        Check(ArmPiV3Json.IsUuid(ArmPiV3Json.NewClientId()), "새 UUID 형식");
        Check(Encoding.UTF8.GetByteCount(arm) <= ArmPiV3.MaxPacketBytes, "패킷 크기 ≤ 2048 B", Encoding.UTF8.GetByteCount(arm) + " B");

        // ACK 해석
        ArmPiAck ack; string err;
        string good = "{\"protocol\":3,\"type\":\"ack\",\"client_id\":\"" + cid +
                      "\",\"sequence\":124,\"accepted\":true,\"reason\":\"ok\",\"emergency_stop\":false,\"state\":\"active\"}";
        Check(ArmPiV3Ack.TryParse(good, out ack, out err) && ack.Sequence == 124 && ack.Accepted && ack.State == "active",
              "정상 ACK", err ?? ack.ToString());
        Check(ArmPiV3Ack.TryParse("{\"protocol\":3,\"type\":\"ack\",\"client_id\":null,\"sequence\":-1,\"accepted\":false," +
                                  "\"reason\":\"invalid_packet\",\"emergency_stop\":false,\"state\":\"idle\"}", out ack, out err)
              && ack.ClientId == null && ack.Sequence == -1, "식별 불가 패킷 ACK(client_id null, sequence −1)", err ?? "");
        Check(!ArmPiV3Ack.TryParse(good.Replace("\"sequence\":124", "\"sequence\":124.0"), out ack, out err), "실수 sequence 거부", err);
        Check(!ArmPiV3Ack.TryParse(good.Replace("\"protocol\":3", "\"protocol\":2"), out ack, out err), "protocol 2 거부", err);
        Check(!ArmPiV3Ack.TryParse(good.Replace("}", ",\"extra\":1}"), out ack, out err), "9번째 필드 거부", err);
        Check(!ArmPiV3Ack.TryParse(good.Replace("\"state\":\"active\"", "\"state\":\"active\",\"state\":\"idle\""), out ack, out err),
              "중복 키 거부", err);
        Check(!ArmPiV3Ack.TryParse(good.Replace(",\"state\":\"active\"", ""), out ack, out err), "필드 누락 거부", err);
        Check(!ArmPiV3Ack.TryParse(good.Replace("\"reason\":\"ok\"", "\"reason\":\"step_limit\""), out ack, out err),
              "모르는 reason 거부", err);
        Check(!ArmPiV3Ack.TryParse(good.Replace("\"accepted\":true", "\"accepted\":1"), out ack, out err), "불리언 자리에 정수 거부", err);
        Check(ArmPiV3Ack.TryParse(good.Replace("\"reason\":\"ok\"", "\"reason\":\"ik_unreachable\"").Replace("\"accepted\":true", "\"accepted\":false"),
                                  out ack, out err) && ack.Reason == ArmPiV3.ReasonIkUnreachable, "v3 신규 reason ik_unreachable", err ?? "");
        Check(!ArmPiV3Ack.TryParse(good + " x", out ack, out err), "객체 뒤 쓰레기 거부", err);

        // 검색 응답
        string nonce = "0123456789abcdef0123456789abcdef";
        string reply = "ARMPI_HERE/1|" + nonce + "|58ad7561bb722863312f66ee26e535e7|armpi-ros2|5001|3";
        ArmPiRobotInfo info;
        Check(ArmPiV3Discovery.TryParseReply(reply, nonce, out info) && info.Name == "armpi-ros2" && info.ControlPort == 5001,
              "검색 응답 해석", reply.Length + "자");
        Check(!ArmPiV3Discovery.TryParseReply(reply.Replace("|3", "|2"), nonce, out info), "protocol 2 응답 거부");
        Check(!ArmPiV3Discovery.TryParseReply(reply.Replace("ARMPI_HERE", "OTHERBOT_HERE"), nonce, out info), "다른 로봇 응답 거부");

        // 좌표 변환
        Vec3 uFwd = new Vec3(0, 0, 1), uRight = new Vec3(1, 0, 0), uUp = new Vec3(0, 1, 0);
        Vec3 r1 = ArmPiFrames.RosFromUnity(uFwd), r2 = ArmPiFrames.RosFromUnity(uRight), r3 = ArmPiFrames.RosFromUnity(uUp);
        Check(r1.X == 1 && r1.Y == 0 && r1.Z == 0, "Unity 앞(+Z) → ROS +x", r1.ToString());
        Check(r2.X == 0 && r2.Y == -1 && r2.Z == 0, "Unity 오른쪽(+X) → ROS −y", r2.ToString());
        Check(r3.X == 0 && r3.Y == 0 && r3.Z == 1, "Unity 위(+Y) → ROS +z", r3.ToString());
        Vec3 rr = ArmPiFrames.UnityFromRos(ArmPiFrames.RosFromUnity(new Vec3(0.3, -0.2, 0.7)));
        Check(Near(rr.X, 0.3, 1e-12) && Near(rr.Y, -0.2, 1e-12) && Near(rr.Z, 0.7, 1e-12), "위치 왕복 변환", rr.ToString());
        // Unity 에서 +Y 축으로 +90°(위에서 볼 때 시계 방향 = 오른쪽으로 돌기) → ROS 에서 yaw −90°
        Quat uq = Quat.AxisAngle(uUp, Math.PI / 2);
        Vec3 uTurned = uq.Rotate(uFwd);
        Check(Near(uTurned.X, 1, 1e-12) && Near(uTurned.Z, 0, 1e-12), "Unity +Y 90° 회전은 앞 → 오른쪽", uTurned.ToString());
        Quat rq = ArmPiFrames.RosFromUnity(uq);
        Vec3 rTurned = rq.Rotate(new Vec3(1, 0, 0));
        Check(Near(rTurned.X, 0, 1e-12) && Near(rTurned.Y, -1, 1e-12), "같은 회전을 ROS 로 바꾸면 +x → −y(오른쪽)", rTurned.ToString());
        Vec3 viaPoint = ArmPiFrames.RosFromUnity(uTurned);
        Check(Near(viaPoint.X, rTurned.X, 1e-12) && Near(viaPoint.Y, rTurned.Y, 1e-12) && Near(viaPoint.Z, rTurned.Z, 1e-12),
              "회전 변환 = 위치 변환과 일치(q_ros·v_ros = M·(q_u·v_u))");
        // 임의 회전에서도 일치하는지
        var rnd = new Random(20261102);
        double maxErr = 0;
        for (int k = 0; k < 1000; k++)
        {
            Quat q = Quat.AxisAngle(new Vec3(rnd.NextDouble() - 0.5, rnd.NextDouble() - 0.5, rnd.NextDouble() - 0.5), (rnd.NextDouble() - 0.5) * 6);
            Vec3 v = new Vec3(rnd.NextDouble() - 0.5, rnd.NextDouble() - 0.5, rnd.NextDouble() - 0.5);
            Vec3 a = ArmPiFrames.RosFromUnity(q.Rotate(v));
            Vec3 b = ArmPiFrames.RosFromUnity(q).Rotate(ArmPiFrames.RosFromUnity(v));
            maxErr = Math.Max(maxErr, (a - b).Length);
            Quat back = ArmPiFrames.UnityFromRos(ArmPiFrames.RosFromUnity(q));
            maxErr = Math.Max(maxErr, Math.Abs(back.X - q.X) + Math.Abs(back.Y - q.Y) + Math.Abs(back.Z - q.Z) + Math.Abs(back.W - q.W));
        }
        Check(maxErr < 1e-12, "무작위 회전 1000개: 회전·위치 변환 일치, 쿼터니언 왕복", "최대 오차 " + maxErr.ToString("E2", CultureInfo.InvariantCulture));
        Quat nose = Quat.AxisAngle(uRight, Math.PI / 2);  // Unity +X 90° = 앞이 아래로
        Check(Near(ArmPiFrames.ControllerPitch(nose), -Math.PI / 2, 1e-12), "컨트롤러 앞이 아래 → pitch −π/2");
        Check(Near(ArmPiFrames.ControllerPitch(Quat.Identity), 0, 1e-12), "컨트롤러 수평 → pitch 0");

        // 기구학
        var kin = new ArmPiKinematics();
        // 기본 자세 목표: IK → 관절 → FK 왕복(관절 기준·범위는 시험용 기본값)
        var home0 = ArmTarget.Home;
        double[] homeJ = new double[4];
        Check(kin.Inverse(home0.X, home0.Y, home0.Z, home0.Pitch, homeJ) == IkStatus.Ok, "기본 자세 목표 IK 풀림");
        double hp;
        Vec3 hpos = kin.Forward(homeJ, out hp);
        Check(Near(hpos.X, home0.X, 1e-9) && Near(hpos.Y, home0.Y, 1e-12) && Near(hpos.Z, home0.Z, 1e-9) && Near(hp, home0.Pitch, 1e-9),
              "기본 자세 IK → FK 왕복", hpos + " pitch " + hp.ToString("0.0000", CultureInfo.InvariantCulture));
        double[] j = new double[4];
        Check(kin.Inverse(hpos.X, hpos.Y, hpos.Z, hp, j) == IkStatus.Ok && Near(j[1], homeJ[1], 1e-9) && Near(j[2], homeJ[2], 1e-9) && Near(j[3], homeJ[3], 1e-9),
              "기본 자세 IK 왕복");
        Check(kin.Inverse(0.28, 0, 0.30, 0, null) == IkStatus.OutOfReach, "(0.28, 0, 0.30, 0) 도달 불가");
        Check(kin.Inverse(0.15, 0, 0.12, -Math.PI / 2, null) == IkStatus.OutOfReach, "사양 예시 (0.15, 0, 0.12, −π/2) 도 공개 링크 길이에서는 도달 불가");
        Check(kin.Inverse(0.25, 0, 0.10, 0, null) == IkStatus.JointLimit, "(0.25, 0, 0.10, 0) 관절 한계");
        Check(kin.Inverse(0.169, 0, 0.12, -0.7854, null) == IkStatus.Ok, "(0.169, 0, 0.12, −π/4) 도달 가능");

        // 컨트롤러 매핑
        var m = new ArmTeleopMapper();
        m.Engage(new Vec3(0, 1.0, 0.3), Quat.AxisAngle(uRight, 1.0263), ArmTarget.Home);
        ArmTarget t = m.Output;
        for (int k = 0; k < 40; k++) t = m.Update(new Vec3(0, 1.0, 0.33), Quat.AxisAngle(uRight, 1.0263), 0.0, 0.05);
        Check(Near(t.X, 0.1347, 1e-9) && Near(t.Z, 0.0753, 1e-9) && Near(t.Pitch, -1.0263, 1e-9),
              "손을 Unity 앞(+Z)으로 3 cm → 손끝 x +3 cm, pitch 유지", t.ToString());
        var m2 = new ArmTeleopMapper();
        m2.Kinematics = kin;  // 예전 기본 동작(IK 안 되면 직전 목표 유지)
        m2.Engage(new Vec3(0, 1, 0), Quat.Identity, ArmTarget.Home);
        ArmTarget t2 = m2.Update(new Vec3(0, 1.5, 0), Quat.Identity, 1.0, 0.05);
        Check(Near(t2.Z - ArmTarget.Home.Z, 0.0075, 1e-9) && t2.Gripper == 1.0,
              "속도 제한: 50 ms 에 최대 7.5 mm(0.15 m/s), 그리퍼는 바로 반영", t2.ToString());
        for (int k = 0; k < 200; k++) t2 = m2.Update(new Vec3(0, 1.5, 0), Quat.Identity, 1.0, 0.05);
        Check(m2.LastIkBlocked && kin.Inverse(t2.X, t2.Y, t2.Z, t2.Pitch, null) == IkStatus.Ok,
              "IK 가 안 풀리는 곳으로 가면 직전 유효 목표에서 멈춤", t2.ToString());
        var m3 = new ArmTeleopMapper();
        m3.Engage(new Vec3(0, 1, 0), Quat.Identity, ArmTarget.Home);
        ArmTarget t3 = m3.Output;
        for (int k = 0; k < 200; k++) t3 = m3.Update(new Vec3(0.5, 1, 0), Quat.Identity, 0.0, 0.05);
        Check(t3.Y >= Workspace.Conservative.YMin - 1e-12 && t3.Y < 0, "Unity 오른쪽 → ROS y 감소, 작업 공간 안", t3.ToString());

        // 목표 보정(로봇 arm.target_policy: nearest 와 같은 규칙)
        var pj = new ArmPiTargetProjector(kin, Workspace.Robot);
        ArmTarget pr; int how;
        Check(pj.Project(ArmTarget.Home, null, out pr, out how) && how == 0 && Near(pr.X, ArmTarget.Home.X, 1e-12),
              "보정: 닿는 목표(기본 자세)는 그대로", pr.ToString());
        Check(pj.Project(new ArmTarget(0.1047, 0.40, 0.0753, -1.0263, 0.5), ArmTarget.Home, out pr, out how) &&
              (how & ArmPiTargetProjector.AdjustBox) != 0 && pj.Reachable(pr) && pr.Gripper == 0.5,
              "보정: 상자 밖(y 0.40) → 상자 안·IK 해 있음, 그리퍼 그대로", pr.ToString() + " how=" + how);
        Check(pj.Project(new ArmTarget(0.32, 0.25, 0.30, 0.0, 1.0), ArmTarget.Home, out pr, out how) &&
              (how & ArmPiTargetProjector.AdjustPosition) != 0 && pj.Reachable(pr),
              "보정: 어떤 자세로도 닿지 않는 모서리 → 기본 자세 쪽으로 당겨 닿는 점", pr.ToString() + " how=" + how);
        Check(!pj.Project(new ArmTarget(0.32, 0.25, 0.30, 0.0, 1.0), null, out pr, out how) || pj.Reachable(pr),
              "보정: 기준(anchor)이 없으면 실패하거나 닿는 점");
        int bad = 0;
        var rng = new Random(11);
        ArmTarget anc = ArmTarget.Home;
        for (int k = 0; k < 2000; k++)
        {
            var want = new ArmTarget(rng.NextDouble() * 0.7 - 0.2, rng.NextDouble() - 0.5, rng.NextDouble() * 0.8 - 0.3,
                                     rng.NextDouble() * 2 * Math.PI - Math.PI, 0);
            if (!pj.Project(want, anc, out pr, out how) || !pj.Reachable(pr)) bad++;
            else anc = pr;
        }
        Check(bad == 0, "보정: 무작위 목표 2000개 모두 상자 안·IK 해 있는 자세로", "실패 " + bad);
        var m4 = new ArmTeleopMapper();
        m4.Projector = pj;
        m4.Engage(new Vec3(0, 1, 0), Quat.Identity, ArmTarget.Home);
        ArmTarget t4 = m4.Output;
        bool blocked4 = false;
        for (int k = 0; k < 200; k++) { t4 = m4.Update(new Vec3(0, 1.5, 0), Quat.Identity, 1.0, 0.05); blocked4 |= m4.LastIkBlocked; }
        Check(!blocked4 && pj.Reachable(t4) && t4.Z > t2.Z + 1e-6,
              "매퍼+보정: 같은 위쪽 이동에서 멈추지 않고 예전보다 더 올라감(닿는 자세 안)", t4.ToString() + " (예전 " + t2.ToString() + ")");
        var m5 = new ArmTeleopMapper();  // 기본: 미리 확인 안 함(로봇이 보정)
        m5.Engage(new Vec3(0, 1, 0), Quat.Identity, ArmTarget.Home);
        ArmTarget t5 = m5.Output;
        for (int k = 0; k < 200; k++) t5 = m5.Update(new Vec3(0, 1.5, 0), Quat.Identity, 1.0, 0.05);
        Check(!m5.LastIkBlocked && Near(t5.Z, Workspace.Robot.ZMax, 1e-9),
              "매퍼 기본: 미리 막지 않고 상자 끝까지 보낸다(로봇 nearest 가 보정)", t5.ToString());
    }

    // ───────────────────────── 2부 ─────────────────────────

    sealed class Link : IDisposable
    {
        public readonly UdpClient Udp;
        public readonly string ClientId = ArmPiV3Json.NewClientId();
        public long Seq;
        public readonly List<double> RttMs = new List<double>();

        public Link(string host, int port)
        {
            Udp = new UdpClient(AddressFamily.InterNetwork);
            Udp.Connect(host, port);  // 연결된 UDP: 로봇 주소·포트가 아닌 곳의 데이터그램은 받지 않는다
            Udp.Client.ReceiveTimeout = 50;
        }

        public long Next() { return ++Seq; }

        /// <summary>보내고 같은 sequence 의 ACK 를 기다린다(없으면 null).</summary>
        public ArmPiAck Exchange(string json, long seq, double timeoutSec = ArmPiV3.AckTimeoutSec)
        {
            var sw = Stopwatch.StartNew();
            byte[] data = Encoding.UTF8.GetBytes(json);
            Udp.Send(data, data.Length);
            while (sw.Elapsed.TotalSeconds < timeoutSec)
            {
                ArmPiAck a = Receive();
                if (a != null && a.Sequence == seq && string.Equals(a.ClientId, ClientId, StringComparison.OrdinalIgnoreCase))
                {
                    RttMs.Add(sw.Elapsed.TotalMilliseconds);
                    return a;
                }
            }
            return null;
        }

        public ArmPiAck Receive()
        {
            try
            {
                IPEndPoint from = null;
                byte[] b = Udp.Receive(ref from);
                ArmPiAck a; string err;
                return ArmPiV3Ack.TryParse(b, b.Length, out a, out err) ? a : null;
            }
            catch (SocketException)
            {
                return null;
            }
        }

        public void Dispose() { Udp.Dispose(); }
    }

    static string S(ArmPiAck a) { return a == null ? "ACK 없음" : a.ToString(); }

    static void NetworkTests(string host, int port, int dport)
    {
        // 검색(유니캐스트)
        using (var ds = new UdpClient(new IPEndPoint(IPAddress.Any, 0)))
        {
            string nonce = ArmPiV3Discovery.NewNonce();
            byte[] req = Encoding.ASCII.GetBytes(ArmPiV3Discovery.Request(nonce));
            ds.Client.ReceiveTimeout = 500;
            ds.Send(req, req.Length, host, dport);
            ArmPiRobotInfo info = null;
            int srcPort = -1;
            try
            {
                IPEndPoint from = null;
                byte[] b = ds.Receive(ref from);
                srcPort = from.Port;
                ArmPiV3Discovery.TryParseReply(Encoding.ASCII.GetString(b), nonce, out info);
            }
            catch (SocketException) { }
            Check(info != null && srcPort == dport, "검색 응답(출발 포트 = " + dport + ")",
                  info == null ? "응답 없음" : "이름 " + info.Name + ", id " + info.DeviceId + ", 제어 포트 " + info.ControlPort);
        }

        var kin = new ArmPiKinematics();
        var home = ArmTarget.Home;
        using (var L = new Link(host, port))
        {
            Console.WriteLine("   client_id " + L.ClientId);
            long s;
            ArmPiAck a;

            s = L.Next(); a = L.Exchange(ArmPiV3Json.Command(L.ClientId, s, false, 0, 0, 0), s);
            Check(a != null && a.Accepted && a.State == "idle", "차체 중립 → idle", S(a));
            s = L.Next(); a = L.Exchange(ArmPiV3Json.ArmCommand(L.ClientId, s, false, home.X, home.Y, home.Z, home.Pitch, 0), s);
            Check(a != null && a.Accepted && a.State == "idle", "팔 중립 → idle (팔 채널 state)", S(a));
            s = L.Next(); a = L.Exchange(ArmPiV3Json.Command(L.ClientId, s, true, 0.3, 0.0, 0.0), s);
            Check(a != null && a.Accepted && a.State == "active", "차체 활성(forward 0.3) → active", S(a));
            s = L.Next(); a = L.Exchange(ArmPiV3Json.Command(L.ClientId, s, true, 0.0, 0.3, 0.0), s);
            Check(a != null && a.Accepted && a.State == "active", "차체 활성(strafe 0.3, v3 신규 필드) → active", S(a));

            bool localOk = kin.Inverse(home.X, home.Y, home.Z, home.Pitch, null) == IkStatus.Ok;
            s = L.Next(); a = L.Exchange(ArmPiV3Json.ArmCommand(L.ClientId, s, true, home.X, home.Y, home.Z, home.Pitch, 0.3), s);
            Check(a != null && a.Accepted && a.State == "active" && localOk, "팔 기본 자세 목표 → 수락(로컬 IK 판정과 일치)", S(a));
            bool localBad = kin.Inverse(0.28, 0, 0.30, 0, null) != IkStatus.Ok;
            s = L.Next(); a = L.Exchange(ArmPiV3Json.ArmCommand(L.ClientId, s, true, 0.28, 0, 0.30, 0, 0.3), s);
            Check(a != null && !a.Accepted && a.Reason == ArmPiV3.ReasonIkUnreachable && localBad,
                  "상자 안·IK 불가 목표 → ik_unreachable(로컬 판정과 일치)", S(a));
            s = L.Next(); a = L.Exchange(ArmPiV3Json.ArmCommand(L.ClientId, s, true, 0.45, 0, 0.10, 0, 0.3), s);
            Check(a != null && !a.Accepted && a.Reason == ArmPiV3.ReasonWorkspaceLimit, "상자 밖 목표(x 0.45) → workspace_limit", S(a));
            // armpi.yaml 상자(x ≤ 0.32) 안, 가짜 로봇 기본 상자(x ≤ 0.30) 밖인 점: 가짜 로봇을 --workspace 로 맞춰 띄웠으면 수락
            bool inRobotBox = Workspace.Robot.Contains(0.31, 0, 0.06) && kin.Inverse(0.31, 0, 0.06, 0, null) == IkStatus.Ok;
            s = L.Next(); a = L.Exchange(ArmPiV3Json.ArmCommand(L.ClientId, s, true, 0.31, 0, 0.06, 0, 0.3), s);
            Check(a != null && a.Accepted && inRobotBox, "(0.31, 0, 0.06, 0): 로봇 상자 안·IK 가능 → 수락(가짜 로봇 --workspace 로봇 값)", S(a));
            s = L.Next(); a = L.Exchange(ArmPiV3Json.ArmCommand(L.ClientId, s, true, home.X, home.Y, home.Z, home.Pitch, 0.3), s);
            Check(a != null && a.Accepted && a.State == "active", "거부 뒤에도 팔 채널 유지(다시 수락)", S(a));

            // 순서
            a = L.Exchange(ArmPiV3Json.Command(L.ClientId, s, true, 0.3, 0, 0), s);
            Check(a != null && !a.Accepted && a.Reason == ArmPiV3.ReasonOutOfOrder, "같은 sequence 재전송 → out_of_order", S(a));

            // 비상정지: ACK 를 받을 때까지 50 ms 마다 새 sequence 로 재전송
            ArmPiAck ea = null;
            var sw = Stopwatch.StartNew();
            for (int k = 0; k < 10 && ea == null; k++)
            {
                s = L.Next();
                ea = L.Exchange(ArmPiV3Json.Event(ArmPiV3.EventEmergencyStop, L.ClientId, s), s, ArmPiV3.EstopRetrySec);
            }
            Check(ea != null && ea.EmergencyStop, "emergency_stop → ACK emergency_stop:true",
                  S(ea) + string.Format(CultureInfo.InvariantCulture, ", {0:0.0} ms", sw.Elapsed.TotalMilliseconds));
            s = L.Next(); a = L.Exchange(ArmPiV3Json.Command(L.ClientId, s, true, 0.3, 0, 0), s);
            Check(a != null && !a.Accepted && a.Reason == ArmPiV3.ReasonEmergencyStop, "비상정지 중 활성 → emergency_stop 거부", S(a));
            s = L.Next(); a = L.Exchange(ArmPiV3Json.ArmCommand(L.ClientId, s, true, home.X, home.Y, home.Z, home.Pitch, 0.3), s);
            Check(a != null && !a.Accepted && a.Reason == ArmPiV3.ReasonEmergencyStop, "비상정지 중 팔 활성 → emergency_stop 거부", S(a));
            s = L.Next(); a = L.Exchange(ArmPiV3Json.Event(ArmPiV3.EventClearEmergency, L.ClientId, s), s);
            Check(a != null && a.Accepted && !a.EmergencyStop, "clear_emergency(소유자) → 해제", S(a));
            s = L.Next(); a = L.Exchange(ArmPiV3Json.Command(L.ClientId, s, true, 0.3, 0, 0), s);
            Check(a != null && !a.Accepted && a.Reason == ArmPiV3.ReasonNeutralRequired, "해제 직후 활성 → neutral_required", S(a));
            s = L.Next(); a = L.Exchange(ArmPiV3Json.Command(L.ClientId, s, false, 0, 0, 0), s);
            Check(a != null && a.Accepted && a.State == "idle", "새 중립 → idle", S(a));
            s = L.Next(); a = L.Exchange(ArmPiV3Json.ArmCommand(L.ClientId, s, false, home.X, home.Y, home.Z, home.Pitch, 0), s);
            Check(a != null && a.Accepted && a.State == "idle", "팔도 새 중립 → idle", S(a));

            // watchdog: 0.6 s 동안 아무것도 보내지 않음 → 다음 활성은 거부(timeout)
            s = L.Next(); a = L.Exchange(ArmPiV3Json.Command(L.ClientId, s, true, 0.3, 0, 0), s);
            Check(a != null && a.Accepted && a.State == "active", "다시 활성", S(a));
            System.Threading.Thread.Sleep(600);
            s = L.Next(); a = L.Exchange(ArmPiV3Json.Command(L.ClientId, s, true, 0.3, 0, 0), s);
            Check(a != null && !a.Accepted && a.State == "timeout" && a.Reason == ArmPiV3.ReasonNeutralRequired,
                  "0.6 s 침묵 뒤 활성 → state timeout, neutral_required", S(a));

            // 다른 UUID 는 lease 동안 not_owner
            using (var L2 = new Link(host, port))
            {
                long s2 = L2.Next();
                ArmPiAck b = L2.Exchange(ArmPiV3Json.Command(L2.ClientId, s2, false, 0, 0, 0), s2);
                Check(b != null && !b.Accepted && b.Reason == ArmPiV3.ReasonNotOwner, "lease 중 다른 조종기 → not_owner", S(b));
            }

            // 끝: 중립 2개 + disconnect
            s = L.Next(); L.Exchange(ArmPiV3Json.Command(L.ClientId, s, false, 0, 0, 0), s);
            s = L.Next(); L.Exchange(ArmPiV3Json.ArmCommand(L.ClientId, s, false, home.X, home.Y, home.Z, home.Pitch, 0), s);
            s = L.Next(); a = L.Exchange(ArmPiV3Json.Event(ArmPiV3.EventDisconnect, L.ClientId, s), s);
            Check(a != null && a.Accepted, "disconnect 수락", S(a));
            s = L.Next(); a = L.Exchange(ArmPiV3Json.Command(L.ClientId, s, false, 0, 0, 0), s);
            Check(a != null && !a.Accepted && a.Reason == ArmPiV3.ReasonNotOwner, "disconnect 한 UUID 재사용 → not_owner", S(a));

            L.RttMs.Sort();
            if (L.RttMs.Count > 0)
            {
                Console.WriteLine(string.Format(CultureInfo.InvariantCulture,
                    "   ACK 왕복 {0}회: 중앙 {1:0.00} ms, 최대 {2:0.00} ms (같은 컨테이너 루프백, 실제 Wi-Fi 아님)",
                    L.RttMs.Count, L.RttMs[L.RttMs.Count / 2], L.RttMs[L.RttMs.Count - 1]));
            }
        }
    }
}
