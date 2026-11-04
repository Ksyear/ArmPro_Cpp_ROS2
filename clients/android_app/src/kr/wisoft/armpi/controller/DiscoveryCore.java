package kr.wisoft.armpi.controller;

import java.io.IOException;
import java.io.UnsupportedEncodingException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.InterfaceAddress;
import java.net.NetworkInterface;
import java.net.SocketTimeoutException;
import java.net.URLDecoder;
import java.nio.charset.StandardCharsets;
import java.security.SecureRandom;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.regex.Pattern;

/**
 * ArmPi Protocol v3 로봇 검색(UDP 5002). Android 클래스에 의존하지 않는다(맥 JVM 에서도 시험 가능).
 *
 * 요청: {@code ARMPI_DISCOVER/1|<32 hex nonce>} → 응답: {@code ARMPI_HERE/1|nonce|<32 hex id>|<URL 인코딩 이름>|<제어 포트>|3}
 * (docs/01_PROTOCOL_V3_SPEC.md 1절). 응답은 출발 포트 5002, 우리 nonce, 형식이 맞는 것만 받는다.
 */
public final class DiscoveryCore {
    public static final int DISCOVERY_PORT = 5002;
    /** 대체 AP(ArmPi-ROS2)에 붙었을 때 로봇 주소. 브로드캐스트가 막힌 휴대폰을 위해 유니캐스트로도 묻는다. */
    public static final String FALLBACK_AP_ROBOT = "192.168.149.1";

    private static final Pattern HEX32 = Pattern.compile("[0-9a-f]{32}");
    private static final Pattern NAME = Pattern.compile("(?:[A-Za-z0-9_.~-]|%[0-9A-Fa-f]{2}){1,384}");
    private static final Pattern DIGITS = Pattern.compile("[0-9]{1,5}");

    private DiscoveryCore() {}

    public static final class Robot {
        public final String id, name, address;
        public final int port;

        Robot(String id, String name, String address, int port) {
            this.id = id;
            this.name = name;
            this.address = address;
            this.port = port;
        }

        @Override public String toString() {
            return name + " (" + address + ", 제어 포트 " + port + ", id " + id.substring(0, 8) + ")";
        }
    }

    /** 검색 대상: 255.255.255.255, 각 인터페이스의 브로드캐스트 주소, 대체 AP 로봇 주소. */
    public static List<InetAddress> defaultTargets() {
        LinkedHashSet<InetAddress> out = new LinkedHashSet<>();
        try {
            out.add(InetAddress.getByName("255.255.255.255"));
        } catch (IOException ignored) {
            // 숫자 주소라 실패하지 않음
        }
        try {
            for (NetworkInterface ni : Collections.list(NetworkInterface.getNetworkInterfaces())) {
                if (!ni.isUp() || ni.isLoopback()) continue;
                for (InterfaceAddress ia : ni.getInterfaceAddresses()) {
                    InetAddress b = ia.getBroadcast();
                    if (b != null) out.add(b);
                }
            }
        } catch (IOException | RuntimeException ignored) {
            // 인터페이스 목록을 못 읽으면 255.255.255.255 만 쓴다
        }
        try {
            out.add(InetAddress.getByName(FALLBACK_AP_ROBOT));
        } catch (IOException ignored) {
            // 숫자 주소라 실패하지 않음
        }
        return new ArrayList<>(out);
    }

    /** targets 로 0.5 s 마다 검색 요청을 보내고 timeoutMs 동안 응답을 모은다. */
    public static List<Robot> discover(List<InetAddress> targets, int port, int timeoutMs) throws IOException {
        byte[] nonceBytes = new byte[16];
        new SecureRandom().nextBytes(nonceBytes);
        StringBuilder sb = new StringBuilder();
        for (byte b : nonceBytes) sb.append(String.format("%02x", b & 0xff));
        String nonce = sb.toString();
        byte[] request = ("ARMPI_DISCOVER/1|" + nonce).getBytes(StandardCharsets.US_ASCII);

        LinkedHashMap<String, Robot> found = new LinkedHashMap<>();
        DatagramSocket sock = new DatagramSocket(null);
        try {
            sock.setReuseAddress(true);
            sock.bind(new InetSocketAddress(0));
            sock.setBroadcast(true);
            sock.setSoTimeout(120);
            long end = System.nanoTime() + timeoutMs * 1_000_000L;
            long nextProbe = 0;
            byte[] buf = new byte[1024];
            while (System.nanoTime() < end) {
                long now = System.nanoTime();
                if (now >= nextProbe) {
                    nextProbe = now + 500_000_000L;
                    for (InetAddress t : targets) {
                        try {
                            sock.send(new DatagramPacket(request, request.length, t, port));
                        } catch (IOException ignored) {
                            // 경로 없는 대상(다른 망의 대체 AP 주소 등)은 건너뛴다
                        }
                    }
                }
                DatagramPacket p = new DatagramPacket(buf, buf.length);
                try {
                    sock.receive(p);
                } catch (SocketTimeoutException e) {
                    continue;
                }
                if (p.getPort() != port) continue;
                Robot r = parseReply(Arrays.copyOf(p.getData(), p.getLength()), nonce, p.getAddress().getHostAddress());
                if (r != null) found.put(r.id + "@" + r.address, r);
            }
        } finally {
            sock.close();
        }
        return new ArrayList<>(found.values());
    }

    /** 응답 1개 해석(파이썬 armpi_v3_client.parse_discovery_reply 와 같은 규칙). 틀리면 null. */
    public static Robot parseReply(byte[] pkt, String nonce, String address) {
        if (pkt.length < 80 || pkt.length > 640) return null;
        for (byte b : pkt) {
            if (b < 32 || b > 126) return null;
        }
        String[] f = new String(pkt, StandardCharsets.US_ASCII).split("\\|", -1);
        if (f.length != 6 || !"ARMPI_HERE/1".equals(f[0]) || !nonce.equals(f[1]) || !HEX32.matcher(f[2]).matches()
                || !NAME.matcher(f[3]).matches() || !"3".equals(f[5]) || !DIGITS.matcher(f[4]).matches()) {
            return null;
        }
        int controlPort = Integer.parseInt(f[4]);
        if (controlPort < 1 || controlPort > 65535) return null;
        String name;
        try {
            name = URLDecoder.decode(f[3], "UTF-8");
        } catch (UnsupportedEncodingException | IllegalArgumentException e) {
            name = f[3];
        }
        return new Robot(f[2], name, address, controlPort);
    }
}
