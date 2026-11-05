package kr.wisoft.armpi.controller;

import java.io.UnsupportedEncodingException;
import java.net.URLEncoder;
import java.util.Locale;
import java.util.regex.Pattern;

/**
 * 앱 안 조종 화면(assets/app_ui/)을 띄우는 주소와, 조종 화면이 보낼 수 있는 요청의 허용 범위.
 * Android API 를 쓰지 않는다(맥에서 단위 시험: clients/android_app/test/run_policy_test.sh).
 *
 * 화면 주소: http://&lt;로봇&gt;:18765/app-ui/control.html?video=... — 이 경로는 앱이 assets 에서 꺼내 준다.
 * 그래서 화면의 출처가 로봇 웹 조종기(http://&lt;로봇&gt;:18765)와 같아, 로봇 쪽 코드를 바꾸지 않고
 * /api/* 를 그대로 쓸 수 있다(게이트웨이는 POST 의 Origin 이 http://&lt;Host&gt; 일 때만 받는다. file:// 화면은 Origin 이 null 이라 거부됨).
 *
 * 허용: 같은 로봇 주소의 18765 /app-ui/&lt;파일&gt;(앱 안 파일), 18765 /api/*(로봇 API), 8080 /stream(카메라).
 * 나머지는 모두 막는다(다른 주소·다른 포트·https·로봇의 다른 페이지).
 */
final class AppUiPolicy {
    static final int WEB_PORT = 18765;
    static final int CAMERA_PORT = 8080;
    static final String PREFIX = "/app-ui/";
    static final String PAGE = "control.html";
    static final String ASSET_DIR = "app_ui/";
    /** 원시 영상은 sensor_data(best-effort)로만 받는다(기본 RELIABLE 은 로봇 안 ROS 통신을 막을 수 있음). */
    static final String VIDEO_PATH = "/stream?topic=/usb_cam/image_raw&type=ros_compressed&qos_profile=sensor_data";

    enum Kind { ASSET, NETWORK, BLOCK }

    static final class Decision {
        final Kind kind;
        final String asset;     // ASSET 일 때 assets 안 경로(app_ui/...)
        final String mime;

        Decision(Kind kind, String asset, String mime) {
            this.kind = kind;
            this.asset = asset;
            this.mime = mime;
        }

        @Override
        public String toString() {
            return kind + (asset != null ? "(" + asset + ")" : "");
        }
    }

    private static final Pattern HOST = Pattern.compile("^[A-Za-z0-9.-]{1,253}$");
    private static final Pattern ASSET_NAME = Pattern.compile("^[a-z0-9_-]{1,64}\\.(html|js|css|png|svg)$");

    final String host;
    final int webPort;

    AppUiPolicy(String host, int webPort) {
        if (host == null || !HOST.matcher(host).matches()) throw new IllegalArgumentException("로봇 주소 형식: " + host);
        if (webPort < 1 || webPort > 65535) throw new IllegalArgumentException("포트: " + webPort);
        this.host = host.toLowerCase(Locale.ROOT);
        this.webPort = webPort;
    }

    String base() {
        return "http://" + host + ":" + webPort;
    }

    String videoUrl() {
        return "http://" + host + ":" + CAMERA_PORT + VIDEO_PATH;
    }

    String pageUrl() {
        try {
            return base() + PREFIX + PAGE + "?video=" + URLEncoder.encode(videoUrl(), "UTF-8");
        } catch (UnsupportedEncodingException e) {
            throw new IllegalStateException(e);
        }
    }

    /** 화면 이동(주 프레임)은 앱 안 조종 화면만 허용한다. */
    boolean navigationAllowed(String scheme, String reqHost, int port, String path) {
        Decision d = decide(scheme, reqHost, port, path);
        return d.kind == Kind.ASSET;
    }

    /**
     * 요청 판정. port 는 URL 에 적힌 값(없으면 -1 → http 기본 80).
     * path 는 쿼리를 뺀 경로(Uri.getPath()).
     */
    Decision decide(String scheme, String reqHost, int port, String path) {
        if (scheme == null || reqHost == null) return block();
        String s = scheme.toLowerCase(Locale.ROOT);
        if (s.equals("about") || s.equals("data") || s.equals("blob")) return new Decision(Kind.NETWORK, null, null);
        if (!s.equals("http")) return block();
        if (!reqHost.toLowerCase(Locale.ROOT).equals(host)) return block();
        int p = port < 0 ? 80 : port;
        String path0 = path == null || path.isEmpty() ? "/" : path;
        if (path0.contains("..") || path0.contains("\\") || path0.contains("%")) return block();
        if (p == webPort) {
            if (path0.startsWith(PREFIX)) {
                String name = path0.substring(PREFIX.length());
                if (!ASSET_NAME.matcher(name).matches()) return block();
                return new Decision(Kind.ASSET, ASSET_DIR + name, mimeOf(name));
            }
            if (path0.startsWith("/api/")) return new Decision(Kind.NETWORK, null, null);
            return block();
        }
        if (p == CAMERA_PORT && path0.equals("/stream")) return new Decision(Kind.NETWORK, null, null);
        return block();
    }

    private static Decision block() {
        return new Decision(Kind.BLOCK, null, null);
    }

    static String mimeOf(String name) {
        if (name.endsWith(".html")) return "text/html";
        if (name.endsWith(".js")) return "text/javascript";
        if (name.endsWith(".css")) return "text/css";
        if (name.endsWith(".png")) return "image/png";
        if (name.endsWith(".svg")) return "image/svg+xml";
        return "application/octet-stream";
    }
}
