package kr.wisoft.armpi.controller;

import java.net.URI;
import java.net.URLDecoder;

/** AppUiPolicy 단위 시험(맥 JVM, JUnit 없이). 실행: clients/android_app/test/run_policy_test.sh */
public final class AppUiPolicyTest {
    private static int pass, fail;

    private static void check(String name, boolean ok) {
        if (ok) pass++; else fail++;
        System.out.println((ok ? "[PASS] " : "[FAIL] ") + name);
    }

    /** Android Uri 처럼 scheme·host·port(없으면 -1)·path(쿼리 제외)를 뽑아 판정한다. */
    private static AppUiPolicy.Decision decide(AppUiPolicy p, String url) {
        URI u = URI.create(url);
        return p.decide(u.getScheme(), u.getHost(), u.getPort(), u.getPath());
    }

    private static AppUiPolicy.Kind kind(AppUiPolicy p, String url) {
        return decide(p, url).kind;
    }

    public static void main(String[] args) throws Exception {
        AppUiPolicy p = new AppUiPolicy("192.168.0.50", 18765);
        String page = p.pageUrl();
        check("화면 주소 = http://IP:18765/app-ui/control.html?video=…", page.startsWith("http://192.168.0.50:18765/app-ui/control.html?video="));
        String video = URLDecoder.decode(page.substring(page.indexOf("video=") + 6), "UTF-8");
        check("영상 주소 = IP:8080 /stream ros_compressed + sensor_data",
                video.equals("http://192.168.0.50:8080/stream?topic=/usb_cam/image_raw&type=ros_compressed&qos_profile=sensor_data"));

        AppUiPolicy.Decision d = decide(p, page);
        check("화면 주소 → assets/app_ui/control.html (text/html)",
                d.kind == AppUiPolicy.Kind.ASSET && "app_ui/control.html".equals(d.asset) && "text/html".equals(d.mime));
        check("주 프레임 이동: 조종 화면만 허용", p.navigationAllowed("http", "192.168.0.50", 18765, "/app-ui/control.html"));
        check("주 프레임 이동: 로봇 /mobile 거부", !p.navigationAllowed("http", "192.168.0.50", 18765, "/mobile"));
        check("주 프레임 이동: 다른 사이트 거부", !p.navigationAllowed("https", "example.com", -1, "/"));

        for (String api : new String[]{"/api/config", "/api/state?screen=ab12", "/api/input", "/api/cmd"}) {
            check("로봇 API 그대로 통과: " + api, kind(p, "http://192.168.0.50:18765" + api) == AppUiPolicy.Kind.NETWORK);
        }
        check("카메라 8080 /stream 통과", kind(p, video + "&t=123") == AppUiPolicy.Kind.NETWORK);
        check("호스트 대소문자 무시", kind(p, "http://192.168.0.50:18765/api/state") == AppUiPolicy.Kind.NETWORK
                && new AppUiPolicy("ArmPi-ROS2.local", 18765).decide("http", "armpi-ros2.LOCAL", 18765, "/api/state").kind
                == AppUiPolicy.Kind.NETWORK);

        String[] blocked = {
            "http://192.168.0.51:18765/api/state",           // 다른 로봇 주소
            "http://192.168.0.50:9090/",                     // rosbridge 포트
            "http://192.168.0.50/api/state",                 // 포트 없음(80)
            "https://192.168.0.50:18765/api/state",          // https
            "http://192.168.0.50:18765/mobile",              // 로봇 화면
            "http://192.168.0.50:18765/index.html",
            "http://192.168.0.50:18765/",
            "http://192.168.0.50:8080/snapshot?topic=/usb_cam/image_raw",   // 스냅샷(RELIABLE 위험)
            "http://192.168.0.50:8080/stream/../x",
            "http://192.168.0.50:18765/app-ui/../api/state",
            "http://192.168.0.50:18765/app-ui/sub/control.html",
            "http://192.168.0.50:18765/app-ui/Control.HTML",
            "http://192.168.0.50:18765/app-ui/",
            "http://evil.example:18765/app-ui/control.html",
            "http://192.168.0.50.evil.example:18765/api/state",
            "file:///android_asset/app_ui/control.html",
            "javascript:alert(1)",
        };
        for (String b : blocked) {
            AppUiPolicy.Kind k;
            try {
                k = kind(p, b);
            } catch (IllegalArgumentException e) {   // URI 로 해석도 안 되는 주소
                k = p.decide(b.substring(0, b.indexOf(':')), null, -1, null).kind;
            }
            check("막음: " + b, k == AppUiPolicy.Kind.BLOCK);
        }
        check("about:blank(닫을 때) 통과", p.decide("about", "", -1, "blank").kind == AppUiPolicy.Kind.NETWORK);
        check("scheme 없음 막음", p.decide(null, "192.168.0.50", 18765, "/api/state").kind == AppUiPolicy.Kind.BLOCK);

        boolean threw = false;
        try { new AppUiPolicy("192.168.0.1/evil", 18765); } catch (IllegalArgumentException e) { threw = true; }
        check("잘못된 로봇 주소는 거부", threw);
        threw = false;
        try { new AppUiPolicy("192.168.0.1", 0); } catch (IllegalArgumentException e) { threw = true; }
        check("잘못된 포트는 거부", threw);

        AppUiPolicy q = new AppUiPolicy("10.0.0.5", 18800);
        check("포트를 바꿔 연결하면 그 포트만 API 허용", q.decide("http", "10.0.0.5", 18800, "/api/cmd").kind == AppUiPolicy.Kind.NETWORK
                && q.decide("http", "10.0.0.5", 18765, "/api/cmd").kind == AppUiPolicy.Kind.BLOCK);

        System.out.println("요약: 통과 " + pass + ", 실패 " + fail);
        if (fail > 0) System.exit(1);
    }
}
