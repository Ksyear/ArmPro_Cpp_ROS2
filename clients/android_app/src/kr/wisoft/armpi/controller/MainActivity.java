package kr.wisoft.armpi.controller;

import android.app.Activity;
import android.content.Context;
import android.content.SharedPreferences;
import android.content.res.Configuration;
import android.graphics.Typeface;
import android.graphics.drawable.GradientDrawable;
import android.net.Uri;
import android.net.wifi.WifiManager;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.text.InputType;
import android.util.TypedValue;
import android.view.Gravity;
import android.view.View;
import android.view.ViewGroup;
import android.view.WindowInsets;
import android.view.WindowInsetsController;
import android.view.WindowManager;
import android.view.inputmethod.EditorInfo;
import android.view.inputmethod.InputMethodManager;
import android.webkit.WebChromeClient;
import android.webkit.WebResourceError;
import android.webkit.WebResourceRequest;
import android.webkit.WebResourceResponse;
import android.webkit.WebSettings;
import android.webkit.WebView;
import android.webkit.WebViewClient;
import android.widget.Button;
import android.widget.EditText;
import android.widget.FrameLayout;
import android.widget.HorizontalScrollView;
import android.widget.LinearLayout;
import android.widget.ScrollView;
import android.widget.TextView;

import java.io.ByteArrayInputStream;
import java.io.InputStream;
import java.net.HttpURLConnection;
import java.net.URL;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * ArmPi 조종기 Android 앱.
 *
 * 첫 화면: 로봇 찾기(UDP 5002 검색) / IP 직접 입력(최근 값 기억) → 연결하면 전체 화면 WebView 로
 * 앱 안 조종 화면(assets/app_ui/control.html)을 연다. 뒤로 가기 = 첫 화면.
 * 조종 화면은 http://&lt;IP&gt;:18765/app-ui/control.html 주소로 열고, 그 경로는 앱이 assets 에서 꺼내 준다
 * (AppUiPolicy). 그래서 로봇 웹 조종기와 출처가 같아 로봇의 /api/* 를 그대로 쓴다(로봇 쪽 변경 없음).
 * 조종 화면이 보낼 수 있는 요청은 고른 로봇의 18765(/api/*)와 8080(/stream)뿐이다.
 * 조종 규칙(입력 0.3 s 끊기면 중립, 손 떼면 중립, 조종권, 비상정지)은 조종 화면과 로봇 서버 쪽에 있다.
 * 앱은 화면 꺼짐 방지, 백그라운드로 갈 때 중립 송신·JS 타이머 정지를 맡는다. 화면은 가로·세로 모두 쓴다.
 */
public class MainActivity extends Activity {
    static final int WEB_PORT = AppUiPolicy.WEB_PORT;
    static final String PREFS = "armpi";
    static final String KEY_RECENT = "recent";
    static final int MAX_RECENT = 5;
    static final String UA_MARK = " ArmPiController-Android/1.1";
    static final int BG = 0xFF0D1014, PANEL = 0xFF1B2128, LINE = 0xFF2E3640, TEXT = 0xFFEEF2F7, DIM = 0xFFA8B2C0,
            ACCENT = 0xFF4EA1FF, BAD = 0xFFFF6B6B, OK = 0xFF3ECF8E;

    private final Handler ui = new Handler(Looper.getMainLooper());
    private SharedPreferences prefs;
    private FrameLayout root;
    private ScrollView home;
    private LinearLayout cols, leftCol;
    private LinearLayout foundList, recentRow;
    private EditText ipEdit;
    private TextView status;
    private Button findBtn, connectBtn;
    private WebView web;
    private boolean paused;
    private boolean discovering;
    private int connectSeq;

    @Override
    protected void onCreate(Bundle state) {
        super.onCreate(state);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        prefs = getSharedPreferences(PREFS, MODE_PRIVATE);
        root = new FrameLayout(this);
        root.setBackgroundColor(BG);
        setContentView(root);
        buildHome();
        setFullscreenUi(false);
        startDiscovery();
    }

    // ───────────────────────── 첫 화면 ─────────────────────────

    private int dp(float v) {
        return Math.round(TypedValue.applyDimension(TypedValue.COMPLEX_UNIT_DIP, v, getResources().getDisplayMetrics()));
    }

    private GradientDrawable box(int color, int stroke) {
        GradientDrawable d = new GradientDrawable();
        d.setColor(color);
        d.setCornerRadius(dp(10));
        if (stroke != 0) d.setStroke(dp(1), stroke);
        return d;
    }

    private TextView text(String s, float sp, int color, boolean bold) {
        TextView t = new TextView(this);
        t.setText(s);
        t.setTextSize(TypedValue.COMPLEX_UNIT_SP, sp);
        t.setTextColor(color);
        if (bold) t.setTypeface(Typeface.DEFAULT_BOLD);
        return t;
    }

    private Button button(String s, boolean primary) {
        Button b = new Button(this);
        b.setText(s);
        b.setAllCaps(false);
        b.setTextColor(primary ? 0xFF00142B : TEXT);
        b.setTextSize(TypedValue.COMPLEX_UNIT_SP, 15);
        b.setBackground(box(primary ? ACCENT : PANEL, primary ? 0 : LINE));
        b.setPadding(dp(14), dp(8), dp(14), dp(8));
        return b;
    }

    private LinearLayout.LayoutParams lp(int w, int h, float weight, int top) {
        LinearLayout.LayoutParams p = new LinearLayout.LayoutParams(w, h, weight);
        p.topMargin = dp(top);
        return p;
    }

    private void buildHome() {
        home = new ScrollView(this);
        home.setFillViewport(true);
        cols = new LinearLayout(this);
        home.addView(cols, new ViewGroup.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.WRAP_CONTENT));

        LinearLayout left = new LinearLayout(this);
        left.setOrientation(LinearLayout.VERTICAL);
        LinearLayout right = new LinearLayout(this);
        right.setOrientation(LinearLayout.VERTICAL);
        leftCol = left;
        cols.addView(left);
        cols.addView(right);
        layoutHome();

        LinearLayout title = new LinearLayout(this);
        title.setOrientation(LinearLayout.HORIZONTAL);
        title.setGravity(Gravity.BOTTOM);
        title.addView(text("ArmPi 조종기", 26, TEXT, true));
        title.addView(text("  앱 " + appVersion(), 13, DIM, false));
        left.addView(title);
        left.addView(text("같은 와이파이의 로봇을 찾거나 IP 를 입력하세요.", 14, DIM, false), lp(-1, -2, 0, 4));
        findBtn = button("로봇 찾기", true);
        findBtn.setOnClickListener(v -> startDiscovery());
        left.addView(findBtn, lp(-1, dp(52), 0, 16));
        foundList = new LinearLayout(this);
        foundList.setOrientation(LinearLayout.VERTICAL);
        left.addView(foundList, lp(-1, -2, 0, 8));

        right.addView(text("로봇 IP 직접 입력", 15, DIM, true));
        LinearLayout row = new LinearLayout(this);
        row.setOrientation(LinearLayout.HORIZONTAL);
        ipEdit = new EditText(this);
        ipEdit.setSingleLine(true);
        ipEdit.setHint("예: 192.168.0.50");
        ipEdit.setTextColor(TEXT);
        ipEdit.setHintTextColor(0xFF66707C);
        ipEdit.setTextSize(TypedValue.COMPLEX_UNIT_SP, 17);
        ipEdit.setInputType(InputType.TYPE_CLASS_TEXT | InputType.TYPE_TEXT_VARIATION_URI);
        ipEdit.setImeOptions(EditorInfo.IME_ACTION_GO);
        ipEdit.setBackground(box(PANEL, LINE));
        ipEdit.setPadding(dp(12), dp(8), dp(12), dp(8));
        ipEdit.setOnEditorActionListener((v, id, ev) -> {
            connect(ipEdit.getText().toString());
            return true;
        });
        List<String> recent = recentList();
        if (!recent.isEmpty()) ipEdit.setText(recent.get(0));
        row.addView(ipEdit, new LinearLayout.LayoutParams(0, dp(52), 1f));
        connectBtn = button("연결", true);
        connectBtn.setOnClickListener(v -> connect(ipEdit.getText().toString()));
        LinearLayout.LayoutParams cp = new LinearLayout.LayoutParams(ViewGroup.LayoutParams.WRAP_CONTENT, dp(52));
        cp.leftMargin = dp(8);
        row.addView(connectBtn, cp);
        right.addView(row, lp(-1, -2, 0, 8));

        HorizontalScrollView hs = new HorizontalScrollView(this);
        recentRow = new LinearLayout(this);
        recentRow.setOrientation(LinearLayout.HORIZONTAL);
        hs.addView(recentRow);
        right.addView(hs, lp(-1, -2, 0, 8));
        refreshRecent();

        status = text("", 14, DIM, false);
        status.setTextIsSelectable(true);
        right.addView(status, lp(-1, -2, 0, 12));
        right.addView(text("· 휴대폰과 로봇이 같은 와이파이에 있어야 합니다.\n"
                + "· 로봇이 와이파이에 못 붙으면 대체 AP 'ArmPi-ROS2'를 켭니다(비밀번호는 담당자에게 확인). "
                + "휴대폰을 그 AP 에 연결한 뒤 192.168.149.1 로 연결하세요.\n"
                + "· 연결하면 앱 안 조종 화면이 열립니다(로봇 웹 조종기 http://<IP>:" + WEB_PORT + " 의 API 사용). 뒤로 가기 = 이 화면.\n"
                + "· 조종 화면: 가로 = 이동 | 카메라 | 비상정지, 세로 = 카메라 / 비상정지 / 이동. 팔은 \"팔 조종\" 버튼으로 엽니다.\n"
                + "· 처음에는 바퀴를 띄우고 시험하세요. 빨간 버튼 = 비상정지(소프트웨어 정지). 전원 스위치를 손 닿는 곳에 두세요.",
                13, DIM, false), lp(-1, -2, 0, 14));
        root.addView(home, new FrameLayout.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.MATCH_PARENT));
    }

    /** 첫 화면: 가로는 두 칸(찾기 | IP 입력), 세로는 위아래로. */
    private void layoutHome() {
        if (cols == null) return;
        boolean portrait = getResources().getConfiguration().orientation == Configuration.ORIENTATION_PORTRAIT;
        cols.setOrientation(portrait ? LinearLayout.VERTICAL : LinearLayout.HORIZONTAL);
        cols.setPadding(dp(portrait ? 18 : 28), dp(18), dp(portrait ? 18 : 28), dp(18));
        LinearLayout.LayoutParams a, b;
        if (portrait) {
            a = new LinearLayout.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.WRAP_CONTENT);
            b = new LinearLayout.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.WRAP_CONTENT);
            b.topMargin = dp(22);
        } else {
            a = new LinearLayout.LayoutParams(0, ViewGroup.LayoutParams.WRAP_CONTENT, 1f);
            a.rightMargin = dp(24);
            b = new LinearLayout.LayoutParams(0, ViewGroup.LayoutParams.WRAP_CONTENT, 1f);
        }
        leftCol.setLayoutParams(a);
        cols.getChildAt(1).setLayoutParams(b);
    }

    @SuppressWarnings("deprecation")
    private String appVersion() {
        try {
            return getPackageManager().getPackageInfo(getPackageName(), 0).versionName;
        } catch (Exception e) {
            return "?";
        }
    }

    @Override
    public void onConfigurationChanged(Configuration c) {
        super.onConfigurationChanged(c);
        layoutHome();   // 조종 화면(WebView)은 스스로 가로·세로 배치를 바꾼다(돌리면 입력 중립)
    }

    private void setStatus(String s, int color) {
        status.setText(s);
        status.setTextColor(color);
    }

    private List<String> recentList() {
        String raw = prefs.getString(KEY_RECENT, "");
        List<String> out = new ArrayList<>();
        for (String s : raw.split("\n")) if (!s.trim().isEmpty()) out.add(s.trim());
        return out;
    }

    private void remember(String host) {
        List<String> list = recentList();
        list.remove(host);
        list.add(0, host);
        while (list.size() > MAX_RECENT) list.remove(list.size() - 1);
        StringBuilder sb = new StringBuilder();
        for (String s : list) sb.append(s).append('\n');
        prefs.edit().putString(KEY_RECENT, sb.toString()).apply();
        refreshRecent();
    }

    private void refreshRecent() {
        recentRow.removeAllViews();
        for (final String h : recentList()) {
            Button b = button(h, false);
            b.setTextSize(TypedValue.COMPLEX_UNIT_SP, 13);
            b.setOnClickListener(v -> {
                ipEdit.setText(h);
                connect(h);
            });
            LinearLayout.LayoutParams p = new LinearLayout.LayoutParams(ViewGroup.LayoutParams.WRAP_CONTENT, dp(40));
            p.rightMargin = dp(6);
            recentRow.addView(b, p);
        }
    }

    // ───────────────────────── 로봇 찾기 ─────────────────────────

    private void startDiscovery() {
        if (discovering) return;
        discovering = true;
        findBtn.setEnabled(false);
        findBtn.setText("찾는 중… (2 초)");
        foundList.removeAllViews();
        final Context app = getApplicationContext();
        new Thread(() -> {
            List<DiscoveryCore.Robot> found = new ArrayList<>();
            String err = null;
            WifiManager.MulticastLock lock = null;
            try {
                WifiManager wm = (WifiManager) app.getSystemService(Context.WIFI_SERVICE);
                if (wm != null) {
                    lock = wm.createMulticastLock("armpi-discovery");
                    lock.setReferenceCounted(false);
                    lock.acquire();
                }
                found = DiscoveryCore.discover(DiscoveryCore.defaultTargets(), DiscoveryCore.DISCOVERY_PORT, 2000);
            } catch (Exception e) {
                err = e.getClass().getSimpleName() + ": " + e.getMessage();
            } finally {
                if (lock != null && lock.isHeld()) lock.release();
            }
            final List<DiscoveryCore.Robot> result = found;
            final String error = err;
            ui.post(() -> showFound(result, error));
        }, "armpi-discovery").start();
    }

    private void showFound(List<DiscoveryCore.Robot> robots, String error) {
        discovering = false;
        findBtn.setEnabled(true);
        findBtn.setText("로봇 다시 찾기");
        foundList.removeAllViews();
        if (robots.isEmpty()) {
            TextView t = text("로봇을 찾지 못했습니다. 같은 와이파이인지 확인하거나 오른쪽에 IP 를 입력하세요."
                    + (error != null ? "\n(" + error + ")" : ""), 14, BAD, false);
            foundList.addView(t);
            return;
        }
        for (final DiscoveryCore.Robot r : robots) {
            Button b = button(r.name + "   " + r.address + "  → 연결", false);
            b.setGravity(Gravity.START | Gravity.CENTER_VERTICAL);
            b.setOnClickListener(v -> {
                ipEdit.setText(r.address);
                connect(r.address);
            });
            foundList.addView(b, lp(-1, dp(52), 0, 6));
        }
        if (robots.size() == 1 && web == null && status.getText().length() == 0) {
            setStatus("로봇 1 대를 찾았습니다. 눌러서 연결하세요.", OK);
        }
    }

    // ───────────────────────── 연결 ─────────────────────────

    private void connect(String input) {
        String host = input == null ? "" : input.trim();
        host = host.replaceFirst("^[hH][tT][tT][pP][sS]?://", "");
        int slash = host.indexOf('/');
        if (slash >= 0) host = host.substring(0, slash);
        int port = WEB_PORT;
        String name = host;
        if (name.matches("^[^:\\[\\]]+:[0-9]{1,5}$")) {
            int c = name.lastIndexOf(':');
            port = Integer.parseInt(name.substring(c + 1));
            name = name.substring(0, c);
        }
        if (!name.matches("^[A-Za-z0-9.-]{1,253}$") || port < 1 || port > 65535) {
            setStatus("주소 형식이 틀렸습니다: " + input + " (예: 192.168.0.50)", BAD);
            return;
        }
        hideKeyboard();
        remember(port == WEB_PORT ? name : name + ":" + port);
        final String h = name;
        final int p = port;
        final String base = "http://" + h + ":" + p;
        final int seq = ++connectSeq;
        connectBtn.setEnabled(false);
        setStatus(h + ":" + p + " 에 연결하는 중…", DIM);
        new Thread(() -> {
            String err = null;
            try {
                HttpURLConnection c = (HttpURLConnection) new URL(base + "/api/config").openConnection();
                c.setConnectTimeout(2500);
                c.setReadTimeout(2500);
                c.setUseCaches(false);
                int code = c.getResponseCode();
                c.disconnect();
                if (code != 200) err = "HTTP " + code;
            } catch (Exception e) {
                err = e.getClass().getSimpleName() + (e.getMessage() != null ? ": " + e.getMessage() : "");
            }
            final String error = err;
            ui.post(() -> {
                if (seq != connectSeq) return;
                connectBtn.setEnabled(true);
                if (error == null) {
                    setStatus("", DIM);
                    openWeb(h, p);
                } else {
                    setStatus("연결 실패 (" + h + ":" + p + ") — " + error + "\n확인: ① 휴대폰과 로봇이 같은 와이파이인지 "
                            + "② 로봇에서 웹 조종기(armpi-web 서비스)가 켜져 있는지 ③ IP 주소가 맞는지", BAD);
                }
            });
        }, "armpi-connect").start();
    }

    private void hideKeyboard() {
        InputMethodManager imm = (InputMethodManager) getSystemService(Context.INPUT_METHOD_SERVICE);
        if (imm != null) imm.hideSoftInputFromWindow(ipEdit.getWindowToken(), 0);
        ipEdit.clearFocus();
    }

    private void openWeb(String host, int port) {
        if (web != null) closeWeb(null, DIM);
        final AppUiPolicy policy;
        try {
            policy = new AppUiPolicy(host, port);
        } catch (IllegalArgumentException e) {
            setStatus("주소 오류: " + e.getMessage(), BAD);
            return;
        }
        final WebView w = new WebView(this);
        WebSettings s = w.getSettings();
        s.setJavaScriptEnabled(true);
        s.setDomStorageEnabled(true);
        s.setMediaPlaybackRequiresUserGesture(false);
        s.setCacheMode(WebSettings.LOAD_NO_CACHE);
        s.setSupportZoom(false);
        s.setBuiltInZoomControls(false);
        s.setDisplayZoomControls(false);
        s.setAllowFileAccess(false);        // 조종 화면은 file:// 이 아니라 아래 shouldInterceptRequest 로 준다
        s.setAllowContentAccess(false);
        s.setUserAgentString(s.getUserAgentString() + UA_MARK);
        w.setBackgroundColor(BG);
        w.setOverScrollMode(View.OVER_SCROLL_NEVER);
        w.setWebChromeClient(new WebChromeClient());
        w.setWebViewClient(new WebViewClient() {
            @Override
            public boolean shouldOverrideUrlLoading(WebView v, WebResourceRequest r) {
                Uri u = r.getUrl();
                return !policy.navigationAllowed(u.getScheme(), u.getHost(), u.getPort(), u.getPath());  // 앱 안 조종 화면만
            }

            @Override
            public WebResourceResponse shouldInterceptRequest(WebView v, WebResourceRequest r) {
                Uri u = r.getUrl();
                AppUiPolicy.Decision d = policy.decide(u.getScheme(), u.getHost(), u.getPort(), u.getPath());
                if (d.kind == AppUiPolicy.Kind.NETWORK) return null;          // 로봇 API·카메라는 그대로
                Map<String, String> h = new HashMap<>();
                h.put("Cache-Control", "no-store");
                if (d.kind == AppUiPolicy.Kind.ASSET) {
                    try {
                        InputStream in = getAssets().open(d.asset);
                        return new WebResourceResponse(d.mime, "utf-8", 200, "OK", h, in);
                    } catch (Exception e) {
                        return new WebResourceResponse("text/plain", "utf-8", 404, "Not Found", h,
                                new ByteArrayInputStream(new byte[0]));
                    }
                }
                return new WebResourceResponse("text/plain", "utf-8", 403, "Forbidden", h,   // 허용 밖 주소는 막는다
                        new ByteArrayInputStream(new byte[0]));
            }

            @Override
            public void onReceivedError(WebView v, WebResourceRequest r, WebResourceError e) {
                if (r.isForMainFrame() && v == web) {
                    closeWeb("조종 화면을 열지 못했습니다: " + e.getDescription(), BAD);
                }
            }

            @Override
            public void onReceivedHttpError(WebView v, WebResourceRequest r, WebResourceResponse resp) {
                if (r.isForMainFrame() && v == web) {
                    closeWeb("조종 화면 응답 오류: HTTP " + resp.getStatusCode(), BAD);
                }
            }
        });
        web = w;
        root.addView(w, new FrameLayout.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT, ViewGroup.LayoutParams.MATCH_PARENT));
        home.setVisibility(View.GONE);
        setFullscreenUi(true);
        w.loadUrl(policy.pageUrl());
        w.requestFocus();
    }

    /** 조종 화면을 닫고 첫 화면으로. 닫기 전에 웹 화면이 중립과 조종권 놓기를 보내게 한다. */
    private void closeWeb(String message, int color) {
        final WebView w = web;
        web = null;
        if (w != null) {
            w.evaluateJavascript("window.armpiNeutral && window.armpiNeutral(true)", null);
            w.setVisibility(View.GONE);
            ui.postDelayed(() -> {
                root.removeView(w);
                w.stopLoading();
                w.loadUrl("about:blank");
                w.destroy();
            }, 300);
        }
        home.setVisibility(View.VISIBLE);
        setFullscreenUi(false);
        if (message != null) setStatus(message, color);
    }

    @Override
    @SuppressWarnings("deprecation")
    public void onBackPressed() {
        if (web != null) {
            closeWeb("연결을 끊고 첫 화면으로 돌아왔습니다.", DIM);
        } else {
            super.onBackPressed();
        }
    }

    // ───────────────────────── 수명 주기 ─────────────────────────

    @Override
    protected void onPause() {
        paused = true;
        final WebView w = web;
        if (w != null) {
            // 백그라운드로 가면 즉시 중립을 보내고, 잠시 뒤 JS 타이머를 멈춘다(입력이 끊기면 서버가 0.3 s 뒤 중립,
            // 1.5 s 뒤 조종권을 놓아 다른 기기가 쓸 수 있게 된다).
            w.evaluateJavascript("window.armpiNeutral && window.armpiNeutral(false)", null);
            ui.postDelayed(() -> {
                if (paused && w == web) {
                    w.onPause();
                    w.pauseTimers();
                }
            }, 150);
        }
        super.onPause();
    }

    @Override
    protected void onResume() {
        super.onResume();
        paused = false;
        if (web != null) {
            web.resumeTimers();
            web.onResume();
            setFullscreenUi(true);
        }
    }

    @Override
    public void onWindowFocusChanged(boolean hasFocus) {
        super.onWindowFocusChanged(hasFocus);
        if (hasFocus && web != null) setFullscreenUi(true);
    }

    @Override
    protected void onDestroy() {
        if (web != null) {
            web.destroy();
            web = null;
        }
        super.onDestroy();
    }

    @SuppressWarnings("deprecation")
    private void setFullscreenUi(boolean on) {
        if (Build.VERSION.SDK_INT >= 30) {
            getWindow().setDecorFitsSystemWindows(!on);
            WindowInsetsController c = getWindow().getInsetsController();
            if (c != null) {
                if (on) {
                    c.hide(WindowInsets.Type.systemBars());
                    c.setSystemBarsBehavior(WindowInsetsController.BEHAVIOR_SHOW_TRANSIENT_BARS_BY_SWIPE);
                } else {
                    c.show(WindowInsets.Type.systemBars());
                }
            }
        } else {
            View d = getWindow().getDecorView();
            d.setSystemUiVisibility(on ? (View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY | View.SYSTEM_UI_FLAG_FULLSCREEN
                    | View.SYSTEM_UI_FLAG_HIDE_NAVIGATION | View.SYSTEM_UI_FLAG_LAYOUT_STABLE
                    | View.SYSTEM_UI_FLAG_LAYOUT_HIDE_NAVIGATION | View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN) : 0);
        }
    }
}
