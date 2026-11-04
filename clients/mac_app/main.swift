// ArmPi Controller — 맥 앱 껍데기 (AppKit + WKWebView, Xcode 없이 swiftc 로 빌드)
//
// 하는 일
//  1. 첫 화면에서 로봇 주소를 받는다(기본 auto = UDP 5002 검색, 최근 값 기억).
//  2. 앱 번들 Contents/Resources/mac_controller/armpi_mac_controller.py 를 /usr/bin/python3 로 실행한다
//     (--robot <주소> --http-port <18765부터 빈 포트> --no-browser --exit-with-stdin).
//  3. 서버가 응답하면 WKWebView 로 http://127.0.0.1:<포트>/ 를 연다. 키 입력이 웹 화면으로 가도록 포커스를 준다.
//  4. 끝낼 때 파이썬에 SIGINT 를 보내 중립·disconnect 를 보내고 끝나게 한다(3 s 안에 안 끝나면 SIGTERM → SIGKILL).
//     앱이 비정상 종료돼도 표준 입력 파이프가 닫혀 파이썬이 스스로 정리한다(--exit-with-stdin).
//
// 시험용: open "ArmPi Controller.app" --args --robot 127.0.0.1   (첫 화면을 건너뛰고 바로 연결)
//         kill -TERM <앱 pid> 는 ⌘Q 와 같은 정상 종료로 처리한다.

import AppKit
import WebKit

let kFirstPort = 18765            // 이 맥의 8765 는 다른 프로그램이 쓰고 있어 18765 부터
let kRecentKey = "recentRobots"
let kLastKey = "lastRobot"
let kMaxRecent = 6

final class AppDelegate: NSObject, NSApplicationDelegate, NSWindowDelegate, WKNavigationDelegate, NSComboBoxDelegate {
    var window: NSWindow!
    var startView: NSView!
    var addrBox: NSComboBox!
    var connectButton: NSButton!
    var statusLabel: NSTextField!
    var spinner: NSProgressIndicator!
    var webView: WKWebView?

    var server: Process?
    var serverInput: Pipe?
    var expectingExit = false
    var terminating = false
    var port = kFirstPort
    var robot = "auto"
    var logTail: [String] = []
    var logHandle: FileHandle?
    var pollTimer: Timer?
    var pollDeadline = Date()

    // ── 시작 ──

    func applicationDidFinishLaunching(_ note: Notification) {
        buildMenu()
        buildWindow()
        NSApp.activate(ignoringOtherApps: true)
        let args = CommandLine.arguments
        if let i = args.firstIndex(of: "--robot"), i + 1 < args.count {
            addrBox.stringValue = args[i + 1]
            connect(nil)
        }
    }

    func applicationShouldTerminateAfterLastWindowClosed(_ sender: NSApplication) -> Bool { true }

    func applicationShouldTerminate(_ sender: NSApplication) -> NSApplication.TerminateReply {
        guard let p = server, p.isRunning else { return .terminateNow }
        terminating = true
        expectingExit = true
        setStatus("종료 중: 로봇에 중립·disconnect 를 보내는 중…", error: false)
        stopProcess(p) {
            NSApp.reply(toApplicationShouldTerminate: true)
        }
        return .terminateLater
    }

    // ── 창·첫 화면 ──

    func buildWindow() {
        window = NSWindow(contentRect: NSRect(x: 0, y: 0, width: 1280, height: 840),
                          styleMask: [.titled, .closable, .miniaturizable, .resizable], backing: .buffered, defer: false)
        window.title = "ArmPi Controller"
        window.minSize = NSSize(width: 760, height: 520)
        window.delegate = self
        window.center()
        window.setFrameAutosaveName("ArmPiControllerMain")

        let v = NSView()
        let title = NSTextField(labelWithString: "ArmPi Controller")
        title.font = .boldSystemFont(ofSize: 26)
        let sub = NSTextField(wrappingLabelWithString:
            "로봇 주소를 넣고 연결하세요. auto = 같은 와이파이에서 자동 검색(UDP 5002).\n"
            + "가짜 로봇으로 시험할 때는 127.0.0.1 (터미널에서 protocol_v3/mock_robot.py --bind 127.0.0.1 실행).")
        sub.textColor = .secondaryLabelColor
        sub.alignment = .center
        sub.preferredMaxLayoutWidth = 520

        addrBox = NSComboBox()
        addrBox.placeholderString = "auto 또는 192.168.x.x"
        addrBox.font = .systemFont(ofSize: 15)
        addrBox.completes = true
        addrBox.numberOfVisibleItems = 8
        addrBox.target = self
        addrBox.action = #selector(connect(_:))
        let recent = recentRobots()
        addrBox.addItems(withObjectValues: recent)
        addrBox.stringValue = UserDefaults.standard.string(forKey: kLastKey) ?? "auto"
        addrBox.translatesAutoresizingMaskIntoConstraints = false
        addrBox.widthAnchor.constraint(equalToConstant: 280).isActive = true

        connectButton = NSButton(title: "연결", target: self, action: #selector(connect(_:)))
        connectButton.keyEquivalent = "\r"
        connectButton.bezelStyle = .rounded
        let row = NSStackView(views: [addrBox, connectButton])
        row.orientation = .horizontal
        row.spacing = 8

        spinner = NSProgressIndicator()
        spinner.style = .spinning
        spinner.controlSize = .small
        spinner.isDisplayedWhenStopped = false

        statusLabel = NSTextField(wrappingLabelWithString: "")
        statusLabel.alignment = .center
        statusLabel.preferredMaxLayoutWidth = 560
        statusLabel.isSelectable = true

        let foot = NSTextField(wrappingLabelWithString:
            "키: W/S 앞뒤 · A/D 옆으로 · Q/E 회전 · Space 비상정지 · 팔 I/K J/L U/O R/F G/H\n"
            + "끝낼 때는 ⌘Q — 조종기가 중립·disconnect 를 보내고 끝납니다. 처음에는 바퀴를 띄우고 시험하세요.")
        foot.textColor = .tertiaryLabelColor
        foot.alignment = .center
        foot.font = .systemFont(ofSize: 12)
        foot.preferredMaxLayoutWidth = 560

        let stack = NSStackView(views: [title, sub, row, spinner, statusLabel, foot])
        stack.orientation = .vertical
        stack.alignment = .centerX
        stack.spacing = 14
        stack.setCustomSpacing(26, after: sub)
        stack.setCustomSpacing(30, after: statusLabel)
        stack.translatesAutoresizingMaskIntoConstraints = false
        v.addSubview(stack)
        NSLayoutConstraint.activate([
            stack.centerXAnchor.constraint(equalTo: v.centerXAnchor),
            stack.centerYAnchor.constraint(equalTo: v.centerYAnchor, constant: -20),
            stack.widthAnchor.constraint(lessThanOrEqualTo: v.widthAnchor, constant: -40),
        ])
        startView = v
        window.contentView = v
        window.makeKeyAndOrderFront(nil)
        window.makeFirstResponder(addrBox)
    }

    func showStart(_ message: String?, error: Bool) {
        if let wv = webView {
            wv.stopLoading()
            wv.navigationDelegate = nil
            webView = nil
        }
        window.contentView = startView
        window.title = "ArmPi Controller"
        connectButton.isEnabled = true
        addrBox.isEnabled = true
        spinner.stopAnimation(nil)
        setStatus(message ?? "", error: error)
        window.makeFirstResponder(addrBox)
    }

    func setStatus(_ text: String, error: Bool) {
        statusLabel?.stringValue = text
        statusLabel?.textColor = error ? .systemRed : .secondaryLabelColor
    }

    // ── 메뉴 ──

    func buildMenu() {
        let main = NSMenu()
        let appItem = NSMenuItem()
        main.addItem(appItem)
        let appMenu = NSMenu()
        appMenu.addItem(withTitle: "ArmPi Controller 정보", action: #selector(NSApplication.orderFrontStandardAboutPanel(_:)), keyEquivalent: "")
        appMenu.addItem(.separator())
        appMenu.addItem(withTitle: "ArmPi Controller 가리기", action: #selector(NSApplication.hide(_:)), keyEquivalent: "h")
        appMenu.addItem(.separator())
        appMenu.addItem(withTitle: "ArmPi Controller 종료", action: #selector(NSApplication.terminate(_:)), keyEquivalent: "q")
        appItem.submenu = appMenu

        let editItem = NSMenuItem()
        main.addItem(editItem)
        let edit = NSMenu(title: "편집")
        edit.addItem(withTitle: "실행 취소", action: Selector(("undo:")), keyEquivalent: "z")
        edit.addItem(withTitle: "실행 복귀", action: Selector(("redo:")), keyEquivalent: "Z")
        edit.addItem(.separator())
        edit.addItem(withTitle: "오려두기", action: #selector(NSText.cut(_:)), keyEquivalent: "x")
        edit.addItem(withTitle: "복사하기", action: #selector(NSText.copy(_:)), keyEquivalent: "c")
        edit.addItem(withTitle: "붙여넣기", action: #selector(NSText.paste(_:)), keyEquivalent: "v")
        edit.addItem(withTitle: "전체 선택", action: #selector(NSText.selectAll(_:)), keyEquivalent: "a")
        editItem.submenu = edit

        let robotItem = NSMenuItem()
        main.addItem(robotItem)
        let rm = NSMenu(title: "로봇")
        rm.addItem(withTitle: "다시 연결", action: #selector(reconnect(_:)), keyEquivalent: "r")
        rm.addItem(withTitle: "로봇 주소 바꾸기…", action: #selector(changeRobot(_:)), keyEquivalent: "l")
        rm.addItem(.separator())
        rm.addItem(withTitle: "기본 브라우저에서 열기", action: #selector(openInBrowser(_:)), keyEquivalent: "")
        rm.addItem(withTitle: "조종기 로그 보기", action: #selector(showLog(_:)), keyEquivalent: "")
        robotItem.submenu = rm

        let winItem = NSMenuItem()
        main.addItem(winItem)
        let wm = NSMenu(title: "윈도우")
        wm.addItem(withTitle: "최소화", action: #selector(NSWindow.performMiniaturize(_:)), keyEquivalent: "m")
        let fs = wm.addItem(withTitle: "전체 화면", action: #selector(NSWindow.toggleFullScreen(_:)), keyEquivalent: "f")
        fs.keyEquivalentModifierMask = [.command, .control]
        winItem.submenu = wm
        NSApp.windowsMenu = wm
        NSApp.mainMenu = main
    }

    @objc func reconnect(_ sender: Any?) {
        if server == nil && webView == nil {
            connect(nil)
            return
        }
        startServer(robot)
    }

    @objc func changeRobot(_ sender: Any?) {
        stopServerSync()
        showStart("로봇 주소를 바꾼 뒤 연결을 누르세요.", error: false)
    }

    @objc func openInBrowser(_ sender: Any?) {
        guard server?.isRunning == true, let url = URL(string: "http://127.0.0.1:\(port)/") else { NSSound.beep(); return }
        NSWorkspace.shared.open(url)
    }

    @objc func showLog(_ sender: Any?) {
        NSWorkspace.shared.open(logURL())
    }

    // ── 연결 ──

    @objc func connect(_ sender: Any?) {
        let text = addrBox.stringValue.trimmingCharacters(in: .whitespacesAndNewlines)
        let addr = text.isEmpty ? "auto" : text
        let ok = addr == "auto" || addr.range(of: "^[A-Za-z0-9.:-]{1,253}$", options: .regularExpression) != nil
        guard ok else {
            setStatus("주소 형식이 틀렸습니다: \(addr) — auto 또는 IP(예: 192.168.149.1)", error: true)
            return
        }
        rememberRobot(addr)
        startServer(addr)
    }

    func recentRobots() -> [String] {
        var list = UserDefaults.standard.stringArray(forKey: kRecentKey) ?? []
        for d in ["auto", "127.0.0.1"] where !list.contains(d) { list.append(d) }
        return list
    }

    func rememberRobot(_ addr: String) {
        var list = UserDefaults.standard.stringArray(forKey: kRecentKey) ?? []
        list.removeAll { $0 == addr }
        list.insert(addr, at: 0)
        if list.count > kMaxRecent { list = Array(list.prefix(kMaxRecent)) }
        UserDefaults.standard.set(list, forKey: kRecentKey)
        UserDefaults.standard.set(addr, forKey: kLastKey)
        addrBox.removeAllItems()
        addrBox.addItems(withObjectValues: recentRobots())
    }

    // ── 파이썬 서버 ──

    func logURL() -> URL {
        let dir = FileManager.default.homeDirectoryForCurrentUser.appendingPathComponent("Library/Logs/ArmPi Controller")
        try? FileManager.default.createDirectory(at: dir, withIntermediateDirectories: true)
        return dir.appendingPathComponent("controller.log")
    }

    func appendLog(_ data: Data) {
        guard let text = String(data: data, encoding: .utf8) ?? String(data: data, encoding: .isoLatin1) else { return }
        logHandle?.write(data)
        for line in text.split(separator: "\n", omittingEmptySubsequences: true) {
            logTail.append(String(line))
        }
        if logTail.count > 40 { logTail.removeFirst(logTail.count - 40) }
    }

    func portFree(_ p: Int) -> Bool {
        let fd = Darwin.socket(AF_INET, SOCK_STREAM, 0)
        if fd < 0 { return false }
        defer { Darwin.close(fd) }
        var yes: Int32 = 1
        setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &yes, socklen_t(MemoryLayout<Int32>.size))
        var addr = sockaddr_in()
        addr.sin_len = UInt8(MemoryLayout<sockaddr_in>.size)
        addr.sin_family = sa_family_t(AF_INET)
        addr.sin_port = in_port_t(UInt16(p).bigEndian)
        addr.sin_addr.s_addr = inet_addr("127.0.0.1")
        let r = withUnsafePointer(to: &addr) {
            $0.withMemoryRebound(to: sockaddr.self, capacity: 1) { Darwin.bind(fd, $0, socklen_t(MemoryLayout<sockaddr_in>.size)) }
        }
        return r == 0
    }

    func startServer(_ addr: String) {
        stopServerSync()
        robot = addr
        guard let res = Bundle.main.resourceURL else { return }
        let script = res.appendingPathComponent("mac_controller/armpi_mac_controller.py")
        guard FileManager.default.fileExists(atPath: script.path) else {
            showStart("앱 안에 조종기 파일이 없습니다: \(script.path)", error: true)
            return
        }
        guard FileManager.default.isExecutableFile(atPath: "/usr/bin/python3") else {
            showStart("/usr/bin/python3 가 없습니다. 터미널에서 xcode-select --install 로 명령줄 도구를 설치하세요.", error: true)
            return
        }
        port = (kFirstPort..<(kFirstPort + 20)).first(where: portFree) ?? kFirstPort

        let fm = FileManager.default
        let url = logURL()
        if !fm.fileExists(atPath: url.path) { fm.createFile(atPath: url.path, contents: nil) }
        logHandle = try? FileHandle(forWritingTo: url)
        logHandle?.seekToEndOfFile()
        let stamp = ISO8601DateFormatter().string(from: Date())
        logHandle?.write("\n===== \(stamp) 시작: 로봇 \(addr), 포트 \(port) =====\n".data(using: .utf8)!)
        logTail = []

        let p = Process()
        p.executableURL = URL(fileURLWithPath: "/usr/bin/python3")
        // -I: 사용자 환경 변수·site 무시, -B: 서명된 앱 번들 안에 .pyc 를 쓰지 않음, -u: 로그 즉시 출력
        p.arguments = ["-I", "-B", "-u", script.path, "--robot", addr, "--http-port", String(port),
                       "--no-browser", "--exit-with-stdin"]
        p.currentDirectoryURL = res.appendingPathComponent("mac_controller")
        var env = ProcessInfo.processInfo.environment
        env["PYTHONDONTWRITEBYTECODE"] = "1"
        env["PYTHONUNBUFFERED"] = "1"
        p.environment = env
        let inPipe = Pipe()
        let outPipe = Pipe()
        p.standardInput = inPipe
        p.standardOutput = outPipe
        p.standardError = outPipe
        outPipe.fileHandleForReading.readabilityHandler = { [weak self] h in
            let data = h.availableData
            if data.isEmpty { h.readabilityHandler = nil; return }
            DispatchQueue.main.async { self?.appendLog(data) }
        }
        p.terminationHandler = { [weak self] proc in
            DispatchQueue.main.async { self?.serverExited(proc) }
        }
        do {
            try p.run()
        } catch {
            showStart("조종기 서버를 시작하지 못했습니다: \(error.localizedDescription)", error: true)
            return
        }
        server = p
        serverInput = inPipe
        expectingExit = false
        connectButton.isEnabled = false
        addrBox.isEnabled = false
        spinner.startAnimation(nil)
        setStatus(addr == "auto" ? "로봇을 찾는 중(UDP 5002)…" : "조종기 서버를 켜는 중(로봇 \(addr))…", error: false)
        pollDeadline = Date().addingTimeInterval(20)
        pollTimer?.invalidate()
        pollTimer = Timer.scheduledTimer(withTimeInterval: 0.25, repeats: true) { [weak self] _ in self?.pollServer() }
    }

    func pollServer() {
        guard let p = server, p.isRunning else { pollTimer?.invalidate(); return }
        if Date() > pollDeadline {
            pollTimer?.invalidate()
            stopServerSync()
            showStart("조종기 서버가 20 s 안에 응답하지 않았습니다.\n" + logTail.suffix(4).joined(separator: "\n"), error: true)
            return
        }
        guard let url = URL(string: "http://127.0.0.1:\(port)/api/config") else { return }
        var req = URLRequest(url: url, cachePolicy: .reloadIgnoringLocalCacheData, timeoutInterval: 1.0)
        req.httpMethod = "GET"
        let expected = p
        URLSession.shared.dataTask(with: req) { [weak self] _, resp, _ in
            guard (resp as? HTTPURLResponse)?.statusCode == 200 else { return }
            DispatchQueue.main.async {
                guard let self = self, self.server === expected, self.webView == nil else { return }
                self.pollTimer?.invalidate()
                self.showWeb()
            }
        }.resume()
    }

    func showWeb() {
        let cfg = WKWebViewConfiguration()
        cfg.applicationNameForUserAgent = "ArmPiController-Mac/1.0"
        cfg.websiteDataStore = .nonPersistent()
        let wv = WKWebView(frame: window.contentLayoutRect, configuration: cfg)
        wv.navigationDelegate = self
        wv.autoresizingMask = [.width, .height]
        if #available(macOS 13.3, *) { wv.isInspectable = true }
        webView = wv
        window.contentView = wv
        window.title = "ArmPi Controller — 로봇 \(robot) (화면 포트 \(port))"
        spinner.stopAnimation(nil)
        if let url = URL(string: "http://127.0.0.1:\(port)/") {
            wv.load(URLRequest(url: url, cachePolicy: .reloadIgnoringLocalCacheData, timeoutInterval: 10))
        }
        window.makeFirstResponder(wv)
    }

    func serverExited(_ proc: Process) {
        guard proc === server else { return }
        let code = proc.terminationStatus
        server = nil
        serverInput = nil
        pollTimer?.invalidate()
        logHandle?.write("===== 종료 코드 \(code) =====\n".data(using: .utf8)!)
        try? logHandle?.close()
        logHandle = nil
        if terminating || expectingExit { return }
        var msg = "조종기 서버가 끝났습니다(코드 \(code))."
        if logTail.contains(where: { $0.contains("로봇을 찾지 못했습니다") }) {
            msg = "로봇을 찾지 못했습니다(UDP 5002 검색).\n같은 와이파이인지 확인하거나 로봇 IP 를 직접 넣으세요(대체 AP 에서는 192.168.149.1)."
        } else if logTail.contains(where: { $0.contains("열기 실패") }) {
            msg = "조종 화면 포트를 열지 못했습니다. 다른 조종기가 켜져 있는지 확인하세요."
        }
        let tail = logTail.suffix(3).joined(separator: "\n")
        showStart(msg + (tail.isEmpty ? "" : "\n\n" + tail), error: true)
    }

    /// SIGINT → (3 s) SIGTERM → (1 s) SIGKILL. 끝나면 done 을 메인 스레드에서 부른다.
    func stopProcess(_ p: Process, done: @escaping () -> Void) {
        p.interrupt()
        let input = serverInput
        DispatchQueue.global().async {
            let t0 = Date()
            while p.isRunning && Date().timeIntervalSince(t0) < 3.0 { usleep(50_000) }
            if p.isRunning { p.terminate() }
            let t1 = Date()
            while p.isRunning && Date().timeIntervalSince(t1) < 1.0 { usleep(50_000) }
            if p.isRunning { kill(p.processIdentifier, SIGKILL) }
            try? input?.fileHandleForWriting.close()
            // terminateLater 동안 앱은 modal panel 모드로 run loop 를 돈다. 그 모드에서도 실행되도록 run loop 에 직접 넣는다.
            RunLoop.main.perform(inModes: [.common, .modalPanel]) { done() }
        }
    }

    func stopServerSync() {
        pollTimer?.invalidate()
        guard let p = server else { return }
        expectingExit = true
        if p.isRunning {
            p.interrupt()
            let t0 = Date()
            while p.isRunning && Date().timeIntervalSince(t0) < 3.0 { usleep(50_000) }
            if p.isRunning { p.terminate(); usleep(500_000) }
            if p.isRunning { kill(p.processIdentifier, SIGKILL) }
        }
        try? serverInput?.fileHandleForWriting.close()
        server = nil
        serverInput = nil
    }

    // ── 웹 화면 ──

    func webView(_ webView: WKWebView, didFinish navigation: WKNavigation!) {
        window.makeFirstResponder(webView)
    }

    func webView(_ webView: WKWebView, didFailProvisionalNavigation navigation: WKNavigation!, withError error: Error) {
        if server?.isRunning == true {
            DispatchQueue.main.asyncAfter(deadline: .now() + 1.0) { [weak webView] in webView?.reload() }
        }
    }

    func webView(_ webView: WKWebView, decidePolicyFor action: WKNavigationAction,
                 decisionHandler: @escaping (WKNavigationActionPolicy) -> Void) {
        // 조종 화면(127.0.0.1)만 앱 안에서 연다. 다른 주소는 기본 브라우저로.
        if let url = action.request.url, url.host != "127.0.0.1", url.scheme == "http" || url.scheme == "https",
           action.targetFrame?.isMainFrame ?? true {
            NSWorkspace.shared.open(url)
            decisionHandler(.cancel)
            return
        }
        decisionHandler(.allow)
    }

    func windowDidBecomeKey(_ notification: Notification) {
        if let wv = webView { window.makeFirstResponder(wv) }
    }
}

let app = NSApplication.shared
app.setActivationPolicy(.regular)
let delegate = AppDelegate()
app.delegate = delegate
// SIGTERM(kill, 로그아웃 스크립트 등)도 ⌘Q 와 같은 정상 종료 경로(파이썬에 SIGINT → 중립·disconnect)로 보낸다.
signal(SIGTERM, SIG_IGN)
let termSource = DispatchSource.makeSignalSource(signal: SIGTERM, queue: .main)
// GCD 블록 안에서 바로 terminate 하면 종료 대기(terminateLater) 중 메인 큐가 막히므로 run loop 로 넘긴다.
termSource.setEventHandler { NSApp.perform(#selector(NSApplication.terminate(_:)), with: nil, afterDelay: 0) }
termSource.resume()
app.run()
