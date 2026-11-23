package org.firstinspires.ftc.teamcode.util;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.net.ServerSocket;
import java.net.Socket;
import java.nio.charset.StandardCharsets;
import java.util.HashSet;
import java.util.Locale;
import java.util.Set;
import java.util.function.Supplier;

/**
 * Tiny web page on the robot that sends keyboard state from a laptop browser, for driving without a
 * gamepad. Open http://192.168.43.1:{@value #PORT} while connected to the robot's Wi-Fi.
 * <p>
 * The page posts the set of held keys ~25 times a second. If nothing arrives for
 * {@link #TIMEOUT_MS} (tab closed, Wi-Fi dropped) every key reads as released, so the robot stops.
 * One server is started per app run and reused by every OpMode run.
 */
public final class KeyboardServer {

    public static final int PORT = 8099;
    public static final long TIMEOUT_MS = 300;

    private static KeyboardServer instance;

    private volatile Set<String> keys = new HashSet<>();
    private volatile long lastUpdateMs = 0;
    private volatile Supplier<String> status = () -> "";

    /** Starts the server the first time, then returns the running one. */
    public static synchronized KeyboardServer get() {
        if (instance == null) {
            instance = new KeyboardServer();
            Thread t = new Thread(instance::serve, "KeyboardServer");
            t.setDaemon(true);
            t.start();
        }
        return instance;
    }

    /** True while {@code key} (lower-case, e.g. "w", "arrowleft", "shift") is held. */
    public boolean isDown(String key) {
        return isConnected() && keys.contains(key);
    }

    /** True if the browser page has sent keys recently. */
    public boolean isConnected() {
        return System.currentTimeMillis() - lastUpdateMs < TIMEOUT_MS;
    }

    /** Text shown under the controls on the page (turret angle, pose, ...). */
    public void setStatus(Supplier<String> status) {
        this.status = status;
    }

    private void serve() {
        try (ServerSocket server = new ServerSocket()) {
            server.setReuseAddress(true);
            server.bind(new java.net.InetSocketAddress(PORT));
            while (true) {
                try (Socket socket = server.accept()) {
                    socket.setSoTimeout(1000);
                    handle(socket);
                } catch (Exception ignored) {
                    // One bad request must not kill the server.
                }
            }
        } catch (Exception e) {
            instance = null; // couldn't bind; try again next OpMode
        }
    }

    private void handle(Socket socket) throws Exception {
        BufferedReader in = new BufferedReader(new InputStreamReader(socket.getInputStream(), StandardCharsets.UTF_8));
        String requestLine = in.readLine();
        if (requestLine == null) return;
        String path = requestLine.split(" ")[1];

        String body;
        String type;
        if (path.startsWith("/keys")) {
            Set<String> held = new HashSet<>();
            int q = path.indexOf("k=");
            if (q >= 0) {
                for (String k : java.net.URLDecoder.decode(path.substring(q + 2), "UTF-8").split(",")) {
                    if (!k.isEmpty()) held.add(k.toLowerCase(Locale.ROOT));
                }
            }
            keys = held;
            lastUpdateMs = System.currentTimeMillis();
            body = status.get();
            type = "text/plain; charset=utf-8";
        } else {
            body = PAGE;
            type = "text/html; charset=utf-8";
        }

        byte[] bytes = body.getBytes(StandardCharsets.UTF_8);
        OutputStream out = socket.getOutputStream();
        out.write(("HTTP/1.1 200 OK\r\nContent-Type: " + type + "\r\nContent-Length: " + bytes.length
                + "\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n").getBytes(StandardCharsets.UTF_8));
        out.write(bytes);
        out.flush();
    }

    private static final String PAGE = "<!doctype html><html><head><meta charset='utf-8'>"
            + "<meta name='viewport' content='width=device-width,initial-scale=1'><title>Robot Keyboard</title>"
            + "<style>body{font:16px system-ui;background:#111;color:#eee;margin:24px}"
            + "kbd{display:inline-block;min-width:1.6em;padding:4px 8px;margin:2px;border:1px solid #555;"
            + "border-radius:6px;background:#222;text-align:center}kbd.on{background:#2e7d32;border-color:#4caf50}"
            + "#conn{font-weight:bold}pre{background:#1b1b1b;padding:12px;border-radius:8px;white-space:pre-wrap}</style>"
            + "</head><body><h2>Robot Keyboard</h2><p id='conn'>connecting…</p>"
            + "<p><kbd id='k-w'>W</kbd><kbd id='k-a'>A</kbd><kbd id='k-s'>S</kbd><kbd id='k-d'>D</kbd> drive &nbsp; "
            + "<kbd id='k-j'>J</kbd><kbd id='k-l'>L</kbd> / <kbd id='k-arrowleft'>&larr;</kbd><kbd id='k-arrowright'>&rarr;</kbd> rotate &nbsp; "
            + "<kbd id='k-shift'>Shift</kbd> fast</p>"
            + "<p><kbd id='k-y'>Y</kbd> turret: aim at center / forward &nbsp; <kbd id='k-r'>R</kbd> reset pose (robot back at start)</p>"
            + "<p>Keep this tab focused. Release all keys or close the tab and the robot stops.</p>"
            + "<pre id='st'></pre><script>"
            + "const held=new Set();const ok=['w','a','s','d','j','l','arrowleft','arrowright','arrowup','arrowdown','shift','y','r'];"
            + "function key(e){return e.key.toLowerCase()}"
            + "addEventListener('keydown',e=>{const k=key(e);if(ok.includes(k)){held.add(k);e.preventDefault();draw()}});"
            + "addEventListener('keyup',e=>{held.delete(key(e));draw()});"
            + "addEventListener('blur',()=>{held.clear();draw()});"
            + "function draw(){document.querySelectorAll('kbd').forEach(el=>el.classList.toggle('on',held.has(el.id.slice(2))))}"
            + "async function tick(){try{const r=await fetch('/keys?k='+encodeURIComponent([...held].join(',')),{cache:'no-store'});"
            + "document.getElementById('st').textContent=await r.text();"
            + "const c=document.getElementById('conn');c.textContent='connected';c.style.color='#4caf50'}"
            + "catch(e){const c=document.getElementById('conn');c.textContent='not connected to robot';c.style.color='#f44336'}"
            + "setTimeout(tick,40)}tick();"
            + "</script></body></html>";
}
