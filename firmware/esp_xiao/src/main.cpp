// ─────────────────────────────────────────────────────────────────────────────
// holOS — ESP32-S3 Xiao WiFi telemetry bridge
//
// Transparent byte-for-byte relay between:
//   • Teensy 4.1 BRIDGE_XBEE (Serial2 @ 57600)  ↔  ESP Xiao Serial1 (D6/D7)
//   • Jetson / PC  ↔  TCP socket on the home router (port TCP_PORT)
//
// The Teensy doesn't know it's no longer talking to an XBee module — the
// existing CRC8-framed protocol (ping/pong, request/reply, telemetry) flows
// through unchanged. Bridge selection happens entirely on the Jetson side:
// the host opens a TCP socket to this Xiao instead of pyserial on the XBee
// dongle.
//
// Single TCP client at a time. If a second client connects, it replaces the
// previous one (covers Jetson crash / WiFi blip without a dangling socket).
//
// Onboard USER_LED (GPIO21, active LOW):
//   error      (WiFi disconnected) — 3 fast blinks + 1 s pause, repeat
//   connected  (WiFi up)           — slow heartbeat (50 ms / 1950 ms)
//   The TCP client state is shown on the /debug page; the LED reflects
//   WiFi only because that's the failure mode you actually need to spot
//   visually (router down vs router up).
//
// Watchdog: if WiFi fails to associate for 60 s on first boot, ESP.restart().
// After that, reboot every 30 s while still down — covers the case where the
// router boots alongside the robot.
// ─────────────────────────────────────────────────────────────────────────────

#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WebServer.h>

#include "config.h"

// ── Globals ──────────────────────────────────────────────────────────────────

static WiFiServer  tcpServer(TCP_PORT);
static WiFiClient  tcpClient;
static WebServer   httpServer(80);

// Lock-free byte rings — single producer / single consumer per direction,
// fed from loop() so no concurrency to worry about.
static uint8_t  rxRing[RX_RING_BYTES];   // UART → TCP queue
static uint16_t rxHead = 0, rxTail = 0;
static uint8_t  txRing[TX_RING_BYTES];   // TCP  → UART queue
static uint16_t txHead = 0, txTail = 0;

// WiFi watchdog timestamps
static uint32_t wifiAttemptStartMs = 0;
static bool     wifiFirstBoot      = true;

// ── Stats ────────────────────────────────────────────────────────────────────
// Plain monotonic counters bumped on every byte / event. Read by the debug
// HTTP page on port 80. Wraparound on 32-bit is fine for diagnostic use
// (4 GB at 57600 baud ~= 4 hours).
static uint32_t statBytesRx        = 0;   // UART → TCP
static uint32_t statBytesTx        = 0;   // TCP  → UART
static uint32_t statRxOverflows    = 0;
static uint32_t statTxOverflows    = 0;
static uint32_t statClientConnects = 0;
static uint32_t statClientReplaced = 0;
static uint32_t statWifiReboots    = 0;
static uint32_t statLastClientMs   = 0;

// ── Tiny ring helpers ────────────────────────────────────────────────────────

static inline bool ringPush(uint8_t* ring, uint16_t cap,
                            uint16_t& head, uint16_t tail, uint8_t b) {
    uint16_t next = (uint16_t)((head + 1) % cap);
    if (next == tail) return false;        // full — drop byte
    ring[head] = b;
    head = next;
    return true;
}

static inline bool ringPop(uint8_t* ring, uint16_t cap,
                           uint16_t head, uint16_t& tail, uint8_t& out) {
    if (head == tail) return false;        // empty
    out = ring[tail];
    tail = (uint16_t)((tail + 1) % cap);
    return true;
}

// ── LED state machine (single-LED patterns, non-blocking) ───────────────────

enum LedState : uint8_t { LED_ERROR, LED_CONNECTING, LED_CONNECTED };

static inline void ledWrite(bool on) {
#if STATUS_LED_ACTIVE_LOW
    digitalWrite(STATUS_LED_PIN, on ? LOW : HIGH);
#else
    digitalWrite(STATUS_LED_PIN, on ? HIGH : LOW);
#endif
}

// Drive the LED based on a derived state, non-blocking. The patterns are
// implemented as plain millis() phase math — no FreeRTOS task needed, the
// relay loop runs often enough to keep the timing visually consistent.
static void ledTick(LedState s) {
    const uint32_t t = millis();
    switch (s) {
        case LED_ERROR: {
            // 3 short blinks + 1 s pause. Cycle = 3 × (100 on + 100 off) + 1000.
            const uint32_t cycle = 3 * (100 + 100) + 1000;
            uint32_t p = t % cycle;
            if (p < 600) {
                uint32_t k = p % 200;
                ledWrite(k < 100);
            } else {
                ledWrite(false);
            }
            break;
        }
        case LED_CONNECTING: {
            // Rapid on/off — 100 ms / 100 ms.
            ledWrite(((t / 100) & 1) == 0);
            break;
        }
        case LED_CONNECTED: {
            // Slow heartbeat — brief 50 ms blip every 2 s.
            ledWrite((t % 2000) < 50);
            break;
        }
    }
}

// ── WiFi ─────────────────────────────────────────────────────────────────────

static void wifiBegin() {
    WiFi.mode(WIFI_STA);
    WiFi.setHostname(WIFI_HOSTNAME);
    WiFi.setAutoReconnect(true);
    WiFi.persistent(false);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    wifiAttemptStartMs = millis();
}

// Reboot if we've been down too long. Router may be booting alongside us at
// match power-on, so the first window is generous (60 s) and subsequent
// retries are shorter (30 s).
static void wifiWatchdog() {
    if (WiFi.status() == WL_CONNECTED) {
        wifiAttemptStartMs = millis();   // keep refreshing while connected
        wifiFirstBoot      = false;
        return;
    }
    const uint32_t timeout = wifiFirstBoot ? WIFI_CONNECT_TIMEOUT_MS
                                           : WIFI_RETRY_REBOOT_MS;
    if (millis() - wifiAttemptStartMs > timeout) {
        Serial.printf("[xiao] WiFi down %lums, restarting…\n",
                      (unsigned long)(millis() - wifiAttemptStartMs));
        statWifiReboots++;
        delay(50);
        ESP.restart();
    }
}

// ── HTTP debug page ──────────────────────────────────────────────────────────

static String fmtBytes(uint32_t b) {
    char buf[32];
    if (b < 1024)             snprintf(buf, sizeof(buf), "%lu B",   (unsigned long)b);
    else if (b < 1024UL*1024) snprintf(buf, sizeof(buf), "%.1f KB", b / 1024.0f);
    else                      snprintf(buf, sizeof(buf), "%.2f MB", b / (1024.0f*1024.0f));
    return String(buf);
}

static String fmtUptime(uint32_t ms) {
    uint32_t s = ms / 1000;
    uint32_t h = s / 3600; s %= 3600;
    uint32_t m = s / 60;   s %= 60;
    char buf[32];
    snprintf(buf, sizeof(buf), "%luh %02lum %02lus",
             (unsigned long)h, (unsigned long)m, (unsigned long)s);
    return String(buf);
}

static uint16_t ringFill(uint16_t head, uint16_t tail, uint16_t cap) {
    return (uint16_t)((head + cap - tail) % cap);
}

static void handleHttpRoot() {
    // Static skeleton — values are filled in by a tiny JS poller that
    // hits /stats.json every 2 s. No full-page reloads.
    static const char PAGE[] PROGMEM =
        "<!doctype html><html><head><meta charset=utf-8>"
        "<title>holOS Xiao bridge</title>"
        "<style>"
        "body{font-family:ui-monospace,Menlo,monospace;background:#111;color:#eee;margin:24px;max-width:560px}"
        "h1{font-size:18px;margin:0 0 16px}"
        "table{width:100%;border-collapse:collapse;margin-bottom:18px}"
        "th{text-align:left;font-weight:normal;color:#888;padding:4px 8px 4px 0;width:160px;vertical-align:top}"
        "td{padding:4px 0}"
        ".ok{color:#5f5}.warn{color:#fa0}.err{color:#f55}"
        ".bar{display:inline-block;width:200px;height:8px;background:#222;vertical-align:middle;margin-left:8px;border:1px solid #333}"
        ".fill{display:block;height:100%;background:#5f5}"
        "h2{font-size:14px;color:#888;margin:16px 0 4px;border-top:1px solid #222;padding-top:12px}"
        "</style></head><body>"
        "<h1>holOS — Xiao WiFi bridge <span id=live style='color:#555;font-size:11px;font-weight:normal'>•</span></h1>"

        "<h2>WiFi</h2><table>"
        "<tr><th>State</th>   <td id=wifi>-</td></tr>"
        "<tr><th>SSID</th>    <td id=ssid>-</td></tr>"
        "<tr><th>IP</th>      <td id=ip>-</td></tr>"
        "<tr><th>MAC</th>     <td id=mac>-</td></tr>"
        "<tr><th>Hostname</th><td id=host>-</td></tr>"
        "<tr><th>RSSI</th>    <td id=rssi>-</td></tr>"
        "<tr><th>Reboots (WiFi)</th><td id=reboots>-</td></tr>"
        "</table>"

        "<h2>TCP relay</h2><table>"
        "<tr><th>Port</th>            <td id=port>-</td></tr>"
        "<tr><th>Client</th>          <td id=client>-</td></tr>"
        "<tr><th>Total connects</th>  <td id=connects>-</td></tr>"
        "<tr><th>Replaced</th>        <td id=replaced>-</td></tr>"
        "<tr><th>Since last connect</th><td id=since>-</td></tr>"
        "</table>"

        "<h2>UART (Teensy)</h2><table>"
        "<tr><th>Baud</th>        <td id=baud>-</td></tr>"
        "<tr><th>Pins</th>        <td id=pins>-</td></tr>"
        "<tr><th>UART → TCP</th><td id=rx>-</td></tr>"
        "<tr><th>TCP → UART</th><td id=tx>-</td></tr>"
        "<tr><th>RX overflows</th><td id=rxov>-</td></tr>"
        "<tr><th>TX overflows</th><td id=txov>-</td></tr>"
        "</table>"

        "<h2>Ring buffers</h2><table>"
        "<tr><th>UART → TCP</th><td id=rxring>-</td></tr>"
        "<tr><th>TCP → UART</th><td id=txring>-</td></tr>"
        "</table>"

        "<h2>System</h2><table>"
        "<tr><th>Uptime</th>      <td id=up>-</td></tr>"
        "<tr><th>Free heap</th>   <td id=heap>-</td></tr>"
        "<tr><th>Reset reason</th><td id=rst>-</td></tr>"
        "<tr><th>Chip</th>        <td id=chip>-</td></tr>"
        "</table>"

        "<p style='color:#555;font-size:11px'>Live • polls /stats.json every 2s • "
        "<a href='/stats.json' style='color:#888'>JSON</a></p>"

        "<script>"
        "const $=id=>document.getElementById(id);"
        "function fb(b){if(b<1024)return b+' B';if(b<1048576)return (b/1024).toFixed(1)+' KB';return (b/1048576).toFixed(2)+' MB'}"
        "function fu(ms){let s=Math.floor(ms/1000);const h=Math.floor(s/3600);s%=3600;const m=Math.floor(s/60);s%=60;return h+'h '+String(m).padStart(2,'0')+'m '+String(s).padStart(2,'0')+'s'}"
        "function bar(fill,cap){const p=cap?Math.floor(fill*100/cap):0;return \"<span class=bar><span class=fill style='width:\"+p+\"%'></span></span> \"+fill+' / '+cap+' B'}"
        "let blink=false;"
        "async function tick(){"
        " try{"
        "  const r=await fetch('/stats.json',{cache:'no-store'});"
        "  if(!r.ok)throw 0;"
        "  const d=await r.json();"
        "  $('wifi').innerHTML=d.wifi_ok?\"<span class=ok>connected</span>\":\"<span class=err>disconnected</span>\";"
        "  $('ssid').textContent=d.ssid||'-';"
        "  $('ip').textContent=d.ip||'-';"
        "  $('mac').textContent=d.mac||'-';"
        "  $('host').textContent=(d.hostname||'-')+'.local';"
        "  $('rssi').textContent=d.wifi_ok?(d.rssi+' dBm'):'-';"
        "  $('reboots').textContent=d.wifi_reboots;"
        "  $('port').textContent=d.tcp_port;"
        "  $('client').innerHTML=d.client_up?(\"<span class=ok>\"+d.client_ip+\"</span>\"):\"<span class=warn>none</span>\";"
        "  $('connects').textContent=d.connects;"
        "  $('replaced').textContent=d.replaced;"
        "  $('since').textContent=d.since_last_ms?fu(d.since_last_ms):'-';"
        "  $('baud').textContent=d.uart_baud+' (8N1)';"
        "  $('pins').textContent='TX D6 (GPIO'+d.uart_tx+') · RX D7 (GPIO'+d.uart_rx+')';"
        "  $('rx').textContent=fb(d.bytes_rx);"
        "  $('tx').textContent=fb(d.bytes_tx);"
        "  $('rxov').innerHTML=d.rx_overflow+(d.rx_overflow?\" <span class=warn>(byte drops!)</span>\":'');"
        "  $('txov').innerHTML=d.tx_overflow+(d.tx_overflow?\" <span class=warn>(byte drops!)</span>\":'');"
        "  $('rxring').innerHTML=bar(d.rx_ring,d.rx_ring_cap);"
        "  $('txring').innerHTML=bar(d.tx_ring,d.tx_ring_cap);"
        "  $('up').textContent=fu(d.uptime_ms);"
        "  $('heap').textContent=fb(d.free_heap);"
        "  $('rst').textContent=d.reset_reason;"
        "  $('chip').textContent=d.chip+' rev '+d.chip_rev;"
        "  blink=!blink;$('live').style.color=blink?'#5f5':'#555';"
        " }catch(e){$('live').style.color='#f55'}"
        "}"
        "tick();setInterval(tick,2000);"
        "</script>"
        "</body></html>";
    httpServer.send_P(200, "text/html; charset=utf-8", PAGE);
}

static void handleHttpStatsJson() {
    const bool clientUp = (tcpClient && tcpClient.connected());
    String j;
    j.reserve(768);
    j += "{";
    j += "\"uptime_ms\":";    j += String(millis()); j += ",";
    j += "\"wifi_ok\":";      j += (WiFi.status() == WL_CONNECTED ? "true" : "false"); j += ",";
    j += "\"ssid\":\"";       j += WiFi.SSID(); j += "\",";
    j += "\"ip\":\"";         j += WiFi.localIP().toString(); j += "\",";
    j += "\"mac\":\"";        j += WiFi.macAddress(); j += "\",";
    j += "\"hostname\":\"";   j += WIFI_HOSTNAME; j += "\",";
    j += "\"rssi\":";         j += String(WiFi.RSSI()); j += ",";
    j += "\"wifi_reboots\":"; j += String(statWifiReboots); j += ",";
    j += "\"tcp_port\":";     j += String(TCP_PORT); j += ",";
    j += "\"client_up\":";    j += (clientUp ? "true" : "false"); j += ",";
    j += "\"client_ip\":\"";  j += (clientUp ? tcpClient.remoteIP().toString() : String("")); j += "\",";
    j += "\"connects\":";     j += String(statClientConnects); j += ",";
    j += "\"replaced\":";     j += String(statClientReplaced); j += ",";
    j += "\"since_last_ms\":";j += String(statLastClientMs ? (millis() - statLastClientMs) : 0UL); j += ",";
    j += "\"uart_baud\":";    j += String(UART_BAUD); j += ",";
    j += "\"uart_tx\":";      j += String(UART_TX_PIN); j += ",";
    j += "\"uart_rx\":";      j += String(UART_RX_PIN); j += ",";
    j += "\"bytes_rx\":";     j += String(statBytesRx); j += ",";
    j += "\"bytes_tx\":";     j += String(statBytesTx); j += ",";
    j += "\"rx_overflow\":";  j += String(statRxOverflows); j += ",";
    j += "\"tx_overflow\":";  j += String(statTxOverflows); j += ",";
    j += "\"rx_ring\":";      j += String(ringFill(rxHead, rxTail, RX_RING_BYTES)); j += ",";
    j += "\"tx_ring\":";      j += String(ringFill(txHead, txTail, TX_RING_BYTES)); j += ",";
    j += "\"rx_ring_cap\":";  j += String(RX_RING_BYTES); j += ",";
    j += "\"tx_ring_cap\":";  j += String(TX_RING_BYTES); j += ",";
    j += "\"free_heap\":";    j += String(ESP.getFreeHeap()); j += ",";
    j += "\"reset_reason\":"; j += String((int)esp_reset_reason()); j += ",";
    j += "\"chip\":\"";       j += ESP.getChipModel(); j += "\",";
    j += "\"chip_rev\":";     j += String(ESP.getChipRevision());
    j += "}";
    httpServer.send(200, "application/json", j);
}

// ── Setup ────────────────────────────────────────────────────────────────────

void setup() {
    pinMode(STATUS_LED_PIN, OUTPUT);
    ledWrite(true);   // light up immediately so we know boot started

    // USB-CDC for logs (totally independent from Serial1 which talks to Teensy).
    Serial.begin(115200);
    Serial.println("\n=== holOS Xiao WiFi bridge ===");

    // UART to Teensy.
    Serial1.begin(UART_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

    // WiFi.
    wifiBegin();

    // Don't trap forever — wifiWatchdog() handles real timeouts in loop().
    // The setup pause just gives the user readable log lines.
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 8000) {
        ledTick(LED_CONNECTING);
        delay(10);
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("[xiao] WiFi ok: %s\n", WiFi.localIP().toString().c_str());
        MDNS.begin(WIFI_HOSTNAME);
        wifiFirstBoot = false;
    } else {
        Serial.println("[xiao] WiFi still connecting — watchdog will reboot if needed");
    }

    tcpServer.begin();
    tcpServer.setNoDelay(true);   // disable Nagle — telemetry frames are tiny
    Serial.printf("[xiao] TCP listening on :%d\n", TCP_PORT);

    // Debug page on port 80 — http://<xiao-ip>/ or http://holos-xiao.local/
    httpServer.on("/",           HTTP_GET, handleHttpRoot);
    httpServer.on("/stats.json", HTTP_GET, handleHttpStatsJson);
    httpServer.begin();
    Serial.println("[xiao] HTTP debug on :80");
}

// ── Loop ─────────────────────────────────────────────────────────────────────

void loop() {
    wifiWatchdog();

    // ── Accept new TCP client (replaces existing one) ─────────────────────
    if (tcpServer.hasClient()) {
        WiFiClient incoming = tcpServer.available();
        if (tcpClient && tcpClient.connected()) {
            Serial.println("[xiao] Replacing existing TCP client");
            tcpClient.stop();
            statClientReplaced++;
        }
        tcpClient = incoming;
        tcpClient.setNoDelay(true);
        statClientConnects++;
        statLastClientMs = millis();
        Serial.printf("[xiao] Client connected: %s\n",
                      tcpClient.remoteIP().toString().c_str());
        // Drop any stale bytes queued from a previous link — the new peer
        // starts the protocol from scratch (ping/pong handshake).
        rxHead = rxTail = 0;
        txHead = txTail = 0;
    }

    // ── UART → ring ───────────────────────────────────────────────────────
    while (Serial1.available()) {
        uint8_t b = (uint8_t)Serial1.read();
        if (!ringPush(rxRing, RX_RING_BYTES, rxHead, rxTail, b)) {
            // Overflow — drop oldest, push newest. Telemetry recovers; the
            // protocol's CRC catches any half-frame fallout downstream.
            rxTail = (uint16_t)((rxTail + 1) % RX_RING_BYTES);
            ringPush(rxRing, RX_RING_BYTES, rxHead, rxTail, b);
            statRxOverflows++;
        }
        statBytesRx++;
    }

    // ── TCP → ring ────────────────────────────────────────────────────────
    if (tcpClient && tcpClient.connected()) {
        int avail = tcpClient.available();
        while (avail-- > 0) {
            int v = tcpClient.read();
            if (v < 0) break;
            if (!ringPush(txRing, TX_RING_BYTES, txHead, txTail, (uint8_t)v)) {
                txTail = (uint16_t)((txTail + 1) % TX_RING_BYTES);
                ringPush(txRing, TX_RING_BYTES, txHead, txTail, (uint8_t)v);
                statTxOverflows++;
            }
            statBytesTx++;
        }
    } else {
        // No client — drain rxRing so UART traffic doesn't pile up indefinitely
        // when the Jetson is disconnected (e.g. during the pre-match boot).
        rxHead = rxTail;
    }

    // ── Ring → TCP ────────────────────────────────────────────────────────
    if (tcpClient && tcpClient.connected() && rxHead != rxTail) {
        uint8_t chunk[128];
        size_t  n = 0;
        uint8_t b;
        while (n < sizeof(chunk) &&
               ringPop(rxRing, RX_RING_BYTES, rxHead, rxTail, b)) {
            chunk[n++] = b;
        }
        if (n > 0) {
            size_t written = tcpClient.write(chunk, n);
            if (written != n) {
                // Socket back-pressure — push the remainder back at the head
                // of the ring for the next loop iteration.
                for (size_t i = written; i < n; ++i) {
                    rxTail = (uint16_t)((rxTail + RX_RING_BYTES - 1) % RX_RING_BYTES);
                    rxRing[rxTail] = chunk[i];
                }
            }
        }
    }

    // ── Ring → UART ───────────────────────────────────────────────────────
    if (txHead != txTail) {
        int room = Serial1.availableForWrite();
        uint8_t b;
        while (room-- > 0 &&
               ringPop(txRing, TX_RING_BYTES, txHead, txTail, b)) {
            Serial1.write(b);
        }
    }

    // ── HTTP debug page ───────────────────────────────────────────────────
    // Cheap call: when no socket is pending, returns in <1 µs. The relay
    // loop above runs hundreds of times per second so a request handled
    // mid-iteration adds at most a few ms of latency to telemetry — still
    // well within the heartbeat window.
    httpServer.handleClient();

    // ── LED state ─────────────────────────────────────────────────────────
    // WiFi down → error blink; WiFi up → heartbeat. The TCP-client state is
    // visible on the /debug page; tying the LED to it would leave the Xiao
    // stuck on "connecting" whenever holOS happens to not be running.
    ledTick(WiFi.status() == WL_CONNECTED ? LED_CONNECTED : LED_ERROR);
}
