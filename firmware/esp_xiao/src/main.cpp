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
    const bool wifiUp   = (WiFi.status() == WL_CONNECTED);
    const bool clientUp = (tcpClient && tcpClient.connected());
    const uint16_t rxFill = ringFill(rxHead, rxTail, RX_RING_BYTES);
    const uint16_t txFill = ringFill(txHead, txTail, TX_RING_BYTES);

    String h;
    h.reserve(2048);
    h += F("<!doctype html><html><head><meta charset=utf-8>"
           "<meta http-equiv=refresh content=2>"
           "<title>holOS Xiao bridge</title>"
           "<style>"
           "body{font-family:ui-monospace,Menlo,monospace;background:#111;color:#eee;margin:24px;max-width:560px}"
           "h1{font-size:18px;margin:0 0 16px}"
           "table{width:100%;border-collapse:collapse;margin-bottom:18px}"
           "th{text-align:left;font-weight:normal;color:#888;padding:4px 8px 4px 0;width:160px;vertical-align:top}"
           "td{padding:4px 0}"
           ".ok{color:#5f5} .warn{color:#fa0} .err{color:#f55}"
           ".bar{display:inline-block;width:200px;height:8px;background:#222;vertical-align:middle;margin-left:8px;border:1px solid #333}"
           ".fill{display:block;height:100%;background:#5f5}"
           "h2{font-size:14px;color:#888;margin:16px 0 4px;border-top:1px solid #222;padding-top:12px}"
           "</style></head><body>");
    h += F("<h1>holOS — Xiao WiFi bridge</h1>");

    h += F("<h2>WiFi</h2><table>");
    h += "<tr><th>State</th><td>";
    h += wifiUp ? F("<span class=ok>connected</span>") : F("<span class=err>disconnected</span>");
    h += "</td></tr>";
    h += "<tr><th>SSID</th><td>"; h += WiFi.SSID(); h += "</td></tr>";
    h += "<tr><th>IP</th><td>";   h += WiFi.localIP().toString(); h += "</td></tr>";
    h += "<tr><th>MAC</th><td>";  h += WiFi.macAddress(); h += "</td></tr>";
    h += "<tr><th>Hostname</th><td>"; h += WIFI_HOSTNAME; h += ".local</td></tr>";
    h += "<tr><th>RSSI</th><td>";
    if (wifiUp) { h += String(WiFi.RSSI()); h += " dBm"; } else { h += "-"; }
    h += "</td></tr>";
    h += "<tr><th>Reboots (WiFi)</th><td>"; h += String(statWifiReboots); h += "</td></tr>";
    h += F("</table>");

    h += F("<h2>TCP relay</h2><table>");
    h += "<tr><th>Port</th><td>"; h += String(TCP_PORT); h += "</td></tr>";
    h += "<tr><th>Client</th><td>";
    if (clientUp) {
        h += F("<span class=ok>");
        h += tcpClient.remoteIP().toString();
        h += F("</span>");
    } else {
        h += F("<span class=warn>none</span>");
    }
    h += "</td></tr>";
    h += "<tr><th>Total connects</th><td>"; h += String(statClientConnects); h += "</td></tr>";
    h += "<tr><th>Replaced</th><td>";       h += String(statClientReplaced); h += "</td></tr>";
    if (statLastClientMs) {
        h += "<tr><th>Since last connect</th><td>";
        h += fmtUptime(millis() - statLastClientMs);
        h += "</td></tr>";
    }
    h += F("</table>");

    h += F("<h2>UART (Teensy)</h2><table>");
    h += "<tr><th>Baud</th><td>"; h += String(UART_BAUD); h += " (8N1)</td></tr>";
    h += "<tr><th>Pins</th><td>TX D6 (GPIO";
    h += String(UART_TX_PIN);
    h += ") · RX D7 (GPIO";
    h += String(UART_RX_PIN);
    h += ")</td></tr>";
    h += "<tr><th>UART → TCP</th><td>"; h += fmtBytes(statBytesRx); h += "</td></tr>";
    h += "<tr><th>TCP → UART</th><td>"; h += fmtBytes(statBytesTx); h += "</td></tr>";
    h += "<tr><th>RX overflows</th><td>";
    h += String(statRxOverflows);
    if (statRxOverflows) h += F(" <span class=warn>(byte drops!)</span>");
    h += "</td></tr>";
    h += "<tr><th>TX overflows</th><td>";
    h += String(statTxOverflows);
    if (statTxOverflows) h += F(" <span class=warn>(byte drops!)</span>");
    h += "</td></tr>";
    h += F("</table>");

    auto bar = [&](uint16_t fill, uint16_t cap) {
        int pct = cap ? (int)((uint32_t)fill * 100 / cap) : 0;
        h += "<span class=bar><span class=fill style='width:";
        h += String(pct);
        h += "%'></span></span> ";
        h += String(fill);
        h += " / ";
        h += String(cap);
        h += " B";
    };
    h += F("<h2>Ring buffers</h2><table>");
    h += "<tr><th>UART → TCP</th><td>"; bar(rxFill, RX_RING_BYTES); h += "</td></tr>";
    h += "<tr><th>TCP → UART</th><td>"; bar(txFill, TX_RING_BYTES); h += "</td></tr>";
    h += F("</table>");

    h += F("<h2>System</h2><table>");
    h += "<tr><th>Uptime</th><td>";   h += fmtUptime(millis()); h += "</td></tr>";
    h += "<tr><th>Free heap</th><td>"; h += fmtBytes(ESP.getFreeHeap()); h += "</td></tr>";
    h += "<tr><th>Reset reason</th><td>"; h += String((int)esp_reset_reason()); h += "</td></tr>";
    h += "<tr><th>Chip</th><td>"; h += ESP.getChipModel(); h += " rev "; h += String(ESP.getChipRevision()); h += "</td></tr>";
    h += F("</table>");

    h += F("<p style='color:#555;font-size:11px'>Auto-refresh every 2 s · "
           "<a href='/stats.json' style='color:#888'>JSON</a></p>"
           "</body></html>");

    httpServer.send(200, "text/html; charset=utf-8", h);
}

static void handleHttpStatsJson() {
    String j;
    j.reserve(512);
    j += "{";
    j += "\"uptime_ms\":";   j += String(millis()); j += ",";
    j += "\"wifi_ok\":";     j += (WiFi.status() == WL_CONNECTED ? "true" : "false"); j += ",";
    j += "\"rssi\":";        j += String(WiFi.RSSI()); j += ",";
    j += "\"ip\":\"";        j += WiFi.localIP().toString(); j += "\",";
    j += "\"client_up\":";   j += (tcpClient && tcpClient.connected() ? "true" : "false"); j += ",";
    j += "\"client_ip\":\""; j += (tcpClient && tcpClient.connected() ? tcpClient.remoteIP().toString() : String("")); j += "\",";
    j += "\"bytes_rx\":";    j += String(statBytesRx); j += ",";
    j += "\"bytes_tx\":";    j += String(statBytesTx); j += ",";
    j += "\"rx_overflow\":"; j += String(statRxOverflows); j += ",";
    j += "\"tx_overflow\":"; j += String(statTxOverflows); j += ",";
    j += "\"rx_ring\":";     j += String(ringFill(rxHead, rxTail, RX_RING_BYTES)); j += ",";
    j += "\"tx_ring\":";     j += String(ringFill(txHead, txTail, TX_RING_BYTES)); j += ",";
    j += "\"connects\":";    j += String(statClientConnects); j += ",";
    j += "\"replaced\":";    j += String(statClientReplaced); j += ",";
    j += "\"wifi_reboots\":";j += String(statWifiReboots); j += ",";
    j += "\"free_heap\":";   j += String(ESP.getFreeHeap());
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
    LedState s;
    if (WiFi.status() != WL_CONNECTED) {
        s = LED_ERROR;
    } else if (!tcpClient || !tcpClient.connected()) {
        s = LED_CONNECTING;
    } else {
        s = LED_CONNECTED;
    }
    ledTick(s);
}
