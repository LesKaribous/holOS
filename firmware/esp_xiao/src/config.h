#pragma once

// ── WiFi credentials ─────────────────────────────────────────────────────────
// Same network as ESP32-CAM and Jetson. Change once here, rebuild, flash.
#define WIFI_SSID       "TP-Link_F0FC"
#define WIFI_PASSWORD   "kaamelott80"

// Optional fixed hostname — helps locate the Xiao on the LAN (e.g. mDNS).
#define WIFI_HOSTNAME   "holos-xiao"

// ── TCP server ───────────────────────────────────────────────────────────────
// The Jetson opens a TCP socket here. One client at a time; any second
// connection forcefully replaces the previous one (covers reconnect after
// Jetson crash / Wi-Fi blip without leaving a dead socket open).
#define TCP_PORT        9000

// ── UART to Teensy 4.1 (BRIDGE_XBEE = Serial2 @ 57600) ───────────────────────
// Xiao S3 Serial1 exposed on D6 (TX, GPIO43) and D7 (RX, GPIO44).
// Wire D6→Teensy RX2, D7→Teensy TX2, GND→GND.
#define UART_BAUD       57600
#define UART_TX_PIN     43   // D6
#define UART_RX_PIN     44   // D7

// ── WiFi reconnect / watchdog ────────────────────────────────────────────────
// First reboot after 60 s of failed connect (router may still be booting),
// then every 30 s if WiFi never recovers. Bytes from UART keep being buffered
// in the meantime — they're dropped only when the buffer overflows.
#define WIFI_CONNECT_TIMEOUT_MS   60000UL
#define WIFI_RETRY_REBOOT_MS      30000UL

// ── Onboard status LED ───────────────────────────────────────────────────────
// Xiao ESP32-S3 USER_LED on GPIO21, active LOW.
// Patterns (handled in main.cpp):
//   • Connecting (WiFi up, no TCP client): rapid on/off (100 ms / 100 ms)
//   • Connected  (WiFi up + TCP client):   slow heartbeat (50 ms on / 1950 ms off)
//   • Error      (WiFi down):              3 fast blinks + 1 s pause, repeat
#define STATUS_LED_PIN          21
#define STATUS_LED_ACTIVE_LOW   1

// ── Buffers ──────────────────────────────────────────────────────────────────
// Bytes piling up in either direction when the peer is slow. 4 KB each side
// covers a full burst of telemetry + occupancy without back-pressuring the
// Teensy (which would drop frames into its ring buffer).
#define RX_RING_BYTES   4096   // UART → TCP
#define TX_RING_BYTES   4096   // TCP  → UART
