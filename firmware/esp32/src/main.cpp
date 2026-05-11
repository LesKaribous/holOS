#include <Arduino.h>
#include <WiFi.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "../include/config.h"
#include "wifi_manager.h"
#include "camera.h"
#include "stream_server.h"

// Cadence du heartbeat (ms ON / ms OFF). Modifiée à chaque étape pour
// donner un retour visuel sur l'avancement du boot.
static volatile uint32_t hb_on_ms  = 100;
static volatile uint32_t hb_off_ms = 900;

static inline void led_write(bool on) {
#if STATUS_LED_ACTIVE_LOW
    digitalWrite(STATUS_LED_PIN, on ? LOW : HIGH);
#else
    digitalWrite(STATUS_LED_PIN, on ? HIGH : LOW);
#endif
}

static void heartbeat_task(void*) {
    pinMode(STATUS_LED_PIN, OUTPUT);
    for (;;) {
        led_write(true);
        vTaskDelay(pdMS_TO_TICKS(hb_on_ms));
        led_write(false);
        vTaskDelay(pdMS_TO_TICKS(hb_off_ms));
    }
}

void setup() {
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // désactive le détecteur brownout

    // Heartbeat le plus tôt possible — survit même si la suite bloque.
    xTaskCreatePinnedToCore(heartbeat_task, "heartbeat", 2048, NULL, 1, NULL, 1);

    Serial.begin(SERIAL_BAUD);
    Serial.setDebugOutput(true);
    delay(500);
    Serial.println("\n=== ESP32-CAM OnBoard ===");

    if (!camera_init()) {
        Serial.println("[FATAL] Caméra non disponible — redémarrage dans 5s");
        hb_on_ms = 50; hb_off_ms = 50; // SOS rapide
        delay(5000);
        ESP.restart();
    }

    // Camera OK -> blink moyen
    hb_on_ms = 100; hb_off_ms = 400;

    if (!wifi_connect()) {
        Serial.println("[FATAL] WiFi non disponible — redémarrage dans 5s");
        hb_on_ms = 50; hb_off_ms = 50;
        delay(5000);
        ESP.restart();
    }

    stream_server_start();

    // Tout OK -> heartbeat lent (1 Hz, flash court)
    hb_on_ms = 50; hb_off_ms = 1950;

    Serial.printf("[OK] Interface disponible sur http://%s/\n",
                  WiFi.localIP().toString().c_str());
    Serial.printf("[OK] Capture directe: http://%s/capture\n",
                  WiFi.localIP().toString().c_str());
}

void loop() {
    delay(1);
}
