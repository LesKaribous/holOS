#include <Arduino.h>
#include <WiFi.h>
#include "wifi_manager.h"
#include "../include/config.h"

bool wifi_connect() {
    Serial.printf("[WiFi] Connexion à %s...\n", WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED) {
        if (millis() - start > WIFI_TIMEOUT_MS) {
            Serial.println("[WiFi] Timeout — connexion échouée");
            return false;
        }
        delay(500);
        Serial.print(".");
    }

    Serial.printf("\n[WiFi] Connecté. IP: %s\n", WiFi.localIP().toString().c_str());
    return true;
}

void wifi_disconnect() {
    WiFi.disconnect(true);
    Serial.println("[WiFi] Déconnecté");
}

bool wifi_is_connected() {
    return WiFi.status() == WL_CONNECTED;
}
