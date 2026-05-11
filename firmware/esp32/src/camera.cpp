#include <Arduino.h>
#include "camera.h"
#include "../include/config.h"

bool camera_init() {
    camera_config_t config = {};

    config.ledc_channel  = LEDC_CHANNEL_0;
    config.ledc_timer    = LEDC_TIMER_0;
    config.pin_d0        = Y2_GPIO_NUM;
    config.pin_d1        = Y3_GPIO_NUM;
    config.pin_d2        = Y4_GPIO_NUM;
    config.pin_d3        = Y5_GPIO_NUM;
    config.pin_d4        = Y6_GPIO_NUM;
    config.pin_d5        = Y7_GPIO_NUM;
    config.pin_d6        = Y8_GPIO_NUM;
    config.pin_d7        = Y9_GPIO_NUM;
    config.pin_xclk      = XCLK_GPIO_NUM;
    config.pin_pclk      = PCLK_GPIO_NUM;
    config.pin_vsync     = VSYNC_GPIO_NUM;
    config.pin_href      = HREF_GPIO_NUM;
    config.pin_sscb_sda  = SIOD_GPIO_NUM;
    config.pin_sscb_scl  = SIOC_GPIO_NUM;
    config.pin_pwdn      = PWDN_GPIO_NUM;
    config.pin_reset     = RESET_GPIO_NUM;
    config.xclk_freq_hz  = 20000000;
    config.pixel_format  = PIXFORMAT_JPEG;
    // esp32-camera ≥ 2.0 only (Arduino-ESP32 core ≥ 2.0.4 / platform
    // espressif32 ≥ 4.x). Older cores ignore these knobs; their
    // defaults (GRAB_WHEN_EMPTY + auto PSRAM) are close enough.
#ifdef CAMERA_GRAB_LATEST
    config.grab_mode     = CAMERA_GRAB_LATEST;
#endif
#ifdef CAMERA_FB_IN_PSRAM
    config.fb_location   = CAMERA_FB_IN_PSRAM;
#endif

    if (psramFound()) {
        config.frame_size   = FRAMESIZE_VGA;
        config.jpeg_quality = 12;
        config.fb_count     = 2;
    } else {
        config.frame_size   = FRAMESIZE_QVGA;
        config.jpeg_quality = 12;
        config.fb_count     = 1;
#ifdef CAMERA_GRAB_WHEN_EMPTY
        config.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;
#endif
#ifdef CAMERA_FB_IN_DRAM
        config.fb_location  = CAMERA_FB_IN_DRAM;
#endif
    }

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("[Camera] Erreur init: 0x%x\n", err);
        return false;
    }

    Serial.println("[Camera] Initialisée");
    return true;
}

camera_fb_t* camera_capture() {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("[Camera] Capture échouée");
    }
    return fb;
}

void camera_return_fb(camera_fb_t* fb) {
    esp_camera_fb_return(fb);
}
