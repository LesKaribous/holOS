#pragma once

// WiFi
#define WIFI_SSID       "TP-Link_F0FC"
#define WIFI_PASSWORD   "kaamelott80"
#define WIFI_TIMEOUT_MS 15000

// Serial
#define SERIAL_BAUD 115200

// Camera model — décommenter le modèle utilisé
#define CAMERA_MODEL_AI_THINKER
//#define CAMERA_MODEL_WROVER_KIT
//#define CAMERA_MODEL_M5STACK_PSRAM
//#define CAMERA_MODEL_M5STACK_WITHOUT_PSRAM

// --- Pins par modèle ---
#if defined(CAMERA_MODEL_WROVER_KIT)
  #define PWDN_GPIO_NUM    -1
  #define RESET_GPIO_NUM   -1
  #define XCLK_GPIO_NUM    21
  #define SIOD_GPIO_NUM    26
  #define SIOC_GPIO_NUM    27
  #define Y9_GPIO_NUM      35
  #define Y8_GPIO_NUM      34
  #define Y7_GPIO_NUM      39
  #define Y6_GPIO_NUM      36
  #define Y5_GPIO_NUM      19
  #define Y4_GPIO_NUM      18
  #define Y3_GPIO_NUM       5
  #define Y2_GPIO_NUM       4
  #define VSYNC_GPIO_NUM   25
  #define HREF_GPIO_NUM    23
  #define PCLK_GPIO_NUM    22

#elif defined(CAMERA_MODEL_M5STACK_PSRAM)
  #define PWDN_GPIO_NUM    -1
  #define RESET_GPIO_NUM   15
  #define XCLK_GPIO_NUM    27
  #define SIOD_GPIO_NUM    25
  #define SIOC_GPIO_NUM    23
  #define Y9_GPIO_NUM      19
  #define Y8_GPIO_NUM      36
  #define Y7_GPIO_NUM      18
  #define Y6_GPIO_NUM      39
  #define Y5_GPIO_NUM       5
  #define Y4_GPIO_NUM      34
  #define Y3_GPIO_NUM      35
  #define Y2_GPIO_NUM      32
  #define VSYNC_GPIO_NUM   22
  #define HREF_GPIO_NUM    26
  #define PCLK_GPIO_NUM    21

#elif defined(CAMERA_MODEL_M5STACK_WITHOUT_PSRAM)
  #define PWDN_GPIO_NUM    -1
  #define RESET_GPIO_NUM   15
  #define XCLK_GPIO_NUM    27
  #define SIOD_GPIO_NUM    25
  #define SIOC_GPIO_NUM    23
  #define Y9_GPIO_NUM      19
  #define Y8_GPIO_NUM      36
  #define Y7_GPIO_NUM      18
  #define Y6_GPIO_NUM      39
  #define Y5_GPIO_NUM       5
  #define Y4_GPIO_NUM      34
  #define Y3_GPIO_NUM      35
  #define Y2_GPIO_NUM      17
  #define VSYNC_GPIO_NUM   22
  #define HREF_GPIO_NUM    26
  #define PCLK_GPIO_NUM    21

#elif defined(CAMERA_MODEL_AI_THINKER)
  #define PWDN_GPIO_NUM    32
  #define RESET_GPIO_NUM   -1
  #define XCLK_GPIO_NUM     0
  #define SIOD_GPIO_NUM    26
  #define SIOC_GPIO_NUM    27
  #define Y9_GPIO_NUM      35
  #define Y8_GPIO_NUM      34
  #define Y7_GPIO_NUM      39
  #define Y6_GPIO_NUM      36
  #define Y5_GPIO_NUM      21
  #define Y4_GPIO_NUM      19
  #define Y3_GPIO_NUM      18
  #define Y2_GPIO_NUM       5
  #define VSYNC_GPIO_NUM   25
  #define HREF_GPIO_NUM    23
  #define PCLK_GPIO_NUM    22

#else
  #error "Aucun modèle de caméra sélectionné dans config.h"
#endif

// Streaming MJPEG
#define STREAM_SERVER_PORT 80
#define PART_BOUNDARY      "123456789000000000000987654321"

// LED de statut (heartbeat). Sur AI-Thinker: GPIO 33 (rouge, active LOW).
#define STATUS_LED_PIN     33
#define STATUS_LED_ACTIVE_LOW 1
