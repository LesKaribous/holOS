#include <Arduino.h>
#include "esp_camera.h"
#include "stream_server.h"
#include "../include/config.h"

httpd_handle_t stream_httpd = NULL;

static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="fr">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>ESP32-CAM</title>
<style>
  body { font-family: sans-serif; text-align: center; margin: 1em; background: #111; color: #eee; }
  button { font-size: 1.2em; padding: 0.8em 1.6em; margin: 1em; cursor: pointer;
           background: #2a7; color: white; border: none; border-radius: 6px; }
  button:disabled { background: #555; cursor: wait; }
  img { max-width: 100%; height: auto; border: 1px solid #333; margin-top: 1em; }
  #status { color: #aaa; min-height: 1.2em; }
</style>
</head>
<body>
  <h1>ESP32-CAM</h1>
  <button id="snap">Prendre une photo</button>
  <div id="status"></div>
  <img id="photo" alt="">
<script>
  const btn = document.getElementById('snap');
  const img = document.getElementById('photo');
  const status = document.getElementById('status');
  let currentUrl = null;
  btn.addEventListener('click', async () => {
    btn.disabled = true;
    status.textContent = 'Capture...';
    const t0 = performance.now();
    try {
      const r = await fetch('/capture?t=' + Date.now(), { cache: 'no-store' });
      if (!r.ok) throw new Error('HTTP ' + r.status);
      const blob = await r.blob();
      if (currentUrl) URL.revokeObjectURL(currentUrl);
      currentUrl = URL.createObjectURL(blob);
      img.src = currentUrl;
      const dt = (performance.now() - t0).toFixed(0);
      status.textContent = `OK — ${(blob.size / 1024).toFixed(1)} kB en ${dt} ms`;
    } catch (e) {
      status.textContent = 'Erreur: ' + e.message;
    } finally {
      btn.disabled = false;
    }
  });
</script>
</body>
</html>
)rawliteral";

static esp_err_t index_handler(httpd_req_t* req) {
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t capture_handler(httpd_req_t* req) {
    uint32_t t0 = millis();
    camera_fb_t* fb = esp_camera_fb_get();
    uint32_t t1 = millis();
    if (!fb) {
        Serial.println("[Capture] Echec esp_camera_fb_get");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");

    size_t len = fb->len;
    esp_err_t res = httpd_resp_send(req, (const char*)fb->buf, len);
    uint32_t t2 = millis();
    esp_camera_fb_return(fb);
    Serial.printf("[Capture] %u B  grab=%u ms  send=%u ms  total=%u ms\n",
                  (unsigned)len, (unsigned)(t1 - t0),
                  (unsigned)(t2 - t1), (unsigned)(t2 - t0));
    return res;
}

void stream_server_start() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = STREAM_SERVER_PORT;
    config.max_uri_handlers = 4;

    httpd_uri_t index_uri = {
        .uri      = "/",
        .method   = HTTP_GET,
        .handler  = index_handler,
        .user_ctx = NULL
    };
    httpd_uri_t capture_uri = {
        .uri      = "/capture",
        .method   = HTTP_GET,
        .handler  = capture_handler,
        .user_ctx = NULL
    };

    if (httpd_start(&stream_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(stream_httpd, &index_uri);
        httpd_register_uri_handler(stream_httpd, &capture_uri);
        Serial.printf("[HTTP] Serveur démarré sur le port %d\n", STREAM_SERVER_PORT);
    } else {
        Serial.println("[HTTP] Erreur démarrage serveur");
    }
}

void stream_server_stop() {
    if (stream_httpd) {
        httpd_stop(stream_httpd);
        stream_httpd = NULL;
    }
}
