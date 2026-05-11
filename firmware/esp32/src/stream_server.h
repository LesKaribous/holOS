#pragma once

#include "esp_http_server.h"

extern httpd_handle_t stream_httpd;

void stream_server_start();
void stream_server_stop();
