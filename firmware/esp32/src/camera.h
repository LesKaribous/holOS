#pragma once

#include "esp_camera.h"

bool camera_init();
camera_fb_t* camera_capture();
void camera_return_fb(camera_fb_t* fb);
