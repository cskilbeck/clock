#pragma once

// call once before anything
void *neopixel_init(int num_leds);

// get where to put your RGB pixels
byte *neopixel_get_frame_buffer();

// call this at, say, 50 or 60Hz
void neopixel_update();
