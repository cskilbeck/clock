#pragma once

esp_err_t init_timezone();
int timezone_offset();

void timezone_task(void *);