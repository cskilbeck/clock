//////////////////////////////////////////////////////////////////////

#include <string.h>
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>
#include <utility>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "types.h"
#include "util.h"
#include "wifi.h"
#include "clock.h"

//////////////////////////////////////////////////////////////////////

#define clear_console "\x1b[2J\x1b[H"

static char const *TAG = "MAIN";

//////////////////////////////////////////////////////////////////////

extern "C" void app_main()
{
    ESP_LOGI(TAG, clear_console "Here we go...");

    if(nvs_flash_init() == ESP_ERR_NVS_NO_FREE_PAGES) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    do_clock();
}
