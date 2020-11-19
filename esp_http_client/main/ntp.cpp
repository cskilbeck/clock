//////////////////////////////////////////////////////////////////////

#include <string.h>
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>
#include <utility>
#include <type_traits>

#include <sys/timeb.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/apps/sntp.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_system.h"

#include "types.h"
#include "util.h"
#include "ntp.h"

static char const *TAG = "CLOCK";

//////////////////////////////////////////////////////////////////////

bool is_sntp_working()
{
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    return timeinfo.tm_year >= (2020 - 1900);    // bogus, but it's the recommended way
}

//////////////////////////////////////////////////////////////////////
// wait for sntp to start

void start_sntp()
{
    static char const *TAG = "SNTP";

    ESP_LOGI(TAG, "Init SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, const_cast<char *>("pool.ntp.org"));
    sntp_init();

    int sntp_cadence = 30;
    int sntp_refresh = sntp_cadence;

    ESP_LOGI(TAG, "Waiting for SNTP");
    while(!is_sntp_working()) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        sntp_refresh -= 1;
        if(sntp_refresh == 0) {
            ESP_LOGI(TAG, "SNTP refresh");
            sntp_stop();
            sntp_init();
            sntp_refresh = sntp_cadence;
        }
    }
    ESP_LOGI(TAG, "Got SNTP connection");
}
