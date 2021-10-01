//////////////////////////////////////////////////////////////////////

#include <string.h>
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>
#include <utility>
#include <algorithm>
#include <type_traits>
#include <sys/timeb.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/apps/sntp.h"
#include "lwip/inet.h"
#include "lwip/ip4_addr.h"
#include "ping/ping.h"

#include "freertos/event_groups.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "esp8266/gpio_struct.h"
#include "esp_ping.h"

#include "types.h"
#include "util.h"
#include "wifi.h"
#include "http.h"
#include "jsmn.h"
#include "message.h"
#include "messenger.h"
#include "timezone.h"
#include "ntp.h"
#include "button.h"

//////////////////////////////////////////////////////////////////////

#define clear_console "\x1b[2J\x1b[HHere we go..."

static char const *TAG = "MAIN";

//////////////////////////////////////////////////////////////////////

constexpr uint32 one_second = 1;
constexpr uint32 one_minute = one_second * 60;
constexpr uint32 one_hour = one_minute * 60;
constexpr uint32 one_day = one_hour * 24;

//////////////////////////////////////////////////////////////////////

control_message_t control_message;
clock_message_t clock_message;
text_message_t text_message;

messenger control_messenger;
messenger clock_messenger;
messenger text_messenger;

//////////////////////////////////////////////////////////////////////

EventGroupHandle_t ping_events;

uint32 ping_count = 3;         // how many pings per report
uint32 ping_timeout = 1000;    // mS till we consider it timed out
uint32 ping_delay = 50;        // mS between pings

bool pinging = false;

//////////////////////////////////////////////////////////////////////

esp_err_t on_ping_result(ping_target_id_t msgType, esp_ping_found *pf)
{
    char const *TAG = "PING";
    if(pf->ping_err == PING_RES_FINISH) {
        if(pf->err_count != 0 || pf->recv_count < pf->send_count) {
            ESP_LOGW(TAG, "PING ERR: Sent:%d Rec:%d Err:%d min(mS):%d max(mS):%d ", pf->send_count, pf->recv_count, pf->err_count, pf->min_time, pf->max_time);
            xEventGroupSetBits(ping_events, 4);
        } else {
            ESP_LOGI(TAG, "PING OK, max %d", pf->max_time);
            xEventGroupSetBits(ping_events, 2);
        }
    }
    return ESP_OK;
}

//////////////////////////////////////////////////////////////////////

template <typename T, size_t N> void show_msg(T const (&msg)[N], int seconds = 4)
{
    size_t len = std::min(7u, N);
    memcpy(text_message.msg, msg, len);
    if(len < 7) {
        memset(text_message.msg + len, 0, 7 - len);
    }
    text_message.seconds = seconds;
    text_messenger.send_message(text_message);
}

//////////////////////////////////////////////////////////////////////

void ping_task(void *)
{
    char const *TAG = "PING";

    ESP_LOGI(TAG, "Ready and waiting...");

    while(true) {
        uint x = xEventGroupWaitBits(ping_events, 7, true, false, portMAX_DELAY);
        if((x & 2) != 0) {
            pinging = false;
            show_msg("pingyes");
        }
        if((x & 4) != 0) {
            pinging = false;
            show_msg("pingerr");
        }
        if((x & 1) != 0) {
            if(!pinging) {
                pinging = true;
                esp_ping_set_target(PING_TARGET_IP_ADDRESS_COUNT, &ping_count, sizeof(uint32_t));
                esp_ping_set_target(PING_TARGET_RCV_TIMEO, &ping_timeout, sizeof(uint32_t));
                esp_ping_set_target(PING_TARGET_DELAY_TIME, &ping_delay, sizeof(uint32_t));
                esp_ping_set_target(PING_TARGET_IP_ADDRESS, &wifi::gateway.addr, sizeof(uint32_t));
                esp_ping_set_target(PING_TARGET_RES_FN, (void *)on_ping_result, sizeof(&on_ping_result));
                ping_init();
                show_msg("ping\0\0\0");
            } else {
                ESP_LOGI(TAG, "Hold your horses!");
                show_msg("not\0yet");
            }
        }
    }
}

//////////////////////////////////////////////////////////////////////

void menu_task(void *)
{
    while(true) {

        // this blocks until a button event
        uint32 b = button_update();

        // ESP_LOGI(TAG, "%08x", b);

        int presses = 0;
        int held = 0;
        int repeats = 0;
        int releases = 0;
        for(int i = 0; i < 3; ++i) {
            if(btn_held(b, i)) {
                held += 1;
            }
            if(btn_repeat(b, i)) {
                repeats += 1;
            }
            if(btn_released(b, i)) {
                releases += 1;
            }
        }

        // button 0 is the middle one
        // button 1 is the bottom one
        // button 2 is the top one

        if(btn_pressed(b, 0)) {
            control_message.brightness = min(63, control_message.brightness + 1);
            presses += 1;
        }
        if(btn_pressed(b, 1)) {
            control_message.brightness = max(14, control_message.brightness - 1);
            presses += 1;
        }

        if(btn_pressed(b, 2)) {
            presses += 1;
            xEventGroupSetBits(ping_events, 1);
        } else if(held == 2) {
            if(repeats == 0) {
                show_msg("both\0\0p");
            }
        } else if(presses == 1 && releases == 0) {
            control_messenger.send_message(control_message);
        }
    }
}

//////////////////////////////////////////////////////////////////////

extern "C" void app_main()
{
    ESP_LOGI(TAG, clear_console);

    if(nvs_flash_init() == ESP_ERR_NVS_NO_FREE_PAGES) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    messenger::init();

    // load control_message from nvs here
    control_message.brightness = 63;

    control_messenger.init(control_message);
    clock_messenger.init(clock_message);
    text_messenger.init(text_message);

    messenger::start();

    ping_events = xEventGroupCreate();
    xTaskCreate(ping_task, "ping_task", 2048, null, 5, null);

    button_init();

    xTaskCreate(menu_task, "menu_task", 2048, null, 5, null);

    // this current task becomes the clock task which sends clock_message_t messages

    // first switch on the wifi
    wifi::init();
    wifi::wait_until(wifi::CONNECTED, portMAX_DELAY);

    xTaskCreate(timezone_task, "timezone_task", 2048, null, 5, null);

    // start sntp client
    start_sntp();

    // sit in a loop sending the time to the clock display controller
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while(true) {

        struct timespec t;
        clock_gettime(CLOCK_REALTIME, &t);
        uint32 daytime = (t.tv_sec + timezone_offset()) % one_day;

        clock_message.hours = daytime / one_hour;
        clock_message.minutes = (daytime % one_hour) / 60;
        clock_message.seconds = daytime % 60;
        clock_message.milliseconds = t.tv_nsec / 1000000;
        clock_messenger.send_message(clock_message);

        vTaskDelayUntil(&xLastWakeTime, 1000 / portTICK_PERIOD_MS);
    }
}
