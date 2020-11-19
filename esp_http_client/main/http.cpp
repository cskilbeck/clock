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
#include "esp_http_client.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp8266/eagle_soc.h"
#include "esp8266/pin_mux_register.h"
#include "esp8266/gpio_struct.h"

#include "types.h"
#include "util.h"
#include "wifi.h"
#include "http.h"

//#include "jsmn.h"
//#include "message.h"
//#include "clock.h"

static char const *TAG = "HTTP";

//////////////////////////////////////////////////////////////////////
// http handler, attach a http_getter to the http client

#define HTTP_LOG ESP_LOGV

static esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    switch(evt->event_id) {

    case HTTP_EVENT_ERROR:
        HTTP_LOG(TAG, "HTTP_EVENT_ERROR");
        break;

    case HTTP_EVENT_ON_CONNECTED:
        HTTP_LOG(TAG, "HTTP_EVENT_ON_CONNECTED");
        break;

    case HTTP_EVENT_HEADER_SENT:
        HTTP_LOG(TAG, "HTTP_EVENT_HEADER_SENT");
        break;

    case HTTP_EVENT_ON_HEADER:
        HTTP_LOG(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
        break;

    case HTTP_EVENT_ON_DATA: {
        HTTP_LOG(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
        http *g = reinterpret_cast<http *>(evt->user_data);
        if(g != null) {
            size_t len = evt->data_len;
            size_t copy_size = min(len, g->capacity - g->length);
            memcpy(g->data + g->length, evt->data, copy_size);
            g->length += copy_size;
            HTTP_LOG(TAG, "got %d bytes, total so far = %d", copy_size, g->length);
        }

    } break;

    case HTTP_EVENT_ON_FINISH:
        HTTP_LOG(TAG, "HTTP_EVENT_ON_FINISH");
        break;

    case HTTP_EVENT_DISCONNECTED:
        HTTP_LOG(TAG, "HTTP_EVENT_DISCONNECTED");
        break;
    }
    return ESP_OK;
}

//////////////////////////////////////////////////////////////////////

void http::alloc(size_t max_size)
{
    data = new byte[max_size];
    capacity = max_size;
    length = 0;
}

//////////////////////////////////////////////////////////////////////

void http::release()
{
    if(data != null) {
        delete[] data;
        data = null;
        length = 0;
        capacity = 0;
    }
}

//////////////////////////////////////////////////////////////////////

esp_err_t http::get(char const *url)
{
    esp_http_client_config_t config;
    memset(&config, 0, sizeof(config));
    config.url = url;
    config.event_handler = _http_event_handler;
    config.user_data = this;
    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_err_t e = esp_http_client_perform(client);

    esp_http_client_cleanup(client);

    status_code = esp_http_client_get_status_code(client);
    content_length = esp_http_client_get_content_length(client);

    if(status_code >= 400) {
        return ESP_ERR_NOT_FOUND;
    }
    return e;
}
