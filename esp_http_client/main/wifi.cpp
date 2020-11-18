//////////////////////////////////////////////////////////////////////

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <cstddef>
#include <utility>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/hw_timer.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "tcpip_adapter.h"
#include "esp_smartconfig.h"
#include "smartconfig_ack.h"

#include "types.h"
#include "util.h"
#include "jsmn.h"
#include "wifi.h"

//////////////////////////////////////////////////////////////////////

namespace
{
    const char *TAG = "wifi_client";

    EventGroupHandle_t wifi_event_group;

    const int CONNECTED_BIT = BIT0;
    const int ESPTOUCH_DONE_BIT = BIT1;

    //////////////////////////////////////////////////////////////////////

    static void sc_callback(smartconfig_status_t status, void *pdata)
    {
        switch(status) {
        case SC_STATUS_WAIT:
            ESP_LOGI(TAG, "SC_STATUS_WAIT");
            break;
        case SC_STATUS_FIND_CHANNEL:
            ESP_LOGI(TAG, "SC_STATUS_FINDING_CHANNEL");
            break;
        case SC_STATUS_GETTING_SSID_PSWD:
            ESP_LOGI(TAG, "SC_STATUS_GETTING_SSID_PSWD");
            break;
        case SC_STATUS_LINK: {
            ESP_LOGI(TAG, "SC_STATUS_LINK");
            wifi_config_t *wifi_config = reinterpret_cast<wifi_config_t *>(pdata);
            ESP_LOGI(TAG, "SSID:%s", wifi_config->sta.ssid);
            ESP_LOGV(TAG, "PASSWORD:%s", wifi_config->sta.password);
            ESP_ERROR_CHECK(esp_wifi_disconnect());
            ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, wifi_config));
            ESP_ERROR_CHECK(esp_wifi_connect());
        } break;
        case SC_STATUS_LINK_OVER:
            ESP_LOGI(TAG, "SC_STATUS_LINK_OVER");
            if(pdata != NULL) {
                sc_callback_data_t *sc_callback_data = (sc_callback_data_t *)pdata;
                switch(sc_callback_data->type) {
                case SC_ACK_TYPE_ESPTOUCH:
                    ESP_LOGI(TAG, "Phone ip: %d.%d.%d.%d", sc_callback_data->ip[0], sc_callback_data->ip[1], sc_callback_data->ip[2], sc_callback_data->ip[3]);
                    ESP_LOGI(TAG, "TYPE: ESPTOUCH");
                    break;
                case SC_ACK_TYPE_AIRKISS:
                    ESP_LOGI(TAG, "TYPE: AIRKISS");
                    break;
                default:
                    ESP_LOGE(TAG, "TYPE: ERROR");
                    break;
                }
            }
            xEventGroupSetBits(wifi_event_group, ESPTOUCH_DONE_BIT);
            break;
        default:
            break;
        }
    }

    //////////////////////////////////////////////////////////////////////

    void wifi_client_task(void *)
    {
        while(true) {
            vTaskDelay(500);
        }
    }

    //////////////////////////////////////////////////////////////////////

    void smartconfig_example_task(void *parm)
    {
        EventBits_t uxBits;
        ESP_ERROR_CHECK(esp_smartconfig_set_type(SC_TYPE_ESPTOUCH_AIRKISS));
        ESP_ERROR_CHECK(esp_smartconfig_start(sc_callback));
        while(1) {
            uxBits = xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT | ESPTOUCH_DONE_BIT, true, false, portMAX_DELAY);
            if(uxBits & CONNECTED_BIT) {
                ESP_LOGI(TAG, "WiFi Connected to ap");
            }
            if(uxBits & ESPTOUCH_DONE_BIT) {
                ESP_LOGI(TAG, "smartconfig over");
                esp_smartconfig_stop();
                xTaskCreate(wifi_client_task, "wifi_client_task", 2048, NULL, 1, NULL);
                vTaskDelete(NULL);
            }
        }
    }

    //////////////////////////////////////////////////////////////////////

    static esp_err_t event_handler(void *ctx, system_event_t *event)
    {
        // For accessing reason codes in case of disconnection
        system_event_info_t *info = &event->event_info;

        switch(event->event_id) {
        case SYSTEM_EVENT_STA_START: {
            wifi_config_t wifi_config;
            memset(&wifi_config, 0, sizeof(wifi_config));
            esp_err_t r = esp_wifi_get_config(ESP_IF_WIFI_STA, &wifi_config);
            if(r == ESP_OK && wifi_config.sta.ssid[0] != 0 && wifi_config.sta.password[0] != 0) {
                ESP_LOGV(TAG, "GOT CONFIG: ssid: %s, password: %s", wifi_config.sta.ssid, wifi_config.sta.password);
                ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
                ESP_ERROR_CHECK(esp_wifi_connect());
                xTaskCreate(wifi_client_task, "wifi_client_task", 2048, NULL, 1, NULL);
            } else {
                ESP_LOGE(TAG, "NO CONFIG found: %d:%s", r, esp_err_to_name(r));
                xTaskCreate(smartconfig_example_task, "smartconfig_example_task", 4096, NULL, 3, NULL);
            }
        } break;
        case SYSTEM_EVENT_STA_GOT_IP:
            xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            ESP_LOGE(TAG, "Disconnect reason : %d", info->disconnected.reason);
            if(info->disconnected.reason == WIFI_REASON_BASIC_RATE_NOT_SUPPORT) {
                // Switch to 802.11 bgn mode
                ESP_LOGE(TAG, "BASIC_RATE_NOT_SUPPORTED, switching to BGN");
                esp_wifi_set_protocol(ESP_IF_WIFI_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
            }
            esp_wifi_connect();
            xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
            break;
        default:
            break;
        }
        return ESP_OK;
    }

}    // namespace

//////////////////////////////////////////////////////////////////////

void initialise_wifi(void)
{
    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
}

//////////////////////////////////////////////////////////////////////

uint32 wifi_wait_until(uint32 bits, int timeout)
{
    return xEventGroupWaitBits(wifi_event_group, bits, pdFALSE, pdTRUE, timeout);
}
