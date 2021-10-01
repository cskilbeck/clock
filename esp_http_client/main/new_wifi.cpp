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
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "tcpip_adapter.h"
#include "esp_smartconfig.h"
#include "smartconfig_ack.h"
#include "lwip/inet.h"
#include "lwip/ip4_addr.h"

#include "types.h"
#include "util.h"
#include "jsmn.h"
#include "wifi.h"

//////////////////////////////////////////////////////////////////////

namespace
{
    char const *TAG = "wifi";

    EventGroupHandle_t s_wifi_event_group;

    //////////////////////////////////////////////////////////////////////

    void smartconfig_task(void *parm)
    {
        char const *TAG = "smartconfig_task";

        ESP_ERROR_CHECK(esp_smartconfig_set_type(SC_TYPE_ESPTOUCH_V2));

        smartconfig_start_config_t cfg = SMARTCONFIG_START_CONFIG_DEFAULT();

        ESP_ERROR_CHECK(esp_smartconfig_start(&cfg));

        while(1) {

            EventBits_t uxBits = xEventGroupWaitBits(s_wifi_event_group, wifi::CONNECTED | wifi::SMARTCONFIG_DONE, true, false, portMAX_DELAY);

            if((uxBits & wifi::CONNECTED) != 0) {
                ESP_LOGI(TAG, "wifi connected");
            }

            if((uxBits & wifi::SMARTCONFIG_DONE) != 0) {
                ESP_LOGI(TAG, "smartconfig complete");
                esp_smartconfig_stop();
                vTaskDelete(NULL);
            }
        }
    }

    //////////////////////////////////////////////////////////////////////

    void wifi_event_handler(void *arg, esp_event_base_t event_base, int32 event_id, void *event_data)
    {
        switch(event_id) {

        case WIFI_EVENT_STA_START: {
            wifi_config_t wifi_config;
            memset(&wifi_config, 0, sizeof(wifi_config));
            esp_err_t err = esp_wifi_get_config(ESP_IF_WIFI_STA, &wifi_config);
            if(err == ESP_OK && wifi_config.sta.ssid[0] != 0 && wifi_config.sta.password[0] != 0) {
                ESP_LOGV(TAG, "GOT CONFIG: ssid: %s, password: %s", wifi_config.sta.ssid, wifi_config.sta.password);
                ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
                ESP_ERROR_CHECK(esp_wifi_connect());
            } else {
                ESP_LOGE(TAG, "NO CONFIG found: %d:%s", err, esp_err_to_name(err));
                xTaskCreate(smartconfig_task, "smartconfig_task", 4096, NULL, 3, NULL);
            }
        } break;

        case WIFI_EVENT_STA_DISCONNECTED:
            esp_wifi_connect();
            xEventGroupClearBits(s_wifi_event_group, wifi::CONNECTED);
            break;
        }
    }

    //////////////////////////////////////////////////////////////////////

    void ip_event_handler(void *arg, esp_event_base_t event_base, int32 event_id, void *event_data)
    {
        system_event_t *event = reinterpret_cast<system_event_t *>(event_data);

        switch(event_id) {

        case IP_EVENT_STA_GOT_IP:

            wifi::ip_address = event->event_info.got_ip.ip_info.ip;
            wifi::gateway = event->event_info.got_ip.ip_info.gw;
            wifi::subnet_mask = event->event_info.got_ip.ip_info.netmask;

            ESP_LOGI(TAG, "got IP address: %s", inet_ntoa(wifi::ip_address));
            ESP_LOGI(TAG, "gateway: %s", inet_ntoa(wifi::gateway));
            ESP_LOGI(TAG, "subnet mask: %s", inet_ntoa(wifi::subnet_mask));

            xEventGroupSetBits(s_wifi_event_group, wifi::CONNECTED);
            break;

        case IP_EVENT_STA_LOST_IP:
            ESP_LOGW(TAG, "lost IP address");
            break;
        }
    }

    //////////////////////////////////////////////////////////////////////

    void smartconfig_event_handler(void *arg, esp_event_base_t event_base, int32 event_id, void *event_data)
    {
        char const *TAG = "smartconfig";

        switch(event_id) {

        case SC_EVENT_SCAN_DONE:
            ESP_LOGI(TAG, "scan done");
            break;

        case SC_EVENT_FOUND_CHANNEL:
            ESP_LOGI(TAG, "found channel");
            break;

        case SC_EVENT_GOT_SSID_PSWD: {
            ESP_LOGI(TAG, "got SSID and password");

            smartconfig_event_got_ssid_pswd_t *evt = (smartconfig_event_got_ssid_pswd_t *)event_data;

            wifi_config_t wifi_config;
            uint8 ssid[33] = { 0 };
            uint8 password[65] = { 0 };
            uint8 rvd_data[33] = { 0 };

            bzero(&wifi_config, sizeof(wifi_config_t));

            memcpy(wifi_config.sta.ssid, evt->ssid, sizeof(wifi_config.sta.ssid));
            memcpy(wifi_config.sta.password, evt->password, sizeof(wifi_config.sta.password));

            wifi_config.sta.bssid_set = evt->bssid_set;

            if(wifi_config.sta.bssid_set) {
                memcpy(wifi_config.sta.bssid, evt->bssid, sizeof(wifi_config.sta.bssid));
            }

            memcpy(ssid, evt->ssid, sizeof(evt->ssid));
            memcpy(password, evt->password, sizeof(evt->password));

            ESP_LOGI(TAG, "SSID:%s", ssid);
            ESP_LOGI(TAG, "PASSWORD:%s", password);

            if(evt->type == SC_TYPE_ESPTOUCH_V2) {
                ESP_ERROR_CHECK(esp_smartconfig_get_rvd_data(rvd_data, sizeof(rvd_data)));
                ESP_LOGI(TAG, "RVD_DATA:%s", rvd_data);
            }

            ESP_ERROR_CHECK(esp_wifi_disconnect());
            ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
            ESP_ERROR_CHECK(esp_wifi_connect());

        } break;

        case SC_EVENT_SEND_ACK_DONE:
            xEventGroupSetBits(s_wifi_event_group, wifi::SMARTCONFIG_DONE);
            break;
        }
    }

}    // namespace

//////////////////////////////////////////////////////////////////////

namespace wifi
{
    ip4_addr_t ip_address;
    ip4_addr_t gateway;
    ip4_addr_t subnet_mask;

    void init(void)
    {
        tcpip_adapter_init();
        s_wifi_event_group = xEventGroupCreate();

        ESP_ERROR_CHECK(esp_event_loop_create_default());

        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

        ESP_ERROR_CHECK(esp_wifi_init(&cfg));
        ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
        ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
        ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, ESP_EVENT_ANY_ID, &ip_event_handler, NULL));
        ESP_ERROR_CHECK(esp_event_handler_register(SC_EVENT, ESP_EVENT_ANY_ID, &smartconfig_event_handler, NULL));
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_start());
    }

    //////////////////////////////////////////////////////////////////////

    uint32 wait_until(uint32 bits, int timeout)
    {
        return xEventGroupWaitBits(s_wifi_event_group, bits, true, false, portMAX_DELAY);
    }
}    // namespace wifi
