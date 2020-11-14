//////////////////////////////////////////////////////////////////////

#include <string.h>
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>
#include <utility>

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

#include "types.h"
#include "util.h"
#include "wifi.h"
#include "jsmn.h"
#include "message.h"
#include "clock.h"

//////////////////////////////////////////////////////////////////////

#define MAX_HTTP_RECV_BUFFER 512
#define STM32_MSG_BUF_SIZE 128

static char const *TAG = "CLOCK";

struct http_getter
{
    virtual void on_data(byte const *data, size_t len) = 0;
};

static int gmt_offset = 0;

static clock_state_enum clock_state = clock_state_idle;
static EventGroupHandle_t clock_event_handle;

struct ip_json
{
    char const *url;
    char const **key;
    int num_keys;

    int get_key_id(char const *v, int len)
    {
        for(int i = 0; i < num_keys; ++i) {
            int l = strlen(key[i]);
            if(len == l && strncmp(v, key[i], len) == 0) {
                return i;
            }
        }
        return -1;
    }
};

char const *ip_api_1_key[3] = { "query", "lat", "lon" };
char const *ip_api_2_key[3] = { "ip", "latitude", "longitude" };

ip_json ip_api_1{ "http://ip-api.com/json?fields=lat,lon,query", ip_api_1_key, countof(ip_api_1_key) };
ip_json ip_api_2{ "http://api.ipapi.com/api/check?access_key=825b0130e3baced2cdd5eeb07a73953b&fields=ip,latitude,longitude", ip_api_2_key, countof(ip_api_2_key) };

// toggle the ip ones until one works
ip_json *ip_apis[2] = { &ip_api_1, &ip_api_2 };
int which_api = 1;

enum
{
    key_ip = 0,
    key_lat,
    key_lon
};

static char const timezone_api_url_fmt[] = "http://api.timezonedb.com/v2.1/get-time-zone?key=VV0F65261GPD&by=position&lat=%f&lng=%f&format=json";
int const json_token_max = 128;
static jsmntok_t tokens[json_token_max]; /* We expect no more than 128 tokens */
static char url_buffer[countof(timezone_api_url_fmt) + 40];

//////////////////////////////////////////////////////////////////////

int clock_timezone_offset()
{
    return gmt_offset;
}

//////////////////////////////////////////////////////////////////////

clock_state_enum get_clock_state()
{
    return clock_state;
}

//////////////////////////////////////////////////////////////////////
// http_getter which just accumulates the content into a buffer

struct http_get : http_getter
{
    byte *data = null;
    size_t capacity = 0;
    size_t length = 0;

    virtual ~http_get()
    {
        release();
    }

    void init(size_t max_size)
    {
        data = new byte[max_size];
        capacity = max_size;
        length = 0;
    }

    void release()
    {
        if(data != null) {
            delete[] data;
            data = null;
        }
    }

    void on_data(byte const *incoming_data, size_t len) override
    {
        size_t copy_size = min(len, capacity - length);
        memcpy(data + length, incoming_data, copy_size);
        length += copy_size;
        ESP_LOGV(TAG, "got %d bytes, total so far = %d", len, length);
    }
};

//////////////////////////////////////////////////////////////////////
// http handler, attach a http_getter to the http client

esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    switch(evt->event_id) {

    case HTTP_EVENT_ERROR:
        ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
        break;

    case HTTP_EVENT_ON_CONNECTED:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
        break;

    case HTTP_EVENT_HEADER_SENT:
        ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
        break;

    case HTTP_EVENT_ON_HEADER:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
        break;

    case HTTP_EVENT_ON_DATA: {
        ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
        http_getter *g = reinterpret_cast<http_getter *>(evt->user_data);
        if(g != null) {
            g->on_data(reinterpret_cast<byte *>(evt->data), evt->data_len);
        }

    } break;

    case HTTP_EVENT_ON_FINISH:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
        break;

    case HTTP_EVENT_DISCONNECTED:
        ESP_LOGD(TAG, "HTTP_EVENT_DISCONNECTED");
        break;
    }
    return ESP_OK;
}

//////////////////////////////////////////////////////////////////////
// setup sntp

void sntp_task(void *)
{
    while(true) {
        vTaskDelay((1000 * 60 * 60) / portTICK_PERIOD_MS);
    }
}

//////////////////////////////////////////////////////////////////////
// format a time_t into a string

void time_to_str(time_t t, char *buf)
{
    int minutes = (t / 60) % 60;
    int hours = (t / 3600) % 24;
    char const *am_pm = "PM";
    if(hours < 12) {
        am_pm = "AM";
    }
    hours %= 12;
    if(hours == 0) {
        hours = 12;
    }
    sprintf(buf, "%2d:%02d %s", hours, minutes, am_pm);
}

//////////////////////////////////////////////////////////////////////
// get current time as a string

char const *current_time(char *buffer)
{
    time_t now;
    time(&now);
    now += clock_timezone_offset();
    time_to_str(now, buffer);
    return buffer;
}

//////////////////////////////////////////////////////////////////////
// send STM32 message with current time & options

void clock_draw(uint32 fg_color, uint32 bg_color)
{
    char buffer[40];
    char const *time_string = "--:--";
    if(get_clock_state() == clock_state_ready) {
        current_time(buffer);
        time_string = buffer;
    }
    ESP_LOGI(TAG, "TIME: %s", time_string);
}

//////////////////////////////////////////////////////////////////////
// get the current timezone for a given geolocation

void clock_update_timezone(float lat, float lon)
{
    ESP_LOGI(TAG, "Update timezone");
    int buffer_len = sprintf(url_buffer, timezone_api_url_fmt, lat, lon);
    ESP_LOGV(TAG, "CLOCK WOULD USE %s (%d)", url_buffer, buffer_len);
    http_get getter;
    getter.init(1024);
    esp_http_client_config_t config;
    memset(&config, 0, sizeof(config));
    config.url = url_buffer;
    config.event_handler = _http_event_handler;
    config.user_data = &getter;
    esp_http_client_handle_t client = esp_http_client_init(&config);

    bool success = false;

    esp_err_t err = esp_http_client_perform(client);

    if(err == ESP_OK) {
        ESP_LOGV(TAG, "Got %d from timezonedb", getter.length);
        jsmn_parser p;
        jsmn_init(&p);
        bool got_offset = false;
        bool got_abbreviation = false;
        char abbrev[16];
        char next_abbrev[16];
        char const *b = reinterpret_cast<char const *>(getter.data);    // borrow to make this readable
        int token_count = jsmn_parse(&p, b, getter.length, tokens, json_token_max);
        for(int i = 1; i < token_count; ++i) {
            jsmntok_t const &key = tokens[i];
            jsmntok_t const &val = tokens[i + 1];
            if(key.eq("status", b)) {
                success = val.eq("OK", b);
                ESP_LOGI(TAG, "TimezoneDB success");
                i += 1;
            } else if(key.eq("abbreviation", b)) {
                val.get_str(abbrev, b);
                ESP_LOGI(TAG, "ABBREV %s", abbrev);
                got_abbreviation = true;
                i += 1;
            } else if(key.eq("nextAbbreviation", b)) {
                val.get_str(next_abbrev, b);
                ESP_LOGI(TAG, "NEXT ABBREV %s", next_abbrev);
                i += 1;
            } else if(key.eq("gmtOffset", b)) {
                val.get_int(gmt_offset, b);
                ESP_LOGI(TAG, "GMT offset %d", gmt_offset);
                got_offset = true;
                i += 1;
            }
        }
        if(got_offset && got_abbreviation) {
            char tz_buffer[20];
            int minutes = gmt_offset / 60;
            int hours = minutes / 60;
            minutes %= 60;
            sprintf(tz_buffer, "<%.5s>%-.2d:%02d", abbrev, -hours, -minutes);
            setenv("TZ", tz_buffer, 1);
            tzset();
            ESP_LOGV(TAG, "TZ=%s", tz_buffer);
        } else {
            ESP_LOGW(TAG, "HUH? TZ not got");
        }
    }
    (void)success;
}

//////////////////////////////////////////////////////////////////////
// get the ip address, latitude and longitude of this esp12
// note geolocation can be wrong!
// user should have an option to correct it with the buttons

static esp_err_t get_location()
{
    ip_json *api = ip_apis[which_api];
    which_api = 1 - which_api;
    http_get getter;
    getter.init(512);
    esp_http_client_config_t config;
    memset(&config, 0, sizeof(config));
    config.url = api->url;    //"http://ip-api.com/json?fields=lat,lon,query";
    config.event_handler = _http_event_handler;
    config.user_data = &getter;
    esp_http_client_handle_t client = esp_http_client_init(&config);

    defer(esp_http_client_cleanup(client));

    esp_err_t err = esp_http_client_perform(client);
    if(err != ESP_OK) {
        return err;
    }
    int status_code = esp_http_client_get_status_code(client);
    int content_length = esp_http_client_get_content_length(client);
    ESP_LOGI(TAG, "HTTP GET Status = %d, content_length = %d", status_code, content_length);
    ESP_LOGI(TAG, "CONTENT: %.*s", getter.length, getter.data);

    if(status_code >= 400) {
        ESP_LOGE(TAG, "HTTP GET request failed: %d", err);
        return ESP_ERR_NOT_FOUND;
    }

    jsmn_parser p;

    bool got_ip_address = false;
    int got_lat_lon = 0;
    char ip_addr[16];

    float ip_lat = 0;    // what we got from ip-api
    float ip_lon = 0;

    jsmn_init(&p);
    char const *b = reinterpret_cast<char const *>(getter.data);
    int token_count = jsmn_parse(&p, b, getter.length, tokens, json_token_max);
    if(token_count == 0 || tokens[0].type != JSMN_OBJECT) {
        return ESP_ERR_NOT_FOUND;
    }

    for(int i = 1; i < token_count; i += 2) {
        jsmntok_t const &key = tokens[i];
        jsmntok_t const &val = tokens[i + 1];
        ESP_LOGV(TAG, "KEY: %.*s, VAL: %.*s", key.len(), b + key.start, val.len(), b + val.start);
        switch(api->get_key_id(b + key.start, key.len())) {
        case key_ip:
            got_ip_address = true;
            val.get_str(ip_addr, b);
            break;
        case key_lat:
            val.get_flt(ip_lat, b);
            got_lat_lon |= 1;
            break;
        case key_lon:
            val.get_flt(ip_lon, b);
            got_lat_lon |= 2;
            break;
        default:
            ESP_LOGE(TAG, "Unknown key: %.*s", val.len(), b + val.start);
            break;
        }
    }

    if(got_lat_lon != 3 || !got_ip_address) {
        ESP_LOGE(TAG, "Error getting geo location: %02x", got_lat_lon);
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "IP: %s, Lat: %f, Lon: %f", ip_addr, ip_lat, ip_lon);
    clock_update_timezone(ip_lat, ip_lon);
    return ESP_OK;
}

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
// calculate a crc for an stm32 message

uint16 crc16(message_t const *m, size_t l)
{
    uint16 crc = 0xffff;
    byte const *p = reinterpret_cast<byte const *>(m);
    byte const *e = p + l;
    for(; p < e; ++p) {
        uint16 x = (crc >> 8) ^ *p;
        x ^= x >> 4;
        crc = (crc << 8) ^ (x << 12) ^ (x << 5) ^ x;
    }
    return crc;
}

//////////////////////////////////////////////////////////////////////

constexpr uint32 one_second = 1;
constexpr uint32 one_minute = one_second * 60;
constexpr uint32 one_hour = one_minute * 60;
constexpr uint32 one_day = one_hour * 24;

message_t msg;

void send_time_to_stm32()
{
    struct timespec t;
    clock_gettime(CLOCK_REALTIME, &t);
    t.tv_sec += clock_timezone_offset();

    uint32 daytime = t.tv_sec % one_day;
    uint32 hours = daytime / one_hour;
    uint32 minutes = (daytime % one_hour) / 60;
    uint32 seconds = daytime % 60;

    msg.digit[0] = '0' + (hours / 10);
    msg.digit[1] = '0' + (hours % 10);
    msg.digit[2] = '0' + (minutes / 10);
    msg.digit[3] = '0' + (minutes % 10);
    msg.digit[4] = '0' + (seconds / 10);
    msg.digit[5] = '0' + (seconds % 10);
    msg.digit[6] = 'A';
    msg.seconds = seconds;
    msg.signature = 0xDA5C;

    msg.crc = crc16(&msg, sizeof(msg) - 2);    // crc must be last 16 bits of msg struct

    uart_write_bytes(UART_NUM_1, reinterpret_cast<char const *>(&msg), sizeof(message_t));
}

//////////////////////////////////////////////////////////////////////

void do_clock()
{
    initialise_wifi();

    clock_state = clock_state_idle;
    ESP_LOGI(TAG, "Clock task begins");
    wifi_wait_until(wifi_event_connected, portMAX_DELAY);

    clock_state = clock_state_initializing;
    ESP_LOGI(TAG, "Wifi connected, getting time and timezone");

    // init sntp
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, const_cast<char *>("pool.ntp.org"));
    wifi_wait_until(wifi_event_connected, portMAX_DELAY);
    sntp_init();

    if(get_location() != ESP_OK) {
        ESP_LOGE(TAG, "Can't get IP/Location/Timezone");
    }

    ESP_LOGI(TAG, "Clock init complete");
    clock_state = clock_state_ready;

    // setup uart1 for sending stm32 messages
    uart_config_t uart_config = { .baud_rate = 115200,
                                  .data_bits = UART_DATA_8_BITS,
                                  .parity = UART_PARITY_DISABLE,
                                  .stop_bits = UART_STOP_BITS_1,
                                  .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
                                  .rx_flow_ctrl_thresh = 0 };
    uart_param_config(UART_NUM_1, &uart_config);
    uart_driver_install(UART_NUM_1, STM32_MSG_BUF_SIZE * 2, 0, 0, NULL, 0);

    while(true) {
        if(is_sntp_working()) {
            send_time_to_stm32();
        } else {
            ESP_LOGI(TAG, "Waiting for SNTP");
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
