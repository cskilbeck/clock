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
#include "driver/gpio.h"
#include "esp8266/eagle_soc.h"
#include "esp8266/pin_mux_register.h"
#include "esp8266/gpio_struct.h"

#include "types.h"
#include "util.h"
#include "wifi.h"
#include "jsmn.h"
#include "message.h"
#include "clock.h"

//////////////////////////////////////////////////////////////////////

#define MAX_HTTP_RECV_BUFFER 512
#define STM32_MSG_BUF_SIZE 128

#define BTN1_POS GPIO_NUM_13
#define BTN2_POS GPIO_NUM_14
#define BTN1_MASK (1 << BTN1_POS)
#define BTN2_MASK (1 << BTN2_POS)

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

uint16 crc16(byte const *p, size_t len)
{
    assert(len != 0);
    uint16 crc = 0xffff;
    byte const *e = p + len;
    do {
        uint16 x = (crc >> 8) ^ *p;
        x ^= x >> 4;
        crc = (crc << 8) ^ (x << 12) ^ (x << 5) ^ x;
    } while(++p < e);
    return crc;
}

//////////////////////////////////////////////////////////////////////

inline size_t get_message_length(uint16 signature)
{
    switch(signature) {
    case clock_message_signature:
        return sizeof(clock_message_t);
    case control_message_signature:
        return sizeof(control_message_t);
    default:
        return 0;
    }
}

//////////////////////////////////////////////////////////////////////

template <typename T> void set_crc(T &msg)
{
    msg.crc = crc16(reinterpret_cast<byte const *>(&msg) + 4, sizeof(T) - 4);
}

EventGroupHandle_t message_events;

enum event_bits : uint32
{
    send_control = 1,    // please send the control message
    sent_control = 2,    // I sent it
    send_clock = 4,      // please send the clock message
    sent_clock = 8       // I sent it
};

control_message_t control_message;
clock_message_t clock_msg;

//////////////////////////////////////////////////////////////////////

constexpr uint32 one_second = 1;
constexpr uint32 one_minute = one_second * 60;
constexpr uint32 one_hour = one_minute * 60;
constexpr uint32 one_day = one_hour * 24;

void send_time_to_stm32()
{
    struct timespec t;
    clock_gettime(CLOCK_REALTIME, &t);
    t.tv_sec += clock_timezone_offset();

    uint32 daytime = t.tv_sec % one_day;
    uint32 hours = daytime / one_hour;
    uint32 minutes = (daytime % one_hour) / 60;
    uint32 seconds = daytime % 60;

    if(hours > 12) {
        hours -= 12;
    }
    if(hours == 0) {
        hours = 12;
    }

    if(hours > 9) {
        clock_msg.digit[0] = '0' + (hours / 10);
    } else {
        clock_msg.digit[0] = 0;
    }

    clock_msg.digit[1] = '0' + (hours % 10);

    clock_msg.digit[2] = '0' + (minutes / 10);
    clock_msg.digit[3] = '0' + (minutes % 10);

    // clock_msg.digit[4] = '0' + (seconds / 10);
    // clock_msg.digit[5] = '0' + (seconds % 10);
    clock_msg.digit[4] = 0;
    clock_msg.digit[5] = 0;

    clock_msg.digit[6] = 0;    //'A';

    clock_msg.seconds = seconds;

    xEventGroupSync(message_events, send_clock, sent_clock, portMAX_DELAY);
}

//////////////////////////////////////////////////////////////////////

void button_task(void *)
{
    static char const *TAG = "BTN";

    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.pin_bit_mask = BTN1_MASK | BTN2_MASK;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    int button_state = 0;
    while(true) {
        uint32 gpio = GPIO.in;
        int btn1 = 1 - ((gpio & BTN1_MASK) >> BTN1_POS);
        int btn2 = 2 - ((gpio & BTN2_MASK) >> (BTN2_POS - 1));
        int new_button_state = btn1 | btn2;
        int button_change = new_button_state ^ button_state;
        button_state = new_button_state;
        int button_press = button_change & button_state;
        int button_release = button_change & ~button_state;

        if(button_press & 2) {
            control_message.brightness = min(63, control_message.brightness + 1);
        }
        if(button_press & 1) {
            control_message.brightness = max(1, control_message.brightness - 1);
        }

        if(button_press || button_release) {
            // whole big menu handler here
            ESP_LOGI(TAG, "press: %d, release: %d, buttons = %d", button_press, button_release, button_state);
            xEventGroupSync(message_events, send_control, sent_control, portMAX_DELAY);
        }
        vTaskDelay(1);    // delay for 10ms
    }
}

//////////////////////////////////////////////////////////////////////

void start_sntp()
{
    static char const *TAG = "SNTP";

    ESP_LOGI(TAG, "Init SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, const_cast<char *>("pool.ntp.org"));
    wifi_wait_until(wifi_event_connected, portMAX_DELAY);
    sntp_init();
}

//////////////////////////////////////////////////////////////////////

template <typename T> void send_message(T &msg, uint32 got_bits, uint32 send_bit, uint32 sent_bit)
{
    if(got_bits & send_bit) {
        constexpr int len = sizeof(T) + 4;
        byte buffer[len];
        buffer[0] = T::signature;
        buffer[1] = sizeof(T);
        memcpy(buffer + 2, &msg, sizeof(msg));
        uint16 crc = crc16((byte const *)&msg, sizeof(msg));
        buffer[len - 2] = crc & 0xff;
        buffer[len - 1] = crc >> 8;
        uart_write_bytes(UART_NUM_1, (char const *)buffer, len);
        ESP_LOGV(TAG, "SENT: %d (%02x%02x ... %02x%02x)", len, buffer[0], buffer[1], buffer[len - 2], buffer[len - 1]);
        xEventGroupSetBits(message_events, sent_bit);
    }
}

void message_task(void *)
{
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
        uint32 requests = xEventGroupWaitBits(message_events, send_control | send_clock, pdTRUE, pdFALSE, portMAX_DELAY);
        send_message(control_message, requests, send_control, sent_control);
        send_message(clock_msg, requests, send_clock, sent_clock);
    }
}

//////////////////////////////////////////////////////////////////////

void do_clock()
{
    // this event group is used to request and notify that
    // messages are sent to the stm32 over the uart
    // two bits per message so up to 16 types of message, just 2 used for now (control, clock)
    message_events = xEventGroupCreate();

    // this task does the actual sending
    xTaskCreate(message_task, "message_sender", 2048, null, 20, null);

    // the button handler task might send some control messages
    xTaskCreate(button_task, "button_task", 2048, NULL, 10, NULL);

    // add other message senders here, can't think what they might be

    // then this current task becomes the clock task which sends clock_message_t messages
    initialise_wifi();

    clock_state = clock_state_idle;
    ESP_LOGI(TAG, "Clock task begins");
    wifi_wait_until(wifi_event_connected, portMAX_DELAY);

    clock_state = clock_state_initializing;
    ESP_LOGI(TAG, "Wifi connected, getting time and timezone");

    // init sntp
    start_sntp();

    if(get_location() != ESP_OK) {
        ESP_LOGE(TAG, "Can't get IP/Location/Timezone");
    }

    ESP_LOGI(TAG, "Clock init complete");
    clock_state = clock_state_ready;

    bool notify = true;
    int sntp_patience = 30;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while(true) {
        if(is_sntp_working()) {
            send_time_to_stm32();
            if(notify) {
                ESP_LOGI(TAG, "Got SNTP connection, clock is running");
                notify = false;
            }
        } else {
            ESP_LOGI(TAG, "Waiting for SNTP");
            sntp_patience -= 1;
            if(sntp_patience == 0) {

                // if SNTP doesn't start after about a minute then
                // we've probably been connected to a duff server
                // so reboot the SNTP client and hope for a different one
                ESP_LOGI(TAG, "SNTP patience exhausted, restarting it");
                sntp_stop();
                sntp_init();
                sntp_patience = 60;
            }
        }
        // wait until some time before next second so next one arrives on the second
        vTaskDelayUntil(&xLastWakeTime, 1000 / portTICK_PERIOD_MS);
    }
}
