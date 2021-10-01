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
#include "esp_log.h"
#include "esp_system.h"
#include "esp_http_client.h"
#include "lwip/inet.h"
#include "lwip/ip4_addr.h"
#include <time.h>

#include "types.h"
#include "util.h"
#include "wifi.h"
#include "http.h"
#include "jsmn.h"
#include "timezone.h"

static char const *TAG = "TIMEZONE";

//////////////////////////////////////////////////////////////////////

static int fi(float f)
{
    return (int)f;
}

static uint ff(float f)
{
    return (uint)std::abs(f * 100000.0f) % 100000;
}

#define FLOAT "%d.%05u"

//////////////////////////////////////////////////////////////////////

static int gmt_offset = 0;

#define MAX_HTTP_RECV_BUFFER 512

// for using different IP getter APIs
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

// toggle the ip APIs until one works
ip_json *ip_apis[2] = { &ip_api_1, &ip_api_2 };
int which_api = 1;

enum
{
    key_ip = 0,
    key_lat,
    key_lon
};

static char const timezone_api_url_fmt[] = "http://api.timezonedb.com/v2.1/get-time-zone?key=VV0F65261GPD&by=position&lat=" FLOAT "&lng=" FLOAT "&format=json";
int const json_token_max = 128;
static jsmntok_t tokens[json_token_max]; /* We expect no more than 128 tokens */
static char url_buffer[countof(timezone_api_url_fmt) + 40];

//////////////////////////////////////////////////////////////////////
// get the current timezone for a given geolocation

static bool get_timezone(float lat, float lon)
{
    bool success = false;

    ESP_LOGI(TAG, "Update timezone");
    int buffer_len = sprintf(url_buffer, timezone_api_url_fmt, fi(lat), ff(lat), fi(lon), ff(lon));
    ESP_LOGV(TAG, "CLOCK WOULD USE %s (%d)", url_buffer, buffer_len);

    http h;
    h.alloc(1024);
    scoped[&]()
    {
        h.release();
    };
    if(h.get(url_buffer) != ESP_OK) {
        ESP_LOGE(TAG, "Get %s failed!", url_buffer);
        return false;
    }

    ESP_LOGV(TAG, "Got %d from timezonedb", h.length);
    jsmn_parser p;
    jsmn_init(&p);
    bool got_offset = false;
    bool got_abbreviation = false;
    char abbrev[16];
    char next_abbrev[16];
    char const *b = reinterpret_cast<char const *>(h.data);    // borrow to make this readable
    int token_count = jsmn_parse(&p, b, h.length, tokens, json_token_max);
    for(int i = 1; i < token_count; ++i) {
        jsmntok_t const &key = tokens[i];
        jsmntok_t const &val = tokens[i + 1];
        if(key.eq("status", b)) {
            if(!val.eq("OK", b)) {
                ESP_LOGE(TAG, "TimeZoneDB API returns %.*s", val.len(), val.ptr(reinterpret_cast<char const *>(h.data)));
                break;
            }
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
        success = false;
    }
    return success;
}

//////////////////////////////////////////////////////////////////////
// get the ip address, latitude and longitude of this esp12
// note geolocation can be wrong!
// user should have an option to correct it with the buttons

static esp_err_t get_location(float *ip_lat_p, float *ip_lon_p)
{
    ip_json *api = ip_apis[which_api];
    which_api = 1 - which_api;
    http h;
    h.alloc(512);
    scoped[&]()
    {
        h.release();
    };
    if(h.get(api->url) != ESP_OK) {
        ESP_LOGE(TAG, "Get %s failed!", api->url);
        return false;
    }
    jsmn_parser p;

    bool got_ip_address = false;
    int got_lat_lon = 0;
    char ip_addr[16];

    float ip_lat = 0;    // what we got from ip-api
    float ip_lon = 0;

    jsmn_init(&p);
    char const *b = reinterpret_cast<char const *>(h.data);
    int token_count = jsmn_parse(&p, b, h.length, tokens, json_token_max);
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
    ESP_LOGI(TAG, "IP: %s, Lat: " FLOAT ", Lon: " FLOAT, ip_addr, fi(ip_lat), ff(ip_lat), fi(ip_lon), ff(ip_lon));
    *ip_lat_p = ip_lat;
    *ip_lon_p = ip_lon;
    return ESP_OK;
}

//////////////////////////////////////////////////////////////////////

int timezone_offset()
{
    return gmt_offset;
}


//////////////////////////////////////////////////////////////////////

void timezone_task(void *)
{
    float ip_lat = 0;    // what we got from ip-api
    float ip_lon = 0;

    // get ip address location

    while(get_location(&ip_lat, &ip_lon) != ESP_OK) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    do {

        // get timezone for location

        while(get_timezone(ip_lat, ip_lon) != ESP_OK) {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }

        // wait until 10 seconds after midnight

        constexpr uint32 one_day = 24 * 60 * 60;
        struct timespec t;
        clock_gettime(CLOCK_REALTIME, &t);
        struct tm tod;
        uint32 daytime = (t.tv_sec + timezone_offset()) % one_day;
        int when_to_wake = t.tv_sec + (one_day - daytime) + 10;
        vTaskDelay((when_to_wake * 1000) / portTICK_PERIOD_MS - xTaskGetTickCount());

    } while(true);
}
