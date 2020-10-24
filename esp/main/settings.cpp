//////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <string.h>
#include <cstdint>
#include <stdarg.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_log.h"

#include "nvs_flash.h"
#include "nvs.h"
#include "jsmn.h"

#include "util.h"
#include "json.h"
#include "debug.h"
#include "jsmn.h"
#include "settings.h"

//////////////////////////////////////////////////////////////////////

static char const *TAG = "settings";

QueueHandle_t settings_queue;
settings_t global_settings;

//////////////////////////////////////////////////////////////////////

struct effect_name
{
    char const *name;
    int effect;
};

//////////////////////////////////////////////////////////////////////

struct speed_action_t
{
    char const *name;
    int mul;
    int add;
};

//////////////////////////////////////////////////////////////////////

// clang-format off
speed_action_t speed_actions[] = {
    { "normal speed", 0, 50 },
    { "fastest", 0, 100 },
    { "slowest", 0, 10 },
    { "slow", 0, 30 },
    { "fast", 0, 80 },
    { "slower", 1, -20 },
    { "faster", 1, 20 }
};

effect_name effect_names[] = {
    { "solid", settings_t::solid_effect },
    { "glow", settings_t::glow_effect },
    { "twinkle", settings_t::twinkle_effect },
    { "sparkle", settings_t::sparkle_effect },
    { "flash", settings_t::flash_effect },
    { "alternate", settings_t::flash_effect },
    { "pulse", settings_t::pulse_effect },
    { "fade", settings_t::fade_effect } 
};
// clang-format on

//////////////////////////////////////////////////////////////////////

speed_action_t const *get_speed_action(char const *name, int json_len)
{
    for(auto const &speed_action : speed_actions) {
        if(strncmp(name, speed_action.name, json_len) == 0) {
            return &speed_action;
        }
    }
    return nullptr;
}

//////////////////////////////////////////////////////////////////////

bool get_effect(jsmntok_t token, char const *json_text, int &effect)
{
    if(token.type != JSMN_STRING) {
        ESP_LOGW(TAG, "EFFECT VALUE IS NOT A STRING");
        return false;
    }
    char const *src = json_text + token.start;
    int json_len = json::len(token);
    for(effect_name const &e : effect_names) {
        if(strncmp(e.name, src, json_len) == 0) {
            effect = e.effect;
            return true;
        }
    }
    ESP_LOGW(TAG, "CAN'T FIND %.*s", json::len(token), json_text + token.start);
    return false;
}

//////////////////////////////////////////////////////////////////////

char const *get_effect_name(int effect)
{
    for(effect_name const &e : effect_names) {
        if(e.effect == effect) {
            return e.name;
        }
    }
    return "unknown_effect!?";
}

//////////////////////////////////////////////////////////////////////

void settings_t::print(char const *banner)
{
    ESP_LOGW(TAG, "%s", banner);
    ESP_LOGI(TAG, "     flags:  %02x", (int)flags);
    ESP_LOGI(TAG, "     effect: %d (%s)", effect, get_effect_name(effect));
    ESP_LOGI(TAG, "     speed:  %d", speed);
    ESP_LOGI(TAG, "     bright: %d", brightness);
    ESP_LOGI(TAG, "     colors:[%d] = { %08x,%08x,%08x}", num_colors, color[0], color[1], color[2]);
}

//////////////////////////////////////////////////////////////////////

void set_brightness_int(settings_t &settings, int x)
{
    if(x < 0) {
        x = 0;
    } else if(x > 100) {
        x = 100;
    }
    settings.brightness = x;
}

//////////////////////////////////////////////////////////////////////

void set_speed_int(settings_t &settings, int x)
{
    if(x < 0) {
        x = 0;
    } else if(x > 100) {
        x = 100;
    }
    settings.speed = x;
}

//////////////////////////////////////////////////////////////////////

bool set_powerstate(settings_t &settings, jsmntok_t const &val, char const *json_text)
{
    bool result;
    if(json::get_bool(val, result, json_text)) {
        if(result) {
            settings.flags |= (uint32)settings_t::flags_t::powerstate;
        } else {
            settings.flags &= ~(uint32)settings_t::flags_t::powerstate;
        }
        return true;
    }
    ESP_LOGW(TAG, "BAD POWER VALUE!?");
    return false;
}

//////////////////////////////////////////////////////////////////////

bool set_speed(settings_t &settings, jsmntok_t const &val, char const *json_text)
{
    uint32 s;
    if(json::get_uint32(val, s, json_text)) {
        if(s < 10) {
            s = 10;
        } else if(s > 100) {
            s = 100;
        }
        set_speed_int(settings, s);
        return true;
    }
    ESP_LOGW(TAG, "BAD SPEED VALUE");
    return false;
}

//////////////////////////////////////////////////////////////////////

bool set_speed_action(settings_t &settings, jsmntok_t const &val, char const *json_text)
{
    if(val.type == JSMN_STRING) {
        speed_action_t const *a = get_speed_action(json_text + val.start, json::len(val));
        if(a != nullptr) {
            set_speed_int(settings, settings.speed * a->mul + a->add);
            return true;
        }
    }
    ESP_LOGW(TAG, "BAD SPEED VALUE");
    return false;
}

//////////////////////////////////////////////////////////////////////

bool set_brightness(settings_t &settings, jsmntok_t const &val, char const *json_text)
{
    int b;
    if(json::get_int(val, b, json_text)) {
        set_brightness_int(settings, b);
        return true;
    }
    ESP_LOGW(TAG, "BAD BRIGHTNESS VALUE");
    return false;
}

//////////////////////////////////////////////////////////////////////

bool set_brightness_delta(settings_t &settings, jsmntok_t const &val, char const *json_text)
{
    int b;
    if(json::get_int(val, b, json_text)) {
        set_brightness_int(settings, settings.brightness + b);
        return true;
    }
    ESP_LOGW(TAG, "BAD BRIGHTNESS VALUE");
    return false;
}

//////////////////////////////////////////////////////////////////////

bool set_colors(settings_t &settings, jsmntok_t const &val, char const *json_text)
{
    if(val.type != JSMN_ARRAY) {
        ESP_LOGW(TAG, "COLORS != ARRAY!?");
    } else {
        if(val.size < 1 || val.size > 3) {
            ESP_LOGW(TAG, "1..3 COLORS ONLY");
        } else {
            settings.num_colors = 0;
            jsmntok_t const *tokens = &val;
            for(int j = 1; j <= val.size; ++j) {
                jsmntok_t const &color_val = tokens[j];
                uint32 c;
                if(!json::get_uint32(color_val, c, json_text)) {
                    ESP_LOGW(TAG, "ERROR READING COLOR %d", j);
                    return false;
                }
                settings.color[settings.num_colors] = uint32(c);
                settings.num_colors += 1;
            }
            return true;
        }
    }
    return false;
}

//////////////////////////////////////////////////////////////////////

bool set_effect(settings_t &settings, jsmntok_t const &val, char const *json_text)
{
    settings.effect = 13;    // solid
    int e;
    if(get_effect(val, json_text, e)) {
        settings.effect = e;
        return true;
    }
    ESP_LOGW(TAG, "BAD EFFECT VALUE!?");
    return false;
}

//////////////////////////////////////////////////////////////////////

void settings_t::load(char const *json_text, size_t json_len)    // static
{
    jsmn_parser p;
    jsmn_init(&p);

    int const token_max = 32;
    jsmntok_t tokens[token_max];
    int token_count = jsmn_parse(&p, json_text, json_len, tokens, countof(tokens));

    bool ok = false;

    if(token_count < 0 || tokens[0].type != JSMN_OBJECT) {
        ESP_LOGW(TAG, "JSON PARSE ERROR %d", token_count);
        return;
    }

    ok = true;

    // don't scan any leavings at the end
    token_count -= 1;
    bool modified = false;

    // settings should be saved to nvram
    global_settings.flags &= ~(uint32)settings_t::flags_t::loaded_from_nvram;

    // defaults to switch on unless they ask to turn off
    global_settings.flags |= (uint32)settings_t::flags_t::powerstate;

    // modify new settings
    for(int i = 1; i < token_count; i += 2) {

        jsmntok_t const &key = tokens[i];
        jsmntok_t const &val = tokens[i + 1];

        if(json::eq(key, "powerstate", json_text)) {
            ok &= set_powerstate(global_settings, val, json_text);
            modified = true;

        } else if(json::eq(key, "action", json_text)) {
            ok &= set_effect(global_settings, val, json_text);
            modified = true;

        } else if(json::eq(key, "speed_set", json_text)) {
            ok &= set_speed(global_settings, val, json_text);
            modified = true;

        } else if(json::eq(key, "speed_action", json_text)) {
            ok &= set_speed_action(global_settings, val, json_text);
            modified = true;

        } else if(json::eq(key, "brightness", json_text)) {
            ok &= set_brightness(global_settings, val, json_text);
            modified = true;

        } else if(json::eq(key, "brightness_delta", json_text)) {
            ok &= set_brightness_delta(global_settings, val, json_text);
            modified = true;

        } else if(json::eq(key, "colors", json_text)) {
            ok &= set_colors(global_settings, val, json_text);
            modified = true;
            i += val.size;
        } else if(json::eq(key, "message", json_text)) {
            debug_flash(debug_color::blue, debug_color::off, 5, 5);
            ESP_LOGI(TAG, "WEBSOCKET CONNECTED: %.*s", json::len(val), json_text + val.start);
        } else if(json::eq(key, "device_id", json_text)) {
            ESP_LOGI(TAG, "DEVICE ID: %.*s", json::len(val), json_text + val.start);
        }
    }

    // if new_settings was changed
    if(modified && ok) {

        global_settings.print("NEW SETTINGS");
        debug_flash(debug_color::green, debug_color::off, 5, 5);
        xQueueSend(settings_queue, &global_settings, portMAX_DELAY);
        global_settings.save();
    }
}

//////////////////////////////////////////////////////////////////////

void settings_t::save()
{
    nvs_handle nvs;
    esp_err_t err = nvs_open("settings", NVS_READWRITE, &nvs);
    if(err != ESP_OK) {
        ESP_LOGW(TAG, "Error (%s) opening NVS handle for writing!", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Saving settings to NVS... ");
        err = nvs_set_blob(nvs, "settings", this, sizeof(*this));
        if(err != ESP_OK) {
            ESP_LOGW(TAG, "Error (%s) writing!", esp_err_to_name(err));
        }
        nvs_close(nvs);
    }
}

//////////////////////////////////////////////////////////////////////

void settings_t::init()
{
    nvs_handle nvs;
    esp_err_t err = nvs_open("settings", NVS_READONLY, &nvs);
    if(err != ESP_OK) {
        settings_t settings;
        settings.speed = 50;
        settings.effect = settings_t::solid_effect;
        settings.flags = 1;
        settings.color[0] = 0xffffff;
        settings.color[1] = 0x00ff00;
        settings.color[2] = 0x0000ff;
        settings.num_colors = 1;
        ESP_LOGW(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
        ESP_LOGI(TAG, "Settings: ");
        ESP_LOGI(TAG, "flags: %02x", settings.flags);
        ESP_LOGI(TAG, "effect: %d", settings.effect);
        ESP_LOGI(TAG, "speed: %d", settings.speed);
        ESP_LOGI(TAG, "brightness: %d", settings.brightness);
        ESP_LOGI(TAG, "num_colors: %d", settings.num_colors);
        ESP_LOGI(TAG, "color[0]: %02x", settings.color[0]);
        ESP_LOGI(TAG, "color[1]: %02x", settings.color[1]);
        ESP_LOGI(TAG, "color[2]: %02x", settings.color[2]);
        xQueueSend(settings_queue, &settings, portMAX_DELAY);
    } else {
        settings_t settings;
        ESP_LOGI(TAG, "Reading settings from NVS... (I reckon it's %d big)", sizeof(settings_t));
        size_t got = sizeof(settings);
        err = nvs_get_blob(nvs, "settings", &settings, &got);
        switch(err) {
        case ESP_OK:
            settings.flags |= (uint32)settings_t::flags_t::loaded_from_nvram;
            settings.flags |= (uint32)settings_t::flags_t::powerstate;
            settings.print("LOADED");
            global_settings = settings;
            xQueueSend(settings_queue, &settings, portMAX_DELAY);
            break;
        case ESP_ERR_NVS_NOT_FOUND:
            ESP_LOGW(TAG, "Settings not found");
            break;
        default:
            ESP_LOGW(TAG, "Error (%s) reading!", esp_err_to_name(err));
        }
        nvs_close(nvs);
    }
}
