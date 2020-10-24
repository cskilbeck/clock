#pragma once

//////////////////////////////////////////////////////////////////////

struct settings_t
{
    enum class flags_t : uint32
    {
        powerstate = 1,          // lights on or off
        loaded_from_nvram = 2    // this was loaded from nvram, don't save it
    };

    uint32 flags = 0;
    int effect = 13;
    int speed = 50;
    int brightness = 100;
    int num_colors = 0;
    uint32 color[3] = { 0xff0000, 0x00ff00, 0x0000ff };

    void print(char const *banner);
    static void init();
    static void load(char const *json_text, size_t json_len);
    void save();

    enum
    {
        solid_effect = 13,
        glow_effect = 17,
        twinkle_effect = 16,
        sparkle_effect = 15,
        flash_effect = 14,
        pulse_effect = 12,
        fade_effect = 12,
    };
};

//////////////////////////////////////////////////////////////////////

extern QueueHandle_t settings_queue;
