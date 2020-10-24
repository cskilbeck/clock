#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "util.h"
#include "debug.h"

//////////////////////////////////////////////////////////////////////

namespace
{
    debug_led_state state;
    debug_color solid_color = debug_color::off;
    debug_color flash_color[2];
    int flash_frames = 0;
    int flash_period;

}    // namespace

//////////////////////////////////////////////////////////////////////

void debug_init()
{
    gpio_config_t c;
    c.pin_bit_mask = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14;
    c.mode = GPIO_MODE_OUTPUT;
    c.pull_up_en = GPIO_PULLUP_DISABLE;
    c.pull_down_en = GPIO_PULLDOWN_DISABLE;
    c.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&c);
}

//////////////////////////////////////////////////////////////////////

void debug_set_color(debug_color c)
{
    solid_color = c;
    flash_frames = 0;
}

//////////////////////////////////////////////////////////////////////

void debug_switch(debug_led_state new_state)
{
    state = new_state;
}

//////////////////////////////////////////////////////////////////////

void debug_flash(debug_color a, debug_color b, int period_frames, int num_flashes)
{
    flash_color[0] = a;
    flash_color[1] = b;
    flash_period = period_frames;
    flash_frames = num_flashes * period_frames * 2;
}

//////////////////////////////////////////////////////////////////////

void debug_update()
{
    debug_color c = (state == debug_led_state::on) ? solid_color : debug_color::off;
    if(flash_frames != 0) {
        flash_frames -= 1;
        c = flash_color[(flash_frames / flash_period) & 1];
    }
    int x = static_cast<int>(c);
    gpio_set_level(GPIO_NUM_12, x & 1);
    gpio_set_level(GPIO_NUM_13, x & 2);
    gpio_set_level(GPIO_NUM_16, x & 4);
}