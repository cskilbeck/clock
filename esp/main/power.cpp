//////////////////////////////////////////////////////////////////////

#include "esp_system.h"
#include "driver/gpio.h"
#include "debug.h"
#include "power.h"

//////////////////////////////////////////////////////////////////////

void powerswitch_init()
{
    gpio_config_t c;
    c.pin_bit_mask = GPIO_Pin_5 | GPIO_Pin_4;
    c.mode = GPIO_MODE_OUTPUT;
    c.pull_up_en = GPIO_PULLUP_DISABLE;
    c.pull_down_en = GPIO_PULLDOWN_DISABLE;
    c.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&c);
}

//////////////////////////////////////////////////////////////////////

void powerswitch_set(bool on_or_off)
{
    int x = int(on_or_off);
    gpio_set_level(GPIO_NUM_5, x);
    gpio_set_level(GPIO_NUM_4, x);
    debug_switch(debug_led_state(on_or_off));
}
