//////////////////////////////////////////////////////////////////////

#include <utility>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "esp8266/gpio_struct.h"

#include "types.h"
#include "util.h"
#include "button.h"

//////////////////////////////////////////////////////////////////////

static char const *TAG = "BUTTON";

//////////////////////////////////////////////////////////////////////

#define NUM_BUTTONS 3

#define BTN1_POS GPIO_NUM_14
#define BTN2_POS GPIO_NUM_13
#define BTN3_POS GPIO_NUM_16

#define BTN1_MASK (1 << BTN1_POS)
#define BTN2_MASK (1 << BTN2_POS)
#define BTN3_MASK (1 << BTN3_POS)

struct btn_def
{
    gpio_num_t pos;
    uint32_t mask;
    gpio_pullup_t pullup;
};

// clang-format off
btn_def const btn_defs[NUM_BUTTONS] = {
    { BTN1_POS, BTN1_MASK, GPIO_PULLUP_ENABLE },
    { BTN2_POS, BTN2_MASK, GPIO_PULLUP_ENABLE },
    { BTN3_POS, BTN3_MASK, GPIO_PULLUP_DISABLE }
};
// clang-format on

QueueHandle_t button_queue;

//////////////////////////////////////////////////////////////////////

struct button
{
    enum : int
    {
        held = 1,
        pressed = 2,
        released = 4,
        autorepeat = 8
    };

    uint8 old_state = 0;
    uint8 state = 0;
    int auto_repeat = 0;

    bool update(uint8 new_state)
    {
        uint8 change = old_state ^ new_state;
        old_state = new_state;

        uint8 press = change & new_state;
        uint8 release = change & ~new_state;

        state = new_state | (press << 1) | (release << 2);

        if(new_state != 0) {
            auto_repeat += 1;
            if((auto_repeat > 0x40) && ((auto_repeat & 0x7) == 1)) {
                state |= pressed | autorepeat;
                change = 1;
            }
        } else {
            auto_repeat = 0;
            state &= ~autorepeat;
        }

        return change != 0;
    }
};

//////////////////////////////////////////////////////////////////////

static uint32 read_gpio_inputs()
{
    uint32 gpio = GPIO.in;
    uint32 pin16 = READ_PERI_REG(RTC_GPIO_IN_DATA) & 1;
    return (gpio & 0xffff) | (pin16 << 16);
}

//////////////////////////////////////////////////////////////////////
// turns out, if you're not too worried about latency/timing, polling
// at 10ms intervals is a perfectly valid way to debounce even quite
// crappy tactile switches

static void IRAM_ATTR button_task(void *)
{
    ESP_LOGI(TAG, "Button task begins");

    button b[NUM_BUTTONS];

    while(true) {

        uint32 gpio = read_gpio_inputs();

        bool changed = false;
        for(int i = 0; i < NUM_BUTTONS; ++i) {
            changed |= b[i].update(1 - ((gpio >> btn_defs[i].pos) & 1));
        }

        if(changed) {
            uint32 button_bits = 0;
            for(int i = 0; i < NUM_BUTTONS; ++i) {
                button_bits = (button_bits << 4) | b[(NUM_BUTTONS - 1) - i].state;
            }
            xQueueSend(button_queue, &button_bits, portMAX_DELAY);
        }
        vTaskDelay(pdMS_TO_TICKS(10));    // delay for 10ms
    }
}

//////////////////////////////////////////////////////////////////////

void button_init()
{
    gpio_config_t io_conf;
    for(int i = 0; i < 3; ++i) {
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.pin_bit_mask = btn_defs[i].mask;
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pull_up_en = btn_defs[i].pullup;
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        gpio_config(&io_conf);
    }

    button_queue = xQueueCreate(10, sizeof(uint32));

    xTaskCreate(button_task, "button_task", 2048, null, 10, null);
}

//////////////////////////////////////////////////////////////////////

uint32 button_update()
{
    uint32 button_bits;
    xQueueReceive(button_queue, &button_bits, portMAX_DELAY);
    return button_bits;
}
