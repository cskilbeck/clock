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

#define BTN1_POS GPIO_NUM_13
#define BTN2_POS GPIO_NUM_14
#define BTN1_MASK (1 << BTN1_POS)
#define BTN2_MASK (1 << BTN2_POS)

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
                change = true;
            }
        } else {
            auto_repeat = 0;
            state &= ~autorepeat;
        }

        return change != 0;
    }
};

//////////////////////////////////////////////////////////////////////

static void IRAM_ATTR button_task(void *)
{
    ESP_LOGI(TAG, "Button task begins");

    int ar_start_loops = 100;    // 1 second autorepeat starts

    button b[2];

    while(true) {

        uint32 gpio = GPIO.in;

        bool changed = false;
        changed |= b[0].update(1 - ((gpio >> BTN1_POS) & 1));
        changed |= b[1].update(1 - ((gpio >> BTN2_POS) & 1));

        if(changed) {
            uint32 button_bits = (b[0].state << 4) | b[1].state;
            xQueueSend(button_queue, &button_bits, portMAX_DELAY);
        }
        vTaskDelay(pdMS_TO_TICKS(10));    // delay for 10ms
    }
}

//////////////////////////////////////////////////////////////////////

void button_init()
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = BTN1_MASK | BTN2_MASK;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

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
