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
#include "http.h"
#include "jsmn.h"
#include "message.h"
#include "messenger.h"
#include "clock.h"
#include "timezone.h"
#include "ntp.h"

static char const *TAG = "CLOCK";

//////////////////////////////////////////////////////////////////////

#define STM32_MSG_BUF_SIZE 128

//////////////////////////////////////////////////////////////////////

#define BTN1_POS GPIO_NUM_13
#define BTN2_POS GPIO_NUM_14
#define BTN1_MASK (1 << BTN1_POS)
#define BTN2_MASK (1 << BTN2_POS)

int button_state;
int button_press;
int button_release;

//////////////////////////////////////////////////////////////////////

constexpr uint32 one_second = 1;
constexpr uint32 one_minute = one_second * 60;
constexpr uint32 one_hour = one_minute * 60;
constexpr uint32 one_day = one_hour * 24;

//////////////////////////////////////////////////////////////////////

control_message_t control_message;
clock_message_t clock_message;

messenger control_messenger;
messenger clock_messenger;

//////////////////////////////////////////////////////////////////////

void button_update()
{
    uint32 gpio = GPIO.in;
    int btn1 = 1 - ((gpio & BTN1_MASK) >> BTN1_POS);
    int btn2 = 2 - ((gpio & BTN2_MASK) >> (BTN2_POS - 1));
    int new_button_state = btn1 | btn2;
    int button_change = new_button_state ^ button_state;
    button_state = new_button_state;
    button_press = button_change & button_state;
    button_release = button_change & ~button_state;
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

    while(true) {
        button_update();

        if(button_press & 2) {
            control_message.brightness = min(63, control_message.brightness + 1);
        }
        if(button_press & 1) {
            control_message.brightness = max(1, control_message.brightness - 1);
        }

        if(button_press || button_release) {
            // whole big menu handler here
            ESP_LOGV(TAG, "press: %d, release: %d, buttons = %d", button_press, button_release, button_state);

            control_messenger.send(control_message);
        }
        vTaskDelay(1);    // delay for 10ms
    }
}

//////////////////////////////////////////////////////////////////////
// this waits for signals from client tasks requesting that
// messages get sent to the stm32

void IRAM_ATTR message_task(void *)
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


    // sit there and send messages as requested
    while(true) {
        messenger::update();
    }
}

//////////////////////////////////////////////////////////////////////

void do_clock()
{
    ESP_LOGI(TAG, "Clock task begins");

    messenger::init();

    control_messenger.init_message(control_message);
    clock_messenger.init_message(clock_message);

    // this task does the actual sending
    xTaskCreate(message_task, "message_sender", 2048, null, 20, null);

    // the button handler task might send some control messages
    xTaskCreate(button_task, "button_task", 2048, NULL, 10, NULL);

    // then this current task becomes the clock task which sends clock_message_t messages
    initialise_wifi();

    wifi_wait_until(wifi_event_connected, portMAX_DELAY);

    ESP_LOGI(TAG, "Wifi connected, getting time and timezone");

    if(init_timezone() != ESP_OK) {
        ESP_LOGE(TAG, "Can't get IP/Location/Timezone");
    }

    ESP_LOGI(TAG, "Clock init complete");

    start_sntp();

    TickType_t xLastWakeTime = xTaskGetTickCount();

    while(true) {

        struct timespec t;
        clock_gettime(CLOCK_REALTIME, &t);
        uint32 daytime = (t.tv_sec + timezone_offset()) % one_day;
        clock_message.hours = daytime / one_hour;
        clock_message.minutes = (daytime % one_hour) / 60;
        clock_message.seconds = daytime % 60;
        clock_message.milliseconds = t.tv_nsec / 1000000;
        clock_messenger.send(clock_message);
        vTaskDelayUntil(&xLastWakeTime, 1000 / portTICK_PERIOD_MS);
    }
}
