//////////////////////////////////////////////////////////////////////

#include <string.h>
#include <stdlib.h>
#include <utility>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/uart.h"

#include "types.h"
#include "util.h"
#include "message.h"
#include "messenger.h"

static char const *TAG = "MESSENGER";
#define MSNGR_LOG ESP_LOGV

//////////////////////////////////////////////////////////////////////

#define STM32_MSG_BUF_SIZE 128

//////////////////////////////////////////////////////////////////////

messenger *messenger::list = null;

EventGroupHandle_t messenger::message_events;    // for notifying of requests to send and acks
int messenger::next_bit;                         // for allocating bits in message_events
uint32 messenger::wait_mask;                     // all the send_req bits for waiting for send requests

//////////////////////////////////////////////////////////////////////
// sit there and send messages as requested

static void IRAM_ATTR messenger_task(void *)
{
    while(true) {
        uint32 requests = xEventGroupWaitBits(messenger::message_events, messenger::wait_mask, true, false, portMAX_DELAY);
        for(messenger *m = messenger::list; m != null; m = m->next) {
            if(requests & m->send_req) {
                uart_write_bytes(UART_NUM_1, (char const *)m->msg, m->header->length);
                MSNGR_LOG(TAG, "SENT SIG %02x LEN %02x CRC %04x", m->header->sig, m->header->length, m->header->crc);
                xEventGroupSetBits(messenger::message_events, m->sent_ack);
            }
        }
    }
}

//////////////////////////////////////////////////////////////////////

void messenger::init()
{
    message_events = xEventGroupCreate();
    next_bit = 0;
    wait_mask = 0;

    // setup uart1 for sending stm32 messages
    uart_config_t uart_config = { .baud_rate = 115200,
                                  .data_bits = UART_DATA_8_BITS,
                                  .parity = UART_PARITY_DISABLE,
                                  .stop_bits = UART_STOP_BITS_1,
                                  .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
                                  .rx_flow_ctrl_thresh = 0 };
    uart_param_config(UART_NUM_1, &uart_config);
    uart_driver_install(UART_NUM_1, STM32_MSG_BUF_SIZE * 2, 0, 0, null, 0);
}

//////////////////////////////////////////////////////////////////////

void messenger::start()
{
    // this task does the actual sending
    xTaskCreate(messenger_task, "messenger_task", 2048, null, 20, null);
}
