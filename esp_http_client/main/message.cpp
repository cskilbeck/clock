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
#include "driver/uart.h"

#include "types.h"
#include "util.h"
#include "message.h"
#include "messenger.h"

static char const *TAG = "message";

//////////////////////////////////////////////////////////////////////

messenger *messenger::list = null;

EventGroupHandle_t messenger::message_events;    // for notifying of requests to send and acks
int messenger::next_bit;                         // for allocating bits in message_events
uint32 messenger::wait_mask;                     // all the send_req bits for waiting for send requests

//////////////////////////////////////////////////////////////////////

void messenger::init()
{
    message_events = xEventGroupCreate();
    next_bit = 0;
    wait_mask = 0;
}

//////////////////////////////////////////////////////////////////////
// calculate a crc for an stm32 message

uint16 IRAM_ATTR crc16(byte const *p, size_t len)
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

// and with copy to somewhere else

uint16 IRAM_ATTR crc16_copy(byte const *p, byte *dst, size_t len)
{
    assert(len != 0);
    uint16 crc = 0xffff;
    byte const *e = p + len;
    do {
        byte c = *p;
        uint16 x = (crc >> 8) ^ c;
        x ^= x >> 4;
        crc = (crc << 8) ^ (x << 12) ^ (x << 5) ^ x;
        *dst++ = c;
    } while(++p < e);
    return crc;
}

//////////////////////////////////////////////////////////////////////

void IRAM_ATTR messenger::update()
{
    uint32 requests = xEventGroupWaitBits(message_events, wait_mask, pdTRUE, pdFALSE, portMAX_DELAY);
    for(messenger *m = list; m != null; m = m->next) {
        if(requests & m->send_req) {

            message_base_t *msg_base = reinterpret_cast<message_base_t *>(m->msg);

            uart_write_bytes(UART_NUM_1, (char const *)msg_base, msg_base->length);

            ESP_LOGV(TAG, "SENT SIG %02x LEN %02x CRC %04x", msg_base->sig, msg_base->length, msg_base->crc);

            xEventGroupSetBits(message_events, m->sent_ack);
        }
    }
}
