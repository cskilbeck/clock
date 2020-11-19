#pragma once

//////////////////////////////////////////////////////////////////////
// for marshalling messages over the uart to the stm32

uint16 IRAM_ATTR crc16(byte const *p, size_t len);
uint16 IRAM_ATTR crc16_copy(byte const *p, byte *dst, size_t len);

struct messenger
{
    byte *msg;          // ptr to the message itself
    uint32 send_req;    // to notify request to send
    uint32 sent_ack;    // to notify it was sent
    messenger *next;    // there's a list of these messengers, one for each message instance

    static messenger *list;                      // this is that list
    static EventGroupHandle_t message_events;    // for notifying of requests to send and acks
    static int next_bit;                         // for allocating bits in message_events
    static uint32 wait_mask;                     // all the send_req bits for waiting for send requests

    // lazy - use template rather than parameters for signature, length
    template <typename T> void init_message(T &m)
    {
        static char const *TAG = "messenger::init";

        assert(next_bit < 32);
        if(!std::is_base_of<message_base_t, T>::value) {
            ESP_LOGE(TAG, "messenger is only for message_base_t derived classes");
            esp_restart();
        }

        msg = new byte[sizeof(T)];    // allocate buffer for copy of the message
        m.sig = T::signature;
        m.length = sizeof(T);
        send_req = 1 << next_bit;
        sent_ack = 1 << (next_bit + 1);
        next_bit += 2;
        wait_mask |= send_req;
        ESP_LOGI(TAG, "Init messenger for struct %d with length %d", T::signature, m.length);
        next = list;
        list = this;
    }

    template <typename T> void send(T const &m)
    {
        // if(sizeof(T) != len) {
        //     ESP_LOGE(TAG, "Wrong length!");
        //     return;
        // }
        // if(T::signature != signature) {
        //     ESP_LOGE(TAG, "Wrong signature!");
        // }

        // have to copy the current message to avoid race when sending
        memcpy(msg, (byte const *)&m, sizeof(T));
        message_base_t *msg_base = (message_base_t *)msg;
        msg_base->crc = crc16((byte const *)msg_base + sizeof(message_base_t), msg_base->length - sizeof(message_base_t));
        xEventGroupSetBits(message_events, send_req);
    }

    static void init();
    static void IRAM_ATTR update();
};
