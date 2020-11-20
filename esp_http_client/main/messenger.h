#pragma once

//////////////////////////////////////////////////////////////////////
// for marshalling messages over the uart to the stm32

uint16 IRAM_ATTR crc16(byte const *p, size_t len);
uint16 IRAM_ATTR crc16_copy(byte const *p, byte *dst, size_t len);

struct messenger
{
    static constexpr char const *TAG = "MESSENGER";

    union
    {
        byte *msg;                   // ptr to buffer for taking a copy for sending
        message_header_t *header;    // it starts with a header
    };

    uint32 send_req;    // xEventGroup bit to notify request to send
    uint32 sent_ack;    // xEventGroup bit to notify it was sent
    messenger *next;    // global list of messengers, one for each message instance

    static messenger *list;                      // this is that list
    static EventGroupHandle_t message_events;    // for notifying of requests to send and acks
    static int next_bit;                         // for allocating bits in message_events
    static uint32 wait_mask;                     // all the req/ack bits for waiting for send requests

    void init_messenger(byte const *p, uint8 sig, uint8 length)
    {
        assert(next_bit < 32);

        // alloc buffer for header and copy of message
        int len = length + sizeof(message_header_t);
        msg = new byte[len];

        // init header
        header->sig = sig;
        header->length = len;

        ESP_LOGI(TAG, "Init messenger for struct %d with length %d", header->sig, header->length);

        // allocate req/ack bits
        send_req = 1 << next_bit;
        sent_ack = 1 << (next_bit + 1);
        next_bit += 2;
        wait_mask |= send_req;

        xEventGroupSetBits(message_events, sent_ack);

        // add to global list of handlers (messengers)
        next = list;
        list = this;
    }

    // lazy - use template rather than parameters for signature, length
    template <typename T> void init_message(T const &m)
    {
        init_messenger(reinterpret_cast<byte const *>(&m), T::signature, sizeof(T));
    }

    // wait for (and clear) ack from last message sent
    void sync(int timeout = portMAX_DELAY)
    {
        xEventGroupWaitBits(message_events, sent_ack, true, true, timeout);
    }

    // copy the message and calculate crc, then set request bit
    template <typename T> void send_message_async(T const &m)
    {
        header->crc = crc16_copy((byte const *)&m, msg + sizeof(message_header_t), sizeof(T));
        xEventGroupClearBits(message_events, sent_ack);
        xEventGroupSetBits(message_events, send_req);
    }

    // copy the message and calculate crc, set request bit, wait for ack
    template <typename T> void send_message(T const &m, int timeout = portMAX_DELAY)
    {
        header->crc = crc16_copy((byte const *)&m, msg + sizeof(message_header_t), sizeof(T));
        xEventGroupSync(message_events, send_req, sent_ack, timeout);
    }

    static void init();     // call this before calling init_messenger()
    static void start();    // call this to kick off the messenger task
};
