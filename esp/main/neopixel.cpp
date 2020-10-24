//////////////////////////////////////////////////////////////////////
// Method:
// 2 DMA buffers: one being actively sent by the DMA hardware, the other being written into by CPU
// wait for vblank then send one and fill the other (toggle send/fill buffers each frame)
// the last entry in the DMA list is small, points at zeros and links to itself
// we don't need the EOF or interrupts because sending one list takes < 16.66ms (for 400 leds)
// the dma lists point into a contiguous area of memory, so filling it is simple
//
// e.g. for 400 leds, we need 400 * 3 (RGB) = 1200 bytes, so 1200 * 8 = 9600 bits
// each bit uses a nibble (for a one: 1110 or for a zero: 1000), so 9600 * 4 = 38400 bits
// we send 16 bit words, so 38400 / 16 = 2400 words of data = 4800 bytes
// we also need some blank on the end for the reset signal - we'll use 16 bytes of zero padding on the end
//
// total dma buffer size 4816 * 2 = 9632 bytes total
//
// memory usage can be reduced significantly at the expense of some complexity by using interrupts
// to fill nibble buffers on the fly but 400 leds = ~10K of buffer size
// so it's not required in this case, not bothering with that noise
//
//////////////////////////////////////////////////////////////////////

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cstddef>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "esp_attr.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_libc.h"
#include "esp_log.h"

#include "rom/ets_sys.h"

#include "driver/gpio.h"
#include "driver/i2s.h"

#include "esp8266/eagle_soc.h"
#include "esp8266/i2s_register.h"
#include "esp8266/i2s_struct.h"
#include "esp8266/pin_mux_register.h"
#include "esp8266/slc_register.h"
#include "esp8266/slc_struct.h"

#include "rom_functions.h"

#include "util.h"
#include "debug.h"

//////////////////////////////////////////////////////////////////////

namespace
{
    char const *TAG = "neopixel";

    //////////////////////////////////////////////////////////////////////
    // table converts bits into nibbles for PL9823 timings

    static uint32 const nibble_lookup[256] = {
        0x88888888, 0x8888888E, 0x888888E8, 0x888888EE, 0x88888E88, 0x88888E8E, 0x88888EE8, 0x88888EEE, 0x8888E888, 0x8888E88E, 0x8888E8E8, 0x8888E8EE, 0x8888EE88, 0x8888EE8E,
        0x8888EEE8, 0x8888EEEE, 0x888E8888, 0x888E888E, 0x888E88E8, 0x888E88EE, 0x888E8E88, 0x888E8E8E, 0x888E8EE8, 0x888E8EEE, 0x888EE888, 0x888EE88E, 0x888EE8E8, 0x888EE8EE,
        0x888EEE88, 0x888EEE8E, 0x888EEEE8, 0x888EEEEE, 0x88E88888, 0x88E8888E, 0x88E888E8, 0x88E888EE, 0x88E88E88, 0x88E88E8E, 0x88E88EE8, 0x88E88EEE, 0x88E8E888, 0x88E8E88E,
        0x88E8E8E8, 0x88E8E8EE, 0x88E8EE88, 0x88E8EE8E, 0x88E8EEE8, 0x88E8EEEE, 0x88EE8888, 0x88EE888E, 0x88EE88E8, 0x88EE88EE, 0x88EE8E88, 0x88EE8E8E, 0x88EE8EE8, 0x88EE8EEE,
        0x88EEE888, 0x88EEE88E, 0x88EEE8E8, 0x88EEE8EE, 0x88EEEE88, 0x88EEEE8E, 0x88EEEEE8, 0x88EEEEEE, 0x8E888888, 0x8E88888E, 0x8E8888E8, 0x8E8888EE, 0x8E888E88, 0x8E888E8E,
        0x8E888EE8, 0x8E888EEE, 0x8E88E888, 0x8E88E88E, 0x8E88E8E8, 0x8E88E8EE, 0x8E88EE88, 0x8E88EE8E, 0x8E88EEE8, 0x8E88EEEE, 0x8E8E8888, 0x8E8E888E, 0x8E8E88E8, 0x8E8E88EE,
        0x8E8E8E88, 0x8E8E8E8E, 0x8E8E8EE8, 0x8E8E8EEE, 0x8E8EE888, 0x8E8EE88E, 0x8E8EE8E8, 0x8E8EE8EE, 0x8E8EEE88, 0x8E8EEE8E, 0x8E8EEEE8, 0x8E8EEEEE, 0x8EE88888, 0x8EE8888E,
        0x8EE888E8, 0x8EE888EE, 0x8EE88E88, 0x8EE88E8E, 0x8EE88EE8, 0x8EE88EEE, 0x8EE8E888, 0x8EE8E88E, 0x8EE8E8E8, 0x8EE8E8EE, 0x8EE8EE88, 0x8EE8EE8E, 0x8EE8EEE8, 0x8EE8EEEE,
        0x8EEE8888, 0x8EEE888E, 0x8EEE88E8, 0x8EEE88EE, 0x8EEE8E88, 0x8EEE8E8E, 0x8EEE8EE8, 0x8EEE8EEE, 0x8EEEE888, 0x8EEEE88E, 0x8EEEE8E8, 0x8EEEE8EE, 0x8EEEEE88, 0x8EEEEE8E,
        0x8EEEEEE8, 0x8EEEEEEE, 0xE8888888, 0xE888888E, 0xE88888E8, 0xE88888EE, 0xE8888E88, 0xE8888E8E, 0xE8888EE8, 0xE8888EEE, 0xE888E888, 0xE888E88E, 0xE888E8E8, 0xE888E8EE,
        0xE888EE88, 0xE888EE8E, 0xE888EEE8, 0xE888EEEE, 0xE88E8888, 0xE88E888E, 0xE88E88E8, 0xE88E88EE, 0xE88E8E88, 0xE88E8E8E, 0xE88E8EE8, 0xE88E8EEE, 0xE88EE888, 0xE88EE88E,
        0xE88EE8E8, 0xE88EE8EE, 0xE88EEE88, 0xE88EEE8E, 0xE88EEEE8, 0xE88EEEEE, 0xE8E88888, 0xE8E8888E, 0xE8E888E8, 0xE8E888EE, 0xE8E88E88, 0xE8E88E8E, 0xE8E88EE8, 0xE8E88EEE,
        0xE8E8E888, 0xE8E8E88E, 0xE8E8E8E8, 0xE8E8E8EE, 0xE8E8EE88, 0xE8E8EE8E, 0xE8E8EEE8, 0xE8E8EEEE, 0xE8EE8888, 0xE8EE888E, 0xE8EE88E8, 0xE8EE88EE, 0xE8EE8E88, 0xE8EE8E8E,
        0xE8EE8EE8, 0xE8EE8EEE, 0xE8EEE888, 0xE8EEE88E, 0xE8EEE8E8, 0xE8EEE8EE, 0xE8EEEE88, 0xE8EEEE8E, 0xE8EEEEE8, 0xE8EEEEEE, 0xEE888888, 0xEE88888E, 0xEE8888E8, 0xEE8888EE,
        0xEE888E88, 0xEE888E8E, 0xEE888EE8, 0xEE888EEE, 0xEE88E888, 0xEE88E88E, 0xEE88E8E8, 0xEE88E8EE, 0xEE88EE88, 0xEE88EE8E, 0xEE88EEE8, 0xEE88EEEE, 0xEE8E8888, 0xEE8E888E,
        0xEE8E88E8, 0xEE8E88EE, 0xEE8E8E88, 0xEE8E8E8E, 0xEE8E8EE8, 0xEE8E8EEE, 0xEE8EE888, 0xEE8EE88E, 0xEE8EE8E8, 0xEE8EE8EE, 0xEE8EEE88, 0xEE8EEE8E, 0xEE8EEEE8, 0xEE8EEEEE,
        0xEEE88888, 0xEEE8888E, 0xEEE888E8, 0xEEE888EE, 0xEEE88E88, 0xEEE88E8E, 0xEEE88EE8, 0xEEE88EEE, 0xEEE8E888, 0xEEE8E88E, 0xEEE8E8E8, 0xEEE8E8EE, 0xEEE8EE88, 0xEEE8EE8E,
        0xEEE8EEE8, 0xEEE8EEEE, 0xEEEE8888, 0xEEEE888E, 0xEEEE88E8, 0xEEEE88EE, 0xEEEE8E88, 0xEEEE8E8E, 0xEEEE8EE8, 0xEEEE8EEE, 0xEEEEE888, 0xEEEEE88E, 0xEEEEE8E8, 0xEEEEE8EE,
        0xEEEEEE88, 0xEEEEEE8E, 0xEEEEEEE8, 0xEEEEEEEE
    };

    //////////////////////////////////////////////////////////////////////
    // DMA (slc?) linked list entry

    struct lldesc_t
    {
        union
        {
            struct
            {
                uint32_t blocksize : 12;
                uint32_t datalen : 12;
                uint32_t unused : 5;
                uint32_t sub_sof : 1;
                uint32_t eof : 1;
                volatile uint32_t owner : 1;    // DMA can change this value (this entry being sent flag thing?)
            };
            uint32 config;
        };

        uint32_t *buf_ptr;
        lldesc_t *next_link_ptr;
    };

    //////////////////////////////////////////////////////////////////////
    // # of uint32s in each nibble buffer (not counting the end blank)

    int num_uint32s;

    //////////////////////////////////////////////////////////////////////

    // nibble buffers which get DMA'ed into the i2s hardware
    byte *dma_buffer[2];

    // DMA list entries
    lldesc_t *slc_entries[2];

    // source RGB buffer, user fills this with 24 bit rgb values
    byte *rgb_buffer;

    // toggle for front/back buffer
    int frame = 0;

    // allocate some zeroed memory
    byte *alloc(int size)
    {
        return reinterpret_cast<byte *>(heap_caps_zalloc(size, MALLOC_CAP_8BIT));
    }

}    // namespace

//////////////////////////////////////////////////////////////////////
// neopixel_init - call this once at the beginning

void neopixel_init(int num_leds)
{
    int constexpr blank_space_bytes = 16;
    int constexpr max_dma_size = 4092;

    // make led_count a multiple of 4 so # of nibbles is a multiple of 8
    int led_count = (num_leds + 3) & -4;

    // this many 24bit RGB entries
    int num_src_bytes = led_count * 3;

    // this many bits (1 nibble each) to be sent
    int num_nibbles = num_src_bytes * 8;

    // so this many bytes to be sent
    int num_bytes = num_nibbles / 2;

    // and including the blank at the end
    int total_size = num_bytes + blank_space_bytes;

    // need this many DMA buffers
    int num_whole_buffers = (total_size + max_dma_size - 1) / max_dma_size;

    // allocate the data equally between them
    int one_buffer_size = (total_size + num_whole_buffers - 1) / num_whole_buffers;

    int total_buffer_size = one_buffer_size * num_whole_buffers;

    int dma_size = num_bytes / num_whole_buffers;

    // and need one extra for the blank reset on the end
    int num_dma_entries = num_whole_buffers + 1;

    // so this many 32 bit words of data are sent
    num_uint32s = num_nibbles / 8;

    // allocate the source buffer which caller will fill in each frame
    rgb_buffer = alloc(led_count * 3);

    ESP_LOGI(TAG, "num_leds: %d (%d requested)", led_count, num_leds);
    ESP_LOGI(TAG, "num_bytes: %d", num_bytes);
    ESP_LOGI(TAG, "num_whole_buffers: %d (%d total)", num_whole_buffers, num_dma_entries);
    ESP_LOGI(TAG, "one_buffer_size: %d", one_buffer_size);
    ESP_LOGI(TAG, "total_buffer_size: %d", total_buffer_size);
    ESP_LOGI(TAG, "total_size: %d", total_size);
    ESP_LOGI(TAG, "dma_size: %d", dma_size);
    ESP_LOGI(TAG, "num_uint32s: %d", num_uint32s);

    // setup double buffered DMA buffers and lists
    for(int b = 0; b < 2; ++b) {

        // buffer of nibbles for sending
        byte *buf = alloc(total_buffer_size);
        dma_buffer[b] = buf;

        // linked dma list for that buffer
        lldesc_t *l = reinterpret_cast<lldesc_t *>(alloc(sizeof(lldesc_t) * num_dma_entries));
        slc_entries[b] = l;

        byte *current_ptr = null;
        for(int i = 0; i < num_whole_buffers; ++i) {
            int offset = i * one_buffer_size;
            int end = offset + one_buffer_size;
            if(end >= num_bytes) {
                end = num_bytes;
            }
            int len = end - offset;
            current_ptr = buf + offset;
            l->config = 0;
            l->blocksize = len;
            l->datalen = len;
            l->owner = 1;
            l->buf_ptr = reinterpret_cast<uint32 *>(current_ptr);
            l->next_link_ptr = l + 1;
            ESP_LOGI(TAG, "buffer %d[%d] is at offset %d, %d long", i, b, current_ptr - buf, l->datalen);
            current_ptr += len;
            l += 1;
        }
        // last one sends the blank at the end and loops back to itself
        l->config = 0;
        l->blocksize = blank_space_bytes;
        l->datalen = blank_space_bytes;
        l->buf_ptr = reinterpret_cast<uint32 *>(current_ptr);
        l->owner = 1;
        l->next_link_ptr = l;
        ESP_LOGI(TAG, "buffer %d[%d] is at offset %d, %d long", num_whole_buffers, b, current_ptr - buf, l->datalen);
    }

    // init the i2s/slc hardware

    I2S_CLK_ENABLE();

    PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0RXD_U, FUNC_I2SO_DATA);

    // no interrupts
    I2S0.int_ena.val = 0;

    // 16 bit left + right channels
    I2S0.conf_chan.tx_chan_mod = 0;

    // not fifo mode
    I2S0.fifo_conf.tx_fifo_mod = 0;

    // dma mode
    I2S0.fifo_conf.dscr_en = 1;

    // 16 bits
    I2S0.conf.bits_mod = 0;

    // normal bit order
    I2S0.conf.msb_right = 0;

    // left then right
    I2S0.conf.right_first = 0;

    // tx master
    I2S0.conf.tx_slave_mod = 0;

    // not sure what this is
    I2S0.conf.tx_msb_shift = 0;

    //
    // tweak clkm_div_num, bck_div_num for specific timing
    //
    // this works for PL9823, datasheet says:
    //
    // zero:  350 (+/- 150)nS =  200nS to  500nS followed by 1360 (+/- 150)ns = 1210nS to 1510nS
    //  one: 1360 (+/- 150)ns = 1210nS to 1510nS followed by  350 (+/- 150)nS =  200nS to  500nS
    //
    // So:
    //     160MHz / (13 * 5 = 65) = ~2.461538462MHz = 406.25nS per bit, 1625nS per nibble
    //
    // Zero:
    //               ┌──────┐
    //               │  T1  │        T2
    //              ─┘      └─────────────────────               406.25nS ON, 1218.75nS OFF
    //               │   1  │   0  │   0  │   0  │
    //
    // One:
    //
    //               ┌────────────────────┐
    //               │        T1          │  T2
    //              ─┘                    └───────
    //               │   1  │   1  │   1  │   0  │              1218.75nS ON,  406.25nS OFF
    //
    // I suspect it's more forgiving than this but I don't have a lot of samples
    // from different batches to find the limit so sticking to datasheet
    //
    // For 400 leds, one frame = 1625nS (T1+T2) * 24 (BPP) * 400 (num_leds) = 15.6mS total + 50uS (reset time) = 15.65mS
    // so we can run at 60Hz (16.66mS)
    // HOWEVER
    // there is a small delay at the start due to the FIFO (looks like about 12uS)
    // and some jitter due to IRQs (WiFi I think) so it's a bit tighter than it
    // seems but gets away with it as far as I can tell
    //
    // NOTE I haven't found a way to use an inverting level shifter (cheaper) because idle state of I2S output = low
    // so just using a non-inverting one (onsemi M74VHC1GT125DF1G) https://www.onsemi.com/pub/Collateral/MC74VHC1GT125-D.PDF
    // which is nice because it has very forgiving logic level inputs but sadly max vcc of 5.5v so 5v ldo required
    // because of 5.9v vcc

    I2S0.conf.bck_div_num = 13;
    I2S0.conf.clkm_div_num = 5;

    // clear any dma interrupts
    SLC0.int_clr.val = SLC0.int_st.val;

    // er... setup dma, no idea how this all works
    SLC0.tx_link.stop = 1;
    SLC0.rx_dscr_conf.rx_fill_mode = 0;
    SLC0.rx_dscr_conf.rx_eof_mode = 0;
    SLC0.rx_dscr_conf.rx_fill_en = 0;
    SLC0.rx_dscr_conf.token_no_replace = 1;
    SLC0.rx_dscr_conf.infor_no_replace = 1;

    // enable dma descriptor thing? or something I guess
    SLC0.conf0.txdata_burst_en = 0;
    SLC0.conf0.txdscr_burst_en = 1;
}

//////////////////////////////////////////////////////////////////////
// neopixel_get_frame_buffer - get ptr to rgb24 frame buffer

byte *neopixel_get_frame_buffer()
{
    return rgb_buffer;
}

//////////////////////////////////////////////////////////////////////
// neopixel_update - call this at 60Hz or similar, update rgb24 frame buffer afterwards

void neopixel_update()
{
    // kick off the current frame, fifo flush means data will start after about 12uS
    // the order of these pokes is very important!
    // especially if wifi or other IRQ noise is happening
    I2S0.conf.val &= ~(I2S_I2S_TX_START | I2S_I2S_RX_START);
    SLC0.rx_link.stop = 1;
    SLC0.rx_link.addr = reinterpret_cast<uint32>(slc_entries[frame]);
    SLC0.conf0.val |= SLC_RXLINK_RST | SLC_TXLINK_RST;
    SLC0.conf0.val &= ~(SLC_RXLINK_RST | SLC_TXLINK_RST);
    I2S0.conf.val |= (I2S_I2S_RESET_MASK | I2S_I2S_TX_START);
    I2S0.conf.val &= ~I2S_I2S_RESET_MASK;
    SLC0.rx_link.start = 1;

    // create the next frame of nibbles from rgb buffer source
    frame = 1 - frame;
    uint32 *dst = reinterpret_cast<uint32 *>(dma_buffer[frame]);
    uint32 *dst_end = dst + num_uint32s;
    byte *src = rgb_buffer;

    while(dst < dst_end) {
        *dst++ = nibble_lookup[*src++];
    }
}
