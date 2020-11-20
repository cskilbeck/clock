////////////////////////////////////////////////////////////////////////////////
// dasclock firmware

#include <cstddef>
#include <type_traits>
#include "main.h"
#include "string.h"
#include "util.h"
#include "../../esp_http_client/main/message.h"

extern byte segments[128];

////////////////////////////////////////////////////////////////////////////////
// LED driver is TLC5949
// see https://www.ti.com/lit/ds/symlink/tlc5949.pdf

// TLC5949 shift register is 193 bits
// data is sent as 200 bit spi packets
// first 7 bits are spilled (ignored)
// 8th (eg 193rd) bit is TLC_SETCONFIG:
//      0 - this is per-led brightness data (12 bits * 16 channels = 192)
//      1 - this is config data (only bits 112..122 are used)
// data sent as bytes, msb first so byte 0, bit 0 is 8th bit, byte 1, bit 7 is 9th bit etc

////////////////////////////////////////////////////////////////////////////////
// column_mask determines whether next column is activated before this packet is sent

struct spi_packet
{
    byte *data;            // spi_data to send
    uint32 column_mask;    // should it activate next column when this one is complete?
};

////////////////////////////////////////////////////////////////////////////////

// clang-format off
uint8 const minutes_map[60] = {
    15, 13, 12, 11, 10,
     9,  7,  6,  5,  4,
     3,  1,  0, 31, 30,
    28, 27, 26, 25, 24,
    22, 21, 20, 19, 18,
    16, 47, 46, 45, 44,
    42, 41, 40, 39, 38,
    36, 35, 34, 33, 32,
    62, 61, 60, 59, 58,
    56, 55, 54, 53, 52,
    50, 49, 48, 79, 78,
    77, 75, 74, 73, 72
};

// to divide a 2 digit number by 10
byte div10[100] =
{
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
    3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
    4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
    5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
    6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
    7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
    8, 8, 8, 8, 8, 8, 8, 8, 8, 8,
    9, 9, 9, 9, 9, 9, 9, 9, 9, 9
};
// clang-format on

uint8 const hours_map[12] = { 14, 8, 2, 29, 23, 17, 43, 37, 63, 57, 51, 76 };

uint8 const colon_map[2] = { 80, 88 };

uint8 const digit_map[8] = { 0, 7, 6, 5, 4, 3, 2, 1 };    // 0 is the semicolon

uint8 const digit_base[7] = { 64, 80, 88, 96, 120, 112, 104 };

////////////////////////////////////////////////////////////////////////////////
// bit numbers for tlc5949 spi packet
// buffer is 193 bits (we send 200, spill the first 7)

#define TLC_SETCONFIG 192

#define TLC_BRIGHTNESS0 112
#define TLC_BRIGHTNESS6 118
#define TLC_BLANK 119
#define TLC_DSPRPT 120
#define TLC_TMGRST 121
#define TLC_ESPWM 122

////////////////////////////////////////////////////////////////////////////////

uint8 spi_data[2][200];    // 25 * 8 * 2
uint8 config0[25];
uint8 config1[25];

spi_packet spi_packets[2][32];
spi_packet *next_spi_packet;

uint16 brightness[128] = { 0 };

uint32 column = 0;
int buffer = 0;

int ambient_light = 0;
int ambient_scale = 65535;
int ambient_target = 65535;
int user_brightness = 63;

// frame timer sets this to 1, main process reads/clears it
volatile int vblank = 0;

// ticks is incremented 1024 times per second
volatile uint32 ticks = 0;

// this was the mibiseconds last time we got a clock message
uint32 millis1024;

////////////////////////////////////////////////////////////////////////////////
// set one bit of config data in a spi buffer

void led_set(byte *data, int bit, int val)
{
    int offset = 24 - (bit / 8);
    int mask = ~(1 << (bit & 7));
    int set = (val & 1) << (bit & 7);
    data[offset] = data[offset] & mask | set;
}

////////////////////////////////////////////////////////////////////////////////

void set_global_brightness(byte b)
{
    b &= 127;
    config0[10] = 0x80 | b;
    config1[10] = 0x00 | b;
}

////////////////////////////////////////////////////////////////////////////////

void set_ascii(int digit, int c, uint16 b = 1023)
{
    uint16 *dst = brightness + digit_base[digit];
    byte x = segments[c];
    for(int i = 0; i < 8; ++i) {
        dst[digit_map[i]] = ((x & 0x80) != 0) ? b : 0;
        x <<= 1;
    }
}

////////////////////////////////////////////////////////////////////////////////

void set_digit(int digit, int number, uint16 b = 1023)
{
    if(number > 9) {
        number += 'A' - 10;
    } else {
        number += '0';
    }
    set_ascii(digit, number);
}

////////////////////////////////////////////////////////////////////////////////

void set_hex(int start_digit, int value, int num_digits, uint16 b = 1023)
{
    int shift = (num_digits - 1) * 4;
    for(int i = 0; i < num_digits; ++i) {
        set_digit(start_digit++, (value >> shift) & 0xf, b);
        shift -= 4;
    }
}

////////////////////////////////////////////////////////////////////////////////

void set_decimal_point(int digit, uint16 b)
{
    brightness[digit_base[digit] + digit_map[0]] = b;
}

////////////////////////////////////////////////////////////////////////////////

void led_data_init()
{
    buffer = 0;
    vblank = 0;

    memset(spi_data, 0, sizeof(spi_data));
    memset(brightness, 0, sizeof(brightness));
    memset(config0, 0, 25);
    memset(config1, 0, 25);

    // pointers into the spi data for the dma handler
    // 4 pointers per column:
    //      0 - set led data
    //      1 - set config0 (blank = 1)
    //      2 - set config1 (blank = 0)
    //      3 - null terminator

    // and all double buffered
    for(int i = 0; i < 2; ++i) {

        byte *s = spi_data[i];
        spi_packet *p = spi_packets[i];

        // 8 columns
        for(int j = 0; j < 8; ++j) {
            p[0].data = s;
            p[0].column_mask = 0;

            p[1].data = config0;
            p[1].column_mask = (7 << 16) | 7;

            p[2].data = config1;
            p[2].column_mask = 0;

            p[3].data = null;

            p += 4;
            s += 25;
        }
    }

    // init the fixed config spi_data (first for blank=1, second for blank=0)

    // config = 1 for both because they're both config packets
    led_set(config0, TLC_SETCONFIG, 1);
    led_set(config1, TLC_SETCONFIG, 1);

    // display repeat = 0
    led_set(config0, TLC_DSPRPT, 0);
    led_set(config1, TLC_DSPRPT, 0);

    // normal pwm mode, not the enhanced spectrum one which
    // would be difficult to chop early so refresh rate
    // would have to be quite low
    led_set(config0, TLC_ESPWM, 0);
    led_set(config1, TLC_ESPWM, 0);

    // initial global brightness setting and blank bits
    set_global_brightness(127);
}

////////////////////////////////////////////////////////////////////////////////
// pack uint16 brightness values into 12 bit buffer (3 nibbles each)

void update_display()
{
    uint16 *b = brightness;
    uint8 *p = spi_data[buffer];
    // 8 columns
    for(int i = 0; i < 8; ++i) {
        p += 1;    // 1st 7 bits are spilled, 8th bit is the TLC_SETCONFIG, data starts after that

        // 16 uint16s (in 2s)
        for(int j = 0; j < 8; ++j) {
            uint16 brt1 = *b++;
            uint16 brt2 = *b++;

            // scale for ambience
            brt1 = (brt1 * ambient_scale) >> 16;
            brt2 = (brt2 * ambient_scale) >> 16;

            // squared for ghetto gamma ramp
            brt1 = (brt1 * brt1) >> 10;
            brt2 = (brt2 * brt2) >> 10;

            // stuff the nibbles
            *p++ = (brt1 >> 4) & 0xff;
            *p++ = ((brt1 << 4) & 0xf0) | ((brt2 >> 8) & 0xf);
            *p++ = brt2 & 0xff;
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
// kick off a spi transfer

static inline void spi_send_packet()
{
    SPI1->CR1 = 0;
    SPI1->CR2 = 0;
    DMA1_Channel1->CCR = 0;

    if(next_spi_packet->data != null) {
        DMA1_Channel1->CPAR = (uint32_t)&SPI1->DR;
        DMA1_Channel1->CMAR = (uint32_t)next_spi_packet->data;
        DMA1_Channel1->CNDTR = 25;
        DMA1_Channel1->CCR = DMA_CCR_MINC | DMA_CCR_DIR | (2 << DMA_CCR_PL_Pos) | DMA_CCR_EN | DMA_CCR_TCIE;
        SPI1->CR1 = SPI_CR1_SPE | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_MSTR;
        SPI1->CR2 = SPI_CR2_TXDMAEN | (0x7 << SPI_CR2_DS_Pos);

        // maybe activate next column
        GPIOA->BSRR = ((7 << 16) | column) & next_spi_packet->column_mask;
        next_spi_packet += 1;
    }
}

////////////////////////////////////////////////////////////////////////////////
// spi packet was sent, send next one if there is one

extern "C" void DMA1_Channel1_IRQHandler()
{
    // clear the irq
    DMA1->IFCR = DMA_IFCR_CGIF1 | DMA_IFCR_CTCIF1;

    // wait for last SPI to complete
    while((SPI1->SR & SPI_SR_BSY) != 0) {
    }

    // latch on
    GPO_LED_LATCH_GPIO_Port->BSRR = GPO_LED_LATCH_Pin;

    // kick next spi_packet if there is one
    spi_send_packet();

    // latch off
    GPO_LED_LATCH_GPIO_Port->BSRR = GPO_LED_LATCH_Pin << 16;
}

////////////////////////////////////////////////////////////////////////////////

uint16 init_crc()
{
    return 0xffff;
}

uint16 update_crc(uint16 cur_crc, byte b)
{
    uint16 x = (cur_crc >> 8) ^ b;
    x ^= x >> 4;
    cur_crc = (cur_crc << 8) ^ (x << 12) ^ (x << 5) ^ x;
    return cur_crc;
}

enum
{
    ss_idle = 0,
    ss_get_len = 1,
    ss_get_crc1 = 2,
    ss_get_crc2 = 3,
    ss_get_body = 4
};

int ss_state;
int ss_len;
int ss_got;
uint16 ss_crc;
volatile byte ss_msg_type;
byte ss_buffer[16];
uint16 crc_maybe;

volatile byte msg_type_received;

void ss_reset()
{
    ss_state = ss_idle;
    ss_got = 0;
}

void on_serial_byte(byte b)
{
    ss_buffer[ss_got] = b;
    ss_got = (ss_got + 1) & 15;
    
    switch(ss_state) {
    case ss_idle:
        switch(b) {
        case control_message_signature:
        case clock_message_signature:
            ss_state = ss_get_len;
            ss_msg_type = b;
            break;
        default:
            ss_reset();
        }
        break;
    case ss_get_len:
        if(b < 8 || b > 16) {    // min/max msg size
            ss_reset();
        } else {
            crc_maybe = init_crc();
            ss_len = b;
            ss_state = ss_get_crc1;
        }
        break;
    case ss_get_crc1:
        ss_crc = b;
        ss_state = ss_get_crc2;
        break;
    case ss_get_crc2:
        ss_crc |= b << 8;
        ss_state = ss_get_body;
        break;
    case ss_get_body:
        crc_maybe = update_crc(crc_maybe, b);
        if(ss_len == ss_got) {
            if(ss_crc == crc_maybe) {
                msg_type_received = ss_msg_type;
                ss_reset();
            }
            ss_reset();
        }
        break;
    }
}

////////////////////////////////////////////////////////////////////////////////
// a byte arrived on the uart - process it
// or the uart went idle - reset the uart state

extern "C" void USART1_IRQHandler()
{
    if(LL_USART_IsActiveFlag_RXNE_RXFNE(USART1)) {         // a byte arrived, process it
        on_serial_byte(static_cast<byte>(USART1->RDR));    // reading RDR clears the IRQ
    }
    if(LL_USART_IsActiveFlag_IDLE(USART1)) {
        LL_USART_ClearFlag_IDLE(USART1);    // for IDLE, clear the iRQ manually
        ss_reset();                 // and reset the state machine
    }
}

////////////////////////////////////////////////////////////////////////////////
// next column irq
// this irq should happen _just_ after the pwm for the most recent column is complete

extern "C" void TIM17_IRQHandler()
{
    // clear isr
    TIM17->SR = 0;

    // move to next column
    column = (column + 1) & 7;

    // prepare to send for this column (4 spi packet ptrs per column - 3 for data, 1 for null terminator)
    next_spi_packet = spi_packets[buffer] + (column * 4);

    // kick 1st spi packet
    spi_send_packet();

    // if we've just kicked off the 1st column
    // notify main process that they can start filling
    // other frame
    if(column == 0) {
        buffer = 1 - buffer;
        vblank = 1;
    }
}

////////////////////////////////////////////////////////////////////////////////

extern "C" void SysTick_Handler()
{
    ticks += 1;
}

////////////////////////////////////////////////////////////////////////////////

uint32 last_second_ticks = 0;

uint32 second_ticks()    // 0..1023 = 1 second
{
    return ticks - last_second_ticks;
}

////////////////////////////////////////////////////////////////////////////////

typedef void (*display_handler)();

void display_intro();
void display_fade();
void display_clock();
void display_test();

display_handler current_display_handler = display_intro;
uint32 display_handler_timestamp;    // ticks when display_handler was last changed

////////////////////////////////////////////////////////////////////////////////

uint32 handler_time()
{
    return ticks - display_handler_timestamp;
}

////////////////////////////////////////////////////////////////////////////////

void set_display_handler(display_handler h)
{
    display_handler_timestamp = ticks;
    current_display_handler = h;
}

////////////////////////////////////////////////////////////////////////////////

void display_intro()
{
    memset(brightness, 0, sizeof(brightness));
    int ht = handler_time();
    int f = (ht << 2) & 8191;
    int pos = (f * (f >> 3)) >> 13;
    int tail_scale = 1024 / (util::min(1023, (f * (f * 11)) >> 20) + 1);
    int head = 1023;
    int y = pos % 60;
    for(int i = 0; i != 60; ++i) {
        brightness[minutes_map[y]] = (head * head) >> 10;
        y -= 1;
        if(y < 0) {
            y = 59;
        }
        head -= tail_scale;
        if(head < 0) {
            head = 0;
        }
    }
    ambient_scale = 0xffff;
    set_global_brightness(127);

    if(ht > 1023) {
        set_display_handler(display_fade);
    }
}

////////////////////////////////////////////////////////////////////////////////

void display_fade()
{
    int ht = handler_time();
    int f = 1024 - ht;
    f = (f * f) >> 10;
    for(int i = 0; i < 60; ++i) {
        brightness[minutes_map[i]] = f;
    }
    ambient_scale = 0xffff;
    set_global_brightness(127);

    if(ht > 1023) {
        set_display_handler(display_clock);
    }
}

////////////////////////////////////////////////////////////////////////////////

char clock_digits[7];
uint32 hours;
uint32 minutes;
uint32 seconds;
uint32 millis;

int sec_ticks;

////////////////////////////////////////////////////////////////////////////////
// show a 2 digit number

void set_number(int d, int x)
{
    if(x > 99) {
        x = 0;
    }
    uint32 h0 = div10[x];
    uint32 h1 = x - (h0 * 10);
    memset(clock_digits, 0, 7);
    clock_digits[d + 0] = h0 + '0';
    clock_digits[d + 1] = h1 + '0';
    for(int i = 0; i < 7; ++i) {
        set_ascii(i, clock_digits[i]);
    }
}

////////////////////////////////////////////////////////////////////////////////
// show the time

void display_clock()
{
    uint32 h0 = div10[hours];
    uint32 h1 = hours - (h0 * 10);
    uint32 m0 = div10[minutes];
    uint32 m1 = minutes - (m0 * 10);
    uint32 s0 = div10[seconds];
    uint32 s1 = seconds - (s0 * 10);

    clock_digits[0] = h0 + '0';
    clock_digits[1] = h1 + '0';
    clock_digits[2] = m0 + '0';
    clock_digits[3] = m1 + '0';
    clock_digits[4] = s0 + '0';
    clock_digits[5] = s1 + '0';

    for(int i = 0; i < 7; ++i) {
        set_ascii(i, clock_digits[i]);
    }
    for(int i = 0; i < seconds; ++i) {
        brightness[minutes_map[i]] = 256;
    }
    for(int i = seconds + 1; i < 60; ++i) {
        brightness[minutes_map[i]] = 0;
    }
    for(int i = 0; i < 12; ++i) {
        brightness[hours_map[i]] = 256;
    }

    brightness[minutes_map[seconds]] = util::min(256, sec_ticks >> 0);

    uint flash = 0;

    colon_flash_mode colon_mode = colon_flash_mode::solid;    // colon_flash_mode(last_msg.colon_flash_mode);
    switch(colon_mode) {
    case colon_flash_mode::off:
        flash = 0;
        break;
    case colon_flash_mode::pulse:
        flash = util::abs(((int)(sec_ticks + 573) & 1023) - 512);
        flash = util::min(1023u, flash << 3) >> 2;
        break;
    case colon_flash_mode::flash:
        flash = ((sec_ticks >> 9) & 1) << 9;
        break;
    case colon_flash_mode::solid:
        flash = 512;
        break;
    }
    brightness[colon_map[0]] = flash;
    brightness[colon_map[1]] = flash;
}

////////////////////////////////////////////////////////////////////////////////

void display_test()
{
    ambient_scale = 0xffff;
    set_global_brightness(127);

    for(int i = 0; i < 128; ++i) {
        brightness[i] = 1023;
    }
    if(handler_time() > 1024) {
        set_display_handler(display_clock);
    }
}

////////////////////////////////////////////////////////////////////////////////

int number_to_display = 0;

void display_number()
{
    set_number(4, number_to_display);
    if(handler_time() > 1023) {
        set_display_handler(display_clock);
    }
}

////////////////////////////////////////////////////////////////////////////////

template <typename T> void process_message(T const &m)
{
}

////////////////////////////////////////////////////////////////////////////////

template <> void process_message<control_message_t>(control_message_t const &m)
{
    user_brightness = m.brightness & 63;
    number_to_display = m.brightness - 13;
    set_display_handler(display_number);
}

////////////////////////////////////////////////////////////////////////////////

template <> void process_message<clock_message_t>(clock_message_t const &m)
{
    // we got a clock message
    // format the digits
    // and note the millis

    // this time is actually a bit late due to the 115200 serial
    // time and some overhead for irqs etc so we should add on
    // some amount before sending it from the esp12

    hours = m.hours;
    minutes = m.minutes;
    seconds = m.seconds;
    millis = m.milliseconds;

    // this was milliseconds in 0..1023 format
    millis1024 = (millis * 67109) >> 16;    // scale from 0..999 to 0..1023 (kinda)

    // this was when the message was received
    last_second_ticks = ticks;

    // later, we use the difference between (ticks - last_second_ticks) + millis1024 to get a new absolute time
}

////////////////////////////////////////////////////////////////////////////////

template <typename T> void handle_message()
{
    process_message<T>(*reinterpret_cast<T const *>(ss_buffer + sizeof(message_header_t)));
}

////////////////////////////////////////////////////////////////////////////////

void update_clock()
{
    if(msg_type_received != 0) {
        switch(msg_type_received) {
        case control_message_signature:
            handle_message<control_message_t>();
            break;
        case clock_message_signature:
            handle_message<clock_message_t>();
            break;
        }
        msg_type_received = 0;
    }

    sec_ticks = second_ticks() + millis1024;
    while(sec_ticks > 1023) {
        last_second_ticks += 1024;
        sec_ticks -= 1024;
        if(++seconds >= 60) {
            seconds = 0;
            if(++minutes >= 60) {
                minutes = 0;
                if(++hours >= 24) {
                    hours = 0;
                }
            }
        }
    }

    (current_display_handler)();
}

////////////////////////////////////////////////////////////////////////////////

void delay(int t)
{
    int n = ticks + t;
    for(int i = ticks; i < t; ++i) {
        __NOP();
    }
}

////////////////////////////////////////////////////////////////////////////////

void update_ambient()
{
    // only if ADC reading is not ongoing (which it won't be)
    if(LL_ADC_REG_IsConversionOngoing(ADC1) != 0) {
        return;
    }

    // read ambient light ADC
    int adc = LL_ADC_REG_ReadConversionData12(ADC1);

    int const ambient_update_speed = 4;
    // high pass filter the ambient_light
    ambient_light = ((ambient_light * 31) >> 5) + adc;
    ambient_target = ((ambient_light * 100) >> 5) + 8192;
    if(ambient_target > 65535) {
        ambient_target = 65535;
    }
    int scaled_ambient = (ambient_target * user_brightness) >> 6;
    if(ambient_scale < scaled_ambient) {
        ambient_scale = util::min(ambient_scale + ambient_update_speed, scaled_ambient);
    }
    if(ambient_scale > scaled_ambient) {
        ambient_scale = util::max(ambient_scale - ambient_update_speed, scaled_ambient);
    }
    set_global_brightness((ambient_scale * user_brightness) >> (9 + 6));

    LL_ADC_REG_StartConversion(ADC1);
}

////////////////////////////////////////////////////////////////////////////////

extern "C" void user_main()
{
    // disable 3-8 mux (all the gates go high)
    GPIOA->BSRR = 1 << (3 + 16);

    // init SysTick wall clock
    // 1024 ticks per second = 0.9765625ms / tick
    // it drifts a lot, no crystal
    LL_InitTick(64000000U, 1024U);
    LL_SYSTICK_EnableIT();
    NVIC_EnableIRQ(SysTick_IRQn);

    // init 32MHz PWM clock
    NVIC_DisableIRQ(TIM14_IRQn);
    TIM14->DIER = 0;
    TIM14->CCER |= TIM_CCER_CC1E;

    led_data_init();

    // Column clock
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    NVIC_EnableIRQ(TIM17_IRQn);
    TIM17->SR &= TIM_SR_UIF;
    TIM17->ARR = 2999;
    TIM17->PSC = 0;
    TIM17->DIER |= TIM_DIER_UIE;
    TIM17->CR1 |= TIM_CR1_CEN;

    LL_ADC_StartCalibration(ADC1);
    while(LL_ADC_IsCalibrationOnGoing(ADC1)) {
    }

    LL_ADC_Enable(ADC1);

    // enable PWM clock
    TIM14->CR1 |= TIM_CR1_CEN;

    // enable 3-8 mux
    GPIOA->BSRR = 1 << 3;

    // kick off serial read from esp12
    LL_USART_DisableRxTimeout(USART1);
    LL_USART_ClearFlag_RTO(USART1);
    LL_USART_EnableIT_RXNE(USART1);
    LL_USART_EnableIT_IDLE(USART1);
    NVIC_SetPriority(USART1_IRQn, 0xff);    // lowest priority for uart irq
    NVIC_EnableIRQ(USART1_IRQn);

    // kick off ADC for ambient light sensor
    LL_ADC_REG_StartConversion(ADC1);

    while(1) {

        // waitvb
        while(vblank == 0) {
            __WFI();
        }
        vblank = 0;

        update_ambient();
        update_clock();
        update_display();
    }
}
