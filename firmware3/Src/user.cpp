////////////////////////////////////////////////////////////////////////////////
// dasclock firmware

#include <cstddef>
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
int frames = 0;
int buffer = 0;

int ambient_light = 0;
int ambient_scale = 65535;

// frame timer sets this to 1, main process reads/clears it
volatile int vblank = 0;

// ticks is incremented 1024 times per second
volatile uint32 ticks = 0;

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

void set_hex(int start_digit, int value, int num_digits, uint16 b = 1023)
{
	int shift = (num_digits - 1) * 4;
	for(int i=0; i<num_digits; ++i) {
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
    frames = 0;
    vblank = 0;

    memset(spi_data, 0, sizeof(spi_data));
    memset(brightness, 0, sizeof(brightness));
    memset(config0, 0, 25);
    memset(config1, 0, 25);

    // pointers into the spi data for the dma handler
    // 4 spi packets per column:
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

void frame_update(uint16 *brightness, byte *spi_data)
{
    uint16 *b = brightness;
    uint8 *p = spi_data;
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
// kick off serial read from esp12

byte uart_buffer[64];

void serial_read()
{
	LL_USART_DisableRxTimeout(USART1);
	LL_USART_ClearFlag_RTO(USART1);
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_2, (uint32_t)&USART1->RDR);
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_2, (uint32_t)uart_buffer);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_2, util::countof(uart_buffer));
	LL_USART_EnableDMAReq_RX(USART1);
    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_2);

	NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
	NVIC_EnableIRQ(USART1_IRQn);
	
	LL_USART_EnableIT_IDLE(USART1);
	LL_DMA_EnableIT_HT(DMA1, LL_DMA_CHANNEL_2);
	LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_2);
}

////////////////////////////////////////////////////////////////////////////////

char const banner[16 + 1] = "HELLOSAILORWORLD";

volatile bool got_serial_data = false;

volatile uint32 new_pos = 0;
uint32 old_pos = 0;

message_t last_msg;

byte &uart_byte(int offset)
{
	return uart_buffer[offset & 63];
}

bool check_word(int offset, uint16 x)
{
	byte lo = x & 0xff;
	byte hi = x >> 8;
	return uart_byte(offset) == lo && uart_byte(offset + 1) == hi;
}

bool validate_message(int start)
{
	if(!check_word(start + offsetof(message_t, signature), 0xDA5C)) {
		return false;
	}
	
    uint16 crc = 0xffff;
	
	int end = start + offsetof(message_t, crc);
	
	byte *msg_ptr = reinterpret_cast<byte *>(&last_msg);
    for(int i = start; i != end; ++i) {
		byte b = uart_byte(i);
		*msg_ptr++ = b;
        uint16 x = (crc >> 8) ^ b;
        x ^= x >> 4;
        crc = (crc << 8) ^ (x << 12) ^ (x << 5) ^ x;
    }
    return check_word(start + offsetof(message_t, crc), crc);
}

////////////////////////////////////////////////////////////////////////////////
// some bytes have arrived on the serial port, notify main loop

void process_serial()
{
	int p = util::countof(uart_buffer) - LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_2);
	if(p != old_pos) {
		got_serial_data = true;
		new_pos = p;
	}
	old_pos = p;
	if(old_pos == util::countof(uart_buffer)) {
		old_pos = 0;
	}
}

////////////////////////////////////////////////////////////////////////////////

extern "C" void DMA1_Channel2_3_IRQHandler()
{
	if(LL_DMA_IsActiveFlag_TC2(DMA1)) {
		LL_DMA_ClearFlag_TC2(DMA1);
		process_serial();
	}
	if(LL_DMA_IsActiveFlag_HT2(DMA1)) {
		LL_DMA_ClearFlag_HT2(DMA1);
		process_serial();
	}
}

////////////////////////////////////////////////////////////////////////////////

extern "C" void USART1_IRQHandler()
{
	if(LL_USART_IsActiveFlag_IDLE(USART1)) {
		LL_USART_ClearFlag_IDLE(USART1);
		process_serial();
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
// Frame updaters

void test()
{
}

////////////////////////////////////////////////////////////////////////////////

void intro()
{
    int f = frames & 8191;
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
}

////////////////////////////////////////////////////////////////////////////////

void fade()
{
    int f = 1024 - (frames - (5 * 1024));
    f = (f * f) >> 10;
    for(int i = 0; i < 60; ++i) {
        brightness[minutes_map[i]] = f;
    }
    ambient_scale = 0xffff;
    set_global_brightness(127);
}

////////////////////////////////////////////////////////////////////////////////

void clock()
{
    int seconds = (ticks >> 10) + 42 + (60 * 23) + (3600 * 20);
    int hours = (seconds / 3600) % 12;
    int minutes = (seconds / 60) % 60;

    int secs = seconds % 60;

    for(int i = 0; i < 12; ++i) {
        brightness[hours_map[i]] = 192;
    }

    for(int i = 0; i < secs; ++i) {
        brightness[minutes_map[i]] = 256;
    }
    brightness[minutes_map[secs]] = (util::min(1023u, (ticks & 1023) * 1) * 1) >> 2;
    for(int i = secs + 1; i < 60; ++i) {
        brightness[minutes_map[i]] = 0;
    }

    //set_digit(0, hours / 10);
    set_digit(0, 0);
    set_digit(1, hours % 10);
    set_digit(2, minutes / 10);
    set_digit(3, minutes % 10);
//    set_digit(4, secs / 10, 620);
//    set_digit(5, secs % 10, 620);
    set_digit(6, 'P', 900);
    
    uint flash = util::abs(((int)(ticks + 573) & 1023) - 512);
    flash = util::min(1023u, flash << 3) >> 2;
    flash = 256;

    brightness[colon_map[0]] = flash;
    brightness[colon_map[1]] = flash;
}

////////////////////////////////////////////////////////////////////////////////

typedef void (*const update_function_t)();

// index into this table at frames / 1024
// so
//  intro 0..4   (0    .. 5119 )
//  fade  5      (5120 .. 6143 )
//  clock 6      (6144 ..      )

update_function_t update_function[] = {
#if 1
    test
#else
    intro, intro, intro, intro, intro, fade, clock
#endif
};

void delay(int t)
{
	int n = ticks + t;
	for(int i=ticks; i<t; ++i) {
		__NOP();
	}
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

    uint32 ambient = 0;
    uint32 ambient_target = 0;

    // enable PWM clock
    TIM14->CR1 |= TIM_CR1_CEN;

    // enable 3-8 mux
    GPIOA->BSRR = 1 << 3;
	
	delay(100000);
	
	serial_read();
	
    while(1) {

        // kick off ADC for ambient light sensor
        LL_ADC_REG_StartConversion(ADC1);

        // waitvb
        while(vblank == 0) {
            __WFI();
        }
        vblank = 0;
        frames += 1;

        // wait for ADC reading to complete (which it will have done ages ago)
        while(LL_ADC_REG_IsConversionOngoing(ADC1) != 0) {
            __nop();
        }

		if(got_serial_data) {
			got_serial_data = false;
			int msg_start = ((int)new_pos - sizeof(message_t)) & 63;
			if(validate_message(msg_start)) {
				
				uart_byte(msg_start + offsetof(message_t, crc)) = 0;	// nobble the crc so we don't mistakenly interpret this as a new message any time soon

				// we have a valid message
				
				message_t const &m = last_msg;
				for(int i=0; i<7; ++i) {
					set_ascii(i, m.digit[i]);
				}
				for(int i = 0; i <= m.seconds; ++i) {
					brightness[minutes_map[i]] = 256;
				}
				//brightness[minutes_map[m.seconds]] = (util::min(1023u, (ticks & 1023) * 1) * 1) >> 2;
				for(int i = m.seconds + 1; i < 60; ++i) {
					brightness[minutes_map[i]] = 0;
				}
			}
		}

        // read ambient light ADC
        int adc = LL_ADC_REG_ReadConversionData12(ADC1);

        // high pass filter the ambient_light
        ambient_light = ((ambient_light * 31) >> 5) + adc;
        ambient_scale = ((ambient_light * 100) >> 5) + 8192;
        if(ambient_scale > 65535) {
            ambient_scale = 65535;
        }

        set_global_brightness(ambient_scale >> 9);
		
        // update frame buffer

        uint f = frames >> 10;
        update_function[util::min(util::countof(update_function) - 1, f)]();
        frame_update(brightness, spi_data[buffer]);
    }
}
