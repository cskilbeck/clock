#include "main.h"
#include "string.h"
#include "util.h"

struct spi_packet
{
    byte *data;                 // spi_data to send
    uint32 column_mask;    // should it activate next column when this one is complete?
};

uint8 const ledmap[128] =
{
    15,13,12,11,10,    8,7,6,5,4,     2,1,0,31,30,     28,27,26,25,24,     22,21,20,19,18
};

#define TLC_SETCONFIG 192

#define TLC_BRIGHTNESS0 112
#define TLC_BRIGHTNESS6 118
#define TLC_BLANK 119
#define TLC_DSPRPT 120
#define TLC_TMGRST 121
#define TLC_ESPWM 122

uint8 spi_data[2][200];    // 25 * 8 * 2
uint8 config0[25];
uint8 config1[25];

spi_packet spi_packets[2][32];
spi_packet *next_spi_packet;

uint16 brightness[128];

uint32 column = 0;
int frames = 0;
int buffer = 0;

byte global_brightness = 127;

volatile int vblank = 0;

extern "C" void SysTick_Handler()
{
}

void led_set(byte *data, int bit, int val)
{
    int offset = 24 - (bit / 8);
    int mask = ~(1 << (bit & 7));
    int set = val << (bit & 7);
    data[offset] = data[offset] & mask | set;
}

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
    for(int i=0; i<2; ++i) {
        byte *s = spi_data[i];
        spi_packet *p = spi_packets[i];
        for(int j=0; j<8; ++j) {
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

    // init the fixed config spi_data (one for blank=1, one for blank=0)
    led_set(config0, TLC_BLANK, 1);
    led_set(config1, TLC_BLANK, 0);
    led_set(config0, TLC_SETCONFIG, 1);
    led_set(config1, TLC_SETCONFIG, 1);
    led_set(config0, TLC_DSPRPT, 0);
    led_set(config1, TLC_DSPRPT, 0);
    led_set(config0, TLC_ESPWM, 0);
    led_set(config1, TLC_ESPWM, 0);

    // full brightness
    for(int i = TLC_BRIGHTNESS0; i <= TLC_BRIGHTNESS6; ++i) {
        led_set(config0, i, 0);
        led_set(config1, i, 0);
    }
}

void frame_update(uint16 *brightness, byte *spi_data)
{
    uint16 *b = brightness;
    uint8 *p = spi_data;
    // 8 columns
    for(int i=0; i<8; ++i) {
        p += 1; // 1st 7 bits are spilled, 8th bit is the TLC_SETCONFIG, data starts after that

        // 16 uint16s (in 2s)
        for(int j = 0; j < 8; ++j) {
            uint16 brt1 = *b++;
            uint16 brt2 = *b++;
            *p++ = (brt1 >> 4) & 0xff;
            *p++ = ((brt1 << 4) & 0xf0) | ((brt2 >> 8) & 0xf);
            *p++ = brt2 & 0xff;
        }
    }
}

static inline void send_next_spi_packet()
{
    SPI1->CR1 = 0;
    SPI1->CR2 = 0;
    DMA1_Channel1->CCR = 0;

    if(next_spi_packet->data != null)
    {
        DMA1_Channel1->CPAR = (uint32_t)&SPI1->DR;
        DMA1_Channel1->CMAR = (uint32_t)next_spi_packet->data;
        DMA1_Channel1->CNDTR = 25;
        DMA1_Channel1->CCR = DMA_CCR_MINC | DMA_CCR_DIR | (2 << DMA_CCR_PL_Pos) | DMA_CCR_EN | DMA_CCR_TCIE;
        SPI1->CR1 = SPI_CR1_SPE | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_MSTR;
        SPI1->CR2 = SPI_CR2_TXDMAEN | (0x7 << SPI_CR2_DS_Pos);
        GPIOA->BSRR = ((7 << 16) | column) & next_spi_packet->column_mask;
        next_spi_packet += 1;
    }
}

extern "C" void DMA1_Channel1_IRQHandler()
{
	// clear the irq
    DMA1->IFCR = DMA_IFCR_CGIF1 | DMA_IFCR_CTCIF1;

    // wait for last SPI to complete
    while((SPI1->SR & SPI_SR_BSY) != 0) {
    }

    // latch on
    GPO_LED_LATCH_GPIO_Port->BSRR = GPO_LED_LATCH_Pin;

    // send the next spi_packet if there is one
    send_next_spi_packet();

    // latch off
    GPO_LED_LATCH_GPIO_Port->BSRR = GPO_LED_LATCH_Pin << 16;
}

// next column irq
// this irq should happen _just_ around the time the pwm for a given column is complete

extern "C" void TIM17_IRQHandler()
{
    TIM17->SR = 0;
    column = (column + 1) & 7;
    next_spi_packet = spi_packets[buffer] + (column * 4);
    send_next_spi_packet();
    if(column == 0) {
        buffer = 1 - buffer;
        vblank = 1;
    }
}

extern "C" void user_main()
{
    // no systick
    SysTick->CTRL  = 0;

    NVIC_DisableIRQ(SysTick_IRQn);
    NVIC_DisableIRQ(TIM14_IRQn);
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    NVIC_EnableIRQ(TIM17_IRQn);

    led_data_init();

    // enable LED PWM clock
    TIM14->DIER = 0;
    TIM14->CCER |= TIM_CCER_CC1E;
    TIM14->CR1 |= TIM_CR1_CEN;

    // start led driver timer
    TIM17->SR &= TIM_SR_UIF;
    TIM17->ARR = 5000;
    TIM17->PSC = 0;
    TIM17->DIER |= TIM_DIER_UIE;
    TIM17->CR1 |= TIM_CR1_CEN;
	
    LL_ADC_StartCalibration(ADC1);
    while(LL_ADC_IsCalibrationOnGoing(ADC1)) {
    }

    LL_ADC_Enable(ADC1);

    uint32 ambient = 0;
    uint32 ambient_target = 0;
    
    GPIOA->BSRR = 1 << 3;
    
    uint32 b = 0;
    while(1) {
        GPO_DEBUG_LED_GPIO_Port->BSRR = GPO_DEBUG_LED_Pin;
        
        LL_ADC_REG_StartConversion(ADC1);
        
        while(vblank == 0) {
            __WFI();
        }

        while(LL_ADC_REG_IsConversionOngoing(ADC1) != 0) {
            __nop();
        }

        LL_ADC_ClearFlag_EOC(ADC1);

        int x = LL_ADC_REG_ReadConversionData12(ADC1);
        
        vblank = 0;
        frames += 1;
        GPO_DEBUG_LED_GPIO_Port->BSRR = GPO_DEBUG_LED_Pin << 16;

        b = (b + 8) & 65535;

        int c = (x - 0x180);
        if(c < 0) {
            c = 0;
        }
        if(c > 4095) {
            c = 4095;
        }
        
        c -= ambient;
        
        if(c < -2) {
            c = -2;
        }
        else if(c > 2) {
            c = 2;
        }
        ambient += c;
        
        if(ambient < 32) {
            ambient = 32;
        }
        
        c = ((ambient * ambient) >> (12 + 3)) + 4;
        
        c = 0 << 4;

        config0[10] = 0x80 | global_brightness;
        config1[10] = global_brightness;

        int q = (frames >> 4) & 1023;
        q = (q * q) >> 10;
        for(int i=0; i<128; ++i) {
            brightness[i] = 0;
        }
        brightness[7] = 1023;
        frame_update(brightness, spi_data[buffer]);
    }
}
