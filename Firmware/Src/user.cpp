#include "main.h"

// #define CLK_Pin GPIO_PIN_5
// #define CLK_GPIO_Port GPIOA
// #define ENABLE_Pin GPIO_PIN_6
// #define ENABLE_GPIO_Port GPIOA
// #define MOSI_Pin GPIO_PIN_7
// #define MOSI_GPIO_Port GPIOA
// #define LATCH_Pin GPIO_PIN_9
// #define LATCH_GPIO_Port GPIOA

#define LATCH_Pin_Position 9
#define MOSI_Pin_Position 7
#define CLK_Pin_Position 5

namespace led
{

// led driver registers

enum reg : int
{
    Write_switch = 2,   // 16 bit on/off mask
    Data_latch = 4,     // set the next pwm value
    Global_latch = 6,   // set the last pwm value and latch them all in
    Write_CR = 7,       // write the config register
    Read_CR = 8,        // read config

    // etc
    
    Start_open_error_detection = 9,
    Start_short_error_detection = 10,
    Start_combined_detection = 11,
    End_error_detection = 12,
    LE_Thermal_error_reading = 13
};

// set a register in the led driver

void set(int data, int reg)
{
    // set all the 'off' bits which are overridden by the 'on' bits
    uint32_t pins = (CLK_Pin << 16) | (MOSI_Pin << 16) | (LATCH_Pin << 16);
    
    // length of latch specifies register to write to

    data = (data << 16) >> (16 - MOSI_Pin_Position);

#define one_bit(n) \
            GPIOA->BSRR = pins | ((data >> (16 - n)) & MOSI_Pin); \
            GPIOA->BSRR = CLK_Pin;

    switch(reg) {
        case 1: one_bit(0);
        case 2: one_bit(1);
        case 3: one_bit(2);
        case 4: one_bit(3);
        case 5: one_bit(4);
        case 6: one_bit(5);
        case 7: one_bit(6);
        case 8: one_bit(7);
        case 9: one_bit(8);
        case 10:one_bit(9);
        case 11:one_bit(10);
        case 12:one_bit(11);
        case 13:one_bit(12);
        case 14:one_bit(13);
        case 15:one_bit(14);
    }

    pins |= LATCH_Pin;
    data >>= reg;

    switch(16-reg) {
        case 1: one_bit(0);
        case 2: one_bit(1);
        case 3: one_bit(2);
        case 4: one_bit(3);
        case 5: one_bit(4);
        case 6: one_bit(5);
        case 7: one_bit(6);
        case 8: one_bit(7);
        case 9: one_bit(8);
        case 10:one_bit(9);
        case 11:one_bit(10);
        case 12:one_bit(11);
        case 13:one_bit(12);
        case 14:one_bit(13);
        case 15:one_bit(14);
    }
}

void set_enabled(bool enabled)
{
    uint32_t bits = ENABLE_Pin;
    if(enabled) {
        bits <<= 16;
    }
    GPIOA->BSRR = bits;
}

}; // namespace led

extern"C" void user_main(void)
{
    // disable systick
    SysTick->CTRL  = ~SysTick_CTRL_ENABLE_Msk;
    
     // start led pwm timer
    HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);

    // high current range, current setting 31, 12 bit pwm
    led::set(0x803f, led::Write_CR);
    
    // switch them all on
    led::set(0xffff, led::Write_switch);

    int b = 0;
    int vel = 1;
    
    int reg[16];
    for(int i=0; i<15; ++i) {
        reg[i] = led::Data_latch;
    }
    reg[15] = led::Global_latch;

    while(1) {
        
        int c[2];
        int b2 = 4095 - b;

        // cube for gamma
        c[0] = (((b * b) >> 12) * b) >> 12;
        c[1] = (((b2 * b2) >> 12) * b2) >> 12;

        // set the led intensities
        GPIOA->BSRR = 1<<10;
        int i;
        for(i=0; i<15; ++i) {
            led::set(c[i & 1], reg[i]);
        }
        // global output disable while we switch to the new column
        led::set_enabled(false);

        led::set(c[i & 1], reg[i]);
        GPIOA->BSRR = 1<<(10+16);

        // set column here
        
        // global output enable the next column
        led::set_enabled(true);
       
        // brightness function
        b += vel;
        if(b < 0 || b > 4095) {
            vel = -vel;
            b += vel;
        }

        // waitvb
        for(volatile int i=0; i<1000; ++i) {
        }
    }
}
