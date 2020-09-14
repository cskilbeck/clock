#include "main.h"

// #define CLK_Pin GPIO_PIN_5
// #define ENABLE_Pin GPIO_PIN_6
// #define MOSI_Pin GPIO_PIN_7
// #define LATCH_Pin GPIO_PIN_3

#define LATCH_Pin_Position 3
#define MOSI_Pin_Position 7
#define CLK_Pin_Position 5

typedef struct packet_t
{
    uint64_t time;
    uint64_t flags;
} packet_t;

namespace led
{

    // led driver registers

    enum reg : int
    {
        Write_switch = 2,    // 16 bit on/off mask
        Data_latch = 4,      // set the next pwm value
        Global_latch = 6,    // set the last pwm value and latch them all in
        Write_CR = 7,        // write the config register
        Read_CR = 8,         // read config

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

#define one_bit(n)                                        \
    GPIOA->BSRR = pins | ((data >> (16 - n)) & MOSI_Pin); \
    GPIOA->BSRR = CLK_Pin;

        switch(reg) {
        case 1:
            one_bit(0);
        case 2:
            one_bit(1);
        case 3:
            one_bit(2);
        case 4:
            one_bit(3);
        case 5:
            one_bit(4);
        case 6:
            one_bit(5);
        case 7:
            one_bit(6);
        case 8:
            one_bit(7);
        case 9:
            one_bit(8);
        case 10:
            one_bit(9);
        case 11:
            one_bit(10);
        case 12:
            one_bit(11);
        case 13:
            one_bit(12);
        case 14:
            one_bit(13);
        case 15:
            one_bit(14);
        }

        pins |= LATCH_Pin;
        data >>= reg;

        switch(16 - reg) {
        case 1:
            one_bit(0);
        case 2:
            one_bit(1);
        case 3:
            one_bit(2);
        case 4:
            one_bit(3);
        case 5:
            one_bit(4);
        case 6:
            one_bit(5);
        case 7:
            one_bit(6);
        case 8:
            one_bit(7);
        case 9:
            one_bit(8);
        case 10:
            one_bit(9);
        case 11:
            one_bit(10);
        case 12:
            one_bit(11);
        case 13:
            one_bit(12);
        case 14:
            one_bit(13);
        case 15:
            one_bit(14);
        }
    }

    void set_enabled(bool enabled)
    {
        GPIOA->BSRR = ENABLE_Pin << (int(enabled) << 4);
    }
    
    void set_column(int column)
    {
        GPIOA->BSRR = (column & 7) | (7 << 16);
    }

};    // namespace led

uint8_t buffers[2][16];
int current_buffer_id = 0;  // which one has the most recent completed read in it
volatile int read_complete = 0;

extern "C" void i2c_read_complete(I2C_HandleTypeDef *hi2c)
{
    HAL_I2C_Slave_Receive_DMA(&hi2c1, buffers[current_buffer_id], 16);
    current_buffer_id = 1 - current_buffer_id;
    read_complete = 1;
}

extern "C" void i2c_listen_complete(I2C_HandleTypeDef *hi2c)
{
}

extern "C" void i2c_error(I2C_HandleTypeDef *hi2c)
{
}

uint16_t scale(uint8_t v)
{
    uint32_t x = (v * 4112U) >> 8;  // scale 0..255 -> 0..4095
    return (uint16_t)((((x * x) >> 12) * x) >> 12);   // ghetto gamma ramp (cube it)
    //return v << 4;
}

int frames = 0;

extern "C" void user_main(void)
{
    // disable systick
    SysTick->CTRL = ~SysTick_CTRL_ENABLE_Msk;

    // leds off to start with
    led::set_enabled(false);

    // start led pwm timer
    HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);

    // high current range, current setting 31, 12 bit pwm
    led::set(0x803f, led::Write_CR);

    // switch them all on
    led::set(0xffff, led::Write_switch);

    int b = 0;
    int vel = 1;

    int reg[16];
    for(int i = 0; i < 15; ++i) {
        reg[i] = led::Data_latch;
    }
    reg[15] = led::Global_latch;

    HAL_I2C_RegisterCallback(&hi2c1, HAL_I2C_SLAVE_RX_COMPLETE_CB_ID, i2c_read_complete);
    //HAL_I2C_RegisterCallback(&hi2c1, HAL_I2C_LISTEN_COMPLETE_CB_ID, i2c_listen_complete);
    //HAL_I2C_RegisterCallback(&hi2c1, HAL_I2C_ERROR_CB_ID, i2c_error);

    current_buffer_id = 0;    
    HAL_I2C_Slave_Receive_DMA(&hi2c1, buffers[1 - current_buffer_id], 16);
    
    packet_t packet;

    while(1) {

        if(read_complete) {
            read_complete = 0;
            memcpy(&packet, buffers[current_buffer_id], 16);
        }
        
        int32_t x = (int32_t)packet.time;
        
        // set the led intensities
        for(int i = 0; i < 16; ++i) {
            if(i == 15) {
                // global output disable while we switch to the new column
                led::set_enabled(false);
            }
            int32_t q = x;
            if(q > 4095) {
                q = 4095;
            }
            led::set(q, reg[i]);
            x -= 4096;
            if(x <= 0) {
                x = 0;
            }
        }
        // set column here
        led::set_column(3);//((frames & 31) > 15) ? 3 : 4);

        // global output enable the next column
        led::set_enabled(true);
        
        // waitvb
        for(volatile int i = 0; i < 1000; ++i) {
        }
        
        frames += 1;
    }
}
