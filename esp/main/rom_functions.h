
#pragma once

extern "C" {

#ifndef i2c_bbpll

#define i2c_bbpll 0x67
#define i2c_bbpll_en_audio_clock_out 4
#define i2c_bbpll_en_audio_clock_out_msb 7
#define i2c_bbpll_en_audio_clock_out_lsb 7
#define i2c_bbpll_hostid 4

/* ROM functions which read/write internal control bus */
uint8_t rom_i2c_readReg(uint8_t block, uint8_t host_id, uint8_t reg_add);
uint8_t rom_i2c_readReg_Mask(uint8_t block, uint8_t host_id, uint8_t reg_add, uint8_t msb, uint8_t lsb);
void rom_i2c_writeReg(uint8_t block, uint8_t host_id, uint8_t reg_add, uint8_t data);
void rom_i2c_writeReg_Mask(uint8_t block, uint8_t host_id, uint8_t reg_add, uint8_t msb, uint8_t lsb, uint8_t data);

#define i2c_writeReg_Mask(block, host_id, reg_add, Msb, Lsb, indata) rom_i2c_writeReg_Mask(block, host_id, reg_add, Msb, Lsb, indata)
#define i2c_writeReg_Mask_def(block, reg_add, indata) i2c_writeReg_Mask(block, block##_hostid, reg_add, reg_add##_msb, reg_add##_lsb, indata)
#endif    // i2c_bbpll

#define I2S_CLK_ENABLE() i2c_writeReg_Mask_def(i2c_bbpll, i2c_bbpll_en_audio_clock_out, 1)
#define I2S_CLK_DISABLE() i2c_writeReg_Mask_def(i2c_bbpll, i2c_bbpll_en_audio_clock_out, 0)

}    // extern "C"
