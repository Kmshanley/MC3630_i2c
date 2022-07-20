#pragma once
#include <driver/i2c.h>
#include <esp_log.h>
#include <esp_err.h>
#include <i2cdev.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define I2C_NUM (I2C_NUM_0)

#define MC3630_I2C_ADDR_LOW 0x4C
#define MC3630_I2C_ADDR_HIGH 0x6C

#define MC3630_INIT_1 0x0F
#define MC3630_INIT_2 0x28
#define MC3630_INIT_3 0x1A

#define MC3630_EXT_STAT_1 0x00
#define MC3630_EXT_STAT_2 0x01

#define MC3630_XOUT_LSB 0x02
#define MC3630_XOUT_MSB 0x03
#define MC3630_YOUT_LSB 0x04
#define MC3630_YOUT_MSB 0x05
#define MC3630_ZOUT_LSB 0x06
#define MC3630_ZOUT_MSB 0x07

#define MC3630_STATUS_1 0x08
#define MC3630_STATUS_2 0x09

#define MC3630_FREG_1 0x0D
#define MC3630_FREG_2 0x0E

#define MC3630_MODE_C 0x10 //Mode Control
#define MC3630_RATE_1 0x11
#define MC3630_SNIFF_C 0x12 //Sniff control
#define MC3630_SNIFFTH_C 0x13 //Sniff threshold control
#define MC3630_SNIFFCF_C 0x14 //Sniff Configuration
#define MC3630_RANGE_C 0x15 //Range resolution control
#define MC3630_FIFO_C 0x16
#define MC3630_INTR_C 0x17
#define MC3630_CHIP_ID 0x18
#define MC3630_INIT_3 0x1A
#define MC3630_PMCR 0x1C

#define MC3630_DMX 0x20
#define MC3630_DMY 0x21
#define MC3630_DMZ 0x22

#define MC3630_RESET 0x24

#define MC3630_INIT2 0x28
#define MC3630_TRIGC 0x29

#define MC3630_XOFFL 0x2A
#define MC3630_XOFFH 0x2B
#define MC3630_YOFFL 0x2C
#define MC3630_YOFFH 0x2D
#define MC3630_ZOFFL 0x2E
#define MC3630_ZOFFH 0x2F

#define MC3630_XGAIN 0x30
#define MC3630_YGAIN 0x31
#define MC3630_ZGAIN 0x32

#define I2C_FREQ_HZ 1000000

typedef enum
{
    MC3630_GAIN_DEFAULT    = 0b00,
    MC3630_GAIN_4X         = 0b01,
    MC3630_GAIN_1X         = 0b10,
    MC3630_GAIN_NOT_USED   = 0b11,
}   MC3630_gain_t;

typedef enum
{
    MC3630_MODE_SLEEP      = 0b000,
    MC3630_MODE_STANDBY    = 0b001,
    MC3630_MODE_SNIFF      = 0b010,
    MC3630_MODE_CWAKE      = 0b101,
    MC3630_MODE_TRIG       = 0b111,
}   MC3630_mode_t;

typedef enum
{
    MC3630_RANGE_2G    = 0b000,
    MC3630_RANGE_4G    = 0b001,
    MC3630_RANGE_8G    = 0b010,
    MC3630_RANGE_16G   = 0b011,
    MC3630_RANGE_12G   = 0b100,
    MC3630_RANGE_END,
}   MC3630_range_t;

typedef enum
{
    MC3630_RESOLUTION_6BIT    = 0b000,
    MC3630_RESOLUTION_7BIT    = 0b001,
    MC3630_RESOLUTION_8BIT    = 0b010,
    MC3630_RESOLUTION_10BIT   = 0b011,
    MC3630_RESOLUTION_12BIT   = 0b100,
    MC3630_RESOLUTION_14BIT   = 0b101,  //(Do not select if FIFO enabled)
    MC3630_RESOLUTION_END,
}   MC3630_resolution_t;

typedef enum
{
    MC3630_CWAKE_SR_DEFAULT_54Hz = 0b0000,
    MC3630_CWAKE_SR_14Hz         = 0b0101,
    MC3630_CWAKE_SR_28Hz         = 0b0110,
    MC3630_CWAKE_SR_54Hz         = 0b0111,
    MC3630_CWAKE_SR_105Hz        = 0b1000,
    MC3630_CWAKE_SR_210Hz        = 0b1001,
    MC3630_CWAKE_SR_400Hz        = 0b1010,
    MC3630_CWAKE_SR_600Hz        = 0b1011,
    MC3630_CWAKE_SR_END,
}   MC3630_cwake_sr_t;

typedef enum
{
    MC3630_SNIFF_SR_DEFAULT_7Hz = 0b0000,
    MC3630_SNIFF_SR_0p4Hz       = 0b0001,
    MC3630_SNIFF_SR_0p8Hz       = 0b0010,
    MC3630_SNIFF_SR_1p5Hz       = 0b0011,
    MC3630_SNIFF_SR_7Hz         = 0b0100,
    MC3630_SNIFF_SR_14Hz        = 0b0101,
    MC3630_SNIFF_SR_28Hz        = 0b0110,
    MC3630_SNIFF_SR_54Hz        = 0b0111,
    MC3630_SNIFF_SR_105Hz       = 0b1000,
    MC3630_SNIFF_SR_210Hz       = 0b1001,
    MC3630_SNIFF_SR_400Hz       = 0b1010,
    MC3630_SNIFF_SR_600Hz       = 0b1011,
    MC3630_SNIFF_SR_END,
}   MC3630_sniff_sr_t;

typedef enum
{
    MC3630_ANDORN_OR = 0,       //triggered when any of the active channels have met detection threshold
	MC3630_ANDORN_AND,          //triggered when all of the active channels have met detection threshold
    MC3630_ANDORN_END,
}   MC3630_andorn_t;

typedef enum
{
    MC3630_SNIFF_MODE_C2P = 0,  //Current to previous 
	MC3630_SNIFF_MODE_C2B,      //Current to baseline
    MC3630_SNIFF_MODE_END,
}   MC3630_sniff_mode_t;

typedef enum 
{
    MC3630_POWER_MODE_LOW = 0b000,
    MC3630_POWER_MODE_ULTRA_LOW = 0b011,
    MC3630_POWER_MODE_PRECISION = 0b100,
}   MC3630_power_mode_t;

typedef struct
{
    i2c_dev_t i2c_dev;              //!< I2C device descriptor
} MC3630_t;

esp_err_t MC3630_init_desc(MC3630_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);
esp_err_t MC3630_free_desc(MC3630_t *dev);
esp_err_t MC3630_init_sensor(MC3630_t *dev);
uint8_t MC3630_getChipID(MC3630_t *dev);
uint8_t MC3630_read_accel(MC3630_t *dev, int16_t * accelX, int16_t * accelY, int16_t * accelZ);
void MC3630_setMode(MC3630_t *dev, MC3630_mode_t mode);
void MC3630_setResolution(MC3630_t *dev, MC3630_resolution_t res);
void MC3630_startSNIFF(MC3630_t *dev, MC3630_sniff_sr_t rate, MC3630_andorn_t logicandor, MC3630_sniff_mode_t sniff_mode, bool activeHigh);
void MC3630_setSniffThreshhold(MC3630_t *dev, char axis, uint8_t sniff_thres);
void MC3630_setSniffDectCount(MC3630_t *dev, char axis, uint8_t sniff_dect_cnt);
void MC3630_setRangeResolution(MC3630_t *dev, MC3630_range_t range, MC3630_resolution_t res);
void MC3630_setPowerMode(MC3630_t *dev, MC3630_power_mode_t sniff_power, MC3630_power_mode_t cwake_power);
uint8_t MC3630_clearInterrupts(MC3630_t *dev);
uint8_t MC3630_getMode(MC3630_t *dev);