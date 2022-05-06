#include "MC3630_i2c.h"

#define I2C_MASTER_NUM              I2C_NUM_0
#define WRITE_BIT I2C_MASTER_WRITE  /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ    /*!< I2C master read */

static const char *TAG = "MC3630 driver";

static inline esp_err_t write_reg_8_nolock(MC3630_t *dev, uint8_t reg, uint8_t data);

esp_err_t MC3630_init_desc(MC3630_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio) 
{
    if (addr != MC3630_I2C_ADDR_HIGH ||  addr != MC3630_I2C_ADDR_LOW)
    {
        ESP_LOGE(TAG, "Invalid I2C address");
        return ESP_ERR_INVALID_ARG;
    }

    dev->i2c_dev.port = port;
    dev->i2c_dev.addr = addr;
    dev->i2c_dev.cfg.sda_io_num = sda_gpio;
    dev->i2c_dev.cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->i2c_dev.cfg.master.clk_speed = I2C_FREQ_HZ;
#endif

    return i2c_dev_create_mutex(&dev->i2c_dev);
}

esp_err_t MC3630_init_sensor(MC3630_t *dev) 
{
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);

    uint8_t data;
    data = 0x01000000; //I2C_EN
    i2c_dev_write_reg(&dev->i2c_dev, MC3630_MODE_C, &data, 1); //Put into standby
    data = 0x40;
    i2c_dev_write_reg(&dev->i2c_dev, MC3630_RESET, &data, 1); //reset

    vTaskDelay(10 / portTICK_PERIOD_MS); //wait for sensor to reset

    data = 0b01;
    i2c_dev_write_reg(&dev->i2c_dev, MC3630_FREG_1, &data, 1); //Set I2C mode
    data = 0x42;
    i2c_dev_write_reg(&dev->i2c_dev, MC3630_INIT_1, &data, 1);

    data = 0x01;
	i2c_dev_write_reg(&dev->i2c_dev, MC3630_DMX, &data, 1);
    data = 0x80;
	i2c_dev_write_reg(&dev->i2c_dev, MC3630_DMY, &data, 1);
    data = 0x00;
	i2c_dev_write_reg(&dev->i2c_dev, MC3630_DMZ, &data, 1);

    i2c_dev_write_reg(&dev->i2c_dev, MC3630_INIT_2, &data, 1);
    i2c_dev_write_reg(&dev->i2c_dev, MC3630_INIT_3, &data, 1);

	uint8_t devID;
    i2c_dev_read_reg(&dev->i2c_dev, MC3630_CHIP_ID, &devID, 1);

    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

	if (devID == 0x71) {
		return true;
	}
	else {
		return false;
	}
}

uint8_t MC3630_getChipID(MC3630_t *dev) {
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    uint8_t devID;
    i2c_dev_read_reg(&dev->i2c_dev, MC3630_CHIP_ID, &devID, 1);
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
	return devID;
}

uint8_t MC3630_read_accel(MC3630_t *dev, int16_t * accelX, int16_t * accelY, int16_t * accelZ) 
{
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    uint8_t rawData[6];
    i2c_dev_read_reg(&dev->i2c_dev, MC3630_XOUT_LSB, &rawData, 6); //read all data in one go

    *accelX = (short)((((unsigned short)rawData[1]) << 8) | rawData[0]);
    *accelY = (short)((((unsigned short)rawData[3]) << 8) | rawData[2]);
    *accelZ = (short)((((unsigned short)rawData[5]) << 8) | rawData[4]);

    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return true;
}

void MC3630_setMode(MC3630_t *dev, MC3630_mode_t mode) {
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    i2c_dev_write_reg(&dev->i2c_dev, MC3630_MODE_C, &mode, 1);
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
}

void MC3630_startSNIFF(MC3630_t *dev, MC3630_sniff_sr_t rate, MC3630_andorn_t logicandor, MC3630_sniff_mode_t sniff_mode) {
    MC3630_setMode(dev, MC3630_MODE_STANDBY); //set to standby to be able to write
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);

    uint8_t value;

    i2c_dev_read_reg(&dev->i2c_dev, MC3630_SNIFFTH_C, &value, 1); //read global sniff control

    switch(logicandor)
    {
    case MC3630_ANDORN_OR:  //Axis or mode
        value &= 0xBF;
        break;
    case MC3630_ANDORN_AND: //Axis and mode
        value |= 0x40;
        break;
    default:
        return;
        break;
    }

    switch (sniff_mode)
    {
    case MC3630_SNIFF_MODE_C2P:
        value &= 0x7F;
        break;
    case MC3630_SNIFF_MODE_C2B:
        value |= 0x80;
        break;
    default:
        return;
        break;
    }

    i2c_dev_write_reg(&dev->i2c_dev, MC3630_SNIFFTH_C, &value, 1); //set global sniff control
    i2c_dev_write_reg(&dev->i2c_dev, MC3630_SNIFF_C, &rate, 1); //set sniff rate

    uint8_t intEnable = 0x4; //mask for sniff interrupt
    i2c_dev_write_reg(&dev->i2c_dev, MC3630_INTR_C, &intEnable, 1); //enable interrupt
    
    uint8_t data = 0x42;
    i2c_dev_write_reg(&dev->i2c_dev, MC3630_INIT_1, &data, 1);
    
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    MC3630_setMode(dev, MC3630_MODE_SNIFF);
}

void MC3630_setSniffThreshhold(MC3630_t *dev, char axis, uint8_t sniff_thres) {

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    uint8_t controlReg;
    i2c_dev_read_reg(&dev->i2c_dev, MC3630_SNIFFCF_C, &controlReg, 1);

    controlReg &= 0b11111000; //zero lower 3 bits

    switch (axis)
    {
        case 'x':
            controlReg |= 0b00000001; //x thres
            break;
        case 'y':
            controlReg |= 0b00000010; //y thres
            break;
        case 'z':
            controlReg |= 0b00000011; //z thres
            break;
        default:
            printf("Error, Invalid Param");
            return;
            break;
    }

    i2c_dev_write_reg(&dev->i2c_dev, MC3630_SNIFFCF_C, &controlReg, 1); //Set the appropriate shadow register

    uint8_t valueReg;
    i2c_dev_read_reg(&dev->i2c_dev, MC3630_SNIFFTH_C, &valueReg, 1);
    valueReg &= 0b11000000; //zero lower 6 bits
    valueReg |= sniff_thres;
    i2c_dev_write_reg(&dev->i2c_dev, MC3630_SNIFFTH_C, &valueReg, 1);

    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

}

void MC3630_setSniffDectCount(MC3630_t *dev, char axis, uint8_t sniff_dect_cnt) {

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    uint8_t controlReg;
    i2c_dev_read_reg(&dev->i2c_dev, MC3630_SNIFFCF_C, &controlReg, 1);

    controlReg &= 0b11111000; //zero lower 3 bits

    switch (axis)
    {
        case 'x':
            controlReg |= 0b00000101; //x count
            break;
        case 'y':
            controlReg |= 0b00000110; //y count
            break;
        case 'z':
            controlReg |= 0b00000111; //z count
            break;
        default:
            printf("Error, Invalid Param");
            return;
            break;
    }
    controlReg |= 0x08; //enable sniff detection for all channels
    i2c_dev_write_reg(&dev->i2c_dev, MC3630_SNIFFCF_C, &controlReg, 1); //Set the appropriate shadow register

    uint8_t valueReg;
    i2c_dev_read_reg(&dev->i2c_dev, MC3630_SNIFFTH_C, &valueReg, 1);
    valueReg &= 0b11000000; //zero lower 6 bits
    valueReg |= sniff_dect_cnt;
    
    i2c_dev_write_reg(&dev->i2c_dev, MC3630_SNIFFTH_C, &valueReg, 1);
    
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

}

void MC3630_setRangeResolution(MC3630_t *dev, MC3630_range_t range, MC3630_resolution_t res) {
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    uint8_t value = range | res;
    i2c_dev_write_reg(&dev->i2c_dev, MC3630_RANGE_C, &value, 1);
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
}

uint8_t MC3630_clearInterrupts(MC3630_t *dev) 
{
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    uint8_t intVal;
    i2c_dev_read_reg(&dev->i2c_dev, MC3630_STATUS_2, &intVal, 1);
    uint8_t val = 0x3; //zero all interrupts
    i2c_dev_write_reg(&dev->i2c_dev, MC3630_STATUS_2, &val, 1);
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    return intVal;
}

uint8_t MC3630_getMode(MC3630_t *dev) {
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    uint8_t mode;
    i2c_dev_read_reg(&dev->i2c_dev, MC3630_STATUS_1, &mode, 1);
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
    return mode;
}

void MC3630_setPowerMode(MC3630_t *dev, MC3630_power_mode_t sniff_power, MC3630_power_mode_t cwake_power) {
    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    uint8_t val = 0;
    val |= sniff_power;
    val = val << 4;
    val |= cwake_power;
    i2c_dev_write_reg(&dev->i2c_dev, MC3630_PMCR, &val, 1);
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);
}

static inline esp_err_t write_reg_8_nolock(MC3630_t *dev, uint8_t reg, uint8_t data)
{
    return i2c_dev_write_reg(&dev->i2c_dev, reg, &data, 1);
}