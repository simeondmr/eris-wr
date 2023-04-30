#include "driver/i2c.h"
#include "driver_adxl345.h"

DriverAdxl345::DriverAdxl345(uint8_t i2cMasterPort, PowerMode powerMode, BwRate bwRate, Range range)
{
    this->i2cMasterPort = i2cMasterPort;
    this->powerMode = powerMode;
    this->bwRate = bwRate;
    this->range = range;
}

esp_err_t DriverAdxl345::init()
{
    if (this->getDeviceId() != ADXL345_SLAVE_ADDR) {
        return ESP_FAIL;
    }
    
    ESP_RETURN_ON_ERROR(i2c_write_byte(this->i2cMasterPort, ADXL345_SLAVE_ADDR, POWER_CTL_REG, MEASUREMENT_MODE),ADXL345_ERROR_TAG, ADXL345_CFG_ERR_MSG);
    ESP_RETURN_ON_ERROR(i2c_write_byte(this->i2cMasterPort, ADXL345_SLAVE_ADDR, DATA_FORMAT_REG, 0x03 | this->range), ADXL345_ERROR_TAG, ADXL345_CFG_ERR_MSG);
    return ESP_OK;
}



uint8_t DriverAdxl345::getDeviceId()
{
    uint8_t addr = 0;
    uint8_t reg = 0x00;
   // printf("Test %d\n", i2c_master_write_read_device(this->i2cMasterPort, ADXL345_SLAVE_ADDR, &reg, 1, &addr, sizeof(addr), 1 / portTICK_RATE_MS));; 

    return i2c_read_byte(this->i2cMasterPort, ADXL345_SLAVE_ADDR, DEV_ID_REG);
   //return addr;
}