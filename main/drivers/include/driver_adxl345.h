#ifndef __DRIVER_ADXL345_H
#define __DRIVER_ADXL345_H

#include "esp_check.h"

#define ADXL345_SLAVE_ADDR      0x53

/* ADXL345's registers */
#define DEV_ID_REG              0x00
#define DATA_FORMAT_REG         0x31
#define POWER_CTL_REG           0x2d

#define MEASUREMENT_MODE        0x08
#define ADXL345_ERROR_TAG       "ADXL345 error"
#define ADXL345_CFG_ERR_MSG     "Configuration error"


class DriverAdxl345 {
public:

    enum PowerMode {
        NORMAL          = (uint8_t) 0,
        REDUCED_POWER   = (uint8_t) 1
    };

    enum BwRate {
        RATE_3200_HZ    = (uint8_t) 0x0f,
        RATE_1600_HZ    = (uint8_t) 0x0e,
        RATE_800_HZ     = (uint8_t) 0x0d,
        RATE_400_HZ     = (uint8_t) 0x0c,
        RATE_200_HZ     = (uint8_t) 0x0b,
        RATE_100_HZ     = (uint8_t) 0x0a,
        RATE_50_HZ      = (uint8_t) 0x09,
        RATE_25_HZ      = (uint8_t) 0x08,
        RATE_12_5_HZ    = (uint8_t) 0x07, 
        RATE_6_25_HZ    = (uint8_t) 0x06,
        RATE_3_13_HZ    = (uint8_t) 0x05,
        RATE_1_56_HZ    = (uint8_t) 0x04,
        RATE_0_78_HZ    = (uint8_t) 0x03,
        RATE_0_39_HZ    = (uint8_t) 0x02,
        RATE_0_20_HZ    = (uint8_t) 0x01,
        RATE_0_10_HZ    = (uint8_t) 0x00
    };

    enum Range {
        G2_RANGE        = (uint8_t) 0x00,
        G4_RANGE        = (uint8_t) 0x01,
        G8_RANGE        = (uint8_t) 0x02,
        G16_RANGE       = (uint8_t) 0x03
    };

    DriverAdxl345(uint8_t i2cMasterPort, PowerMode powerMode, BwRate bwRate, Range range);

    esp_err_t init();
    uint8_t getDeviceId();

private:
    uint8_t i2cMasterPort;
    PowerMode powerMode;
    BwRate bwRate;
    Range range;


};

#endif