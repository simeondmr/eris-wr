/**
 * @author: Simeon Tornabene, alias dmr
 * Note: driver implementation, using the following Datasheet https://www.infineon.com/dgdl/Infineon-DPS310-DataSheet-v01_02-EN.pdf?fileId=5546d462576f34750157750826c42242, Revision v1.2
 * */

#ifndef __DRIVER_DPS310_H
#define __DRIVER_DPS310_H

#include "esp_check.h"

/* Scale factor values */
#define SCALE_FACTOR_RATE1          524288
#define SCALE_FACTOR_RATE2          1572864
#define SCALE_FACTOR_RATE4          3670016
#define SCALE_FACTOR_RATE8          7864320
#define SCALE_FACTOR_RATE16         253952
#define SCALE_FACTOR_RATE32         516096
#define SCALE_FACTOR_RATE64         1040384
#define SCALE_FACTOR_RATE128        2088960

/* Product and Revision ID masks */
#define REV_ID                      0xf0
#define PROD_ID                     0x0f

/* Registers address */
#define PSR_B2_REG                  0x00
#define PSR_B1_REG                  0x01
#define PSR_B0_REG                  0x02
#define TMP_B2_REG                  0x03
#define TMP_B1_REG                  0x04
#define TMP_B0_REG                  0x05
#define PRS_CFG_REG                 0x06
#define TMP_CFG_REG                 0x07
#define MEAS_CFG_REG                0x08
#define CFG_REG                     0x09
#define INT_STS_REG                 0x0a
#define FIFO_STS_REG                0x0b
#define RESET_REG                   0x0c
#define PRODUCT_ID_REG              0x0d
#define COEFF_FROM_REG              0x10
#define COEFF_TO_REG                0x21
#define COEFF_REG_NUM_REG           COEFF_TO_REG - COEFF_FROM_REG
#define COEF_SRCE_REG               0x28

/* DPS310's i2c slave address*/
#define DPS310_SLAVE_ADDR           0x77

/* Product and Revision ID value */
#define PRODUCT_ID_VAL              0x00
#define REV_ID_VAL                  0x01

/* Number or coefficients */
#define N_COEFF                     0x09

/* Coefficients' array indexes */
#define C0_INDEX                    0x00
#define C1_INDEX                    0x01
#define C00_INDEX                   0x02
#define C10_INDEX                   0x03
#define C01_INDEX                   0x04
#define C11_INDEX                   0x05
#define C20_INDEX                   0x06
#define C21_INDEX                   0x07
#define C30_INDEX                   0x08

/* Coefficients register's val index */
#define C0_0_VAL_INDEX              0x00
#define C0_1_C1_0_REG_VAL_INDEX     0x01
#define C1_1_VAL_INDEX              0x02
#define C00_0_VAL_INDEX             0x03
#define C00_1_VAL_INDEX             0x04
#define C00_2_C10_0_VAL_INDEX       0x05
#define C10_1_VAL_INDEX             0x06
#define C10_2_VAL_INDEX             0x07
#define C01_0_VAL_INDEX             0x08
#define C01_1_VAL_INDEX             0x09
#define C11_0_VAL_INDEX             0x0a
#define C11_1_VAL_INDEX             0x0b
#define C20_0_VAL_INDEX             0x0c
#define C20_1_VAL_INDEX             0x0d
#define C21_0_VAL_INDEX             0x0e
#define C21_1_VAL_INDEX             0x0f
#define C30_0_VAL_INDEX             0x10
#define C30_1_VAL_INDEX             0x11


/* Number of bits for Pressure and Temperature for two's complement  */
#define MEASUREMENT_BIT_LEN         24

/* Coefficient's number of bits*/
#define C0_BIT_LEN                  0x0c
#define C1_BIT_LEN                  0x0c
#define C00_BIT_LEN                 0x14
#define C10_BIT_LEN                 0x14
#define C01_BIT_LEN                 0x10
#define C11_BIT_LEN                 0x10
#define C20_BIT_LEN                 0x10
#define C21_BIT_LEN                 0x10
#define C30_BIT_LEN                 0x10

/* Status masks*/
#define TMP_RDY_MASK                0x02
#define PRS_RDY_MASK                0x0d
#define COEFF_RDY_VAL               0x03

#define N_RTRY_DATA_RDY             0x64

/*Error TAG and MSGs*/
#define DPS310_ERROR_TAG            "DPS310 error"
#define DPS310_RESET_ERR_MSG        "Reset error"
#define DPS310_CFG_ERR_MSG          "Configuration error"
#define DPS310_TMP_WAIT_TIMEOUT     "Temperature waiting timeout"
#define DPS310_PRESS_WAIT_TIMEOUT   "Pressure waiting timeout"
#define DPS310_MEASUREMENT_ERR      "Measurement error"
#define DPS310_I2C_ERR              "I2C ERR"


/* Soft reset value, for RESET register */
#define SOFT_RESET_REGISTER_VAL     0x09

class DriverDPS310 {
public:
    struct ProdRevId
    {
        uint8_t revID;
        uint8_t prodID;
    };

    struct TmpMeasure {
        double tmpRawScal;
        double tmpComp;
    };

    struct PressMeasure {
        double pressRawScal;
        double pressComp;
    };

    struct Measure {
        TmpMeasure tmpMeasure;
        PressMeasure pressMeasure;
    };

    enum PressMeasurementRate {
        PM_RATE_1       = (uint8_t) 0x00,
        PM_RATE_2       = (uint8_t) 0x10,
        PM_RATE_4       = (uint8_t) 0x20,
        PM_RATE_8       = (uint8_t) 0x30,
        PM_RATE_16      = (uint8_t) 0x40,
        PM_RATE_32      = (uint8_t) 0x50,
        PM_RATE_64      = (uint8_t) 0x60,
        PM_RATE_128     = (uint8_t) 0x70 
    };

    enum PressOverSamplingRate {
        PO_RATE_1       = (uint8_t) 0x00,
        PO_RATE_2       = (uint8_t) 0x01,
        PO_RATE_4       = (uint8_t) 0x02,
        PO_RATE_8       = (uint8_t) 0x03,
        PO_RATE_16      = (uint8_t) 0x04,
        PO_RATE_32      = (uint8_t) 0x05,
        PO_RATE_64      = (uint8_t) 0x06,
        PO_RATE_128     = (uint8_t) 0x07 
    };

    enum InternalExternalSensor {
        INTERNAL_SENS   = (uint8_t) 0x00,
        EXTERNAL_SENS   = (uint8_t) 0x80
    };

    enum TmpMeasurementRate {
        TMPM_RATE_1     = (uint8_t) 0x00,
        TMPM_RATE_2     = (uint8_t) 0x10,
        TMPM_RATE_4     = (uint8_t) 0x20,
        TMPM_RATE_8     = (uint8_t) 0x30,
        TMPM_RATE_16    = (uint8_t) 0x40,
        TMPM_RATE_32    = (uint8_t) 0x50,
        TMPM_RATE_64    = (uint8_t) 0x60,
        TMPM_RATE_128   = (uint8_t) 0x70 
    };

    enum TmpOversamplingRate {
        TMPO_RATE_1     = (uint8_t) 0x00,
        TMPO_RATE_2     = (uint8_t) 0x01,
        TMPO_RATE_4     = (uint8_t) 0x02,
        TMPO_RATE_8     = (uint8_t) 0x03,
        TMPO_RATE_16    = (uint8_t) 0x04,
        TMPO_RATE_32    = (uint8_t) 0x05,
        TMPO_RATE_64    = (uint8_t) 0x06,
        TMPO_RATE_128   = (uint8_t) 0x07 
    };

    enum MeasurementConfig {
        IDLE_STOP       = (uint8_t) 0x00,
        P_MEAS          = (uint8_t) 0x01,
        TMP_MEAS        = (uint8_t) 0x02,
        CONT_P_MEAS     = (uint8_t) 0x05,
        CONT_TMP_MEAS   = (uint8_t) 0x06,
        CONT_P_TMP_MEAS = (uint8_t) 0x07
    };

    enum ConfigRegister {
        INT_HL          = (uint8_t) 0x80,
        INT_FIFO        = (uint8_t) 0x40,
        INT_TMP         = (uint8_t) 0x20,
        INT_PRS         = (uint8_t) 0x10,
        T_SHIFT         = (uint8_t) 0x08,
        P_SHIFT         = (uint8_t) 0x04,
        FIFO_EN         = (uint8_t) 0x02,
        SPI_MODE        = (uint8_t) 0x01
    };

    DriverDPS310(uint8_t i2cMasterPort, 
                PressMeasurementRate pmRate, 
                PressOverSamplingRate pOverSamplRate, 
                InternalExternalSensor internalExternalSensor,
                TmpMeasurementRate tmpmRate, 
                TmpOversamplingRate tmpoRate, 
                MeasurementConfig measurementConfig);

    /**
     * getProdRevId()
     * @arg1: product revision id
     * Return:
     * - ESP_OK - if a correct initialization was done
     * - ESP_FAIL - in case of initialization error
     **/
    esp_err_t getProdRevId(DriverDPS310::ProdRevId *prodRevId);

    /**
     * init() - execute the DPS310 initialization
     * 
     * Return:
     * - ESP_OK - if a correct initialization was done
     * - ESP_FAIL - in case of initialization error
     **/
    esp_err_t init();

    /**
     * readTmp() - perform a temperature reading
     * @arg1: where to save the readed temperature
     * Return:
     * - ESP_OK - if a correct reading was done
     * - ESP_FAIL - in case of reading error
     **/
    esp_err_t readTmp(TmpMeasure *tmpMeasure);

    /**
     * readPress() - perform a pressure reading
     * @arg1: where to save the readed pressure
     * Return:
     * - ESP_OK - if a correct reading was done
     * - ESP_FAIL - in case of reading error
     **/
    esp_err_t readPress(double tmpRawScal, PressMeasure *pressMeasure);

    /**
     * execMeasurement() - perform a temperature and pressure reading
     * @arg1: where to save the readed measurement
     * Return:
     * - ESP_OK - if a correct reading was done
     * - ESP_FAIL - in case of reading error
     **/
    esp_err_t execMeasurement(Measure *measure);

    /**
     * softReset() - perform a DPS310 soft reset
     * @arg1: where to save the readed pressure
     * Return:
     * - ESP_OK - if soft reset is ok
     * - ESP_FAIL - in case of error
     **/
    esp_err_t softReset();

private:
    private:
    uint8_t i2cMasterPort;
    PressMeasurementRate pmRate;
    PressOverSamplingRate poRate;
    InternalExternalSensor internalExternalSensor;
    TmpMeasurementRate tmpmRate;
    TmpOversamplingRate tmpoRate; 
    MeasurementConfig measurementConfig;
    int32_t coeff[N_COEFF];
    
    const uint32_t scaleFactor[8] {
        SCALE_FACTOR_RATE1,
        SCALE_FACTOR_RATE2,
        SCALE_FACTOR_RATE4,
        SCALE_FACTOR_RATE8,
        SCALE_FACTOR_RATE16,
        SCALE_FACTOR_RATE32,
        SCALE_FACTOR_RATE64,
        SCALE_FACTOR_RATE128
    };

    /**
     * readCoeff() - read DPS310's coefficients from registers
     * 
     * Return:
     * - ESP_OK - if a correct reading was done
     * - ESP_FAIL - in case of error
     **/
    esp_err_t readCoeff();

    /**
     * waitCoeffAndRdy() - wait until DPS310's coefficient are ready to be read
     * 
     * Return:
     * - ESP_OK - if a correct reading was done
     * - ESP_FAIL - in case of error
     **/
    esp_err_t waitCoeffAndRdy();

    /**
     * waitTmp() - wait until temperature is ready to be read
     * 
     * Return:
     * - ESP_OK - if a correct reading was done
     * - ESP_FAIL - in case of error
     **/
    esp_err_t waitTmp();

    /**
     * waitPress() - wait until pressure is ready to be read
     * 
     * Return:
     * - ESP_OK - if a correct reading was done
     * - ESP_FAIL - in case of error
     **/
    esp_err_t waitPress();

    int32_t getTwosComplement(int32_t value, uint8_t length) {
        if (value & ((uint32_t)1 << (length - 1))) {
                value -= (uint32_t)1 << length;
            }
        return value;
    }
};

#endif