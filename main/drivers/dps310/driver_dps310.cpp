#include "driver/i2c.h"
#include "driver_dps310.h"

DriverDPS310::DriverDPS310(uint8_t i2cMasterPort, 
                PressMeasurementRate pmRate, 
                PressOverSamplingRate poRate, 
                InternalExternalSensor internalExternalSensor,
                TmpMeasurementRate tmpmRate, 
                TmpOversamplingRate tmpoRate, 
                MeasurementConfig measurementConfig): i2cMasterPort(i2cMasterPort) {
                    this->pmRate = pmRate;
                    this->poRate = poRate;
                    this->internalExternalSensor = internalExternalSensor;
                    this->tmpmRate = tmpmRate;
                    this->tmpoRate = tmpoRate;
                    this->measurementConfig = measurementConfig;
}

esp_err_t DriverDPS310::getProdRevId(DriverDPS310::ProdRevId *prodRevId)
{
    uint8_t productID;
    uint8_t productIDReg = PRODUCT_ID_REG;

    ESP_RETURN_ON_ERROR(i2c_master_write_read_device(this->i2cMasterPort, DPS310_SLAVE_ADDR, &productIDReg, sizeof(productIDReg), &productID, sizeof(productID), 1000 / portTICK_RATE_MS),
                        DPS310_ERROR_TAG,
                        DPS310_I2C_ERR
    );
    
    *prodRevId = {
        .revID = (uint8_t) ((productID & REV_ID) >> 4),
        .prodID = (uint8_t) (productID & PROD_ID)
    };

    return ESP_OK;
}

esp_err_t DriverDPS310::init()
{
    DriverDPS310::ProdRevId prodRevId;
    esp_err_t err = this->getProdRevId(&prodRevId);

    if (err == ESP_FAIL || prodRevId.prodID != PRODUCT_ID_VAL || prodRevId.revID != REV_ID_VAL) {
        return ESP_FAIL;
    }

    ESP_RETURN_ON_ERROR(this->waitCoeffAndRdy(), DPS310_ERROR_TAG, DPS310_CFG_ERR_MSG);
    ESP_RETURN_ON_ERROR(this->readCoeff(), DPS310_ERROR_TAG, DPS310_CFG_ERR_MSG);
    
    ESP_RETURN_ON_ERROR(i2c_write_byte(this->i2cMasterPort, DPS310_SLAVE_ADDR, PRS_CFG_REG, this->pmRate | this->poRate), DPS310_ERROR_TAG, DPS310_CFG_ERR_MSG);
    ESP_RETURN_ON_ERROR(i2c_write_byte(this->i2cMasterPort, DPS310_SLAVE_ADDR, TMP_CFG_REG, this->internalExternalSensor | this->tmpmRate | this->tmpoRate), DPS310_ERROR_TAG, DPS310_CFG_ERR_MSG);

    uint8_t configRegister = 0;//fifo disabled

    if (this->poRate >= PO_RATE_16) {
        configRegister |= DriverDPS310::ConfigRegister::P_SHIFT;
    }

    if (this->tmpoRate >= TMPO_RATE_16) {
        configRegister |= DriverDPS310::ConfigRegister::T_SHIFT;
    }

    ESP_RETURN_ON_ERROR(i2c_write_byte(this->i2cMasterPort, DPS310_SLAVE_ADDR, CFG_REG, configRegister), DPS310_ERROR_TAG, DPS310_CFG_ERR_MSG);
    ESP_RETURN_ON_ERROR(i2c_write_byte(this->i2cMasterPort, DPS310_SLAVE_ADDR, MEAS_CFG_REG, measurementConfig), DPS310_ERROR_TAG, DPS310_CFG_ERR_MSG);

    return ESP_OK;
}


esp_err_t DriverDPS310::readCoeff()
{
    uint8_t coeffRegVal[COEFF_REG_NUM_REG];


    for (uint8_t i = COEFF_FROM_REG; i <= COEFF_TO_REG; i++) {
        ESP_RETURN_ON_ERROR(i2c_master_write_read_device(this->i2cMasterPort, DPS310_SLAVE_ADDR, &i, sizeof(uint8_t), &coeffRegVal[i - COEFF_FROM_REG], sizeof(uint8_t), 1000 / portTICK_RATE_MS),
                            DPS310_ERROR_TAG,
                            DPS310_I2C_ERR
        );
    }

    coeff[C0_INDEX] = this->getTwosComplement((coeffRegVal[C0_0_VAL_INDEX] << 4) | (coeffRegVal[C0_1_C1_0_REG_VAL_INDEX] >> 4), C0_BIT_LEN);
    coeff[C1_INDEX] = this->getTwosComplement(((coeffRegVal[C0_1_C1_0_REG_VAL_INDEX] & 0x0f) << 8) | coeffRegVal[C1_1_VAL_INDEX], C1_BIT_LEN);
    coeff[C00_INDEX] = this->getTwosComplement((coeffRegVal[C00_0_VAL_INDEX] << 12) | ((coeffRegVal[C00_1_VAL_INDEX] << 4) | coeffRegVal[C00_2_C10_0_VAL_INDEX] >> 4), C00_BIT_LEN);
    coeff[C10_INDEX] = this->getTwosComplement(((coeffRegVal[C00_2_C10_0_VAL_INDEX] & 0x0f) << 16) | ((coeffRegVal[C10_1_VAL_INDEX] << 8) | coeffRegVal[C10_2_VAL_INDEX]), C10_BIT_LEN);
    coeff[C01_INDEX] = this->getTwosComplement((coeffRegVal[C01_0_VAL_INDEX] << 8) | coeffRegVal[C01_1_VAL_INDEX], C01_BIT_LEN);
    coeff[C11_INDEX] = this->getTwosComplement((coeffRegVal[C11_0_VAL_INDEX] << 8) | coeffRegVal[C11_1_VAL_INDEX], C11_BIT_LEN);
    coeff[C20_INDEX] = this->getTwosComplement((coeffRegVal[C20_0_VAL_INDEX] << 8) | coeffRegVal[C20_1_VAL_INDEX], C20_BIT_LEN);
    coeff[C21_INDEX] = this->getTwosComplement((coeffRegVal[C21_0_VAL_INDEX] << 8) | coeffRegVal[C21_1_VAL_INDEX], C21_BIT_LEN);
    coeff[C30_INDEX] = this->getTwosComplement((coeffRegVal[C30_0_VAL_INDEX] << 8) | coeffRegVal[C30_1_VAL_INDEX], C30_BIT_LEN);

    return ESP_OK;
}

esp_err_t DriverDPS310::readTmp(DriverDPS310::TmpMeasure *tmpMeasure)
{
    if (this->measurementConfig != MeasurementConfig::TMP_MEAS 
        && this->measurementConfig != MeasurementConfig::CONT_TMP_MEAS 
        && this->measurementConfig != MeasurementConfig::CONT_P_TMP_MEAS) {
        return ESP_FAIL;
    }

    ESP_RETURN_ON_ERROR(this->waitTmp(), DPS310_ERROR_TAG, DPS310_TMP_WAIT_TIMEOUT);

    uint8_t tmpB2;
    uint8_t tmpB1;
    uint8_t tmpB0;
    uint8_t tmpB2Reg = TMP_B2_REG;
    uint8_t tmpB1Reg = TMP_B1_REG;
    uint8_t tmpB0Reg = TMP_B0_REG;

    ESP_RETURN_ON_ERROR(i2c_master_write_read_device(this->i2cMasterPort, DPS310_SLAVE_ADDR, &tmpB2Reg, sizeof(tmpB2Reg), &tmpB2, sizeof(tmpB2), 1000 / portTICK_RATE_MS),
                        DPS310_ERROR_TAG,
                        DPS310_I2C_ERR
    );

    ESP_RETURN_ON_ERROR(i2c_master_write_read_device(this->i2cMasterPort, DPS310_SLAVE_ADDR, &tmpB1Reg, sizeof(tmpB1Reg), &tmpB1, sizeof(tmpB1), 1000 / portTICK_RATE_MS),
                        DPS310_ERROR_TAG,
                        DPS310_I2C_ERR
    );

    ESP_RETURN_ON_ERROR(i2c_master_write_read_device(this->i2cMasterPort, DPS310_SLAVE_ADDR, &tmpB0Reg, sizeof(tmpB0Reg), &tmpB0, sizeof(tmpB0), 1000 / portTICK_RATE_MS),
                        DPS310_ERROR_TAG,
                        DPS310_I2C_ERR
    );

    double tmpRawScal = (double) this->getTwosComplement((int32_t)tmpB2 << 16 | (int32_t)tmpB1 << 8 | (int32_t) tmpB0, MEASUREMENT_BIT_LEN) / scaleFactor[this->tmpoRate];

    *tmpMeasure = {
        .tmpRawScal = tmpRawScal,
        .tmpComp = coeff[C0_INDEX] * 0.5 + coeff[C1_INDEX] * tmpRawScal
    };

    return ESP_OK;
}

esp_err_t DriverDPS310::readPress(double tmpRawScal, PressMeasure *pressMeasure)
{
    if (this->measurementConfig != MeasurementConfig::P_MEAS 
        && this->measurementConfig != MeasurementConfig::CONT_P_MEAS 
        && this->measurementConfig != MeasurementConfig::CONT_P_TMP_MEAS) {
        return ESP_FAIL;
    }

    ESP_RETURN_ON_ERROR(this->waitPress(), DPS310_ERROR_TAG, DPS310_PRESS_WAIT_TIMEOUT);

    uint8_t psrB2;
    uint8_t psrB1;
    uint8_t psrB0;
    uint8_t psrB2Reg = PSR_B2_REG;
    uint8_t psrB1Reg = PSR_B1_REG;
    uint8_t psrB0Reg = PSR_B0_REG;

    ESP_RETURN_ON_ERROR(i2c_master_write_read_device(this->i2cMasterPort, DPS310_SLAVE_ADDR, &psrB2Reg, sizeof(psrB2Reg), &psrB2, sizeof(psrB2), 1000 / portTICK_RATE_MS),
                        DPS310_ERROR_TAG,
                        DPS310_I2C_ERR
    );

    ESP_RETURN_ON_ERROR(i2c_master_write_read_device(this->i2cMasterPort, DPS310_SLAVE_ADDR, &psrB1Reg, sizeof(psrB1Reg), &psrB1, sizeof(psrB1), 1000 / portTICK_RATE_MS),
                        DPS310_ERROR_TAG,
                        DPS310_I2C_ERR
    );

    ESP_RETURN_ON_ERROR(i2c_master_write_read_device(this->i2cMasterPort, DPS310_SLAVE_ADDR, &psrB0Reg, sizeof(psrB0Reg), &psrB0, sizeof(psrB0), 1000 / portTICK_RATE_MS),
                        DPS310_ERROR_TAG,
                        DPS310_I2C_ERR
    );

    double pressRawScal = (double) this->getTwosComplement((int32_t)psrB2 << 16 | (int32_t)psrB1 << 8 | (int32_t)psrB0, MEASUREMENT_BIT_LEN) / scaleFactor[this->poRate];
    
    double pressComp = coeff[C00_INDEX] + pressRawScal * (coeff[C10_INDEX] + pressRawScal * (coeff[C20_INDEX] + pressRawScal * coeff[C30_INDEX])) + tmpRawScal * coeff[C01_INDEX] + tmpRawScal * pressRawScal * (coeff[C11_INDEX] + pressRawScal * coeff[C21_INDEX]);
    
    *pressMeasure = {
        .pressRawScal = pressRawScal,
        .pressComp = pressComp
    };

    return ESP_OK;
}

esp_err_t DriverDPS310::execMeasurement(DriverDPS310::Measure *measure)
{
    if (this->measurementConfig != CONT_P_TMP_MEAS) {
        return ESP_FAIL;
    }

    TmpMeasure tmpMeasure;
    PressMeasure pressMeasure;
    ESP_RETURN_ON_ERROR(this->readTmp(&tmpMeasure), DPS310_ERROR_TAG, DPS310_MEASUREMENT_ERR);
    ESP_RETURN_ON_ERROR(this->readPress(tmpMeasure.tmpRawScal, &pressMeasure), DPS310_ERROR_TAG, DPS310_MEASUREMENT_ERR);

    *measure = {
        .tmpMeasure = tmpMeasure,
        .pressMeasure = pressMeasure
    };

    return ESP_OK;
}

esp_err_t DriverDPS310::waitCoeffAndRdy()
{
    uint8_t coeffRdy = 0;
    uint8_t coeffRegVal;
    uint8_t measCfgReg = MEAS_CFG_REG;

    for (int i = 0;i < N_RTRY_DATA_RDY;i++) {
        ESP_RETURN_ON_ERROR(i2c_master_write_read_device(this->i2cMasterPort, DPS310_SLAVE_ADDR, &measCfgReg, sizeof(measCfgReg), &coeffRegVal, sizeof(coeffRegVal), 1000 / portTICK_RATE_MS),
                            DPS310_ERROR_TAG,
                            DPS310_I2C_ERR
        );
        
        coeffRdy = coeffRegVal >> 6;

        if (coeffRdy == COEFF_RDY_VAL) { //bit dei coefficienti e rdy alti
            return ESP_OK;
        } else {
            vTaskDelay(10/portTICK_PERIOD_MS);
        }
    }

    return ESP_FAIL;
}

esp_err_t DriverDPS310::waitTmp()
{
    uint8_t tmpRdy = 0;
    uint8_t coeffRegVal;
    uint8_t measCfgReg = MEAS_CFG_REG;
    
    for (int i = 0;i < N_RTRY_DATA_RDY;i++) {
        ESP_RETURN_ON_ERROR(i2c_master_write_read_device(this->i2cMasterPort, DPS310_SLAVE_ADDR, &measCfgReg, sizeof(measCfgReg), &coeffRegVal, sizeof(coeffRegVal), 1000 / portTICK_RATE_MS),
                            DPS310_ERROR_TAG,
                            DPS310_I2C_ERR
        );

        tmpRdy = coeffRegVal >> 4;

        if ((tmpRdy & TMP_RDY_MASK) > 0) { //bit dei coefficienti e rdy alti
            return ESP_OK;
        } else {
            vTaskDelay(10/portTICK_PERIOD_MS);
        }
    }

    return ESP_FAIL;
}

esp_err_t DriverDPS310::waitPress()
{
    uint8_t pressRdy = 0;
    uint8_t coeffRegVal;
    uint8_t measCfgReg = MEAS_CFG_REG;
    
    for (int i = 0;i < N_RTRY_DATA_RDY;i++) {
        ESP_RETURN_ON_ERROR(i2c_master_write_read_device(this->i2cMasterPort, DPS310_SLAVE_ADDR, &measCfgReg, sizeof(measCfgReg), &coeffRegVal, sizeof(coeffRegVal), 1000 / portTICK_RATE_MS),
                            DPS310_ERROR_TAG,
                            DPS310_I2C_ERR
        );

        pressRdy = coeffRegVal >> 4;
        
        if ((pressRdy & PRS_RDY_MASK) > 0) { //bit dei coefficienti e rdy alti
            return ESP_OK;
        } else {
            vTaskDelay(10/portTICK_PERIOD_MS);
        }
    }

    return ESP_FAIL;
}

esp_err_t DriverDPS310::softReset()
{
    ESP_RETURN_ON_ERROR(i2c_write_byte(this->i2cMasterPort, DPS310_SLAVE_ADDR, RESET_REG, SOFT_RESET_REGISTER_VAL), DPS310_ERROR_TAG, DPS310_RESET_ERR_MSG);
    vTaskDelay(500/portTICK_PERIOD_MS);
    
    return ESP_OK;
}

