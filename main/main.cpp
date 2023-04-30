#include <stdio.h>
#include "esp_system.h"
#include "driver_dps310.h"
#include "driver_adxl345.h"
#include "driver/i2c.h"
#include "esp_log.h"
#define MASTER_CLOCK_SPEED 400000
#define SDA_PIN 7
#define SCL_PIN 8
#define I2C_MASTER_PORT 0
#define ERIS_TAG "ERIS"
#define MIN_M_PARACHUTE_OPEN 1

extern "C" {
    void app_main(void);
}

void app_main(void)
{
   vTaskDelay(1000/portTICK_PERIOD_MS);
   i2c_config_t conf;
   conf.mode = I2C_MODE_MASTER;
   conf.sda_io_num = SDA_PIN;
   conf.scl_io_num = SCL_PIN;
   conf.master.clk_speed = MASTER_CLOCK_SPEED;
   conf.clk_flags = 0;
   i2c_param_config(0, &conf);
   i2c_driver_install(0, I2C_MODE_MASTER, 0, 0, 0);

   DriverDPS310 *driverDPS310 = new DriverDPS310(
      I2C_MASTER_PORT, 
      DriverDPS310::PressMeasurementRate::PM_RATE_128, 
      DriverDPS310::PressOverSamplingRate::PO_RATE_128, 
      DriverDPS310::InternalExternalSensor::EXTERNAL_SENS,
      DriverDPS310::TmpMeasurementRate::TMPM_RATE_128, 
      DriverDPS310::TmpOversamplingRate::TMPO_RATE_128, 
      DriverDPS310::MeasurementConfig::CONT_P_TMP_MEAS
   );

   driverDPS310->init();

   DriverDPS310::Measure measure;

   driverDPS310->execMeasurement(&measure);

   const double press1m = 0.125;
   const double heightZero = (measure.pressMeasure.pressComp / 100) / press1m;
   double heightLast = 0.0;
   double heightCurr = 0.0;
   const double heightDelta = 0.05;
   const double heightParachute = 1.10;
   bool canOpenParachute = false;

   while (true) {
      DriverDPS310::Measure measure;

      driverDPS310->execMeasurement(&measure);

      heightCurr = heightZero - (measure.pressMeasure.pressComp / 100 / press1m);

      if (heightCurr >= MIN_M_PARACHUTE_OPEN) {
         canOpenParachute = true;
      }

      if (heightLast == 0.0) {
         heightLast = heightCurr;
      }

      if (heightCurr >= heightParachute) {
         ESP_LOGI(ERIS_TAG, "Height reached");
      }

      bool climb = heightCurr - heightLast >= heightDelta;
      bool drop = heightCurr - heightLast <= -heightDelta;

      if (climb) {
         ESP_LOGI(ERIS_TAG, "Climb");
      }

      if (drop) {
         ESP_LOGI(ERIS_TAG, "Drop");
      }

      ESP_LOGI(ERIS_TAG, "Height: %f m, pressure: %f", heightCurr, measure.pressMeasure.pressComp);

      if (canOpenParachute && drop && heightCurr < heightParachute) {
         ESP_LOGI(ERIS_TAG, "Parachute opening!!");
      }

      heightLast = heightCurr;


      vTaskDelay(1/portTICK_PERIOD_MS);
   }
}