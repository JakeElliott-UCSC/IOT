#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/i2c.h"

// Constants
#define I2C_MASTER_SCL_IO    8    /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO    10    /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM       I2C_NUM_0 /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ   100000   /*!< I2C master clock frequency */
#define SHTC3_SENSOR_ADDR    0x70     /*!< Slave address of the SHTC3 sensor */
#define IMU_SENSOR_ADDR      0x68     /*!< Slave address of the IMU sensor */

#define ACK_CHECK_EN         0x1     /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS        0x0     /*!< I2C master will not check ack from slave */
#define ACK_VAL              0x0     /*!< I2C ack value */
#define NACK_VAL             0x1     /*!< I2C nack value */

static const char *TAG = "main";

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
}

void app_main(void)
{
    // Initialize I2C
    i2c_master_init();

    // Create ICM42670 sensor instance
    icm42670_handle_t sensor = icm42670_create(I2C_MASTER_NUM, ICM42670_ID);
    if (sensor == NULL) {
        ESP_LOGI(TAG, "Failed to initialize ICM42670");
        return;
    }

    // Configure the ICM42670 sensor
    icm42670_cfg_t icm_config = {
        .gyro_fs = GYRO_FS_250DPS,
        .gyro_odr = 0x0B,
        .acce_fs = ACCE_FS_2G,
        .acce_odr = 0x0A
    };
    if (icm42670_config(sensor, &icm_config) != ESP_OK) {
        ESP_LOGI(TAG, "Failed to configure ICM42670");
        return;
    }

    // Set accelerometer power mode
    if (icm42670_acce_set_pwr(sensor, ICM42670_ACCE_PWR_ON) != ESP_OK) {
        ESP_LOGI(TAG, "Failed to set accelerometer power mode");
        return;
    }

    // Set gyroscope power mode
    if (icm42670_gyro_set_pwr(sensor, ICM42670_GYRO_PWR_ON) != ESP_OK) {
        ESP_LOGI(TAG, "Failed to set gyroscope power mode");
        return;
    }

    icm42670_value_t acce_value;

    while (1) {
        // Read accelerometer data
        if (icm42670_get_acce_value(sensor, &acce_value) == ESP_OK) {
            // Determine the inclination
            if (acce_value.x > 0.5) {
                ESP_LOGI(TAG, "UP");
            } else if (acce_value.x < -0.5) {
                ESP_LOGI(TAG, "DOWN");
            }

            if (acce_value.y > 0.5) {
                ESP_LOGI(TAG, "RIGHT");
            } else if (acce_value.y < -0.5) {
                ESP_LOGI(TAG, "LEFT");
            }
        } else {
            ESP_LOGI(TAG, "Failed to read accelerometer data");
        }

        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second
    }
}