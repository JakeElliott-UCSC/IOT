#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "icm42670.h"

#define TAG "ICM42670"

// I2C configuration
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_FREQ_HZ 100000

void i2c_master_init()
{
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &i2c_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C param config failed");
    }

    err = i2c_driver_install(I2C_MASTER_NUM, i2c_conf.mode, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed");
    }
}

void app_main(void)
{
    float accel_x, accel_y, accel_z;

    // Initialize I2C
    i2c_master_init();

    // Initialize the ICM42670
    // if (icm42670_init() != ESP_OK) {
    //     ESP_LOGI(TAG, "Failed to initialize ICM42670");
    //     return;
    // }

    // while (1) {
    //     // Read accelerometer data
    //     if (icm42670_read_accel(&accel_x, &accel_y, &accel_z) == ESP_OK) {
    //         // Determine the inclination
    //         if (accel_x > 0.5) {
    //             ESP_LOGI(TAG, "UP");
    //         } else if (accel_x < -0.5) {
    //             ESP_LOGI(TAG, "DOWN");
    //         }

    //         if (accel_y > 0.5) {
    //             ESP_LOGI(TAG, "RIGHT");
    //         } else if (accel_y < -0.5) {
    //             ESP_LOGI(TAG, "LEFT");
    //         }
    //     } else {
    //         ESP_LOGI(TAG, "Failed to read accelerometer data");
    //     }

    //     vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second
    // }
}
