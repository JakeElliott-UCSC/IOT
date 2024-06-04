#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "icm42670.h"

#define I2C_MASTER_SCL_IO 22  // Set these to your I2C pins
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000

static const char *TAG = "ICM42670";

void icm42670_task(void *arg) {
    icm42670_dev_t icm42670;
    icm42670.i2c_port = I2C_MASTER_NUM;
    icm42670.i2c_addr = ICM42670_I2C_ADDRESS;
    icm42670.scl_io = I2C_MASTER_SCL_IO;
    icm42670.sda_io = I2C_MASTER_SDA_IO;
    icm42670.clk_speed = I2C_MASTER_FREQ_HZ;

    icm42670_init(&icm42670);

    while (1) {
        float ax, ay, az;
        icm42670_get_accel_data(&icm42670, &ax, &ay, &az);

        // Threshold for detecting inclination
        float threshold = 0.5;
        char direction[20] = "";

        if (az > threshold) {
            strcat(direction, "UP ");
        } else if (az < -threshold) {
            strcat(direction, "DOWN ");
        }

        if (ax > threshold) {
            strcat(direction, "LEFT");
        } else if (ax < -threshold) {
            strcat(direction, "RIGHT");
        }

        ESP_LOGI(TAG, "%s", direction);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);

    xTaskCreate(icm42670_task, "icm42670_task", 2048, NULL, 5, NULL);
}
