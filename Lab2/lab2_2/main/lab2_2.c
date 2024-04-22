#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"

#define I2C_MASTER_SCL_IO           8                /*!< GPIO number for I2C master clock */
#define I2C_MASTER_SDA_IO           10               /*!< GPIO number for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_0        /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ          100000           /*!< I2C master clock frequency */
#define SHTC3_SENSOR_ADDR           0x70             /*!< I2C address for SHTC3 sensor */

static const char *TAG = "SHTC3";

void i2c_master_init() {
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

esp_err_t i2c_master_read_byte(i2c_port_t i2c_num, uint8_t addr, uint8_t* data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, data, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

void shtc3_read_temperature() {
    uint8_t temp_h, temp_l;
    float temperature;
    
    // Read temperature from SHTC3
    if (i2c_master_read_byte(I2C_MASTER_NUM, SHTC3_SENSOR_ADDR, &temp_h) == ESP_OK &&
        i2c_master_read_byte(I2C_MASTER_NUM, SHTC3_SENSOR_ADDR + 1, &temp_l) == ESP_OK) {
        temperature = ((temp_h << 8) | temp_l) * 175.0 / 65535.0 - 45.0;
        ESP_LOGI(TAG, "Temperature: %.2f °C", temperature);
    } else {
        ESP_LOGE(TAG, "Failed to read temperature from SHTC3");
    }
}

void app_main() {
    ESP_LOGI(TAG, "Initializing I2C Master...");
    i2c_master_init();

    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        shtc3_read_temperature();
    }
}
