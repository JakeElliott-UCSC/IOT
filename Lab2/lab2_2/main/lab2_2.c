#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "sdkconfig.h"

// Constants
#define I2C_MASTER_SCL_IO           9        // SCL pin
#define I2C_MASTER_SDA_IO           8        // SDA pin
#define I2C_MASTER_FREQ_HZ          100000   // I2C master clock frequency
#define I2C_MASTER_NUM              I2C_NUM_0 // I2C port number
#define SHTC3_SENSOR_ADDR           0x70     // SHTC3 I2C address
#define WRITE_BIT                   I2C_MASTER_WRITE // I2C master write
#define READ_BIT                    I2C_MASTER_READ  // I2C master read
#define ACK_CHECK_EN                0x1              // I2C master will check ack from slave
#define ACK_CHECK_DIS               0x0              // I2C master will not check ack from slave
#define ACK_VAL                     0x0              // I2C ack value
#define NACK_VAL                    0x1              // I2C nack value

// Function Prototypes
static esp_err_t i2c_master_init(void);
static esp_err_t shtc3_read_temperature(float *temperature);

void app_main(void) {
    ESP_ERROR_CHECK(i2c_master_init());
    float temperature = 0;

    while (1) {
        ESP_ERROR_CHECK(shtc3_read_temperature(&temperature));
        printf("Temperature: %.2f °C\n", temperature);
        vTaskDelay(pdMS_TO_TICKS(1000)); // Wait for 1000 ms
    }
}

// Initialize the I2C master interface
static esp_err_t i2c_master_init(void) {
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

// Read temperature from SHTC3
static esp_err_t shtc3_read_temperature(float *temperature) {
    uint8_t data_h, data_l;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SHTC3_SENSOR_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0x7C, ACK_CHECK_EN); // Wakeup command
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SHTC3_SENSOR_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0xE7, ACK_CHECK_EN); // Read temperature command
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SHTC3_SENSOR_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, &data_h, ACK_VAL);
    i2c_master_read_byte(cmd, &data_l, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    
    if (ret == ESP_OK) {
        *temperature = ((data_h << 8) | data_l) * 175.0 / 65535.0 - 45;
    }
    return ret;
}
