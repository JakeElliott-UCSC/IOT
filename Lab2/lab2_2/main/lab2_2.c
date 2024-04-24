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

/**
 * @brief Read temperature value from SHTC3 sensor
 */
static esp_err_t read_temperature(float *temperature)
{
    uint8_t sensor_data[3];
    esp_err_t ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SHTC3_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0x7C, ACK_CHECK_EN); // Temperature read command
    i2c_master_write_byte(cmd, 0xA2, ACK_CHECK_EN);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SHTC3_SENSOR_ADDR << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
    i2c_master_read(cmd, sensor_data, 6, I2C_MASTER_LAST_NACK); // Read 6 bytes data
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK) {
        // Convert the data
        uint16_t temp_raw = (sensor_data[0] << 8) | sensor_data[1];
        //uint16_t humid_raw = (sensor_data[3] << 8) | sensor_data[4];
        *temperature = -45 + (175 * ((float)temp_raw / 65535));
        float fahrenheit = *temperature * 1.8 + 32;
        // this value is not returned by the function
        //float humidity = (100 * ((float)humid_raw/65535));
        ESP_LOGI(TAG, "Read temperature: %.2f C (%.2f F)", *temperature, fahrenheit);
        //ESP_LOGI(TAG, "Read Humidity: %.2f %%", humidity);
    } else {
        ESP_LOGE(TAG, "Failed to read temperature!");
    }
    return ret;
}

/**
 * @brief Read humidity value from SHTC3 sensor
 */
static esp_err_t read_humidity(float *humidity)
{
    uint8_t sensor_data[3];
    esp_err_t ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    // Send read command
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SHTC3_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0x5C, ACK_CHECK_EN); // Humidity read command
    i2c_master_write_byte(cmd, 0x24, ACK_CHECK_EN);
    // Read data from sensor
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SHTC3_SENSOR_ADDR << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
    i2c_master_read(cmd, sensor_data, 6, I2C_MASTER_LAST_NACK); // Read 6 bytes data
    i2c_master_stop(cmd);

    // send and read from sensor
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    // end transmission
    i2c_cmd_link_delete(cmd);

    // Process data if OK
    if (ret == ESP_OK) {
        // Convert the data
        //uint16_t temp_raw = (sensor_data[0] << 8) | sensor_data[1];
        uint16_t humid_raw = (sensor_data[0] << 8) | sensor_data[1];
        //*temperature = -45 + (175 * ((float)temp_raw / 65535));
        // this value is not returned by the function
        *humidity = (100 * ((float)humid_raw/65535));
        //ESP_LOGI(TAG, "Read temperature: %.2f C", *temperature);
        ESP_LOGI(TAG, "Read Humidity: %.2f %%", *humidity);
    } else {
        ESP_LOGE(TAG, "Failed to read humidity!");
    }

    return ret;
}

static void WakeupSHTC3(){

    esp_err_t ret;

    // Wake up the sensor
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SHTC3_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0x35, ACK_CHECK_EN); // Wakeup command
    i2c_master_write_byte(cmd, 0x17, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    // Send and recieve with sensor
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    // End Transmission
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Sensor wake-up command failed!");
    }
    else {
        ESP_LOGE(TAG, "Sensor Awake");
    }

    // Wait for sensor to wake up
    vTaskDelay(pdMS_TO_TICKS(10)); // Delay for sensor wakeup

    return;
}

static void ShutdownSHTC3() {

    esp_err_t ret;

    // Wake up the sensor
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SHTC3_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0xB0, ACK_CHECK_EN); // Sleep command
    i2c_master_write_byte(cmd, 0x98, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    // Send and recieve with sensor
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    // End Transmission
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Sensor sleep command failed!");
    }
    else {
        ESP_LOGE(TAG, "Sensor Asleep\n");
    }

    // Wait for sensor to finish sleeping
    vTaskDelay(pdMS_TO_TICKS(10)); // Delay for sensor sleep
    return;
}

static void flushSHTC3() {
    uint8_t sensor_data[3];
    esp_err_t ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();


    i2c_master_start(cmd);
    i2c_master_read(cmd, sensor_data, 6, I2C_MASTER_LAST_NACK); // Read 6 bytes data
    i2c_master_stop(cmd);


    // Send and recieve with sensor
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    // End Transmission
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        ESP_LOGI(TAG, "Flush failed successfully!!");
    }

    // Wait for sensor to finish sleeping
    vTaskDelay(pdMS_TO_TICKS(10)); // Delay for sensor sleep
    return;
}

void app_main(void)
{
    float temperature = 0.0;
    float humidity = 0.0;
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C Initialized Successfully");
    // Trying to remove the initial error message
    // shutdown, wait, wake up, flush initial message, shutdown, wait
    ShutdownSHTC3();
    vTaskDelay(pdMS_TO_TICKS(1000));
    WakeupSHTC3();
    flushSHTC3();
    ShutdownSHTC3();
    vTaskDelay(pdMS_TO_TICKS(1000));

    // wake up, read temp and humidity, wait 2 seconds, shutdown
    while (1) {
        WakeupSHTC3();
        printf("Temperature and Humidity:\n");
        if (read_temperature(&temperature) == ESP_OK) {
            //printf("Temperature: %.2f°C\n", temperature);
            //printf("Temperature and Humidity:\n");
        } else {
            printf("Failed to read temperature!\n");
        }

        if (read_humidity(&humidity) == ESP_OK) {
            //printf("Temperature: %.2f°C\n", temperature);
            //printf("Temperature and Humidity:\n");
        } else {
            printf("Failed to read humidity!\n");
        }
        ShutdownSHTC3();
        vTaskDelay(pdMS_TO_TICKS(2000)); // Poll every 2 seconds
    }
}
