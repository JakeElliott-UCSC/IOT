#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_timer.h"

// Ultra Sound constants
#define TRIG_PIN GPIO_NUM_4
#define ECHO_PIN GPIO_NUM_5

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


// distance sensor initialization
static void init_hcsr04() {
    esp_rom_gpio_pad_select_gpio(TRIG_PIN);
    esp_rom_gpio_pad_select_gpio(ECHO_PIN);
    gpio_set_direction(TRIG_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(ECHO_PIN, GPIO_MODE_INPUT);
    gpio_set_level(TRIG_PIN, 0);
}

// activate distance sensor
static int64_t measure_distance(float temperature) {
    // Send trigger pulse
    gpio_set_level(TRIG_PIN, 1);
    esp_rom_delay_us(10);  // 10 microseconds delay
    gpio_set_level(TRIG_PIN, 0);

    // Wait for echo start
    while (gpio_get_level(ECHO_PIN) == 0) {
    }

    int64_t start_time = esp_timer_get_time();

    // Wait for echo end
    while (gpio_get_level(ECHO_PIN) == 1) {
    }

    int64_t end_time = esp_timer_get_time();
    int64_t duration = end_time - start_time;

    // determine speed of sound in this temperature
    float speed = (331 + (0.61*temperature))/10000;

    // Calculate distance in centimeters (previously using 0.034 for speed)
    int64_t distance = (duration * speed) / 2;

    return distance;
}






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

// checksum: the checksum byte provided by the sensor
// data: the two bytes representing sensor data
uint8_t crc8(uint8_t checksum, uint16_t data) {
    uint8_t crc = 0xFF; // Initialization value
    uint8_t polynomial = 0x31; // Polynomial 0x31 (x^8 + x^5 + x^4 + 1)
    uint8_t data_byte;

    if (data == 0x0000) {
        return 0xAC;
    }

    // Process high byte
    data_byte = (uint8_t)(data >> 8); // Extract high byte
    crc ^= data_byte; // Initial XOR with high byte

    for (int i = 0; i < 8; i++) { // Process each bit of high byte
        if (crc & 0x80) { // If the leftmost (8th) bit is set
            crc = (crc << 1) ^ polynomial; // Left shift and XOR with polynomial
        } else {
            crc <<= 1; // Just left shift
        }
    }

    // Process low byte
    data_byte = (uint8_t)(data & 0xFF); // Extract low byte
    crc ^= data_byte; // Initial XOR with low byte

    for (int i = 0; i < 8; i++) { // Process each bit of low byte
        if (crc & 0x80) { // If the leftmost (8th) bit is set
            crc = (crc << 1) ^ polynomial; // Left shift and XOR with polynomial
        } else {
            crc <<= 1; // Just left shift
        }
    }

    if (crc == checksum) {
        return 1;
    }
    else {
        return 0;
    }
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

    // do not progress if data is not OK
    if (ret == ESP_OK) {
    } else {
        ESP_LOGE(TAG, "Failed to read temperature!");
        return ret;
    }
    
    // Extract Data
    uint16_t temp_raw = (sensor_data[0] << 8) | sensor_data[1];


    // check data for accuracy
    uint8_t crc = sensor_data[2];
    if (crc8(crc,temp_raw)) {
        //ESP_LOGI(TAG, "Temperature Good Read");
    }
    else {
        ESP_LOGE(TAG, "Temperature Bad Read");
    }

    // Convert to Celsius
    *temperature = -45 + (175 * ((float)temp_raw / 65535));
    // Convert to Farenheit
    float fahrenheit = *temperature * 1.8 + 32;
    // Return temperature
    //ESP_LOGI(TAG, "Read temperature: %.2f C (%.2f F)", *temperature, fahrenheit);
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
    } else {
        ESP_LOGE(TAG, "Failed to read humidity!");
    }

    // Extract the data
    uint16_t humid_raw = (sensor_data[0] << 8) | sensor_data[1];

    // Convert the data
    *humidity = (100 * ((float)humid_raw/65535));
    //ESP_LOGI(TAG, "Read Humidity: %.2f %%", *humidity);

    // check data for accuracy
    uint8_t crc = sensor_data[2];


    if (crc8(crc,humid_raw)) {
        //ESP_LOGI(TAG, "Humidity Good Read");
    }
    else {
        ESP_LOGE(TAG, "Humidity Bad Read");
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
        //ESP_LOGE(TAG, "Sensor Awake");
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
        //ESP_LOGE(TAG, "Sensor Asleep\n");
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
    // initialize distance sensor
    init_hcsr04();
    float temperature = 0.0;
    float humidity = 0.0;
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C Initialized Successfully");
    // Trying to remove the initial error message
    // flush initial message
    flushSHTC3();
    // wake up, read temp and humidity, wait 2 seconds, shutdown
    while (1) {
        WakeupSHTC3();
        //printf("Temperature and Humidity:\n");
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

        int64_t distance = measure_distance(temperature);

        //printf("Temperature: %f\n",temperature);
        printf("Distance: %lld cm at %d C\n", distance,(int)temperature);
        vTaskDelay(pdMS_TO_TICKS(2000)); // Poll every 2 seconds
    }
}
