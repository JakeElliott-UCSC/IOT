#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "DFRobot_LCD.h"





/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    i2c_port_t i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
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
        ESP_LOGI(TAG, "Temperature Good Read");
    }
    else {
        ESP_LOGE(TAG, "Temperature Bad Read");
    }

    // Convert to Celsius
    *temperature = -45 + (175 * ((float)temp_raw / 65535));
    // Convert to Farenheit
    float fahrenheit = *temperature * 1.8 + 32;
    // Return temperature
    ESP_LOGI(TAG, "Read temperature: %.2f C (%.2f F)", *temperature, fahrenheit);
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
    ESP_LOGI(TAG, "Read Humidity: %.2f %%", *humidity);

    // check data for accuracy
    uint8_t crc = sensor_data[2];


    if (crc8(crc,humid_raw)) {
        ESP_LOGI(TAG, "Humidity Good Read");
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







// data variables
float temperature = 0.0;
float humidity = 0.0;


extern "C" void app_main(void)
{
    // initialize i2c on the chip
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C Initialized Successfully");

    // flush initial message from temp sensor
    flushSHTC3();


    // initial verification of life
    printf("Hello World - Lab 3.3\n");
    vTaskDelay(pdMS_TO_TICKS(2000));

    DFRobot_LCD lcd(16,2,LCD_ADDRESS,RGB_ADDRESS);  //16 characters and 2 lines of show

    lcd.init();

    while (1) {
        // begin writing at the top of lcd
        lcd.setCursor(0,0);
        // set color
        lcd.setRGB(0,255,0);

        WakeupSHTC3();
        printf("Temperature and Humidity:\n");
        if (read_temperature(&temperature) == ESP_OK) {
            //printf("Temperature: %.2f°C\n", temperature);
            //printf("Temperature and Humidity:\n");
            lcd.printstr("Temp: ");
            lcd.write(to_string((int)temperature));
            lcd.printstr("C");
        } else {
            printf("Failed to read temperature!\n");
        }

        // write humidity below temperature
        lcd.setCursor(0,1);

        if (read_humidity(&humidity) == ESP_OK) {
            //printf("Temperature: %.2f°C\n", temperature);
            //printf("Temperature and Humidity:\n");
            lcd.printstr("Hum : ");
            lcd.write(to_string((int)humidity));
            lcd.printstr("%");
        } else {
            printf("Failed to read humidity!\n");
        }
        ShutdownSHTC3();
        vTaskDelay(pdMS_TO_TICKS(2000)); // Poll every 2 seconds

        

        lcd.printstr("Hello CSE121!");

        
        
        lcd.printstr("Elliott");

        printf("Hello World - Lab 3.3\n");
        vTaskDelay(pdMS_TO_TICKS(2000)); // Poll every 2 seconds
    }
}
