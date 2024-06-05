#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "icm42670.h"

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

#define TILT_THRESHOLD       500

#define UP_TOGGLE            0b1000
#define DOWN_TOGGLE          0b0100
#define LEFT_TOGGLE          0b0010
#define RIGHT_TOGGLE         0b0001

uint8_t tilt_flag = 0;







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

// LEFT  -Y
// RIGHT +Y
// UP    +X
// DOWN  -X
void tiltEvent(float x, float y, float z){
    // UP
    if (x > TILT_THRESHOLD) {
        tilt_flag = tilt_flag ^ UP_TOGGLE;
    }
    // DOWN
    else if (x < (TILT_THRESHOLD * -1)) {
        tilt_flag = tilt_flag ^ DOWN_TOGGLE;
    }
    // RIGHT
    if (y > TILT_THRESHOLD) {
        tilt_flag = tilt_flag ^ RIGHT_TOGGLE;
    }
    // LEFT
    else if (y < (TILT_THRESHOLD * -1)) {
        tilt_flag = tilt_flag ^ LEFT_TOGGLE;
    }

    switch (tilt_flag) {
        case 0b00001000:
            ESP_LOGI(TAG, "UP");
            break;
        case 0b00000100:
            ESP_LOGI(TAG, "DOWN");
            break;
        case 0b00000010:
            ESP_LOGI(TAG, "LEFT");
            break;
        case 0b00000001:
            ESP_LOGI(TAG, "RIGHT");
            break;
        case 0b00001010:
            ESP_LOGI(TAG, "UP LEFT");
            break;
        case 0b00001001:
            ESP_LOGI(TAG, "UP RIGHT");
            break;
        case 0b00000110:
            ESP_LOGI(TAG, "DOWN LEFT");
            break;
        case 0b00000101:
            ESP_LOGI(TAG, "DOWN RIGHT");
            break;
    }
    
}













void app_main(void)
{
    // Initialize I2C
    i2c_master_init();

    // Create ICM42670 sensor instance
    icm42670_handle_t sensor = icm42670_create(I2C_MASTER_NUM, IMU_SENSOR_ADDR);
    if (sensor == NULL) {
        ESP_LOGI(TAG, "Failed to initialize ICM42670");
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second

    // Configure the ICM42670 sensor
    icm42670_cfg_t icm_config = {
        .gyro_fs = GYRO_FS_250DPS,
        .gyro_odr = GYRO_ODR_1600HZ,
        .acce_fs = ACCE_FS_2G,
        .acce_odr = ACCE_ODR_1600HZ
    };
    while (icm42670_config(sensor, &icm_config) != ESP_OK){
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP_LOGE(TAG, "Failed to configure ICM42670");
    }
    // if (icm42670_config(sensor, &icm_config) != ESP_OK) {
    //     ESP_LOGI(TAG, "Failed to configure ICM42670");
    //     return;
    // }

    vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second

    // Set accelerometer power mode
    while (icm42670_acce_set_pwr(sensor, ACCE_PWR_ON) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set accelerometer power mode");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    // if (icm42670_acce_set_pwr(sensor, ACCE_PWR_ON) != ESP_OK) {
    //     ESP_LOGI(TAG, "Failed to set accelerometer power mode");
    //     return;
    // }

    vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second

    // Set gyroscope power mode
    while (icm42670_gyro_set_pwr(sensor, GYRO_PWR_STANDBY) != ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP_LOGE(TAG, "Failed to set gyroscope power mode");
    }
    // if (icm42670_gyro_set_pwr(sensor, GYRO_PWR_STANDBY) != ESP_OK) {
    //     ESP_LOGI(TAG, "Failed to set gyroscope power mode");
    //     return;
    // }

    vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second

    icm42670_value_t acce_value;
    icm42670_value_t gyro;

    printf("entering while loop\n");
    while (1) {
        

        icm42670_get_gyro_value(sensor, &gyro);
        //printf("Gyro x, y, z: %f, %f, %f\n",gyro.x,gyro.y,gyro.z);
        tiltEvent(gyro.x,gyro.y,gyro.z);


        // // Read accelerometer data
        // if (icm42670_get_acce_value(sensor, &acce_value) == ESP_OK) {
        //     // Determine the inclination
        //     if (acce_value.x > 0.5) {
        //         ESP_LOGI(TAG, "UP");
        //     } else if (acce_value.x < -0.5) {
        //         ESP_LOGI(TAG, "DOWN");
        //     }

        //     if (acce_value.y > 0.5) {
        //         ESP_LOGI(TAG, "RIGHT");
        //     } else if (acce_value.y < -0.5) {
        //         ESP_LOGI(TAG, "LEFT");
        //     }
        // } else {
        //     ESP_LOGE(TAG, "Failed to read accelerometer data");
        // }

        vTaskDelay(pdMS_TO_TICKS(10)); // Delay for 1 second
    }
}