#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/i2c.h"
// #include "icm42670.h"

// Constants
#define I2C_MASTER_SCL_IO    8    /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO    10    /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM       I2C_NUM_0 /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ   100000   /*!< I2C master clock frequency */
#define SHTC3_SENSOR_ADDR    0x70     /*!< Slave address of the SHTC3 sensor */
#define IMU_SENSOR_ADDR      0x68     /*!< Slave address of the IMU sensor */

#define GYROX1 0X11
#define GYROX0 0X12
#define GYROY1 0X13
#define GYROY0 0X14

#define POWER_MAN 0x1F
#define POWER_SET 0x0C

#define GYRO_CONFIG 0x20
#define GYRO_SET 0x6C

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
    tilt_flag = 0;
}


/**
 * @brief Send Data to gyro sensor
 */
static esp_err_t write_gyro(uint8_t reg, uint8_t data) {
    esp_err_t ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();


    // Send read command
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (IMU_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
    i2c_master_stop(cmd);

    // send to sensor
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    // end transmission
    i2c_cmd_link_delete(cmd);

    // Process data if OK
    if (ret == ESP_OK) {
    } else {
        ESP_LOGE(TAG, "Failed to send data!");
    }
    return ret;
}




/**
 * @brief Send Data to gyro sensor
 */
static esp_err_t read_gyro(int16_t *x,int16_t *y,int16_t *z) {
    uint8_t sensor_data[6];
    esp_err_t ret;


    // i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    // // Send read command
    // cmd = i2c_cmd_link_create();
    // // Read data from sensor
    // i2c_master_start(cmd);
    // i2c_master_write_byte(cmd, (SHTC3_SENSOR_ADDR << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
    // i2c_master_read(cmd, sensor_data, 6, I2C_MASTER_LAST_NACK); // Read 6 bytes data
    // i2c_master_stop(cmd);

    // // send and read from sensor
    // ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    // // end transmission
    // i2c_cmd_link_delete(cmd);

    uint8_t reg_buff[] = {GYROX1};

    ret = i2c_master_write_read_device(I2C_MASTER_WRITE, IMU_SENSOR_ADDR, reg_buff, sizeof(reg_buff), sensor_data, sizeof(sensor_data), 1000 / portTICK_PERIOD_MS);

    // Process data if OK
    if (ret == ESP_OK) {
    } else {
        ESP_LOGE(TAG, "Failed to read data!");
    }

    *x = (int16_t)((sensor_data[0] << 8) + sensor_data[1]);
    *y = (int16_t)((sensor_data[2] << 8) + sensor_data[3]);
    *z = (int16_t)((sensor_data[4] << 8) + sensor_data[5]);

    return ret;
}






void app_main(void)
{
    // Initialize I2C
    i2c_master_init();

    // Create ICM42670 sensor instance
    // icm42670_handle_t sensor = icm42670_create(I2C_MASTER_NUM, IMU_SENSOR_ADDR);
    // if (sensor == NULL) {
    //     ESP_LOGI(TAG, "Failed to initialize ICM42670");
    //     return;
    // }
    // vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second

    // Configure the ICM42670 sensor
    // icm42670_cfg_t icm_config = {
    //     .gyro_fs = GYRO_FS_250DPS,
    //     .gyro_odr = GYRO_ODR_1600HZ,
    //     .acce_fs = ACCE_FS_2G,
    //     .acce_odr = ACCE_ODR_1600HZ
    // };
    // while (icm42670_config(sensor, &icm_config) != ESP_OK){
    //     vTaskDelay(pdMS_TO_TICKS(1000));
    //     ESP_LOGE(TAG, "Failed to configure ICM42670");
    // }
    // if (icm42670_config(sensor, &icm_config) != ESP_OK) {
    //     ESP_LOGI(TAG, "Failed to configure ICM42670");
    //     return;
    // }

    // vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second

    // Set accelerometer power mode
    // while (icm42670_acce_set_pwr(sensor, ACCE_PWR_ON) != ESP_OK) {
    //     ESP_LOGE(TAG, "Failed to set accelerometer power mode");
    //     vTaskDelay(pdMS_TO_TICKS(1000));
    // }
    // if (icm42670_acce_set_pwr(sensor, ACCE_PWR_ON) != ESP_OK) {
    //     ESP_LOGI(TAG, "Failed to set accelerometer power mode");
    //     return;
    // }

    // vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second

    // Set gyroscope power mode
    // while (icm42670_gyro_set_pwr(sensor, GYRO_PWR_STANDBY) != ESP_OK) {
    //     vTaskDelay(pdMS_TO_TICKS(1000));
    //     ESP_LOGE(TAG, "Failed to set gyroscope power mode");
    // }
    // if (icm42670_gyro_set_pwr(sensor, GYRO_PWR_STANDBY) != ESP_OK) {
    //     ESP_LOGI(TAG, "Failed to set gyroscope power mode");
    //     return;
    // }

    // vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second

    // icm42670_value_t acce_value;
    // icm42670_value_t gyro;














    // config sensor
    write_gyro(POWER_MAN,POWER_SET);
    write_gyro(GYRO_CONFIG,GYRO_SET);



    int16_t gyroX = 0;
    int16_t gyroY = 0;
    int16_t gyroZ = 0;




    printf("entering while loop\n");
    while (1) {

        read_gyro(&gyroX,&gyroY,&gyroZ);
        printf("Gyro x, y, z: %d, %d, %d\n",gyroX,gyroY,gyroZ);
        // if (icm42670_get_gyro_value(sensor, &gyro) != ESP_OK ) {
        //     ESP_LOGE(TAG, "get gyro failed");
        // }
        // printf("Gyro x, y, z: %f, %f, %f\n",gyro.x,gyro.y,gyro.z);
        // tiltEvent(gyro.x,gyro.y,gyro.z);


        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second
    }
}