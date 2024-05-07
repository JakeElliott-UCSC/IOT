#include <stdio.h>
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
    //  {
    //     .mode = I2C_MODE_MASTER,
    //     .sda_io_num = I2C_MASTER_SDA_IO,
    //     .scl_io_num = I2C_MASTER_SCL_IO,
    //     .sda_pullup_en = GPIO_PULLUP_ENABLE,
    //     .scl_pullup_en = GPIO_PULLUP_ENABLE,
    //     .master.clk_speed = I2C_MASTER_FREQ_HZ,
    // };
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
}



extern "C" void app_main(void)
{
    // initialize i2c on the chip
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C Initialized Successfully");

    // initial verification of life
    printf("Hello World - Lab 3.3\n");
    vTaskDelay(pdMS_TO_TICKS(2000));

    DFRobot_LCD lcd(16,2,LCD_ADDRESS,RGB_ADDRESS);  //16 characters and 2 lines of show

    lcd.init();

    while (1) {

        lcd.setCursor(0,0);

        lcd.setRGB(0,255,0);



        printf("Hello World - Lab 3.3\n");
        vTaskDelay(pdMS_TO_TICKS(2000)); // Poll every 2 seconds
    }
}
