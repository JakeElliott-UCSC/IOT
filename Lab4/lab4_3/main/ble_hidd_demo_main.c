/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_hidd_prf_api.h"
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "driver/gpio.h"
#include "hid_dev.h"

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



int up_count = 0;
int down_count = 0;
int left_count = 0;
int right_count = 0;



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
void tiltEvent(uint16_t id,float x, float y, float z){
    int vertical_delta = 0;
    int horizontal_delta = 0;

    // UP
    if (x > TILT_THRESHOLD) {
        up_count++;
        down_count = 0;
    }
    // DOWN
    else if (x < (TILT_THRESHOLD * -1)) {
        up_count = 0;
        down_count++;
    }
    // RIGHT
    if (y > TILT_THRESHOLD) {
        left_count = 0;
        right_count++;
    }
    // LEFT
    else if (y < (TILT_THRESHOLD * -1)) {
        left_count++;
        right_count = 0;
    }

    // up will tend positive, down will tend negative
    if (abs(vertical_delta) <= 10) {
        vertical_delta = 2*(up_count - down_count);
    }
    else if ((up_count - down_count) < 0) {
        vertical_delta = -10;
    }
    else {
        vertical_delta = 10;
    }
    
    // right will tend positive, left will tend negative
    if (abs(horizontal_delta) <= 10) {
        horizontal_delta = 2*(right_count - left_count);
    }
    else if ((right_count - left_count) < 0) {
        horizontal_delta = -10;
    }
    else {
        horizontal_delta = 10;
    }
    
    
    esp_hidd_send_mouse_value(id, 0, horizontal_delta, vertical_delta);  // Move mouse left
    vTaskDelay(pdMS_TO_TICKS(10));

    
}




/**
 * Brief:
 * This example Implemented BLE HID device profile related functions, in which the HID device
 * has 4 Reports (1 is mouse, 2 is keyboard and LED, 3 is Consumer Devices, 4 is Vendor devices).
 * Users can choose different reports according to their own application scenarios.
 * BLE HID profile inheritance and USB HID class.
 */

/**
 * Note:
 * 1. Win10 does not support vendor report , So SUPPORT_REPORT_VENDOR is always set to FALSE, it defines in hidd_le_prf_int.h
 * 2. Update connection parameters are not allowed during iPhone HID encryption, slave turns
 * off the ability to automatically update connection parameters during encryption.
 * 3. After our HID device is connected, the iPhones write 1 to the Report Characteristic Configuration Descriptor,
 * even if the HID encryption is not completed. This should actually be written 1 after the HID encryption is completed.
 * we modify the permissions of the Report Characteristic Configuration Descriptor to `ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE_ENCRYPTED`.
 * if you got `GATT_INSUF_ENCRYPTION` error, please ignore.
 */

#define HID_DEMO_TAG "HID_DEMO"


static uint16_t hid_conn_id = 0;
static bool sec_conn = false;
static bool move_mouse_left = false;
#define CHAR_DECLARATION_SIZE   (sizeof(uint8_t))

static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param);

#define HIDD_DEVICE_NAME            "HID"
static uint8_t hidd_service_uuid128[] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x12, 0x18, 0x00, 0x00,
};

static esp_ble_adv_data_t hidd_adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
    .appearance = 0x03c0,       //HID Generic,
    .manufacturer_len = 0,
    .p_manufacturer_data =  NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(hidd_service_uuid128),
    .p_service_uuid = hidd_service_uuid128,
    .flag = 0x6,
};

static esp_ble_adv_params_t hidd_adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x30,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    //.peer_addr            =
    //.peer_addr_type       =
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};


static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param)
{
    switch(event) {
        case ESP_HIDD_EVENT_REG_FINISH: {
            if (param->init_finish.state == ESP_HIDD_INIT_OK) {
                //esp_bd_addr_t rand_addr = {0x04,0x11,0x11,0x11,0x11,0x05};
                esp_ble_gap_set_device_name(HIDD_DEVICE_NAME);
                esp_ble_gap_config_adv_data(&hidd_adv_data);

            }
            break;
        }
        case ESP_BAT_EVENT_REG: {
            break;
        }
        case ESP_HIDD_EVENT_DEINIT_FINISH:
	     break;
		case ESP_HIDD_EVENT_BLE_CONNECT: {
            ESP_LOGI(HID_DEMO_TAG, "ESP_HIDD_EVENT_BLE_CONNECT");
            hid_conn_id = param->connect.conn_id;
            break;
        }
        case ESP_HIDD_EVENT_BLE_DISCONNECT: {
            sec_conn = false;
            ESP_LOGI(HID_DEMO_TAG, "ESP_HIDD_EVENT_BLE_DISCONNECT");
            esp_ble_gap_start_advertising(&hidd_adv_params);
            break;
        }
        case ESP_HIDD_EVENT_BLE_VENDOR_REPORT_WRITE_EVT: {
            ESP_LOGI(HID_DEMO_TAG, "%s, ESP_HIDD_EVENT_BLE_VENDOR_REPORT_WRITE_EVT", __func__);
            ESP_LOG_BUFFER_HEX(HID_DEMO_TAG, param->vendor_write.data, param->vendor_write.length);
            break;
        }
        case ESP_HIDD_EVENT_BLE_LED_REPORT_WRITE_EVT: {
            ESP_LOGI(HID_DEMO_TAG, "ESP_HIDD_EVENT_BLE_LED_REPORT_WRITE_EVT");
            ESP_LOG_BUFFER_HEX(HID_DEMO_TAG, param->led_write.data, param->led_write.length);
            break;
        }
        default:
            break;
    }
    return;
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&hidd_adv_params);
        break;
     case ESP_GAP_BLE_SEC_REQ_EVT:
        for(int i = 0; i < ESP_BD_ADDR_LEN; i++) {
             ESP_LOGD(HID_DEMO_TAG, "%x:",param->ble_security.ble_req.bd_addr[i]);
        }
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
	 break;
     case ESP_GAP_BLE_AUTH_CMPL_EVT:
        sec_conn = true;
        esp_bd_addr_t bd_addr;
        memcpy(bd_addr, param->ble_security.auth_cmpl.bd_addr, sizeof(esp_bd_addr_t));
        ESP_LOGI(HID_DEMO_TAG, "remote BD_ADDR: %08x%04x",\
                (bd_addr[0] << 24) + (bd_addr[1] << 16) + (bd_addr[2] << 8) + bd_addr[3],
                (bd_addr[4] << 8) + bd_addr[5]);
        ESP_LOGI(HID_DEMO_TAG, "address type = %d", param->ble_security.auth_cmpl.addr_type);
        ESP_LOGI(HID_DEMO_TAG, "pair status = %s",param->ble_security.auth_cmpl.success ? "success" : "fail");
        if(!param->ble_security.auth_cmpl.success) {
            ESP_LOGE(HID_DEMO_TAG, "fail reason = 0x%x",param->ble_security.auth_cmpl.fail_reason);
        }
        break;
    default:
        break;
    }
}

void hid_demo_task(void *pvParameters)
{
    icm42670_value_t gyro;
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
    vTaskDelay(1000 / portTICK_PERIOD_MS);
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
    while(1) {
        
        icm42670_get_gyro_value(sensor, &gyro);
        tiltEvent(hid_conn_id,gyro.x,gyro.y,gyro.z);

        // vTaskDelay(2000 / portTICK_PERIOD_MS);
        // if (sec_conn) {
        //     ESP_LOGI(HID_DEMO_TAG, "Send the volume");
        //     move_mouse_left = true;
        //     //uint8_t key_vaule = {HID_KEY_A};
        //     //esp_hidd_send_keyboard_value(hid_conn_id, 0, &key_vaule, 1);
        //     //esp_hidd_send_consumer_value(hid_conn_id, HID_CONSUMER_VOLUME_UP, true);
        //     // esp_hidd_send_mouse_value(hid_conn_id,0);
        //     for (int i = 0; i < 100; i++) {
        //         esp_hidd_send_mouse_value(hid_conn_id, 0, -10, 0);  // Move mouse left
        //         vTaskDelay(pdMS_TO_TICKS(10));
        //     }
        //     vTaskDelay(pdMS_TO_TICKS(1000));
        //     if (move_mouse_left) {
        //         move_mouse_left = false;
        //         esp_hidd_send_mouse_value(hid_conn_id, 0, 0, 0);  // Stop mouse
        //         for (int i = 0; i < 100; i++) {
        //             esp_hidd_send_mouse_value(hid_conn_id, 0, 10, 0);  // Move mouse right
        //             vTaskDelay(pdMS_TO_TICKS(10));
        //         }
                
        //         //esp_hidd_send_consumer_value(hid_conn_id, HID_CONSUMER_VOLUME_UP, false);
        //         //esp_hidd_send_consumer_value(hid_conn_id, HID_CONSUMER_VOLUME_DOWN, true);
        //         vTaskDelay(pdMS_TO_TICKS(1000));
        //         //esp_hidd_send_consumer_value(hid_conn_id, HID_CONSUMER_VOLUME_DOWN, false);
        //         esp_hidd_send_mouse_value(hid_conn_id, 0, 0, 0);  // Stop mouse
        //     }
        // }
    }
}


void app_main(void)
{
    // Initialize I2C
    i2c_master_init();



    esp_err_t ret;

    // Initialize NVS.
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(HID_DEMO_TAG, "%s initialize controller failed", __func__);
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(HID_DEMO_TAG, "%s enable controller failed", __func__);
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(HID_DEMO_TAG, "%s init bluedroid failed", __func__);
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(HID_DEMO_TAG, "%s init bluedroid failed", __func__);
        return;
    }

    if((ret = esp_hidd_profile_init()) != ESP_OK) {
        ESP_LOGE(HID_DEMO_TAG, "%s init bluedroid failed", __func__);
    }

    ///register the callback function to the gap module
    esp_ble_gap_register_callback(gap_event_handler);
    esp_hidd_register_callbacks(hidd_event_callback);

    /* set the security iocap & auth_req & key size & init key response key parameters to the stack*/
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_BOND;     //bonding with peer device after authentication
    esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;           //set the IO capability to No output No input
    uint8_t key_size = 16;      //the key size should be 7~16 bytes
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
    /* If your BLE device act as a Slave, the init_key means you hope which types of key of the master should distribute to you,
    and the response key means which key you can distribute to the Master;
    If your BLE device act as a master, the response key means you hope which types of key of the slave should distribute to you,
    and the init key means which key you can distribute to the slave. */
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));

    xTaskCreate(&hid_demo_task, "hid_task", 2048, NULL, 5, NULL);
}
