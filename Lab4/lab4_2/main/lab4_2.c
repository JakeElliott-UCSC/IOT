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

#define TAG "BT_MOUSE"

static esp_hidd_dev_t *hidd_dev = NULL;

static void bt_hid_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param);

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(TAG, "%s init bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(TAG, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    esp_hidd_profile_t profile = {
        .device_name = "ESP32C3_MOUSE",
        .vendor_id = 0x1234,
        .product_id = 0x5678,
        .version = 0x0100,
        .manufacturer_name = "Espressif",
    };

    ret = esp_hidd_dev_register_callbacks(bt_hid_callback);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "HID device callback register failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_hidd_dev_init(&profile, &hidd_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "HID device init failed: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "Bluetooth HID Mouse initialized and ready to connect");
}

static void bt_hid_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param) {
    switch (event) {
        case ESP_HIDD_EVENT_BATTERY_EVT:
            ESP_LOGI(TAG, "Battery event");
            break;
        case ESP_HIDD_EVENT_REG_FINISH:
            if (param->init_finish.state == ESP_HIDD_INIT_OK) {
                esp_bd_addr_t rand_addr = {0x1a, 0x2b, 0x3c, 0x4d, 0x5e, 0x6f};
                esp_bt_gap_set_scan_mode(ESP_BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE);
                esp_hidd_dev_connect(hidd_dev, rand_addr);
                ESP_LOGI(TAG, "HID device registered and connected");
            } else {
                ESP_LOGE(TAG, "HID device registration failed");
            }
            break;
        case ESP_HIDD_EVENT_DEINIT_FINISH:
            ESP_LOGI(TAG, "HID device de-initialized");
            break;
        case ESP_HIDD_EVENT_BLE_CONNECT:
            ESP_LOGI(TAG, "HID device connected");
            break;
        case ESP_HIDD_EVENT_BLE_DISCONNECT:
            ESP_LOGI(TAG, "HID device disconnected");
            break;
        case ESP_HIDD_EVENT_OUTPUT:
            ESP_LOGI(TAG, "HID output event");
            break;
        default:
            ESP_LOGI(TAG, "Unhandled HID event: %d", event);
            break;
    }
}
