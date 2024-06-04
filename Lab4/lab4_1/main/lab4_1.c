#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "icm42670.h"

#define TAG "ICM42670"

void app_main(void)
{
    float accel_x, accel_y, accel_z;

    // Initialize the ICM42670
    if (icm42670_init() != ESP_OK) {
        ESP_LOGI(TAG, "Failed to initialize ICM42670");
        return;
    }

    while (1) {
        // Read accelerometer data
        if (icm42670_read_accel(&accel_x, &accel_y, &accel_z) == ESP_OK) {
            // Determine the inclination
            if (accel_x > 0.5) {
                ESP_LOGI(TAG, "UP");
            } else if (accel_x < -0.5) {
                ESP_LOGI(TAG, "DOWN");
            }

            if (accel_y > 0.5) {
                ESP_LOGI(TAG, "RIGHT");
            } else if (accel_y < -0.5) {
                ESP_LOGI(TAG, "LEFT");
            }
        } else {
            ESP_LOGI(TAG, "Failed to read accelerometer data");
        }

        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second
    }
}
