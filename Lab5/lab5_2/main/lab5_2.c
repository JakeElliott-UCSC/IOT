#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "esp_log.h"

#define ADC_CHANNEL ADC1_CHANNEL_0  // GPIO 0 (ADC1_CH0) is used for reading

static const char *TAG = "ADC Example";

void app_main(void)
{
    // Configure ADC
    adc1_config_width(ADC_WIDTH_BIT_12);  // Set ADC width to 12 bits
    adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN_DB_0);  // Set attenuation to 0dB

    while (1) {
        int adc_value = adc1_get_raw(ADC_CHANNEL);  // Get ADC value
        //ESP_LOGI(TAG, "ADC Value: %d", adc_value);  // Log ADC value
        if (adc_value > 40) {
            ESP_LOGI(TAG, "Light On");
        }
        else {
            ESP_LOGE(TAG, "Light Off");
        }
        vTaskDelay(pdMS_TO_TICKS(500));  // Delay for 1 second
    }
}
