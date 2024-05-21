#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "esp_log.h"

#define ADC_CHANNEL ADC1_CHANNEL_0  // GPIO 0 (ADC1_CH0) is used for reading

static const char *TAG = "ADC Example";

int debounceArray[4];
int debouncedSignal = 0;

void debounceSignal(int signal) {
    // enter values into array
    int oldValue0;
    int oldValue1;
    int oldValue2;
    oldValue0 = debounceArray[0];
    oldValue1 = debounceArray[1];
    oldValue2 = debounceArray[2];
    if (signal > 40) {
        debounceArray[0] = 1;
        debounceArray[1] = oldValue0;
        debounceArray[2] = oldValue1;
        debounceArray[3] = oldValue2;
    }
    else {
        debounceArray[0] = 0;
        debounceArray[1] = oldValue0;
        debounceArray[2] = oldValue1;
        debounceArray[3] = oldValue2;
    }

    // interpret debounce array
    int highSignal = debounceArray[0] + debounceArray[1] + debounceArray[2] + debounceArray[3];
    if (highSignal == 3) {
        debouncedSignal = 1;
    }
    else if (highSignal == 0) {
        debouncedSignal = 0;
    }
}



void app_main(void)
{
    // Configure ADC
    adc1_config_width(ADC_WIDTH_BIT_12);  // Set ADC width to 12 bits
    adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN_DB_0);  // Set attenuation to 0dB

    while (1) {
        int adc_value = adc1_get_raw(ADC_CHANNEL);  // Get ADC value
        //ESP_LOGI(TAG, "ADC Value: %d", adc_value);  // Log ADC value

        debounceSignal(adc_value);

        if (debouncedSignal) {
            ESP_LOGI(TAG, "Light On");
        }
        else {
            ESP_LOGE(TAG, "Light Off");
        }
        vTaskDelay(pdMS_TO_TICKS(250));  // Delay for 1 second
    }
}
