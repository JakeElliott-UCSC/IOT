#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define ADC_CHANNEL ADC1_CHANNEL_0  // GPIO 0 (ADC1_CH0) is used for reading
#define LED_PIN GPIO_NUM_2          // GPIO 2 is used for the LED
#define ADC_THRESHOLD 2000          // Threshold for turning on the LED

static const char *TAG = "ADC_LED_Example";

void app_main(void)
{
    // Configure ADC
    adc1_config_width(ADC_WIDTH_BIT_12);  // Set ADC width to 12 bits
    adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN_DB_0);  // Set attenuation to 0dB

    // Configure LED pin
    esp_rom_gpio_pad_select_gpio(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

    while (1) {
        // liveliness signal
        ESP_LOGI(TAG, "Hello World");
        // set gpio pin 2 to high
        gpio_set_level(LED_PIN, 1);
        // Delay for 1 second
        vTaskDelay(pdMS_TO_TICKS(1000));

        gpio_set_level(LED_PIN, 0);

        vTaskDelay(pdMS_TO_TICKS(1000));


        // int adc_value = adc1_get_raw(ADC_CHANNEL);  // Get ADC value
        // ESP_LOGI(TAG, "ADC Value: %d", adc_value);  // Log ADC value

        // if (adc_value > ADC_THRESHOLD) {
        //     gpio_set_level(LED_PIN, 1);  // Turn on LED
        // } else {
        //     gpio_set_level(LED_PIN, 0);  // Turn off LED
        // }

        // vTaskDelay(pdMS_TO_TICKS(1000));  // Delay for 1 second
    }
}
