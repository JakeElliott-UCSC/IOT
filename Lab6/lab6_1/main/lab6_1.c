#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#define TRIG_PIN GPIO_NUM_4
#define ECHO_PIN GPIO_NUM_5

static void init_hcsr04() {
    esp_rom_gpio_pad_select_gpio(TRIG_PIN);
    esp_rom_gpio_pad_select_gpio(ECHO_PIN);
    gpio_set_direction(TRIG_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(ECHO_PIN, GPIO_MODE_INPUT);
    gpio_set_level(TRIG_PIN, 0);
}

static int64_t measure_distance() {
    // Send trigger pulse
    gpio_set_level(TRIG_PIN, 1);
    esp_rom_delay_us(10);  // 10 microseconds delay
    gpio_set_level(TRIG_PIN, 0);

    // Wait for echo start
    while (gpio_get_level(ECHO_PIN) == 0) {
    }

    int64_t start_time = esp_timer_get_time();

    // Wait for echo end
    while (gpio_get_level(ECHO_PIN) == 1) {
    }

    int64_t end_time = esp_timer_get_time();
    int64_t duration = end_time - start_time;

    // Calculate distance in centimeters
    int64_t distance = (duration * 0.034) / 2;

    return distance;
}

void app_main() {
    init_hcsr04();

    while (1) {
        int64_t distance = measure_distance();
        printf("Distance: %lld cm\n", distance);
        vTaskDelay(pdMS_TO_TICKS(1000));  // Wait for 1 second
    }
}
