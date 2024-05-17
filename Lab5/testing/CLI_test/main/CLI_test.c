#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_log.h"

#define UART_NUM UART_NUM_0
#define BUF_SIZE (1024)

static const char *TAG = "send";

void print_message(int count, const char *message) {
    for (int i = 0; i < count; i++) {
        printf("%s\n", message);
    }
}

void parse_and_execute_command(char *input) {
    int count;
    char message[BUF_SIZE];

    if (sscanf(input, "%d '%[^\']'", &count, message) == 2) {
        print_message(count, message);
    } else {
        ESP_LOGI(TAG, "Invalid command format. Usage: <count> '<message>'");
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "ESP32C3 Application Started");

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    uint8_t data[BUF_SIZE];
    while (1) {
        int len = uart_read_bytes(UART_NUM, data, BUF_SIZE - 1, 20 / portTICK_PERIOD_MS);
        if (len > 0) {
            data[len] = '\0'; // Null-terminate the received data
            //parse_and_execute_command((char *)data);
            printf("first character: %c",data[0]);
        }
        vTaskDelay(100 / portTICK_PERIOD_MS); // Delay to prevent task from consuming too much CPU
    }
}
