#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <math.h>

#define ADC_CHANNEL ADC1_CHANNEL_0  // GPIO 0 (ADC1_CH0) is used for reading
#define LED_PIN GPIO_NUM_2          // GPIO 2 is used for the LED
#define ADC_THRESHOLD 2000          // Threshold for turning on the LED

// 1 = .
// 0 = -
int MorseArray[123] = {1,2,3};




static const char *TAG = "ADC_LED_Example";

// print morse character and a space after
void printMorseCharacter(int code) {

    int i = 1;
    int j = 0;

    int character = 0;


    while (character != 2) {

        //printf("flag\n");
        //printf("i: %d, j: %d\n",i,j);
        character = ((code % ((int)pow(10,i))) / ((int)pow(10,j)));
        //printf("character num: %d\n",character);

        switch (character) {
            case 0:
                printf(".");
                break;
            case 1:
                printf("-");
                break;
            case 2:
                printf(" ");
                break;
        }
        i++;
        j++;
        if (i > 20) {
            ESP_LOGE(TAG, "ERROR: printMorseCharacter exceeded 20 iterations");
            break;
        }
    }
}

void printMorseString(const char* message){
    // iterate over string until message is complete
    // message must be null terminated
    int x = 0;


    while (message[x] != NULL) {

        x++;
        if (x > 15) {
            ESP_LOGE(TAG, "ERROR: printMorseString exceeded 15 iterations");
            break;
        }
    }
}












void app_main(void)
{
    // set morse code values
    MorseArray[(int)' '] = 2;

    MorseArray[(int)'a'] = 210;
    MorseArray[(int)'b'] = 20001;
    MorseArray[(int)'c'] = 20101;
    MorseArray[(int)'d'] = 2001;
    MorseArray[(int)'e'] = 20;
    MorseArray[(int)'f'] = 20100;
    MorseArray[(int)'g'] = 2011;
    MorseArray[(int)'h'] = 20000;
    MorseArray[(int)'i'] = 200;
    MorseArray[(int)'j'] = 21110;
    MorseArray[(int)'k'] = 2101;
    MorseArray[(int)'l'] = 20010;
    MorseArray[(int)'m'] = 211;
    MorseArray[(int)'n'] = 201;
    MorseArray[(int)'o'] = 2111;
    MorseArray[(int)'p'] = 20110;
    MorseArray[(int)'q'] = 21011;
    MorseArray[(int)'r'] = 2010;
    MorseArray[(int)'s'] = 2000;
    MorseArray[(int)'t'] = 21;
    MorseArray[(int)'u'] = 2100;
    MorseArray[(int)'v'] = 21000;
    MorseArray[(int)'w'] = 2110;
    MorseArray[(int)'x'] = 21001;
    MorseArray[(int)'y'] = 21101;
    MorseArray[(int)'z'] = 20011;

    MorseArray[(int)'A'] = 210;
    MorseArray[(int)'B'] = 20001;
    MorseArray[(int)'C'] = 20101;
    MorseArray[(int)'D'] = 2001;
    MorseArray[(int)'E'] = 20;
    MorseArray[(int)'F'] = 20100;
    MorseArray[(int)'G'] = 2011;
    MorseArray[(int)'H'] = 20000;
    MorseArray[(int)'I'] = 200;
    MorseArray[(int)'J'] = 21110;
    MorseArray[(int)'K'] = 2101;
    MorseArray[(int)'L'] = 20010;
    MorseArray[(int)'M'] = 211;
    MorseArray[(int)'N'] = 201;
    MorseArray[(int)'O'] = 2111;
    MorseArray[(int)'P'] = 20110;
    MorseArray[(int)'Q'] = 21011;
    MorseArray[(int)'R'] = 2010;
    MorseArray[(int)'S'] = 2000;
    MorseArray[(int)'T'] = 21;
    MorseArray[(int)'U'] = 2100;
    MorseArray[(int)'V'] = 21000;
    MorseArray[(int)'W'] = 2110;
    MorseArray[(int)'X'] = 21001;
    MorseArray[(int)'Y'] = 21101;
    MorseArray[(int)'Z'] = 20011;

    MorseArray[(int)'1'] = 211110;
    MorseArray[(int)'2'] = 211100;
    MorseArray[(int)'3'] = 211000;
    MorseArray[(int)'4'] = 210000;
    MorseArray[(int)'5'] = 200000;
    MorseArray[(int)'6'] = 200001;
    MorseArray[(int)'7'] = 200011;
    MorseArray[(int)'8'] = 200111;
    MorseArray[(int)'9'] = 201111;
    MorseArray[(int)'0'] = 211111;

    // Configure ADC
    adc1_config_width(ADC_WIDTH_BIT_12);  // Set ADC width to 12 bits
    adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN_DB_0);  // Set attenuation to 0dB

    // Configure LED pin
    esp_rom_gpio_pad_select_gpio(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

    // 60 seems too quick. 125 might be the fastest
    int time_delta = 250;



    // print the letter Q code
    printMorseCharacter(MorseArray[(int)'H']);
    printMorseCharacter(MorseArray[(int)'E']);
    printMorseCharacter(MorseArray[(int)'L']);
    printMorseCharacter(MorseArray[(int)'L']);
    printMorseCharacter(MorseArray[(int)'O']);

    printMorseCharacter(MorseArray[(int)' ']);

    printMorseCharacter(MorseArray[(int)'E']);
    printMorseCharacter(MorseArray[(int)'S']);
    printMorseCharacter(MorseArray[(int)'P']);
    printMorseCharacter(MorseArray[(int)'3']);
    printMorseCharacter(MorseArray[(int)'2']);
    printf("\n");


    while (1) {
        // liveliness signal
        ESP_LOGI(TAG, "Hello World");
        
        // set gpio pin 2 to high
        gpio_set_level(LED_PIN, 1);
        // Delay for 1 second
        vTaskDelay(pdMS_TO_TICKS(time_delta));

        gpio_set_level(LED_PIN, 0);

        vTaskDelay(pdMS_TO_TICKS(time_delta));


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
