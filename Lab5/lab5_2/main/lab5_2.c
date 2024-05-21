#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "esp_log.h"

#define ADC_CHANNEL ADC1_CHANNEL_0  // GPIO 0 (ADC1_CH0) is used for reading

#define READ_ARRAY_SIZE 10

static const char *TAG = "ADC Example";

int debounceArray[4];
int debouncedSignal = 0;

int readArray[READ_ARRAY_SIZE];

int charIncoming = 1;
int morseSignal = 0;


// dash - high for 800
// dot - high for 100


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

// each bit read is 0.025 seconds
// 1 = - (4 bits)
// 0 = . (32 bits)
// 2 = space
int readMorse(int signal) {
    
    // feed new data into array
    int oldval = readArray[0];
    int shiftval = readArray[0];
    readArray[0] = signal;

    // shift old data over to the right
    int i = 0;
    for (i = 1; i < READ_ARRAY_SIZE; i++) {
        shiftval = readArray[i];
        readArray[i] = oldval;
        oldval = shiftval;
    }

    // interpret the data (first 10 bits)
    int morseCharacter = 0;
    for (i=0;i<READ_ARRAY_SIZE;i++) {
        morseCharacter += readArray[i];
    }

    if (morseCharacter == 0){
        return 2;
    }
    else if (morseCharacter == 10) {
        return 1;
    }
    else if ((readArray[0] + readArray[READ_ARRAY_SIZE-1] == 0) && morseCharacter > 4) {
        return 0;
    }

    //printf("morseCharacter: %d\n",morseCharacter);
    // if (morseCharacter > 6) {
    //     //printf("-");
    //     return 1;
    // }
    // else if (morseCharacter > 4) {
    //     //printf(".");
    //     return 0;
    // }
    // else if (morseCharacter == 0) {
    //     //printf(" ");
    //     return 2;
    // }
    return 10;
}


void printMorse(int symbol) {
    switch (symbol) {
        case 0:
        printf(".");
        break;
        case 1:
        printf("-");
        break;
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

        // if (debouncedSignal) {
        //     ESP_LOGI(TAG, "Light On");
        // }
        // else {
        //     ESP_LOGE(TAG, "Light Off");
        // }

        morseSignal = readMorse(debouncedSignal);
        printf("Morse Signal: %d\n",morseSignal);

        // if we see a space character, a new morse character is comming
        if (morseSignal == 2) {
            charIncoming = 1;
        }
        // if we just recieved a character, stop expecting one for now
        else {
            charIncoming = 0;
        }

        if (charIncoming) {
            printMorse(morseSignal);
        }




        vTaskDelay(pdMS_TO_TICKS(20));  // Delay for 0.02 seconds
    }
}
