#include "hardware-test.h"
#include "hardware.h"
#include "esp_log.h"
#include "../Device_Drivers/AD7450_ADC.h"
#include "test_data_gen.h"
#include "freertos/FreeRTOS.h"

#include <math.h>

#define TARGET_BUCKET 7

static const char *TAG = "HARDWARE_TEST";

int test_adc(void) {
    uint16_t buf[64];
    
    if ( AD7450_init() != 0) {
        // #if DEBUG
        ESP_LOGE(TAG, "test_adc init failed");
        // #endif
        return -1;
    }
    
    while(1) {
        if ( AD7450_Read(buf, 1) != 0) {
            ESP_LOGE(TAG, "test_adc failed");   
            return -1;
        }

        for (int i = 0; i < 1; i++) {
            ESP_LOGI(TAG, "buf[%d]=%u", i, buf[i]);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    return 0;
}

int test_signal_gen(void) {
    // Dummy test for signal generator
    float freq = 1000.0f;
    if (signal_gen_start(freq) != 0) {
        #if DEBUG
        ESP_LOGE(TAG, "test_signal_gen failed");
        #endif
        return -1;
    }
    ESP_LOGI(TAG, "test_signal_gen passed");
    return 0;
}

int test_inamp_pots(void) {
    // uint16_t in_amp_scr = 0;
    // uint16_t in_amp_sense = 200;

    // Dummy test for inamp pots
    if (init_inamp_pots() != 0) {
        #if DEBUG
        ESP_LOGE(TAG, "test_inamp_pots init failed");
        #endif
        return -1;
    }
    
    for (int i = 512; i > 0; i--) {
        vTaskDelay(pdMS_TO_TICKS(100));
        ESP_LOGI(TAG, "Setting src/sns gain to %d", i);
        if (set_src_inamp_gain(i) != 0) {
            ESP_LOGE(TAG, "test_inamp_pots set src gain failed");
            return -1;
        }
        
        // if (set_sense_inamp_gain(i) != 0) {
        //     ESP_LOGE(TAG, "test_inamp_pots set sense gain failed");
        //     return -1;
        // }
    }

    ESP_LOGI(TAG, "test_inamp_pots passed");
    return 0;
}

int test_mux(void) {
    // Dummy test for mux
    if (init_mux() != 0) {
        #if DEBUG
        ESP_LOGE(TAG, "test_mux init failed");
        #endif
        return -1;
    }
    int ch1 = 1, ch2 = 1, ch3 = 1, ch4 = 1;
    while(1) {
        if (set_mux(ch1, ch2, ch3, ch4) != 0) {
            #if DEBUG
            ESP_LOGE(TAG, "test_mux set failed");
            #endif
            return -1;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    ESP_LOGI(TAG, "test_mux passed");
    return 0;
}



int test_dsp(bool clipped, float clip_percent) {
    int16_t samples[TEST_GEN_N];
    float test_freq = 54000.0f; // 20 kHz test signal

    #if DEBUG
    ESP_LOGI(TAG, "test_dsp: clipped=%d, clip_percent=%0.2f", clipped, clip_percent);
    #endif
    
    //generate_sine_int16(samples, test_freq);
    generate_sine_int16_multi_random_amp_clipped(samples, test_freq, 3.0f, 5.0f, clipped, clip_percent);
    
    uint16_t amplitude = (uint16_t)dsp_freq_amp(samples, TEST_GEN_N, TARGET_BUCKET, TARGET_BUCKET);
    ESP_LOGI(TAG, "test_dsp: Generated %0.1f Hz, Amplitude: %u", test_freq, amplitude);
    

    bool clip_detected = detect_opamp_clipping(samples, TEST_GEN_N, 100, 10, 31);

    ESP_LOGI(TAG, "test_dsp: clip_detected=%d, clipped=%d, clip_percent=%0.2f", clip_detected, clipped, clip_percent);


    return 0;
}

void test_function(void) {
    // test_signal_gen();
    // test_inamp_pots();
    // test_mux();

    set_src_inamp_gain(511);
    set_sense_inamp_gain(300);

    set_mux(1, 2, 3, 4);
    
    test_adc();

    while (1) {
        uint16_t mag = test_peak_to_peak();
        printf("mag: %d\n", mag);
        vTaskDelay(pdMS_TO_TICKS(100));

    }

}

uint16_t test_peak_to_peak() {
    uint16_t buf[64];

    if (adcRead(buf, 64) != 0) {
        ESP_LOGE(TAG, "test_adc failed");   
        return -1;
    }


    // for (int i = 0; i < 64; i++) {
    //     ESP_LOGI(TAG, "buf[%d] = %d", i, buf[i]);
    // }
    return test_std_dev_mag((int16_t *)buf, 64, 2);


}



uint16_t test_std_dev_mag_test(int16_t* buf, uint16_t buf_len, float std_multiplier) {
    if (buf_len == 0) return 0;

    // Use signed 32-bit to prevent overflow/underflow from int16_t inputs
    int32_t rolling_sum = 0;
    for (uint16_t i = 0; i < buf_len; i++) {
        rolling_sum += buf[i];
    }

    float mean = (float)rolling_sum / (float)buf_len;
    float variance_sum = 0;

    for (uint16_t i = 0; i < buf_len; i++) {
        float diff = (float)buf[i] - mean;
        variance_sum += diff * diff;
    }

    float variance = variance_sum / (float)buf_len;
    float sigma = sqrtf(variance);

    // // This is the simplified version of your range logic
    // float std_range = std_multiplier * sigma * mean;

    // // printf("mean:%f sigma:%f\n", mean, sigma);

    // Safety check: Ensure we don't overflow uint16_t (max 65535)
    // if (std_range > 65535.0f) return 65535;
    
    return (uint16_t)sigma;
}