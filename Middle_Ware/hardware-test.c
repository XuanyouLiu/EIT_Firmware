#include "hardware-test.h"
#include "hardware.h"
#include "esp_log.h"
#include "../Device_Drivers/AD7450_ADC.h"
#include "test_data_gen.h"
#include "freertos/FreeRTOS.h"
#include "esp_timer.h"

#include <math.h>

#define TARGET_BUCKET 7
#define ADC_STREAM_ACTIVE_MS 400
#define ADC_STREAM_BREAK_MS 1000

static const char *TAG = "HARDWARE_TEST";

int test_adc(void) {
    if ( AD7450_init() != 0) {
        // #if DEBUG
        ESP_LOGE(TAG, "test_adc init failed");
        // #endif
        return -1;
    }

            set_mux(1, 2, 3, 4);

    
    while (1) {
        int64_t window_start_us = esp_timer_get_time();
        int64_t window_end_us = window_start_us + ((int64_t)ADC_STREAM_ACTIVE_MS * 1000);

        uint16_t adc_buffer[250];
        uint16_t buffer_index = 0;

        while (esp_timer_get_time() < window_end_us) {
            uint16_t adc_value = 0;
            if (adcRead(&adc_value, 1) != 0) {
                continue;
            }
            
            adc_buffer[buffer_index++] = adc_value;
            
            // Print buffer when full
            if (buffer_index >= 250) {
                for (uint16_t i = 0; i < 250; i++) {
                    printf("ADC: %u\n", adc_buffer[i]);
                }
                buffer_index = 0;
            }
        }

        // Print remaining samples in buffer
        if (buffer_index > 0) {
            for (uint16_t i = 0; i < buffer_index; i++) {
                printf("ADC: %u\n", adc_buffer[i]);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(ADC_STREAM_BREAK_MS));
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
    
    for (int i = 500; i > 0; i -= 5) {
        vTaskDelay(pdMS_TO_TICKS(100));
        ESP_LOGI(TAG, "Setting src/sns gain to %d", i);
        // if (set_src_inamp_gain(i) != 0) {
        //     ESP_LOGE(TAG, "test_inamp_pots set src gain failed");
        //     return -1;
        // }
        
        if (set_sense_inamp_gain(i) != 0) {
            ESP_LOGE(TAG, "test_inamp_pots set sense gain failed");
            return -1;
        }
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
        set_mux(1, 2, 3, 4);

    // test_signal_gen();
    // test_inamp_pots();
    set_src_inamp_gain(80);
    set_sense_inamp_gain(10);
    // test_mux();



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
    return test_std_dev_mag((int16_t *)buf, 64, 1);


}



uint16_t test_std_dev_mag_test(int16_t* buf, uint16_t buf_len, float std_multiplier) {
    if (buf_len == 0) return 0;

    // Use signed 32-bit to prevent overflow/underflow from int16_t inputs
    int32_t rolling_sum = 0;
    for (uint16_t i = 0; i < buf_len; i++) {
        rolling_sum += buf[i];
    }

    float mean = (float)rolling_sum / (float)buf_len;
    float abs_dev_sum = 0;

    for (uint16_t i = 0; i < buf_len; i++) {
        float diff = (float)buf[i] - mean;
        abs_dev_sum += (diff >= 0 ? diff : -diff); // absolute value
    }

    float mean_abs_dev = abs_dev_sum / (float)buf_len;

    // Return mean absolute deviation scaled by the multiplier
    return (uint16_t)(mean_abs_dev * 1);
}