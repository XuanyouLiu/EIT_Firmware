#include <stdio.h>
#include "measurement.h"
#include "calibration.h"
#include "hardware.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "MEASUREMENT";

#define AMP_FILTER_ALPHA 1.0f

extern uint16_t test_peak_to_peak();

int16_t adc_packet_buffers[MAX_ADC_PACKETS][ADC_READINGS_PER_PACKET] = {0};

void measurement_task(void* args) {
    const uint8_t total_measurements = NUM_ELECTRODE_PAIRS * NUM_SENSE_PAIRS;

    ESP_LOGI(TAG, "Measurement task starting");

    int16_t amps[NUM_ELECTRODE_PAIRS * NUM_SENSE_PAIRS] = {0};

    while (1) {
        vTaskDelay(1);
        int idx = 0;

        set_src_inamp_gain(300);
        set_sense_inamp_gain(30);

        for (uint8_t src_elec_pair = 0; src_elec_pair < NUM_ELECTRODE_PAIRS; src_elec_pair++) {
            for (uint8_t sense_elec_pair = 0; sense_elec_pair < NUM_SENSE_PAIRS; sense_elec_pair++) {
                Calibration_t* curr_config = &calibration_table[src_elec_pair][sense_elec_pair];
                
                if (set_mux(curr_config->src_pos, curr_config->src_neg, 
                            curr_config->sense_pos, curr_config->sense_neg) != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to set mux");
                    continue;
                }


                uint16_t amplitude = test_peak_to_peak();

                if (idx < total_measurements) {
                    float previous = (float)amps[idx];
                    float filtered = (1.0f - AMP_FILTER_ALPHA) * previous +
                                     AMP_FILTER_ALPHA * (float)amplitude;
                    amps[idx] = (int16_t)filtered;
                    idx++;
                }
            }
        }

        for (uint8_t i = 0; i < total_measurements; i++) {
            printf("%d ", amps[i]);
        }
        printf("\n");

        #if DEBUG
        ESP_LOGI(TAG, "End of Cycle");
        #endif
    }
}