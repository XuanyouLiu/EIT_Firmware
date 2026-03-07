#include <inttypes.h>
#include <stdio.h>

#include "measurement.h"
#include "calibration.h"
#include "hardware.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"

static const char *TAG = "MEASUREMENT";

#define AMP_FILTER_ALPHA 0.5f

// #define PROFILE_SAMPLE_RATE

uint16_t adc_packet_buffers[MAX_ADC_PACKETS][ADC_READINGS_PER_PACKET] = {0};

uint16_t calc_peak_to_peak(void) {
    uint16_t buf[512];

    if (adcRead(buf, 512) != 0) {
        ESP_LOGE(TAG, "Failed to read ADC samples");
        return 0;
    }

    return calc_std_dev_mag((int16_t *)buf, 512, 1);
}

void measurement_task(void* args) {
    const uint8_t total_measurements = NUM_ELECTRODE_PAIRS * NUM_SENSE_PAIRS;

    ESP_LOGI(TAG, "Measurement task starting");

    uint16_t amps[NUM_ELECTRODE_PAIRS * NUM_SENSE_PAIRS] = {0};
#ifdef PROFILE_SAMPLE_RATE
    uint32_t prev_us = 0;
#endif

    set_src_inamp_gain(SCR_RDATA_CONST);
    set_sense_inamp_gain(SNS_RDATA_CONST);

    while (1) {
        int idx = 0;

        for (uint8_t src_elec_pair = 0; src_elec_pair < NUM_ELECTRODE_PAIRS; src_elec_pair++) {
            for (uint8_t sense_elec_pair = 0; sense_elec_pair < NUM_SENSE_PAIRS; sense_elec_pair++) {
                Calibration_t* curr_config = &pair_calibration_map[src_elec_pair][sense_elec_pair];

                if (set_mux(curr_config->src_pos, curr_config->src_neg,
                            curr_config->sense_pos, curr_config->sense_neg) != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to set mux");
                    continue;
                }

                esp_rom_delay_us(300);

                uint16_t amplitude = calc_peak_to_peak();

                if (idx < total_measurements) {
                    float previous = (float)amps[idx];
                    float filtered = (1.0f - AMP_FILTER_ALPHA) * previous +
                                     AMP_FILTER_ALPHA * (float)amplitude;
                    uint16_t filtered_u16 = (uint16_t)filtered;
                    amps[idx] = amplitude;
                    curr_config->ewma_amp = filtered_u16;
                    idx++;
                }

            }

        }

        for (uint8_t i = 0; i < total_measurements; i++) {
            printf("%u ", amps[i]);
        }
        printf("\n");


#ifdef PROFILE_SAMPLE_RATE
        uint32_t timestamp_us = (uint32_t)esp_timer_get_time();
        uint32_t delta_us = (prev_us > 0) ? (timestamp_us - prev_us) : 0;
        prev_us = timestamp_us;
        printf("delta_us: %" PRIu32 "\n", delta_us);
#else
        // vTaskDelay(pdMS_TO_TICKS(100));
#endif

        #if DEBUG
        ESP_LOGI(TAG, "End of Cycle");
        #endif
    }
}
