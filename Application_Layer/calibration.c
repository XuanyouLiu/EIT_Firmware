#include <stdio.h>
#include "calibration.h"
#include "hardware.h"
#include "esp_err.h"
#include "esp_log.h"

static const char *TAG = "CALIBRATION";

const uint16_t SCR_RDATA_CONST = 80; //Fixed source gain value
const uint16_t SNS_RDATA_CONST = 100; //Fixed sense gain value

/* 2d array to hold calibration values and electrode mappings */
Calibration_t pair_calibration_map[NUM_ELECTRODE_PAIRS][NUM_SENSE_PAIRS] = {
    [0][0] = {.src_pos = 1, .src_neg = 2, .sense_pos = 3, .sense_neg = 4},
    [0][1] = {.src_pos = 1, .src_neg = 2, .sense_pos = 4, .sense_neg = 5},
    [0][2] = {.src_pos = 1, .src_neg = 2, .sense_pos = 5, .sense_neg = 6},
    [0][3] = {.src_pos = 1, .src_neg = 2, .sense_pos = 6, .sense_neg = 7},
    [0][4] = {.src_pos = 1, .src_neg = 2, .sense_pos = 7, .sense_neg = 8},
    [1][0] = {.src_pos = 2, .src_neg = 3, .sense_pos = 4, .sense_neg = 5},
    [1][1] = {.src_pos = 2, .src_neg = 3, .sense_pos = 5, .sense_neg = 6},
    [1][2] = {.src_pos = 2, .src_neg = 3, .sense_pos = 6, .sense_neg = 7},
    [1][3] = {.src_pos = 2, .src_neg = 3, .sense_pos = 7, .sense_neg = 8},
    [1][4] = {.src_pos = 2, .src_neg = 3, .sense_pos = 8, .sense_neg = 1},
    [2][0] = {.src_pos = 3, .src_neg = 4, .sense_pos = 5, .sense_neg = 6},
    [2][1] = {.src_pos = 3, .src_neg = 4, .sense_pos = 6, .sense_neg = 7},
    [2][2] = {.src_pos = 3, .src_neg = 4, .sense_pos = 7, .sense_neg = 8},
    [2][3] = {.src_pos = 3, .src_neg = 4, .sense_pos = 8, .sense_neg = 1},
    [2][4] = {.src_pos = 3, .src_neg = 4, .sense_pos = 1, .sense_neg = 2},
    [3][0] = {.src_pos = 4, .src_neg = 5, .sense_pos = 6, .sense_neg = 7},
    [3][1] = {.src_pos = 4, .src_neg = 5, .sense_pos = 7, .sense_neg = 8},
    [3][2] = {.src_pos = 4, .src_neg = 5, .sense_pos = 8, .sense_neg = 1},
    [3][3] = {.src_pos = 4, .src_neg = 5, .sense_pos = 1, .sense_neg = 2},
    [3][4] = {.src_pos = 4, .src_neg = 5, .sense_pos = 2, .sense_neg = 3},
    [4][0] = {.src_pos = 5, .src_neg = 6, .sense_pos = 7, .sense_neg = 8},
    [4][1] = {.src_pos = 5, .src_neg = 6, .sense_pos = 8, .sense_neg = 1},
    [4][2] = {.src_pos = 5, .src_neg = 6, .sense_pos = 1, .sense_neg = 2},
    [4][3] = {.src_pos = 5, .src_neg = 6, .sense_pos = 2, .sense_neg = 3},
    [4][4] = {.src_pos = 5, .src_neg = 6, .sense_pos = 3, .sense_neg = 4},
    [5][0] = {.src_pos = 6, .src_neg = 7, .sense_pos = 8, .sense_neg = 1},
    [5][1] = {.src_pos = 6, .src_neg = 7, .sense_pos = 1, .sense_neg = 2},
    [5][2] = {.src_pos = 6, .src_neg = 7, .sense_pos = 2, .sense_neg = 3},
    [5][3] = {.src_pos = 6, .src_neg = 7, .sense_pos = 3, .sense_neg = 4},
    [5][4] = {.src_pos = 6, .src_neg = 7, .sense_pos = 4, .sense_neg = 5},
    [6][0] = {.src_pos = 7, .src_neg = 8, .sense_pos = 1, .sense_neg = 2},
    [6][1] = {.src_pos = 7, .src_neg = 8, .sense_pos = 2, .sense_neg = 3},
    [6][2] = {.src_pos = 7, .src_neg = 8, .sense_pos = 3, .sense_neg = 4},
    [6][3] = {.src_pos = 7, .src_neg = 8, .sense_pos = 4, .sense_neg = 5},
    [6][4] = {.src_pos = 7, .src_neg = 8, .sense_pos = 5, .sense_neg = 6},
    [7][0] = {.src_pos = 8, .src_neg = 1, .sense_pos = 2, .sense_neg = 3},
    [7][1] = {.src_pos = 8, .src_neg = 1, .sense_pos = 3, .sense_neg = 4},
    [7][2] = {.src_pos = 8, .src_neg = 1, .sense_pos = 4, .sense_neg = 5},
    [7][3] = {.src_pos = 8, .src_neg = 1, .sense_pos = 5, .sense_neg = 6},
    [7][4] = {.src_pos = 8, .src_neg = 1, .sense_pos = 6, .sense_neg = 7},
};

void calibration_task(void* args) {


    /* Small startup delay */

    if ( set_src_inamp_gain(SCR_RDATA_CONST) != ESP_OK) {
        #if DEBUG
        ESP_LOGE(TAG, "Failed to set source inamp gain");
        #endif
        vTaskDelete(NULL);
        return;
    }

    if (set_sense_inamp_gain(SNS_RDATA_CONST) != ESP_OK) {
        #if DEBUG
        ESP_LOGE(TAG, "Failed to set fixed sense inamp gain");
        #endif
        vTaskDelete(NULL);
        return;
    }

    /* Populate reference_amp tare baseline for each pair when hand is at rest */
    for (uint8_t src_elec_pair = 0; src_elec_pair < NUM_ELECTRODE_PAIRS; src_elec_pair++) {
        for (uint8_t sense_elec_pair = 0; sense_elec_pair < NUM_SENSE_PAIRS; sense_elec_pair++) {
            Calibration_t* curr_config = &pair_calibration_map[src_elec_pair][sense_elec_pair];

            /* Set mux to src_elec_pair, sense_elec_pair */
            if (set_mux(curr_config->src_pos,
                        curr_config->src_neg,
                        curr_config->sense_pos,
                        curr_config->sense_neg) != ESP_OK) {
                #if DEBUG
                ESP_LOGE(TAG, "Failed to set mux for tare pair [%u][%u]", src_elec_pair, sense_elec_pair);
                #endif
                continue;
            }

            vTaskDelay(1);

            uint16_t tare_samples[BUFFER_LEN] = {0};
            size_t buffer_len = sizeof(tare_samples) / sizeof(tare_samples[0]);
            if (adcRead(tare_samples, buffer_len) != ESP_OK) {
                #if DEBUG
                ESP_LOGE(TAG, "Failed to read tare for pair [%u][%u]", src_elec_pair, sense_elec_pair);
                #endif
                continue;
            }

            curr_config->reference_amp = test_std_dev_mag((int16_t*)tare_samples, buffer_len, 2);

            ESP_LOGI(TAG, "Tare[%u][%u] reference_amp=%u",
                     src_elec_pair,
                     sense_elec_pair,
                     curr_config->reference_amp);
        }
    }

    /* Signal to the measurement task */
    if ( xSemaphoreGive(sem_cal_to_meas) != pdTRUE ) {
        #if DEBUG
        ESP_LOGE(TAG, "Failed to give semaphore");
        #endif
    }

    vTaskDelete(NULL);

}

void calibrate(void) {


    /* Small startup delay */
    vTaskDelay(pdMS_TO_TICKS(1000));

    if ( set_src_inamp_gain(SCR_RDATA_CONST) != ESP_OK) {
        #if DEBUG
        ESP_LOGE(TAG, "Failed to set source inamp gain");
        #endif
        return;
    }

    if (set_sense_inamp_gain(SNS_RDATA_CONST) != ESP_OK) {
        #if DEBUG
        ESP_LOGE(TAG, "Failed to set fixed sense inamp gain");
        #endif
        return;
    }

    /* Populate reference_amp tare baseline for each pair when hand is at rest */
    for (uint8_t src_elec_pair = 0; src_elec_pair < NUM_ELECTRODE_PAIRS; src_elec_pair++) {
        for (uint8_t sense_elec_pair = 0; sense_elec_pair < NUM_SENSE_PAIRS; sense_elec_pair++) {
            Calibration_t* curr_config = &pair_calibration_map[src_elec_pair][sense_elec_pair];

            /* Set mux to src_elec_pair, sense_elec_pair */
            if (set_mux(curr_config->src_pos,
                        curr_config->src_neg,
                        curr_config->sense_pos,
                        curr_config->sense_neg) != ESP_OK) {
                #if DEBUG
                ESP_LOGE(TAG, "Failed to set mux for tare pair [%u][%u]", src_elec_pair, sense_elec_pair);
                #endif
                continue;

            }


            uint16_t tare_samples[BUFFER_LEN] = {0};
            size_t buffer_len = sizeof(tare_samples) / sizeof(tare_samples[0]);
            if (adcRead(tare_samples, buffer_len) != ESP_OK) {
                #if DEBUG
                ESP_LOGE(TAG, "Failed to read tare for pair [%u][%u]", src_elec_pair, sense_elec_pair);
                #endif
                continue;
            }

            curr_config->reference_amp = test_std_dev_mag((int16_t*)tare_samples, buffer_len, 2);

            ESP_LOGI(TAG, "Tare[%u][%u] reference_amp=%u",
                     src_elec_pair,
                     sense_elec_pair,
                     curr_config->reference_amp);
        }
    }

    /* Signal to the measurement task */
    if ( xSemaphoreGive(sem_cal_to_meas) != pdTRUE ) {
        #if DEBUG
        ESP_LOGE(TAG, "Failed to give semaphore");
        #endif
    }

}