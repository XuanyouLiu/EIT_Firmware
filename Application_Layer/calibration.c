#include <stdio.h>
#include "calibration.h"
#include "hardware.h"
#include "esp_err.h"
#include "esp_log.h"

static const char *TAG = "CALIBRATION";

const uint16_t SCR_RDATA_CONST = 70; //Fixed source gain value
const uint16_t SNS_RDATA_CONST = 15; //Fixed sense gain value

#define TOTAL_TARE_MEASUREMENTS (NUM_ELECTRODE_PAIRS * NUM_SENSE_PAIRS)
#define TARE_PRINT_COUNT 40

/* 2d array to hold calibration values and electrode mappings */



#define ADJACENT_40

#ifdef ADJACENT_40
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
#endif 


#ifdef STAR_40
Calibration_t pair_calibration_map[NUM_ELECTRODE_PAIRS][NUM_SENSE_PAIRS] = {
    [0][0] = {.src_pos = 1, .src_neg = 4, .sense_pos = 7, .sense_neg = 2},
    [0][1] = {.src_pos = 1, .src_neg = 4, .sense_pos = 2, .sense_neg = 5},
    [0][2] = {.src_pos = 1, .src_neg = 4, .sense_pos = 5, .sense_neg = 8},
    [0][3] = {.src_pos = 1, .src_neg = 4, .sense_pos = 8, .sense_neg = 3},
    [0][4] = {.src_pos = 1, .src_neg = 4, .sense_pos = 3, .sense_neg = 6},
    [1][0] = {.src_pos = 4, .src_neg = 7, .sense_pos = 2, .sense_neg = 5},
    [1][1] = {.src_pos = 4, .src_neg = 7, .sense_pos = 5, .sense_neg = 8},
    [1][2] = {.src_pos = 4, .src_neg = 7, .sense_pos = 8, .sense_neg = 3},
    [1][3] = {.src_pos = 4, .src_neg = 7, .sense_pos = 3, .sense_neg = 6},
    [1][4] = {.src_pos = 4, .src_neg = 7, .sense_pos = 6, .sense_neg = 1},
    [2][0] = {.src_pos = 7, .src_neg = 2, .sense_pos = 5, .sense_neg = 8},
    [2][1] = {.src_pos = 7, .src_neg = 2, .sense_pos = 8, .sense_neg = 3},
    [2][2] = {.src_pos = 7, .src_neg = 2, .sense_pos = 3, .sense_neg = 6},
    [2][3] = {.src_pos = 7, .src_neg = 2, .sense_pos = 6, .sense_neg = 1},
    [2][4] = {.src_pos = 7, .src_neg = 2, .sense_pos = 1, .sense_neg = 4},
    [3][0] = {.src_pos = 2, .src_neg = 5, .sense_pos = 8, .sense_neg = 3},
    [3][1] = {.src_pos = 2, .src_neg = 5, .sense_pos = 3, .sense_neg = 6},
    [3][2] = {.src_pos = 2, .src_neg = 5, .sense_pos = 6, .sense_neg = 1},
    [3][3] = {.src_pos = 2, .src_neg = 5, .sense_pos = 1, .sense_neg = 4},
    [3][4] = {.src_pos = 2, .src_neg = 5, .sense_pos = 4, .sense_neg = 7},
    [4][0] = {.src_pos = 5, .src_neg = 8, .sense_pos = 3, .sense_neg = 6},
    [4][1] = {.src_pos = 5, .src_neg = 8, .sense_pos = 6, .sense_neg = 1},
    [4][2] = {.src_pos = 5, .src_neg = 8, .sense_pos = 1, .sense_neg = 4},
    [4][3] = {.src_pos = 5, .src_neg = 8, .sense_pos = 4, .sense_neg = 7},
    [4][4] = {.src_pos = 5, .src_neg = 8, .sense_pos = 7, .sense_neg = 2},
    [5][0] = {.src_pos = 8, .src_neg = 3, .sense_pos = 6, .sense_neg = 1},
    [5][1] = {.src_pos = 8, .src_neg = 3, .sense_pos = 1, .sense_neg = 4},
    [5][2] = {.src_pos = 8, .src_neg = 3, .sense_pos = 4, .sense_neg = 7},
    [5][3] = {.src_pos = 8, .src_neg = 3, .sense_pos = 7, .sense_neg = 2},
    [5][4] = {.src_pos = 8, .src_neg = 3, .sense_pos = 2, .sense_neg = 5},
    [6][0] = {.src_pos = 3, .src_neg = 6, .sense_pos = 1, .sense_neg = 4},
    [6][1] = {.src_pos = 3, .src_neg = 6, .sense_pos = 4, .sense_neg = 7},
    [6][2] = {.src_pos = 3, .src_neg = 6, .sense_pos = 7, .sense_neg = 2},
    [6][3] = {.src_pos = 3, .src_neg = 6, .sense_pos = 2, .sense_neg = 5},
    [6][4] = {.src_pos = 3, .src_neg = 6, .sense_pos = 5, .sense_neg = 8},
    [7][0] = {.src_pos = 6, .src_neg = 1, .sense_pos = 4, .sense_neg = 7},
    [7][1] = {.src_pos = 6, .src_neg = 1, .sense_pos = 7, .sense_neg = 2},
    [7][2] = {.src_pos = 6, .src_neg = 1, .sense_pos = 2, .sense_neg = 5},
    [7][3] = {.src_pos = 6, .src_neg = 1, .sense_pos = 5, .sense_neg = 8},
    [7][4] = {.src_pos = 6, .src_neg = 1, .sense_pos = 8, .sense_neg = 3},
};
#endif 

#ifdef ADJACENT_20
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
    
};
#endif 

static void capture_tare_frame(uint16_t tare_values[TOTAL_TARE_MEASUREMENTS]) {
    uint8_t idx = 0;

    for (uint8_t src_elec_pair = 0; src_elec_pair < NUM_ELECTRODE_PAIRS; src_elec_pair++) {
        for (uint8_t sense_elec_pair = 0; sense_elec_pair < NUM_SENSE_PAIRS; sense_elec_pair++) {
            Calibration_t* curr_config = &pair_calibration_map[src_elec_pair][sense_elec_pair];
            uint16_t amplitude = 0;

            if (set_mux(curr_config->src_pos,
                        curr_config->src_neg,
                        curr_config->sense_pos,
                        curr_config->sense_neg) != ESP_OK) {
                #if DEBUG
                ESP_LOGE(TAG, "Failed to set mux for tare pair [%u][%u]", src_elec_pair, sense_elec_pair);
                #endif
            } else {
                vTaskDelay(1);

                uint16_t tare_samples[BUFFER_LEN] = {0};
                size_t buffer_len = sizeof(tare_samples) / sizeof(tare_samples[0]);
                if (adcRead(tare_samples, buffer_len) != ESP_OK) {
                    #if DEBUG
                    ESP_LOGE(TAG, "Failed to read tare for pair [%u][%u]", src_elec_pair, sense_elec_pair);
                    #endif
                } else {
                    amplitude = calc_std_dev_mag((int16_t*)tare_samples, buffer_len, 2);
                }
            }

            tare_values[idx++] = amplitude;
        }
    }
}

static void print_tare_frame(const uint16_t tare_values[TOTAL_TARE_MEASUREMENTS]) {
    for (uint8_t i = 0; i < TOTAL_TARE_MEASUREMENTS; i++) {
        printf("%u ", tare_values[i]);
    }
    printf("\n");
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

    /* Print 40 tare frames only; no tare baseline is stored internally */
    uint16_t tare_values[TOTAL_TARE_MEASUREMENTS] = {0};

    ESP_LOGI(TAG, "Printing %u tare measurements", TARE_PRINT_COUNT);
    printf("TARE\n");

    for (uint8_t tare_capture = 0; tare_capture < TARE_PRINT_COUNT; tare_capture++) {
        capture_tare_frame(tare_values);
        print_tare_frame(tare_values);
    }
}