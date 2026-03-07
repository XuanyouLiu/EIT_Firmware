#pragma once

#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"


#define NUM_ELECTRODE_PAIRS 8
#define NUM_SENSE_PAIRS (NUM_ELECTRODE_PAIRS - 3)
#define BUFFER_LEN 64
#define CALIBRATION_DSP_BUCKET 7



void calibrate(void);

/* Structure that holds calibration values and electrode number mappings*/
typedef struct {
    const uint8_t src_pos; //positive source electrode number
    const uint8_t src_neg; //negative source electrode number
    
    const uint8_t sense_pos; //positive sense electrode number
    const uint8_t sense_neg; //negative sense electrode number
        
    uint16_t ewma_amp;
    
} Calibration_t;

extern const uint16_t SCR_RDATA_CONST; //Fixed source gain value
extern const uint16_t SNS_RDATA_CONST; //Fixed sense gain value
/* 2d array to hold calibration values and electrode mappings */
extern Calibration_t pair_calibration_map [NUM_ELECTRODE_PAIRS][NUM_SENSE_PAIRS];

extern TaskHandle_t meas_task;




