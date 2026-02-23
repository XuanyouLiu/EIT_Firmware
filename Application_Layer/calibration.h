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



void calibration_task(void* args);
void calibrate(void);

/* Structure that holds calibration values and electrode number mappings*/
typedef struct {
    const uint8_t src_pos; //positive source electrode number
    const uint8_t src_neg; //negative source electrode number
    
    const uint8_t sense_pos; //positive sense electrode number
    const uint8_t sense_neg; //negative sense electrode number
        
    uint16_t reference_amp; //The tare amplitude sensed in the calibration 
    uint16_t ewma_amp; //Baseline-subtracted exponential moving average amplitude
    
} Calibration_t;

extern const uint16_t SCR_RDATA_CONST; //Fixed source gain value
extern const uint16_t SNS_RDATA_CONST; //Fixed sense gain value
/* 2d array to hold calibration values and electrode mappings */
extern Calibration_t pair_calibration_map [NUM_ELECTRODE_PAIRS][NUM_SENSE_PAIRS];

/* Tasks and Primative Handles*/
extern TaskHandle_t cal_task;
extern TaskHandle_t meas_task;
extern SemaphoreHandle_t sem_cal_to_meas;




