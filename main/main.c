#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_task_wdt.h"


// #include "../Device_Drivers/AD5270_DigiPot.h"
// #include "../Device_Drivers/AD5930_SigGen.h"

#include "../Application_Layer/measurement.h"
#include "../Application_Layer/calibration.h"
#include "../Middle_Ware/hardware.h"
#include "../Middle_Ware/hardware-test.h"

#define SIG_GEN_FREQ (2000.0f) 

static const char *TAG = "MAIN";

/* Task Details for Measurement Task */
TaskHandle_t meas_task;
static const char* meas_task_name = "MeasurementTask";
static const configSTACK_DEPTH_TYPE meas_task_stack_depth = 4000;
static const UBaseType_t meas_task_priority = 5;

/* Semaphore for syncing of calibration and measurement task*/
SemaphoreHandle_t sem_cal_to_meas = NULL;

void app_main(void)
{

    esp_task_wdt_deinit();

    /* Create Semaphore */
    sem_cal_to_meas = xSemaphoreCreateBinary();

    if (sem_cal_to_meas == NULL ) {
        #if DEBUG
        ESP_LOGE(TAG, "Failed to create semaphore");
        #endif
    }

    

    /* Initialize SPI Bus */
    if (init_spi() != ESP_OK) {
        #if DEBUG
        ESP_LOGE(TAG, "Failed to init SPI");
        #endif
        return;
    }


    /* Initialize ADC */
    if (adc_init() != ESP_OK) {
        #if DEBUG
        ESP_LOGE(TAG, "Failed to init ADC");
        #endif
        return;
    }


    if (init_mux() != ESP_OK) {
        #if DEBUG
        ESP_LOGE(TAG, "Failed to init Mux");
        #endif
    }


    if (init_inamp_pots() != ESP_OK) {
    #if DEBUG
    ESP_LOGE(TAG, "Failed to init inamp pots");
    #endif
         return;
    }



    if (signal_gen_start(SIG_GEN_FREQ) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize Signal Generator");
        return;
    }


        
    test_function();

    
    /* Run calibration */
    calibrate();

    /* Create a Task for Measurement */
    if ( xTaskCreate( &measurement_task, meas_task_name, meas_task_stack_depth, NULL, meas_task_priority, &meas_task ) != pdPASS ) {
        #if DEBUG
        ESP_LOGE(TAG, "Failed to create measurement task");
        #endif
    }

}
