#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/usb_serial_jtag.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_task_wdt.h"

// #include "../Device_Drivers/AD5270_DigiPot.h"
// #include "../Device_Drivers/AD5930_SigGen.h"

#include "../Application_Layer/measurement.h"
#include "../Application_Layer/calibration.h"
#include "../Middle_Ware/hardware.h"
#include "../Middle_Ware/hardware-test.h"

#define SIG_GEN_FREQ (50000.0f) 

static const char *TAG = "MAIN";

/* Task Details for Measurement Task */
TaskHandle_t meas_task;
static const char* meas_task_name = "MeasurementTask";
static const configSTACK_DEPTH_TYPE meas_task_stack_depth = 4000;
static const UBaseType_t meas_task_priority = 5;

static void init_serial_command_input(void)
{
    usb_serial_jtag_driver_config_t usb_serial_config = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_serial_config));
}

static void wait_for_serial_command(char command_lower,
                                    char command_upper,
                                    const char *description)
{
    printf("Send '%c' to start %s.\n", command_lower, description);

    while (1) {
        char input = 0;
        int bytes_read = usb_serial_jtag_read_bytes(&input, 1, pdMS_TO_TICKS(10));
        if (bytes_read <= 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (input == command_lower || input == command_upper) {
            printf("Starting %s.\n", description);
            return;
        }
        printf("Incorrect char. You entered: %c\n", (char)input);
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "app_main start");
    ESP_LOGI(TAG, "Firmware build: %s %s", __DATE__, __TIME__);

    esp_task_wdt_deinit();
    init_serial_command_input();

    

    /* Initialize SPI Bus */
    if (init_spi() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init SPI");
        return;
    }


    /* Initialize ADC */
    if (adc_init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init ADC");
        return;
    }


    if (init_mux() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init Mux");
        return;
    }


    if (init_inamp_pots() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init inamp pots");
         return;
    }



    if (signal_gen_start(SIG_GEN_FREQ) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize Signal Generator");
        return;
    }


        
    // test_function();

    
    // wait_for_serial_command('c', 'C', "calibration");

    /* Run calibration */
    // calibrate();

    // wait_for_serial_command('m', 'M', "measurement");

    /* Create a Task for Measurement */
    if ( xTaskCreate( &measurement_task, meas_task_name, meas_task_stack_depth, NULL, meas_task_priority, &meas_task ) != pdPASS ) {
        ESP_LOGE(TAG, "Failed to create measurement task");
    }

}
