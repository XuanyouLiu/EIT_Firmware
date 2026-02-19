#include <stdio.h>
#include <string.h>
#include "measurement.h"
#include "calibration.h"
#include "hardware.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
//#include <rom/ets_sys.h>
//#include "esp_timer.h"

#define TARGET_BUCKET 7

static const char *TAG = "MEASUREMENT";

extern uint16_t test_peak_to_peak();

/* Initially cleared out */
int16_t adc_packet_buffers[MAX_ADC_PACKETS][ADC_READINGS_PER_PACKET] = {0};

/* * Helper function to read h`ardware and calculate magnitude 
 * Note: Assumes test_std_dev_mag and AD7450_Read are available via headers
 */

void measurement_task(void* args) {

    // /* Wait for calibration task to end */
    // if ( xSemaphoreTake( sem_cal_to_meas, portMAX_DELAY ) != pdPASS ) {
    //     #if DEBUG
    //     ESP_LOGE(TAG, "Semaphore failed");
    //     #endif
    // }

    ESP_LOGI(TAG, "Measurement task starting");

    /* Holds the magnitude for that electrode configuration */
    int16_t amps[NUM_ELECTRODE_PAIRS * NUM_SENSE_PAIRS];

    while (1) {

        vTaskDelay(1);
        int idx = 0; // reset index
        
        // Ensure gains are set
        set_src_inamp_gain(SCR_RDATA_CONST);
        set_sense_inamp_gain(max_calibrated_sense_rdata);
                set_src_inamp_gain(511);
                set_sense_inamp_gain(10);
        for (uint8_t src_elec_pair = 0; src_elec_pair < NUM_ELECTRODE_PAIRS; src_elec_pair++) {
            for (uint8_t sense_elec_pair = 0; sense_elec_pair < NUM_SENSE_PAIRS; sense_elec_pair++) {
                Calibration_t* curr_config = &calibration_table[src_elec_pair][sense_elec_pair];
                
                if (set_mux(curr_config->src_pos, curr_config->src_neg, 
                            curr_config->sense_pos, curr_config->sense_neg) != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to set mux");
                    continue;
                }

                // Small delay for settling
                // ets_delay_us(100);

                /* * MODIFICATION: 
                 * Original adcRead commented out because test_peak_to_peak reads 
                 * the hardware directly into its own local buffer.
                 */
                // if (adcRead((int16_t*)adc_packet_buffers[idx], ADC_READINGS_PER_PACKET, 0) != ESP_OK) {
                //     #if DEBUG
                //     ESP_LOGE(TAG, "Failed to read ADC");
                //     #endif
                //     continue;
                // }

                /* * MODIFICATION:
                 * Replaced dsp_freq_amp with test_peak_to_peak()
                 */
                uint16_t amplitude = test_peak_to_peak();
                

                // Calculate difference: calibration_table->reference_amp - amplitude
                if (idx < (NUM_ELECTRODE_PAIRS * NUM_SENSE_PAIRS)) {
                    
                    amps[idx] = amplitude;// - (int16_t)curr_config->reference_amp;
                    
                    // printf("Idx:%u Ref:%d Amp:%u Diff:%d\n", 
                    //        idx, 
                    //        (int)curr_config->reference_amp, 
                    //        (unsigned)amplitude, 
                    //        amps[idx]);
                           
                    idx++;
                                            vTaskDelay(pdMS_TO_TICKS(1));

                } 
                

            }
        }

        for (uint8_t i = 0; i < 40; i++) {
            
                
               printf("%d ", amps[ i ] );
        
        }
        printf("\n");


        #if DEBUG
        ESP_LOGI(TAG, "End of Cycle");
        #endif
    }
}