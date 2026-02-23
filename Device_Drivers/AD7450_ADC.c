#include "AD7450_ADC.h"
#include "../Middle_Ware/hardware.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>
#include "soc/spi_struct.h"  // For spi_dev_t and GPSPI2

#define USE_DIRECT_REGISTERS

static const char *TAG = "AD7450";
static spi_device_handle_t ad7450_handle = NULL;

static inline uint16_t AD7450_Read_Direct_Registers(void);

int AD7450_init() {
    if (ad7450_handle != NULL) {
        ESP_LOGW(TAG, "AD7450 already initialized");
        return ESP_OK;
    }

    spi_device_interface_config_t devcfg = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .mode = 2, // CPOL=1, CPHA=0 (SCLK idle high, sample on rising edge)
        .clock_speed_hz = SPI_MASTER_FREQ_80M, // 8 MHz
        .spics_io_num = PIN_CS_ADC,
        .queue_size = AD7450_QUEUE_SIZE,
    };

    esp_err_t ret = spi_bus_add_device(SPI2_HOST, &devcfg, &ad7450_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add device to SPI bus: %s", esp_err_to_name(ret));
        return ret;
    }

    // Perform a dummy read to ensure the device is powered up and in normal mode
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 16;
    t.flags = SPI_TRANS_USE_RXDATA; // Use TX data to ensure MOSI is driven (though ignored by ADC)
    
    ret = spi_device_transmit(ad7450_handle, &t);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Dummy read failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "AD7450 initialized successfully");
    return ESP_OK;
}

int AD7450_Read(uint16_t *buf, uint32_t len)
{
#ifdef USE_DIRECT_REGISTERS
    // Use direct register access (bypasses all HAL/LL layers)
    for (uint32_t i = 0; i < len; i++) {
        buf[i] = AD7450_Read_Direct_Registers();
    }
    return 0;
#else
    // Use ESP-IDF SPI driver
    if (!ad7450_handle) return -1;

    spi_transaction_t t;
    uint8_t rx[2];

    memset(&t, 0, sizeof(t));
    t.length = 16;
    t.rx_buffer = rx;

    for (uint32_t i = 0; i < len; i++) {
        if (spi_device_polling_transmit(ad7450_handle, &t) != ESP_OK) {
            return -1;
        }

        buf[i] = (uint16_t)((rx[0] << 8) | rx[1]);
    }

    return 0;
#endif
}

/**
 * @brief Direct register-level 16-bit SPI transaction (bypasses all LL wrapper functions)
 * 
 * This function writes directly to SPI2 peripheral registers in the exact sequence
 * captured from the trace output, replicating a 16-bit full-duplex transaction
 * without calling any spi_ll_* functions.
 * 
 * IMPORTANT: SPI bus must be initialized first via AD7450_init()
 * This function manually controls CS and writes to registers directly.
 * 
 * @return 16-bit value read from SPI data buffer
 */
static inline uint16_t AD7450_Read_Direct_Registers(void)
{
    spi_dev_t *hw = &GPSPI2;
    
    static bool cs_gpio_configured = false;
    if (!cs_gpio_configured) {
        gpio_reset_pin(PIN_CS_ADC);
        gpio_set_direction(PIN_CS_ADC, GPIO_MODE_OUTPUT);
        gpio_set_level(PIN_CS_ADC, 1);
        cs_gpio_configured = true;
        for (volatile int i = 0; i < 100; i++);
    }
    
    gpio_set_level(PIN_CS_ADC, 0);
    
    hw->dma_int_raw.val = 0x00000000;
    
    while (hw->cmd.val != 0);
    
    hw->ctrl.val = 0x00340000;
    hw->user.val = 0x18000241;
    
    hw->ms_dlen.val = 0x0000000f;
    hw->addr = 0x00000000;
    hw->misc.val = 0x2000003e;
    
    hw->cmd.update = 1;
    while (hw->cmd.update);
    
    hw->cmd.usr = 1;
    
    while ((hw->dma_int_raw.val & 0x00001000) == 0);
    
    uint32_t data_word = hw->data_buf[0];
    
    gpio_set_level(PIN_CS_ADC, 1);
    
    uint8_t lsb = (data_word >> 0) & 0xFF;
    uint8_t msb = (data_word >> 8) & 0xFF;
    
    return ((uint16_t)msb << 8) | lsb;
}
