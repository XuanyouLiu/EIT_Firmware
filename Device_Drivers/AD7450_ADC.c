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
        .clock_speed_hz = SPI_MASTER_FREQ_8M, // 8 MHz
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
uint16_t AD7450_Read_Direct_Registers(void)
{
    static const char *TAG = "AD7450_Direct";
    // uint32_t start = esp_timer_get_time();
    
    // Get direct access to SPI2 peripheral registers
    spi_dev_t *hw = &GPSPI2;
    
    // ========== CRITICAL: Reconfigure CS as GPIO (once) ==========
    // The SPI driver configured CS under peripheral control, but for direct register
    // access we need to control it as a standard GPIO
    static bool cs_gpio_configured = false;
    if (!cs_gpio_configured) {
        gpio_reset_pin(PIN_CS_ADC);  // Reset to GPIO mode
        gpio_set_direction(PIN_CS_ADC, GPIO_MODE_OUTPUT);
        gpio_set_level(PIN_CS_ADC, 1);  // Idle high
        cs_gpio_configured = true;
        
        // Small delay to stabilize
        for (volatile int i = 0; i < 100; i++);
    }
    
    // Assert CS LOW (active)
    gpio_set_level(PIN_CS_ADC, 0);
    
    // 1. spi_ll_clear_int_stat: Clear interrupt status
    hw->dma_int_raw.val = 0x00000000;
    
    // 2. spi_ll_get_running_cmd: Check if command is running (read-only check)
    // Ensure no transaction is running before starting new one
    while (hw->cmd.val != 0) {
        // Wait for any previous command to finish
    }
    
    // 3. spi_ll_master_set_line_mode: Set to single-line mode (full-duplex)
    hw->ctrl.val = 0x00340000;
    hw->user.val = 0x18000241;  // usr_mosi=1, usr_miso=1
    
    // 4-5. Dummy and delay config (already in user.val, no changes needed)
    
    // 6 & 7. Set data length to 16 bits (hardware stores as bitlen-1)
    hw->ms_dlen.val = 0x0000000f;  // 15 = (16 bits - 1)
    
    // 8-10. Address and command phases disabled (already in user.val)
    
    // 11. Set address register (unused but write 0 for clean state)
    hw->addr = 0x00000000;
    
    // 12. CS keep-active configuration
    hw->misc.val = 0x2000003e;  // cs_keep_active = 0
    
    // 13-14. MOSI/MISO already enabled in step 3 user.val
    
    // 15. Apply config (no update needed, registers already set)
    
    // 16. Start SPI transaction by setting usr bit
    hw->cmd.usr = 1;  // Hardware auto-clears when transaction begins
    
    // Small delay to ensure cmd.usr write takes effect
    asm volatile("nop; nop; nop; nop;");
    asm volatile("nop; nop; nop; nop;");
    asm volatile("nop; nop; nop; nop;");
    asm volatile("nop; nop; nop; nop;");
    
    // 17. Poll for transaction completion (wait for trans_done bit)
    while ((hw->dma_int_raw.val & 0x00001000) == 0) {
        // Spin until bit 12 (trans_done) is set
    }
    
    // 18. Read result from SPI data buffer
    uint32_t data_word = hw->data_buf[0];
    
    // ========== Deassert CS ==========
    gpio_set_level(PIN_CS_ADC, 1);  // CS HIGH (inactive)
    
    // Extract 16-bit result from 32-bit data buffer
    // data_buf layout: [byte0][byte1][byte2][byte3] = bits[7:0][15:8][23:16][31:24]
    uint8_t byte0 = (data_word >> 0) & 0xFF;
    uint8_t byte1 = (data_word >> 8) & 0xFF;
    
    // Assemble as MSB-first 16-bit value (matching AD7450_Read format)
    uint16_t result = ((uint16_t)byte0 << 8) | byte1;
    
    // uint32_t end = esp_timer_get_time();
    // ESP_LOGI(TAG, "AD7450_Read_Direct_Registers took %lu us", end - start);
    
    return result;
}