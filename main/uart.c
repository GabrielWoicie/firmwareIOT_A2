#include "uart.h"
#include "driver/uart.h"
#include "esp_log.h"



// Initialize UART
void uart_init(void) {
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    // Configure the UART driver
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, UART_BUF_SIZE, UART_BUF_SIZE, 0, NULL, 0));

    ESP_LOGI(TAG, "UART initialized on TXD_PIN=%d, RXD_PIN=%d at %d baud", TXD_PIN, RXD_PIN, UART_BAUD_RATE);
}

// Write a variable-size buffer to UART
void uart_write(const uint8_t *data, size_t size) {
    if (data == NULL || size == 0) {
        // ESP_LOGW(TAG, "Attempt to write empty data");
        return;
    }

    int bytes_written = uart_write_bytes(UART_NUM, (const char *)data, size);
    if (bytes_written < 0) {
         ESP_LOGE(TAG, "UART write failed");
    } else {
         ESP_LOGI(TAG, "Wrote %d bytes to UART", bytes_written);
    }
}

// Read data from UART
int uart_read(uint8_t *buffer, size_t size) {
    if (buffer == NULL || size == 0) {
        // ESP_LOGW(TAG, "Attempt to read into an invalid buffer");
        return -1;
    }

    int bytes_read = uart_read_bytes(UART_NUM, buffer, size, pdMS_TO_TICKS(20)); // Timeout of 10 second
    if (bytes_read > 0) {
         ESP_LOGI(TAG, "Read %d bytes from UART", bytes_read);
    } else if (bytes_read == 0) {
        // ESP_LOGW(TAG, "UART read timeout with no data");
    } else {
        // ESP_LOGE(TAG, "UART read error");
    }

    return bytes_read;
}
