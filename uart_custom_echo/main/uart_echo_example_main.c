#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#define UART_PORT       UART_NUM_0
#define BUF_SIZE        1024
#define UART_BAUD_RATE  115200

static void uart_echo_task(void *arg)
{
    uint8_t data[BUF_SIZE];

    while (1) {
        int len = uart_read_bytes(UART_PORT, data, BUF_SIZE - 1, 20 / portTICK_PERIOD_MS);

        if (len > 0) {
            for (int i = 0; i < len; i++) {
                uint8_t ch = data[i];

                if (ch >= '0' && ch <= '9') {
                    // digit: add 1, wrap 9 → 0
                    uint8_t result = ((ch - '0') + 1) % 10 + '0';
                    uart_write_bytes(UART_PORT, (char*)&result, 1);
                    printf("\nReceived: %c  →  Sent: %c\n", ch, result);
                } else {
                    // non-digit: echo as-is
                    uart_write_bytes(UART_PORT, (char*)&ch, 1);
                }
            }
        }
    }
}

void app_main(void)
{
    uart_config_t uart_config = {
        .baud_rate  = UART_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
    };

    uart_driver_install(UART_PORT, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_PORT, &uart_config);

    printf("UART ready — type a number to get number+1\n");

    xTaskCreate(uart_echo_task, "uart_echo_task", 2048, NULL, 10, NULL);
}
