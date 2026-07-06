#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

#define BUTTON_GPIO     GPIO_NUM_0    // Boot button
#define LED_GPIO        GPIO_NUM_22   // Green LED

static QueueHandle_t gpio_evt_queue = NULL;

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void button_task(void* arg)
{
    uint32_t io_num;
    uint8_t led_state = 0;

    for (;;) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            vTaskDelay(50 / portTICK_PERIOD_MS);  // debounce
            if (gpio_get_level(BUTTON_GPIO) == 0) {
                led_state = !led_state;
                gpio_set_level(LED_GPIO, led_state);
                printf("Button pressed! LED = %s\n", led_state ? "ON" : "OFF");
            }
        }
    }
}

void app_main(void)
{
    // LED output config
    gpio_config_t led_conf = {
        .pin_bit_mask = (1ULL << LED_GPIO),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&led_conf);

    // Button input config
    gpio_config_t btn_conf = {
        .pin_bit_mask = (1ULL << BUTTON_GPIO),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&btn_conf);

    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(button_task, "button_task", 2048, NULL, 10, NULL);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_GPIO, gpio_isr_handler, (void*) BUTTON_GPIO);

    printf("Ready — press Boot button to toggle LED\n");
}