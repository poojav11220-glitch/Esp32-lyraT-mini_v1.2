/*
 * Deep Sleep + Wake — ESP32 LyraT-Mini v1.2
 *
 * LEDs:
 *   Green (GPIO22) — ON while awake, OFF during deep sleep
 *   Blue  (GPIO27) — blinks 3x as warning before sleeping
 *
 * Wake source: BOOT button (GPIO0)
 * Wake count : saved in NVS, survives deep sleep and power cuts
 */

#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "driver/gpio.h"

#define TAG         "deep_sleep"
#define WAKE_GPIO   GPIO_NUM_0    /* BOOT button — active LOW        */
#define GREEN_LED   GPIO_NUM_22   /* ON = awake,  OFF = sleeping     */
#define BLUE_LED    GPIO_NUM_27   /* blinks as warning before sleep  */
#define SLEEP_SEC   3             /* seconds awake before sleeping   */

/* ── LED helpers ────────────────────────────────────────────────────────────── */

static void leds_init(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << GREEN_LED) | (1ULL << BLUE_LED),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
    gpio_set_level(GREEN_LED, 0);
    gpio_set_level(BLUE_LED,  0);
}

static void blink(gpio_num_t pin, int times, int on_ms, int off_ms)
{
    for (int i = 0; i < times; i++) {
        gpio_set_level(pin, 1);
        vTaskDelay(pdMS_TO_TICKS(on_ms));
        gpio_set_level(pin, 0);
        vTaskDelay(pdMS_TO_TICKS(off_ms));
    }
}

/* ── NVS helpers ────────────────────────────────────────────────────────────── */

static int32_t nvs_read_count(nvs_handle_t h)
{
    int32_t val = 0;
    esp_err_t err = nvs_get_i32(h, "wake_count", &val);
    if (err == ESP_ERR_NVS_NOT_FOUND) val = 0;
    return val;
}

/* ── app_main ───────────────────────────────────────────────────────────────── */

void app_main(void)
{
    leds_init();

    /* green ON immediately — board is awake */
    gpio_set_level(GREEN_LED, 1);

    /* init NVS */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    /* read, increment, save wake count */
    nvs_handle_t nvs;
    ESP_ERROR_CHECK(nvs_open("storage", NVS_READWRITE, &nvs));
    int32_t count = nvs_read_count(nvs) + 1;
    ESP_ERROR_CHECK(nvs_set_i32(nvs, "wake_count", count));
    ESP_ERROR_CHECK(nvs_commit(nvs));
    nvs_close(nvs);

    /* decode wake reason */
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    const char *reason;
    switch (cause) {
        case ESP_SLEEP_WAKEUP_EXT0:  reason = "BOOT button (GPIO0)"; break;
        case ESP_SLEEP_WAKEUP_TIMER: reason = "Timer expired";        break;
        default:                      reason = "Power-on / RST";      break;
    }

    ESP_LOGI(TAG, "╔══════════════════════════════╗");
    ESP_LOGI(TAG, "  Boot #%" PRId32 "  [GREEN LED ON]", count);
    ESP_LOGI(TAG, "  Wake reason : %s", reason);
    ESP_LOGI(TAG, "  Sleeping in %d sec...", SLEEP_SEC);
    ESP_LOGI(TAG, "╚══════════════════════════════╝");

    /* stay awake — green LED on, readable on serial */
    vTaskDelay(pdMS_TO_TICKS(SLEEP_SEC * 1000));

    /* blue LED blinks 3x as warning — sleep is coming */
    ESP_LOGI(TAG, "Blue LED blinks 3x → going to sleep...");
    blink(BLUE_LED, 3, 150, 150);

    /* green OFF — board sleeps */
    gpio_set_level(GREEN_LED, 0);

    ESP_LOGI(TAG, "Sleeping now. Press BOOT to wake. [BOTH LEDs OFF]");

    /* configure BOOT button as wake source (wakes on LOW) */
    ESP_ERROR_CHECK(esp_sleep_enable_ext0_wakeup(WAKE_GPIO, 0));

    esp_deep_sleep_start();
    /* never returns — wake is a full restart */
}
