/*
 * Stopwatch — DS1302 RTC + ESP32 LyraT-Mini v1.2
 *
 * DS1302 wiring (same as Real_time / rtc_alarm):
 *   DS1302 CLK  → TCLK (GPIO13)
 *   DS1302 DAT  → TDO  (GPIO15)
 *   DS1302 RST  → TMS  (GPIO14)
 *
 * BOOT button (GPIO0) controls:
 *   Short press  (<1.5s) → START / STOP toggle
 *   Long press   (≥1.5s) → RESET to 00:00:00.0
 *
 * LEDs:
 *   Green LED → ON while stopwatch is RUNNING
 *   Blue LED  → blinks once on every button press (feedback)
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "rom/ets_sys.h"

#define TAG  "SW"

/* ── DS1302 pins ─────────────────────────────────────────────────────────── */
#define PIN_CLK   GPIO_NUM_13
#define PIN_DAT   GPIO_NUM_15
#define PIN_RST   GPIO_NUM_14

/* ── Board GPIOs ─────────────────────────────────────────────────────────── */
#define GREEN_LED   GPIO_NUM_22
#define BLUE_LED    GPIO_NUM_27
#define BOOT_BTN    GPIO_NUM_0    /* active LOW */

#define LONG_PRESS_MS   1500      /* hold ≥ 1.5s = reset */
#define DELAY()  ets_delay_us(2)

/* ── DS1302 registers ────────────────────────────────────────────────────── */
#define REG_SEC   0x80
#define REG_MIN   0x82
#define REG_HOUR  0x84
#define REG_DATE  0x86
#define REG_MON   0x88
#define REG_YEAR  0x8C
#define REG_WP    0x8E

/* ── BCD helpers ─────────────────────────────────────────────────────────── */
static uint8_t bcd2dec(uint8_t b) { return (b >> 4) * 10 + (b & 0x0F); }

/* ── DAT direction ───────────────────────────────────────────────────────── */
static void dat_out(void) { gpio_set_direction(PIN_DAT, GPIO_MODE_OUTPUT); }
static void dat_in(void)  {
    gpio_set_direction(PIN_DAT, GPIO_MODE_INPUT);
    gpio_set_pull_mode(PIN_DAT, GPIO_PULLUP_ONLY);
}

/* ── DS1302 byte I/O ─────────────────────────────────────────────────────── */
static void write_byte(uint8_t byte)
{
    dat_out();
    for (int i = 0; i < 8; i++) {
        gpio_set_level(PIN_DAT, (byte >> i) & 1);
        DELAY();
        gpio_set_level(PIN_CLK, 1); DELAY();
        gpio_set_level(PIN_CLK, 0); DELAY();
    }
}

static uint8_t read_byte(void)
{
    dat_in();
    uint8_t val = 0;
    for (int i = 0; i < 8; i++) {
        DELAY();
        if (gpio_get_level(PIN_DAT)) val |= (1 << i);
        gpio_set_level(PIN_CLK, 1); DELAY();
        gpio_set_level(PIN_CLK, 0);
    }
    return val;
}

static uint8_t ds1302_read(uint8_t reg)
{
    gpio_set_level(PIN_RST, 1); DELAY();
    write_byte(reg | 0x01);
    uint8_t val = read_byte();
    gpio_set_level(PIN_RST, 0); DELAY();
    return val;
}

/* ── Read wall-clock time from DS1302 ────────────────────────────────────── */
typedef struct { uint8_t hour, min, sec; } rtc_time_t;

static rtc_time_t rtc_now(void)
{
    rtc_time_t t;
    t.sec  = bcd2dec(ds1302_read(REG_SEC)  & 0x7F);
    t.min  = bcd2dec(ds1302_read(REG_MIN));
    t.hour = bcd2dec(ds1302_read(REG_HOUR) & 0x3F);
    return t;
}

/* ── GPIO init ───────────────────────────────────────────────────────────── */
static void hw_init(void)
{
    gpio_config_t ds = {
        .pin_bit_mask = (1ULL << PIN_CLK) | (1ULL << PIN_RST),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&ds);
    gpio_set_level(PIN_CLK, 0);
    gpio_set_level(PIN_RST, 0);
    dat_out();
    gpio_set_level(PIN_DAT, 0);

    gpio_reset_pin(GREEN_LED);
    gpio_reset_pin(BLUE_LED);
    gpio_config_t led = {
        .pin_bit_mask = (1ULL << GREEN_LED) | (1ULL << BLUE_LED),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&led);
    gpio_set_level(GREEN_LED, 0);
    gpio_set_level(BLUE_LED,  0);

    gpio_config_t btn = {
        .pin_bit_mask = (1ULL << BOOT_BTN),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&btn);
}

/* ── Blue LED blink — button press feedback ──────────────────────────────── */
static void led_blink_blue(void)
{
    gpio_set_level(BLUE_LED, 1);
    vTaskDelay(pdMS_TO_TICKS(80));
    gpio_set_level(BLUE_LED, 0);
}

/* ── Detect button press: returns hold duration in ms ───────────────────── */
static int button_wait_press(void)
{
    /* Wait for press (falling edge) */
    while (gpio_get_level(BOOT_BTN) != 0)
        vTaskDelay(pdMS_TO_TICKS(10));

    int64_t press_start = esp_timer_get_time();

    /* Wait for release (rising edge) */
    while (gpio_get_level(BOOT_BTN) == 0)
        vTaskDelay(pdMS_TO_TICKS(10));

    int held_ms = (int)((esp_timer_get_time() - press_start) / 1000);
    vTaskDelay(pdMS_TO_TICKS(50));   /* debounce */
    return held_ms;
}

/* ── Format elapsed microseconds → HH:MM:SS.t ───────────────────────────── */
static void format_elapsed(int64_t us, char *buf)
{
    int64_t total_ms = us / 1000;
    int tenths  = (total_ms / 100) % 10;
    int seconds = (total_ms / 1000) % 60;
    int minutes = (total_ms / 60000) % 60;
    int hours   = (int)(total_ms / 3600000);
    sprintf(buf, "%02d:%02d:%02d.%d", hours, minutes, seconds, tenths);
}

/* ── app_main ────────────────────────────────────────────────────────────── */
void app_main(void)
{
    hw_init();

    ESP_LOGI(TAG, "Stopwatch ready");
    ESP_LOGI(TAG, "Short press BOOT = START / STOP");
    ESP_LOGI(TAG, "Long  press BOOT = RESET");

    typedef enum { STOPPED, RUNNING } sw_state_t;
    sw_state_t state    = STOPPED;
    int64_t    start_us = 0;      /* esp_timer value when started    */
    int64_t    accum_us = 0;      /* accumulated time before pause   */
    char       elapsed[16];

    while (1) {
        /* ── Button handling (non-blocking poll) ── */
        if (gpio_get_level(BOOT_BTN) == 0) {
            int held = button_wait_press();
            led_blink_blue();

            if (held >= LONG_PRESS_MS) {
                /* RESET */
                state    = STOPPED;
                accum_us = 0;
                gpio_set_level(GREEN_LED, 0);
                ESP_LOGW(TAG, "RESET  → 00:00:00.0");

            } else if (state == STOPPED) {
                /* START */
                state    = RUNNING;
                start_us = esp_timer_get_time();
                gpio_set_level(GREEN_LED, 1);
                ESP_LOGI(TAG, "START");

            } else {
                /* STOP */
                accum_us += esp_timer_get_time() - start_us;
                state     = STOPPED;
                gpio_set_level(GREEN_LED, 0);
                format_elapsed(accum_us, elapsed);
                ESP_LOGW(TAG, "STOP   → %s", elapsed);
            }
        }

        /* ── Live display while running (every 100ms) ── */
        if (state == RUNNING) {
            int64_t total = accum_us + (esp_timer_get_time() - start_us);
            format_elapsed(total, elapsed);
            rtc_time_t t = rtc_now();
            ESP_LOGI(TAG, "[RUNNING]  %s     RTC %02d:%02d:%02d",
                     elapsed, t.hour, t.min, t.sec);
            vTaskDelay(pdMS_TO_TICKS(100));
        } else {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
}
