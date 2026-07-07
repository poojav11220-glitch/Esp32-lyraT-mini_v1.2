/*
 * RTC Alarm — DS1302 + ESP32 LyraT-Mini v1.2
 *
 * DS1302 wiring (same as Real_time project):
 *   DS1302 CLK  → TCLK (GPIO13)
 *   DS1302 DAT  → TDO  (GPIO15)
 *   DS1302 RST  → TMS  (GPIO14)
 *
 * Behaviour:
 *   - Reads time from DS1302 every second
 *   - When time matches ALARM_HOUR:ALARM_MIN → Green LED blinks fast + alert
 *   - Press BOOT button (GPIO0) to dismiss alarm
 *   - Blue LED blinks once every 5s as heartbeat (clock is running)
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "rom/ets_sys.h"

#define TAG  "ALARM"

/* ── Set your alarm time here (24-hour format) ───────────────────────────── */
#define ALARM_HOUR   14    /* alarm at 14:25 */
#define ALARM_MIN    25

/* ── Force-set RTC time on flash (set to 0 after first flash) ────────────── */
#define FORCE_SET_TIME  1
#define SET_HOUR   14
#define SET_MIN    20
#define SET_SEC    00
#define SET_DATE   06
#define SET_MONTH  07
#define SET_YEAR   26

/* ── DS1302 pins (JTAG header) ───────────────────────────────────────────── */
#define PIN_CLK   GPIO_NUM_13
#define PIN_DAT   GPIO_NUM_15
#define PIN_RST   GPIO_NUM_14

/* ── Board GPIOs ─────────────────────────────────────────────────────────── */
#define GREEN_LED   GPIO_NUM_22
#define BLUE_LED    GPIO_NUM_27
#define BOOT_BTN    GPIO_NUM_0    /* active LOW */

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
static uint8_t dec2bcd(uint8_t d) { return ((d / 10) << 4) | (d % 10);  }

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

static void ds1302_write(uint8_t reg, uint8_t val)
{
    gpio_set_level(PIN_RST, 1); DELAY();
    write_byte(reg);
    write_byte(val);
    gpio_set_level(PIN_RST, 0); DELAY();
}

static uint8_t ds1302_read(uint8_t reg)
{
    gpio_set_level(PIN_RST, 1); DELAY();
    write_byte(reg | 0x01);
    uint8_t val = read_byte();
    gpio_set_level(PIN_RST, 0); DELAY();
    return val;
}

/* ── GPIO init ───────────────────────────────────────────────────────────── */
static void hw_init(void)
{
    /* DS1302 CLK + RST as output */
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

    /* LEDs as output */
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

    /* BOOT button as input with pull-up */
    gpio_config_t btn = {
        .pin_bit_mask = (1ULL << BOOT_BTN),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&btn);
}

/* ── Time struct ─────────────────────────────────────────────────────────── */
typedef struct { uint8_t hour, min, sec, date, month, year; } rtc_time_t;

static rtc_time_t ds1302_get_time(void)
{
    rtc_time_t t;
    t.sec   = bcd2dec(ds1302_read(REG_SEC)  & 0x7F);
    t.min   = bcd2dec(ds1302_read(REG_MIN));
    t.hour  = bcd2dec(ds1302_read(REG_HOUR) & 0x3F);
    t.date  = bcd2dec(ds1302_read(REG_DATE));
    t.month = bcd2dec(ds1302_read(REG_MON));
    t.year  = bcd2dec(ds1302_read(REG_YEAR));
    return t;
}

/* ── Set RTC time ────────────────────────────────────────────────────────── */
static void ds1302_set_time(uint8_t h, uint8_t m, uint8_t s,
                             uint8_t d, uint8_t mo, uint8_t y)
{
    ds1302_write(REG_WP,   0x00);
    ds1302_write(REG_SEC,  dec2bcd(s));
    ds1302_write(REG_MIN,  dec2bcd(m));
    ds1302_write(REG_HOUR, dec2bcd(h));
    ds1302_write(REG_DATE, dec2bcd(d));
    ds1302_write(REG_MON,  dec2bcd(mo));
    ds1302_write(REG_YEAR, dec2bcd(y));
    ds1302_write(REG_WP,   0x80);
}

/* ── Ensure clock is running; force-set if FORCE_SET_TIME=1 ─────────────── */
static void ds1302_ensure_running(void)
{
#if FORCE_SET_TIME
    ds1302_set_time(SET_HOUR, SET_MIN, SET_SEC, SET_DATE, SET_MONTH, SET_YEAR);
    ESP_LOGW(TAG, "Time force-set to 20%02d-%02d-%02d %02d:%02d:%02d",
             SET_YEAR, SET_MONTH, SET_DATE, SET_HOUR, SET_MIN, SET_SEC);
    ESP_LOGW(TAG, "Set FORCE_SET_TIME=0 and reflash to stop overwriting time");
#else
    uint8_t raw = ds1302_read(REG_SEC);
    if (raw & 0x80) {
        /* Clock halted (new chip / dead battery) — set from compile time */
        uint8_t h  = (__TIME__[0]-'0')*10 + (__TIME__[1]-'0');
        uint8_t m  = (__TIME__[3]-'0')*10 + (__TIME__[4]-'0');
        uint8_t s  = (__TIME__[6]-'0')*10 + (__TIME__[7]-'0');
        uint8_t d  = (__DATE__[4]==' ') ? (__DATE__[5]-'0') :
                     (__DATE__[4]-'0')*10 + (__DATE__[5]-'0');
        const char *mn = "JanFebMarAprMayJunJulAugSepOctNovDec";
        uint8_t mo = 1;
        for (int i = 0; i < 12; i++)
            if (mn[i*3]==__DATE__[0] && mn[i*3+1]==__DATE__[1] && mn[i*3+2]==__DATE__[2])
                { mo = i + 1; break; }
        uint8_t y = (__DATE__[9]-'0')*10 + (__DATE__[10]-'0');
        ds1302_set_time(h, m, s, d, mo, y);
        ESP_LOGW(TAG, "Clock halted — set from compile time");
    } else {
        ESP_LOGI(TAG, "Battery OK — using saved time");
    }
#endif
}

/* ── Alarm ring: blink green LED until BOOT button pressed ──────────────── */
static void alarm_ring(void)
{
    ESP_LOGW(TAG, "╔══════════════════════════╗");
    ESP_LOGW(TAG, "║   *** ALARM RINGING ***  ║");
    ESP_LOGW(TAG, "║  Press BOOT to dismiss   ║");
    ESP_LOGW(TAG, "╚══════════════════════════╝");

    while (gpio_get_level(BOOT_BTN) != 0) {   /* wait for BOOT press */
        gpio_set_level(GREEN_LED, 1);
        vTaskDelay(pdMS_TO_TICKS(150));
        gpio_set_level(GREEN_LED, 0);
        vTaskDelay(pdMS_TO_TICKS(150));
    }

    gpio_set_level(GREEN_LED, 0);
    ESP_LOGI(TAG, "Alarm dismissed");
    vTaskDelay(pdMS_TO_TICKS(500));   /* debounce button */
}

/* ── app_main ────────────────────────────────────────────────────────────── */
void app_main(void)
{
    hw_init();
    ds1302_ensure_running();

    ESP_LOGI(TAG, "RTC Alarm ready");
    ESP_LOGI(TAG, "Alarm set for %02d:%02d", ALARM_HOUR, ALARM_MIN);
    ESP_LOGI(TAG, "Press BOOT button to dismiss alarm when it rings");

    int heartbeat = 0;
    bool alarm_fired = false;   /* prevent re-triggering within same minute */

    while (1) {
        rtc_time_t t = ds1302_get_time();

        ESP_LOGI(TAG, "20%02d-%02d-%02d  %02d:%02d:%02d  [alarm %02d:%02d]",
                 t.year, t.month, t.date,
                 t.hour, t.min, t.sec,
                 ALARM_HOUR, ALARM_MIN);

        /* Trigger alarm at exact minute, second 0 */
        if (t.hour == ALARM_HOUR && t.min == ALARM_MIN && t.sec == 0 && !alarm_fired) {
            alarm_fired = true;
            alarm_ring();
        }

        /* Reset fired flag once minute has passed */
        if (t.min != ALARM_MIN) {
            alarm_fired = false;
        }

        /* Blue LED heartbeat every 5 seconds */
        heartbeat++;
        if (heartbeat >= 5) {
            heartbeat = 0;
            gpio_set_level(BLUE_LED, 1);
            vTaskDelay(pdMS_TO_TICKS(100));
            gpio_set_level(BLUE_LED, 0);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
