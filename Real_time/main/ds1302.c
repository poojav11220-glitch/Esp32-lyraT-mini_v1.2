/*
 * DS1302 Real-Time Clock — ESP32 LyraT-Mini v1.2
 *
 * Wiring:
 *   DS1302 VCC  → 3V3
 *   DS1302 GND  → GND
 *   DS1302 CLK  → TCLK (GPIO13)
 *   DS1302 DAT  → TDO  (GPIO15)
 *   DS1302 RST  → TMS  (GPIO14)
 *
 * Auto time-set logic:
 *   First boot / battery dead  → sets time from compile timestamp
 *   Battery running            → reads saved time directly (no reset)
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "rom/ets_sys.h"

#define TAG      "RTC"

/* ── GPIO pins (JTAG header) ─────────────────────────────────────────────── */
#define PIN_CLK  GPIO_NUM_13   /* TCLK */
#define PIN_DAT  GPIO_NUM_15   /* TDO  */
#define PIN_RST  GPIO_NUM_14   /* TMS  */

/* ── Set to 1 to force-correct the time, then set back to 0 and reflash ─── */
#define FORCE_SET_TIME  1

/* ── Correct time — update these values, flash once, then set FORCE_SET=0 ── */
#define SET_HOUR   10
#define SET_MIN    05
#define SET_SEC    00
#define SET_DATE   04
#define SET_MONTH  07
#define SET_YEAR   26

#define DELAY()  ets_delay_us(2)

/* ── DS1302 register write addresses ─────────────────────────────────────── */
#define REG_SEC   0x80   /* bit7 = CH (Clock Halt) */
#define REG_MIN   0x82
#define REG_HOUR  0x84
#define REG_DATE  0x86
#define REG_MON   0x88
#define REG_DAY   0x8A
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

/* ── Byte I/O (LSB first) ────────────────────────────────────────────────── */
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

/* ── Register access ─────────────────────────────────────────────────────── */
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
static void ds1302_gpio_init(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << PIN_CLK) | (1ULL << PIN_RST),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
    gpio_set_level(PIN_CLK, 0);
    gpio_set_level(PIN_RST, 0);
    dat_out();
    gpio_set_level(PIN_DAT, 0);
}

/* ── Parse compile-time date/time ────────────────────────────────────────── */
/* __TIME__ = "HH:MM:SS"     e.g. "10:05:42"           */
/* __DATE__ = "Mmm DD YYYY"  e.g. "Jul  4 2026"        */

static uint8_t compile_hour(void) {
    return (__TIME__[0]-'0')*10 + (__TIME__[1]-'0');
}
static uint8_t compile_min(void) {
    return (__TIME__[3]-'0')*10 + (__TIME__[4]-'0');
}
static uint8_t compile_sec(void) {
    return (__TIME__[6]-'0')*10 + (__TIME__[7]-'0');
}
static uint8_t compile_month(void) {
    const char *mn = "JanFebMarAprMayJunJulAugSepOctNovDec";
    for (int i = 0; i < 12; i++)
        if (mn[i*3]==__DATE__[0] && mn[i*3+1]==__DATE__[1] && mn[i*3+2]==__DATE__[2])
            return i + 1;
    return 1;
}
static uint8_t compile_date(void) {
    /* __DATE__[4] is ' ' or digit, __DATE__[5] is digit */
    uint8_t tens = (__DATE__[4] == ' ') ? 0 : (__DATE__[4] - '0');
    return tens * 10 + (__DATE__[5] - '0');
}
static uint8_t compile_year(void) {
    /* last 2 digits of year: __DATE__[9] and [10] */
    return (__DATE__[9]-'0')*10 + (__DATE__[10]-'0');
}

/* ── Set time ────────────────────────────────────────────────────────────── */
static void ds1302_set_time(uint8_t hour, uint8_t min, uint8_t sec,
                             uint8_t date, uint8_t month, uint8_t year)
{
    ds1302_write(REG_WP,   0x00);
    ds1302_write(REG_SEC,  dec2bcd(sec));   /* writing clears CH bit → starts clock */
    ds1302_write(REG_MIN,  dec2bcd(min));
    ds1302_write(REG_HOUR, dec2bcd(hour));
    ds1302_write(REG_DATE, dec2bcd(date));
    ds1302_write(REG_MON,  dec2bcd(month));
    ds1302_write(REG_YEAR, dec2bcd(year));
    ds1302_write(REG_WP,   0x80);
}

/* ── Read time ───────────────────────────────────────────────────────────── */
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

/* ── app_main ────────────────────────────────────────────────────────────── */
void app_main(void)
{
    ds1302_gpio_init();

    /* Check CH (Clock Halt) bit — set when chip is new or battery died */
    uint8_t raw_sec = ds1302_read(REG_SEC);
    bool clock_halted = (raw_sec & 0x80) != 0;

#if FORCE_SET_TIME
    /* Force-correct the time — set FORCE_SET_TIME=0 and reflash after this */
    ds1302_set_time(SET_HOUR, SET_MIN, SET_SEC, SET_DATE, SET_MONTH, SET_YEAR);
    ESP_LOGW(TAG, "FORCE SET: 20%02d-%02d-%02d %02d:%02d:%02d",
             SET_YEAR, SET_MONTH, SET_DATE, SET_HOUR, SET_MIN, SET_SEC);
    ESP_LOGW(TAG, "Now set FORCE_SET_TIME=0 in code and reflash!");
#else
    if (clock_halted) {
        /* First boot or dead battery — auto-set from compile timestamp */
        uint8_t h = compile_hour();
        uint8_t m = compile_min();
        uint8_t s = compile_sec();
        uint8_t d = compile_date();
        uint8_t mo = compile_month();
        uint8_t y = compile_year();
        ds1302_set_time(h, m, s, d, mo, y);
        ESP_LOGW(TAG, "Clock halted — set from compile time: 20%02d-%02d-%02d %02d:%02d:%02d",
                 y, mo, d, h, m, s);
    } else {
        ESP_LOGI(TAG, "Battery OK — reading saved time");
    }
#endif

    ESP_LOGI(TAG, "DS1302 RTC running");

    while (1) {
        rtc_time_t t = ds1302_get_time();
        ESP_LOGI(TAG, "20%02d-%02d-%02d   %02d:%02d:%02d",
                 t.year, t.month, t.date,
                 t.hour, t.min, t.sec);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
