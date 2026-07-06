/*
 * LyraT-Mini v1.2 two-codec echo
 *
 * Microphone  → ES7243 ADC → I2S1 (GPIO 0/32/33/36)  ──► ESP32
 * Headphone   ← ES8311 DAC ← I2S0 (GPIO 5/25/26)     ◄── ESP32
 * I2C shared  SDA=GPIO18, SCL=GPIO23
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s_std.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_codec_dev_defaults.h"
#include "esp_codec_dev.h"
#include "es8311_codec.h"
#include "es7243e_adc.h"

static const char *TAG = "mic_echo";

/* Sample config */
#define SAMPLE_RATE     16000
#define MCLK_MULTIPLE   256
#define RECV_BUF_SIZE   2400   /* bytes per I2S read */

/* I2C (shared by both codecs) */
#define I2C_SDA   GPIO_NUM_18
#define I2C_SCL   GPIO_NUM_23

/* I2S0 — ES8311 DAC (headphone output), TX only */
#define I2S0_BCK   GPIO_NUM_5
#define I2S0_WS    GPIO_NUM_25
#define I2S0_DOUT  GPIO_NUM_26

/* I2S1 — ES7243 ADC (microphone input), RX only */
#define I2S1_MCLK  GPIO_NUM_0
#define I2S1_BCK   GPIO_NUM_32
#define I2S1_WS    GPIO_NUM_33
#define I2S1_DIN   GPIO_NUM_36

/* Power amplifier enable */
#define PA_CTRL_IO  GPIO_NUM_21

static i2s_chan_handle_t      tx_handle = NULL;
static i2s_chan_handle_t      rx_handle = NULL;
static esp_codec_dev_handle_t dac_hdl   = NULL;
static esp_codec_dev_handle_t adc_hdl   = NULL;

/* ── I2S init ──────────────────────────────────────────────────────────── */

static esp_err_t i2s_driver_init(void)
{
    /* I2S0: TX only → ES8311 DAC */
    i2s_chan_config_t tx_chan = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    tx_chan.auto_clear = true;
    ESP_ERROR_CHECK(i2s_new_channel(&tx_chan, &tx_handle, NULL));

    i2s_std_config_t tx_std = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                         I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S0_BCK,
            .ws   = I2S0_WS,
            .dout = I2S0_DOUT,
            .din  = I2S_GPIO_UNUSED,
        },
    };
    tx_std.clk_cfg.mclk_multiple = MCLK_MULTIPLE;
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_handle, &tx_std));
    ESP_ERROR_CHECK(i2s_channel_enable(tx_handle));

    /* I2S1: RX only → ES7243 ADC
     * ES7243 requires I2S clock running before es7243_codec_new() is called */
    i2s_chan_config_t rx_chan = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&rx_chan, NULL, &rx_handle));

    i2s_std_config_t rx_std = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                         I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S1_MCLK,
            .bclk = I2S1_BCK,
            .ws   = I2S1_WS,
            .dout = I2S_GPIO_UNUSED,
            .din  = I2S1_DIN,
        },
    };
    rx_std.clk_cfg.mclk_multiple = MCLK_MULTIPLE;
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_handle, &rx_std));
    ESP_ERROR_CHECK(i2s_channel_enable(rx_handle));

    ESP_LOGI(TAG, "I2S0 (ES8311 TX) + I2S1 (ES7243 RX) ready");
    return ESP_OK;
}

/* ── Codec init ─────────────────────────────────────────────────────────── */

static esp_err_t codec_init(void)
{
    /* Shared I2C bus */
    i2c_master_bus_handle_t i2c_bus = NULL;
    i2c_master_bus_config_t i2c_cfg = {
        .i2c_port                     = I2C_NUM_0,
        .sda_io_num                   = I2C_SDA,
        .scl_io_num                   = I2C_SCL,
        .clk_source                   = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt            = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_cfg, &i2c_bus));

    esp_codec_dev_sample_info_t sample = {
        .bits_per_sample = 16,
        .channel         = 2,
        .channel_mask    = 0x03,
        .sample_rate     = SAMPLE_RATE,
    };
    const audio_codec_gpio_if_t *gpio_if = audio_codec_new_gpio();

    /* ── ES8311 DAC (headphone output) ── */
    audio_codec_i2c_cfg_t es8311_i2c = {
        .port       = I2C_NUM_0,
        .addr       = ES8311_CODEC_DEFAULT_ADDR,
        .bus_handle = i2c_bus,
    };
    const audio_codec_ctrl_if_t *es8311_ctrl = audio_codec_new_i2c_ctrl(&es8311_i2c);
    assert(es8311_ctrl);

    audio_codec_i2s_cfg_t i2s0_data = {
        .port      = I2S_NUM_0,
        .tx_handle = tx_handle,
        .rx_handle = NULL,
    };
    const audio_codec_data_if_t *es8311_data = audio_codec_new_i2s_data(&i2s0_data);
    assert(es8311_data);

    es8311_codec_cfg_t es8311_cfg = {
        .ctrl_if     = es8311_ctrl,
        .gpio_if     = gpio_if,
        .codec_mode  = ESP_CODEC_DEV_WORK_MODE_DAC,
        .master_mode = false,
        .use_mclk    = false,   /* ES8311 clocks from BCLK on this board */
        .pa_pin      = PA_CTRL_IO,
        .pa_reverted = false,
        .hw_gain     = {.pa_voltage = 5.0, .codec_dac_voltage = 3.3},
        .mclk_div    = MCLK_MULTIPLE,
    };
    const audio_codec_if_t *es8311_if = es8311_codec_new(&es8311_cfg);
    assert(es8311_if);

    esp_codec_dev_cfg_t dac_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_OUT,
        .codec_if = es8311_if,
        .data_if  = es8311_data,
    };
    dac_hdl = esp_codec_dev_new(&dac_cfg);
    assert(dac_hdl);

    if (esp_codec_dev_open(dac_hdl, &sample) != ESP_CODEC_DEV_OK) {
        ESP_LOGE(TAG, "ES8311 open failed");
        return ESP_FAIL;
    }
    esp_codec_dev_set_out_vol(dac_hdl, 100);
    ESP_LOGI(TAG, "ES8311 DAC init OK — headphone output ready");

    /* ── ES7243 ADC (microphone input) ── */
    audio_codec_i2c_cfg_t es7243_i2c = {
        .port       = I2C_NUM_0,
        .addr       = ES7243E_CODEC_DEFAULT_ADDR,
        .bus_handle = i2c_bus,
    };
    const audio_codec_ctrl_if_t *es7243_ctrl = audio_codec_new_i2c_ctrl(&es7243_i2c);
    assert(es7243_ctrl);

    audio_codec_i2s_cfg_t i2s1_data = {
        .port      = I2S_NUM_1,
        .tx_handle = NULL,
        .rx_handle = rx_handle,
    };
    const audio_codec_data_if_t *es7243_data = audio_codec_new_i2s_data(&i2s1_data);
    assert(es7243_data);

    es7243e_codec_cfg_t es7243_cfg = {
        .ctrl_if = es7243_ctrl,
    };
    const audio_codec_if_t *es7243_if = es7243e_codec_new(&es7243_cfg);
    assert(es7243_if);

    esp_codec_dev_cfg_t adc_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_IN,
        .codec_if = es7243_if,
        .data_if  = es7243_data,
    };
    adc_hdl = esp_codec_dev_new(&adc_cfg);
    assert(adc_hdl);

    if (esp_codec_dev_open(adc_hdl, &sample) != ESP_CODEC_DEV_OK) {
        ESP_LOGE(TAG, "ES7243 open failed");
        return ESP_FAIL;
    }
    esp_codec_dev_set_in_gain(adc_hdl, 6.0);
    ESP_LOGI(TAG, "ES7243E ADC init OK — microphone ready");

    return ESP_OK;
}

/* ── Echo task ──────────────────────────────────────────────────────────── */

static void echo_task(void *args)
{
    int16_t *buf = malloc(RECV_BUF_SIZE);
    assert(buf);
    size_t bytes_read = 0, bytes_written = 0;
    int print_ctr = 0;

    ESP_LOGI(TAG, "Echo running — speak into MIC hole, hear in headphone");

    while (1) {
        memset(buf, 0, RECV_BUF_SIZE);

        /* Read from ES7243 mic (I2S1) */
        i2s_channel_read(rx_handle, buf, RECV_BUF_SIZE, &bytes_read, 1000);

        /* VU meter every 20 reads (~0.5 sec) */
        if (++print_ctr >= 20) {
            print_ctr = 0;
            int count = (int)(bytes_read / sizeof(int16_t));
            int64_t sum = 0;
            for (int i = 0; i < count; i++) {
                int v = buf[i];
                sum += v < 0 ? -v : v;
            }
            int avg = count > 0 ? (int)(sum / count) : 0;
            int bars = (avg * 25) / 32768;
            if (bars > 25) bars = 25;
            char meter[26] = {0};
            for (int i = 0; i < 25; i++) meter[i] = i < bars ? '#' : '.';
            printf("MIC |%s| level=%d\n", meter, avg);
        }

        /* Software gain: 3x so voice (~12000) → ~36000 → clips to full scale */
        int n = (int)(bytes_read / sizeof(int16_t));
        for (int i = 0; i < n; i++) {
            int32_t s = (int32_t)buf[i] * 3;
            if (s >  32767) s =  32767;
            if (s < -32768) s = -32768;
            buf[i] = (int16_t)s;
        }

        /* Write to ES8311 DAC (I2S0) */
        i2s_channel_write(tx_handle, buf, bytes_read, &bytes_written, 1000);
    }
    vTaskDelete(NULL);
}

/* ── Entry point ────────────────────────────────────────────────────────── */

void app_main(void)
{
    ESP_LOGI(TAG, "LyraT-Mini two-codec echo starting");
    ESP_ERROR_CHECK(i2s_driver_init());
    ESP_ERROR_CHECK(codec_init());
    xTaskCreate(echo_task, "echo", 8192, NULL, 5, NULL);
}
