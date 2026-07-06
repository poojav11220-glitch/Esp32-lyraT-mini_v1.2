/*
 * Bluetooth A2DP Sink — ESP32 LyraT-Mini v1.2
 *
 * Phone connects to "LyraT Mini Speaker" via Bluetooth.
 * Audio flows: Phone → A2DP (SBC) → stream buffer → I2S → ES8311 → headphone jack.
 * AVRCP target lets the phone control volume remotely.
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"

#include "nvs_flash.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2s_std.h"
#include "driver/i2c_master.h"

/* Bluetooth */
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"

/* Codec — include device header directly so it is always available */
#include "esp_codec_dev.h"
#include "esp_codec_dev_defaults.h"
#include "es8311_codec.h"

/* ── constants ──────────────────────────────────────────────────────────────── */

#define TAG             "bt_speaker"
#define DEVICE_NAME     "LyraT Mini Speaker"
#define DEFAULT_VOLUME  70

/* hardware pins (LyraT-Mini v1.2) */
#define I2C_SDA     GPIO_NUM_18
#define I2C_SCL     GPIO_NUM_23
#define I2S_BCK     GPIO_NUM_5
#define I2S_WS      GPIO_NUM_25
#define I2S_DOUT    GPIO_NUM_26
#define PA_CTRL     GPIO_NUM_21

/* audio pipeline */
#define STREAM_BUF_SIZE  (32 * 1024)   /* 32 KB ≈ 180 ms at 44100 Hz stereo 16-bit */
#define I2S_READ_SIZE    4096

/* ── module state ───────────────────────────────────────────────────────────── */

static i2s_chan_handle_t      tx_handle    = NULL;
static esp_codec_dev_handle_t dac_hdl      = NULL;
static StreamBufferHandle_t   audio_buf    = NULL;
static int                    current_rate = 44100;

/* ── I2S init ───────────────────────────────────────────────────────────────── */

static void i2s_init(int sample_rate)
{
    i2s_chan_config_t ch = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    ch.auto_clear = true;
    ESP_ERROR_CHECK(i2s_new_channel(&ch, &tx_handle, NULL));

    i2s_std_config_t std = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
                        I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_BCK,
            .ws   = I2S_WS,
            .dout = I2S_DOUT,
            .din  = I2S_GPIO_UNUSED,
        },
    };
    std.clk_cfg.mclk_multiple = 256;

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_handle, &std));
    ESP_ERROR_CHECK(i2s_channel_enable(tx_handle));
}

/* ── Codec init ─────────────────────────────────────────────────────────────── */

static void codec_init(int sample_rate)
{
    /* enable headphone power amplifier — we manage this pin ourselves */
    gpio_config_t pa_io = {
        .pin_bit_mask = BIT64(PA_CTRL),
        .mode         = GPIO_MODE_OUTPUT,
    };
    gpio_config(&pa_io);
    gpio_set_level(PA_CTRL, 1);

    /* I2C master bus */
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port          = I2C_NUM_0,
        .sda_io_num        = I2C_SDA,
        .scl_io_num        = I2C_SCL,
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t i2c_bus;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &i2c_bus));

    /* I2C *control* interface — used to read/write ES8311 registers */
    audio_codec_i2c_cfg_t i2c_cfg = {
        .port       = I2C_NUM_0,
        .addr       = ES8311_CODEC_DEFAULT_ADDR,
        .bus_handle = i2c_bus,
    };
    const audio_codec_ctrl_if_t *i2c_ctrl = audio_codec_new_i2c_ctrl(&i2c_cfg);

    /* ES8311 codec — pa_pin=-1 because we drive PA_CTRL manually above */
    es8311_codec_cfg_t es_cfg = {
        .ctrl_if     = i2c_ctrl,
        .gpio_if     = NULL,
        .codec_mode  = ESP_CODEC_DEV_WORK_MODE_DAC,
        .pa_pin      = -1,
        .pa_reverted = false,
        .master_mode = false,
        .use_mclk    = false,
        .digital_mic = false,
    };
    const audio_codec_if_t *es_if = es8311_codec_new(&es_cfg);

    /* I2S *data* interface — carries PCM samples to/from I2S peripheral */
    audio_codec_i2s_cfg_t i2s_cfg = {
        .port      = I2S_NUM_0,
        .rx_handle = NULL,
        .tx_handle = tx_handle,
    };
    const audio_codec_data_if_t *i2s_if = audio_codec_new_i2s_data(&i2s_cfg);

    /* assemble codec device */
    esp_codec_dev_cfg_t dev_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_OUT,
        .codec_if = es_if,
        .data_if  = i2s_if,
    };
    dac_hdl = esp_codec_dev_new(&dev_cfg);

    esp_codec_dev_sample_info_t fs = {
        .sample_rate     = sample_rate,
        .channel         = 2,
        .bits_per_sample = 16,
    };
    ESP_ERROR_CHECK(esp_codec_dev_open(dac_hdl, &fs));
    ESP_ERROR_CHECK(esp_codec_dev_set_out_vol(dac_hdl, DEFAULT_VOLUME));
}

/* ── dynamic sample-rate update ─────────────────────────────────────────────── */

static void apply_sample_rate(int rate)
{
    if (rate == current_rate) return;
    current_rate = rate;
    ESP_LOGI(TAG, "Sample rate → %d Hz", rate);

    i2s_channel_disable(tx_handle);
    i2s_std_clk_config_t clk = I2S_STD_CLK_DEFAULT_CONFIG(rate);
    clk.mclk_multiple = 256;
    i2s_channel_reconfig_std_clock(tx_handle, &clk);
    i2s_channel_enable(tx_handle);

    if (dac_hdl) {
        esp_codec_dev_sample_info_t fs = {
            .sample_rate     = rate,
            .channel         = 2,
            .bits_per_sample = 16,
        };
        esp_codec_dev_close(dac_hdl);
        esp_codec_dev_open(dac_hdl, &fs);
        esp_codec_dev_set_out_vol(dac_hdl, DEFAULT_VOLUME);
    }
}

/* ── I2S playback task ──────────────────────────────────────────────────────── */

static void i2s_task(void *arg)
{
    static uint8_t buf[I2S_READ_SIZE];
    static uint8_t silence[512];
    memset(silence, 0, sizeof(silence));
    size_t bw = 0;

    /* pre-fill DMA buffers to avoid click on first audio */
    for (int i = 0; i < 8; i++)
        i2s_channel_write(tx_handle, silence, sizeof(silence), &bw, 50);

    while (1) {
        size_t got = xStreamBufferReceive(audio_buf, buf, sizeof(buf),
                                          pdMS_TO_TICKS(20));
        if (got > 0) {
            i2s_channel_write(tx_handle, buf, got, &bw, portMAX_DELAY);
        } else {
            /* keep I2S clock running during silence — prevents glitch on resume */
            i2s_channel_write(tx_handle, silence, sizeof(silence), &bw, 10);
        }
    }
}

/* ── A2DP callbacks ─────────────────────────────────────────────────────────── */

/* called from BT task — push audio into stream buffer, I2S task drains it */
static void a2d_data_cb(const uint8_t *data, uint32_t len)
{
    if (audio_buf && len > 0)
        xStreamBufferSend(audio_buf, data, len, 0);
}

static void a2d_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param)
{
    switch (event) {

    case ESP_A2D_CONNECTION_STATE_EVT:
        if (param->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
            ESP_LOGI(TAG, "Phone connected! Play music now.");
            /* stop advertising so a second phone cannot interrupt */
            esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE,
                                     ESP_BT_NON_DISCOVERABLE);
        } else if (param->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
            ESP_LOGI(TAG, "Phone disconnected — waiting for connection...");
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE,
                                     ESP_BT_GENERAL_DISCOVERABLE);
        }
        break;

    case ESP_A2D_AUDIO_STATE_EVT:
        ESP_LOGI(TAG, "Audio %s",
                 param->audio_stat.state == ESP_A2D_AUDIO_STATE_STARTED
                 ? "streaming" : "stopped");
        break;

    case ESP_A2D_AUDIO_CFG_EVT: {
        /* ESP_A2D_SBC_CIE_SF_* bitmasks defined in esp_a2dp_api.h */
        uint8_t sf = param->audio_cfg.mcc.cie.sbc_info.samp_freq;
        int rate = 16000;
        if      (sf & ESP_A2D_SBC_CIE_SF_44K) rate = 44100;
        else if (sf & ESP_A2D_SBC_CIE_SF_48K) rate = 48000;
        else if (sf & ESP_A2D_SBC_CIE_SF_32K) rate = 32000;
        apply_sample_rate(rate);
        break;
    }

    default:
        break;
    }
}

/* ── GAP callback ───────────────────────────────────────────────────────────── */

static void gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_BT_GAP_AUTH_CMPL_EVT:
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS)
            ESP_LOGI(TAG, "Paired with: %s", param->auth_cmpl.device_name);
        else
            ESP_LOGW(TAG, "Pairing failed (%d)", param->auth_cmpl.stat);
        break;
    case ESP_BT_GAP_PIN_REQ_EVT: {
        /* legacy PIN fallback */
        ESP_LOGI(TAG, "PIN request — using 0000");
        esp_bt_pin_code_t pin = {'0', '0', '0', '0'};
        esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin);
        break;
    }
    default:
        break;
    }
}

/* ── AVRCP target callback ──────────────────────────────────────────────────── */

static void avrc_tg_cb(esp_avrc_tg_cb_event_t event,
                        esp_avrc_tg_cb_param_t *param)
{
    if (event == ESP_AVRC_TG_SET_ABSOLUTE_VOLUME_CMD_EVT && dac_hdl) {
        /* phone sends 0–127, codec expects 0–100 */
        int vol = (int)param->set_abs_vol.volume * 100 / 127;
        esp_codec_dev_set_out_vol(dac_hdl, vol);
        ESP_LOGI(TAG, "Volume → %d%%", vol);
    }
}

/* ── app_main ───────────────────────────────────────────────────────────────── */

void app_main(void)
{
    /* NVS is required by Bluetooth to store pairing info across reboots */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    /* stream buffer decouples the BT task (producer) from I2S task (consumer) */
    audio_buf = xStreamBufferCreate(STREAM_BUF_SIZE, 512);
    configASSERT(audio_buf);

    /* audio hardware */
    i2s_init(current_rate);
    codec_init(current_rate);

    /* Bluetooth controller — Classic BT only saves ~40 KB vs dual-mode */
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT));

    /* Bluedroid stack */
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    /* GAP — set name and enable "Just Works" SSP (no PIN dialog on phone) */
    ESP_ERROR_CHECK(esp_bt_gap_register_callback(gap_cb));
    esp_bt_gap_set_device_name(DEVICE_NAME);
    uint8_t iocap = ESP_BT_IO_CAP_NONE;
    esp_bt_gap_set_security_param(ESP_BT_SP_IOCAP_MODE, &iocap, sizeof(iocap));

    /* A2DP sink */
    ESP_ERROR_CHECK(esp_a2d_register_callback(a2d_cb));
    ESP_ERROR_CHECK(esp_a2d_sink_register_data_callback(a2d_data_cb));
    ESP_ERROR_CHECK(esp_a2d_sink_init());

    /* AVRCP target — lets phone send volume/play/pause commands */
    ESP_ERROR_CHECK(esp_avrc_tg_init());
    ESP_ERROR_CHECK(esp_avrc_tg_register_callback(avrc_tg_cb));

    /* start advertising */
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);

    /* I2S playback task — stack 4096 is enough since all buffers are static */
    xTaskCreate(i2s_task, "i2s", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    ESP_LOGI(TAG, "  Bluetooth Speaker Ready!");
    ESP_LOGI(TAG, "  Connect phone to: \"%s\"", DEVICE_NAME);
    ESP_LOGI(TAG, "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
}
