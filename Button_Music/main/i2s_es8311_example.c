#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s_std.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "esp_codec_dev_defaults.h"
#include "esp_codec_dev.h"
#include "es8311_codec.h"

static const char *TAG = "btn_music";

/* ── Audio config ── */
#define SAMPLE_RATE      16000
#define MCLK_MULTIPLE    256
#define CHUNK_SAMPLES    1024
#define CHUNK_BYTES      (CHUNK_SAMPLES * 2 * sizeof(int16_t))

/* ── I2C / I2S pins ── */
#define I2C_SDA     GPIO_NUM_18
#define I2C_SCL     GPIO_NUM_23
#define I2S0_BCK    GPIO_NUM_5
#define I2S0_WS     GPIO_NUM_25
#define I2S0_DOUT   GPIO_NUM_26
#define PA_CTRL_IO  GPIO_NUM_21

/* ── Buttons ──────────────────────────────────────────────────────────────
 * GPIO39 (ADC1_CH3): PLAY and SET share this pin via resistor divider.
 *   - PLAY pressed → pulls almost to GND  → ADC raw < PLAY_THRESH
 *   - SET  pressed → partial pull          → PLAY_THRESH ≤ raw < SET_THRESH
 *   - No button    → pulled high           → raw ≥ SET_THRESH
 *
 * GPIO0  (BOOT button): direct GPIO, active-LOW, internal pull-up.
 * ────────────────────────────────────────────────────────────────────────*/
#define ADC_CHANNEL     ADC_CHANNEL_3   /* GPIO39 */
#define PLAY_THRESH     400             /* ADC raw < this → PLAY pressed  */
#define SET_THRESH      2800            /* ADC raw < this → SET  pressed  */
#define BTN_BOOT        GPIO_NUM_0

/* ── Notes ── */
#define NOTE_C4   262
#define NOTE_D4   294
#define NOTE_E4   330
#define NOTE_F4   349
#define NOTE_G4   392
#define NOTE_A4   440
#define NOTE_REST 0

typedef struct { uint16_t freq; uint16_t ms; } Note;

static const Note song_twinkle[] = {
    {NOTE_C4,350},{NOTE_C4,350},{NOTE_G4,350},{NOTE_G4,350},
    {NOTE_A4,350},{NOTE_A4,350},{NOTE_G4,700},
    {NOTE_F4,350},{NOTE_F4,350},{NOTE_E4,350},{NOTE_E4,350},
    {NOTE_D4,350},{NOTE_D4,350},{NOTE_C4,700},
    {NOTE_G4,350},{NOTE_G4,350},{NOTE_F4,350},{NOTE_F4,350},
    {NOTE_E4,350},{NOTE_E4,350},{NOTE_D4,700},
    {NOTE_G4,350},{NOTE_G4,350},{NOTE_F4,350},{NOTE_F4,350},
    {NOTE_E4,350},{NOTE_E4,350},{NOTE_D4,700},
    {NOTE_C4,350},{NOTE_C4,350},{NOTE_G4,350},{NOTE_G4,350},
    {NOTE_A4,350},{NOTE_A4,350},{NOTE_G4,700},
    {NOTE_F4,350},{NOTE_F4,350},{NOTE_E4,350},{NOTE_E4,350},
    {NOTE_D4,350},{NOTE_D4,350},{NOTE_C4,700},
    {NOTE_REST,0}
};

static const Note song_mary[] = {
    {NOTE_E4,300},{NOTE_D4,300},{NOTE_C4,300},{NOTE_D4,300},
    {NOTE_E4,300},{NOTE_E4,300},{NOTE_E4,600},
    {NOTE_D4,300},{NOTE_D4,300},{NOTE_D4,600},
    {NOTE_E4,300},{NOTE_G4,300},{NOTE_G4,600},
    {NOTE_E4,300},{NOTE_D4,300},{NOTE_C4,300},{NOTE_D4,300},
    {NOTE_E4,300},{NOTE_E4,300},{NOTE_E4,300},{NOTE_E4,300},
    {NOTE_D4,300},{NOTE_D4,300},{NOTE_E4,300},{NOTE_D4,300},
    {NOTE_C4,900},
    {NOTE_REST,0}
};

#define SONG_COUNT 3
static const char *song_names[SONG_COUNT] = {
    "Canon in D", "Twinkle Twinkle", "Mary Had a Little Lamb"
};

extern const uint8_t canon_pcm_start[] asm("_binary_canon_pcm_start");
extern const uint8_t canon_pcm_end[]   asm("_binary_canon_pcm_end");

/* ── State ── */
static i2s_chan_handle_t      tx_handle = NULL;
static esp_codec_dev_handle_t dac_hdl   = NULL;
static volatile bool          is_playing = true;
static volatile int           volume     = 70;
static volatile int           song_num   = 0;
static adc_oneshot_unit_handle_t adc_hdl = NULL;

/* ── Button poll task ─────────────────────────────────────────────────────
 * Polls GPIO39 via ADC every 20ms to detect PLAY vs SET.
 * Acts on button RELEASE (not press) to avoid bounce.
 * ────────────────────────────────────────────────────────────────────────*/
static void button_task(void *arg)
{
    /* 0=none, 1=PLAY, 2=SET */
    int  last_btn    = 0;
    TickType_t press_t = 0;

    while (1) {
        int raw = 0;
        adc_oneshot_read(adc_hdl, ADC_CHANNEL, &raw);

        int btn = 0;
        if      (raw < PLAY_THRESH) btn = 1;   /* PLAY */
        else if (raw < SET_THRESH)  btn = 2;   /* SET  */

        if (btn != 0 && last_btn == 0) {
            /* Press detected */
            press_t  = xTaskGetTickCount();
            last_btn = btn;
            if (btn == 2) printf("DBG: PLAY pressed (raw=%d)\n", raw);
            else          printf("DBG: SET  pressed (raw=%d)\n", raw);

        } else if (btn == 0 && last_btn != 0) {
            /* Release detected — act now */
            TickType_t held = xTaskGetTickCount() - press_t;

            if (last_btn == 2) {
                /* Physical PLAY button (mid raw ~841): toggle play/pause */
                is_playing = !is_playing;
                printf(">> %s\n", is_playing ? "PLAYING" : "PAUSED");

            } else {
                /* Physical SET button (low raw ~315): tap=Vol UP, hold=Vol DOWN */
                if (held > pdMS_TO_TICKS(500)) {
                    volume -= 10;
                    if (volume < 10) volume = 10;
                    printf(">> Vol DOWN -> %d\n", (int)volume);
                } else {
                    volume += 10;
                    if (volume > 100) volume = 100;
                    printf(">> Vol UP   -> %d\n", (int)volume);
                }
                esp_codec_dev_set_out_vol(dac_hdl, volume);
            }
            last_btn = 0;
        }

        /* BOOT button checked here too (GPIO0, active-LOW) */
        if (gpio_get_level(BTN_BOOT) == 0) {
            vTaskDelay(pdMS_TO_TICKS(40));  /* debounce */
            if (gpio_get_level(BTN_BOOT) == 0) {
                song_num = (song_num + 1) % SONG_COUNT;
                printf(">> Song: %s\n", song_names[song_num]);
                /* wait for release */
                while (gpio_get_level(BTN_BOOT) == 0)
                    vTaskDelay(pdMS_TO_TICKS(20));
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/* ── I2S init ── */
static esp_err_t i2s_driver_init(void)
{
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
    return ESP_OK;
}

/* ── Codec init ── */
static esp_err_t codec_init(void)
{
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

    const audio_codec_gpio_if_t *gpio_if = audio_codec_new_gpio();
    es8311_codec_cfg_t es8311_cfg = {
        .ctrl_if     = es8311_ctrl,
        .gpio_if     = gpio_if,
        .codec_mode  = ESP_CODEC_DEV_WORK_MODE_DAC,
        .master_mode = false,
        .use_mclk    = false,
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

    ESP_ERROR_CHECK(esp_codec_dev_open(dac_hdl, &sample) != ESP_CODEC_DEV_OK
                    ? ESP_FAIL : ESP_OK);
    esp_codec_dev_set_out_vol(dac_hdl, volume);
    ESP_LOGI(TAG, "ES8311 DAC ready — volume %d", (int)volume);
    return ESP_OK;
}

/* ── Music playback task ── */
static void play_task(void *arg)
{
    static int16_t buf[CHUNK_SAMPLES * 2];
    static int16_t sil[CHUNK_SAMPLES * 2];
    memset(sil, 0, sizeof(sil));

    size_t bw = 0;
    for (int i = 0; i < 4; i++)
        i2s_channel_write(tx_handle, sil, sizeof(sil), &bw, 100);

    ESP_LOGI(TAG, "Playing: %s", song_names[0]);

    int    cur_song = 0;
    size_t pcm_offset = 0;
    const uint8_t *pcm_data = canon_pcm_start;
    size_t pcm_len = canon_pcm_end - canon_pcm_start;
    int    note_idx = 0;
    int    note_frames_left = 0;
    float  phase = 0.0f;

    while (1) {
        int sn = song_num;
        if (sn != cur_song) {
            cur_song = sn;
            pcm_offset = 0;
            note_idx = 0;
            note_frames_left = 0;
            phase = 0.0f;
            i2s_channel_write(tx_handle, sil, sizeof(sil), &bw, 100);
            ESP_LOGI(TAG, "Now: %s", song_names[cur_song]);
        }

        if (!is_playing) {
            i2s_channel_write(tx_handle, sil, sizeof(sil), &bw, 100);
            continue;
        }

        if (cur_song == 0) {
            /* PCM file playback */
            size_t chunk = pcm_len - pcm_offset;
            if (chunk > CHUNK_BYTES) chunk = CHUNK_BYTES;
            int frames = (int)(chunk / sizeof(int16_t));
            const int16_t *src = (const int16_t *)(pcm_data + pcm_offset);
            for (int i = 0; i < frames; i++) {
                int32_t s = (int32_t)src[i] * 4;
                if (s >  32767) s =  32767;
                if (s < -32768) s = -32768;
                buf[2*i]   = (int16_t)s;
                buf[2*i+1] = (int16_t)s;
            }
            i2s_channel_write(tx_handle, buf, frames * 2 * sizeof(int16_t), &bw, 1000);
            pcm_offset += chunk;
            if (pcm_offset >= pcm_len) {
                i2s_channel_write(tx_handle, sil, sizeof(sil), &bw, 100);
                pcm_offset = 0;
            }

        } else {
            /* Synthesized melody */
            const Note *melody = (cur_song == 1) ? song_twinkle : song_mary;
            if (note_frames_left <= 0) {
                if (melody[note_idx].ms == 0) { note_idx = 0; phase = 0.0f; }
                note_frames_left = (SAMPLE_RATE * melody[note_idx].ms) / 1000;
            }
            int frames = CHUNK_SAMPLES;
            if (frames > note_frames_left) frames = note_frames_left;

            int freq = melody[note_idx].freq;
            if (freq == 0) {
                memset(buf, 0, frames * 2 * sizeof(int16_t));
            } else {
                float inc = (2.0f * 3.14159265f * freq) / SAMPLE_RATE;
                for (int i = 0; i < frames; i++) {
                    int16_t s = (int16_t)(26000.0f * sinf(phase));
                    buf[2*i] = s; buf[2*i+1] = s;
                    phase += inc;
                    if (phase >= 2.0f * 3.14159265f) phase -= 2.0f * 3.14159265f;
                }
            }
            i2s_channel_write(tx_handle, buf, frames * 2 * sizeof(int16_t), &bw, 1000);
            note_frames_left -= frames;
            if (note_frames_left <= 0) {
                note_idx++;
                memset(buf, 0, 64 * 2 * sizeof(int16_t));
                i2s_channel_write(tx_handle, buf, 64 * 2 * sizeof(int16_t), &bw, 50);
            }
        }
    }
}

/* ── Entry point ── */
void app_main(void)
{
    ESP_LOGI(TAG, "Button Music Player starting");
    ESP_LOGI(TAG, "PLAY(tap)=Play/Pause  SET(tap)=VolUP  SET(hold)=VolDOWN  BOOT=NextSong");

    /* ADC for GPIO39 (PLAY + SET via resistor divider) */
    adc_oneshot_unit_init_cfg_t adc_cfg = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc_cfg, &adc_hdl));
    adc_oneshot_chan_cfg_t ch_cfg = {
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_hdl, ADC_CHANNEL, &ch_cfg));

    /* BOOT button: GPIO0 input with pull-up, polled in button_task */
    gpio_config_t boot_conf = {
        .pin_bit_mask = (1ULL << BTN_BOOT),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&boot_conf);

    ESP_ERROR_CHECK(i2s_driver_init());
    ESP_ERROR_CHECK(codec_init());

    xTaskCreate(button_task, "btn",  4096, NULL, 10, NULL);
    xTaskCreate(play_task,   "play", 8192, NULL,  5, NULL);
}
