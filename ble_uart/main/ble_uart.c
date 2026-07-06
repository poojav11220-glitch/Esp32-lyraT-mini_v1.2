/*
 * BLE UART Terminal — ESP32 LyraT-Mini v1.2
 *
 * Implements Nordic UART Service (NUS) as a BLE GATT server.
 *
 * Connect with phone app:
 *   Android — "Serial Bluetooth Terminal" or "nRF Connect"
 *   iPhone  — "nRF Connect" or "LightBlue"
 *
 * Flow:
 *   Phone writes text → RX characteristic → ESP32 parses command
 *   ESP32 sends reply → TX characteristic (notify) → Phone displays
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_gatt_common_api.h"
#include "esp_system.h"
#include "esp_chip_info.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"

/* ── constants ──────────────────────────────────────────────────────────────── */

#define TAG          "ble_uart"
#define DEVICE_NAME  "LyraT BLE UART"
#define MAX_DATA_LEN 512
#define CHAR_DECL_SIZE sizeof(uint8_t)

/*
 * Nordic UART Service UUIDs — 128-bit, stored little-endian.
 * Service: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
 * RX char: 6E400002-B5A3-F393-E0A9-E50E24DCCA9E  (Phone → ESP32, Write)
 * TX char: 6E400003-B5A3-F393-E0A9-E50E24DCCA9E  (ESP32 → Phone, Notify)
 */
static uint8_t NUS_SVC_UUID[16]    = {0x9e,0xca,0xdc,0x24,0x0e,0xe5,0xa9,0xe0,0x93,0xf3,0xa3,0xb5,0x01,0x00,0x40,0x6e};
static uint8_t NUS_RX_CHAR_UUID[16]= {0x9e,0xca,0xdc,0x24,0x0e,0xe5,0xa9,0xe0,0x93,0xf3,0xa3,0xb5,0x02,0x00,0x40,0x6e};
static uint8_t NUS_TX_CHAR_UUID[16]= {0x9e,0xca,0xdc,0x24,0x0e,0xe5,0xa9,0xe0,0x93,0xf3,0xa3,0xb5,0x03,0x00,0x40,0x6e};

/* standard BLE declaration UUIDs (16-bit) */
static const uint16_t uuid_primary_svc   = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t uuid_char_decl     = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t uuid_char_cfg      = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

/* characteristic properties */
static const uint8_t prop_write  = ESP_GATT_CHAR_PROP_BIT_WRITE |
                                   ESP_GATT_CHAR_PROP_BIT_WRITE_NR;
static const uint8_t prop_notify = ESP_GATT_CHAR_PROP_BIT_NOTIFY;

/* CCCD: phone writes 0x0001 here to enable notifications */
static uint16_t cccd_val = 0x0000;

/* ── GATT attribute table ───────────────────────────────────────────────────── */

enum {
    IDX_SVC,           /* service declaration */
    IDX_CHAR_RX,       /* RX characteristic declaration */
    IDX_CHAR_VAL_RX,   /* RX characteristic value (phone writes here) */
    IDX_CHAR_TX,       /* TX characteristic declaration */
    IDX_CHAR_VAL_TX,   /* TX characteristic value (ESP32 notifies here) */
    IDX_CHAR_CFG_TX,   /* TX CCCD — phone writes 0x0001 to enable notify */
    ATTR_IDX_NB,
};

static const esp_gatts_attr_db_t gatt_db[ATTR_IDX_NB] = {

    /* Service declaration — value is the 128-bit NUS service UUID */
    [IDX_SVC] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&uuid_primary_svc, ESP_GATT_PERM_READ,
         ESP_UUID_LEN_128, ESP_UUID_LEN_128, NUS_SVC_UUID}
    },

    /* RX characteristic declaration */
    [IDX_CHAR_RX] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&uuid_char_decl, ESP_GATT_PERM_READ,
         CHAR_DECL_SIZE, CHAR_DECL_SIZE, (uint8_t *)&prop_write}
    },

    /* RX characteristic value — phone writes command text here */
    [IDX_CHAR_VAL_RX] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_128, NUS_RX_CHAR_UUID, ESP_GATT_PERM_WRITE,
         MAX_DATA_LEN, 0, NULL}
    },

    /* TX characteristic declaration */
    [IDX_CHAR_TX] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&uuid_char_decl, ESP_GATT_PERM_READ,
         CHAR_DECL_SIZE, CHAR_DECL_SIZE, (uint8_t *)&prop_notify}
    },

    /* TX characteristic value — ESP32 sends responses here via notify */
    [IDX_CHAR_VAL_TX] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_128, NUS_TX_CHAR_UUID, ESP_GATT_PERM_READ,
         MAX_DATA_LEN, 0, NULL}
    },

    /* TX CCCD — phone writes 0x0001 to subscribe to notifications */
    [IDX_CHAR_CFG_TX] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&uuid_char_cfg,
         ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
         sizeof(uint16_t), sizeof(cccd_val), (uint8_t *)&cccd_val}
    },
};

/* ── advertising data ───────────────────────────────────────────────────────── */

/* primary advert: name + flags */
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp        = false,
    .include_name        = true,
    .include_txpower     = true,
    .min_interval        = 0x0006,
    .max_interval        = 0x0010,
    .appearance          = 0x00,
    .manufacturer_len    = 0,
    .p_manufacturer_data = NULL,
    .service_data_len    = 0,
    .p_service_data      = NULL,
    .service_uuid_len    = 0,
    .p_service_uuid      = NULL,
    .flag                = ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT,
};

/* scan response: full 128-bit NUS service UUID so apps can filter by service */
static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp        = true,
    .include_name        = false,
    .include_txpower     = false,
    .service_uuid_len    = sizeof(NUS_SVC_UUID),
    .p_service_uuid      = NUS_SVC_UUID,
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min       = 0x20,
    .adv_int_max       = 0x40,
    .adv_type          = ADV_TYPE_IND,
    .own_addr_type     = BLE_ADDR_TYPE_PUBLIC,
    .channel_map       = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

/* ── runtime state ──────────────────────────────────────────────────────────── */

static uint16_t      handle_table[ATTR_IDX_NB];
static esp_gatt_if_t gatts_if_g    = ESP_GATT_IF_NONE;
static uint16_t      conn_id_g     = 0xFFFF;
static bool          connected     = false;
static bool          notify_on     = false;
/* track which adv configs are done before starting advertising */
static uint8_t       adv_cfg_done  = 0;
#define ADV_CFG_BIT      (1 << 0)
#define SCAN_RSP_CFG_BIT (1 << 1)

/* ── send notification to phone ─────────────────────────────────────────────── */

static void ble_send(const char *text)
{
    if (!connected || !notify_on) return;
    size_t len = strlen(text);
    if (len > MAX_DATA_LEN) len = MAX_DATA_LEN;
    esp_ble_gatts_send_indicate(gatts_if_g, conn_id_g,
                                handle_table[IDX_CHAR_VAL_TX],
                                (uint16_t)len, (uint8_t *)text, false);
}

/* ── command parser ─────────────────────────────────────────────────────────── */

static void handle_command(const uint8_t *data, uint16_t len)
{
    char cmd[MAX_DATA_LEN + 1];
    if (len >= sizeof(cmd)) len = sizeof(cmd) - 1;
    memcpy(cmd, data, len);

    /* strip trailing CR/LF that phone apps sometimes append */
    while (len > 0 && (cmd[len - 1] == '\n' || cmd[len - 1] == '\r'))
        len--;
    cmd[len] = '\0';

    ESP_LOGI(TAG, "CMD: \"%s\"", cmd);

    char resp[MAX_DATA_LEN + 64];

    if (strcmp(cmd, "hello") == 0) {
        ble_send("Hello from LyraT Mini!\r\n");

    } else if (strcmp(cmd, "ping") == 0) {
        ble_send("pong\r\n");

    } else if (strcmp(cmd, "uptime") == 0) {
        int64_t us   = esp_timer_get_time();
        int64_t secs = us / 1000000;
        int     h    = (int)(secs / 3600);
        int     m    = (int)((secs % 3600) / 60);
        int     s    = (int)(secs % 60);
        snprintf(resp, sizeof(resp), "Uptime: %dh %dm %ds\r\n", h, m, s);
        ble_send(resp);

    } else if (strcmp(cmd, "mem") == 0) {
        uint32_t free_heap = esp_get_free_heap_size();
        uint32_t min_heap  = esp_get_minimum_free_heap_size();
        snprintf(resp, sizeof(resp),
                 "Free heap : %lu bytes\r\nMin heap  : %lu bytes\r\n",
                 (unsigned long)free_heap, (unsigned long)min_heap);
        ble_send(resp);

    } else if (strcmp(cmd, "chip") == 0) {
        esp_chip_info_t info;
        esp_chip_info(&info);
        const char *model = "ESP32";
        switch (info.model) {
            case CHIP_ESP32S2: model = "ESP32-S2"; break;
            case CHIP_ESP32S3: model = "ESP32-S3"; break;
            case CHIP_ESP32C3: model = "ESP32-C3"; break;
            default: model = "ESP32"; break;
        }
        snprintf(resp, sizeof(resp),
                 "Chip  : %s rev%d\r\nCores : %d\r\nFlash : %s\r\n",
                 model, info.revision, info.cores,
                 (info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
        ble_send(resp);

    } else if (strcmp(cmd, "status") == 0) {
        ble_send("Board  : ESP32 LyraT-Mini v1.2\r\n");
        ble_send("IDF    : v6.0.2\r\n");
        ble_send("Service: Nordic UART (NUS)\r\n");

    } else if (strcmp(cmd, "reset") == 0) {
        ble_send("Restarting board...\r\n");
        vTaskDelay(pdMS_TO_TICKS(300));  /* let the notify flush */
        esp_restart();

    } else if (strcmp(cmd, "help") == 0) {
        ble_send("--- Commands ---\r\n");
        ble_send("  hello          greet the board\r\n");
        ble_send("  ping           check connection\r\n");
        ble_send("  uptime         time since boot\r\n");
        ble_send("  mem            free heap memory\r\n");
        ble_send("  chip           chip model & info\r\n");
        ble_send("  status         board / IDF info\r\n");
        ble_send("  echo <text>    echo text back\r\n");
        ble_send("  repeat <n> <t> send <t> n times\r\n");
        ble_send("  reset          restart the board\r\n");
        ble_send("  help           show this list\r\n");

    } else if (strncmp(cmd, "echo ", 5) == 0) {
        snprintf(resp, sizeof(resp), "%s\r\n", cmd + 5);
        ble_send(resp);

    } else if (strncmp(cmd, "repeat ", 7) == 0) {
        /* repeat <count> <text> */
        char *p     = cmd + 7;
        int   count = (int)strtol(p, &p, 10);
        if (*p == ' ') p++;          /* skip space between count and text */
        if (count < 1)  count = 1;
        if (count > 20) count = 20;  /* cap to avoid flooding */
        for (int i = 0; i < count; i++) {
            snprintf(resp, sizeof(resp), "[%d] %s\r\n", i + 1, p);
            ble_send(resp);
            vTaskDelay(pdMS_TO_TICKS(50));   /* small gap between notifies */
        }

    } else if (len == 0) {
        /* ignore blank lines */

    } else {
        snprintf(resp, sizeof(resp), "Unknown: '%s'. Type 'help'\r\n", cmd);
        ble_send(resp);
    }
}

/* ── GATT server callback ───────────────────────────────────────────────────── */

static void gatts_cb(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
                     esp_ble_gatts_cb_param_t *param)
{
    switch (event) {

    case ESP_GATTS_REG_EVT:
        gatts_if_g = gatts_if;
        adv_cfg_done = ADV_CFG_BIT | SCAN_RSP_CFG_BIT;
        esp_ble_gap_set_device_name(DEVICE_NAME);
        esp_ble_gap_config_adv_data(&adv_data);
        esp_ble_gap_config_adv_data(&scan_rsp_data);
        esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, ATTR_IDX_NB, 0);
        break;

    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
        if (param->add_attr_tab.status == ESP_GATT_OK &&
            param->add_attr_tab.num_handle == ATTR_IDX_NB) {
            memcpy(handle_table, param->add_attr_tab.handles, sizeof(handle_table));
            esp_ble_gatts_start_service(handle_table[IDX_SVC]);
            ESP_LOGI(TAG, "GATT table created — service started");
        } else {
            ESP_LOGE(TAG, "Attribute table creation failed: status=%d handles=%d",
                     param->add_attr_tab.status, param->add_attr_tab.num_handle);
        }
        break;

    case ESP_GATTS_CONNECT_EVT:
        conn_id_g  = param->connect.conn_id;
        connected  = true;
        notify_on  = false;
        ESP_LOGI(TAG, "Phone connected — enable notifications in app");
        /* request lower latency for snappier responses */
        esp_ble_conn_update_params_t p = {
            .min_int = 0x10, .max_int = 0x20,
            .latency = 0,    .timeout = 400,
        };
        memcpy(p.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        esp_ble_gap_update_conn_params(&p);
        break;

    case ESP_GATTS_DISCONNECT_EVT:
        connected = false;
        notify_on = false;
        ESP_LOGI(TAG, "Phone disconnected — restarting advertising");
        esp_ble_gap_start_advertising(&adv_params);
        break;

    case ESP_GATTS_WRITE_EVT:
        if (param->write.is_prep) break;   /* ignore long-write prepare phase */

        if (param->write.handle == handle_table[IDX_CHAR_CFG_TX] &&
            param->write.len == 2) {
            /* phone toggled notifications via CCCD */
            uint16_t cccd = (param->write.value[1] << 8) | param->write.value[0];
            notify_on = (cccd == 0x0001);
            ESP_LOGI(TAG, "Notifications %s", notify_on ? "ON" : "OFF");
            if (notify_on)
                ble_send("Ready! Type 'help' for commands.\r\n");

        } else if (param->write.handle == handle_table[IDX_CHAR_VAL_RX]) {
            /* phone sent a command */
            handle_command(param->write.value, param->write.len);
        }
        break;

    default:
        break;
    }
}

/* ── GAP callback ───────────────────────────────────────────────────────────── */

static void gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        adv_cfg_done &= ~ADV_CFG_BIT;
        if (adv_cfg_done == 0) esp_ble_gap_start_advertising(&adv_params);
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        adv_cfg_done &= ~SCAN_RSP_CFG_BIT;
        if (adv_cfg_done == 0) esp_ble_gap_start_advertising(&adv_params);
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        ESP_LOGI(TAG, "Advertising started");
        break;
    default:
        break;
    }
}

/* ── app_main ───────────────────────────────────────────────────────────────── */

void app_main(void)
{
    /* NVS required by Bluetooth */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    /* release Classic BT memory — we only need BLE */
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));

    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gatts_cb));
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_cb));
    ESP_ERROR_CHECK(esp_ble_gatts_app_register(0));

    /* allow larger MTU for longer messages */
    ESP_ERROR_CHECK(esp_ble_gatt_set_local_mtu(512));

    ESP_LOGI(TAG, "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    ESP_LOGI(TAG, "  BLE UART Ready!");
    ESP_LOGI(TAG, "  Device name : \"%s\"", DEVICE_NAME);
    ESP_LOGI(TAG, "  Phone app   : nRF Connect / Serial BT Terminal");
    ESP_LOGI(TAG, "  Service     : Nordic UART Service (NUS)");
    ESP_LOGI(TAG, "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
}
