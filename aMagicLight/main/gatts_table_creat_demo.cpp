/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/****************************************************************************
 *
 * This demo showcases creating a GATT database using a predefined attribute table.
 * It acts as a GATT server and can send adv data, be connected by client.
 * Run the gatt_client demo, the client demo will automatically connect to the gatt_server_service_table demo.
 * Client demo will enable GATT server's notify after connection. The two devices will then exchange
 * data.
 *
 ****************************************************************************/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "gatts_table_creat_demo.h"
#include "esp_gatt_common_api.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "freertos/queue.h"
#include <stdio.h>
#include <iostream>
#include <time.h>

#define GATTS_TABLE_TAG "GATTS_TABLE_DEMO"

#define PROFILE_NUM 1
#define PROFILE_APP_IDX 0
#define ESP_APP_ID 0x55
#define SAMPLE_DEVICE_NAME "ESP_GATTS_DEMO"
#define SVC_INST_ID 0

/* The max length of characteristic value. When the GATT client performs a write or prepare write operation,
 *  the data length must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
 */
#define GATTS_DEMO_CHAR_VAL_LEN_MAX 500
#define PREPARE_BUF_MAX_SIZE 1024
#define CHAR_DECLARATION_SIZE (sizeof(uint8_t))

#define ADV_CONFIG_FLAG (1 << 0)
#define SCAN_RSP_CONFIG_FLAG (1 << 1)

static uint8_t adv_config_done = 0;

uint16_t heart_rate_handle_table[HRS_IDX_NB];

typedef struct
{
    uint8_t *prepare_buf;
    int prepare_len;
} prepare_type_env_t;

static prepare_type_env_t prepare_write_env;

#define CONFIG_SET_RAW_ADV_DATA
#ifdef CONFIG_SET_RAW_ADV_DATA
static uint8_t raw_adv_data[] = {
    /* flags */
    0x02,
    0x01,
    0x06,
    /* tx power*/
    0x02,
    0x0a,
    0xeb,
    /* service uuid */
    0x03,
    0x03,
    0xFF,
    0x00,
    /* device name */
    0x0C,
    0x09,
    'a',
    'M',
    'a',
    'g',
    'i',
    'c',
    'L',
    'i',
    'g',
    'h',
    't',
};
static uint8_t raw_scan_rsp_data[] = {
    /* flags */
    0x02, 0x01, 0x06,
    /* tx power */
    0x02, 0x0a, 0xeb,
    /* service uuid */
    0x03, 0x03, 0xFF, 0x00};

#else
static uint8_t service_uuid[16] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    // first uuid, 16bit, [12],[13] is the value
    0xfb,
    0x34,
    0x9b,
    0x5f,
    0x80,
    0x00,
    0x00,
    0x80,
    0x00,
    0x10,
    0x00,
    0x00,
    0xFF,
    0x00,
    0x00,
    0x00,
};

/* The length of adv data must be less than 31 bytes */
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006, // slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0010, // slave connection max interval, Time = max_interval * 1.25 msec
    .appearance = 0x00,
    .manufacturer_len = 0,       // TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL, // test_manufacturer,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(service_uuid),
    .p_service_uuid = service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

// scan response data
static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp = true,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006,
    .max_interval = 0x0010,
    .appearance = 0x00,
    .manufacturer_len = 0,       // TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL, //&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(service_uuid),
    .p_service_uuid = service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};
#endif /* CONFIG_SET_RAW_ADV_DATA */

static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

struct gatts_profile_inst
{
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
                                        esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst heart_rate_profile_tab[PROFILE_NUM] = {
    [PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE, /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

/* Service */
static const uint16_t GATTS_SERVICE_UUID_TEST = 0x00FF;
static const uint16_t GATTS_CHAR_UUID_TEST_A = 0xFF01;
static const uint16_t GATTS_CHAR_UUID_TEST_B = 0xFF02;
static const uint16_t GATTS_CHAR_UUID_TEST_C = 0xFF03;

static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint8_t char_prop_read = ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t char_prop_write = ESP_GATT_CHAR_PROP_BIT_WRITE;
static const uint8_t char_prop_read_write_notify = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t heart_measurement_ccc[2] = {0x00, 0x00};
static const uint8_t char_value[4] = {0x11, 0x22, 0x33, 0x44};

/* Full Database Description - Used to add attributes into the database */
static const esp_gatts_attr_db_t gatt_db[HRS_IDX_NB] =
    {
        // Service Declaration
        [IDX_SVC] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ, sizeof(uint16_t), sizeof(GATTS_SERVICE_UUID_TEST), (uint8_t *)&GATTS_SERVICE_UUID_TEST}},

        /* Characteristic Declaration */
        [IDX_CHAR_A] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},

        /* Characteristic Value */
        [IDX_CHAR_VAL_A] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEST_A, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},

        /* Client Characteristic Configuration Descriptor */
        [IDX_CHAR_CFG_A] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(heart_measurement_ccc), (uint8_t *)heart_measurement_ccc}},

        /* Characteristic Declaration */
        [IDX_CHAR_B] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read}},

        /* Characteristic Value */
        [IDX_CHAR_VAL_B] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEST_B, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},

        /* Characteristic Declaration */
        [IDX_CHAR_C] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_write}},

        /* Characteristic Value */
        [IDX_CHAR_VAL_C] =
            {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEST_C, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},

};

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event)
    {
#ifdef CONFIG_SET_RAW_ADV_DATA
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
        adv_config_done &= (~ADV_CONFIG_FLAG);
        if (adv_config_done == 0)
        {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
        adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
        if (adv_config_done == 0)
        {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
#else
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~ADV_CONFIG_FLAG);
        if (adv_config_done == 0)
        {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
        if (adv_config_done == 0)
        {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
#endif
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        /* advertising start complete event to indicate advertising start successfully or failed */
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "advertising start failed");
        }
        else
        {
            ESP_LOGI(GATTS_TABLE_TAG, "advertising start successfully");
        }
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "Advertising stop failed");
        }
        else
        {
            ESP_LOGI(GATTS_TABLE_TAG, "Stop adv successfully\n");
        }
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
        ESP_LOGI(GATTS_TABLE_TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                 param->update_conn_params.status,
                 param->update_conn_params.min_int,
                 param->update_conn_params.max_int,
                 param->update_conn_params.conn_int,
                 param->update_conn_params.latency,
                 param->update_conn_params.timeout);
        break;
    default:
        break;
    }
}

void example_prepare_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGI(GATTS_TABLE_TAG, "prepare write, handle = %d, value len = %d", param->write.handle, param->write.len);
    esp_gatt_status_t status = ESP_GATT_OK;
    if (prepare_write_env->prepare_buf == NULL)
    {
        prepare_write_env->prepare_buf = (uint8_t *)malloc(PREPARE_BUF_MAX_SIZE * sizeof(uint8_t));
        prepare_write_env->prepare_len = 0;
        if (prepare_write_env->prepare_buf == NULL)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "%s, Gatt_server prep no mem", __func__);
            status = ESP_GATT_NO_RESOURCES;
        }
    }
    else
    {
        if (param->write.offset > PREPARE_BUF_MAX_SIZE)
        {
            status = ESP_GATT_INVALID_OFFSET;
        }
        else if ((param->write.offset + param->write.len) > PREPARE_BUF_MAX_SIZE)
        {
            status = ESP_GATT_INVALID_ATTR_LEN;
        }
    }
    /*send response when param->write.need_rsp is true */
    if (param->write.need_rsp)
    {
        esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));
        if (gatt_rsp != NULL)
        {
            gatt_rsp->attr_value.len = param->write.len;
            gatt_rsp->attr_value.handle = param->write.handle;
            gatt_rsp->attr_value.offset = param->write.offset;
            gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
            memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);
            esp_err_t response_err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, gatt_rsp);
            if (response_err != ESP_OK)
            {
                ESP_LOGE(GATTS_TABLE_TAG, "Send response error");
            }
            free(gatt_rsp);
        }
        else
        {
            ESP_LOGE(GATTS_TABLE_TAG, "%s, malloc failed", __func__);
        }
    }
    if (status != ESP_GATT_OK)
    {
        return;
    }
    memcpy(prepare_write_env->prepare_buf + param->write.offset,
           param->write.value,
           param->write.len);
    prepare_write_env->prepare_len += param->write.len;
}

void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param)
{
    if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC && prepare_write_env->prepare_buf)
    {
        esp_log_buffer_hex(GATTS_TABLE_TAG, prepare_write_env->prepare_buf, prepare_write_env->prepare_len);
    }
    else
    {
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATT_PREP_WRITE_CANCEL");
    }
    if (prepare_write_env->prepare_buf)
    {
        free(prepare_write_env->prepare_buf);
        prepare_write_env->prepare_buf = NULL;
    }
    prepare_write_env->prepare_len = 0;
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event)
    {
    case ESP_GATTS_REG_EVT:
    {
        esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(SAMPLE_DEVICE_NAME);
        if (set_dev_name_ret)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "set device name failed, error code = %x", set_dev_name_ret);
        }
#ifdef CONFIG_SET_RAW_ADV_DATA
        esp_err_t raw_adv_ret = esp_ble_gap_config_adv_data_raw(raw_adv_data, sizeof(raw_adv_data));
        if (raw_adv_ret)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "config raw adv data failed, error code = %x ", raw_adv_ret);
        }
        adv_config_done |= ADV_CONFIG_FLAG;
        esp_err_t raw_scan_ret = esp_ble_gap_config_scan_rsp_data_raw(raw_scan_rsp_data, sizeof(raw_scan_rsp_data));
        if (raw_scan_ret)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "config raw scan rsp data failed, error code = %x", raw_scan_ret);
        }
        adv_config_done |= SCAN_RSP_CONFIG_FLAG;
#else
        // config adv data
        esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
        if (ret)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "config adv data failed, error code = %x", ret);
        }
        adv_config_done |= ADV_CONFIG_FLAG;
        // config scan response data
        ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
        if (ret)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "config scan response data failed, error code = %x", ret);
        }
        adv_config_done |= SCAN_RSP_CONFIG_FLAG;
#endif
        esp_err_t create_attr_ret = esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, HRS_IDX_NB, SVC_INST_ID);
        if (create_attr_ret)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "create attr table failed, error code = %x", create_attr_ret);
        }
    }
    break;
    case ESP_GATTS_READ_EVT:
    {
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_READ_EVT");
        break;
    }
    case ESP_GATTS_WRITE_EVT:
    {
        if (!param->write.is_prep)
        {
            // the data length of gattc write  must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
            ESP_LOGI(GATTS_TABLE_TAG, "GATT_WRITE_EVT, handle = %d, value len = %d, value :", param->write.handle, param->write.len);
            esp_log_buffer_hex(GATTS_TABLE_TAG, param->write.value, param->write.len);
            if (heart_rate_handle_table[IDX_CHAR_CFG_A] == param->write.handle && param->write.len == 2)
            {
                uint16_t descr_value = param->write.value[1] << 8 | param->write.value[0];
                if (descr_value == 0x0001)
                {
                    ESP_LOGI(GATTS_TABLE_TAG, "notify enable");
                    uint8_t notify_data[15];
                    for (int i = 0; i < sizeof(notify_data); ++i)
                    {
                        notify_data[i] = i % 0xff;
                    }
                    // the size of notify_data[] need less than MTU size
                    esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, heart_rate_handle_table[IDX_CHAR_VAL_A],
                                                sizeof(notify_data), notify_data, false);
                }
                else if (descr_value == 0x0002)
                {
                    ESP_LOGI(GATTS_TABLE_TAG, "indicate enable");
                    uint8_t indicate_data[15];
                    for (int i = 0; i < sizeof(indicate_data); ++i)
                    {
                        indicate_data[i] = i % 0xff;
                    }
                    // the size of indicate_data[] need less than MTU size
                    esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, heart_rate_handle_table[IDX_CHAR_VAL_A],
                                                sizeof(indicate_data), indicate_data, true);
                }
                else if (descr_value == 0x0000)
                {
                    ESP_LOGI(GATTS_TABLE_TAG, "notify/indicate disable ");
                }
                else
                {
                    ESP_LOGE(GATTS_TABLE_TAG, "unknown descr value");
                    esp_log_buffer_hex(GATTS_TABLE_TAG, param->write.value, param->write.len);
                }
            }
            /* send response when param->write.need_rsp is true*/
            if (param->write.need_rsp)
            {
                esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
            }
        }
        else
        {
            /* handle prepare write */
            example_prepare_write_event_env(gatts_if, &prepare_write_env, param);
        }
        break;
    }
    case ESP_GATTS_EXEC_WRITE_EVT:
    {
        // the length of gattc prepare write data must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_EXEC_WRITE_EVT");
        example_exec_write_event_env(&prepare_write_env, param);
        break;
    }
    case ESP_GATTS_MTU_EVT:
    {
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
        break;
    }
    case ESP_GATTS_CONF_EVT:
    {
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONF_EVT, status = %d, attr_handle %d", param->conf.status, param->conf.handle);
        break;
    }
    case ESP_GATTS_START_EVT:
    {
        ESP_LOGI(GATTS_TABLE_TAG, "SERVICE_START_EVT, status %d, service_handle %d", param->start.status, param->start.service_handle);
        break;
    }
    case ESP_GATTS_CONNECT_EVT:
    {
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONNECT_EVT, conn_id = %d", param->connect.conn_id);
        esp_log_buffer_hex(GATTS_TABLE_TAG, param->connect.remote_bda, 6);
        esp_ble_conn_update_params_t conn_params = {0};
        memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        /* For the iOS system, please refer to Apple official documents about the BLE connection parameters restrictions. */
        conn_params.latency = 0;
        conn_params.max_int = 0x20; // max_int = 0x20*1.25ms = 40ms
        conn_params.min_int = 0x10; // min_int = 0x10*1.25ms = 20ms
        conn_params.timeout = 400;  // timeout = 400*10ms = 4000ms
        // start sent the update connection parameters to the peer device.
        esp_ble_gap_update_conn_params(&conn_params);
        break;
    }
    case ESP_GATTS_DISCONNECT_EVT:
    {
        ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_DISCONNECT_EVT, reason = 0x%x", param->disconnect.reason);
        esp_ble_gap_start_advertising(&adv_params);
        break;
    }
    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
    {
        if (param->add_attr_tab.status != ESP_GATT_OK)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "create attribute table failed, error code=0x%x", param->add_attr_tab.status);
        }
        else if (param->add_attr_tab.num_handle != HRS_IDX_NB)
        {
            ESP_LOGE(GATTS_TABLE_TAG, "create attribute table abnormally, num_handle (%d) \
                        doesn't equal to HRS_IDX_NB(%d)",
                     param->add_attr_tab.num_handle, HRS_IDX_NB);
        }
        else
        {
            ESP_LOGI(GATTS_TABLE_TAG, "create attribute table successfully, the number handle = %d\n", param->add_attr_tab.num_handle);
            memcpy(heart_rate_handle_table, param->add_attr_tab.handles, sizeof(heart_rate_handle_table));
            esp_ble_gatts_start_service(heart_rate_handle_table[IDX_SVC]);
        }
        break;
    }
    case ESP_GATTS_STOP_EVT:
        break;
    case ESP_GATTS_OPEN_EVT:
        break;
    case ESP_GATTS_CANCEL_OPEN_EVT:
        break;
    case ESP_GATTS_CLOSE_EVT:
        break;
    case ESP_GATTS_LISTEN_EVT:
        break;
    case ESP_GATTS_CONGEST_EVT:
        break;
    case ESP_GATTS_UNREG_EVT:
        break;
    case ESP_GATTS_DELETE_EVT:
        break;
    default:
        break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{

    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT)
    {
        if (param->reg.status == ESP_GATT_OK)
        {
            heart_rate_profile_tab[PROFILE_APP_IDX].gatts_if = gatts_if;
        }
        else
        {
            ESP_LOGE(GATTS_TABLE_TAG, "reg app failed, app_id %04x, status %d",
                     param->reg.app_id,
                     param->reg.status);
            return;
        }
    }
    do
    {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++)
        {
            /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
            if (gatts_if == ESP_GATT_IF_NONE || gatts_if == heart_rate_profile_tab[idx].gatts_if)
            {
                if (heart_rate_profile_tab[idx].gatts_cb)
                {
                    heart_rate_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}

void mybleinit()
{
    esp_err_t ret;

    /* Initialize NVS. */
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "gatts register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "gap register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gatts_app_register(ESP_APP_ID);
    if (ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "gatts app register error, error code = %x", ret);
        return;
    }

    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret)
    {
        ESP_LOGE(GATTS_TABLE_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }
}

TaskHandle_t TaskList[4];
enum
{
    TASK_LDR,
    TASK_LIGHT,
    TASK_LD2410B,
    TASK_BOW,
    TASK_NUM,
};

#define LED_PIN_NUM 12
#define BOX_PIN_NUM 25
#define LD2410B_PIN_NUM 21
int val_array[3] = {0};
uint8_t val_i = 0;

static ledc_timer_config_t ledc_timer_ls = {
    .speed_mode = LEDC_LOW_SPEED_MODE,    // 定时器模式（“高速”或“低速”）
    .duty_resolution = LEDC_TIMER_13_BIT, // 设置分辨率,最大为2^13-1
    .timer_num = LEDC_TIMER_1,            // 设置定时器源（0-3）
    .freq_hz = 5000,                      // PWM信号频率
    .clk_cfg = LEDC_AUTO_CLK,             // 配置LEDC时钟源（这里是自动选择）
};
static ledc_channel_config_t Led_Pwm_Num = {
    .gpio_num = LED_PIN_NUM,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_1,
    .timer_sel = LEDC_TIMER_1,
    .duty = 0,
    .hpoint = 0,
};

#define MAX_DUTY ((1 << ledc_timer_ls.duty_resolution) - 1)
void Led_Init()
{
    ledc_timer_config(&ledc_timer_ls);
    ledc_channel_config(&Led_Pwm_Num);
}
class PIDController
{
public:
    PIDController(float Kp, float Ki, float Kd) : m_Kp(Kp), m_Ki(Ki), m_Kd(Kd),
                                                  m_errorIntegral(0.0), m_previousError(0.0)
    {
    }

    // 更新PID控制器并返回控制输出
    float update(float setpoint, float currentInput, float dt)
    {
        float error = setpoint - currentInput;
        m_errorIntegral += error * dt;
        float derivative = (error - m_previousError) / dt;

        float output = m_Kp * error + m_Ki * m_errorIntegral + m_Kd * derivative;

        // 可选：限制输出范围或进行抗积分饱和处理

        m_previousError = error; // 更新前一次误差值

        return output;
    }

private:
    float m_Kp;            // 比例增益
    float m_Ki;            // 积分增益
    float m_Kd;            // 微分增益
    float m_errorIntegral; // 积分项
    float m_previousError; // 前一次误差值
};
float Kp = 1.0, Ki = 0.5, Kd = 0.2;
PIDController pid(Kp, Ki, Kd);
u_int16_t setlight = 1000;
int DT = 50;
bool led_onoff = 0;              // led开关，可小程序强制;
u_int16_t led_buff = 500;        // led增益，可小程序调整;
u_int8_t led_min = 52;           // led上阈值，可小程序调整;
u_int8_t led_max = 255;          // led下阈值，可小程序调整;
u_int8_t ld2410_threshold = 140; // ld2410阈值，可小程序调整;
u_int16_t bow_min = 52;          // 记忆合金上阈值，可小程序调整;
u_int16_t bow_max = 800;         // 记忆合金下阈值，可小程序调整;
u_int16_t bow_mid = 450;         // 记忆合金下阈值，可小程序调整;
u_int16_t bow_time = 1500;       // 记忆合金鞠躬时间，可小程序调整;

QueueHandle_t xMyLDRQueue;
void myLDRTimerCallback(TimerHandle_t pxTimer)
{
    BaseType_t xHigherPriorityTaskWoken_LDR = pdFALSE;

    // 消息内容（这里仅作为示例）
    bool ldr_isr = true;

    // 尝试从ISR向队列发送消息
    if (xQueueSendFromISR(xMyLDRQueue, &ldr_isr, &xHigherPriorityTaskWoken_LDR) != pdPASS)
    {
        // 发送失败，处理错误情况...
    }

    // 检查是否有高优先级任务被唤醒
    if (xHigherPriorityTaskWoken_LDR == pdTRUE)
    {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken_LDR); // 在所有消息发送完毕后，检查是否需要进行上下文切换
    }
}

#define RX_BUF_SIZE 128
#define LD2410B_UART_RX 16
#define LD2410B_UART_TX 17
void myLDRTask(void *pvParameters)
{
    xMyLDRQueue = xQueueCreate(5, sizeof(bool));
    if (xMyLDRQueue == NULL)
    {
        ESP_LOGE("xMyLD2410BQueue_TIME", "Queue creat fail !!!");
    }
    TimerHandle_t myLDRTimer = NULL;
    myLDRTimer = xTimerCreate("myLDRTimer", DT / portTICK_PERIOD_MS,
                              pdTRUE, NULL, myLDRTimerCallback);
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);
    esp_adc_cal_characteristics_t *adcChar = (esp_adc_cal_characteristics_t *)calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t cal_mode = esp_adc_cal_characterize(ADC_UNIT_1,
                                                            ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, adcChar);
    bool ldr_en = false;
    while (1)
    {
        if (xQueueReceive(xMyLDRQueue, &ldr_en, portMAX_DELAY))
        {

            if (!ldr_en)
            {
                if (xTimerStop(myLDRTimer, portMAX_DELAY) == pdPASS)
                {
                    printf("Repeating timer stopped.\n");
                }
                val_array[val_i] = adc1_get_raw(ADC1_CHANNEL_6);
                if (!val_array[val_i])
                {
                    return;
                }
                val_i++;
                if (val_i++ >= 3)
                {
                    val_i = 0;
                }
                if (val_array[2] == 0)
                {
                    return;
                }
                int val = (val_array[0] + val_array[1] + val_array[2]) / 3;
                int voltage = esp_adc_cal_raw_to_voltage(val, adcChar);
                float controlOutput = pid.update(float(setlight) * 0.1f, voltage, DT) * led_onoff * led_buff / 500;
                if (controlOutput < float(led_min) / 255.0f)
                {
                    controlOutput = 0;
                }
                if (controlOutput > float(led_min) / 255.0f)
                {
                    controlOutput = float(led_min) / 255.0f;
                }
                ledc_set_duty_and_update(Led_Pwm_Num.speed_mode, Led_Pwm_Num.channel, controlOutput * MAX_DUTY, 0);
                if (xTimerIsTimerActive(myLDRTimer)) // 检查定时器是否处于暂停状态
                {
                    if(xTimerReset(myLDRTimer, portMAX_DELAY)==pdFALSE)
                    {
                        ESP_LOGE("myLDRTimer", "Timer reset fail !!!");

                    }
                }
                
            }
        }
        else
        {
            vTaskDelay(20 / portTICK_PERIOD_MS);
        }
    }
    vTaskDelete(NULL);
}
TaskHandle_t myLDRTaskHandle_t = NULL;

/*
myLD2410BTask：
    该任务通过串口指令对LD2410B进行设置
*/
void myLD2410BTask(void *pvParameters)
{
    bool LD2410_SER_EN = false;
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, LD2410B_UART_TX, LD2410B_UART_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    while (1)
    {
        if (LD2410_SER_EN)
        {
            char data[100] = {0}; // ld2410b at
            const int len = strlen(data);
            const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
        }
        else
        {
            vTaskDelay(1000 / portTICK_RATE_MS);
        }
    }
    vTaskDelete(NULL);
}
TaskHandle_t *myLD2410BTaskHandle_t = &TaskList[TASK_LD2410B];

static ledc_channel_config_t Box_Pwm_Num = {
    .gpio_num = BOX_PIN_NUM,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_2,
    .timer_sel = LEDC_TIMER_1,
    .duty = 0,
    .hpoint = 0,
};

QueueHandle_t xMyLD2410BQueue_TIME;
QueueHandle_t xMyLD2410BQueue_BOX_EN;
QueueHandle_t xMyLD2410BQueue_BOW_LOCATION;
void LD2410B_ISR_Handler(void *arg)
{
    BaseType_t xHigherPriorityTaskWoken_LD2410B_TIME = pdFALSE;
    BaseType_t xHigherPriorityTaskWoken_LD2410B_BOX_EN = pdFALSE;
    BaseType_t xHigherPriorityTaskWoken_LD2410B_BOW_LOCATION = pdFALSE;
    u_int32_t gpio_num = (u_int32_t)arg;
    bool gpio_level = gpio_get_level((gpio_num_t)gpio_num);
    // 鞠躬
    bool BOX_EN = true;
    bool BOW_LOCATION = true;
    if (gpio_level)
    {
        vTaskDelay(10 / portTICK_RATE_MS);
        if ((gpio_get_level((gpio_num_t)gpio_num)) == gpio_level)
        {
            BOX_EN = true;
            BOW_LOCATION = true;
        }
        else
        {
            return;
        }
    }
    // 起身
    else
    {
        vTaskDelay(10 / portTICK_RATE_MS);
        if ((gpio_get_level((gpio_num_t)gpio_num)) == gpio_level)
        {
            BOX_EN = true;
            BOW_LOCATION = false;
            time_t time_a = time(NULL);
            if (xQueueSendFromISR(xMyLD2410BQueue_TIME, &time_a, &xHigherPriorityTaskWoken_LD2410B_TIME) != pdPASS)
            {
            }
        }
        else
        {
            return;
        }
    }
    if (xQueueSendFromISR(xMyLD2410BQueue_BOX_EN, &BOX_EN, &xHigherPriorityTaskWoken_LD2410B_BOX_EN) != pdTRUE)
    {
    }
    if (xQueueSendFromISR(xMyLD2410BQueue_BOW_LOCATION, &BOW_LOCATION, &xHigherPriorityTaskWoken_LD2410B_BOW_LOCATION) != pdPASS)
    {
    }
    if (xHigherPriorityTaskWoken_LD2410B_TIME || xHigherPriorityTaskWoken_LD2410B_BOX_EN || xHigherPriorityTaskWoken_LD2410B_BOW_LOCATION)
    {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken_LD2410B_TIME || xHigherPriorityTaskWoken_LD2410B_BOX_EN || xHigherPriorityTaskWoken_LD2410B_BOW_LOCATION); // 唤醒任务以处理中断
    }
}

void myBowTask(void *pvParameters)
{
    bool box_en = 0;
    bool bow_location = 0; // true鞠躬，flase起身
    time_t time_isr = time(NULL);
    // bool time_get = false;

    xMyLD2410BQueue_TIME = xQueueCreate(5, sizeof(time_t));
    if (xMyLD2410BQueue_TIME == NULL)
    {
        ESP_LOGE("xMyLD2410BQueue_TIME", "Queue creat fail !!!");
    }
    xMyLD2410BQueue_BOX_EN = xQueueCreate(5, sizeof(bool));
    if (xMyLD2410BQueue_BOX_EN == NULL)
    {
        ESP_LOGE("xMyLD2410BQueue_BOX_EN", "Queue creat fail !!!");
    }
    xMyLD2410BQueue_BOW_LOCATION = xQueueCreate(5, sizeof(bool));
    if (xMyLD2410BQueue_BOW_LOCATION == NULL)
    {
        ESP_LOGE("xMyLD2410BQueue_BOW_LOCATION", "Queue creat fail !!!");
    }

    ledc_channel_config(&Box_Pwm_Num);
    gpio_set_direction((gpio_num_t)LD2410B_PIN_NUM, GPIO_MODE_INPUT);
    gpio_set_intr_type((gpio_num_t)LD2410B_PIN_NUM, GPIO_INTR_NEGEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add((gpio_num_t)LD2410B_PIN_NUM, LD2410B_ISR_Handler, (void *)LD2410B_PIN_NUM);
    while (1)
    {
        if (xQueueReceive(xMyLD2410BQueue_BOX_EN, &box_en, 1000 / portTICK_PERIOD_MS) != pdFALSE)
        {
            if (box_en)
            {
                if (xQueueReceive(xMyLD2410BQueue_BOW_LOCATION, &bow_location, 100 / portTICK_PERIOD_MS))
                {
                    if (bow_location)
                    {
                        ledc_fade_func_install(0);
                        ledc_set_fade_with_time(Box_Pwm_Num.speed_mode, Box_Pwm_Num.channel, MAX_DUTY * bow_min / 1000, bow_time);
                        ledc_fade_start(Box_Pwm_Num.speed_mode, Box_Pwm_Num.channel, LEDC_FADE_NO_WAIT);
                        vTaskDelay(bow_time / portTICK_RATE_MS);
                        ledc_set_fade_with_time(Box_Pwm_Num.speed_mode, Box_Pwm_Num.channel, MAX_DUTY * bow_max / 1000, bow_time);
                        ledc_fade_start(Box_Pwm_Num.speed_mode, Box_Pwm_Num.channel, LEDC_FADE_NO_WAIT);
                        vTaskDelay(bow_time / portTICK_RATE_MS);
                        ledc_set_fade_with_time(Box_Pwm_Num.speed_mode, Box_Pwm_Num.channel, MAX_DUTY * bow_mid / 1000, bow_time);
                        ledc_fade_start(Box_Pwm_Num.speed_mode, Box_Pwm_Num.channel, LEDC_FADE_NO_WAIT);
                        vTaskDelay(bow_time / portTICK_RATE_MS);
                        box_en = false;
                    }
                    else
                    {
                        if (xQueueReceive(xMyLD2410BQueue_TIME, &time_isr, 100 / portTICK_PERIOD_MS))
                        {
                        }
                    }
                }
            }
        }
        else
        {
            time_t time_b = time(NULL);
            if (box_en && time_b - time_isr > bow_time)
            {
                ledc_fade_func_install(0);
                ledc_set_fade_with_time(Box_Pwm_Num.speed_mode, Box_Pwm_Num.channel, MAX_DUTY * bow_max / 1000, bow_time);
                ledc_fade_start(Box_Pwm_Num.speed_mode, Box_Pwm_Num.channel, LEDC_FADE_NO_WAIT);
                vTaskDelay(bow_time / portTICK_RATE_MS);
                ledc_set_fade_with_time(Box_Pwm_Num.speed_mode, Box_Pwm_Num.channel, MAX_DUTY * bow_min / 1000, bow_time);
                ledc_fade_start(Box_Pwm_Num.speed_mode, Box_Pwm_Num.channel, LEDC_FADE_NO_WAIT);
                vTaskDelay(bow_time / portTICK_RATE_MS);
                ledc_set_fade_with_time(Box_Pwm_Num.speed_mode, Box_Pwm_Num.channel, MAX_DUTY * bow_mid / 1000, bow_time);
                ledc_fade_start(Box_Pwm_Num.speed_mode, Box_Pwm_Num.channel, LEDC_FADE_NO_WAIT);
                vTaskDelay(bow_time / portTICK_RATE_MS);
                box_en = false;
            }
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
    }

    vTaskDelete(NULL);
}

TaskHandle_t *myBowTaskHandle_t = &TaskList[TASK_BOW];

TaskHandle_t mybleTaskHandle_t = NULL;
void mybleTask(void *pvParameters)
{
    mybleinit();
}

extern "C" void app_main(void);
void app_main(void)
{
    xTaskCreatePinnedToCore(mybleTask, "mybleTask",
                            2 * 1024, NULL, 20, &mybleTaskHandle_t, 0);
    // 鞠躬
    xTaskCreatePinnedToCore(myBowTask, "myBowTask",
                            2 * 1024, NULL, 10 - TASK_BOW, myBowTaskHandle_t, 0);

    xTaskCreatePinnedToCore(myLDRTask, "myLDRTask",
                            2 * 1024, NULL, 10 - TASK_BOW, &myLDRTaskHandle_t, 1);
}
