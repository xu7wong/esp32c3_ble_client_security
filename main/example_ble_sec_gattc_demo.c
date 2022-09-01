/*
 * SPDX-FileCopyrightText: 2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */



/****************************************************************************
*
* This file is for gatt_security_client demo. It can scan ble device, connect one device that needs to be encrypted.
* run gatt_security_server demo, the gatt_security_client demo will automatically connect the gatt_security_server,
* then paring and bonding.
*
****************************************************************************/

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "nvs.h"
#include "nvs_flash.h"

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/twai.h"

// #include "sdkconfig.h"

#define GATTC_TAG             "SEC_GATTC"
#define LOOP_TAG             "LOOP"
#define UART1_TAG             "UART1"
#define CAN_TAG                 "CAN_BUS"
//#define REMOTE_SERVICE_UUID   0x331a36f5245945ea9d956142f0c4b307
//ESP_GATT_UUID_HEART_RATE_SVC
//#define REMOTE_NOTIFY_UUID    0x2A37

//331a36f5-2459-45ea-9d95-6142f0c4b307
const uint8_t REMOTE_SERVICE_UUID_ARRAY[] = {
                 0x07,0xb3,0xc4,0xf0,
                 0x42,0x61,0x95,0x9d,
                 0xea,0x45,0x59,0x24,
                 0xf5,0x36,0x1a,0x33,
    
    };

//a73e9a10-628f-4494-a099-12efaf72258f
const uint8_t REMOTE_CHAR_NOTIFY_UUID_ARRAY[] = {
                 0x8f,0x25,0x72,0xaf,
                 0xef,0x12,0x99,0xa0,
                 0x94,0x44,0x8f,0x62,
                 0x10,0x9a,0x3e,0xa7,
    
    };
//a73e9a10-628f-4494-a099-12efaf72258f
const uint8_t REMOTE_CHAR_WRITE_UUID_ARRAY[] = {
                 0x33,0x88,0xa2,0x1c,
                 0xe4,0x9c,0xec,0x94,
                 0x95,0x49,0x23,0x08,
                 0x40,0x60,0xda,0xa9,
    
    };
static esp_gattc_char_elem_t *char_elem_result   = NULL;
static esp_gattc_descr_elem_t *descr_elem_result = NULL;

///Declare static functions
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);


// static esp_bt_uuid_t remote_filter_service_uuid = {
//     .len = ESP_UUID_LEN_16,
//     .uuid = {.uuid16 = REMOTE_SERVICE_UUID,},
// };

//331a36f5-2459-45ea-9d95-6142f0c4b307
//  0x33,0x1a,0x36,0xf5,
//  0x24,0x59,0x45,0xea,
//  0x9d,0x95,0x61,0x42,
//  0xf0,0xc4,0xb3,0x07

static esp_bt_uuid_t remote_filter_service_uuid = {
  .len = ESP_UUID_LEN_128,
  .uuid = {
    .uuid128 = {
      REMOTE_SERVICE_UUID_ARRAY[0],
      REMOTE_SERVICE_UUID_ARRAY[1],
      REMOTE_SERVICE_UUID_ARRAY[2],
      REMOTE_SERVICE_UUID_ARRAY[3],
      REMOTE_SERVICE_UUID_ARRAY[4],
      REMOTE_SERVICE_UUID_ARRAY[5],
      REMOTE_SERVICE_UUID_ARRAY[6],
      REMOTE_SERVICE_UUID_ARRAY[7],
      REMOTE_SERVICE_UUID_ARRAY[8],
      REMOTE_SERVICE_UUID_ARRAY[9],
      REMOTE_SERVICE_UUID_ARRAY[10],
      REMOTE_SERVICE_UUID_ARRAY[11],
      REMOTE_SERVICE_UUID_ARRAY[12],
      REMOTE_SERVICE_UUID_ARRAY[13],
      REMOTE_SERVICE_UUID_ARRAY[14],
      REMOTE_SERVICE_UUID_ARRAY[15],
    },
  }
};

// static esp_bt_uuid_t remote_filter_service_uuid;

//  = {
//     .len = ESP_UUID_LEN_128,
//     .uuid = {.uuid128 = REMOTE_SERVICE_UUID,},
//    .uuid.uuid128 =
//            {
                

//                  0x07,0xb3,0xc4,0xf0,
//                  0x42,0x61,0x95,0x9d,
//                  0xea,0x45,0x59,0x24,
//                  0xf5,0x36,0x1a,0x33,
//    },

// };

static bool connect = false;
static bool get_service = false;
static bool ble_device_ready = false;

static uint32_t time_request_MSG = 0;
static uint32_t time_request_SCAN = 0;
static uint32_t tick_interval_MSG = 200;
static uint8_t message_send_index = 0;
static bool scan_busy = true;
static const int allowed_minimum_rssi = -60;
static const char remote_device_name[] = "UB-*****";
static char connected_device_name[10];
const uint8_t remote_device_name_prefix_length = 3;
static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_RANDOM,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50,
    .scan_window            = 0x30,
    .scan_duplicate         = BLE_SCAN_DUPLICATE_DISABLE
};


#define PROFILE_NUM 1
#define PROFILE_A_APP_ID 0
#define INVALID_HANDLE   0

struct gattc_profile_inst {
    esp_gattc_cb_t gattc_cb;
    uint16_t gattc_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_start_handle;
    uint16_t service_end_handle;
    uint16_t notify_char_handle;
    uint16_t write_char_handle;
    esp_bd_addr_t remote_bda;
};

/* One gatt-based profile one app_id and one gattc_if, this array will store the gattc_if returned by ESP_GATTS_REG_EVT */
static struct gattc_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_A_APP_ID] = {
        .gattc_cb = gattc_profile_event_handler,
        .gattc_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

static const int uart_num = UART_NUM_1;
#define UART_RX_BUF_SIZE 127
static uint8_t uart_rx_buffer[UART_RX_BUF_SIZE];
static bool can_bus_listen = false;

static uint16_t MODBUS_CRC16_V1( const uint8_t *buf, uint16_t len );
static bool MODBUS_CRC_pass(uint8_t* data, uint16_t data_len);

static const char *esp_key_type_to_str(esp_ble_key_type_t key_type)
{
   const char *key_str = NULL;
   switch(key_type) {
    case ESP_LE_KEY_NONE:
        key_str = "ESP_LE_KEY_NONE";
        break;
    case ESP_LE_KEY_PENC:
        key_str = "ESP_LE_KEY_PENC";
        break;
    case ESP_LE_KEY_PID:
        key_str = "ESP_LE_KEY_PID";
        break;
    case ESP_LE_KEY_PCSRK:
        key_str = "ESP_LE_KEY_PCSRK";
        break;
    case ESP_LE_KEY_PLK:
        key_str = "ESP_LE_KEY_PLK";
        break;
    case ESP_LE_KEY_LLK:
        key_str = "ESP_LE_KEY_LLK";
        break;
    case ESP_LE_KEY_LENC:
        key_str = "ESP_LE_KEY_LENC";
        break;
    case ESP_LE_KEY_LID:
        key_str = "ESP_LE_KEY_LID";
        break;
    case ESP_LE_KEY_LCSRK:
        key_str = "ESP_LE_KEY_LCSRK";
        break;
    default:
        key_str = "INVALID BLE KEY TYPE";
        break;

    }
     return key_str;
}

static char *esp_auth_req_to_str(esp_ble_auth_req_t auth_req)
{
   char *auth_str = NULL;
   switch(auth_req) {
    case ESP_LE_AUTH_NO_BOND:
        auth_str = "ESP_LE_AUTH_NO_BOND";
        break;
    case ESP_LE_AUTH_BOND:
        auth_str = "ESP_LE_AUTH_BOND";
        break;
    case ESP_LE_AUTH_REQ_MITM:
        auth_str = "ESP_LE_AUTH_REQ_MITM";
        break;
    case ESP_LE_AUTH_REQ_BOND_MITM:
        auth_str = "ESP_LE_AUTH_REQ_BOND_MITM";
        break;
    case ESP_LE_AUTH_REQ_SC_ONLY:
        auth_str = "ESP_LE_AUTH_REQ_SC_ONLY";
        break;
    case ESP_LE_AUTH_REQ_SC_BOND:
        auth_str = "ESP_LE_AUTH_REQ_SC_BOND";
        break;
    case ESP_LE_AUTH_REQ_SC_MITM:
        auth_str = "ESP_LE_AUTH_REQ_SC_MITM";
        break;
    case ESP_LE_AUTH_REQ_SC_MITM_BOND:
        auth_str = "ESP_LE_AUTH_REQ_SC_MITM_BOND";
        break;
    default:
        auth_str = "INVALID BLE AUTH REQ";
        break;
   }

   return auth_str;
}

static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    esp_ble_gattc_cb_param_t *p_data = (esp_ble_gattc_cb_param_t *)param;

    switch (event) {
    case ESP_GATTC_REG_EVT:
        ESP_LOGI(GATTC_TAG, "REG_EVT");
        esp_ble_gap_config_local_privacy(true);
        break;
    case ESP_GATTC_OPEN_EVT:
        if (param->open.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "open failed, error status = %x", p_data->open.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "open success");
        gl_profile_tab[PROFILE_A_APP_ID].conn_id = p_data->open.conn_id;
        memcpy(gl_profile_tab[PROFILE_A_APP_ID].remote_bda, p_data->open.remote_bda, sizeof(esp_bd_addr_t));
        ESP_LOGI(GATTC_TAG, "REMOTE BDA:");
        esp_log_buffer_hex(GATTC_TAG, gl_profile_tab[PROFILE_A_APP_ID].remote_bda, sizeof(esp_bd_addr_t));
        esp_err_t mtu_ret = esp_ble_gattc_send_mtu_req (gattc_if, p_data->open.conn_id);
        if (mtu_ret){
            ESP_LOGE(GATTC_TAG, "config MTU error, error code = %x", mtu_ret);
        }
        break;
    case ESP_GATTC_CFG_MTU_EVT:
        if (param->cfg_mtu.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG,"config mtu failed, error status = %x", param->cfg_mtu.status);
        }
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_CFG_MTU_EVT, Status %d, MTU %d, conn_id %d", param->cfg_mtu.status, param->cfg_mtu.mtu, param->cfg_mtu.conn_id);
        esp_ble_gattc_search_service(gattc_if, param->cfg_mtu.conn_id, &remote_filter_service_uuid);
        break;
    case ESP_GATTC_SEARCH_RES_EVT: {
        ESP_LOGI(GATTC_TAG, "SEARCH RES: conn_id = %x is primary service %d", p_data->search_res.conn_id, p_data->search_res.is_primary);
        ESP_LOGI(GATTC_TAG, "start handle %d end handle %d current handle value %d", p_data->search_res.start_handle, p_data->search_res.end_handle, p_data->search_res.srvc_id.inst_id);
        if (p_data->search_res.srvc_id.uuid.len == ESP_UUID_LEN_128 && (memcmp(p_data->search_res.srvc_id.uuid.uuid.uuid128,REMOTE_SERVICE_UUID_ARRAY,sizeof(REMOTE_SERVICE_UUID_ARRAY))==0)) {
            
        //if (p_data->search_res.srvc_id.uuid.len == ESP_UUID_LEN_16 && p_data->search_res.srvc_id.uuid.uuid.uuid16 == REMOTE_SERVICE_UUID) {
            //ESP_LOGI(GATTC_TAG, "UUID128: %x", p_data->search_res.srvc_id.uuid.uuid.uuid128);
            ESP_LOGI(GATTC_TAG, "found matched service UUID128"); 
            get_service = true;
            gl_profile_tab[PROFILE_A_APP_ID].service_start_handle = p_data->search_res.start_handle;
            gl_profile_tab[PROFILE_A_APP_ID].service_end_handle = p_data->search_res.end_handle;
        }
        break;
    }
    case ESP_GATTC_SEARCH_CMPL_EVT:
        if (p_data->search_cmpl.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "search service failed, error status = %x", p_data->search_cmpl.status);
            break;
        }
        if(p_data->search_cmpl.searched_service_source == ESP_GATT_SERVICE_FROM_REMOTE_DEVICE) {
            ESP_LOGI(GATTC_TAG, "Get service information from remote device %d", get_service);
        } else if (p_data->search_cmpl.searched_service_source == ESP_GATT_SERVICE_FROM_NVS_FLASH) {
            ESP_LOGI(GATTC_TAG, "Get service information from flash");
        } else {
            ESP_LOGI(GATTC_TAG, "unknown service source");
        }
        if (get_service){
            uint16_t count  = 0;
            uint16_t offset = 0;
            esp_gatt_status_t ret_status = esp_ble_gattc_get_attr_count(gattc_if,
                                                                        gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                                                        ESP_GATT_DB_CHARACTERISTIC,
                                                                        gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                                                                        gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                                                                        INVALID_HANDLE,
                                                                        &count);
            if (ret_status != ESP_GATT_OK){
                ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_attr_count error, %d", __LINE__);
            }
            if (count > 0){
                char_elem_result = (esp_gattc_char_elem_t *)malloc(sizeof(esp_gattc_char_elem_t) * count);
                if (!char_elem_result){
                    ESP_LOGE(GATTC_TAG, "gattc no mem");
                }else{
                    ret_status = esp_ble_gattc_get_all_char(gattc_if,
                                                            gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                                            gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                                                            gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                                                            char_elem_result,
                                                            &count,
                                                            offset);
                    if (ret_status != ESP_GATT_OK){
                        ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_all_char error, %d", __LINE__);
                    }
                    if (count > 0){
                        
                        for (int i = 0; i < count; ++i)
                        {
                            ESP_LOGI(GATTC_TAG, "Get char information len = %d", char_elem_result[i].uuid.len, char_elem_result[i].uuid.uuid.uuid128);
                            if (char_elem_result[i].uuid.len == ESP_UUID_LEN_128 && (memcmp(char_elem_result[i].uuid.uuid.uuid128,REMOTE_CHAR_NOTIFY_UUID_ARRAY,sizeof(REMOTE_CHAR_NOTIFY_UUID_ARRAY))==0) && (char_elem_result[i].properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY))
                            {
                                ESP_LOGI(GATTC_TAG, "esp_ble_gattc_register_for_notify");
                                gl_profile_tab[PROFILE_A_APP_ID].notify_char_handle = char_elem_result[i].char_handle;
                                // esp_ble_gattc_register_for_notify (gattc_if,
                                //                                    gl_profile_tab[PROFILE_A_APP_ID].remote_bda,
                                //                                    char_elem_result[i].char_handle);
                                //break;
                            }
                            else if (char_elem_result[i].uuid.len == ESP_UUID_LEN_128 && (memcmp(char_elem_result[i].uuid.uuid.uuid128,REMOTE_CHAR_WRITE_UUID_ARRAY,sizeof(REMOTE_CHAR_WRITE_UUID_ARRAY))==0) && (char_elem_result[i].properties & ESP_GATT_CHAR_PROP_BIT_WRITE))
                            {
                                ESP_LOGI(GATTC_TAG, "esp_ble_gattc found for write");
                                gl_profile_tab[PROFILE_A_APP_ID].write_char_handle = char_elem_result[i].char_handle;
                            }
                        }

                        esp_ble_gattc_register_for_notify (gattc_if,
                                                                   gl_profile_tab[PROFILE_A_APP_ID].remote_bda,
                                                                   gl_profile_tab[PROFILE_A_APP_ID].notify_char_handle);
                    }
                }
                free(char_elem_result);
            }
        }

        break;
    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
        if (p_data->reg_for_notify.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "reg for notify failed, error status = %x", p_data->reg_for_notify.status);
            break;
        }

            uint16_t count = 0;
            uint16_t offset = 0;
            uint16_t notify_en = 1;
            esp_gatt_status_t ret_status = esp_ble_gattc_get_attr_count(gattc_if,
                                                                        gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                                                        ESP_GATT_DB_DESCRIPTOR,
                                                                        gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                                                                        gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                                                                        p_data->reg_for_notify.handle,
                                                                        &count);
            if (ret_status != ESP_GATT_OK){
                ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_attr_count error, %d", __LINE__);
            }
            if (count > 0){
                descr_elem_result = malloc(sizeof(esp_gattc_descr_elem_t) * count);
                if (!descr_elem_result){
                    ESP_LOGE(GATTC_TAG, "malloc error, gattc no mem");
                }else{
                    ret_status = esp_ble_gattc_get_all_descr(gattc_if,
                                                             gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                                             p_data->reg_for_notify.handle,
                                                             descr_elem_result,
                                                             &count,
                                                             offset);
                if (ret_status != ESP_GATT_OK){
                    ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_all_descr error, %d", __LINE__);
                }

                    for (int i = 0; i < count; ++i)
                    {
                        ESP_LOGI(GATTC_TAG, "Get desc information len = %d", descr_elem_result[i].uuid.len);
                        if (descr_elem_result[i].uuid.len == ESP_UUID_LEN_16 && descr_elem_result[i].uuid.uuid.uuid16 == ESP_GATT_UUID_CHAR_CLIENT_CONFIG)
                        {
                            esp_ble_gattc_write_char_descr (gattc_if,
                                                            gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                                                            descr_elem_result[i].handle,
                                                            sizeof(notify_en),
                                                            (uint8_t *)&notify_en,
                                                            ESP_GATT_WRITE_TYPE_RSP,
                                                            ESP_GATT_AUTH_REQ_SIGNED_NO_MITM);

                            break;
                        }
                    }
                }
                free(descr_elem_result);
            }

        break;
    }
    case ESP_GATTC_NOTIFY_EVT:
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_NOTIFY_EVT, receive notify value:");
        esp_log_buffer_hex(GATTC_TAG, p_data->notify.value, p_data->notify.value_len);
        break;
    case ESP_GATTC_WRITE_DESCR_EVT:
        if (p_data->write.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "write descr failed, error status = %x", p_data->write.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "> write descr success, bt is ready");
        // uint8_t value[]={'a', 'b'};
        // esp_ble_gattc_write_char(gattc_if,
        //                         gl_profile_tab[PROFILE_A_APP_ID].conn_id,
        //                         gl_profile_tab[PROFILE_A_APP_ID].write_char_handle,
                                
        //                         sizeof(value),
        //                         value,
        //                         ESP_GATT_WRITE_TYPE_RSP,
        //                         ESP_GATT_AUTH_REQ_SIGNED_NO_MITM);
        message_send_index = 0;
        ble_device_ready = true;
        
        break;
    case ESP_GATTC_SRVC_CHG_EVT: {
        esp_bd_addr_t bda;
        memcpy(bda, p_data->srvc_chg.remote_bda, sizeof(esp_bd_addr_t));
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_SRVC_CHG_EVT, bd_addr:");
        esp_log_buffer_hex(GATTC_TAG, bda, sizeof(esp_bd_addr_t));
        break;
    }
    case ESP_GATTC_WRITE_CHAR_EVT:
        if (p_data->write.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "write char failed, error status = %x", p_data->write.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "Write char success ");
        break;
    case ESP_GATTC_DISCONNECT_EVT:
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_DISCONNECT_EVT, reason = 0x%x", p_data->disconnect.reason);
        ble_device_ready = false;
        connect = false;
        get_service = false;
        scan_busy = false;
        break;
    default:
        break;
    }
}

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    uint8_t *adv_name = NULL;
    uint8_t adv_name_len = 0;
    switch (event) {
    case ESP_GAP_BLE_SET_LOCAL_PRIVACY_COMPLETE_EVT:
        if (param->local_privacy_cmpl.status != ESP_BT_STATUS_SUCCESS){
            ESP_LOGE(GATTC_TAG, "config local privacy failed, error code =%x", param->local_privacy_cmpl.status);
            break;
        }
        esp_err_t scan_ret = esp_ble_gap_set_scan_params(&ble_scan_params);
        if (scan_ret){
            ESP_LOGE(GATTC_TAG, "set scan params error, error code = %x", scan_ret);
        }
        break;
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
        //the unit of the duration is second
        uint32_t duration = 5;
        scan_busy = true;
        esp_ble_gap_start_scanning(duration);
        break;
    }
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        //scan start complete event to indicate scan start successfully or failed
        if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTC_TAG, "scan start failed, error status = %x", param->scan_start_cmpl.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "Scan start success");
        break;
    case ESP_GAP_BLE_PASSKEY_REQ_EVT:                           /* passkey request event */
        /* Call the following function to input the passkey which is displayed on the remote device */
        //esp_ble_passkey_reply(gl_profile_tab[PROFILE_A_APP_ID].remote_bda, true, 0x00);
        ESP_LOGI(GATTC_TAG, "ESP_GAP_BLE_PASSKEY_REQ_EVT");
        break;
    case ESP_GAP_BLE_OOB_REQ_EVT: {
        ESP_LOGI(GATTC_TAG, "ESP_GAP_BLE_OOB_REQ_EVT");
        uint8_t tk[16] = {1}; //If you paired with OOB, both devices need to use the same tk
        esp_ble_oob_req_reply(param->ble_security.ble_req.bd_addr, tk, sizeof(tk));
        break;
    }
    case ESP_GAP_BLE_LOCAL_IR_EVT:                               /* BLE local IR event */
        ESP_LOGI(GATTC_TAG, "ESP_GAP_BLE_LOCAL_IR_EVT");
        break;
    case ESP_GAP_BLE_LOCAL_ER_EVT:                               /* BLE local ER event */
        ESP_LOGI(GATTC_TAG, "ESP_GAP_BLE_LOCAL_ER_EVT");
        break;
    case ESP_GAP_BLE_SEC_REQ_EVT:
        /* send the positive(true) security response to the peer device to accept the security request.
        If not accept the security request, should send the security response with negative(false) accept value*/
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
        break;
    case ESP_GAP_BLE_NC_REQ_EVT:
        /* The app will receive this evt when the IO has DisplayYesNO capability and the peer device IO also has DisplayYesNo capability.
        show the passkey number to the user to confirm it with the number displayed by peer device. */
        esp_ble_confirm_reply(param->ble_security.ble_req.bd_addr, true);
        ESP_LOGI(GATTC_TAG, "ESP_GAP_BLE_NC_REQ_EVT, the passkey Notify number:%d", param->ble_security.key_notif.passkey);
        break;
    case ESP_GAP_BLE_PASSKEY_NOTIF_EVT:  ///the app will receive this evt when the IO  has Output capability and the peer device IO has Input capability.
        ///show the passkey number to the user to input it in the peer device.
        ESP_LOGI(GATTC_TAG, "The passkey Notify number:%06d", param->ble_security.key_notif.passkey);
        break;
    case ESP_GAP_BLE_KEY_EVT:
        //shows the ble key info share with peer device to the user.
        ESP_LOGI(GATTC_TAG, "key type = %s", esp_key_type_to_str(param->ble_security.ble_key.key_type));
        break;
    case ESP_GAP_BLE_AUTH_CMPL_EVT: {
        esp_bd_addr_t bd_addr;
        memcpy(bd_addr, param->ble_security.auth_cmpl.bd_addr, sizeof(esp_bd_addr_t));
        ESP_LOGI(GATTC_TAG, "remote BD_ADDR: %08x%04x",\
                (bd_addr[0] << 24) + (bd_addr[1] << 16) + (bd_addr[2] << 8) + bd_addr[3],
                (bd_addr[4] << 8) + bd_addr[5]);
        ESP_LOGI(GATTC_TAG, "address type = %d", param->ble_security.auth_cmpl.addr_type);
        ESP_LOGI(GATTC_TAG, "pair status = %s",param->ble_security.auth_cmpl.success ? "success" : "fail");
        if (!param->ble_security.auth_cmpl.success) {
            ESP_LOGI(GATTC_TAG, "fail reason = 0x%x",param->ble_security.auth_cmpl.fail_reason);
        } else {
            ESP_LOGI(GATTC_TAG, "auth mode = %s",esp_auth_req_to_str(param->ble_security.auth_cmpl.auth_mode));
        }
        break;
    }
    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        switch (scan_result->scan_rst.search_evt) {
        case ESP_GAP_SEARCH_INQ_RES_EVT:
            //esp_log_buffer_hex(GATTC_TAG, scan_result->scan_rst.bda, 6);
            //ESP_LOGI(GATTC_TAG, "Searched Adv Data Len %d, Scan Response Len %d, rssi=%d", scan_result->scan_rst.adv_data_len, scan_result->scan_rst.scan_rsp_len, scan_result->scan_rst.rssi);
            adv_name = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv,
                                                ESP_BLE_AD_TYPE_NAME_CMPL, &adv_name_len);
            //ESP_LOGI(GATTC_TAG, "Searched Device Name Len %d", adv_name_len);
            //esp_log_buffer_char(GATTC_TAG, adv_name, adv_name_len);
            //ESP_LOGI(GATTC_TAG, "\n");
            if (adv_name != NULL) {
                if (strlen(remote_device_name) == adv_name_len && strncmp((char *)adv_name, remote_device_name, remote_device_name_prefix_length) == 0) {
                    memset(connected_device_name, 0, sizeof(connected_device_name));
                    memcpy(connected_device_name, (char*)adv_name, sizeof(connected_device_name)-1);
                    ESP_LOGI(GATTC_TAG, "searched device %s, rssi = %d\n", (char *)adv_name, scan_result->scan_rst.rssi);
                    if(scan_result->scan_rst.rssi >= allowed_minimum_rssi){
                        if (connect == false) {
                        connect = true;
                        ESP_LOGI(GATTC_TAG, "connect to the remote device.");
                        esp_ble_gap_stop_scanning();
                        esp_ble_gattc_open(gl_profile_tab[PROFILE_A_APP_ID].gattc_if, scan_result->scan_rst.bda, scan_result->scan_rst.ble_addr_type, true);
                        }
                    }
                    else{
                        ESP_LOGI(GATTC_TAG, "remote device rssi is too weak");
                    }
                    
                }
            }
            break;
        case ESP_GAP_SEARCH_INQ_CMPL_EVT:
            scan_busy = false;
            ESP_LOGI(GATTC_TAG, "ESP_GAP_SEARCH_INQ_CMPL_EVT ??");
            break;
        default:
            break;
        }
        break;
    }

    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        if (param->scan_stop_cmpl.status != ESP_BT_STATUS_SUCCESS){
            ESP_LOGE(GATTC_TAG, "Scan stop failed, error status = %x", param->scan_stop_cmpl.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "Stop scan successfully");
        break;

    default:
        break;
    }
}

static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    ESP_LOGI(GATTC_TAG, "EVT %d, gattc if %d", event, gattc_if);

    /* If event is register event, store the gattc_if for each profile */
    if (event == ESP_GATTC_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile_tab[param->reg.app_id].gattc_if = gattc_if;
        } else {
            ESP_LOGI(GATTC_TAG, "Reg app failed, app_id %04x, status %d",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }

    /* If the gattc_if equal to profile A, call profile A cb handler,
     * so here call each profile's callback */
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            if (gattc_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                    gattc_if == gl_profile_tab[idx].gattc_if) {
                if (gl_profile_tab[idx].gattc_cb) {
                    gl_profile_tab[idx].gattc_cb(event, gattc_if, param);
                }
            }
        }
    } while (0);
}
void ble_send_bytes(uint8_t *bytes, uint8_t len)
{
    if (!ble_device_ready)
        return;
    esp_ble_gattc_write_char(gl_profile_tab[PROFILE_A_APP_ID].gattc_if,
                             gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                             gl_profile_tab[PROFILE_A_APP_ID].write_char_handle,
                             len,
                             bytes,
                             ESP_GATT_WRITE_TYPE_RSP,
                             ESP_GATT_AUTH_REQ_SIGNED_NO_MITM);
}

void ble_trigger_scan(){
    esp_err_t scan_ret = esp_ble_gap_set_scan_params(&ble_scan_params);
        if (scan_ret){
            ESP_LOGE(GATTC_TAG, "set scan params error, error code = %x", scan_ret);
        }
}
void can_loop()
{
    twai_message_t message;
    if (twai_receive(&message, pdMS_TO_TICKS(10000)) == ESP_OK)
    {
        ESP_LOGI(CAN_TAG, "Message received");
    }
    else
    {
        //ESP_LOGE(CAN_TAG, "Failed to receive message");
        return;
    }

    // Process received message
    if (message.extd)
    {
        ESP_LOGI(CAN_TAG, "Message is in Extended Format");
    }
    else
    {
        ESP_LOGI(CAN_TAG, "Message is in Standard Format");
    }
    ESP_LOGI(CAN_TAG, "ID is %d\n", message.identifier);
    if (!(message.rtr))
    {
        for (int i = 0; i < message.data_length_code; i++)
        {
            ESP_LOGI(CAN_TAG, "Data byte %d = %d\n", i, message.data[i]);
        }
    }
}
void app_main(void)
{
    // Initialize NVS.
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    
    uart_config_t uart_config = {
        .baud_rate = 19200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122
    };
    ESP_ERROR_CHECK(uart_driver_install(uart_num, UART_RX_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    // Set UART pins(TX: IO4, RX: IO5, RTS: IO18, CTS: IO19)
    ESP_ERROR_CHECK(uart_set_pin(uart_num, 3, 4, -1, -1));
    // Set RS485 half duplex mode
    ESP_ERROR_CHECK(uart_set_mode(uart_num, UART_MODE_RS485_HALF_DUPLEX));

    // Set read timeout of UART TOUT feature
    ESP_ERROR_CHECK(uart_set_rx_timeout(uart_num, 5));
    // char* msg = "abc\r\n";
    // if (uart_write_bytes(uart_num, (uint8_t*)msg, strlen(msg)) != strlen(msg)) {
    //     ESP_LOGE(UART1_TAG, "Send data critical failure.");
    // }

    //TWAI_GENERAL_CONFIG_DEFAULT(tx_io_num, rx_io_num, op_mode)
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(5, 8, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    //Install TWAI driver
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        ESP_LOGI(CAN_TAG, "Driver installed");
        //Start TWAI driver
        if (twai_start() == ESP_OK)
        {
            ESP_LOGI(CAN_TAG, "Driver started");
        }
        else
        {
            ESP_LOGE(CAN_TAG, "Failed to start driver");
            return;
        }
    } else {
        ESP_LOGE(CAN_TAG, "CAN bus failed to install driver");
    }

    
    

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s init bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    //register the  callback function to the gap module
    ret = esp_ble_gap_register_callback(esp_gap_cb);
    if (ret){
        ESP_LOGE(GATTC_TAG, "%s gap register error, error code = %x\n", __func__, ret);
        return;
    }

    //register the callback function to the gattc module
    ret = esp_ble_gattc_register_callback(esp_gattc_cb);
    if(ret){
        ESP_LOGE(GATTC_TAG, "%s gattc register error, error code = %x\n", __func__, ret);
        return;
    }

    ret = esp_ble_gattc_app_register(PROFILE_A_APP_ID);
    if (ret){
        ESP_LOGE(GATTC_TAG, "%s gattc app register error, error code = %x\n", __func__, ret);
    }

    ret = esp_ble_gatt_set_local_mtu(200);
    if (ret){
        ESP_LOGE(GATTC_TAG, "set local  MTU failed, error code = %x", ret);
    }

    /* set the security iocap & auth_req & key size & init key response key parameters to the stack*/
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_SC_MITM_BOND;     //bonding with peer device after authentication
    esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;           //set the IO capability to No output No input
    uint8_t key_size = 16;      //the key size should be 7~16 bytes
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t oob_support = ESP_BLE_OOB_DISABLE;
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_OOB_SUPPORT, &oob_support, sizeof(uint8_t));
    /* If your BLE device act as a Slave, the init_key means you hope which types of key of the master should distribute to you,
    and the response key means which key you can distribute to the Master;
    If your BLE device act as a master, the response key means you hope which types of key of the slave should distribute to you,
    and the init key means which key you can distribute to the slave. */
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));

    while(1){
        uint32_t t = xTaskGetTickCount();
        if(t - time_request_SCAN >= 200){
            time_request_SCAN = t;
            ESP_LOGI(LOOP_TAG, "Loop time_request_SCAN = %d, ready = %d, scan busy=%d", connect, ble_device_ready, scan_busy);
            //char* msg = "VER\r\n";
            //ble_send_bytes((uint8_t*)msg,strlen(msg));
            if(!connect && !scan_busy){
                ble_trigger_scan();
            }
        }
        // if (ble_device_ready)
        // {

        //     if (t - time_request_MSG >= tick_interval_MSG)
        //     {
        //         time_request_MSG = t;

        //         if (message_send_index == 0)
        //         {
        //             message_send_index = 1;
        //             tick_interval_MSG = 50;
        //             char *msg = "%%%\r\n";
        //             ble_send_bytes((uint8_t *)msg, strlen(msg));
        //         }
        //         else if (message_send_index == 1)
        //         {
        //             message_send_index = 2;
        //             tick_interval_MSG = 50;
        //             char *msg = "MIRRA\r\n";
        //             ble_send_bytes((uint8_t *)msg, strlen(msg));
        //         }
        //         else if (message_send_index == 2)
        //         {
        //             message_send_index = 3;
        //             tick_interval_MSG = 250;
        //             // char* msg = "ABCDEF\r\n";
        //             // ble_send_bytes((uint8_t*)msg,strlen(msg));
        //             if (strlen(connected_device_name) > 0)
        //             {
        //                 uint8_t msg[strlen(connected_device_name) + 2];
        //                 memcpy(msg, (uint8_t *)connected_device_name, strlen(connected_device_name));
        //                 msg[strlen(connected_device_name)] = '\r';
        //                 msg[strlen(connected_device_name) + 1] = '\n';
        //                 ble_send_bytes(msg, strlen(connected_device_name) + 2);
        //             }
        //         }
        //         else if (message_send_index == 3)
        //         {
        //             message_send_index = 4;
        //             tick_interval_MSG = 50;
        //             char *msg = "%%%\r\n";
        //             ble_send_bytes((uint8_t *)msg, strlen(msg));
        //         }
        //         else if (message_send_index == 4)
        //         {
        //             message_send_index = 5;
        //             tick_interval_MSG = 50;
        //             char *msg = "MIRRL\r\n";
        //             ble_send_bytes((uint8_t *)msg, strlen(msg));
        //         }
        //         else if (message_send_index == 5)
        //         {
        //             message_send_index = 6;
        //             tick_interval_MSG = 200;
        //             char *msg = "RMZ\r\n";
        //             ble_send_bytes((uint8_t *)msg, strlen(msg));
        //         }
        //         else if (message_send_index == 6)
        //         {
        //             message_send_index = 7;
        //             tick_interval_MSG = 500;
        //             char *msg = "RMS\r\n";
        //             ble_send_bytes((uint8_t *)msg, strlen(msg));
        //         }
        //         else if (message_send_index == 7)
        //         {
        //             message_send_index = 8;
        //             tick_interval_MSG = 50;
        //             char *msg = "RMC\r\n";
        //             ble_send_bytes((uint8_t *)msg, strlen(msg));
        //         }
        //         else if (message_send_index == 8)
        //         {
        //             message_send_index = 9;
        //             tick_interval_MSG = 200;
        //             char *msg = "%%%\r\n";
        //             ble_send_bytes((uint8_t *)msg, strlen(msg));
        //         }
        //     }
        // }
        int uart_rx_len = uart_read_bytes(uart_num, uart_rx_buffer, UART_RX_BUF_SIZE, 1);

        //Write data back to UART
        if (uart_rx_len > 4) {
            
            //char* msg = "abc\n";
            if(MODBUS_CRC_pass(uart_rx_buffer, uart_rx_len)){
                ESP_LOGI(UART1_TAG, "uart_read_bytes len=%d", uart_rx_len);
                if(uart_rx_buffer[0]==10){
                    if (uart_rx_buffer[1] == 0x03 && uart_rx_len == 8)
                    {
                        uint16_t reg_len = (uart_rx_buffer[4]<<8)+uart_rx_buffer[5];
                        
                        uint8_t msg[5+reg_len*2];
                        memset(msg, 0, 5+reg_len*2);
                        msg[0] = uart_rx_buffer[0];
                        msg[1] = 0x03;
                        msg[2] = (uint8_t)(reg_len*2);
                        //uint8_t msg[] = {uart_rx_buffer[0], 0x03, 0x06, 0xAE, 0x41, 0x56, 0x52, 0x43, 0x40, 0x49, 0xAD};
                        uint16_t crc16 = MODBUS_CRC16_V1(msg, 3+reg_len*2);
                        msg[3+reg_len*2] = (uint8_t)(crc16 & 0xFF);
                        msg[4+reg_len*2] = (uint8_t)(crc16 / 256);
                        if (uart_write_bytes(uart_num, msg, 5+reg_len*2) != 5+reg_len*2)
                        {
                            ESP_LOGE(UART1_TAG, "Send data critical failure.");
                        }
                    }
                    else if (uart_rx_buffer[1] == 0x10 && uart_rx_len > 9)
                    {
                        uint16_t reg_addr = (uart_rx_buffer[2]<<8)+uart_rx_buffer[3];
                        uint16_t reg_len = (uart_rx_buffer[4]<<8)+uart_rx_buffer[5];
                        
                        uint8_t msg[8];
                        memset(msg, 0, 8);
                        memcpy(msg, uart_rx_buffer, 6);
                        uint16_t crc16 = MODBUS_CRC16_V1(msg, 6);
                        msg[6] = (uint8_t)(crc16 & 0xFF);
                        msg[7] = (uint8_t)(crc16 / 256);
                        
                        if (uart_write_bytes(uart_num, msg, 8) != 8)
                        {
                            ESP_LOGE(UART1_TAG, "Send data critical failure.");
                        }
                    }
                }
            }
            else{
                ESP_LOGI(UART1_TAG, "uart bytes illigal, len=%d", uart_rx_len);
            }
            
        }
        if(can_bus_listen){
            can_loop();
        }
        
        vTaskDelay(1);
    }
}


static uint16_t MODBUS_CRC16_V1( const uint8_t *buf, uint16_t len )
{
  uint16_t crc1 = 0xFFFF;
  uint16_t i = 0;
  uint8_t b = 0;

  for ( i = 0; i < len; i++ )
  {
    crc1 ^= buf[i];

    for ( b = 0; b < 8; b++ )
    {
      if ( crc1 & 0x0001 )
      {
        crc1 >>= 1;
        crc1 ^= 0xA001;
      }
      else
      {
        crc1 >>= 1;
      }
    }
  }

  return crc1;
}
static bool MODBUS_CRC_pass(uint8_t* data, uint16_t data_len) {
  uint16_t crcValue = MODBUS_CRC16_V1(data, data_len - 2);
  if (((data[data_len - 1] << 8) + data[data_len - 2]) == crcValue) {

    return true;
  }
  return false;
}