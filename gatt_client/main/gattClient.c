#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "nvs.h"
#include "nvs_flash.h"
#include <ctype.h>
#include <math.h>

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "mqtt_client.h"

#define GATTC_TAG "GATTC_DEMO"
#define REMOTE_SERVICE_UUID        0x00FF
#define REMOTE_NOTIFY_CHAR_UUID    0xFF01
#define PROFILE_NUM      1
#define PROFILE_A_APP_ID 0
#define INVALID_HANDLE   0
#define JSON_PAYLOAD_SIZE 512 // ou até maior, dependendo do seu caso

//static uint8_t gatts_xxx_uuid128[ESP_UUID_LEN_128] = {0x06, 0x18, 0x7a, 0xec, 0xbe, 0x11, 0x11, 0xea, 0x00, 0x16, 0x02, 0x42, 0x01, 0x13, 0x00, 0x04};
//eb, e0, cc , b0, 7a0a, 4b0c, 8a1a, 6ff2997da3a6
// UUID: ebe0ccb0-7a0a-4b0c-8a1a-6ff2997da3a6
static uint8_t gatts_xxx_uuid128[ESP_UUID_LEN_128] = { 
    0xa6, 0xa3, 0x7d, 0x99, 0xf2, 0x6f, 0x1a, 0x8a, 
    0x0c, 0x4b, 0x0a, 0x7a, 0xb0, 0xcc, 0xe0, 0xeb 
};
//filtrar o serviço
static uint8_t gatts_uart_service_uuid128[ESP_UUID_LEN_128] = {
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
    0x93, 0xf3, 0xa3, 0xb5, 0x01, 0x00, 0x40, 0x6e
};
//filtrar a característica (enviar Tx / receber Rx)
static esp_bt_uuid_t remote_uart_tx_char_uuid = {
    .len = ESP_UUID_LEN_128,
    .uuid = {.uuid128 = { 
        0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
        0x93, 0xf3, 0xa3, 0xb5, 0x03, 0x00, 0x40, 0x6e
    }},
};


static esp_bt_uuid_t remote_uart_rx_char_uuid = {
    .len = ESP_UUID_LEN_128,
    .uuid = {.uuid128 = { 
        0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
        0x93, 0xf3, 0xa3, 0xb5, 0x02, 0x00, 0x40, 0x6e
    }},
};

const char *data = "LED1.set();\n";

//esp_gattc_char_elem_t rx_char_elem_result[256];
uint16_t rx_count = 0;

// static esp_bt_uuid_t service_uuid = {
//     .len = ESP_UUID_LEN_128,
//     .uuid = {.uuid128 = { 0xa6, 0xa3, 0x7d, 0x99, 0xf2, 0x6f, 0x1a, 0x8a, 0x0c, 0x4b, 0x0a, 0x7a, 0xb0, 0xcc, 0xe0, 0xeb }},
// };

static const char remote_device_name[] = "Bangle.js 0f66";
static esp_mqtt_client_handle_t mqtt_client = NULL;
static bool connect    = false;
static bool get_server = false;
static esp_gattc_char_elem_t *char_elem_result   = NULL;
static esp_gattc_descr_elem_t *descr_elem_result = NULL;
//extern esp_mqtt_client_handle_t mqtt_client;

/* Declare static functions */
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);
static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);


static esp_bt_uuid_t remote_filter_service_uuid = {
    .len = ESP_UUID_LEN_128,
    .uuid = {.uuid128 = { 0xa6, 0xa3, 0x7d, 0x99, 0xf2, 0x6f, 0x1a, 0x8a, 0x0c, 0x4b, 0x0a, 0x7a, 0xb0, 0xcc, 0xe0, 0xeb }},
};

static esp_bt_uuid_t remote_filter_char_uuid = {
    .len = ESP_UUID_LEN_128,
    .uuid = {.uuid128 = { 0xa6, 0xa3, 0x7d, 0x99, 0xf2, 0x6f, 0x1a, 0x8a, 0x0c, 0x4b, 0x0a, 0x7a, 0xc1, 0xcc, 0xe0, 0xeb }},
};

// static esp_bt_uuid_t notify_descr_uuid = {
//     .len = ESP_UUID_LEN_128,
//     .uuid = {.uuid128 = { 0xa6, 0xa3, 0x7d, 0x99, 0xf2, 0x6f, 0x1a, 0x8a, 0x0c, 0x4b, 0x0a, 0x7a, 0xb0, 0xcc, 0xe0, 0xeb }},
// };

static esp_bt_uuid_t notify_descr_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG,},
};

static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50,
    .scan_window            = 0x30,
    .scan_duplicate         = BLE_SCAN_DUPLICATE_DISABLE
};

struct gattc_profile_inst {
    esp_gattc_cb_t gattc_cb;
    uint16_t gattc_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_start_handle;
    uint16_t service_end_handle;
    uint16_t char_handle;
    esp_bd_addr_t remote_bda;

        // 👉 Adicione os dois campos abaixo:
    uint16_t tx_char_handle;
    uint16_t rx_char_handle;
};

/* One gatt-based profile one app_id and one gattc_if, this array will store the gattc_if returned by ESP_GATTS_REG_EVT */
static struct gattc_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_A_APP_ID] = {
        .gattc_cb = gattc_profile_event_handler,
        .gattc_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

static char g_time[32];
static char g_temp[32];
static char g_steps[32];
static char g_hr[32];
static char g_accel[64];

void vTaskSolicitaDados(void *pvParameters)
{
    while (1)
    {
        if (connect && gl_profile_tab[PROFILE_A_APP_ID].tx_char_handle != INVALID_HANDLE)
        {
            // 1. Timestamp
            const char *message_time = "print(getTime());\n";
            esp_err_t ret = esp_ble_gattc_write_char(
                gl_profile_tab[PROFILE_A_APP_ID].gattc_if,
                gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                gl_profile_tab[PROFILE_A_APP_ID].tx_char_handle,
                strlen(message_time),
                (uint8_t *)message_time,
                ESP_GATT_WRITE_TYPE_RSP,
                ESP_GATT_AUTH_REQ_NONE
            );

            vTaskDelay(100 / portTICK_PERIOD_MS); // Espaço entre comandos

            // 2. Temperatura
            const char *message_temp = "print(E.getTemperature());\n";
            ret = esp_ble_gattc_write_char(
                gl_profile_tab[PROFILE_A_APP_ID].gattc_if,
                gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                gl_profile_tab[PROFILE_A_APP_ID].tx_char_handle,
                strlen(message_temp),
                (uint8_t *)message_temp,
                ESP_GATT_WRITE_TYPE_RSP,
                ESP_GATT_AUTH_REQ_NONE
            );

            vTaskDelay(100 / portTICK_PERIOD_MS);

            // 3. Frequência cardíaca (BPM)
            const char *message_hr = "print(Bangle.getHealthStatus().bpm);\n";
            ret = esp_ble_gattc_write_char(
                gl_profile_tab[PROFILE_A_APP_ID].gattc_if,
                gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                gl_profile_tab[PROFILE_A_APP_ID].tx_char_handle,
                strlen(message_hr),
                (uint8_t *)message_hr,
                ESP_GATT_WRITE_TYPE_RSP,
                ESP_GATT_AUTH_REQ_NONE
            );
        }

        vTaskDelay(5000 / portTICK_PERIOD_MS); // intervalo entre ciclos
    }
}

void mqtt_app_start(void) {
    esp_mqtt_client_config_t mqtt_cfg = {
    //.broker.address.uri = "mqtt://localhost:1883",
    .broker.address.uri = "mqtt://192.168.27.101:1883",
    //.broker.address.uri = "mqtt://127.0.0.1:1883",
    };
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_start(mqtt_client);
}

float extract_first_float(const char *s) {
    while (*s) {
        if (isdigit((unsigned char)*s) || (*s == '-' && isdigit((unsigned char)*(s+1)))) {
            char *endptr;
            float val = strtof(s, &endptr);
            if (s != endptr) return val;
            s = endptr;
        } else {
            s++;
        }
    }
    return NAN;
}

static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    esp_ble_gattc_cb_param_t *p_data = (esp_ble_gattc_cb_param_t *)param;

    switch (event) {
    case ESP_GATTC_REG_EVT:
        ESP_LOGI(GATTC_TAG, "REG_EVT");
        esp_err_t scan_ret = esp_ble_gap_set_scan_params(&ble_scan_params);
        if (scan_ret){
            ESP_LOGE(GATTC_TAG, "set scan params error, error code = %x", scan_ret);
        }
        break;
    case ESP_GATTC_CONNECT_EVT:{
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_CONNECT_EVT conn_id %d, if %d", p_data->connect.conn_id, gattc_if);
        gl_profile_tab[PROFILE_A_APP_ID].conn_id = p_data->connect.conn_id;
        memcpy(gl_profile_tab[PROFILE_A_APP_ID].remote_bda, p_data->connect.remote_bda, sizeof(esp_bd_addr_t));
        ESP_LOGI(GATTC_TAG, "REMOTE BDA:");
        esp_log_buffer_hex(GATTC_TAG, gl_profile_tab[PROFILE_A_APP_ID].remote_bda, sizeof(esp_bd_addr_t));
        esp_err_t mtu_ret = esp_ble_gattc_send_mtu_req (gattc_if, p_data->connect.conn_id);
        if (mtu_ret){
            ESP_LOGE(GATTC_TAG, "config MTU error, error code = %x", mtu_ret);
        }
        break;
    }
    case ESP_GATTC_OPEN_EVT:
        if (param->open.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "open failed, status %d", p_data->open.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "open success");
        break;
    case ESP_GATTC_DIS_SRVC_CMPL_EVT:
        if (param->dis_srvc_cmpl.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "discover service failed, status %d", param->dis_srvc_cmpl.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "discover service complete conn_id %d", param->dis_srvc_cmpl.conn_id);
        esp_ble_gattc_search_service(gattc_if, param->dis_srvc_cmpl.conn_id, NULL);
        break;
    case ESP_GATTC_CFG_MTU_EVT:
        if (param->cfg_mtu.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG,"config mtu failed, error status = %x", param->cfg_mtu.status);
        }
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_CFG_MTU_EVT, Status %d, MTU %d, conn_id %d", param->cfg_mtu.status, param->cfg_mtu.mtu, param->cfg_mtu.conn_id);
        break;
    case ESP_GATTC_SEARCH_RES_EVT: {
        ESP_LOGI(GATTC_TAG, "SEARCH RES: conn_id = %x is primary service %d", p_data->search_res.conn_id, p_data->search_res.is_primary);
        ESP_LOGI(GATTC_TAG, "start handle %d end handle %d current handle value %d", p_data->search_res.start_handle, p_data->search_res.end_handle, p_data->search_res.srvc_id.inst_id);
        //encontrar o serviço definido acima
        if (p_data->search_res.srvc_id.uuid.len == ESP_UUID_LEN_128) {
            if (memcmp(p_data->search_res.srvc_id.uuid.uuid.uuid128, gatts_uart_service_uuid128, ESP_UUID_LEN_128) == 0) {
                ESP_LOGI(GATTC_TAG, "UART service found");
                get_server = true;
                gl_profile_tab[PROFILE_A_APP_ID].service_start_handle = p_data->search_res.start_handle;
                gl_profile_tab[PROFILE_A_APP_ID].service_end_handle = p_data->search_res.end_handle;
            }
        }

        break;
    }
    case ESP_GATTC_SEARCH_CMPL_EVT:
        if (p_data->search_cmpl.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "search service failed, error status = %x", p_data->search_cmpl.status);
            break;
        }
        if(p_data->search_cmpl.searched_service_source == ESP_GATT_SERVICE_FROM_REMOTE_DEVICE) {
            ESP_LOGI(GATTC_TAG, "Get service information from remote device");
        } else if (p_data->search_cmpl.searched_service_source == ESP_GATT_SERVICE_FROM_NVS_FLASH) {
            ESP_LOGI(GATTC_TAG, "Get service information from flash");
        } else {
            ESP_LOGI(GATTC_TAG, "unknown service source");
        }
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_SEARCH_CMPL_EVT");
        if (get_server){
            uint16_t count = 0;
            esp_gatt_status_t status = esp_ble_gattc_get_attr_count( gattc_if,
                                                                     p_data->search_cmpl.conn_id,
                                                                     ESP_GATT_DB_CHARACTERISTIC,
                                                                     gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                                                                     gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                                                                     INVALID_HANDLE,
                                                                     &count);
            if (status != ESP_GATT_OK){
                ESP_LOGE(GATTC_TAG, "esp_ble_gattc_get_attr_count error");
                break;
            }

            // ... seu código inicial ...

            if (count > 0) {
                char_elem_result = malloc(sizeof(esp_gattc_char_elem_t) * count);
                if (!char_elem_result) {
                    ESP_LOGE(GATTC_TAG, "gattc no mem");
                    break;
                }

                // --- Característica para ESCREVER no Bangle.js (Seu TX / Bangle.js RX) ---
                // Buscar pela UUID que o Bangle.js RECEBE (6E400002...)
                uint16_t current_count = count;
                status = esp_ble_gattc_get_char_by_uuid(
                    gattc_if,
                    p_data->search_cmpl.conn_id,
                    gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                    gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                    remote_uart_rx_char_uuid, // Use a UUID do RX do Bangle.js (onde você escreve)
                    char_elem_result,
                    &current_count
                );
                if (status == ESP_GATT_OK && current_count > 0) {
                    gl_profile_tab[PROFILE_A_APP_ID].tx_char_handle = char_elem_result[0].char_handle; // Armazenar como seu TX
                    ESP_LOGI(GATTC_TAG, "Bangle RX (My TX) handle: 0x%04x, Properties: 0x%02x",
                            gl_profile_tab[PROFILE_A_APP_ID].tx_char_handle, char_elem_result[0].properties);
                } else {
                    ESP_LOGW(GATTC_TAG, "Bangle RX (My TX) characteristic not found (UUID 6E400002...).");
                }

                // --- Característica para RECEBER NOTIFICAÇÕES do Bangle.js (Seu RX / Bangle.js TX) ---
                // Buscar pela UUID que o Bangle.js TRANSMITE (6E400003...)
                current_count = count; // Resetar count para a próxima busca
                status = esp_ble_gattc_get_char_by_uuid(
                    gattc_if,
                    p_data->search_cmpl.conn_id,
                    gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
                    gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
                    remote_uart_tx_char_uuid, // Use a UUID do TX do Bangle.js (onde você recebe notificações)
                    char_elem_result,
                    &current_count
                );
                if (status == ESP_GATT_OK && current_count > 0) {
                    gl_profile_tab[PROFILE_A_APP_ID].rx_char_handle = char_elem_result[0].char_handle; // Armazenar como seu RX
                    ESP_LOGI(GATTC_TAG, "Bangle TX (My RX) handle: 0x%04x, Properties: 0x%02x",
                            gl_profile_tab[PROFILE_A_APP_ID].rx_char_handle, char_elem_result[0].properties);

                    if (char_elem_result[0].properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY) {
                        ESP_LOGI(GATTC_TAG, "Characteristic supports NOTIFY. Registering for notify on Bangle TX (My RX).");
                        esp_ble_gattc_register_for_notify(
                            gattc_if,
                            gl_profile_tab[PROFILE_A_APP_ID].remote_bda,
                            gl_profile_tab[PROFILE_A_APP_ID].rx_char_handle);
                    } else {
                        ESP_LOGW(GATTC_TAG, "Bangle TX (My RX) characteristic does NOT support NOTIFY.");
                    }
                } else {
                    ESP_LOGW(GATTC_TAG, "Bangle TX (My RX) characteristic not found (UUID 6E400003...).");
                }

                free(char_elem_result);
            } else {
                ESP_LOGE(GATTC_TAG, "no char found");
            }
        }
        break;
    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_REG_FOR_NOTIFY_EVT");

        if (p_data->reg_for_notify.status != ESP_GATT_OK) {
            ESP_LOGE(GATTC_TAG, "REG FOR NOTIFY falhou: status = %d", p_data->reg_for_notify.status);
            break;
        }

        uint16_t count = 0;
        esp_gatt_status_t ret_status = esp_ble_gattc_get_attr_count(
            gattc_if,
            gl_profile_tab[PROFILE_A_APP_ID].conn_id,
            ESP_GATT_DB_DESCRIPTOR,
            gl_profile_tab[PROFILE_A_APP_ID].service_start_handle,
            gl_profile_tab[PROFILE_A_APP_ID].service_end_handle,
            gl_profile_tab[PROFILE_A_APP_ID].rx_char_handle,
            &count
        );

        if (ret_status != ESP_GATT_OK) {
            ESP_LOGE(GATTC_TAG, "Erro ao obter contagem de descritores");
            break;
        }

        if (count > 0) {
            esp_gattc_descr_elem_t *descr_elem_result = malloc(sizeof(esp_gattc_descr_elem_t) * count);
            if (!descr_elem_result) {
                ESP_LOGE(GATTC_TAG, "Erro malloc para descritores");
                break;
            }

            ret_status = esp_ble_gattc_get_descr_by_char_handle(
                gattc_if,
                gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                gl_profile_tab[PROFILE_A_APP_ID].rx_char_handle,
                notify_descr_uuid,
                descr_elem_result,
                &count
            );

            if (ret_status != ESP_GATT_OK) {
                ESP_LOGE(GATTC_TAG, "Erro ao obter descritor por handle");
                free(descr_elem_result);
                break;
            }

            if (count > 0) {
                uint16_t notify_en = 1;

                ret_status = esp_ble_gattc_write_char_descr(
                    gattc_if,
                    gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                    descr_elem_result[0].handle,
                    sizeof(notify_en),
                    (uint8_t *)&notify_en,
                    ESP_GATT_WRITE_TYPE_RSP,
                    ESP_GATT_AUTH_REQ_NONE
                );

                if (ret_status != ESP_GATT_OK) {
                    ESP_LOGE(GATTC_TAG, "Erro ao habilitar notificações");
                    free(descr_elem_result);
                    break;
                }

                ESP_LOGI(GATTC_TAG, "Notificações habilitadas no handle: 0x%04x", gl_profile_tab[PROFILE_A_APP_ID].rx_char_handle);

                // Enviar comando para ativar sensor de frequência cardíaca no Bangle.js
                const char *cmd_activate_hrm = "Bangle.setHRMPower(1);\n";
                esp_err_t ret = esp_ble_gattc_write_char(
                    gattc_if,
                    gl_profile_tab[PROFILE_A_APP_ID].conn_id,
                    gl_profile_tab[PROFILE_A_APP_ID].tx_char_handle,
                    strlen(cmd_activate_hrm),
                    (uint8_t *)cmd_activate_hrm,
                    ESP_GATT_WRITE_TYPE_RSP,
                    ESP_GATT_AUTH_REQ_NONE
                );
                if (ret == ESP_OK) {
                    ESP_LOGI(GATTC_TAG, "Comando para ativar HRM enviado.");
                } else {
                    ESP_LOGE(GATTC_TAG, "Falha ao enviar comando para ativar HRM: %d", ret);
                }

                // Criar a task que enviará os comandos periódicos para os sensores (timestamp, temperatura e HRM)
                BaseType_t result = xTaskCreate(
                    vTaskSolicitaDados,
                    "SolicitaDados",
                    4096,
                    NULL,
                    5,
                    NULL
                );

                if (result != pdPASS) {
                    ESP_LOGE(GATTC_TAG, "Falha ao criar task vTaskSolicitaDados");
                } else {
                    ESP_LOGI(GATTC_TAG, "Task vTaskSolicitaDados iniciada");
                }

            } else {
                ESP_LOGE(GATTC_TAG, "Nenhum descritor CCC encontrado para RX characteristic");
            }

            free(descr_elem_result);
        } else {
            ESP_LOGE(GATTC_TAG, "Nenhum descritor encontrado para RX characteristic");
        }

        break;
    }


    case ESP_GATTC_NOTIFY_EVT: {
        char raw_msg[p_data->notify.value_len + 1];
        memcpy(raw_msg, p_data->notify.value, p_data->notify.value_len);
        raw_msg[p_data->notify.value_len] = '\0';

        ESP_LOGI(GATTC_TAG, "Received notification: %s", raw_msg);

        char *line = strtok(raw_msg, "\n");
        while (line != NULL) {
            float val = extract_first_float(line);

            if (!isnan(val)) {
                if (val > 100000) { // timestamp UNIX
                    char payload[64];
                    snprintf(payload, sizeof(payload), "{\"timestamp\":%.0f}", val);
                    esp_mqtt_client_publish(mqtt_client, "banglejs2/time", payload, 0, 1, 0);
                    ESP_LOGI("banglejs2/time", "%s", payload);

                } else if (val >= 10 && val < 50) { // Temperatura em ºC
                    char payload[64];
                    snprintf(payload, sizeof(payload), "{\"temperature\":%.2f}", val);
                    esp_mqtt_client_publish(mqtt_client, "banglejs2/data", payload, 0, 1, 0);
                    ESP_LOGI("banglejs2/data", "%s", payload);

                } else if (val >= 50 && val <= 220) { // Batimentos cardíacos BPM
                    char payload[64];
                    snprintf(payload, sizeof(payload), "{\"bpm\":%.0f}", val);
                    esp_mqtt_client_publish(mqtt_client, "banglejs2/hr", payload, 0, 1, 0);
                    ESP_LOGI("banglejs2/hr", "%s", payload);

                } else {
                    ESP_LOGI(GATTC_TAG, "Linha ignorada (valor fora dos limites): %s", line);
                }
            } else {
                ESP_LOGI(GATTC_TAG, "Linha ignorada (sem float): %s", line);
            }
            line = strtok(NULL, "\n");
        }
        break;
    }

    //modelo solicitar informações
    case ESP_GATTC_WRITE_DESCR_EVT:
        if (p_data->write.status != ESP_GATT_OK) {
            ESP_LOGE(GATTC_TAG, "write descr failed, error status = %x", p_data->write.status);
            break;
        }

        ESP_LOGI(GATTC_TAG, "write descr success ");
        // Enviar primeiro comando de tempo
        const char *message_time = "print(getTime());\n";
        esp_ble_gattc_write_char(gattc_if,
            gl_profile_tab[PROFILE_A_APP_ID].conn_id,
            gl_profile_tab[PROFILE_A_APP_ID].tx_char_handle,
            strlen(message_time),
            (uint8_t *)message_time,
            ESP_GATT_WRITE_TYPE_RSP,
            ESP_GATT_AUTH_REQ_NONE);

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
        ESP_LOGI(GATTC_TAG, "write char success ");
        break;
    case ESP_GATTC_DISCONNECT_EVT:
        connect = false;
        get_server = false;
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_DISCONNECT_EVT, reason = %d", p_data->disconnect.reason);
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
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
        //the unit of the duration is second
        uint32_t duration = 30;
        esp_ble_gap_start_scanning(duration);
        break;
    }
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        //scan start complete event to indicate scan start successfully or failed
        if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTC_TAG, "scan start failed, error status = %x", param->scan_start_cmpl.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "scan start success");

        break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        switch (scan_result->scan_rst.search_evt) {
        case ESP_GAP_SEARCH_INQ_RES_EVT:
            esp_log_buffer_hex(GATTC_TAG, scan_result->scan_rst.bda, 6);
            ESP_LOGI(GATTC_TAG, "searched Adv Data Len %d, Scan Response Len %d", scan_result->scan_rst.adv_data_len, scan_result->scan_rst.scan_rsp_len);
            adv_name = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv,
                                                ESP_BLE_AD_TYPE_NAME_CMPL, &adv_name_len);
            ESP_LOGI(GATTC_TAG, "searched Device Name Len %d", adv_name_len);
            esp_log_buffer_char(GATTC_TAG, adv_name, adv_name_len);

#if CONFIG_EXAMPLE_DUMP_ADV_DATA_AND_SCAN_RESP
            if (scan_result->scan_rst.adv_data_len > 0) {
                ESP_LOGI(GATTC_TAG, "adv data:");
                esp_log_buffer_hex(GATTC_TAG, &scan_result->scan_rst.ble_adv[0], scan_result->scan_rst.adv_data_len);
            }
            if (scan_result->scan_rst.scan_rsp_len > 0) {
                ESP_LOGI(GATTC_TAG, "scan resp:");
                esp_log_buffer_hex(GATTC_TAG, &scan_result->scan_rst.ble_adv[scan_result->scan_rst.adv_data_len], scan_result->scan_rst.scan_rsp_len);
            }
#endif
            ESP_LOGI(GATTC_TAG, " ");

            if (adv_name != NULL) {
                if (strlen(remote_device_name) == adv_name_len && strncmp((char *)adv_name, remote_device_name, adv_name_len) == 0) {
                    ESP_LOGI(GATTC_TAG, "searched device %s", remote_device_name);
                    if (connect == false) {
                        connect = true;
                        ESP_LOGI(GATTC_TAG, "connect to the remote device.");
                        esp_ble_gap_stop_scanning();
                        esp_ble_gattc_open(gl_profile_tab[PROFILE_A_APP_ID].gattc_if, scan_result->scan_rst.bda, scan_result->scan_rst.ble_addr_type, true);
                    }
                }
            }
            break;
        case ESP_GAP_SEARCH_INQ_CMPL_EVT:
            break;
        default:
            break;
        }
        break;
    }

    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        if (param->scan_stop_cmpl.status != ESP_BT_STATUS_SUCCESS){
            ESP_LOGE(GATTC_TAG, "scan stop failed, error status = %x", param->scan_stop_cmpl.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "stop scan successfully");
        break;

    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS){
            ESP_LOGE(GATTC_TAG, "adv stop failed, error status = %x", param->adv_stop_cmpl.status);
            break;
        }
        ESP_LOGI(GATTC_TAG, "stop adv successfully");
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
         ESP_LOGI(GATTC_TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
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

static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    /* If event is register event, store the gattc_if for each profile */
    if (event == ESP_GATTC_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile_tab[param->reg.app_id].gattc_if = gattc_if;
        } else {
            ESP_LOGI(GATTC_TAG, "reg app failed, app_id %04x, status %d",
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

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"

// ... Seus includes de BLE e demais features
// Ex: #include "mqtt_client.h" etc.

#define EXAMPLE_WIFI_SSID "FRANCISCA"
#define EXAMPLE_WIFI_PASS "00254180"

static const char *TAG = "MAIN";

static EventGroupHandle_t wifi_event_group;
const int WIFI_CONNECTED_BIT = BIT0;

extern void mqtt_app_start(void);  // Já existe no seu código!

// Handle de evento Wi-Fi/IP
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                              int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
        ESP_LOGI(TAG, "WiFi conectado, IP obtido.");
        mqtt_app_start();  // Só aqui inicia MQTT!
    }
}

static void wifi_init_sta(void)
{
    wifi_event_group = xEventGroupCreate();

    // Inicializações do ESP-IDF (sempre nessa ordem!)
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                ESP_EVENT_ANY_ID,
                &wifi_event_handler,
                NULL,
                NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                IP_EVENT_STA_GOT_IP,
                &wifi_event_handler,
                NULL,
                NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_WIFI_SSID,
            .password = EXAMPLE_WIFI_PASS,
            // .threshold.authmode = WIFI_AUTH_WPA2_PSK, // coloque se sua rede exigir
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );
    ESP_LOGI(TAG, "wifi_init_sta finalizado.");
}

// Sua main adaptada
void app_main(void)
{
    // Inicializa NVS (necessário antes de WiFi/BLE)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Inicialize BLE do jeito padrão (já está no seu código)
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG, "BT Controller init failed: %s", esp_err_to_name(ret));
        return;
    }
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(TAG, "BT Controller enable failed: %s", esp_err_to_name(ret));
        return;
    }
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(TAG, "Bluedroid init failed: %s", esp_err_to_name(ret));
        return;
    }
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(TAG, "Bluedroid enable failed: %s", esp_err_to_name(ret));
        return;
    }
    // Callbacks do GAP/GATTC
    ret = esp_ble_gap_register_callback(esp_gap_cb);
    if (ret) {
        ESP_LOGE(TAG, "gap register failed, error code = %x", ret);
        return;
    }
    ret = esp_ble_gattc_register_callback(esp_gattc_cb);
    if(ret){
        ESP_LOGE(TAG, "gattc register failed, error code = %x", ret);
        return;
    }
    ret = esp_ble_gattc_app_register(PROFILE_A_APP_ID);
    if (ret){
        ESP_LOGE(TAG, "gattc app register failed, error code = %x", ret);
        return;
    }
    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret){
        ESP_LOGE(TAG, "set local MTU failed, error code = %x", local_mtu_ret);
    }

    // 1) Inicialize Wi-Fi e aguarde evento de IP!
    wifi_init_sta();

    // 2) NÃO chame mqtt_app_start() aqui!
    // Ele será iniciado automaticamente, APENAS após o Wi-Fi conectar.

    // 3) BLE segue funcionando, MQTT só conecta quando Wi-Fi ok!
}
