#include "ble_driver.h"
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Các thư viện lõi của NimBLE BLE Stack
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "BLE_DRIVER";
static uint8_t ble_addr_type;

// Cờ báo trạng thái xác thực chủ xe toàn cục
static bool g_is_paired_successfully = false;

// Khai báo trước các hàm nội bộ (Forward Declaration)
static void ble_app_advertise(void);
static void ble_app_on_sync(void);
static void ble_host_task(void *param);
static int ble_gap_event(struct ble_gap_event *event, void *arg);
static int ble_custom_gatt_handler(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);

// --- ĐỊNH NGHĨA HỆ THỐNG GATT SERVICE BẢO MẬT ---
#define CUSTOM_SERVICE_UUID         0x78, 0x56, 0x34, 0x12, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12
#define CUSTOM_CHARACTERISTIC_UUID  0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF

static const struct ble_gatt_svc_def ble_custom_services[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
       .uuid = BLE_UUID128_DECLARE(CUSTOM_SERVICE_UUID),
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = BLE_UUID128_DECLARE(CUSTOM_CHARACTERISTIC_UUID),
                .access_cb = ble_custom_gatt_handler,
                .flags = BLE_GATT_CHR_F_WRITE, 
            },
            {0} 
        }
    },
    {0} 
};

// Hàm cấu hình cấu trúc dữ liệu phát quảng bá BLE
static void ble_app_advertise(void) {
    struct ble_hs_adv_fields fields;
    int rc;

    memset(&fields, 0, sizeof(fields));

    // 1. Cấu hình Flags quảng bá công khai
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    // 2. Chèn Custom 128-bit UUID vào gói tin quảng bá quảng cáo
    ble_uuid128_t uuid = BLE_UUID128_INIT(CUSTOM_SERVICE_UUID);
    fields.uuids128 = &uuid;
    fields.num_uuids128 = 1;
    fields.uuids128_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Loi cau hinh goi tin Adv; rc=%d", rc);
        return;
    }

    // 3. Cấu hình gói phản hồi quét (Scan Response) để gửi Tên thiết bị xe máy
    struct ble_hs_adv_fields rsp_fields;
    memset(&rsp_fields, 0, sizeof(rsp_fields));
    
    const char *device_name = "Vario-Smartkey"; 
    rsp_fields.name = (uint8_t *)device_name;
    rsp_fields.name_len = strlen(device_name);
    rsp_fields.name_is_complete = 1;

    rc = ble_gap_adv_rsp_set_fields(&rsp_fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "Loi cau hinh goi tin Scan Response; rc=%d", rc);
        return;
    }

    // 4. Định nghĩa tham số phát quảng bá và cài đặt hàm bắt sự kiện GAP Event
    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND; 
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN; 

    rc = ble_gap_adv_start(ble_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Loi bat dau phat BLE; rc=%d", rc);
        return;
    }
    ESP_LOGI(TAG, "BLE dang phat UUID va ten: %s thanh cong!", device_name);
}

// Hàm Callback xử lý các sự kiện kết nối của Bluetooth (GAP)
static int ble_gap_event(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            ESP_LOGI(TAG, "⚡ Co thiet bi ket noi vao mang! Cho chuoi key tu App Flutter...");
            break;

        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGW(TAG, "❌ Thiet bi da ngat ket noi BLE.");
            g_is_paired_successfully = false; 
            break;
            
        case BLE_GAP_EVENT_ADV_COMPLETE:
            ESP_LOGI(TAG, "Chu ky phat quang ba ket thuc.");
            break;
    }
    return 0;
}

// KÊNH BẢO MẬT: Hàm xử lý chính xác khi App Flutter ghi dữ liệu Key xuống ESP32-C3
static int ble_custom_gatt_handler(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        char received_key[32] = {0};
        uint16_t len = OS_MBUF_PKTLEN(ctxt->om);

        if (len < sizeof(received_key) - 1) {
            ble_hs_mbuf_to_flat(ctxt->om, received_key, len, NULL);
            received_key[len] = '\0';
            
            ESP_LOGI(TAG, "Nhan duoc mat ma tu App: %s", received_key);

            // 🔥 ĐÃ SỬA: Đồng bộ so sánh chuẩn xác biến received_key
            if (strcmp(received_key, "ThuanAn_Vario_2026") == 0) 
            {
                ESP_LOGI(TAG, "==> CHÍNH XÁC: DA XAC THUC CHU XE THANH CONG!");
                g_is_paired_successfully = true;

                // 🔥 KÍCH HOẠT HÀM CẦU NỐI ĐỂ NHẢ RELAY MỞ KHÓA XE TRONG VÀI PHẦN TRIỆU GIÂY
                void main_system_auth_success(void); 
                main_system_auth_success();          
                
                return 0;
            } 
            else 
            {
                g_is_paired_successfully = false;
                ESP_LOGE(TAG, "==> CANH BAO: SAI MAT MA BAO MAT!");
                return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
            }
        }
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }
    return BLE_ATT_ERR_REQ_NOT_SUPPORTED;
}

// Callback xử lý khi NimBLE Host đồng bộ thành công với Controller hệ thống
static void ble_app_on_sync(void) {
    ble_hs_id_infer_auto(0, &ble_addr_type);
    ESP_LOGI(TAG, "NimBLE Stack da dong bo hoa san sang nhan lenh phat.");
}

// Task FreeRTOS chạy nền duy trì luồng xử lý giao thức BLE
static void ble_host_task(void *param) {
    ESP_LOGI(TAG, "BLE Host Task dang chay...");
    nimble_port_run(); 
    nimble_port_freertos_deinit();
}

// --- TRIỂN KHAI CÁC HÀM CÔNG KHAI ---

void ble_driver_init(void) {
    ESP_ERROR_CHECK(nimble_port_init());

    ble_svc_gap_device_name_set("Vario-Smartkey");

    ble_svc_gatt_init();
    int rc = ble_gatts_count_cfg(ble_custom_services);
    if (rc == 0) {
        rc = ble_gatts_add_svcs(ble_custom_services);
    }
    if (rc != 0) {
        ESP_LOGE(TAG, "Loi dang ky GATT Service; rc=%d", rc);
        return;
    }

    ble_hs_cfg.sync_cb = ble_app_on_sync;
    nimble_port_freertos_init(ble_host_task);
}

void ble_driver_start_advertising(void) {
    g_is_paired_successfully = false; 
    ESP_LOGI(TAG, "Kich hoat lenh phat quang ba tu Luong Dieu Khien...");
    ble_hs_id_infer_auto(0, &ble_addr_type); 
    ble_app_advertise();
}

void ble_driver_stop_advertising(void) {
    ble_gap_adv_stop();
    ESP_LOGW(TAG, "Da chu dong dung phat quang ba BLE de chuan bi di ngu an toan.");
}

bool ble_is_app_paired_successfully(void) {
    return g_is_paired_successfully;
}