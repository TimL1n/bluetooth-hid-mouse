/*
 * Lab 4.3: Bluetooth Motion-Controlled Mouse (ICM-42670-P + ESP32-C3)
 * Integrates Labs 4.1 (accelerometer) and 4.2 (BLE HID mouse)
 * Features: Tilt control, speed levels, acceleration, BOOT button click
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_hidd_prf_api.h"
#include "hid_dev.h"

#define TAG "LAB4_3"

// ========== BOOT BUTTON ==========
#define BOOT_BUTTON_GPIO    9
#define BUTTON_ACTIVE_LEVEL 0

// ========== I2C Configuration ==========
#define I2C_MASTER_SCL_IO   8
#define I2C_MASTER_SDA_IO   10
#define I2C_MASTER_NUM      I2C_NUM_0
#define I2C_MASTER_FREQ_HZ  100000

// ========== ICM-42670-P Registers ==========
#define REG_DEVICE_CONFIG    0x11
#define REG_WHO_AM_I         0x75
#define REG_PWR_MGMT0        0x1F
#define REG_ACCEL_CONFIG0    0x21
#define REG_ACCEL_X1         0x0B
#define REG_ACCEL_Y1         0x0D
#define REG_ACCEL_Z1         0x0F
#define WHOAMI_EXPECT        0x67

// ========== Tilt Tuning ==========
#define DEADZONE_LSB         800   // Ignore small movements
#define A_BIT_THRESHOLD      3000  // A_BIT tilted
#define A_LOT_THRESHOLD      8000  // A_LOT tilted

// ========== Mouse Control ==========
#define MOUSE_UPDATE_MS      20    // 50 Hz update rate
#define ACCEL_TIME_STEP_MS   100   // Time increment for acceleration

// Mouse speed levels
#define SPEED_A_BIT          3     // Slow movement
#define SPEED_A_LOT          8     // Fast movement

// Acceleration multipliers
#define ACCEL_LEVEL_1        1
#define ACCEL_LEVEL_2        2
#define ACCEL_LEVEL_3        4
#define ACCEL_LEVEL_4        6

// ========== Global State ==========
static uint8_t s_icm_addr = 0x68;
static uint16_t s_hid_conn_id = 0;
static bool s_secured = false;
static bool s_button_pressed = false;

// Acceleration tracking
typedef struct {
    int32_t consecutive_time_ms;
    uint8_t accel_multiplier;
} accel_state_t;

static accel_state_t accel_x = {0, 1};
static accel_state_t accel_y = {0, 1};

// ========== I2C Functions ==========
static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        .clk_flags = 0,
    };
    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK) return ret;
    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) return ret;
    i2c_set_timeout(I2C_MASTER_NUM, 0xFFFFF);
    return ESP_OK;
}

static esp_err_t i2c_cmd_begin_retry(i2c_cmd_handle_t cmd)
{
    const int max_attempts = 3;
    for (int attempt = 1; attempt <= max_attempts; ++attempt) {
        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
        if (ret == ESP_OK) return ESP_OK;
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    return ESP_FAIL;
}

static esp_err_t i2c_write_u8(uint8_t dev, uint8_t reg, uint8_t val)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, val, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_cmd_begin_retry(cmd);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t i2c_read_u8(uint8_t dev, uint8_t reg, uint8_t *val)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, val, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_cmd_begin_retry(cmd);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t read_axis_2b(uint8_t msb_reg, int16_t *out)
{
    uint8_t hi = 0, lo = 0;
    esp_err_t r = i2c_read_u8(s_icm_addr, msb_reg, &hi);
    if (r != ESP_OK) return r;
    r = i2c_read_u8(s_icm_addr, msb_reg + 1, &lo);
    if (r != ESP_OK) return r;
    *out = (int16_t)((hi << 8) | lo);
    return ESP_OK;
}

static esp_err_t read_accel(int16_t *ax, int16_t *ay, int16_t *az)
{
    esp_err_t r;
    r = read_axis_2b(REG_ACCEL_X1, ax); if (r != ESP_OK) return r;
    r = read_axis_2b(REG_ACCEL_Y1, ay); if (r != ESP_OK) return r;
    r = read_axis_2b(REG_ACCEL_Z1, az); if (r != ESP_OK) return r;
    return ESP_OK;
}

// ========== IMU Initialization ==========
static bool probe_addr(uint8_t addr)
{
    uint8_t who = 0;
    if (i2c_read_u8(addr, REG_WHO_AM_I, &who) == ESP_OK && who == WHOAMI_EXPECT) {
        s_icm_addr = addr;
        ESP_LOGI(TAG, "ICM-42670 at 0x%02X (WHO_AM_I=0x%02X)", addr, who);
        return true;
    }
    return false;
}

static esp_err_t icm_init(void)
{
    ESP_LOGI(TAG, "Scanning for ICM-42670...");
    if (!probe_addr(0x68) && !probe_addr(0x69)) {
        ESP_LOGE(TAG, "ICM-42670 not found");
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "Soft reset...");
    i2c_write_u8(s_icm_addr, REG_DEVICE_CONFIG, 0x01);
    vTaskDelay(pdMS_TO_TICKS(20));

    uint8_t who;
    i2c_read_u8(s_icm_addr, REG_WHO_AM_I, &who);
    if (who != WHOAMI_EXPECT) {
        ESP_LOGE(TAG, "WHO_AM_I mismatch: 0x%02X", who);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Enabling accelerometer...");
    i2c_write_u8(s_icm_addr, REG_PWR_MGMT0, 0x03);
    vTaskDelay(pdMS_TO_TICKS(10));
    i2c_write_u8(s_icm_addr, REG_ACCEL_CONFIG0, 0x69);
    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_LOGI(TAG, "Sensor ready");
    return ESP_OK;
}

// ========== Button Functions ==========
static void IRAM_ATTR button_isr_handler(void* arg)
{
    s_button_pressed = true;
}

static void button_init(void)
{
    gpio_config_t btn_conf = {
        .pin_bit_mask = (1ULL << BOOT_BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,  // Trigger on press
    };
    gpio_config(&btn_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BOOT_BUTTON_GPIO, button_isr_handler, NULL);
    ESP_LOGI(TAG, "BOOT button initialized on GPIO%d", BOOT_BUTTON_GPIO);
}

// ========== Acceleration Calculation ==========
static uint8_t calculate_acceleration(accel_state_t *state, bool is_tilted)
{
    if (!is_tilted) {
        state->consecutive_time_ms = 0;
        state->accel_multiplier = ACCEL_LEVEL_1;
        return ACCEL_LEVEL_1;
    }

    // Increment time tilted
    state->consecutive_time_ms += MOUSE_UPDATE_MS;

    // Determine acceleration level based on time
    if (state->consecutive_time_ms < 100) {
        state->accel_multiplier = ACCEL_LEVEL_1;
    } else if (state->consecutive_time_ms < 300) {
        state->accel_multiplier = ACCEL_LEVEL_2;
    } else if (state->consecutive_time_ms < 600) {
        state->accel_multiplier = ACCEL_LEVEL_3;
    } else {
        state->accel_multiplier = ACCEL_LEVEL_4;
    }

    return state->accel_multiplier;
}

// ========== Mouse Control Task ==========
static void mouse_control_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Mouse control task started");
    vTaskDelay(pdMS_TO_TICKS(1000));

    int16_t ax, ay, az;
    int8_t mouse_dx, mouse_dy;

    while (1) {
        // Handle button click
        if (s_button_pressed && s_secured) {
            s_button_pressed = false;
            ESP_LOGI(TAG, "CLICK!");
            esp_hidd_send_mouse_value(s_hid_conn_id, 0x01, 0, 0);  // Left button down
            vTaskDelay(pdMS_TO_TICKS(50));
            esp_hidd_send_mouse_value(s_hid_conn_id, 0x00, 0, 0);  // Release
        }

        if (!s_secured) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // Read accelerometer
        if (read_accel(&ax, &ay, &az) != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(MOUSE_UPDATE_MS));
            continue;
        }

        // Apply deadzone
        if (ax > -DEADZONE_LSB && ax < DEADZONE_LSB) ax = 0;
        if (ay > -DEADZONE_LSB && ay < DEADZONE_LSB) ay = 0;

        // Calculate X movement (LEFT/RIGHT)
        mouse_dx = 0;
        bool x_tilted = false;
        if (ax < -A_LOT_THRESHOLD) {
            // A_LOT_LEFT
            x_tilted = true;
            mouse_dx = -SPEED_A_LOT;
        } else if (ax < -A_BIT_THRESHOLD) {
            // A_BIT_LEFT
            x_tilted = true;
            mouse_dx = -SPEED_A_BIT;
        } else if (ax > A_LOT_THRESHOLD) {
            // A_LOT_RIGHT
            x_tilted = true;
            mouse_dx = SPEED_A_LOT;
        } else if (ax > A_BIT_THRESHOLD) {
            // A_BIT_RIGHT
            x_tilted = true;
            mouse_dx = SPEED_A_BIT;
        }

        // Apply X acceleration
        uint8_t x_accel = calculate_acceleration(&accel_x, x_tilted);
        mouse_dx *= x_accel;

        // Calculate Y movement (UP/DOWN)
        mouse_dy = 0;
        bool y_tilted = false;
        if (ay < -A_LOT_THRESHOLD) {
            // A_LOT_DOWN
            y_tilted = true;
            mouse_dy = SPEED_A_LOT;
        } else if (ay < -A_BIT_THRESHOLD) {
            // A_BIT_DOWN
            y_tilted = true;
            mouse_dy = SPEED_A_BIT;
        } else if (ay > A_LOT_THRESHOLD) {
            // A_LOT_UP
            y_tilted = true;
            mouse_dy = -SPEED_A_LOT;
        } else if (ay > A_BIT_THRESHOLD) {
            // A_BIT_UP
            y_tilted = true;
            mouse_dy = -SPEED_A_BIT;
        }

        // Apply Y acceleration
        uint8_t y_accel = calculate_acceleration(&accel_y, y_tilted);
        mouse_dy *= y_accel;

        // Send mouse movement
        if (mouse_dx != 0 || mouse_dy != 0) {
            esp_hidd_send_mouse_value(s_hid_conn_id, 0, mouse_dx, mouse_dy);
            
            // Debug output (occasional)
            static int dbg_count = 0;
            if ((++dbg_count % 25) == 0) {
                ESP_LOGI(TAG, "Move: dx=%d (accel=%d), dy=%d (accel=%d)", 
                         mouse_dx, x_accel, mouse_dy, y_accel);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(MOUSE_UPDATE_MS));
    }
}

// ========== BLE Callbacks ==========
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT: {
        esp_ble_adv_params_t adv_params = {
            .adv_int_min = 0x20,
            .adv_int_max = 0x40,
            .adv_type = ADV_TYPE_IND,
            .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
            .channel_map = ADV_CHNL_ALL,
            .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
        };
        esp_ble_gap_start_advertising(&adv_params);
        break;
    }
    case ESP_GAP_BLE_AUTH_CMPL_EVT:
        s_secured = param->ble_security.auth_cmpl.success;
        ESP_LOGI(TAG, "Pairing %s", s_secured ? "SUCCESS" : "FAILED");
        break;
    default:
        break;
    }
}

static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param)
{
    switch (event) {
    case ESP_HIDD_EVENT_REG_FINISH:
        if (param->init_finish.state == ESP_HIDD_INIT_OK) {
            esp_ble_gap_set_device_name("ESP32C3_Mouse");
            
            uint8_t hidd_service_uuid128[] = {
                0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
                0x00, 0x10, 0x00, 0x00, 0x12, 0x18, 0x00, 0x00,
            };
            
            esp_ble_adv_data_t adv_data = {
                .set_scan_rsp = false,
                .include_name = true,
                .include_txpower = true,
                .min_interval = 0x0006,
                .max_interval = 0x0010,
                .appearance = ESP_BLE_APPEARANCE_GENERIC_HID,
                .service_uuid_len = sizeof(hidd_service_uuid128),
                .p_service_uuid = hidd_service_uuid128,
                .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
            };
            
            esp_ble_gap_config_adv_data(&adv_data);
        }
        break;
    case ESP_HIDD_EVENT_BLE_CONNECT:
        s_hid_conn_id = param->connect.conn_id;
        ESP_LOGI(TAG, "BLE Connected");
        break;
    case ESP_HIDD_EVENT_BLE_DISCONNECT:
        s_secured = false;
        ESP_LOGI(TAG, "BLE Disconnected, restarting advertising");
        break;
    default:
        break;
    }
}

// ========== Main ==========
void app_main(void)
{
    ESP_LOGI(TAG, "Lab 4.3: Motion-Controlled BLE Mouse");

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize I2C and accelerometer
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized");
    
    if (icm_init() != ESP_OK) {
        ESP_LOGE(TAG, "Sensor init failed");
        return;
    }

    // Initialize button
    button_init();

    // Initialize Bluetooth
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));

    esp_bluedroid_config_t bdcfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bluedroid_init_with_cfg(&bdcfg));
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    ESP_ERROR_CHECK(esp_hidd_profile_init());
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));
    ESP_ERROR_CHECK(esp_hidd_register_callbacks(hidd_event_callback));

    // Security settings
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_BOND;
    esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;
    uint8_t key_size = 16;
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    
    ESP_ERROR_CHECK(esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(auth_req)));
    ESP_ERROR_CHECK(esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(iocap)));
    ESP_ERROR_CHECK(esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(key_size)));
    ESP_ERROR_CHECK(esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(init_key)));
    ESP_ERROR_CHECK(esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(rsp_key)));

    ESP_LOGI(TAG, "Starting mouse control task...");
    xTaskCreate(mouse_control_task, "mouse_ctrl", 4096, NULL, 5, NULL);
}
