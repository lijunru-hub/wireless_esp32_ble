#include <stdio.h>
#include "unity.h"
#include "driver/i2c.h"
#include "i2c_bus.h"
#include "mpu6050.h"
#include "esp_system.h"
#include "gatts_table_creat_demo.h"
#include "esp_log.h"

#define I2C_MASTER_SCL_IO           14          /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO           13          /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_1   /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ          100000      /*!< I2C master clock frequency */
#define HAL_SWAP32(d) __builtin_bswap32((d))

static int data_raw[3] = {0};

static const char *TAG = "mpu6050_read";
static TaskHandle_t THhandle;
static uint16_t spp_conn_id = 0xffff;
static esp_gatt_if_t spp_gatts_if = 0xff;

static i2c_bus_handle_t i2c_bus = NULL;
static mpu6050_handle_t mpu6050 = NULL;

/**
 * @brief i2c master initialization
 */
static void mpu6050_init()
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_bus = i2c_bus_create(I2C_MASTER_NUM, &conf);
    mpu6050 = mpu6050_create(i2c_bus, MPU6050_I2C_ADDRESS);
}

static void mpu6050_test_deinit()
{
    mpu6050_delete(&mpu6050);
    i2c_bus_delete(&i2c_bus);
}

void mpu6050_read_task(void* arg)
{
    uint8_t mpu6050_deviceid;
    mpu6050_acce_value_t acce;
    mpu6050_gyro_value_t gyro;
    complimentary_angle_t complimentary_angle;
    int aola[3] = {0};
    float yaw = 0;
    mpu6050_get_deviceid(mpu6050, &mpu6050_deviceid);
    printf("mpu6050 device ID is: 0x%02x\n", mpu6050_deviceid);
    mpu6050_wake_up(mpu6050);
    mpu6050_set_acce_fs(mpu6050, ACCE_FS_4G);
    mpu6050_set_gyro_fs(mpu6050, GYRO_FS_500DPS);
    while (1)
    {
        ble_state_t ble_state = get_ble_status();
        // ESP_LOGI(TAG,"BLE STATUS:%d",ble_state);
        switch (ble_state)
        {
        case BLE_CONNECT:
            spp_conn_id = get_ble_conn_id();
            spp_gatts_if = get_ble_gatts_if();
            break;
        case BLE_START:
            mpu6050_get_acce(mpu6050, &acce);
            mpu6050_get_gyro(mpu6050, &gyro);
            mpu6050_complimentory_filter(mpu6050, &acce, &gyro, &complimentary_angle);
            yaw += gyro.gyro_z * 0.01;
            ESP_LOGD(TAG, "roll: %.2f, pitch: %.2f, yaw: %.2f\n", complimentary_angle.roll, complimentary_angle.pitch, yaw);
            aola[0] = (int)(complimentary_angle.roll * 100);
            aola[1] = (int)(complimentary_angle.pitch * 100);
            aola[2] = (int)(yaw * 100);
            data_raw[0] = HAL_SWAP32(aola[0]);
            data_raw[1] = HAL_SWAP32(aola[1]);
            data_raw[2] = HAL_SWAP32(aola[2]);
            esp_ble_gatts_set_attr_value(42,sizeof(data_raw), data_raw);
            esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, 42, sizeof(data_raw), data_raw, false);
            break;
        case BLE_STOP:
            break;
        default:
            break;
        }
        vTaskDelay(10);
    }
    vTaskDelete(NULL);
    mpu6050_test_deinit();
}

void app_main(void) {
    gatts_table_init();
    mpu6050_init();
    xTaskCreate(mpu6050_read_task, "mpu6050_read_task", 4*1024, NULL, 5, &THhandle);
}