/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "esp_log.h"
#include "gatts_table_creat_demo.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "soc/adc_periph.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "esp_private/adc_private.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "hal/misc.h"

static const char *TAG = "adc_read";
static TaskHandle_t THhandle;
static uint16_t spp_conn_id = 0xffff;
static esp_gatt_if_t spp_gatts_if = 0xff;

typedef unsigned short int uint16;
typedef unsigned long int uint32;
 


/*---------------------------------------------------------------
        ADC General Macros
---------------------------------------------------------------*/
//ADC1 Channels
#define EXAMPLE_ADC1_CHAN0          ADC_CHANNEL_2    //GPIO3
#define EXAMPLE_ADC1_CHAN1          ADC_CHANNEL_3    //GPIO4
#define EXAMPLE_ADC1_CHAN2          ADC_CHANNEL_4    //GPIO5
#define EXAMPLE_ADC_ATTEN           ADC_ATTEN_DB_11

static adc_oneshot_unit_handle_t adc_handle;

static int adc_raw[3] = {0};
static int voltage[3];
static bool example_adc_calibration_init(adc_unit_t unit, adc_atten_t atten, adc_cali_handle_t *out_handle);
static void example_adc_calibration_deinit(adc_cali_handle_t handle);

void adc_init(void)
{
    //-------------ADC1 Init---------------//

    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
        ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc_handle));

    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = EXAMPLE_ADC_ATTEN,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, EXAMPLE_ADC1_CHAN0, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, EXAMPLE_ADC1_CHAN1, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, EXAMPLE_ADC1_CHAN2, &config));

    //-------------ADC1 Calibration Init---------------//
    adc_cali_handle_t adc1_cali_handle = NULL;
    bool do_calibration1 = example_adc_calibration_init(ADC_UNIT_1, EXAMPLE_ADC_ATTEN, &adc1_cali_handle);
}


/*---------------------------------------------------------------
        ADC Calibration
---------------------------------------------------------------*/
static bool example_adc_calibration_init(adc_unit_t unit, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }
    return calibrated;
}

static void example_adc_calibration_deinit(adc_cali_handle_t handle)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Curve Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));

#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Line Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
#endif
}

void adc_read_task(void* arg)
{
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
            ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, EXAMPLE_ADC1_CHAN0, &adc_raw[0]));
            ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, EXAMPLE_ADC1_CHAN1, &adc_raw[1]));
            ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, EXAMPLE_ADC1_CHAN2, &adc_raw[2]));
            adc_raw[0] = HAL_SWAP32(adc_raw[0]);
            adc_raw[1] = HAL_SWAP32(adc_raw[1]);
            adc_raw[2] = HAL_SWAP32(adc_raw[2]);
            esp_ble_gatts_set_attr_value(42,sizeof(adc_raw), adc_raw);
            esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, 42, sizeof(adc_raw), adc_raw, false);
            break;
        case BLE_STOP:
            break;
        default:
            break;
        }
        vTaskDelay(10);
    }
}

// void app_main(void) {
//     gatts_table_init();
//     adc_init();
//     xTaskCreate(adc_read_task, "adc_read_task", 4*1024, NULL, 5, &THhandle);
// }



