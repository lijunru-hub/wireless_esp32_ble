/*
 * SPDX-FileCopyrightText: 2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_gatts_api.h"


/* Attributes State Machine */
enum
{
    IDX_SVC,
    IDX_CHAR_A,
    IDX_CHAR_VAL_A,
    IDX_CHAR_CFG_A,

    IDX_CHAR_B,
    IDX_CHAR_VAL_B,

    IDX_CHAR_C,
    IDX_CHAR_VAL_C,

    HRS_IDX_NB,
};

typedef enum {
    BLE_DISCONNECT = -1,
    BLE_CONNECT,
    BLE_START,
    BLE_STOP,
} ble_state_t;

/* 两个字节作为控制命令
 * 第一个字节: ACTION
 * 第二个字节: EVENT
 */
typedef enum {
    CONTROL = 0x11,
} ble_message_action_t;

typedef enum {
    START_EVENT = 0x01,
    STOP_EVENT = 0x02,
} ble_message_event_t;

void gatts_table_init(void);

ble_state_t get_ble_status();

void set_ble_status(ble_state_t ble_state);

uint16_t get_ble_conn_id();

esp_gatt_if_t get_ble_gatts_if();