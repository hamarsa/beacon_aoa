/**
 * Copyright (c) 2020, Beelinker
 */
 
#ifndef BLE_DEV_INF_H__
#define BLE_DEV_INF_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_gatt.h"
//#include "ble_db_discovery.h"
#include "ble_srv_common.h"
//#include "nrf_ble_gq.h"
#include "nrf_sdh_ble.h"

#include "sdk_config.h"

#ifdef __cplusplus
extern "C" {
#endif

// Device Information Service Parameters
#define DEVINFO_BTADDRESS                 0
#define DEVINFO_FWRELEASETIME             1
#define DEVINFO_SERIAL_NUMBER             2
#define DEVINFO_FIRMWARE_REV              3
#define DEVINFO_HARDWARE_REV              4
#define DEVINFO_MANUFACTUREDATE           5

#define DEVINFO_BTADDRESS_LEN             6

#define BASE_UUID     {{0xFD, 0xA5, 0x06, 0x93, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x00, 0x00, 0x40, 0x6E}} /**< Used vendor specific UUID. */

//String attribute length
#ifndef DEVINFO_STR_ATTR_LEN
  #define DEVINFO_STR_ATTR_LEN            15
#endif

typedef struct{
    uint16_t    service_handle; 
    ble_gatts_char_handles_t handle;
}dev_inf_service_t;

uint32_t ble_dev_inf_init(void);

uint8_t ble_dev_inf_set_parameter(uint8_t param, uint8_t len, void *value);

#ifdef __cplusplus
}
#endif

#endif // BLE_NUS_C_H__

/** @} */
