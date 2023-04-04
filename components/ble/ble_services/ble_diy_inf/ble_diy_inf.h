/**
 * Copyright (c) 2020, Beelinker
 */
 
#ifndef BLE_DIY_INF_H__
#define BLE_DIY_INF_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_gatt.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"
#include "ble_link_ctx_manager.h"
#include "sdk_config.h"

#ifdef __cplusplus
extern "C" {
#endif

// Profile Parameters
#define SIMPLEPROFILE_CHAR1                   0  // RW uint8 - Profile Characteristic 1 value
#define SIMPLEPROFILE_CHAR2                   1  // RW uint8 - Profile Characteristic 2 value
#define SIMPLEPROFILE_CHAR3                   2  // RW uint8 - Profile Characteristic 3 value
#define SIMPLEPROFILE_CHAR4                   3  // RW uint8 - Profile Characteristic 4 value
#define SIMPLEPROFILE_CHAR5                   4  // RW uint8 - Profile Characteristic 5 value
#define SIMPLEPROFILE_CHAR6                   5  // RW uint8 - Profile Characteristic 6 value
#define SIMPLEPROFILE_CHAR7                   6  // RW uint8 - Profile Characteristic 7 value  
#define SIMPLEPROFILE_CHAR60                  7  // RW uint8 - Profile Characteristic 60 value 
  
// Simple Profile Service UUID
#define SIMPLEPROFILE_SERV_UUID             0xFFF0
// Key Pressed UUID
#define SIMPLEPROFILE_CHAR1_UUID            0xFFF1
#define SIMPLEPROFILE_CHAR2_UUID            0xFFF2
#define SIMPLEPROFILE_CHAR3_UUID            0xFFF3
#define SIMPLEPROFILE_CHAR4_UUID            0xFFF4
#define SIMPLEPROFILE_CHAR5_UUID            0xFFF5
#define SIMPLEPROFILE_CHAR6_UUID            0xFFF6
#define SIMPLEPROFILE_CHAR7_UUID            0xFFF7
#define SIMPLEPROFILE_CHAR60_UUID           0xFF60

/**@brief   Macro for defining a ble_diy_inf instance.
 *
 * @param     _name            Name of the instance.
 * @param[in] _max_clients Maximum number of clients connected at a time.
 * @hideinitializer
 */
#define BLE_DEV_DIY_DEF(_name, _max_clients)                      \
    BLE_LINK_CTX_MANAGER_DEF(CONCAT_2(_name, _link_ctx_storage),  \
                             (_max_clients),                  \
                             sizeof(ble_diy_inf_client_context_t));   \
    static diy_inf_service_t _name =                                      \
    {                                                             \
        .p_link_ctx_storage = &CONCAT_2(_name, _link_ctx_storage) \
    };                                                            \
    NRF_SDH_BLE_OBSERVER(_name ## _obs,                           \
                         BLE_CONN_PARAMS_BLE_OBSERVER_PRIO,               \
                         ble_diy_inf_on_ble_evt,                      \
                         &_name)

/*********************************************************************/  
typedef struct
{
    bool is_notification_enabled; /**< Variable to indicate if the peer has enabled notification of the RX characteristic.*/
} ble_diy_inf_client_context_t;

typedef struct{ 
    uint16_t                    service_handle; 
    ble_gatts_char_handles_t    r_w_handle;
	ble_gatts_char_handles_t    r_handle;
    blcm_link_ctx_storage_t     * const p_link_ctx_storage;
}diy_inf_service_t;

uint32_t ble_diy_inf_init(void);

void ble_diy_inf_on_ble_evt(ble_evt_t const *p_ble_evt, void *p_context);

uint8_t ble_diy_inf_set_parameter(uint8_t param, uint8_t len, void *value);

bool get_wattr_flg(void);

void set_wattr_flg(bool flg);

#ifdef __cplusplus
}
#endif

#endif // BLE_NUS_C_H__

/** @} */
