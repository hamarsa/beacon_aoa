/**
 * Copyright (c) 2014 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */


#include <stdint.h>
#include <string.h>
#include <nrfx_nvmc.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "app_timer.h"
#include "ble_dev_inf.h"
#include "ble_diy_inf.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp_btn_ble.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_power.h"
#include "nrf_delay.h"
#include "nvram.h"
#include "ibeaconinf.h"
#include "crc.h"

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#if NRFX_CHECK(NRFX_WDT_ENABLED)
#include <nrfx_wdt.h>
#endif

#include "vbat.h"

#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */

#define DEVICE_NAME                     "BeeLinker"                                 /**< Name of device. Will be included in the advertising data. */
#define SERVICE_UUID_TYPE               BLE_UUID_TYPE_BLE                           /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define APP_ADV_INTERVAL                480                                         /**< The advertising interval (in units of 0.625 ms. This value corresponds to 300 ms). */

#define APP_ADV_DURATION                0                                           /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define APP_ADV_TXPOWER                 0

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

#define APP_BEACON_INFO_LENGTH          0x1B                                        /**< Total length of information advertised by the Beacon. */
 
#define BLE_UUID_SERVICE 0xFFF0 /**< The UUID of the Nordic UART Service. */
             
#define DATA_BUF_LEN                          200
#define DEFAULT_UART_AT_TEST_LEN              4
#define DEFAULT_UART_AT_RSP_LEN               6
#define DEFAULT_UART_AT_CMD_LEN               49

#define CONNECTED_TIMEOUT    APP_TIMER_TICKS(60000) //1min
#define SYS_RESTART_TIMEOUT  APP_TIMER_TICKS(600000) //10min

/*!
 * Defines the radio events status
 */
typedef union
{
    uint8_t value;
    
    struct SR_EVENTS
    {
        uint8_t     adv_updata_event       : 1;
    }events;
}app_process_events_t;

app_process_events_t app_process_events = { .value = 0 };

NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */
_APP_TIMER_DEF(adv_updata_id);
_APP_TIMER_DEF(sys_restart_timeout_id);

static uint16_t   vbat;
static uint16_t   m_conn_handle          = BLE_CONN_HANDLE_INVALID;                 /**< Handle of the current connection. */

static uint8_t m_beacon_info[31] =                    /**< Information advertised by the Beacon. */
{  
    0
};

ibeaconinf_t sys_inf;

static uint8_t data_array[DATA_BUF_LEN];

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for initializing the timer module.
 */
static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    
    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);
                                                                                   
    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t           err_code;
    ble_gap_addr_t     addr;
    
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);
    
    sd_ble_gap_addr_get(&addr);
    if(addr.addr_type != BLE_GAP_ADDR_TYPE_PUBLIC)
    {
        addr.addr_type = BLE_GAP_ADDR_TYPE_PUBLIC;
        sd_ble_gap_addr_set(&addr);
    }
    
    ble_dev_inf_set_parameter(DEVINFO_BTADDRESS, BLE_GAP_ADDR_LEN, addr.addr);
    ble_dev_inf_set_parameter(DEVINFO_MANUFACTUREDATE, sizeof(sys_inf.mDate), sys_inf.mDate);    
    ble_dev_inf_init();
    
    ble_diy_inf_set_parameter(SIMPLEPROFILE_CHAR2, DEFAULT_UUID_LEN, sys_inf.uuidValue);
    ble_diy_inf_set_parameter(SIMPLEPROFILE_CHAR3, sizeof(uint8_t),  &sys_inf.txPower);
    ble_diy_inf_set_parameter(SIMPLEPROFILE_CHAR4, sizeof(uint16_t), &vbat);
    ble_diy_inf_set_parameter(SIMPLEPROFILE_CHAR5, sizeof(uint16_t), &sys_inf.majorValue);
    ble_diy_inf_set_parameter(SIMPLEPROFILE_CHAR6, sizeof(uint16_t), &sys_inf.minorValue);
    ble_diy_inf_set_parameter(SIMPLEPROFILE_CHAR7, sizeof(uint8_t),  &sys_inf.txInterval);
    ble_diy_inf_set_parameter(SIMPLEPROFILE_CHAR60, sizeof(uint8_t), &sys_inf.Rxp);    
    ble_diy_inf_init();
}


/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            break;
        case BLE_ADV_EVT_IDLE:
            break;
        default:
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            set_wattr_flg(false);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            // LED indication will be changed when advertising starts.
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            if(get_wattr_flg())
            {
                memset(data_array, 0, sizeof(data_array));
                memcpy(data_array, &sys_inf.txPower, sizeof(sys_inf));
                data_array[sizeof(sys_inf)] = crc8(0, data_array, sizeof(sys_inf));
                nvram_block_write(data_array, sizeof(sys_inf)+sizeof(uint8_t));
                NVIC_SystemReset();
            }
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        //m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
    }
}


/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;

        default:
            break;
    }
}

volatile uint8_t w_data_ptr = 0;
volatile uint8_t r_data_ptr = 0;
volatile uint8_t overflow   = 0;
/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. 
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t * p_event)
{
    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY: 
            if(overflow < DATA_BUF_LEN)
            {
                UNUSED_VARIABLE(app_uart_get(&data_array[w_data_ptr++]));
                w_data_ptr = w_data_ptr%DATA_BUF_LEN;
                overflow ++;    
            }          
            break;

        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}
/**@snippet [Handling the data received over UART] */


/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(void)
{
    uint32_t                     err_code;
    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
#if defined (UART_PRESENT)
        .baud_rate    = NRF_UART_BAUDRATE_9600
#else
        .baud_rate    = NRF_UARTE_BAUDRATE_115200
#endif
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);
}
/**@snippet [UART Initialization] */

const tx_pow_t  tx_power_table[]=
{
    {-40,0xA0}, {-20,0xAB},  {-16,0xB0},
    {-12,0xB1}, {-8,0xB2},   {-4,0xBD},
    {0,0xB5},   {4,0XD0},
};

const tx_interval_t tx_interval_table[]=
{
    {1,1400},   //875ms
    {2,800},    //500ms
    {3,480},    //300ms
    {4,400},    //250ms
    {5,320},    //200ms
    {10,160},   //100ms
    {20,80},    //50ms
    {30,48},    //30ms
    {50,32}     //20ms
};
/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t               err_code;
    uint32_t               adv_interval;   
    
    const int8_t           tx_power_level = 0;
   		
    ble_advertising_init_t      init;
    
    ble_advdata_manuf_data_t    manuf_specific_data;
    
    memset(&init, 0, sizeof(init));
    memset(&manuf_specific_data, 0, sizeof(manuf_specific_data));
        
    //tx_power_level = tx_power_table[6];
          
    manuf_specific_data.company_identifier  = 0x1234;
    manuf_specific_data.data.p_data         = m_beacon_info;
    manuf_specific_data.data.size           = APP_BEACON_INFO_LENGTH;
    
    init.advdata.p_manuf_specific_data      = &manuf_specific_data;
    init.advdata.name_type          = BLE_ADVDATA_NO_NAME;
    init.advdata.include_appearance = false;
    init.advdata.flags              = 0;
    
    init.srdata.name_type              = BLE_ADVDATA_FULL_NAME;
        
    //adv_interval = 160*100;     
    adv_interval = 160;    

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = adv_interval;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;
    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);
    
    err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, m_advertising.adv_handle, tx_power_level); 
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);   
}

static void app_advertising_data_update( uint16_t adv_type )
{
    ret_code_t  err_code;
    
    ble_advdata_t x_advdata;
    ble_advdata_t x_srdata;
    ble_adv_modes_config_t x_config;
    ble_advdata_manuf_data_t    x_manuf_specific_data;
    
    uint8_t adv_data_len = 0;
    
    memset(&x_advdata, 0 , sizeof( ble_advdata_t ) );
    memset(&x_srdata, 0, sizeof( ble_advdata_t ) );
    memset(&x_config, 0, sizeof( ble_adv_modes_config_t ) );
    memset(&x_manuf_specific_data, 0, sizeof( ble_advdata_manuf_data_t ) );
       
    memset(m_beacon_info, 0, sizeof( m_beacon_info ) );
    
    x_config.ble_adv_fast_interval = 160*100;
    adv_data_len = APP_BEACON_INFO_LENGTH;
    
    /* Update beacon_info */
    m_beacon_info[0] = 1;
                                              
    sd_ble_gap_adv_stop(m_advertising.adv_handle);
    
    {       
        x_manuf_specific_data.company_identifier = 0x1234;
        x_manuf_specific_data.data.p_data = m_beacon_info;
        x_manuf_specific_data.data.size   = adv_data_len;
        
        x_advdata.p_manuf_specific_data = &x_manuf_specific_data;
        x_advdata.include_appearance = false;
        x_advdata.name_type = BLE_ADVDATA_NO_NAME;
        x_advdata.flags = 0;
        
        x_srdata.name_type =  BLE_ADVDATA_NO_NAME;
        
        x_config.ble_adv_fast_enabled  = true;     
        x_config.ble_adv_fast_timeout  = APP_ADV_DURATION;
        
        ble_advertising_modes_config_set( &m_advertising, &x_config );
        
        err_code = ble_advertising_advdata_update( &m_advertising, &x_advdata, &x_srdata );
        APP_ERROR_CHECK(err_code); 

        err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
        APP_ERROR_CHECK(err_code);    
    }
}

/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
}

uint16_t get_current_handle(void)
{ 
    return m_conn_handle;
}

//Consistent with setting dev_information
const uint8_t D_FRT[10] ={'2','0','2','0','-','0','9','-','3','0'};                
const uint8_t D_FR[14]={'F','M','V','E','R','S','I','O','N','_','0','0','0','1'}; 

const  uint8_t D_CKey[16]={0xDE,0x48,0x2B,0x1C,0x22,0x1C,0x6C,0x30,0x3C,0xF0,0x50,0xEB,0x00,0x20,0xB0,0xBD}; 
static uint8_t  data_res[DEFAULT_UART_AT_CMD_LEN];

void uart_data_tx(uint8_t *buf, uint8_t len)
{
    if((buf == NULL) || (len == 0))
        return;
    
    do{
        app_uart_put(*buf++);     
    }while(--len);
}

void uart_at_handle(uint8_t *buf)
{
    uint8_t     mac_res[6];
    
    //uuid
    memcpy(&sys_inf.uuidValue, &data_res[5], DEFAULT_UUID_LEN);
    //major & minor
    memcpy(&sys_inf.majorValue, &data_res[21], sizeof(uint32_t));
    //hwvr
    memcpy(&sys_inf.hwvr, &data_res[35], sizeof(uint32_t));
    //txPower
    memcpy(&sys_inf.txPower, &data_res[39], sizeof(uint8_t));
    if(sys_inf.txPower >= sizeof(tx_power_table)/sizeof(tx_pow_t))
    {
        sys_inf.txPower = APP_ADV_TXPOWER;
        sys_inf.Rxp     = tx_power_table[APP_ADV_TXPOWER].rxp;
    }
    else
    {
        sys_inf.Rxp     = tx_power_table[sys_inf.txPower].rxp;
    }           
    //txInterval
    memcpy(&sys_inf.txInterval, &data_res[40], sizeof(uint8_t));
    //mdate
    memcpy(&sys_inf.mDate, &data_res[25], sizeof(sys_inf.mDate));
    
    /***** Just to get through production. ***********/
    memcpy(mac_res, &data_res[41], sizeof(mac_res));
    memset(data_res, 0, sizeof(data_res));
    memcpy(data_res, mac_res, sizeof(mac_res));
    
    data_res[6] = sys_inf.txPower;
    data_res[7] = sys_inf.txInterval;
    
    memcpy(&data_res[8], sys_inf.majorValue, sizeof(uint32_t));
    memcpy(&data_res[12], sys_inf.uuidValue, DEFAULT_UUID_LEN);
    memcpy(&data_res[28], sys_inf.mDate, sizeof(sys_inf.mDate));
    
    data_res[38] = sys_inf.Rxp;
    
    memcpy(&data_res[39], &sys_inf.hwvr[0], sizeof(sys_inf.hwvr)); 
    
    if(nrfx_nvmc_page_erase(BL_BACKUP_ADD) != NRFX_SUCCESS)
        return;
    
    nrfx_nvmc_bytes_write(BL_BACKUP_ADD, data_res, 43);
    
    uart_data_tx((uint8_t *)"OK+1\r\n", 6);
}
void uart_at_resp(void)
{
    uint8_t const *dst;
    uint8_t crc_8;
    
    memset(data_res, 0, sizeof(data_res));
    
    dst = (const uint8_t *)BL_BACKUP_ADD;
    
    memcpy(data_res, dst, 43);
    
    uart_data_tx(((uint8_t *)"OK+"), 3);
    uart_data_tx(data_res, 43);
    uart_data_tx((uint8_t *)D_FRT,      sizeof(D_FRT));
    uart_data_tx((uint8_t *)&D_FR[10],  sizeof(uint32_t));
    uart_data_tx((uint8_t *)D_CKey,     sizeof(D_CKey));
    
    nrf_delay_ms(100); //wait for uart
    
    sys_inf.atFlag = AT_FLAG_DONE;
    memset(data_res, 0, sizeof(data_res));
    memcpy(data_res, &sys_inf.txPower, sizeof(sys_inf));
    crc_8 = crc8(0, &sys_inf.txPower, sizeof(sys_inf));
    data_res[sizeof(sys_inf)] = crc_8;
                            
    nvram_block_write(data_res, sizeof(sys_inf)+sizeof(uint8_t)); 

    NVIC_SystemReset();           
}

void uart_data_handle(void)
{
    uint8_t     r_heard;
    uint8_t     rx_cnt;
    uint8_t     data_len;
    uint8_t     *ptr;
    uint8_t     res;
    
    r_heard = r_data_ptr;
    
    ptr = data_array;
    
    rx_cnt  = overflow;
    
    if(rx_cnt < DEFAULT_UART_AT_TEST_LEN)
        return;
     
    if(*(ptr + r_heard) != 'A')
         goto  INVALID_PACKET;
    
    r_heard ++;
    r_heard = r_heard%DATA_BUF_LEN;
    
    if(*(ptr + r_heard) != 'T')
        goto  INVALID_PACKET;
    
    r_heard ++;
    r_heard = r_heard%DATA_BUF_LEN;    
    
    if(*(ptr + r_heard) == '\r')
    {
        r_heard ++;
        r_heard = r_heard%DATA_BUF_LEN;
        
        if(*(ptr + r_heard) == '\n')
        {
            uart_data_tx((uint8_t *)"OK\r\n", sizeof(uint32_t));
            
            data_len = DEFAULT_UART_AT_TEST_LEN;
            
            goto COMPLETE_PACKET;
        }
    }
    else if(*(ptr + r_heard) == '+')
    {
        r_heard ++;
        r_heard = r_heard%DATA_BUF_LEN;
        
        if(*(ptr + r_heard) == '1')
        {
            r_heard ++;
            r_heard = r_heard%DATA_BUF_LEN;

            if(*(ptr + r_heard) == '=')
            {
                if(rx_cnt >= DEFAULT_UART_AT_CMD_LEN)
                {
                    rx_cnt = DEFAULT_UART_AT_CMD_LEN;
                    
                    memset(data_res, 0, sizeof(data_res));
                    
                    if(r_data_ptr + rx_cnt <= DATA_BUF_LEN)
                    {       
                        r_heard += (rx_cnt - sizeof("AT+1=") +1);
                        memcpy(data_res, ptr + r_data_ptr, rx_cnt);
                    }
                    else
                    {
                        res = DATA_BUF_LEN - r_data_ptr;
                        r_heard = rx_cnt - res;
                        memcpy(data_res, ptr + r_data_ptr, res);
                        memcpy(data_res + res, ptr, r_heard);                        
                    }
                    
                    uart_at_handle(data_res);
                    
                    data_len = DEFAULT_UART_AT_CMD_LEN;
                    
                    goto COMPLETE_PACKET;  
                }
                else
                    return;
            }
            else
               return;
        }
        else if(*(ptr+r_heard) == '?')
        {       
            uart_at_resp();           
        }   
    }
    else
        goto INVALID_PACKET;
                  
      
INVALID_PACKET:
        r_data_ptr ++;
        r_data_ptr = r_data_ptr%DATA_BUF_LEN;
        overflow --;  
COMPLETE_PACKET:  
        overflow    -= data_len;
        r_data_ptr   = ++r_heard;
}

void adv_updata_timeout_handle(void * p_context)
{
    app_process_events.events.adv_updata_event = 1;
}

void sys_restart_timeout_handle(void * p_context)
{
    NVIC_SystemReset();   
}

static void app_sys_event_handle( void )
{ 
    if( app_process_events.value == 0 )
        return;
    
    if( app_process_events.events.adv_updata_event )
    {
        app_process_events.events.adv_updata_event = 0;
        
        app_advertising_data_update( 1 );
    }
}

#if NRFX_CHECK(NRFX_WDT_ENABLED)
nrfx_wdt_config_t wdt_config = NRFX_WDT_DEAFULT_CONFIG;
nrfx_wdt_channel_id wdt_channel_id;
#endif  

/**@brief Application main function.
 */
int main(void)
{   
    nrf_power_dcdcen_set(true);
    
    timers_init();

    nvram_init();
        
    nvram_block_read(&sys_inf.txPower, sizeof(sys_inf));
    
#if NRFX_CHECK(DEMO)  
    power_management_init();
#else    
    if(sys_inf.atFlag == AT_FLAG_DEFAULT)
    {
        uart_init();   
    }
    else
    {
        power_management_init();
    }
#endif
    
    nrf_delay_ms(2);
    vbat_init();
    nrf_delay_ms(2);
    
    vbat = vbat_read_value();
 
#if NRFX_CHECK(NRFX_WDT_ENABLED)
    nrfx_wdt_init(&wdt_config, NULL);
    nrfx_wdt_channel_alloc(&wdt_channel_id);
    nrfx_wdt_enable();
#endif  
    
    ble_stack_init();
    
    gap_params_init();
    
    gatt_init();
    
    services_init();
    
    advertising_init();
    
    conn_params_init();

    advertising_start();
    
    app_timer_create(&adv_updata_id, APP_TIMER_MODE_REPEATED, adv_updata_timeout_handle);
    
    app_timer_create(&sys_restart_timeout_id, APP_TIMER_MODE_SINGLE_SHOT, sys_restart_timeout_handle);
    
    app_timer_start(sys_restart_timeout_id, SYS_RESTART_TIMEOUT, NULL);
    
    for (;;)
    {  
#if NRFX_CHECK(DEMO)  
        idle_state_handle();
#else        
        if(sys_inf.atFlag == AT_FLAG_DEFAULT)
        {
            uart_data_handle();
        }
        else
        {
           idle_state_handle();
        }
#endif
        
        app_sys_event_handle( );
                   
#if NRFX_CHECK(NRFX_WDT_ENABLED)
        nrfx_wdt_feed();
#endif
    }        
}


/**
 * @}
 */
