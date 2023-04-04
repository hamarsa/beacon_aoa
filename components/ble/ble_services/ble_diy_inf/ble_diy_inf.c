/**
 * Copyright (c) 2020, Beelinker
 */
#include "sdk_common.h"
#include <stdlib.h>
#include "ble.h"
#include "ble_diy_inf.h"
#include "ble_gattc.h"
#include "app_error.h"
#include "ibeaconinf.h"

static uint8_t char1[2] ={0};

static uint8_t char2[16] = {0};

static uint8_t char3 = 0;

static uint8_t char4[2] = {0};

static uint8_t char5[2] = {0};

static uint8_t char6[2];

static uint8_t char7 = 0;

static uint8_t char60 = 0;

static bool    key_done = false;
static bool    w_attr   = false;

extern ibeaconinf_t sys_inf;

BLE_DEV_DIY_DEF(m_diy, NRF_SDH_BLE_TOTAL_LINK_COUNT);

extern uint16_t get_current_handle(void);
static void ble_gatts_evt_write_handler(ble_gatts_evt_write_t const *p_evt_write);

uint32_t ble_diy_inf_init(void)
{
    ret_code_t              err_code;
    ble_uuid_t              service_uuid;    
    ble_add_char_params_t   add_char_params;
    
    VERIFY_PARAM_NOT_NULL(&m_diy);
    
    service_uuid.type = BLE_UUID_TYPE_BLE;
    service_uuid.uuid = SIMPLEPROFILE_SERV_UUID;
 
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, 
                                        &service_uuid, 
                                        &m_diy.service_handle);
    VERIFY_SUCCESS(err_code);
    
    //key -- 0xFFF1
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid                = SIMPLEPROFILE_CHAR1_UUID;
    add_char_params.uuid_type           = BLE_UUID_TYPE_BLE;
    add_char_params.max_len             = sizeof(char1);
    add_char_params.init_len            = sizeof(char1);
    add_char_params.is_var_len          = true;
    add_char_params.p_init_value        = char1;
    add_char_params.char_props.read     = 1;
    add_char_params.char_props.write    = 1;
    add_char_params.read_access         = SEC_OPEN;
    add_char_params.write_access        = SEC_OPEN;    
    err_code = characteristic_add(m_diy.service_handle, &add_char_params, &m_diy.r_w_handle);
    
    //UUID -- 0xFFF2
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid                = SIMPLEPROFILE_CHAR2_UUID;
    add_char_params.uuid_type           = BLE_UUID_TYPE_BLE;
    add_char_params.max_len             = sizeof(char2);
    add_char_params.init_len            = sizeof(char2);
    add_char_params.is_var_len          = true;
    add_char_params.p_init_value        = char2;
    add_char_params.char_props.read     = 1;
    add_char_params.char_props.write    = 1;
    add_char_params.read_access         = SEC_OPEN; 
    add_char_params.write_access        = SEC_OPEN;
    err_code = characteristic_add(m_diy.service_handle, &add_char_params, &m_diy.r_w_handle);    
    
    //Tx Power -- 0xFFF3    
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid                = SIMPLEPROFILE_CHAR3_UUID;
    add_char_params.uuid_type           = BLE_UUID_TYPE_BLE;
    add_char_params.max_len             = sizeof(char3);
    add_char_params.init_len            = sizeof(char3);
    add_char_params.is_var_len          = true;
    add_char_params.p_init_value        = &char3;
    add_char_params.char_props.read     = 1;
    add_char_params.char_props.write    = 1;
    add_char_params.read_access         = SEC_OPEN; 
    add_char_params.write_access        = SEC_OPEN;
    err_code = characteristic_add(m_diy.service_handle, &add_char_params, &m_diy.r_w_handle);    
    
    //Battry -- 0xFFF4
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid                = SIMPLEPROFILE_CHAR4_UUID;
    add_char_params.uuid_type           = BLE_UUID_TYPE_BLE;
    add_char_params.max_len             = sizeof(char4);
    add_char_params.init_len            = sizeof(char4);
    add_char_params.is_var_len          = true;
    add_char_params.p_init_value        = char4;
    add_char_params.char_props.read     = 1;
    add_char_params.read_access         = SEC_OPEN;   
    err_code = characteristic_add(m_diy.service_handle, &add_char_params, &m_diy.r_handle);
  
    //Major -- 0xFFF5    
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid                = SIMPLEPROFILE_CHAR5_UUID;
    add_char_params.uuid_type           = BLE_UUID_TYPE_BLE;
    add_char_params.max_len             = sizeof(char5);
    add_char_params.init_len            = sizeof(char5);
    add_char_params.is_var_len          = true;
    add_char_params.p_init_value        = char5;
    add_char_params.char_props.read     = 1;
    add_char_params.char_props.write    = 1;
    add_char_params.read_access     = SEC_OPEN;
    add_char_params.write_access        = SEC_OPEN;    
    err_code = characteristic_add(m_diy.service_handle, &add_char_params, &m_diy.r_w_handle);    
    
    //Minor -- 0xFFF6    
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid                = SIMPLEPROFILE_CHAR6_UUID;
    add_char_params.uuid_type           = BLE_UUID_TYPE_BLE;
    add_char_params.max_len             = sizeof(char6);
    add_char_params.init_len            = sizeof(char6);
    add_char_params.is_var_len          = true;
    add_char_params.p_init_value        = char6;
    add_char_params.char_props.read     = 1;
    add_char_params.char_props.write    = 1;
    add_char_params.read_access         = SEC_OPEN; 
    add_char_params.write_access        = SEC_OPEN;    
    err_code = characteristic_add(m_diy.service_handle, &add_char_params, &m_diy.r_w_handle);     

    //Beacon Interval -- 0xFFF7    
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid                = SIMPLEPROFILE_CHAR7_UUID;
    add_char_params.uuid_type           = BLE_UUID_TYPE_BLE;
    add_char_params.max_len             = sizeof(char7);
    add_char_params.init_len            = sizeof(char7);
    add_char_params.is_var_len          = true;
    add_char_params.p_init_value        = &char7;
    add_char_params.char_props.read     = 1;
    add_char_params.char_props.write    = 1;
    add_char_params.read_access         = SEC_OPEN; 
    add_char_params.write_access        = SEC_OPEN;    
    err_code = characteristic_add(m_diy.service_handle, &add_char_params, &m_diy.r_w_handle);  
    
    //RXP -- 0xFF60    
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid                = SIMPLEPROFILE_CHAR60_UUID;
    add_char_params.uuid_type           = BLE_UUID_TYPE_BLE;
    add_char_params.max_len             = sizeof(char60);
    add_char_params.init_len            = sizeof(char60);
    add_char_params.is_var_len          = true;
    add_char_params.p_init_value        = &char60;
    add_char_params.char_props.read     = 1;
    add_char_params.read_access         = SEC_OPEN;   
    err_code = characteristic_add(m_diy.service_handle, &add_char_params, &m_diy.r_handle);  
    
    return err_code;
}

uint8_t ble_diy_inf_set_parameter(uint8_t param, uint8_t len, void *value)
{
    uint8_t ret = 1;

    switch (param)
    {
        case SIMPLEPROFILE_CHAR1:
        if(len == sizeof(char1))
        {
            memcpy(char1, value, len);
        }
        else
        {
            ret = 0;
        }
        break;

        case SIMPLEPROFILE_CHAR2:
        if(len == sizeof(char2))
        {
            memset(char2, 0, sizeof(char2));
            memcpy(char2, value, len);
        }
        else
        {
            ret = 0;
        }
        break;
        
        case SIMPLEPROFILE_CHAR3:
        if(len == sizeof(char3))
        {
            char3 = *((uint8_t *)value);
        }
        else
        {
            ret = 0;
        }
        break;

        case SIMPLEPROFILE_CHAR4:
        if(len == sizeof(char4))
        {
            memset(char4, 0, sizeof(char4));
            memcpy(char4, value, len);
        }
        else
        {
            ret = 0;
        }
        break;

        case SIMPLEPROFILE_CHAR5:
        if(len == sizeof(char5))
        {
            memset(char5, 0, sizeof(char5));
            memcpy(char5, value, len);
        }
        else
        {
            ret = 0;
        }
        break;

        case SIMPLEPROFILE_CHAR6:
        if(len == sizeof(char6))
        {
            memset(char6, 0, sizeof(char6));
            memcpy(char6, value, len);
        }
        else
        {
            ret = 0;
        }
        break;
        
        case SIMPLEPROFILE_CHAR7:
        if(len == sizeof(char7))
        {
            char7 = *((uint8_t *)value);
        }
        else
        {
            ret = 0;
        }
        break;

        case SIMPLEPROFILE_CHAR60:
        if(len == sizeof(char60))
        {
            char60 = *((uint8_t *)value);
        }
        else
        {
            ret = 0;
        }
        break;        

        default:
            ret = 0;
        break;
    }

    return (ret);
}

void ble_diy_inf_on_ble_evt(ble_evt_t const *p_ble_evt, void *p_context)
{
    if ((p_context == NULL) || (p_ble_evt == NULL))
    {
        return;
    }
    
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GATTS_EVT_WRITE:
            ble_gatts_evt_write_handler(p_evt_write);     
        break;
					
      default: 
        break;
    }
}

static void ble_gatts_evt_write_handler(ble_gatts_evt_write_t const *p_evt_write)
{
    ble_gatts_value_t       value_res;
    
    uint8_t                 w_data[20] = {0};  
    uint8_t                 w_len = 0;  
    uint8_t                 i = 0;
    
    w_len = p_evt_write->len;
    
    if(w_len == 0)
        return;
    
    memset(&value_res, 0, sizeof(value_res));
    
    for(i=0; i<w_len; i++)
    {
        w_data[i] = p_evt_write->data[i];
    }
       
    switch(p_evt_write->uuid.uuid)
    {
        case SIMPLEPROFILE_CHAR1_UUID:
            if((w_data[0] == 0xFE) && (w_data[1] == 0x01))
            {
                uint8_t key_resp[2] = {0xFE, 0x02};
                              
                value_res.len       = sizeof(key_resp);
                value_res.p_value   = key_resp;
                
                sd_ble_gatts_value_set(get_current_handle(), p_evt_write->handle, &value_res);
                
                key_done = true;
            }
            else
                key_done = false;
            break;
        
        case SIMPLEPROFILE_CHAR2_UUID:
            if((key_done == true) && (w_len == sizeof(char2)))
            {
                (void)memcpy(char2, w_data, sizeof(char2));
                (void)memcpy(&sys_inf.uuidValue, w_data, sizeof(char2));
                                
                w_attr = true;
            }
            break;
        
        case SIMPLEPROFILE_CHAR3_UUID:
            if((key_done == true) && (w_len == sizeof(char3)))
            {
                char3 = w_data[0];
                sys_inf.txPower = char3;
                
                w_attr = true;
            }            
            break;
                                      
        case SIMPLEPROFILE_CHAR5_UUID:
            if((key_done == true) && (w_len == sizeof(char5)))
            {
                (void)memcpy(char5, w_data, sizeof(char5));
                (void)memcpy(&sys_inf.majorValue, w_data, sizeof(char5));
                
                w_attr = true;
            }            
            break;

        case SIMPLEPROFILE_CHAR6_UUID:
            if((key_done == true) && (w_len == sizeof(char6)))
            {
                (void)memcpy(char6, w_data, sizeof(char6));
                (void)memcpy(&sys_inf.minorValue, w_data, sizeof(char6));
                
                w_attr = true;
            }            
            break;

        case SIMPLEPROFILE_CHAR7_UUID:
            if((key_done == true) && (w_len == sizeof(char7)))
            {
                char7 = w_data[0];
                sys_inf.txInterval = char7;
                
                w_attr = true;
            }            
            break;
                      
        default:
            break;        
    }   
}

bool get_wattr_flg(void)
{
    return w_attr;
}

void set_wattr_flg(bool flg)
{
    w_attr = flg;
}
