/**
 * Copyright (c) 2020, Beelinker
 */
#include "sdk_common.h"
#include <stdlib.h>
#include "ble.h"
#include "ble_dev_inf.h"
#include "ble_gattc.h"
#include "ble_srv_common.h"
#include "app_error.h"

//static ble_uuid_t val_uuid[6];

//static ble_gatts_attr_t attr_char_value[6];

static uint8_t      mac_address[DEVINFO_BTADDRESS_LEN]={0};
static uint8_t      fw_release_time[10] = "2020-09-30";
static uint8_t      serial_number[13]   = "a02c2f151b15";
static uint8_t      firmware_rev[15]    = "FMVERSION_0001";
static uint8_t      hardware_rev[15]    = "HWVERSION_0001";
static uint8_t      manufacture_date[10] = "2020-09-30";

uint32_t ble_dev_inf_init(void)
{
    ret_code_t              err_code;
    dev_inf_service_t       dev_inf_service;  
    ble_uuid_t              service_uuid;    
    ble_add_char_params_t   add_char_params;
    
    service_uuid.type = BLE_UUID_TYPE_BLE;
    service_uuid.uuid = BLE_UUID_DEVICE_INFORMATION_SERVICE;
 
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, 
                                        &service_uuid, 
                                        &dev_inf_service.service_handle);
    VERIFY_SUCCESS(err_code);
    
    //Device MAC Address Declaration -- 0x2A23
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid            = BLE_UUID_BTADDRESS;
    add_char_params.uuid_type       = BLE_UUID_TYPE_BLE;
    add_char_params.max_len         = DEVINFO_BTADDRESS_LEN;
    add_char_params.init_len        = DEVINFO_BTADDRESS_LEN;
    add_char_params.is_var_len      = true;
    add_char_params.p_init_value    = mac_address;
    add_char_params.char_props.read = 1;
    add_char_params.read_access     = SEC_OPEN;  
    err_code = characteristic_add(dev_inf_service.service_handle, &add_char_params, &dev_inf_service.handle);
    
    //Firmware Revision Release Time Declaration -- 0x2A24
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid            = BLE_UUID_FWRELEASETIME;
    add_char_params.uuid_type       = BLE_UUID_TYPE_BLE;
    add_char_params.max_len         = sizeof(fw_release_time);
    add_char_params.init_len        = sizeof(fw_release_time);
    add_char_params.is_var_len      = true;
    add_char_params.p_init_value    = fw_release_time;
    add_char_params.char_props.read = 1;
    add_char_params.read_access     = SEC_OPEN; 
    err_code = characteristic_add(dev_inf_service.service_handle, &add_char_params, &dev_inf_service.handle);    
    
    //Serial Number String Declaration -- 0x2A25    
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid            = BLE_UUID_SERIAL_NUMBER_STRING_CHAR;
    add_char_params.uuid_type       = BLE_UUID_TYPE_BLE;
    add_char_params.max_len         = sizeof(serial_number);
    add_char_params.init_len        = sizeof(serial_number);
    add_char_params.is_var_len      = true;
    add_char_params.p_init_value    = serial_number;
    add_char_params.char_props.read = 1;
    add_char_params.read_access     = SEC_OPEN; 
    err_code = characteristic_add(dev_inf_service.service_handle, &add_char_params, &dev_inf_service.handle);    
    
    //Firmware Revision String Declaration -- 0x2A26
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid            = BLE_UUID_FIRMWARE_REVISION_STRING_CHAR;
    add_char_params.uuid_type       = BLE_UUID_TYPE_BLE;
    add_char_params.max_len         = sizeof(firmware_rev);
    add_char_params.init_len        = sizeof(firmware_rev);
    add_char_params.is_var_len      = true;
    add_char_params.p_init_value    = firmware_rev;
    add_char_params.char_props.read = 1;
    add_char_params.read_access     = SEC_OPEN;   
    err_code = characteristic_add(dev_inf_service.service_handle, &add_char_params, &dev_inf_service.handle);
  
    //Hardware Revision String Declaration -- 0x2A27    
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid            = BLE_UUID_HARDWARE_REVISION_STRING_CHAR;
    add_char_params.uuid_type       = BLE_UUID_TYPE_BLE;
    add_char_params.max_len         = sizeof(hardware_rev);
    add_char_params.init_len        = sizeof(hardware_rev);
    add_char_params.is_var_len      = true;
    add_char_params.p_init_value    = hardware_rev;
    add_char_params.char_props.read = 1;
    add_char_params.read_access     = SEC_OPEN;   
    err_code = characteristic_add(dev_inf_service.service_handle, &add_char_params, &dev_inf_service.handle);    
    
    //Manufacture Date String Declaration -- 0x2A2B    
    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid            = BLE_UUID_MANUFACTUREDATE;
    add_char_params.uuid_type       = BLE_UUID_TYPE_BLE;
    add_char_params.max_len         = sizeof(manufacture_date);
    add_char_params.init_len        = sizeof(manufacture_date);
    add_char_params.is_var_len      = true;
    add_char_params.p_init_value    = manufacture_date;
    add_char_params.char_props.read = 1;
    add_char_params.read_access     = SEC_OPEN;   
    err_code = characteristic_add(dev_inf_service.service_handle, &add_char_params, &dev_inf_service.handle);     
    
    return err_code;
}

uint8_t ble_dev_inf_set_parameter(uint8_t param, uint8_t len, void *value)
{
    uint8_t ret = 1;

    switch (param)
    {
        case DEVINFO_BTADDRESS:
        if(len == DEVINFO_BTADDRESS_LEN)
        {
            memcpy(mac_address, value, len);
        }
        else
        {
            ret = 0;
        }
        break;

        case DEVINFO_FWRELEASETIME:
        if(len <= sizeof(fw_release_time))
        {
            memset(fw_release_time, 0, sizeof(fw_release_time));
            memcpy(fw_release_time, value, len);
        }
        else
        {
            ret = 0;
        }
        break;
        
        case DEVINFO_SERIAL_NUMBER:
        if(len <= sizeof(serial_number))
        {
            memset(serial_number, 0, sizeof(serial_number));
            memcpy(serial_number, value, len);
        }
        else
        {
            ret = 0;
        }
        break;

        case DEVINFO_FIRMWARE_REV:
        if(len <= DEVINFO_STR_ATTR_LEN)
        {
            memset(firmware_rev, 0, DEVINFO_STR_ATTR_LEN);
            memcpy(firmware_rev, value, len);
        }
        else
        {
            ret = 0;
        }
        break;

        case DEVINFO_HARDWARE_REV:
        if(len <= DEVINFO_STR_ATTR_LEN)
        {
            memset(hardware_rev, 0, DEVINFO_STR_ATTR_LEN);
            memcpy(hardware_rev, value, len);
        }
        else
        {
            ret = 0;
        }
        break;

        case DEVINFO_MANUFACTUREDATE:
        if(len <= sizeof(manufacture_date))
        {
            memset(manufacture_date, 0, sizeof(manufacture_date));
            memcpy(manufacture_date, value, len);
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
