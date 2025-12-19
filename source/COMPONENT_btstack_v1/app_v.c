/*
 * (c) 2016-2025, Infineon Technologies AG, or an affiliate of Infineon
 * Technologies AG. All rights reserved.
 * This software, associated documentation and materials ("Software") is
 * owned by Infineon Technologies AG or one of its affiliates ("Infineon")
 * and is protected by and subject to worldwide patent protection, worldwide
 * copyright laws, and international treaty provisions. Therefore, you may use
 * this Software only as provided in the license agreement accompanying the
 * software package from which you obtained this Software. If no license
 * agreement applies, then any use, reproduction, modification, translation, or
 * compilation of this Software is prohibited without the express written
 * permission of Infineon.
 *
 * Disclaimer: UNLESS OTHERWISE EXPRESSLY AGREED WITH INFINEON, THIS SOFTWARE
 * IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING, BUT NOT LIMITED TO, ALL WARRANTIES OF NON-INFRINGEMENT OF
 * THIRD-PARTY RIGHTS AND IMPLIED WARRANTIES SUCH AS WARRANTIES OF FITNESS FOR A
 * SPECIFIC USE/PURPOSE OR MERCHANTABILITY.
 * Infineon reserves the right to make changes to the Software without notice.
 * You are responsible for properly designing, programming, and testing the
 * functionality and safety of your intended application of the Software, as
 * well as complying with any legal requirements related to its use. Infineon
 * does not guarantee that the Software will be free from intrusion, data theft
 * or loss, or other breaches ("Security Breaches"), and Infineon shall have
 * no liability arising out of any Security Breaches. Unless otherwise
 * explicitly approved by Infineon, the Software may not be used in any
 * application where a failure of the Product or any consequences of the use
 * thereof can reasonably be expected to result in personal injury.
 */

/** @file
 *
 * App stack v1 functions and handlers
 *
 */
#include "app.h"
#include "wiced_hal_mia.h"
#include "wiced_bt_ota_firmware_upgrade.h"

#if CHIP!=20829 && defined WICED_BT_TRACE_ENABLE
# undef WICED_BT_TRACE
# define WICED_BT_TRACE(format,...)   wiced_printf(NULL, 0, (format"\n"), ##__VA_ARGS__)
#endif

void hidd_enable_interrupt(wiced_bool_t en)
{
    wiced_hal_mia_enable_mia_interrupt(en);
    wiced_hal_mia_enable_lhl_interrupt(en);
}


wiced_bt_gatt_status_t app_gatt_read_req_handler( uint16_t conn_id, wiced_bt_gatt_read_t * p_read_data )
{
#ifdef OTA_FIRMWARE_UPGRADE
    // if read request is for the OTA FW upgrade service, pass it to the library to process
    if (wiced_ota_fw_upgrade_is_gatt_handle(p_read_data->handle))
    {
        WICED_BT_TRACE("OTA Read - handle:%04x", p_read_data->handle );
        return wiced_ota_fw_upgrade_read_handler(conn_id, p_read_data);
    }
    else
#endif
    // Check if the write request is for the Android TV Voice service
    if (is_audio_handle(p_read_data->handle))
    {
        return audio_gatt_read_handler(conn_id, p_read_data);
    }

    return gatt_req_read_default_handler( conn_id, p_read_data );
}
