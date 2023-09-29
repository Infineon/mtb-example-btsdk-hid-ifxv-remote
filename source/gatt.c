/*
 * Copyright 2016-2023, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */

/** @file
 *
 * GATT callback function and handlers
 *
 */

#include "app.h"
#include "cycfg_gap.h"
#include "cycfg_gatt_db.h"
#include "wiced_bt_ota_firmware_upgrade.h"

#if CHIP!=20829 && defined WICED_BT_TRACE_ENABLE
# undef WICED_BT_TRACE
# define WICED_BT_TRACE(format,...)   wiced_printf(NULL, 0, (format"\n"), ##__VA_ARGS__)
#endif

#if GATT_TRACE==1
# define APP_GATT_TRACE        WICED_BT_TRACE
# define APP_GATT_TRACE2(...)
#elif GATT_TRACE>1
# define APP_GATT_TRACE        WICED_BT_TRACE
# define APP_GATT_TRACE2       WICED_BT_TRACE
#else
# define APP_GATT_TRACE(...)
# define APP_GATT_TRACE2(...)
#endif

gatt_t gatt = {0};

/********************************************************************
 * Function Name: gatt_write_default_handler
 ********************************************************************
 * Summary:
 *  Default handler to process write request or command from peer device.
 *  The event calls application gatt_write_handler first. When it is not
 *  handled in application, this default handler is called.
 ********************************************************************/
wiced_bt_gatt_status_t gatt_write_default_handler( uint16_t conn_id, wiced_bt_gatt_write_req_t * p_wr_data )
{
    APP_GATT_TRACE2("In gatt_write_default_handler");
    if(link_conn_id() != conn_id)
    {
        WICED_BT_TRACE("gatt: write handle to an invalid conn_id:%04x", conn_id );
        return WICED_BT_GATT_ERROR;
    }
    else
    {
        const gatt_db_lookup_table_t * p_attribute = hidd_get_attribute(p_wr_data->handle);
        wiced_bt_gatt_status_t result = WICED_BT_GATT_SUCCESS;

        APP_GATT_TRACE("write_attrib - conn:0x%04x hdl:0x%04x off:%d len:%d", conn_id, p_wr_data->handle, p_wr_data->offset, p_wr_data->val_len );
        APP_GATT_TRACE2("Data: %A", p_wr_data->p_val, p_wr_data->val_len);

        if(p_attribute)
        {
            if(p_wr_data->offset > p_attribute->max_len)
            {
                APP_GATT_TRACE("Invalid offset, max_len=%d, ofst:%d", p_attribute->max_len, p_wr_data->offset);
                result = WICED_BT_GATT_INVALID_OFFSET;
            }
            else if((p_wr_data->val_len + p_wr_data->offset) > p_attribute->max_len)
            {
                APP_GATT_TRACE("Invalid len");
                result = WICED_BT_GATT_INVALID_ATTR_LEN;
            }
            else
            {
                APP_GATT_TRACE2("write_attrib - success");
                // write the data
                memcpy(p_attribute->p_data + p_wr_data->offset, p_wr_data->p_val, p_wr_data->val_len);

                app_check_cccd_flags(p_wr_data->handle);
            }

//#ifndef DISABLE_ENCRYPTION
#if 0
            // The device must be bonded to be consider success
            if ((result == WICED_BT_GATT_SUCCESS) && !link_is_bonded() )
            {
                APP_GATT_TRACE2("WICED_BT_GATT_INSUF_AUTHENTICATION");
                result = WICED_BT_GATT_INSUF_AUTHENTICATION;
            }
#endif
        }
        else
        {
            result = WICED_BT_GATT_INVALID_HANDLE;
        }

        // Whenever there is an activity, restart the idle timer
        // hidd_le_link_restart_idle_timer();

        return result;
    }
}

/********************************************************************
 * Function Name: gatt_conn_state_change
 ********************************************************************
 * Summary:
 *  Handles connection state change. This function is called when the
 *  link is up or down. It calls link module for the link event.
 *******************************************************************/
static wiced_bt_gatt_status_t gatt_conn_state_change( wiced_bt_gatt_connection_status_t * p_status )
{
    if(p_status->connected)
    {
        hidd_clear_cccd_flags();        // new link, clear all cccd flags
        return link_up( p_status );
    }
    else
    {
        return link_down( p_status );
    }
}

static wiced_bt_gatt_status_t gatt_operation_complete(wiced_bt_gatt_operation_complete_t *p_data)
{
    switch ( p_data->op )
    {
         case GATTC_OPTYPE_CONFIG:
            WICED_BT_TRACE( "mtu size: %d", p_data->response_data.mtu );
            gatt.peer_mtu = p_data->response_data.mtu;
            break;
    }

    return WICED_BT_GATT_SUCCESS;
}

/*
 *
 */
static wiced_bt_gatt_status_t gatt_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data)
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_SUCCESS;

    APP_GATT_TRACE2("hidd_gatt_callback event: %d", event);

    switch(event)
    {
        case GATT_CONNECTION_STATUS_EVT:
            APP_GATT_TRACE2("GATT_CONNECTION_STATUS_EVT");
            result = gatt_conn_state_change(&p_data->connection_status);
            break;

        case GATT_OPERATION_CPLT_EVT:
            APP_GATT_TRACE2("GATT_OPERATION_CPLT_EVT");
            result = gatt_operation_complete(&p_data->operation_complete);
            break;

        case GATT_DISCOVERY_CPLT_EVT:
            APP_GATT_TRACE2("GATT_DISCOVERY_CPLT_EVT");
            break;

        case GATT_ATTRIBUTE_REQUEST_EVT:
            APP_GATT_TRACE2("GATT_ATTRIBUTE_REQUEST_EVT");
            result = gatt_req_cb(&p_data->attribute_request);
            break;

        case GATT_CONGESTION_EVT:
            gatt.congested = p_data->congestion.congested;
            WICED_BT_TRACE("congested:%d", gatt.congested);
            if (gatt.congested_cb)
            {
                gatt.congested_cb(gatt.congested);
            }
            break;

        default:
            return gatt_v_callback(event, p_data);
    }

    return result;
}

/*
 * Process indication confirm. If client wanted us to use indication instead of
 * notifications we have to wait for confirmation after every message sent.
 * For example if user pushed button twice very fast
 * we will send first message, then
 * wait for confirmation, then
 * send second message, then
 * wait for confirmation and
 * if configured start idle timer only after that.
 */
wiced_bt_gatt_status_t gatt_req_conf_handler( uint16_t conn_id, uint16_t handle )
{
    APP_GATT_TRACE("gatt_req_conf_handler, conn %d hdl %d", conn_id, handle );

#ifdef OTA_FIRMWARE_UPGRADE
    // if indication confirmation is for the OTA FW upgrade service, pass it to the library to process
    if (wiced_ota_fw_upgrade_is_gatt_handle(handle))
    {
        APP_GATT_TRACE("OTA req_conf_handler %04x", handle );
        return wiced_ota_fw_upgrade_indication_cfm_handler(conn_id, handle);
    }
#endif

    return WICED_BT_GATT_SUCCESS;
}

/********************************************************************
 * Function Name: gatt_initialize
 ********************************************************************
 * Summary:
 *  Initialize gatt database.
 *  The advertisement data CY_BT_ADV_PACKET_DATA_SIZ and cy_bt_adv_packet_data
 *  are generated by BT Configurator in cycfg_gap.h/c.
 *  The gatt database gatt_database, and gatt_database_len are generated by
 *  BT Configurator in cycfg_gatt_db.h/c.
 ********************************************************************/
wiced_bt_gatt_status_t gatt_initialize()
{
    wiced_bt_ble_set_raw_advertisement_data(CY_BT_ADV_PACKET_DATA_SIZE, cy_bt_adv_packet_data);

    /* GATT DB Initialization */
    if (gatt_db_init( gatt_database, gatt_database_len ) == WICED_BT_GATT_SUCCESS)
    {
        /* Register with stack to receive GATT callback */
        if (wiced_bt_gatt_register( gatt_callback ) == WICED_BT_GATT_SUCCESS)
        {
            return WICED_BT_GATT_SUCCESS;
        }
    }
    return WICED_BT_GATT_ERROR;

}

/*
 * gatt_get_att_mtu_size
 */
uint16_t gatt_get_att_mtu_size()
{
    return wiced_blehidd_get_att_mtu_size( host_addr() );
}

void gatt_register_congested_callback( gatt_congested_cb_t cb )
{
    gatt.congested_cb = cb;
}
