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
 * GATT callback function and handlers
 *
 */

#include "app.h"
#include "ota.h"
#include "cycfg_gap.h"
#include "cycfg_gatt_db.h"
#include "wiced_bt_ota_firmware_upgrade.h"

#if CHIP!=20829 && defined WICED_BT_TRACE_ENABLE
# undef WICED_BT_TRACE
# define WICED_BT_TRACE(format,...)   wiced_printf(NULL, 0, (format"\n"), ##__VA_ARGS__)
#endif

#define APP_GATT_TRACE0        WICED_BT_TRACE
#if GATT_TRACE
# define APP_GATT_TRACE        WICED_BT_TRACE
# if GATT_TRACE>1
#  define APP_GATT_TRACE2      WICED_BT_TRACE
#  if GATT_TRACE>2
#   define APP_GATT_TRACE3     WICED_BT_TRACE
#  else
#   define APP_GATT_TRACE3(...)
#  endif
# else
#  define APP_GATT_TRACE2(...)
#  define APP_GATT_TRACE3(...)
# endif
#else
# define APP_GATT_TRACE(...)
# define APP_GATT_TRACE2(...)
# define APP_GATT_TRACE3(...)
#endif

#if GATT_TRACE > 1
#define STR(x) #x
static const char* eventStr[] =
{
    STR(GATT_CONNECTION_STATUS_EVT),        /**< GATT connection status change. Event data: #wiced_bt_gatt_connection_status_t */
    STR(GATT_OPERATION_CPLT_EVT),           /**< GATT client events. Event data: #wiced_bt_gatt_event_data_t */
    STR(GATT_DISCOVERY_RESULT_EVT),         /**< GATT attribute discovery result. Event data: #wiced_bt_gatt_discovery_result_t */
    STR(GATT_DISCOVERY_CPLT_EVT),           /**< GATT attribute discovery complete. Event data: #wiced_bt_gatt_event_data_t */
    STR(GATT_ATTRIBUTE_REQUEST_EVT),        /**< GATT attribute request (from remote client). Event data: #wiced_bt_gatt_attribute_request_t */
    STR(GATT_CONGESTION_EVT),               /**< GATT congestion (running low in tx buffers). Event data: #wiced_bt_gatt_congestion_event_t TODO: add more details regarding congestion */
    STR(GATT_GET_RESPONSE_BUFFER_EVT),      /**< GATT buffer request, typically sized to max of bearer mtu - 1 */
    STR(GATT_APP_BUFFER_TRANSMITTED_EVT),   /**< GATT buffer transmitted event,  check \ref wiced_bt_gatt_buffer_transmitted_t*/
};

/****************************************************/
const char* getGattEventStr(wiced_bt_management_evt_t event)
{
    if (event >= sizeof(eventStr) / sizeof(uint8_t*))
    {
        return "** UNKNOWN **";
    }

    return eventStr[event];
}
#endif // GATT_TRACE

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
    if(link_conn_id() != conn_id)
    {
        WICED_BT_TRACE("GATT write: Invalid conn_id:%04x!!", conn_id );
        return WICED_BT_GATT_ERROR;
    }
    else
    {
        const gatt_db_lookup_table_t * p_attribute = hidd_get_attribute(p_wr_data->handle);
        wiced_bt_gatt_status_t result = WICED_BT_GATT_SUCCESS;

        APP_GATT_TRACE("GATT write: hdl:0x%04x off:%d len:%d data:%A", p_wr_data->handle, p_wr_data->offset, p_wr_data->val_len, p_wr_data->p_val, p_wr_data->val_len);

        if(p_attribute)
        {
            if(p_wr_data->offset > p_attribute->max_len)
            {
                WICED_BT_TRACE("GATT write: Invalid offset, max_len=%d, ofst:%d", p_attribute->max_len, p_wr_data->offset);
                result = WICED_BT_GATT_INVALID_OFFSET;
            }
            else if((p_wr_data->val_len + p_wr_data->offset) > p_attribute->max_len)
            {
                WICED_BT_TRACE("GATT write: Invalid len");
                result = WICED_BT_GATT_INVALID_ATTR_LEN;
            }
            else
            {
                APP_GATT_TRACE2("write_attrib - success");
                // write the data
                memcpy(p_attribute->p_data + p_wr_data->offset, p_wr_data->p_val, p_wr_data->val_len);
                app_check_cccd_flags(p_wr_data->handle);
            }

            // The device must be bonded to be consider success
            if ((result == WICED_BT_GATT_SUCCESS) && cfg_sec_mask() && !link_is_bonded())
            {
                APP_GATT_TRACE("Insufficient authentication");
                result = WICED_BT_GATT_INSUF_AUTHENTICATION;
            }
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
    APP_GATT_TRACE("GATT: link state change %d", p_status->connected);

#ifdef OTA_FIRMWARE_UPGRADE
    // Pass connection up event to the OTA FW upgrade library
    wiced_ota_fw_upgrade_connection_status_event(p_status);
#endif

    if(p_status->connected)
    {
        bt_stop_reconnect_timer();
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

         default:
            APP_GATT_TRACE2("op %d complete", p_data->op);
            break;
    }

    return WICED_BT_GATT_SUCCESS;
}

/*
 *
 */
static wiced_bt_gatt_status_t gatt_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data)
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_REQ_NOT_SUPPORTED;

    APP_GATT_TRACE2("--- %s (%d)", getGattEventStr(event), event);

    switch(event)
    {
        case GATT_CONNECTION_STATUS_EVT:
            result = gatt_conn_state_change(&p_data->connection_status);
            break;

        case GATT_OPERATION_CPLT_EVT:
            result = gatt_operation_complete(&p_data->operation_complete);
            break;

        case GATT_DISCOVERY_CPLT_EVT:
            WICED_BT_TRACE("congested: %d", gatt.congested);
            break;

        case GATT_ATTRIBUTE_REQUEST_EVT:
            result = gatt_req_cb(&p_data->attribute_request);
            break;

        case GATT_CONGESTION_EVT:
            gatt.congested = p_data->congestion.congested;
            WICED_BT_TRACE("congested: %d", gatt.congested);
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
    APP_GATT_TRACE("REQ_TYPE_CONF conn %d hdl %d", conn_id, handle );

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
    /* GATT DB Initialization */
    if (gatt_db_init( gatt_database, gatt_database_len ) == WICED_BT_GATT_SUCCESS)
    {
        /* Register with stack to receive GATT callback */
        if (wiced_bt_gatt_register( gatt_callback ) == WICED_BT_GATT_SUCCESS)
        {
            wiced_bt_ble_set_raw_advertisement_data(CY_BT_ADV_PACKET_DATA_SIZE, cy_bt_adv_packet_data);
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
