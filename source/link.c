/*
 * $ Copyright YEAR Cypress Semiconductor $
 */

/**
 * file link.c
 *
 * Link management function
 *
 * Abstract: This file contains function link managing
 */
#include "wiced_bt_gatt.h"
#include "wiced_bt_l2c.h"
#include "app.h"

#if CHIP!=20829 && defined WICED_BT_TRACE_ENABLE
# undef WICED_BT_TRACE
# define WICED_BT_TRACE(format,...)   wiced_printf(NULL, 0, (format"\n"), ##__VA_ARGS__)
#endif

#define MAX_CONN            2

/******************************************************************************
 *  typedef
 ******************************************************************************/
typedef struct
{
    wiced_bt_gatt_connection_status_t   connection_status;
    uint16_t                            acl_conn_handle;
    wiced_bt_device_address_t           bd_addr;                    // Remote device address (from connection_status)
    wiced_bt_ble_conn_params_t          conn_params;

    uint8_t                             transport:2;                // 0:no link (BT_TRANSPORT_NONE), 1:Classic (BT_TRANSPORT_BR_EDR) , 2:LE (BT_TRANSPORT_LE)
    uint8_t                             encrypted:1;                // encrypted:1, not encrypted:0
    uint8_t                             link_parameter_updated:1;   // 1:connection parameter update, 0: no connection parameter update since connection
    uint8_t                             indicate_pending:1;         // 1:waiting for indication confirm, 0:no indication pending
    uint8_t                             bonded:1;                   // 1:bonded, 0:not bonded
} link_state_t;

/******************************************************************************
 *  local variables
 ******************************************************************************/
static struct
{
    link_state_t   conn[MAX_CONN];
    link_state_t * active;

} link = {0};

/******************************************************************************
 *  Private functions
 ******************************************************************************/

/********************************************************************
 * Function Name: link_get_state
 ********************************************************************
 * Summary:
 *  Returns link state pointer for the given connection id.
 ********************************************************************/
static link_state_t * link_get_state(uint16_t conn_id)
{
    // find the connection id
    for (uint8_t idx=0; idx < MAX_CONN; idx++)
    {
        if (link.conn[idx].connection_status.conn_id == conn_id)
        {
            link.active = &link.conn[idx];
            return link.active;
        }
    }
    return NULL;
}

/******************************************************************************
 *  Public functions
 ******************************************************************************/

/********************************************************************
 * Function Name: link_transport
 ********************************************************************
 * Summary:
 *    Returns the link transport.
 *    (BT_TRANSPORT_NONE, BT_TRANSPORT_BR_EDR or BT_TRANSPORT_LE)
 ********************************************************************/
uint8_t link_transport()
{
    return link.active ? link.active->transport : BT_TRANSPORT_NONE;
}

/*******************************************************************************
 * Function Name: link_conn_id
 ********************************************************************************
 * Summary:
 *    Return last active link conn_id. If no link, it returns 0.
 *******************************************************************************/
uint16_t link_conn_id()
{
    return link.active ? link.active->connection_status.conn_id : 0;
}

/*******************************************************************************
 * Function Name: link_acl_conn_handle
 ********************************************************************************
 * Summary:
 *    Return last active ACL connection handle. If no link, it returns 0.
 *******************************************************************************/
uint16_t link_acl_conn_handle()
{
    return link.active ? link.active->acl_conn_handle : 0;
}

/*******************************************************************************
 * Function Name: link_first_acl_conn_handle
 ********************************************************************************
 * Summary:
 *    Return first active ACL connection handle. If no link, it returns 0.
 *******************************************************************************/
uint16_t link_first_acl_conn_handle()
{
    return ((link.conn[0].connection_status.conn_id) ? wiced_bt_dev_get_acl_conn_handle(link.conn[0].bd_addr, BT_TRANSPORT_LE) : 0);
}


/*******************************************************************************
 * Function Name: link_set_encrypted
 ********************************************************************************
 * Summary:
 *    Set the connection link encrypted state
 *******************************************************************************/
void link_set_encrypted(wiced_bool_t set)
{
    if (link.active)
    {
        link.active->encrypted = set;
    }
}

/*******************************************************************************
 * Function Name: link_is_encrypted
 ********************************************************************************
 * Summary:
 *    Return TRUE if link is encrypted
 *******************************************************************************/
wiced_bool_t link_is_encrypted()
{
    if (link.active)
    {
        return link.active->encrypted;
    }
    return FALSE;
}

/*******************************************************************************
 * Function Name: link_set_parameter_updated
 ********************************************************************************
 * Summary:
 *    Set the connection link parameter updated state
 *******************************************************************************/
void link_set_parameter_updated(wiced_bool_t set)
{
    if (link.active)
    {
        link.active->link_parameter_updated = set;
    }
}

/*******************************************************************************
 * Function Name: link_is_parameter_updated
 ********************************************************************************
 * Summary:
 *    Return TUE if the link parameter is updated
 *******************************************************************************/
wiced_bool_t link_is_parameter_updated()
{
    if (link.active)
    {
        return link.active->link_parameter_updated;
    }
    return FALSE;
}

/*******************************************************************************
 * Function Name: link_set_indication_pending
 ********************************************************************************
 * Summary:
 *    Set the indication flag is pending for this link
 *******************************************************************************/
void link_set_indication_pending(wiced_bool_t set)
{
    if (link.active)
    {
        link.active->indicate_pending = set;
    }
}

/*******************************************************************************
 * Function Name: link_is_indication_pending
 ********************************************************************************
 * Summary:
 *    Return TRUE if device is waiting for host indication confirmation
 *******************************************************************************/
wiced_bool_t link_is_indication_pending()
{
    if (link.active)
    {
        return link.active->indicate_pending;
    }
    return FALSE;
}

/*******************************************************************************
 * Function Name: link_set_bonded
 ********************************************************************************
 * Summary:
 *    Set the connection bonded state
 *******************************************************************************/
void link_set_bonded(wiced_bool_t set)
{
    if (link.active)
    {
        link.active->bonded = set;
    }
}

/*******************************************************************************
 * Function Name: link_is_bonded
 ********************************************************************************
 * Summary:
 *    Return TRUE if the link is bonded
 *
 * Parameters:
 *    none
 *
 * Return:
 *    bonded state
 *
 *******************************************************************************/
wiced_bool_t link_is_bonded()
{
    if (link.active)
    {
        return link.active->bonded;
    }
    return FALSE;
}

/*******************************************************************************
 * Function Name: link_up
 ********************************************************************************
 * Summary:
 *    This function should be called when link is established
 *******************************************************************************/
wiced_bt_gatt_status_t link_up( wiced_bt_gatt_connection_status_t * p_status )
{
    link_state_t * new_conn = NULL;

    // find the empty slot
    for (uint8_t idx=0; idx < MAX_CONN; idx++)
    {
        if ((link.conn[idx].connection_status.conn_id == p_status->conn_id) || !link.conn[idx].connection_status.conn_id)
        {
            if (link.conn[idx].connection_status.conn_id)
            {
                WICED_BT_TRACE("** Warning: connection id %04x is already up",  p_status->conn_id);
            }
            new_conn = &link.conn[idx];
            break;
        }
    }
    // If we cannot find an empty slot, we have reached the max connection we can support
    if (new_conn == NULL)
    {
        WICED_BT_TRACE("Max link %d reached",  MAX_CONN);
        return WICED_BT_GATT_NO_RESOURCES;
    }
    link.active = new_conn;

    // save the connection info
    memcpy(&new_conn->connection_status, p_status, sizeof(wiced_bt_gatt_connection_status_t));
    memcpy(new_conn->bd_addr, new_conn->connection_status.bd_addr, BD_ADDR_LEN);
    new_conn->transport = BT_TRANSPORT_LE;
    new_conn->acl_conn_handle = wiced_bt_dev_get_acl_conn_handle(new_conn->bd_addr, BT_TRANSPORT_LE);

    //configure ATT MTU size with peer device
    WICED_BT_TRACE("Configure MTU with size %d",  ENCODE_BUF_SIZE+5);
    wiced_bt_gatt_configure_mtu(link_conn_id(), ENCODE_BUF_SIZE+5);

    wiced_bt_l2cap_enable_update_ble_conn_params (new_conn->bd_addr, TRUE);

    // if this is a known host, we restore the cccd flags from NVRAM; otherwise, clear all flags */
    host_restore_cccd_flags(new_conn->bd_addr);

    // notify application the link is up
    app_link_up(p_status);

    return WICED_BT_GATT_SUCCESS;
}

/*******************************************************************************
 * Function Name: link_down
 ********************************************************************************
 * Summary:
 *    This function should be called when link is down
 *******************************************************************************/
wiced_bt_gatt_status_t link_down( wiced_bt_gatt_connection_status_t *p_status )
{
    link_state_t * conn = link_get_state(p_status->conn_id);

    if (conn == NULL)
    {
        // link down with invalid conn_id should not happen
        WICED_BT_TRACE("Invalid conn_id for link down event",  p_status->conn_id);
        return WICED_BT_GATT_DATABASE_OUT_OF_SYNC;
    }

    // delete the connection info
    memset(conn, 0, sizeof(link_state_t));

    link.active = NULL;

    // if we still have link is connected, we change active link to the connected link
    for (int idx=0; idx < MAX_CONN; idx++)
    {
        if (link.conn[idx].connection_status.conn_id)
        {
            link.active = &link.conn[idx];

            // restore or clear CCCD flags based on bd_addr info in NVRAM
            host_restore_cccd_flags(link.active->bd_addr);
            break;
        }
    }

    // notify application the link is down
    app_link_down(p_status);

    return WICED_BT_GATT_SUCCESS;
}

/*******************************************************************************
 * Function Name: link_is_connected
 ********************************************************************************
 * Summary:
 *    Return TRUE if the link is connected
 *******************************************************************************/
wiced_bool_t link_is_connected()
{
    return (wiced_bool_t) link.active;
}

/*******************************************************************************
 * Function Name: link_connection_status
 ********************************************************************************
 * Summary:
 *    returns current connection status data
 *******************************************************************************/
wiced_bt_gatt_connection_status_t * link_connection_status()
{
    return link.active ? &link.active->connection_status : NULL;
}

/********************************************************************
 * Function Name: bt_set_acl_conn_interval
 ********************************************************************
 * Summary:
 *  Change Set connection interval
 * ********************************************************************/
wiced_bool_t link_set_acl_conn_interval(uint16_t interval_min, uint16_t interval_max, uint16_t latency, uint16_t timeout)
{
    if (link.active)
    {
        wiced_bt_ble_get_connection_parameters(link.active->bd_addr, &link.active->conn_params);

        if ( (link.active->conn_params.conn_interval < interval_min) || (link.active->conn_params.conn_interval  > interval_max) )
        {
            WICED_BT_TRACE("Set connection interval from %d to min:%d, max:%d",link.active->conn_params.conn_interval, interval_min, interval_max);
            wiced_bt_l2cap_update_ble_conn_params(link.active->bd_addr, interval_min, interval_max, latency, timeout);
            return TRUE;
        }
    }
    return FALSE;
}

/********************************************************************
 * Function Name: link_get_acl_conn_param
 ********************************************************************
 * Summary:
 *  returns link connection paraemter
 * ********************************************************************/
wiced_bt_ble_conn_params_t * link_get_acl_conn_param()
{
    if (link.active)
    {
        return &link.active->conn_params;
    }
    // avoid return NULL
    return &link.conn[0].conn_params;
}

/* end of file */
