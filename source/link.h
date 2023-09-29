/*
 * $ Copyright YEAR Cypress Semiconductor $
 */

/**
 * file link.h
 *
 * Link management header file
 *
 * Abstract: This file defines an interface for managing link
 */
#pragma once

#include "wiced_bt_gatt.h"

#define BT_TRANSPORT_NONE   0

#define HID_LINK_CONNECTED    4
#define HID_LINK_DISCONNECTED 1
#define HID_LINK_MASK         0x7f
#define NON_ISOC_ACL_CONN_INTERVAL 6
#define NON_ISOC_ACL_LINK_SUPERVISION_TIMEOUT  200     // 2 sec timeout
#define ISOC_ACL_LINK_SUPERVISION_TIMEOUT     1500     // 15 sec timeout

#if BTSTACK_VER < 0x03000001
#define wiced_bt_dev_get_acl_conn_handle(a,t) link.conn[0].connection_status.conn_id
#endif
/********************************************************************
 * Function Name: link_transport
 ********************************************************************
 * Summary:
 *    Returns the link transport.
 *    (BT_TRANSPORT_NONE, BT_TRANSPORT_BR_EDR or BT_TRANSPORT_LE)
 ********************************************************************/
uint8_t link_transport();

/*******************************************************************************
 * Function Name: link_conn_id
 ********************************************************************************
 * Summary:
 *    Return last active link conn_id. If no link, it returns 0.
 *
 * Parameters:
 *    none
 *
 * Return:
 *    none
 *
 *******************************************************************************/
uint16_t link_conn_id();

/*******************************************************************************
 * Function Name: link_acl_conn_handle
 ********************************************************************************
 * Summary:
 *    Return last active ACL connection handle. If no link, it returns 0.
 *
 * Parameters:
 *    none
 *
 * Return:
 *    none
 *
 *******************************************************************************/
uint16_t link_acl_conn_handle();

/*******************************************************************************
 * Function Name: link_first_acl_conn_handle
 ********************************************************************************
 * Summary:
 *    Return first active ACL connection handle. If no link, it returns 0.
 *
 * Parameters:
 *    none
 *
 * Return:
 *    none
 *
 *******************************************************************************/
uint16_t link_first_acl_conn_handle();

/*******************************************************************************
 * Function Name: link_set_encrypted
 ********************************************************************************
 * Summary:
 *    Set the connection link encrypted state
 *
 * Parameters:
 *    wiced_bool_t  -- new state
 *
 * Return:
 *    none
 *
 *******************************************************************************/
void link_set_encrypted(wiced_bool_t set);

/*******************************************************************************
 * Function Name: link_is_encrypted
 ********************************************************************************
 * Summary:
 *    Return TRUE if link is encrypted
 *
 * Parameters:
 *    none
 *
 * Return:
 *    encrypted state
 *
 *******************************************************************************/
wiced_bool_t link_is_encrypted();

/*******************************************************************************
 * Function Name: link_set_parameter_updated
 ********************************************************************************
 * Summary:
 *    Set the connection link parameter updated state
 *
 * Parameters:
 *    conn_id       -- connection id
 *    wiced_bool_t  -- new state
 *
 * Return:
 *    none
 *
 *******************************************************************************/
void link_set_parameter_updated(wiced_bool_t set);

/*******************************************************************************
 * Function Name: link_is_parameter_updated
 ********************************************************************************
 * Summary:
 *    Return TRUE if the link parameter is updated
 *
 * Parameters:
 *    none
 *
 * Return:
 *    link_parameter_updated state
 *
 *******************************************************************************/
wiced_bool_t link_is_parameter_updated();

/*******************************************************************************
 * Function Name: link_set_indication_pending
 ********************************************************************************
 * Summary:
 *    Set the indication pending flag state
 *
 * Parameters:
 *    conn_id       -- connection id
 *    wiced_bool_t  -- new state
 *
 * Return:
 *    none
 *
 *******************************************************************************/
void link_set_indication_pending(wiced_bool_t set);

/*******************************************************************************
 * Function Name: link_is_indication_pending
 ********************************************************************************
 * Summary:
 *    Return TRUE if device is waiting for host indication confirmation
 *
 * Parameters:
 *    none
 *
 * Return:
 *    indication_pending state
 *
 *******************************************************************************/
wiced_bool_t link_is_indication_pending();

/*******************************************************************************
 * Function Name: link_set_bonded
 ********************************************************************************
 * Summary:
 *    Set the connection bonded state
 *
 * Parameters:
 *    conn_id       -- connection id
 *    wiced_bool_t  -- new state
 *
 * Return:
 *    none
 *
 *******************************************************************************/
void link_set_bonded(wiced_bool_t set);

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
wiced_bool_t link_is_bonded();

/*******************************************************************************
 * Function Name: link_up
 ********************************************************************************
 * Summary:
 *    This function should be called when link is established
 *
 * Parameters:
 *    p_status:  gatt connection status data
 *
 * Return:
 *    wiced_bt_gatt_status_t
 *
 *******************************************************************************/
wiced_bt_gatt_status_t link_up( wiced_bt_gatt_connection_status_t * p_status );

/*******************************************************************************
 * Function Name: link_down
 ********************************************************************************
 * Summary:
 *    This function should be called when link is down
 *
 * Parameters:
 *    p_status:  gatt connection status data
 *
 * Return:
 *    wiced_bt_gatt_status_t
 *
 *******************************************************************************/
wiced_bt_gatt_status_t link_down( wiced_bt_gatt_connection_status_t * p_status );

/*******************************************************************************
 * Function Name: link_is_connected
 ********************************************************************************
 * Summary:
 *    Return TRUE if the link is connected
 *
 * Parameters:
 *    none
 *
 * Return:
 *    link connected state
 *******************************************************************************/
wiced_bool_t link_is_connected();

/*******************************************************************************
 * Function Name: link_connection_status
 ********************************************************************************
 * Summary:
 *    returns current connection status data
 *
 * Parameters:
 *    none
 *
 * Return:
 *    link connected status
 *******************************************************************************/
wiced_bt_gatt_connection_status_t * link_connection_status();

/********************************************************************
 * Function Name: bt_set_acl_conn_interval
 ********************************************************************
 * Summary:
 *  Change Set connection interval
 * ********************************************************************/
wiced_bool_t link_set_acl_conn_interval(uint16_t interval_min, uint16_t interval_max, uint16_t latency, uint16_t timeout);

/********************************************************************
 * Function Name: link_get_acl_conn_param
 ********************************************************************
 * Summary:
 *  returns link connection paraemter
 * ********************************************************************/
wiced_bt_ble_conn_params_t * link_get_acl_conn_param();

/* end of file */
