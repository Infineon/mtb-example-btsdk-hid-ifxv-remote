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
#ifndef LINK_H__
#define LINK_H__

#include "wiced_bt_gatt.h"
#include "wiced_bt_ble.h"
#include "wiced_timer.h"

#define BT_TRANSPORT_NONE   0

#define HID_LINK_CONNECTED    4
#define HID_LINK_DISCONNECTED 1
#define HID_LINK_MASK         0x7f

#if BTSTACK_VER < 0x03000001
#define wiced_bt_dev_get_acl_conn_handle(a,t) link.conn[0].connection_status.conn_id
#endif

#define MAX_CONN            2

#define DEFERRED_LINK_PARAM_UPDATE_TIME 20  // differ link parameter update for 20 sec after link is up

/******************************************************************************
 *  typedef
 ******************************************************************************/
typedef struct
{
    wiced_bt_gatt_connection_status_t   connection_status;
    uint16_t                            acl_conn_handle;
    wiced_bt_device_address_t           bd_addr;                    // Remote device address (from connection_status)
    wiced_bt_ble_conn_params_t          conn_params;
    wiced_bt_dev_ble_pairing_info_t     pairing_info;

    uint8_t                             transport:2;                // 0:no link (BT_TRANSPORT_NONE), 1:Classic (BT_TRANSPORT_BR_EDR) , 2:LE (BT_TRANSPORT_LE)
    uint8_t                             encrypted:1;                // encrypted:1, not encrypted:0
    uint8_t                             link_parameter_updated:1;   // 1:connection parameter update, 0: no connection parameter update since connection
    uint8_t                             indicate_pending:1;         // 1:waiting for indication confirm, 0:no indication pending
    uint8_t                             bonded:1;                   // 1:bonded, 0:not bonded
} link_state_t;

/******************************************************************************
 *  link struct
 ******************************************************************************/
typedef struct
{
    link_state_t   conn[MAX_CONN];
    link_state_t * active;
#ifdef SKIP_CONNECT_PARAM_UPDATE_EVEN_IF_NO_PREFERED
    wiced_timer_t  deferred_link_update_timer;
#endif
} link_t;

extern link_t link;

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
 * Function Name: link_conn_update_complete
 ********************************************************************************
 * Summary:
 *    This function is called on Link parameter update complete event
 *
 * Parameters:
 *    p_data       -- pointer to event data
 *
 * Return:
 *    none
 *
 *******************************************************************************/
void link_conn_update_complete(wiced_bt_ble_connection_param_update_t * p_data);

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

/********************************************************************************
 * Function Name: UINT8 link_params_update_is_expected()
 ********************************************************************************
 * Summary: Check the le parameters is updated to expected setting or not
 *
 * Parameters:
 *  none
 *
 * Return:
 *  FALSE : not expected configuration le parameters
 *  TRUE  : expected configuration le parameters
 *
 *******************************************************************************/
wiced_bool_t link_params_update_is_expected(void);

/********************************************************************************
 * Function Name: UINT8 link_pairing_complete()
 ********************************************************************************
 * Summary: This function is called on pairing complete event
 *
 * Parameters:
 *  addr - bd address
 *  p_info - pointer of pairing information
 *
 * Return:
 *
 *******************************************************************************/
void link_pairing_complete( BD_ADDR addr, wiced_bt_dev_ble_pairing_info_t * p_info );

/********************************************************************************
 * Function Name: void link_check_conn_parameter()
 ********************************************************************************
 * Summary:
 *    This function checks for current link parameter. If it is not within perfered configration
 *    setting, it will request new link parameter based on the configuration settings.
 *
 * Parameters:
 *    none
 *
 * Return:
 *    none
 *******************************************************************************/
void link_check_conn_parameter();

/********************************************************************************
 * Summary:
 *    This function is called once at power up to initialize link submodule.
 *
 * Parameters:
 *    none
 *
 * Return:
 *    none
 *******************************************************************************/
void link_init();

#endif // LINK_H__
/* end of file */
