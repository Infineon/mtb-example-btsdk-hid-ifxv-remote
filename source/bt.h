/*
 * $ Copyright YEAR Cypress Semiconductor $
 */

/**
 * file bt.h
 *
 * BT management header file
 *
 */

#pragma once

#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_cfg.h"

/***********************************************************
 *  wiced_result_t bt_init()
 ***********************************************************
 * Summary:
 *   Initialize BT Management stack
 *
 * Parameters:
 *   none
 *
 * Return:
 *   wiced_result_t
 *
 ***********************************************************/
wiced_result_t bt_init();

/********************************************************************
 * Function Name: bt_enter_reconnect
 ********************************************************************
 * Summary:
 *  It the device is paired, it starts directed adv or undirect with filter
 *  for reconnecting to paired host.
 *
 * Parameters:
 *    none
 *
 * Return:
 *    none
 *
 ********************************************************************/
void bt_enter_reconnect(void);

/********************************************************************
 * Function Name: bt_enter_pairing
 ********************************************************************
 * Summary:
 *  Starts undirect advs
 *
 * Parameters:
 *    none
 *
 * Return:
 *    none
 *
 ********************************************************************/
#define bt_enter_pairing() bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL)

/********************************************************************
 * Function Name: bt_stop_advertisement
 ********************************************************************
 * Summary:
 *  Stops advertisement.
 *
 * Parameters:
 *    none
 *
 * Return:
 *    none
 *
 ********************************************************************/
#define bt_stop_advertisement() bt_start_advertisements(BTM_BLE_ADVERT_OFF, 0, NULL)

/********************************************************************
 * Function Name: bt_disconnect
 ********************************************************************
 * Summary:
 *  Disconnect the link.
 *
 * Parameters:
 *    none
 *
 * Return:
 *    none
 *
 ********************************************************************/
#define bt_disconnect() wiced_bt_gatt_disconnect( link_conn_id() )

/********************************************************************
 * Function Name: bt_get_advertising_mode
 ********************************************************************
 * Summary:
 *  It returns the current advertizing mode. If returns 0 (BTM_BLE_ADVERT_OFF) if the
 *  device is not advertising.
 *
 * Parameters:
 *    none
 *
 * Return:
 *    wiced_bt_ble_advert_mode_t
 *
 ********************************************************************/
#define bt_get_advertising_mode() wiced_bt_ble_get_current_advert_mode()

/********************************************************************
 * Function Name: bt_is_advertising
 ********************************************************************
 * Summary:
 *  Returns TRUE (none zero) if the device is advertising
 *
 * Parameters:
 *    none
 *
 * Return:
 *    Returns TRUE (none zero) or FALSE (0)
 *
 ********************************************************************/
#define bt_is_advertising() bt_get_advertising_mode()

/********************************************************************
 * Function Name: bt_enter_connect()
 ********************************************************************
 * Summary:
 *  When the device is paired, it reconnect to the paired host.
 *  Otherwise, it enter pairing for a new host.
 *
 * Parameters:
 *  none
 *
 * Return:
 *  none
 *
 *******************************************************************/
void bt_enter_connect();

/********************************************************************
 * Function Name: bt_start_advertisements()
 ********************************************************************
 * Summary:
 *  Saves the current adv mode and calls wiced_bt_start_advertisements()
 *
 * Parameters:
 *  see wiced_bt_start_advertisements()
 *
 * Return:
 *  wiced_result_t
 *
 *******************************************************************/
wiced_result_t bt_start_advertisements(wiced_bt_ble_advert_mode_t advert_mode, wiced_bt_ble_address_type_t directed_advertisement_bdaddr_type, wiced_bt_device_address_ptr_t directed_advertisement_bdaddr_ptr);


/* end of file */
