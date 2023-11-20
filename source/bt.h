/*
 * $ Copyright YEAR Cypress Semiconductor $
 */

/**
 * file bt.h
 *
 * BT management header file
 *
 */
#ifndef BT_H__
#define BT_H__

#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_cfg.h"

typedef struct {
    wiced_bt_ble_advert_mode_t  adv_mode, intended_adv_mode;
    uint8_t                     adv_bdAddr[BD_ADDR_LEN];
    uint16_t                    conn_interval;      /**< updated connection interval */
    uint16_t                    conn_latency;       /**< updated connection latency */
    uint16_t                    supervision_timeout;/**< updated supervision timeout */
} aon_bt_data_t;

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
 * Function Name: bt_delayed_reconnect
 ********************************************************************
 * Summary:
 *  Delay for a period of time before entering reconnect
 *
 * Parameters:
 *    time_to_wait_in_ms -- time to wait in ms before entering reconnect
 *
 * Return:
 *    none
 *
 ********************************************************************/
void bt_delayed_reconnect(uint32_t time_to_wait_in_ms);

/********************************************************************
 * Function Name: bt_stop_reconnect_timer
 ********************************************************************
 * Summary:
 *  Stop delayed reconnect timer. If link is up, this function should be called to stop
 *  the reconnect timer.
 *
 * Parameters:
 *    none
 *
 * Return:
 *    none
 *
 ********************************************************************/
void bt_stop_reconnect_timer();

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
wiced_result_t bt_enter_pairing();

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
#define bt_disconnect() wiced_bt_gatt_disconnect(link_conn_id())

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
 * Function Name: bt_stop_filtering_adv()
 ********************************************************************
 * Summary:
 *  Saves the current adv mode and calls wiced_bt_start_advertisements()
 *******************************************************************/
#ifdef FILTER_ACCEPT_LIST_FOR_ADVERTISING
void bt_stop_filtering_adv();
#else
 #define bt_stop_filtering_adv()
#endif

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

#endif // BT_H__
/* end of file */
