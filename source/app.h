/*
 * $ Copyright YEAR Cypress Semiconductor $
 */

/**
 * file app.h
 *
 * This is the application for IFXV-Device. This application should be used tested with IFX-Voice supported host.
 *
 */
#ifndef APP_H__
#define APP_H__

#include "hidd_lib.h"
#include "wiced_bt_trace.h"
#include "hci_control_api.h"
#include "bt.h"
#include "gatt.h"
#include "host.h"
#include "link.h"
#include "led.h"
#include "button.h"
#include "battery.h"
#include "key.h"
#include "hci.h"
#include "audio.h"
#include "app_v.h"
#include "ota.h"
#include "ir.h"
#include "findme.h"
#include "sds.h"
#include "nvram.h"

#ifdef WICED_EVAL
 #define RED_LED        WICED_PLATFORM_LED_2
 #define LINK_LED       WICED_PLATFORM_LED_1
#else
 #define RED_LED        WICED_PLATFORM_LED_1
 #define LINK_LED       WICED_PLATFORM_LED_2
#endif

#define CONNECT_KEY_INDEX  0

#if CUSTOM_KEY_MATRIX == 1
 #define NUM_KEYSCAN_ROWS   3  // Num of Rows in keyscan matrix
 #define NUM_KEYSCAN_COLS   5  // Num of Cols in keyscan matrix

 #define AUDIO_KEY_INDEX    0
 #define HOME_KEY_INDEX     9
 #define BACK_KEY_INDEX     6
 #define IR_KEY_INDEX       14 // Doesn't exists, use NC key index
 #define MUTE_KEY_INDEX     8
#else
 #define NUM_KEYSCAN_ROWS   3  // Num of Rows in keyscan matrix
 #define NUM_KEYSCAN_COLS   7  // Num of Cols in keyscan matrix

 #define AUDIO_KEY_INDEX    1
 #define HOME_KEY_INDEX     13
 #define BACK_KEY_INDEX     14
 #define IR_KEY_INDEX       17
 #define MUTE_KEY_INDEX     20
#endif

#define CONNECT_INDEX       HOME_KEY_INDEX // HOME button hold for 10 sec to enter pairing
#define NUM_MAX_KEY (NUM_KEYSCAN_ROWS*NUM_KEYSCAN_COLS)

#define CONNECT_COMBO       (1<<CONNECT_INDEX)   // CONNECT COMBO BITS
#define CONNECT_COMBO_HOLD_TIME 10               // Hold for 10 sec.

#ifdef SUPPORT_KEYSCAN
 #define key_active() keyscanActive()
#else
 #define key_active() button_down()
#endif

/*******************************************************************************
 * macros
 ********************************************************************************/

/*******************************************************************************
 * extern
 *******************************************************************************/

/*******************************************************************************
 * Report ID defines
 *******************************************************************************/
// Input report id
typedef enum {
    RPT_ID_IN_STD_KEY      =0x01,
    RPT_ID_IN_BIT_MAPPED   =0x02,
    RPT_ID_IN_BATTERY      =0x03,
    RPT_ID_IN_MEDIA_KEY    =0x0a,
    RPT_ID_IN_CNT_CTL      =0xcc,
    RPT_ID_IN_AUDIO_DATA   =HIDD_VOICE_REPORT_ID,
    RPT_ID_IN_AUDIO_CTL    =HIDD_VOICE_CTL_REPORT_ID,
    RPT_ID_IN_NOT_USED     =0xff,
    RPT_ID_CLIENT_CHAR_CONF=0xff,
} rpt_id_in_e;

// Output report id
typedef enum {
    RPT_ID_OUT_KB_LED      =0x01,
    RPT_ID_OUT_AUDIO_DATA  =HIDD_VOICE_REPORT_ID,
    RPT_ID_OUT_AUDIO_CTL   =HIDD_VOICE_CTL_REPORT_ID,
} rpt_id_out_e;

/******************************************************************************
 *     Public Function Definitions
 ******************************************************************************/

/********************************************************************
 * Function Name: app_check_cccd_flags()
 ********************************************************************
 * Summary: This function is called from gatt default attribute write.
 *          Check if the write handle is CCCD handle. If it is, update
 *          the flags.
 * Parameters:
 *  uint16_t handel - handle to be checked
 *
 * Return:
 *  TRUE if the handle is CCCD handle.
 *
 *******************************************************************/
wiced_bool_t app_check_cccd_flags( uint16_t handle );

/********************************************************************
 * Function Name: app_remove_host_bonding
 ********************************************************************
 * Summary:
 *  Virtual cable unplug.
 *  This function will remove all HID host information from NVRAM.
 *
 * Parameters:
 *    none
 *
 * Return:
 *    none
 *
 ********************************************************************/
void app_remove_host_bonding(void);

/********************************************************************
 * Function Name: app_gatt_write_handler
 ********************************************************************
 * Summary:
 *  This function is called when GATT handle write req event is recieved.
 *
 * Parameters:
 *  uint16_t conn_id    -- Connection ID
 *  wiced_bt_gatt_write_req_t *      -- Pointer to gatt_write data
 *
 * Return:
 *  wiced_bt_gatt_status_t
 *
 *******************************************************************/
wiced_bt_gatt_status_t app_gatt_write_handler( uint16_t conn_id, wiced_bt_gatt_write_req_t * p_wr_data );

/********************************************************************
 * Function Name: app_link_up
 ********************************************************************
 * Summary:
 *  This function is called when link is up
 *
 * Parameters:
 *  wiced_bt_gatt_connection_status_t * p_status -- pointer to the connection status.
 *
 * Return:
 *  none
 *
 *******************************************************************/
void app_link_up(wiced_bt_gatt_connection_status_t * p_status);

/********************************************************************
 * Function Name: app_link_down
 ********************************************************************
 * Summary:
 *  This function is called when link is down
 *
 * Parameters:
 *  wiced_bt_gatt_connection_status_t * p_status -- pointer to the connection status.
 *
 * Return:
 *  none
 *
 *******************************************************************/
void app_link_down(wiced_bt_gatt_connection_status_t * p_status);

/********************************************************************
 * Function Name: app_adv_state_changed
 ********************************************************************
 * Summary:
 *  This function is called when advertisment state is changed
 *
 * Parameters:
 *  wiced_bt_ble_advert_mode_t adv  -- new advertisment mode.
 *
 * Return:
 *  none
 *
 *******************************************************************/
void app_adv_state_changed(wiced_bt_ble_advert_mode_t old_adv, wiced_bt_ble_advert_mode_t adv);

/********************************************************************
 * Function Name: app_shutdown
 ********************************************************************
 * Summary:
 *  This function is called when battery level reaches shutdown voltage.
 *  The device should put power consumption to the lowest to prevent battery
 *  leakage before shutdown.
 *
 * Parameters:
 *  none
 *
 * Return:
 *  none
 *
 *******************************************************************/
void app_shutdown(void);

/********************************************************************
 * Function Name: app_init
 ********************************************************************
 * Summary:
 *  When BT Management Stack is initialized successfully, this function is called.
 *
 * Parameters:
 *  none
 *
 * Return:
 *  none
 *
 *******************************************************************/
wiced_result_t app_init(void);

/********************************************************************
 * Function Name: application_start()
 ********************************************************************
 *  Entry point to the application. Set device configuration and start Bluetooth
 *  stack initialization.  The actual application initialization (app_init) will
 *  be called when stack reports that Bluetooth device is ready.
 *
 * Parameters:
 *  none
 *
 * Return:
 *  none
 *
 ********************************************************************/
void application_start( void );

#endif // APP_H__
