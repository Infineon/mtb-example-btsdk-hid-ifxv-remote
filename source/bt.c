/*
 * $ Copyright YEAR Cypress Semiconductor $
 */

/**
 * file bt.c
 *
 * BT management functions
 *
 */

#include "wiced_bt_stack.h"
#include "wiced_timer.h"
#include "wiced_memory.h"
#include "cycfg_bt_settings.h"
#include "cycfg_gap.h"
#include "nvram.h"
#include "app.h"
#include "bt_v.h"

#if CHIP!=20829 && defined WICED_BT_TRACE_ENABLE
# undef WICED_BT_TRACE
# define WICED_BT_TRACE(format,...)   wiced_printf(NULL, 0, (format"\n"), ##__VA_ARGS__)
#endif

#if BT_TRACE==1
# define APP_BT_TRACE          WICED_BT_TRACE
# define APP_BT_TRACE2(...)
#elif BT_TRACE>1
# define APP_BT_TRACE          WICED_BT_TRACE
# define APP_BT_TRACE2         WICED_BT_TRACE
#else
# define APP_BT_TRACE(...)
# define APP_BT_TRACE2(...)
#endif

#define AUTO_RECONNECT_DELAY 500

#if BT_TRACE

#define STR(x) #x
static const char* eventStr[] =
{
    STR(BTM_ENABLED_EVT),
    STR(BTM_DISABLED_EVT),
    STR(BTM_POWER_MANAGEMENT_STATUS_EVT),
 #ifdef WICED_X
    STR(BTM_RE_START_EVT),
 #endif
    STR(BTM_PIN_REQUEST_EVT),
    STR(BTM_USER_CONFIRMATION_REQUEST_EVT),
    STR(BTM_PASSKEY_NOTIFICATION_EVT),
    STR(BTM_PASSKEY_REQUEST_EVT),
    STR(BTM_KEYPRESS_NOTIFICATION_EVT),
    STR(BTM_PAIRING_IO_CAPABILITIES_BR_EDR_REQUEST_EVT),
    STR(BTM_PAIRING_IO_CAPABILITIES_BR_EDR_RESPONSE_EVT),
    STR(BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT),
    STR(BTM_PAIRING_COMPLETE_EVT),
    STR(BTM_ENCRYPTION_STATUS_EVT),
    STR(BTM_SECURITY_REQUEST_EVT),
    STR(BTM_SECURITY_FAILED_EVT),
    STR(BTM_SECURITY_ABORTED_EVT),
    STR(BTM_READ_LOCAL_OOB_DATA_COMPLETE_EVT),
    STR(BTM_REMOTE_OOB_DATA_REQUEST_EVT),
    STR(BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT),
    STR(BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT),
    STR(BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT),
    STR(BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT),
    STR(BTM_BLE_SCAN_STATE_CHANGED_EVT),
    STR(BTM_BLE_ADVERT_STATE_CHANGED_EVT),
    STR(BTM_SMP_REMOTE_OOB_DATA_REQUEST_EVT),
    STR(BTM_SMP_SC_REMOTE_OOB_DATA_REQUEST_EVT),
    STR(BTM_SMP_SC_LOCAL_OOB_DATA_NOTIFICATION_EVT),
    STR(BTM_SCO_CONNECTED_EVT),
    STR(BTM_SCO_DISCONNECTED_EVT),
    STR(BTM_SCO_CONNECTION_REQUEST_EVT),
    STR(BTM_SCO_CONNECTION_CHANGE_EVT),
    STR(BTM_BLE_CONNECTION_PARAM_UPDATE),
    STR(BTM_BLE_PHY_UPDATE_EVT),
};

/****************************************************/
const char* getStackEventStr(wiced_bt_management_evt_t event)
{
    if (event >= sizeof(eventStr) / sizeof(uint8_t*))
    {
        return "** UNKNOWN **";
    }

    return eventStr[event];
}
#endif // BT_TRACE

/******************************************************************************
 * Defines
 ******************************************************************************/
typedef struct {
    wiced_bt_ble_advert_mode_t  adv_mode;
    uint8_t                     adv_bdAddr[BD_ADDR_LEN];
    uint16_t                    conn_interval;      /**< updated connection interval */
    uint16_t                    conn_latency;       /**< updated connection latency */
    uint16_t                    supervision_timeout;/**< updated supervision timeout */
    wiced_timer_t               reconnect_timer;
} bt_data_t;

/******************************************************************************
 * Structures
 ******************************************************************************/
static bt_data_t bt = {0};

/******************************************************************************
 *     Private Functions
 ******************************************************************************/

static void reconnect_timer_cb( TIMER_PARAM_TYPE arg )
{
    APP_BT_TRACE("reconnect_timer_cb");
    bt_enter_reconnect();
}

/********************************************************************
 * Function Name: app_bt_management
 ********************************************************************
 * Summary:
 *  This is the callback function from BT Management.
 ********************************************************************/
static wiced_result_t app_bt_management(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t result = WICED_BT_SUCCESS;
    uint8_t *p_keys;

    APP_BT_TRACE("=== %s (%d)", getStackEventStr(event), event);    // hidd default handler

    switch( event )
    {
        /* Bluetooth  stack enabled */
        case BTM_ENABLED_EVT:
            if ( p_event_data->enabled.status == WICED_BT_SUCCESS )
            {
#if BTSTACK_VER >= 0x03000001
                wiced_bt_dev_read_local_addr_ext(&bt.dev);      // read extended device info
                WICED_BT_TRACE("Local Addr: %B", dev_info()->local_addr);
#else
 #ifdef LE_LOCAL_PRIVACY_SUPPORT
                WICED_BT_TRACE("RPA: %B", wiced_btm_get_private_bda());
 #else
                {
                    wiced_bt_device_address_t  bda = { 0 };
                    wiced_bt_dev_read_local_addr(bda);
                    WICED_BT_TRACE("Local Addr: %B", bda);
                }
 #endif
#endif
                host_init();
                app_init();
            }
            else
            {
                WICED_BT_TRACE("** BT Enable failed, status:%d", p_event_data->enabled.status);
            }
            break;

        case BTM_USER_CONFIRMATION_REQUEST_EVT:
            APP_BT_TRACE("Numeric_value: %d", p_event_data->user_confirmation_request.numeric_value);
            wiced_bt_dev_confirm_req_reply( WICED_BT_SUCCESS , p_event_data->user_confirmation_request.bd_addr);
            break;

        case BTM_PASSKEY_NOTIFICATION_EVT:
            APP_BT_TRACE("BDA %B, Key %d", p_event_data->user_passkey_notification.bd_addr, p_event_data->user_passkey_notification.passkey );
            wiced_bt_dev_confirm_req_reply(WICED_BT_SUCCESS, p_event_data->user_passkey_notification.bd_addr );
            break;

        case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:
            /* save keys to NVRAM */
            p_keys = (uint8_t *)&p_event_data->local_identity_keys_update;
            nvram_write ( VS_ID_LOCAL_IDENTITY, p_keys, sizeof( wiced_bt_local_identity_keys_t ));
            break;

        case  BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
            /* read keys from NVRAM */
            p_keys = (uint8_t *)&p_event_data->local_identity_keys_request;
            if (!nvram_read( VS_ID_LOCAL_IDENTITY, p_keys, sizeof(wiced_bt_local_identity_keys_t)))
            {
                APP_BT_TRACE("Local Identity Key not available");
                result = WICED_BT_ERROR;
            }
            else
            {
                APP_BT_TRACE2("Local Identity Key %A", p_keys, BTM_SECURITY_LOCAL_KEY_DATA_LEN);
            }
            break;

        case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
            {
                wiced_bt_device_link_keys_t *p_link_keys = &p_event_data->paired_device_link_keys_update;
                APP_BT_TRACE("BdAddr:%B", p_link_keys->bd_addr );
                host_set_link_key(p_link_keys->bd_addr, p_link_keys, BT_TRANSPORT_LE);
                APP_BT_TRACE("mask           :  x%02X", p_link_keys->key_data.le_keys_available_mask);
                APP_BT_TRACE("LE Addr Type   :  %d", p_link_keys->key_data.ble_addr_type);
                APP_BT_TRACE("Static AddrType:  %d", p_link_keys->key_data.static_addr_type);
                APP_BT_TRACE("Static Addr    :  %B", p_link_keys->key_data.static_addr);
                APP_BT_TRACE("  irk: %A", &(p_link_keys->key_data.le_keys.irk), LINK_KEY_LEN);
#if SMP_INCLUDED == TRUE && SMP_LE_SC_INCLUDED == TRUE
                APP_BT_TRACE(" pltk: %A", &(p_link_keys->key_data.le_keys.pltk), LINK_KEY_LEN);
                APP_BT_TRACE("pcsrk: %A", &(p_link_keys->key_data.le_keys.pcsrk), LINK_KEY_LEN);
                APP_BT_TRACE(" lltk: %A", &(p_link_keys->key_data.le_keys.lltk), LINK_KEY_LEN);
                APP_BT_TRACE("lcsrk: %A", &(p_link_keys->key_data.le_keys.lcsrk), LINK_KEY_LEN);
#else
                APP_BT_TRACE("  ltk: %A", &(p_link_keys->key_data.le_keys.ltk), LINK_KEY_LEN);
                APP_BT_TRACE(" csrk: %A", &(p_link_keys->key_data.le_keys.csrk), LINK_KEY_LEN);
#endif
            }
            break;

        case  BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
            {
                wiced_bt_device_link_keys_t * link_key = host_get_link_key(p_event_data->paired_device_link_keys_request.bd_addr);
                if ((link_key != NULL) && (&p_event_data->paired_device_link_keys_request != NULL))
                {
                    memcpy(&p_event_data->paired_device_link_keys_request, link_key, sizeof(wiced_bt_device_link_keys_t));
                    link_set_bonded(TRUE); // We have the link key, it is a bonded device
                }
                else
                {
                    APP_BT_TRACE("No link key available for %B", p_event_data->paired_device_link_keys_request.bd_addr);
                    result = WICED_BT_ERROR;
                }
            }
            break;

        case BTM_ENCRYPTION_STATUS_EVT:
            APP_BT_TRACE("result=%d (%sencrypted)", p_event_data->encryption_status.result, p_event_data->encryption_status.result == WICED_SUCCESS? "": "not ");
            link_set_encrypted(p_event_data->encryption_status.result == WICED_SUCCESS);
            break;

        case BTM_PAIRING_COMPLETE_EVT:
            if(p_event_data->pairing_complete.transport == BT_TRANSPORT_LE)
            {
                wiced_bt_dev_ble_pairing_info_t * p_info;
                p_info =  &p_event_data->pairing_complete.pairing_complete_info.ble;

                if (!p_info->reason )
                {
                    APP_BT_TRACE("Pairing Success");
                    link_pairing_complete(p_event_data->pairing_complete.bd_addr, p_info);
                }
                else
                {
                    //SMP result callback: failed
                    WICED_BT_TRACE("Pairing failed reason:%d", p_info->reason);
                    bt_disconnect();
                }
            }
            else
            {
                 APP_BT_TRACE("Not LE BTM_PAIRING_COMPLETE_EVT (Transport=%d), ignored", p_event_data->pairing_complete.transport);
            }
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
            {
                wiced_bt_dev_ble_io_caps_req_t * cap_req = &p_event_data->pairing_io_capabilities_ble_request;
                cap_req->local_io_cap = BTM_IO_CAPABILITIES_NONE;
                cap_req->oob_data = BTM_OOB_NONE;
                cap_req->auth_req = BTM_LE_AUTH_REQ_SC | BTM_LE_AUTH_REQ_BOND;              /* LE sec bonding */
                cap_req->max_key_size = 16;
                cap_req->init_keys = (BTM_LE_KEY_PENC|BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_PLK);
                cap_req->resp_keys = (BTM_LE_KEY_PENC|BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_PLK);
            }
            break;

        case BTM_SECURITY_REQUEST_EVT:
            if (host_transport() == BT_TRANSPORT_LE)
            {
                APP_BT_TRACE("Clear CCCD's");
                host_set_cccd_flags(p_event_data->security_request.bd_addr, 0, 0);
            }
             /* Use the default security */
            wiced_bt_ble_security_grant(p_event_data->security_request.bd_addr,  WICED_BT_SUCCESS);
            break;

        case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
            {
                wiced_bt_ble_advert_mode_t new_adv_mode = p_event_data->ble_advert_state_changed;

                if (new_adv_mode == BTM_BLE_ADVERT_OFF && !link_is_connected())
                {
                    switch (bt.adv_mode) {
#if !defined(FILTER_ACCEPT_LIST_FOR_ADVERTISING) && defined(SWITCH_DIRECT_TO_UNDIRECT_ADV)
                    case BTM_BLE_ADVERT_DIRECTED_LOW:
                        result = bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, host_addr_type(), bt.adv_bdAddr);
                        // start high duty cycle directed advertising.
                        if (result != WICED_BT_SUCCESS)
                        {
                            WICED_BT_TRACE("Failed to start high duty cycle undirected advertising!!!");
                        }
                        return result;
#endif
                    // The adv is stopped. Check the flags to see if we should restart reconnect
                    default:
                        // Stop adv.
                        bt_stop_filtering_adv();

#if defined(AUTO_RECONNECT) || defined(ENDLESS_LE_ADVERTISING)
                        // Restart reconnect if the compiler flags says so
                        bt_delayed_reconnect(AUTO_RECONNECT_DELAY);
#endif
                        break;
                    }
                }
                app_adv_state_changed(bt.adv_mode, new_adv_mode);
                bt.adv_mode = new_adv_mode;
            }
            break;

        case BTM_BLE_SCAN_STATE_CHANGED_EVT:
            APP_BT_TRACE("scan state changed to %d", p_event_data->ble_scan_state_changed );
            break;

        case BTM_BLE_CONNECTION_PARAM_UPDATE:
            if (p_event_data->ble_connection_param_update.status == WICED_BT_SUCCESS)
            {
                WICED_BT_TRACE("Link parameter updated to interval:%d latency:%d timeout:%d",
                        p_event_data->ble_connection_param_update.conn_interval,
                        p_event_data->ble_connection_param_update.conn_latency,
                        p_event_data->ble_connection_param_update.supervision_timeout);
                link_conn_update_complete(&p_event_data->ble_connection_param_update);
            }
            break;

		case BTM_BLE_PHY_UPDATE_EVT:
			APP_BT_TRACE("Phy update - status:%d Tx:%d Rx:%d BDA %B", p_event_data->ble_phy_update_event.status,
				p_event_data->ble_phy_update_event.tx_phy,
				p_event_data->ble_phy_update_event.rx_phy,
				p_event_data->ble_phy_update_event.bd_address);
			break;

        default:
            WICED_BT_TRACE("Unhandled management event: %d!!!", event );
            break;
    }

    return result;
}

/******************************************************************************
 *     Public Functions
 ******************************************************************************/

/********************************************************************
 * Function Name: bt_enter_reconnect
 ********************************************************************
 * Summary:
 *  If the device is paired, it starts directed adv or undirect with filter
 *  for reconnecting to paired host.
 ********************************************************************/
void bt_enter_reconnect(void)
{
    APP_BT_TRACE("bt_enter_reconnect");

    //if low battery shutdown, do nothing
    if (wiced_hal_batmon_is_low_battery_shutdown())
    {
        WICED_BT_TRACE("Battery is in shutdown state, aborting reconnect", bt.adv_bdAddr);
        return;
    }

    //if reconnect timer is running, stop it
    bt_stop_reconnect_timer();

    if (host_is_paired())
    {
        if (!link_is_connected())
        {
            //NOTE!!! wiced_bt_start_advertisement could modify the value of bdAddr, so MUST use a copy.
            memcpy(bt.adv_bdAddr, host_addr(), BD_ADDR_LEN);
            APP_BT_TRACE("Enter reconnect to %B, type:%d", bt.adv_bdAddr, host_addr_type());

            // start high duty cycle directed advertising.
            if (bt_start_advertisements(BTM_BLE_ADVERT_DIRECTED_HIGH, host_addr_type(), bt.adv_bdAddr))
            {
                WICED_BT_TRACE("Failed to start high duty cycle directed advertising!!!");
            }
        }
    }
    else
    {
        APP_BT_TRACE("Enter reconnect -- not paired");
    }
}

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
void bt_delayed_reconnect(uint32_t time_to_wait_in_ms)
{
    APP_BT_TRACE("reconnect timer started for %d ms", time_to_wait_in_ms);
    wiced_start_timer(&bt.reconnect_timer, time_to_wait_in_ms);
}

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
void bt_stop_reconnect_timer()
{
    APP_BT_TRACE("reconnect timer stopped");
    wiced_stop_timer(&bt.reconnect_timer);
}

/********************************************************************
 * Function Name: bt_enter_pairing
 ********************************************************************
 * Summary:
 *  Starts undirect advs. If it is already connected and not allowed to enter pairing while connected,
 *  it returns error.
 ********************************************************************/
wiced_result_t bt_enter_pairing()
{
    APP_BT_TRACE("bt_enter_pairing");
#ifndef CONNECTED_ADVERTISING_SUPPORTED
    if (link_is_connected())
    {
        return WICED_ERROR;
    }
#endif
    return bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL);
}

/********************************************************************
 * Function Name: bt_init
 ********************************************************************
 * Summary:
 *  This is one of the first function to be called upon device reset.
 *  It initialize BT Stack. When BT stack is up, it will call application
 *  to continue system initialization.
 ********************************************************************/
wiced_result_t bt_init()
{
    APP_BT_TRACE("bt_init");

    wiced_init_timer( &bt.reconnect_timer, reconnect_timer_cb, 0, WICED_MILLI_SECONDS_TIMER );
    return bt_v_init(app_bt_management);
}

/********************************************************************
 * Function Name: bt_enter_connect()
 ********************************************************************
 * Summary:
 *  When the device is paired, it reconnect to the paired host.
 *  Otherwise, it enter pairing for a new host.
 *******************************************************************/
void bt_enter_connect()
{
    APP_BT_TRACE("bt_enter_connect");
    host_is_paired() ? bt_enter_reconnect() : bt_enter_pairing();
}

#ifdef FILTER_ACCEPT_LIST_FOR_ADVERTISING
/********************************************************************
 * Function Name: bt_stop_filtering_adv()
 ********************************************************************
 * Summary:
 *  Saves the current adv mode and calls wiced_bt_start_advertisements()
 *******************************************************************/
void bt_stop_filtering_adv()
{
    wiced_btm_ble_update_advertisement_filter_policy(BTM_BLE_ADV_POLICY_ACCEPT_CONN_AND_SCAN);   //  accept all scan and conn requests
    wiced_bt_ble_clear_filter_accept_list();
    sds_set_filtering_adv(FALSE, NULL);
}
#endif

/********************************************************************
 * Function Name: bt_start_advertisements()
 ********************************************************************
 * Summary:
 *  Saves the current adv mode and calls wiced_bt_start_advertisements()
 *******************************************************************/
wiced_result_t bt_start_advertisements(wiced_bt_ble_advert_mode_t advert_mode, wiced_bt_ble_address_type_t directed_advertisement_bdaddr_type, wiced_bt_device_address_ptr_t directed_advertisement_bdaddr_ptr)
{
#ifdef FILTER_ACCEPT_LIST_FOR_ADVERTISING
    switch (advert_mode) {
    case BTM_BLE_ADVERT_DIRECTED_HIGH:
        APP_BT_TRACE("Reconnect using filtering undirected adv");
        wiced_bt_ble_update_advertising_filter_accept_list(WICED_TRUE, directed_advertisement_bdaddr_ptr); // add the device to the list
        wiced_btm_ble_update_advertisement_filter_policy(BTM_BLE_ADV_POLICY_FILTER_CONN_FILTER_SCAN);    // accept scan and connect request only from the list
        sds_set_filtering_adv(TRUE, directed_advertisement_bdaddr_ptr);
        advert_mode = BTM_BLE_ADVERT_UNDIRECTED_HIGH;
        break;
    case BTM_BLE_ADVERT_OFF:
        bt_stop_filtering_adv();
        break;
    }
#endif

    wiced_result_t result = wiced_bt_start_advertisements( advert_mode, directed_advertisement_bdaddr_type, directed_advertisement_bdaddr_ptr);

    if (result != WICED_SUCCESS)
    {
        bt_stop_filtering_adv();
    }

    return result;
}

/* end of file */
