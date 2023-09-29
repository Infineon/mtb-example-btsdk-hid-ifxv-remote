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

/******************************************************************************
 * Defines
 ******************************************************************************/
typedef struct {
    wiced_bt_ble_advert_mode_t  adv_mode, intended_adv_mode;
    uint8_t                     adv_bdAddr[BD_ADDR_LEN];
    uint16_t                    conn_interval;      /**< updated connection interval */
    uint16_t                    conn_latency;       /**< updated connection latency */
    uint16_t                    supervision_timeout;/**< updated supervision timeout */
} bt_data_t;

/******************************************************************************
 * Structures
 ******************************************************************************/
static bt_data_t bt = {0};

/******************************************************************************
 *     Private Functions
 ******************************************************************************/

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

    APP_BT_TRACE("=== BT stack event %d", event);

    // hidd default handler
    switch( event )
    {
        /* Bluetooth  stack enabled */
        case BTM_ENABLED_EVT:
            APP_BT_TRACE("BTM_ENABLED_EVT");
            if ( p_event_data->enabled.status == WICED_BT_SUCCESS )
            {
#ifdef ENABLE_BT_SPY_LOG
                /* Register HCI Trace callback */
                wiced_bt_dev_register_hci_trace( (wiced_bt_hci_trace_cback_t*) cybt_debug_uart_send_hci_trace );
#else
                hci_control_enable_trace();
#endif
                WICED_BT_TRACE("BTM initialized");
#if BTSTACK_VER >= 0x03000001
                wiced_bt_dev_read_local_addr_ext(&bt.dev);      // read extended device info
                WICED_BT_TRACE("Local Addr: %B", dev_info()->local_addr);
#else
                {
                    wiced_bt_device_address_t  bda = { 0 };
                    wiced_bt_dev_read_local_addr(bda);
                    WICED_BT_TRACE("Local Addr: %B", bda);
                }
#endif
                host_init();
                app_init();
#ifdef AUTO_PAIRING
                bt_enter_pairing();
#endif
            }
            else
            {
                WICED_BT_TRACE("** BT Enable failed, status:%d", p_event_data->enabled.status);
            }
            break;

        case BTM_USER_CONFIRMATION_REQUEST_EVT:
            APP_BT_TRACE("BTM_USER_CONFIRMATION_REQUEST_EVT: Numeric_value: %d", p_event_data->user_confirmation_request.numeric_value);
            wiced_bt_dev_confirm_req_reply( WICED_BT_SUCCESS , p_event_data->user_confirmation_request.bd_addr);
            break;

        case BTM_PASSKEY_NOTIFICATION_EVT:
            APP_BT_TRACE("PassKey Notification. BDA %B, Key %d", p_event_data->user_passkey_notification.bd_addr, p_event_data->user_passkey_notification.passkey );
            wiced_bt_dev_confirm_req_reply(WICED_BT_SUCCESS, p_event_data->user_passkey_notification.bd_addr );
            break;

        case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:
            APP_BT_TRACE("BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT");
            /* save keys to NVRAM */
            p_keys = (uint8_t *)&p_event_data->local_identity_keys_update;
            nvram_write ( VS_ID_LOCAL_IDENTITY, p_keys, sizeof( wiced_bt_local_identity_keys_t ));
            break;

        case  BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
            APP_BT_TRACE("BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT");
            /* read keys from NVRAM */
            p_keys = (uint8_t *)&p_event_data->local_identity_keys_request;
            if (!nvram_read( VS_ID_LOCAL_IDENTITY, p_keys, sizeof(wiced_bt_local_identity_keys_t)))
            {
                WICED_BT_TRACE("Local Identity Key not available");
            }
            break;

        case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
            {
                wiced_bt_device_link_keys_t *p_link_keys = &p_event_data->paired_device_link_keys_update;
                APP_BT_TRACE("BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT   BdAddr:%B", p_link_keys->bd_addr );
                host_set_link_key(p_link_keys->bd_addr, p_link_keys, link_transport());
                link_set_bonded(TRUE); // Now we are bonded
            }
            break;

        case  BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
            APP_BT_TRACE("BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT");
            if (host_get_link_key(p_event_data->paired_device_link_keys_request.bd_addr, &p_event_data->paired_device_link_keys_request))
            {
                link_set_bonded(TRUE); // We have the link key, it is a bonded device
            }
            else
            {
                WICED_BT_TRACE("requsted %B link_key not available", p_event_data->paired_device_link_keys_request.bd_addr);
                result = WICED_BT_ERROR;
            }
            break;

        case BTM_ENCRYPTION_STATUS_EVT:
            APP_BT_TRACE("BTM_ENCRYPTION_STATUS_EVT, result=%d (%sencrypted)", p_event_data->encryption_status.result, p_event_data->encryption_status.result == WICED_SUCCESS? "": "not ");
            link_set_encrypted(p_event_data->encryption_status.result == WICED_SUCCESS);
            break;

        case BTM_PAIRING_COMPLETE_EVT:
            APP_BT_TRACE("BTM_PAIRING_COMPLETE_EVT: %d ", p_event_data->pairing_complete.pairing_complete_info.ble.reason);
            hci_control_send_pairing_complete_evt( p_event_data->pairing_complete.pairing_complete_info.ble.reason, p_event_data->pairing_complete.pairing_complete_info.ble.resolved_bd_addr, BT_DEVICE_TYPE_BLE );
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
            APP_BT_TRACE("BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT");
            p_event_data->pairing_io_capabilities_ble_request.local_io_cap = BTM_IO_CAPABILITIES_NONE;
            p_event_data->pairing_io_capabilities_ble_request.oob_data = BTM_OOB_NONE;
            p_event_data->pairing_io_capabilities_ble_request.auth_req = BTM_LE_AUTH_REQ_SC_BOND;              /* LE sec bonding */
            p_event_data->pairing_io_capabilities_ble_request.max_key_size = 16;
            p_event_data->pairing_io_capabilities_ble_request.init_keys = (BTM_LE_KEY_PENC|BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_PLK);
            p_event_data->pairing_io_capabilities_ble_request.resp_keys = (BTM_LE_KEY_PENC|BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_PLK);
            break;

        case BTM_SECURITY_REQUEST_EVT:
            APP_BT_TRACE("BTM_SECURITY_REQUEST_EVT");
            if (host_transport() == BT_TRANSPORT_LE)
            {
                APP_BT_TRACE("Clear CCCD's");
                host_set_cccd_flags(p_event_data->security_request.bd_addr, 0, 0);
            }
             /* Use the default security */
            wiced_bt_ble_security_grant(p_event_data->security_request.bd_addr,  WICED_BT_SUCCESS);
            break;

        case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
            APP_BT_TRACE("BTM_BLE_ADVERT_STATE_CHANGED_EVT");
            {
                wiced_bt_ble_advert_mode_t      new_adv_mode = p_event_data->ble_advert_state_changed;

                if (new_adv_mode == BTM_BLE_ADVERT_OFF && !link_is_connected())
                {
                    // if the adv is off and previous state was BTM_BLE_ADVERT_DIRECTED_HIGH, we switch to BTM_BLE_ADVERT_DIRECTED_LOW
                    if (bt.intended_adv_mode == BTM_BLE_ADVERT_DIRECTED_HIGH)
                    {
                        // start high duty cycle directed advertising.
                        if (bt_start_advertisements(BTM_BLE_ADVERT_DIRECTED_LOW, host_addr_type(), bt.adv_bdAddr))
                        {
                            WICED_BT_TRACE("Failed to start low duty cycle directed advertising!!!");
                        }
                        break;
                    }
                    // if the adv is off and previous state was BTM_BLE_ADVERT_UNDIRECTED_HIGH, we switch to BTM_BLE_ADVERT_UNDIRECTED_LOW
                    else if (bt.intended_adv_mode == BTM_BLE_ADVERT_UNDIRECTED_HIGH)
                    {
                        // start high duty cycle directed advertising.
                        if (bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_LOW, 0, NULL))
                        {
                            WICED_BT_TRACE("Failed to start low duty cycle undirected advertising!!!");
                        }
                        break;
                    }
                }
                app_adv_state_changed(bt.adv_mode, new_adv_mode);
                bt.adv_mode = new_adv_mode;
            }
            break;

        case BTM_BLE_SCAN_STATE_CHANGED_EVT:
            APP_BT_TRACE("Scan State Change: %d", p_event_data->ble_scan_state_changed );
            break;

        case BTM_BLE_CONNECTION_PARAM_UPDATE:
            APP_BT_TRACE("BTM_BLE_CONNECTION_PARAM_UPDATE status:%d interval:%d latency:%d timeout:%d",
                        p_event_data->ble_connection_param_update.status,
                        p_event_data->ble_connection_param_update.conn_interval,
                        p_event_data->ble_connection_param_update.conn_latency,
                        p_event_data->ble_connection_param_update.supervision_timeout);
            if (!p_event_data->ble_connection_param_update.status)
            {
                link_get_acl_conn_param()->conn_interval = p_event_data->ble_connection_param_update.conn_interval,
                link_get_acl_conn_param()->conn_latency = p_event_data->ble_connection_param_update.conn_latency,
                link_get_acl_conn_param()->supervision_timeout = p_event_data->ble_connection_param_update.supervision_timeout;
                link_set_parameter_updated(TRUE);
            }
            break;

		case BTM_BLE_PHY_UPDATE_EVT:
			APP_BT_TRACE("PHY update: status:%d Tx:%d Rx:%d BDA %B", p_event_data->ble_phy_update_event.status,
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
    APP_BT_TRACE("Enter reconnect");

    if (host_is_paired())
    {
        //NOTE!!! wiced_bt_start_advertisement could modify the value of bdAddr, so MUST use a copy.
        memcpy(bt.adv_bdAddr, host_addr(), BD_ADDR_LEN);

        // start high duty cycle directed advertising.
        if (bt_start_advertisements(BTM_BLE_ADVERT_DIRECTED_HIGH, host_addr_type(), bt.adv_bdAddr))
        {
            WICED_BT_TRACE("Failed to start high duty cycle directed advertising!!!");
        }
    }
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
    host_is_paired() ? bt_enter_reconnect() : bt_enter_pairing();
}

/********************************************************************
 * Function Name: bt_start_advertisements()
 ********************************************************************
 * Summary:
 *  Saves the current adv mode and calls wiced_bt_start_advertisements()
 *******************************************************************/
wiced_result_t bt_start_advertisements(wiced_bt_ble_advert_mode_t advert_mode, wiced_bt_ble_address_type_t directed_advertisement_bdaddr_type, wiced_bt_device_address_ptr_t directed_advertisement_bdaddr_ptr)
{
    bt.intended_adv_mode = advert_mode;
    return wiced_bt_start_advertisements( advert_mode, directed_advertisement_bdaddr_type, directed_advertisement_bdaddr_ptr);
}

/* end of file */
