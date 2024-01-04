/*
 * $ Copyright YEAR Cypress Semiconductor $
 */

/**
 * file app.c
 *
 * This is the application for IFXV-Device. This application should be used tested with IFX-Voice supported host.
 *
 */
#include "wiced_bt_types.h"
#include "wiced_bt_dev.h"
#include "wiced_hal_mia.h"
#include "wiced_memory.h"
#include "wiced_sleep.h"
#include "app.h"
#include "cycfg_pins.h"
#include "hci_control_api.h"

#if CHIP!=20829 && defined WICED_BT_TRACE_ENABLE
# undef WICED_BT_TRACE
# define WICED_BT_TRACE(format,...)   wiced_printf(NULL, 0, (format"\n"), ##__VA_ARGS__)
#endif

#define AUTO_RECONNECT_DELAY_WHEN_BOOT       5000
#define AUTO_RECONNECT_DELAY_WHEN_DISCONNECT 500
#ifdef ALLOW_SDS_IN_DISCOVERABLE
 //For cold boot, give it more time before allowing SDS; otherwise, it might reset.
 #define SDS_ALLOWED_IN_MS_WHEN_COLD_BOOT    5000
#else
 #define SDS_ALLOWED_IN_MS_WHEN_COLD_BOOT    2000
#endif
#define SDS_ALLOWED_IN_MS_WHEN_WARM_BOOT     2000

#include "wiced_bt_uuid.h"

/**********************************************************************************************
 * Report Table.
 **********************************************************************************************/
static hidd_rpt_t rpt_map[] =
{
    // STD keyboard Input report
    {
        .rpt_id           =RPT_ID_IN_STD_KEY,
        .rpt_type         =WICED_HID_REPORT_TYPE_INPUT,
        .handle_cccd      =HDLD_HIDS_IN_RPT_KB_CLIENT_CHAR_CONFIG,
        .handle_val       =HDLC_HIDS_IN_RPT_KB_VALUE,
    },
    //Bitmapped input report
    {
        .rpt_id           =RPT_ID_IN_BIT_MAPPED,
        .rpt_type         =WICED_HID_REPORT_TYPE_INPUT,
        .handle_cccd      =HDLD_HIDS_IN_RPT_CONSUMER_BITMAP_CLIENT_CHAR_CONFIG,
        .handle_val       =HDLC_HIDS_IN_RPT_CONSUMER_BITMAP_VALUE,
    },
    //Battery input report
    {
        .rpt_id           =RPT_ID_IN_BATTERY,
        .rpt_type         =WICED_HID_REPORT_TYPE_INPUT,
        .handle_cccd      =HDLD_BAS_BATTERY_LEVEL_CLIENT_CHAR_CONFIG,
        .handle_val       =HDLC_BAS_BATTERY_LEVEL_VALUE,
    },
    // Std output report
    {
        .rpt_id           =RPT_ID_OUT_KB_LED,
        .rpt_type         =WICED_HID_REPORT_TYPE_OUTPUT,
        .handle_cccd      =0, // no cccd for output
        .handle_val       =HDLC_HIDS_OUT_RPT_KB_LED_VALUE,
    },
#ifdef HDLS_ATVS
    // Audio data
    {
        .rpt_id           =RPT_ID_IN_AUDIO_DATA,
        .rpt_type         =WICED_HID_REPORT_TYPE_INPUT,
        .handle_cccd      =HDLD_ATVS_ATV_READ_CHAR_CCCD,
        .handle_val       =HDLC_ATVS_ATV_READ_CHAR_VALUE,
    },
    // Audio Ctl
    {
        .rpt_id           =RPT_ID_IN_AUDIO_CTL,
        .rpt_type         =WICED_HID_REPORT_TYPE_INPUT,
        .handle_cccd      =HDLD_ATVS_ATV_CONTROL_CHAR_CCCD,
        .handle_val       =HDLC_ATVS_ATV_CONTROL_CHAR_VALUE,
    },
#endif
#if defined(HDLS_IFXVS)
    // Audio data
    {
        .rpt_id           =RPT_ID_IN_AUDIO_DATA,
        .rpt_type         =WICED_HID_REPORT_TYPE_INPUT,
        .handle_cccd      =HDLD_IFXVS_IFXVS_DATA_CCCD,
        .handle_val       =HDLC_IFXVS_IFXVS_DATA_VALUE,
    },
    // Audio Ctl
    {
        .rpt_id           =RPT_ID_IN_AUDIO_CTL,
        .rpt_type         =WICED_HID_REPORT_TYPE_INPUT,
        .handle_cccd      =HDLD_IFXVS_IFXVS_DEVICE_CCCD,
        .handle_val       =HDLC_IFXVS_IFXVS_DEVICE_VALUE,
    },
#endif
};

#define RPT_SIZE (sizeof(rpt_map) / sizeof(hidd_rpt_t))

/******************************************************************************
 *     Pritvate Functions
 ******************************************************************************/

/********************************************************************
 * Function Name: app_hidd_rpt()
 ********************************************************************
 * Summary:
 *  When device receives hid report from host, this function is called.
 *
 * Parameters:
 *  uint8_t rpt_id       // report id
 *  uint8_t rpt_type     // report type
 *  uint8_t * p_data     // data pointer
 *  uint16_t len         // data length
 *
 * Return:
 *  none
 *
 *******************************************************************/
static wiced_bt_gatt_status_t app_hidd_rpt( uint8_t rpt_id, uint8_t type, uint16_t conn_id, wiced_bt_gatt_write_req_t * p_wr_data )
{
    // let the default handler to write data
    wiced_bt_gatt_status_t status = gatt_write_default_handler( conn_id, p_wr_data );

    switch (type) {
    case HID_PAR_REP_TYPE_OUTPUT:
        /* Example of this usage could be to turn on/off LED or to set the mode */
        break;

    case HID_PAR_REP_TYPE_FEATURE:
        /* HID feature report is not supported in this app */
        break;
    }

    return status;
}

/********************************************************************
 * Function Name: app_cccd_flags_changed()
 ********************************************************************
 * Summary: This function is called when CCCD flags have been changed.
 *
 * Parameters:
 *  uint16_t notif_flags    -- Notification flags.
 *  uint16_t indi_flags     -- Indication Flags
 *
 * Return:
 *  none
 *
 *******************************************************************/
static void app_cccd_flags_changed()
{
    uint16_t nflags, iflags, host_nflags, host_iflags;
    host_get_cccd_flags( host_addr(), &host_nflags, &host_iflags );
    hidd_get_cccd_flags( &nflags, &iflags );

    // if CCCD flag is changed, update host pairing info
    if (( nflags != host_nflags ) || ( iflags != host_iflags ))
    {
        host_set_cccd_flags( host_addr(), nflags, iflags );
    }

    WICED_BT_TRACE( "CCCD flags: %04x", nflags);
}

/******************************************************************************
 *  hidd configuration
 ******************************************************************************/
static hidd_cfg_t hidd_cfg =
{
    .rpt_table_size         = RPT_SIZE,
    .rpt_table              = rpt_map,
    .rpt_cb                 = app_hidd_rpt,
    .gatt_lookup_table      = app_gatt_db_ext_attr_tbl,
//    .gatt_lookup_table_size = app_gatt_db_ext_attr_tbl_size,     // need to assign at run time.
};

/////////////////////////////////////////////////////////////////////////////////
/// This is a callback function from keyscan when key action is detected
/////////////////////////////////////////////////////////////////////////////////
static void app_key_detected(uint8_t keyCode, uint8_t keyDown)
{
    switch (keyCode) {
    case END_OF_SCAN_CYCLE:
        break;
    case ROLLOVER:
        WICED_BT_TRACE("ROLLOVER key");
        break;
    default:
        WICED_BT_TRACE("kc:%d %c", keyCode, keyDown ? 'D':'U');
        key_process_event(keyCode, keyDown);
    }
}

#ifdef TESTING_USING_HCI

/////////////////////////////////////////////////////////////////////////////////
/// Translate HCI control keyCode to this system's keyCode before processing the key
/////////////////////////////////////////////////////////////////////////////////
static void app_hci_key_event(uint8_t keyCode, wiced_bool_t keyDown)
{
    // Translate to app defined key
    switch (keyCode)
    {
        case HCI_CONTROL_HIDD_KEY_CONNECT:
            if (keyDown)
            {
                bt_enter_connect();
            }
            return;

        case HCI_CONTROL_HIDD_KEY_AUDIO:
            keyCode = AUDIO_KEY_INDEX;
            break;

        case HCI_CONTROL_HIDD_KEY_HOME:
            keyCode = HOME_KEY_INDEX;        // Home key, hold for 10s to enter pairing
            break;

        case HCI_CONTROL_HIDD_KEY_MUTE:
            keyCode = MUTE_KEY_INDEX;
            break;

        case HCI_CONTROL_HIDD_KEY_BACK:
            keyCode = BACK_KEY_INDEX;
            break;

#ifdef SUPPORT_IR
        case HCI_CONTROL_HIDD_KEY_IR:
            keyCode = IR_KEY_INDEX;
            break;
#endif
        // case HCI_CONTROL_HIDD_KEY_POWER:
        // case HCI_CONTROL_HIDD_KEY_MOTION:
        default:
            WICED_BT_TRACE("Function not supported");
            return;
    }

    app_key_detected(keyCode, keyDown);
}
#endif

////////////////////////////////////////////////////////////////////////////////
/// Function Name: APP_sleep_handler
////////////////////////////////////////////////////////////////////////////////
/// Summary:
///    Sleep permit query to check if sleep is allowed and sleep time
///
/// Parameters:
///  type -- query type. It can be WICED_SLEEP_POLL_TIME_TO_SLEEP or WICED_SLEEP_POLL_SLEEP_PERMISSION
///
/// Return:
///  WICED_SLEEP_NOT_ALLOWED when not allow to sleep.
///
///  If the query type is WICED_SLEEP_POLL_TIME_TO_SLEEP,
///     it returns WICED_SLEEP_MAX_TIME_TO_SLEEP or the time to sleep
///
///  otherwise, the query type should be WICED_SLEEP_POLL_SLEEP_PERMISSION.
///     It returns WICED_SLEEP_ALLOWED_WITHOUT_SHUTDOWN when allowed to sleep, but no SDS nor ePDS.
///     otherwise, it returns WICED_SLEEP_ALLOWED_WITH_SHUTDOWN when allowed to enter SDS/ePDS
///
////////////////////////////////////////////////////////////////////////////////
static uint32_t app_sleep_handler(wiced_sleep_poll_type_t type )
{
    uint32_t ret = WICED_SLEEP_NOT_ALLOWED;

#if SLEEP_ALLOWED
    switch(type)
    {
        case WICED_SLEEP_POLL_TIME_TO_SLEEP:
            ret = WICED_SLEEP_MAX_TIME_TO_SLEEP;
 #if SFI_DEEP_SLEEP
            // In 20835, sfi CS may contains glitch that wakes up Flash result in high current. Apply workaround to put sfi into powerdown
            if (pmu_attemptSleepState == 5 )
            {
                nvram_exit_deep_sleep(FALSE);
                nvram_enter_deep_sleep();
            }
 #endif
            break;

        case WICED_SLEEP_POLL_SLEEP_PERMISSION:
            ret = WICED_SLEEP_ALLOWED_WITHOUT_SHUTDOWN;
 #if SLEEP_ALLOWED > 1
            if (!wiced_hidd_is_transport_detection_polling_on() && sds_is_allowed()

                // If LE connection is established and expected link parameter update,
                // we don't deep sleep until the params update successfully.
                && (!link_is_connected() || link_params_update_is_expected())

                // Not advertising
                && (bt_get_advertising_mode()==BTM_BLE_ADVERT_OFF)

                // a key is not down
                && !key_active()

                // audio is not active
                && !audio_is_active()

                // IR is not active
                && !ir_is_active()

                // Findme is not active
                && !findme_is_active()

                // OTA firmware upgrade is not active
                && !ota_is_active()
               )
            {
                sds_save_data_to_aon();
                ret = WICED_SLEEP_ALLOWED_WITH_SHUTDOWN;
            }
 #endif
            break;
    }
#endif
    return ret;
}

/*******************************************************************************
 * sleep configuration
 *******************************************************************************/
wiced_sleep_config_t    sleep_config = {
    .sleep_mode             = WICED_SLEEP_MODE_NO_TRANSPORT,  //sleep_mode
    .host_wake_mode         = 0,                              //host_wake_mode
    .device_wake_mode       = 0,                              //device_wake_mode
    .device_wake_source     = WICED_SLEEP_WAKE_SOURCE_GPIO | WICED_SLEEP_WAKE_SOURCE_KEYSCAN | WICED_SLEEP_WAKE_SOURCE_QUAD,  //device_wake_source
    .device_wake_gpio_num   = 255,                            //must set device_wake_gpio_num to 255 for WICED_SLEEP_MODE_NO_TRANSPORT
    .sleep_permit_handler   = app_sleep_handler,              //sleep_permit_handler
#if defined(CYW20819A1) || defined(CYW20820A1)
    .post_sleep_cback_handler=NULL,                           //post_sleep_handler
#endif
};

/////////////////////////////////////////////////////////////////////////////////
/// app_cold_boot
/////////////////////////////////////////////////////////////////////////////////
static void app_cold_boot()
{
#ifdef WICED_EVAL
    if (button_down())
    {
        WICED_BT_TRACE( "User button pressed during start up -> removing pairing info" );
        if (host_is_paired())
        {
            app_remove_host_bonding();
        }
    }
#endif

    bt_disconnect();
    if (host_is_paired())
    {
        WICED_BT_TRACE("bonded info in NVRAM");
#ifdef START_ADV_WHEN_POWERUP_NO_CONNECTED
        bt_enter_reconnect();
#else
 #ifdef AUTO_RECONNECT
        bt_delayed_reconnect(AUTO_RECONNECT_DELAY_WHEN_BOOT);
 #endif
#endif
    }
    else
    {

#ifdef START_ADV_WHEN_POWERUP_NO_CONNECTED
        bt_enter_pairing();
#else
        button_check_boot_action();
#endif
    }
}

/////////////////////////////////////////////////////////////////////////////////
/// app_cold_boot
/////////////////////////////////////////////////////////////////////////////////
static void app_init_action()
{
    if(wiced_hal_mia_is_reset_reason_por())
    {
        sds_allowed_in_ms(SDS_ALLOWED_IN_MS_WHEN_COLD_BOOT);
        WICED_BT_TRACE("cold boot");
        app_cold_boot();
    }
    else
    {
        sds_allowed_in_ms(SDS_ALLOWED_IN_MS_WHEN_WARM_BOOT);
        WICED_BT_TRACE("warm boot");
        sds_wake();
        button_check_boot_action();
    }
}

/******************************************************************************
 *     Public Functions
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
wiced_bool_t app_check_cccd_flags( uint16_t handle )
{
    // Check if the handle is for CCCD
    for (int i = 0; i < RPT_SIZE; i++)
    {
        if (handle == rpt_map[i].handle_cccd)
        {
            app_cccd_flags_changed();
            return TRUE;
        }
    }
    return FALSE;
}

/********************************************************************
 * Function Name: app_remove_host_bonding
 ********************************************************************
 * Summary:
 *  Virtual cable unplug.
 *  This function will remove all HID host information from NVRAM.
 ********************************************************************/
void app_remove_host_bonding(void)
{
    // Stop adv if it is advertising
    bt_stop_advertisement();

    if (link_is_connected())
    {
        WICED_BT_TRACE( "Remove bonding, disconnecting link" );
        bt_disconnect();
    }

    while (host_is_paired())
    {
        uint8_t *bonded_bdadr = host_addr();

        WICED_BT_TRACE( "remove bonded device : %B", bonded_bdadr );
        wiced_bt_dev_delete_bonded_device( bonded_bdadr );

        host_remove();
    }
    hci_control_send_paired_host_info();
}

/********************************************************************
 * Function Name: app_gatt_write_handler
 ********************************************************************
 * Summary:
 *  This function is called when GATT handle write req event is recieved.
 *******************************************************************/
wiced_bt_gatt_status_t app_gatt_write_handler( uint16_t conn_id, wiced_bt_gatt_write_req_t * p_wr_data )
{
    uint16_t handle = p_wr_data->handle;
    wiced_bt_gatt_status_t result = WICED_BT_GATT_ATTRIBUTE_NOT_FOUND;

#ifdef SUPPORT_AUDIO
    // Check if the write request is for audio handle
    if (is_audio_handle(handle))
    {
        result = audio_gatt_write_handler(conn_id, p_wr_data);
    }
    else
#endif
#ifdef SUPPORT_FINDME
    if ( (result = findme_gatts_req_write_handler(conn_id, p_wr_data)) != WICED_BT_GATT_NOT_FOUND )
    {
        return result;
    }
    else
#endif
#ifdef OTA_FIRMWARE_UPGRADE
    // if write request is for the OTA FW upgrade service, pass it to the library to process
    if (wiced_ota_fw_upgrade_is_gatt_handle(p_wr_data->handle))
    {
        if (!ota_fw_upgrade_initialized)
        {
            WICED_BT_TRACE("OTA upgrade Init");

            /* OTA Firmware upgrade Initialization */
            if (wiced_ota_fw_upgrade_init(ECDSA256_PUBLIC_KEY, NULL, NULL) == WICED_FALSE)
            {
                WICED_BT_TRACE("OTA upgrade Init failure!!!");
                return WICED_BT_GATT_ERR_UNLIKELY;
            }
            ota_fw_upgrade_initialized = WICED_TRUE;
        }

        if (HANDLE_OTA_FW_UPGRADE_DATA == p_wr_data->handle)
        {
            WICED_BT_TRACE("OTA data: off:%d len:%d", p_wr_data->offset, p_wr_data->val_len);
        }
        else
        {
            WICED_BT_TRACE("OTA GATT write: hdl:0x%04x off:%d len:%d data:%A", p_wr_data->handle, p_wr_data->offset, p_wr_data->val_len, p_wr_data->p_val, p_wr_data->val_len);
        }
        return wiced_ota_fw_upgrade_write_handler(conn_id, p_wr_data);
    }
    else
#endif
    /* Check if this is HIDD GATT */
    if ( is_hidd_handle(handle)  || is_bas_handle(handle))
    {
        // Check if the write request is for the audio command
        result = hidd_gatt_write_handler( conn_id, p_wr_data );
    }
    if (result == WICED_BT_GATT_ATTRIBUTE_NOT_FOUND)
    {
        // let the default handler to take care of it
        result = gatt_write_default_handler( conn_id, p_wr_data);
    }

    return result;
}

/********************************************************************
 * Function Name: app_link_up
 ********************************************************************
 * Summary:
 *  This function is called when link is up
 *******************************************************************/
void app_link_up(wiced_bt_gatt_connection_status_t * p_status)
{
    //configure ATT MTU size with peer device
    WICED_BT_TRACE("Configure MTU with size %d",  MAX_MTU_SIZE);
    wiced_bt_gatt_configure_mtu(link_conn_id(), MAX_MTU_SIZE);

    // enable ghost detection
    kscan_enable_ghost_detection(TRUE);

    link_set_acl_conn_interval(wiced_bt_cfg_settings.ble_scan_cfg.conn_min_interval,
                               wiced_bt_cfg_settings.ble_scan_cfg.conn_max_interval,
                               wiced_bt_cfg_settings.ble_scan_cfg.conn_latency,
                               wiced_bt_cfg_settings.ble_scan_cfg.conn_supervision_timeout);
    led_on(LINK_LED);
    hidd_set_conn_id(p_status->conn_id);
    hci_control_send_connect_evt( p_status->addr_type, p_status->bd_addr, p_status->conn_id, p_status->link_role );

    // if link is not bonded, give it 10 sec for bonding before allowing SDS
    if (!link_is_bonded())
    {
        sds_allowed_in_ms(10000); // no SDS for 10 sec.
    }
}

/********************************************************************
 * Function Name: app_link_down
 ********************************************************************
 * Summary:
 *  This function is called when link is down
 *******************************************************************/
void app_link_down(wiced_bt_gatt_connection_status_t * p_status)
{
    uint16_t conn_id = link_conn_id();  // if we have multiple links, the link_conn_id() can return the secondary conn_id.

    hidd_set_conn_id(conn_id);
    hci_control_send_disconnect_evt( p_status->reason, p_status->conn_id );

    // disable Ghost detection
    kscan_enable_ghost_detection(FALSE);

    // if no more link, we turn off LED
    if (!conn_id)
    {
        led_off(LINK_LED);
    }
    else
    {
        p_status = link_connection_status();
        if (p_status)
        {
            WICED_BT_TRACE("conn_id:%04x peer_addr:%B type:%d now is active", p_status->conn_id, p_status->bd_addr, p_status->addr_type);
            hci_control_send_connect_evt( p_status->addr_type, p_status->bd_addr, p_status->conn_id, p_status->link_role );
        }
    }

#ifdef AUTO_RECONNECT
    bt_delayed_reconnect(AUTO_RECONNECT_DELAY_WHEN_DISCONNECT);
#endif
}

/********************************************************************
 * Function Name: app_adv_state_changed
 ********************************************************************
 * Summary:
 *  This function is called when advertisment state is changed
 *******************************************************************/
void app_adv_state_changed(wiced_bt_ble_advert_mode_t old_adv, wiced_bt_ble_advert_mode_t adv)
{
    if (adv == BTM_BLE_ADVERT_OFF)
    {
        WICED_BT_TRACE("Advertisement Stopped");
        led_blink_stop(LINK_LED);
    }
    else
    {
        if (old_adv == BTM_BLE_ADVERT_OFF)
        {
            wiced_bt_device_address_t  bda;
#ifdef LE_LOCAL_PRIVACY_SUPPORT
            memcpy(bda, wiced_btm_get_private_bda(), BD_ADDR_LEN);
            char * ad_type = "random private address";
#else
            wiced_bt_dev_read_local_addr(bda);
            char * ad_type = "public address";
#endif
            if (adv <= BTM_BLE_ADVERT_DIRECTED_LOW)
            {
                WICED_BT_TRACE("Direct adv %d started with %s %B to host %BaddrType:%d", adv, ad_type, bda, host_addr(), host_addr_type());
            }
            else
            {
                WICED_BT_TRACE("Undirect adv %d started with %s %B", adv, ad_type, bda);
            }
            led_blink(LINK_LED, 0, host_is_paired() ? 200: 500);     // use faster blink LINK line to indicate reconnecting
        }
        else
        {
            WICED_BT_TRACE("Advertisement State Change: %d -> %d", old_adv, adv);
        }
    }
    hci_control_send_advertisement_state_evt(adv);
}

/********************************************************************
 * Function Name: app_shutdown
 ********************************************************************
 * Summary:
 *  This function is called when battery level reaches shutdown voltage.
 *  The device should put power consumption to the lowest to prevent battery
 *  leakage before shutdown.
 *******************************************************************/
void app_shutdown(void)
{
    WICED_BT_TRACE("app_shutdown");

    if(link_is_connected())
    {
        WICED_BT_TRACE( "app_shutdown disconnect" );
        bt_disconnect();
    }

    hidd_enable_interrupt(FALSE); // Disable Interrupts
}

/********************************************************************
 * Function Name: app_init
 ********************************************************************
 * Summary:
 *  When BT Management Stack is initialized successfully, this function is called.
 *******************************************************************/
wiced_result_t app_init(void)
{
    WICED_BT_TRACE( "app_init" );

    sds_init();
    link_init();
    hci_control_register_key_handler(app_hci_key_event);

    gatt_initialize();
    hidd_cfg.gatt_lookup_table_size = app_gatt_db_ext_attr_tbl_size;

    wiced_hidd_app_init(BT_DEVICE_TYPE_BLE);
    /* Allow peer to pair */
    wiced_bt_set_pairable_mode(WICED_TRUE, 0);
    wiced_btm_ble_update_advertisement_filter_policy(0);

    wiced_sleep_configure( &sleep_config );

    /* Initialize each submodule */
    bat_init(app_shutdown);
    audio_init();
    key_init(NUM_KEYSCAN_ROWS, NUM_KEYSCAN_COLS, app_key_detected);
    button_init();
    findme_init();
    hidd_init(&hidd_cfg);

#ifdef SUPPORT_IR
    ir_init(IR_TX_GPIO);
    hci_control_set_capability(audio_capability(), 0, HCI_CONTROL_HIDD_IR_SUPPORT);
#else
    hci_control_set_capability(audio_capability(), 0, 0);
#endif
    hidd_enable_interrupt(TRUE);

    app_init_action();
    WICED_BT_TRACE("Free RAM bytes=%d bytes", wiced_memory_get_free_bytes());

    return WICED_BT_SUCCESS;
}

/*******************************************************************************
 * Function Name: application_start()
 ********************************************************************************
 *  Entry point to the application. Set device configuration and start Bluetooth
 *  stack initialization.  The actual application initialization (app_init) will
 *  be called when stack reports that Bluetooth device is ready.
 ******************************************************************************/
void application_start( void )
{
    // Initialize LED/UART for debug
    wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_PUART);
#ifdef ENABLE_BT_SPY_LOG
    WICED_BT_TRACE("Routing debug msg to WICED_UART, open ClientControl/BtSpy");
    // Initialize LED/UART for debug
    wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_WICED_UART);
#endif
    led_init(led_count, platform_led);

    bt_init();                                // WICED_BT_TRACE starts to work after bt_init() for 20829
    nvram_init(NULL);                         // use default default NVRAM read/write
    hci_control_init();

    WICED_BT_TRACE("\n<< %s >>", app_gap_device_name);
#ifdef TESTING_USING_HCI
    WICED_BT_TRACE("TESTING_USING_HCI");
#endif
    WICED_BT_TRACE("DEV=%d btstack:%d.%d.%d", CHIP, WICED_BTSTACK_VERSION_MAJOR, WICED_BTSTACK_VERSION_MINOR, WICED_BTSTACK_VERSION_PATCH);

#if defined(ALLOW_SDS_IN_DISCOVERABLE) || defined(ENDLESS_LE_ADVERTISING)
    ((wiced_bt_cfg_ble_advert_settings_t *) &wiced_bt_cfg_settings.ble_advert_cfg)->low_duty_duration = 0;
#endif

    WICED_BT_TRACE("SLEEP_ALLOWED=%d",SLEEP_ALLOWED);
    WICED_BT_TRACE("LED=%d",LED_SUPPORT);

#ifdef OTA_FIRMWARE_UPGRADE
    WICED_BT_TRACE("OTA_FW_UPGRADE");
 #ifdef OTA_SECURE_FIRMWARE_UPGRADE
    WICED_BT_TRACE("OTA_SEC_FW_UPGRADE");
 #endif
#endif
#ifdef HDLS_IFXVS
    WICED_BT_TRACE("IFX-Voice Profile");
#endif
#ifdef HDLS_ATVS
    WICED_BT_TRACE("Google-Voice Profile");
#endif
#ifdef MSBC_ENCODER
    WICED_BT_TRACE("Codec=MSBC");
#elif defined ADPCM_ENCODER
    WICED_BT_TRACE("Codec=ADPCM");
#elif defined OPUS_ENCODER
    WICED_BT_TRACE("Codec=OPUS");
#endif
#ifdef SUPPORT_DIGITAL_MIC
    WICED_BT_TRACE("MIC=Digital");
#else
    WICED_BT_TRACE("MIC=Analog");
#endif
#ifdef AUTO_RECONNECT
    WICED_BT_TRACE("AUTO_RECONNECT");
#endif
#ifdef SKIP_CONNECT_PARAM_UPDATE_EVEN_IF_NO_PREFERED
    WICED_BT_TRACE("SKIP_PARAM_UPDATE");
#endif
#ifdef START_ADV_WHEN_POWERUP_NO_CONNECTED
    WICED_BT_TRACE("START_ADV_ON_POWERUP");
#endif
#ifdef CONNECTED_ADVERTISING_SUPPORTED
    WICED_BT_TRACE("ENABLE_CONNECTED_ADV");
#endif
#ifdef ENDLESS_LE_ADVERTISING
    WICED_BT_TRACE("ENDLESS_ADV");
#endif
#ifdef LE_LOCAL_PRIVACY_SUPPORT
    WICED_BT_TRACE("LE_LOCAL_PRIVACY_SUPPORT");
#endif
#ifdef SUPPORT_IR
    WICED_BT_TRACE("ENABLE_IR");
#endif
#ifdef SUPPORT_FINDME
    WICED_BT_TRACE("ENABLE_FINDME");
#endif

#if BT_TRACE
    WICED_BT_TRACE("BT_TRACE=%d", BT_TRACE);
#endif
#if GATT_TRACE
    WICED_BT_TRACE("GATT_TRACE=%d", GATT_TRACE);
#endif
#if HIDD_TRACE
    WICED_BT_TRACE("HIDD_TRACE=%d", HIDD_TRACE);
#endif
#if SDS_TRACE
    WICED_BT_TRACE("SDS_TRACE=%d", SDS_TRACE);
#endif
#if HCI_TRACE
    WICED_BT_TRACE("HCI_TRACE=%d", HCI_TRACE);
#endif
#if LED_TRACE
    WICED_BT_TRACE("LED_TRACE=%d", LED_TRACE);
#endif
#if KEY_TRACE
    WICED_BT_TRACE("KEY_TRACE=%d", KEY_TRACE);
#endif
#if NVRAM_TRACE
    WICED_BT_TRACE("NVRAM_TRACE=%d", NVRAM_TRACE);
#endif
#if IR_TRACE
    WICED_BT_TRACE("IR_TRACE=%d", IR_TRACE);
#endif
#if FINDME_TRACE
    WICED_BT_TRACE("FINDME_TRACE=%d", FINDME_TRACE);
#endif
#if AUDIO_TRACE
    WICED_BT_TRACE("AUDIO_TRACE=%d", AUDIO_TRACE);
#endif
#if MIC_TRACE
    WICED_BT_TRACE("MIC_TRACE=%d", MIC_TRACE);
#endif
#if CODEC_TRACE
    WICED_BT_TRACE("CODEC_TRACE=%d", CODEC_TRACE);
#endif
#if PROTOCOL_TRACE
    WICED_BT_TRACE("PROTOCOL_TRACE=%d", PROTOCOL_TRACE);
#endif
#ifdef LE_LOCAL_PRIVACY_SUPPORT
    WICED_BT_TRACE("LE_LOCAL_PRIVACY_SUPPORT");
#endif

}

/* end of file */
