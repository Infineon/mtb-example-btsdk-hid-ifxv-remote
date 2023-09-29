/*
 * $ Copyright YEAR Cypress Semiconductor $
 */

/**
 * file app.c
 *
 * This is the HID over ISOC demo application for HID Device. This application should be used together with
 * isoc_hidh, HID Host, for the demo.
 *
 */
#include "wiced_bt_types.h"
#include "wiced_bt_dev.h"
#include "wiced_memory.h"
#include "app.h"
#include "cycfg_pins.h"
#include "hci_control_api.h"
#include "nvram.h"

#if CHIP!=20829 && defined WICED_BT_TRACE_ENABLE
# undef WICED_BT_TRACE
# define WICED_BT_TRACE(format,...)   wiced_printf(NULL, 0, (format"\n"), ##__VA_ARGS__)
#endif

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
    // Std output report
    {
        .rpt_id           =RPT_ID_OUT_KB_LED,
        .rpt_type         =WICED_HID_REPORT_TYPE_OUTPUT,
        .handle_cccd      =0, // no cccd for output
        .handle_val       =HDLC_HIDS_OUT_RPT_KB_LED_VALUE,
    },
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
        WICED_BT_TRACE("roll over key");
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

        // case HCI_CONTROL_HIDD_KEY_POWER:
        // case HCI_CONTROL_HIDD_KEY_IR:
        // case HCI_CONTROL_HIDD_KEY_MOTION:
        default:
            WICED_BT_TRACE("Function not supported");
            return;
    }

    app_key_detected(keyCode, keyDown);
}
#endif

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
    if (link_is_connected())
    {
        WICED_BT_TRACE( "Remove bonding, disconnecting link" );
        bt_disconnect();
    }

    while (host_is_paired())
    {
        uint8_t *bonded_bdadr = host_addr();

#ifdef FILTER_ACCEPT_LIST_FOR_ADVERTISING
        //stop advertising anyway before Filter Accept List operation
        bt_stop_advertisement();

        //remove from Filter Accept List
        WICED_BT_TRACE( "remove from Filter Accept List : %B", bonded_bdadr );
        wiced_bt_ble_update_advertising_filter_accept_list( WICED_FALSE, bonded_bdadr );

        //clear whitelist
        wiced_bt_ble_clear_filter_accept_list();
#endif

        WICED_BT_TRACE( "remove bonded device : %B", bonded_bdadr );
        wiced_bt_dev_delete_bonded_device( bonded_bdadr );

        host_remove();
    }
}

#ifdef OTA_FIRMWARE_UPGRADE
#include "wiced_bt_ota_firmware_upgrade.h"
static ota_fw_upgrade_status_callback_t ota_fw_upgrade_status_callback = NULL;
static uint8_t  ota_fw_upgrade_initialized = WICED_FALSE;
#ifdef OTA_SECURE_FIRMWARE_UPGRADE
 #include "bt_types.h"
 #include "p_256_multprecision.h"
 #include "p_256_ecc_pp.h"

 // If secure version of the OTA firmware upgrade is used, the app should be linked with the ecdsa256_pub.c
 // which exports the public key
 extern Point    ecdsa256_public_key;
#endif // OTA_SECURE_FIRMWARE_UPGRADE
/*
 *
 */
void register_ota_fw_upgrade_status_callback(ota_fw_upgrade_status_callback_t cb)
{
    ota_fw_upgrade_status_callback = cb;
}

/*
 * Process write request or command from peer device
 */
void ota_fw_upgrade_status(uint8_t status)
{
    if (ota_fw_upgrade_status_callback)
    {
        ota_fw_upgrade_status_callback(status);
    }
}
#endif // OTA_FIRMWARE_UPGRADE

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

    // Check if the write request is for audio handle
    if (is_audio_handle(handle))
    {
        result = audio_gatt_write_handler(conn_id, p_wr_data);
    }
    else
#ifdef OTA_FIRMWARE_UPGRADE
    // if write request is for the OTA FW upgrade service, pass it to the library to process
    if (wiced_ota_fw_upgrade_is_gatt_handle(p_wr_data->handle))
    {
        WICED_BT_TRACE("\nOTA hidd_gatt_write_handler %04x", p_wr_data->handle );
        if (!ota_fw_upgrade_initialized)
        {
            WICED_BT_TRACE("\nOTA upgrade Init");
            /* OTA Firmware upgrade Initialization */
 #ifdef OTA_SECURE_FIRMWARE_UPGRADE
            if (wiced_ota_fw_upgrade_init(&ecdsa256_public_key, ota_fw_upgrade_status, NULL) == WICED_FALSE)
 #else
            if (wiced_ota_fw_upgrade_init(NULL, ota_fw_upgrade_status, NULL) == WICED_FALSE)
 #endif
            {
                WICED_BT_TRACE("\nOTA upgrade Init failure!!!");
                return WICED_BT_GATT_ERR_UNLIKELY;
            }
            ota_fw_upgrade_initialized = WICED_TRUE;
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
    WICED_BT_TRACE("Link up, conn_id:%04x peer_addr:%B type:%d", p_status->conn_id, p_status->bd_addr, p_status->addr_type);

#ifdef OTA_FIRMWARE_UPGRADE
    // Pass connection up event to the OTA FW upgrade library
    wiced_ota_fw_upgrade_connection_status_event(p_status);
#endif

    link_set_acl_conn_interval(wiced_bt_cfg_settings.ble_scan_cfg.conn_min_interval,
                               wiced_bt_cfg_settings.ble_scan_cfg.conn_max_interval,
                               wiced_bt_cfg_settings.ble_scan_cfg.conn_latency,
                               wiced_bt_cfg_settings.ble_scan_cfg.conn_supervision_timeout);
    led_on(LINK_LED);
    hidd_set_conn_id(p_status->conn_id);
    hci_control_send_connect_evt( p_status->addr_type, p_status->bd_addr, p_status->conn_id, p_status->link_role );
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

    WICED_BT_TRACE("Link down, id:0x%04x reason: %d (0x%02x)",  p_status->conn_id, p_status->reason, p_status->reason);

    hidd_set_conn_id(conn_id);
    hci_control_send_disconnect_evt( p_status->reason, p_status->conn_id );

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
            WICED_BT_TRACE("Advertisement %d started", adv);
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

#if ( defined(CYW20706A2) || defined(CYW20719B1) || defined(CYW20721B1) || defined(CYW43012C0) )
    /* Initialize wiced app */
    wiced_bt_app_init();
#endif

    hci_control_register_key_handler(app_hci_key_event);

    gatt_initialize();
    hidd_cfg.gatt_lookup_table_size = app_gatt_db_ext_attr_tbl_size;
    hidd_init(&hidd_cfg);

    /* Initialize each submodule */
    button_init();
    bat_init(app_shutdown);
    audio_init();
    key_init(NUM_KEYSCAN_ROWS, NUM_KEYSCAN_COLS, app_key_detected);
    hci_control_set_capability(audio_capability(), 0, 0);

#ifdef WICED_EVAL
    if (button_down())
    {
        WICED_BT_TRACE( "User button pressed during start up" );
        if (host_is_paired())
        {
            app_remove_host_bonding();
        }
        WICED_BT_TRACE( "Enter pairing" );
    }
#endif
    bt_enter_pairing();

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
    led_init(led_count, platform_led);

    bt_init();                                // WICED_BT_TRACE starts to work after bt_init()
    nvram_init(NULL);                         // use default default NVRAM read/write
    hci_control_init();

    WICED_BT_TRACE("\n<< %s >>", app_gap_device_name);
    WICED_BT_TRACE("DEV=%d btstack:%d.%d.%d", CHIP, WICED_BTSTACK_VERSION_MAJOR, WICED_BTSTACK_VERSION_MINOR, WICED_BTSTACK_VERSION_PATCH);
#ifdef SUPPORT_DIGITAL_MIC
    WICED_BT_TRACE("MIC=Digital");
#else
    WICED_BT_TRACE("MIC=Analog");
#endif
    WICED_BT_TRACE("SLEEP_ALLOWED=%d",SLEEP_ALLOWED);
#if BT_TRACE
    WICED_BT_TRACE("BT_TRACE=%d", BT_TRACE);
#endif
#if GATT_TRACE
    WICED_BT_TRACE("GATT_TRACE=%d", GATT_TRACE);
#endif
#if HIDD_TRACE
    WICED_BT_TRACE("HIDD_TRACE=%d", HIDD_TRACE);
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
}

/* end of file */
