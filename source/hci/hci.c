/*
 * (c) 2016-2025, Infineon Technologies AG, or an affiliate of Infineon
 * Technologies AG. All rights reserved.
 * This software, associated documentation and materials ("Software") is
 * owned by Infineon Technologies AG or one of its affiliates ("Infineon")
 * and is protected by and subject to worldwide patent protection, worldwide
 * copyright laws, and international treaty provisions. Therefore, you may use
 * this Software only as provided in the license agreement accompanying the
 * software package from which you obtained this Software. If no license
 * agreement applies, then any use, reproduction, modification, translation, or
 * compilation of this Software is prohibited without the express written
 * permission of Infineon.
 *
 * Disclaimer: UNLESS OTHERWISE EXPRESSLY AGREED WITH INFINEON, THIS SOFTWARE
 * IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING, BUT NOT LIMITED TO, ALL WARRANTIES OF NON-INFRINGEMENT OF
 * THIRD-PARTY RIGHTS AND IMPLIED WARRANTIES SUCH AS WARRANTIES OF FITNESS FOR A
 * SPECIFIC USE/PURPOSE OR MERCHANTABILITY.
 * Infineon reserves the right to make changes to the Software without notice.
 * You are responsible for properly designing, programming, and testing the
 * functionality and safety of your intended application of the Software, as
 * well as complying with any legal requirements related to its use. Infineon
 * does not guarantee that the Software will be free from intrusion, data theft
 * or loss, or other breaches ("Security Breaches"), and Infineon shall have
 * no liability arising out of any Security Breaches. Unless otherwise
 * explicitly approved by Infineon, the Software may not be used in any
 * application where a failure of the Product or any consequences of the use
 * thereof can reasonably be expected to result in personal injury.
 */

/** @file
 *
 * HCI hidd handling routines
 *
 */
#include "wiced_transport.h"
#include "wiced_timer.h"
#include "hci_control_api.h"
#if defined(CYW43012C0)
 #include "wiced_hal_watchdog.h"
#else
 #include "wiced_hal_wdog.h"
#endif

#include "app.h"

#if CHIP!=20829 && defined WICED_BT_TRACE_ENABLE
# undef WICED_BT_TRACE
# define WICED_BT_TRACE(format,...)   wiced_printf(NULL, 0, (format"\n"), ##__VA_ARGS__)
#endif

#if HCI_TRACE
 #define APP_HCI_TRACE      WICED_BT_TRACE
 #if AUDIO_TRACE>1
  #define APP_HCI_TRACE2    WICED_BT_TRACE
 #else
  #define APP_HCI_TRACE2(...)
 #endif
#else
 #define APP_HCI_TRACE(...)
 #define APP_HCI_TRACE2(...)
#endif

#if defined(TESTING_USING_HCI)


////////////////////////////////////////////////////////////////////////////////
// defines
////////////////////////////////////////////////////////////////////////////////
#define READ_LITTLE_ENDIAN_TO_UINT16(into, m,dl) (into) = ((m)[0] | ((m)[1]<<8)); (m) +=2; (dl)-=2;
#define AUDIO_SAMPLE_DATA_BUFFER_SIZE 400    // store up to 400 samples

typedef struct
{
    // application key handler pointer
    hidd_app_hci_key_callback_t registered_app_key_handler;
    int16_t buffer[AUDIO_SAMPLE_DATA_BUFFER_SIZE];
    int16_t buffer_cnt;
    int16_t req_cnt;
    wiced_timer_t timer;
} hci_data_t;

////////////////////////////////////////////////////////////////////////////////
// var
////////////////////////////////////////////////////////////////////////////////
static hci_data_t hci = {0};
static uint8_t dev_capabilities[DEVICE_CAPABILITY_LEN]={0,0,0};

/********************************************************************
 * Function Name: host_commit_timer_cb
 ********************************************************************
 * Summary:
 *  This function is the timeout handler for commit_timer
 *******************************************************************/
static void hci_timer_cb( TIMER_PARAM_TYPE arg )
{
    if (audio_data_from_hci())
    {
        // We have room now, request more audio data from HCI
        hci_control_send_audio_data_req();
    }
}

/*
 *  send audio data request event
 *
 *  count -- sample count of 16-bit data
 */
static void send_audio_data_req()
{
    uint8_t   tx_buf[2];
    uint16_t  count = hci.req_cnt;

    if (count > MAX_SEND_HCI_SAMPLE_CNT)
    {
        count = MAX_SEND_HCI_SAMPLE_CNT;
    }

    if (count)
    {
        hci.req_cnt -= count;
        APP_HCI_TRACE2(" Requesting audio data %d words from HCI, pending req:%d", count, hci.req_cnt);

        // save count in little-ending format
        tx_buf[0] = count & 0xff;
        tx_buf[1] = count >> 8;
        hci_control_send_data( HCI_CONTROL_HIDD_EVENT_AUDIO_DATA_REQ, tx_buf, 2 );
    }
    else
    {
        APP_HCI_TRACE2(" Already received all reqeusted %d words, do nothing");
    }
}


static void handle_audio_data(int16_t * p_data, uint16_t sample_cnt)
{
    APP_HCI_TRACE2("  Received audio data %d words from HCI, buf_cnt:%d, pending req_cnt:%d", sample_cnt, hci.buffer_cnt, hci.req_cnt);
    while (sample_cnt)
    {
        uint16_t cnt = sample_cnt;

        if ((sample_cnt + hci.buffer_cnt) > audio_wdata_cnt())
        {
            cnt = audio_wdata_cnt() - hci.buffer_cnt;
        }

        // copy data to buffer
        memcpy(&hci.buffer[hci.buffer_cnt], p_data, cnt * 2);
        sample_cnt -= cnt;
        p_data += cnt;
        hci.buffer_cnt += cnt;

        APP_HCI_TRACE2("  sample_cnt:%d, buf_cnt:%d, req_cnt%d",sample_cnt, hci.buffer_cnt, hci.req_cnt);
        if (hci.buffer_cnt >= audio_wdata_cnt())
        {
            APP_HCI_TRACE2(" ** Received all requested data %d words from HCI", hci.buffer_cnt);
            audio_process_input_data(hci.buffer, hci.buffer_cnt);
            hci.buffer_cnt = 0;
        }
        else
        {
            send_audio_data_req();
        }
    }
}

/*
 *  Send advertise state change event to uart
 */
void hci_control_send_advertisement_state_evt( uint8_t state )
{
    hci_control_send_data( HCI_CONTROL_LE_EVENT_ADVERTISEMENT_STATE, &state, 1 );
}

/*
 *  Pass protocol traces up through the UART
 */
void hci_hci_trace_cback( wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t * p_data )
{
    //send the trace
    hidd_transport_send_hci_trace( type, p_data, length );
}

/*
 * handle reset command from UART
 */
void hci_hidd_dev_handle_reset_cmd( void )
{
    // trip watch dog now.
    wiced_hal_wdog_reset_system( );
}

/*
 * handle command from UART to configure traces
 */
void hci_control_handle_trace_enable( uint8_t *p_data )
{
    uint8_t hci_trace_enable = *p_data++;
    wiced_debug_uart_types_t route_debug = (wiced_debug_uart_types_t)*p_data;

    APP_HCI_TRACE("HCI Traces:%s DebugRoute:%d", hci_trace_enable?"Enable":"Disable", route_debug);

    if ( hci_trace_enable )
    {
        /* Register callback for receiving hci traces */
        // Disable while streaming audio over the uart.
        wiced_bt_dev_register_hci_trace( hci_hci_trace_cback );
    }
    else
    {
        wiced_bt_dev_register_hci_trace( NULL);
    }

// 20820/20819 has an issue for routing to WICED_UART, the BR/EDR pairing will fail. We don't change it.
#if !is_20819Family || !defined(BR_EDR_SUPPORT)
    // We don't handle HCI_UART/DBG_UART option, force it to WICED_UART/PUART
    switch (route_debug) {
    case WICED_ROUTE_DEBUG_TO_HCI_UART:
        route_debug = WICED_ROUTE_DEBUG_TO_WICED_UART;
        break;
    case WICED_ROUTE_DEBUG_TO_DBG_UART:
        route_debug = WICED_ROUTE_DEBUG_TO_PUART;
        break;
    default:
        break;
    }

    wiced_set_debug_uart( route_debug );
#endif
}

/*
 * handle command to send paired host device address
 */
void hci_control_send_paired_host_info()
{
    uint8_t   tx_buf[(BD_ADDR_LEN+1)*HIDD_HOST_LIST_MAX+1];
    uint8_t   len = host_get_info(tx_buf);
    APP_HCI_TRACE("Send host info, len=%d count = %d", len, tx_buf[0] & 0x7f); // mask out bit 7 of tx_buf[0] that contains link up/down status
    hci_control_send_data( HCI_CONTROL_HIDD_EVENT_HOST_ADDR, tx_buf, len );
}

/*
 * send device state change
 */
void hci_control_send_state_change(uint8_t transport, uint8_t state)
{
    uint8_t   tx_buf[3];

    tx_buf[0] = 0;  // reserved for host id
    tx_buf[1] = transport;
    tx_buf[2] = state & HID_LINK_MASK;
    APP_HCI_TRACE("Send state change %d", state);
    hci_control_send_data( HCI_CONTROL_HIDD_EVENT_STATE_CHANGE, tx_buf, 3 );
}

/*
 * handle command to send local Bluetooth device address
 */
void hci_hidd_dev_handle_set_local_bda( uint8_t *bda )
{
    uint8_t   tx_buf[15];
    uint8_t   cmd = 0;
    int result;

    BD_ADDR bd_addr;
    STREAM_TO_BDADDR (bd_addr, bda);
    wiced_bt_set_local_bdaddr( bd_addr, BLE_ADDR_PUBLIC );
    APP_HCI_TRACE("Set address: %B", bd_addr);
#ifdef BR_EDR_SUPPORT
    wiced_bt_l2cap_set_desire_role(HCI_ROLE_PERIPHERAL);
#endif
}

/* Handle get version command */
void hci_control_misc_handle_get_version( void )
{
    uint8_t   tx_buf[15];
    uint8_t   cmd = 0;
    uint32_t  chip = CHIP;

    tx_buf[cmd++] = WICED_SDK_MAJOR_VER;
    tx_buf[cmd++] = WICED_SDK_MINOR_VER;
    tx_buf[cmd++] = WICED_SDK_REV_NUMBER;
    tx_buf[cmd++] = WICED_SDK_BUILD_NUMBER & 0xFF;
    tx_buf[cmd++] = (WICED_SDK_BUILD_NUMBER>>8) & 0xFF;
    tx_buf[cmd++] = chip & 0xFF;
    tx_buf[cmd++] = (chip>>8) & 0xFF;
    tx_buf[cmd++] = (chip>>24) & 0xFF;
    tx_buf[cmd++] = 0;  // not used

    /* Send MCU app the supported features */
    tx_buf[cmd++] = HCI_CONTROL_GROUP_HIDD;

    hci_control_send_data( HCI_CONTROL_MISC_EVENT_VERSION, tx_buf, cmd );

    hci_control_send_data( HCI_CONTROL_HIDD_EVENT_CAPABILITY, dev_capabilities, DEVICE_CAPABILITY_LEN );
}

/////////////////////////////////////////////////////////////////////////////////////////////
/// handle key from HCI
/////////////////////////////////////////////////////////////////////////////////////////////
void hci_control_key(uint8_t * p_data, uint32_t data_len)
{
    if (data_len==2)
    {
        if (hci.registered_app_key_handler)
        {
            hci.registered_app_key_handler(p_data[0], (wiced_bool_t) p_data[1]);
        }
        else
        {
            APP_HCI_TRACE("Function not supported");
        }
    }
    else
    {
        APP_HCI_TRACE("Invalid parameter length = %d", data_len);
    }
}

/******************************************************************************************/
/******************************************************************************************/
void hci_control_send_report( uint8_t channel, uint8_t type, uint8_t *p_data, uint16_t length )
{
    APP_HCI_TRACE("cci_send_report ch:%d type:%d len:%d", channel, type, length);

    //send report only when link is connected
    if(link_is_connected())
    {
        if (!cfg_sec_mask() || link_is_encrypted())
        {
            uint8_t rpt_id = *p_data++;
            hidd_send_data(rpt_id, type, p_data, --length);
        }
        else
        {
            WICED_BT_TRACE("Link is not encrypted");
        }
    }
    else //enter discoverable/reconnecting
    {
        APP_HCI_TRACE("not connected, trying to reconnect...", channel, type, length);
        bt_enter_connect();
    }
}

/******************************************************************************************/
/******************************************************************************************/
void hci_hidd_handle_command( uint16_t cmd_opcode, uint8_t * p_data, uint32_t data_len )
{
    switch( cmd_opcode )
    {
    case HCI_CONTROL_COMMAND_RESET:
        APP_HCI_TRACE("Cmd:%04X -- Reset", cmd_opcode);
        hci_hidd_dev_handle_reset_cmd( );
        break;

    case HCI_CONTROL_COMMAND_TRACE_ENABLE:
        APP_HCI_TRACE("Cmd:%04X -- Trace %d %d", cmd_opcode, *p_data, *(p_data+1));
        hci_control_handle_trace_enable( p_data );
        break;

    case HCI_CONTROL_COMMAND_SET_LOCAL_BDA:
        APP_HCI_TRACE("Cmd:%04X -- Set address %B", cmd_opcode, p_data);
        hci_hidd_dev_handle_set_local_bda( p_data );
        break;

    case HCI_CONTROL_HIDD_COMMAND_ACCEPT_PAIRING:
        if ( bt_is_advertising())
        {
            APP_HCI_TRACE("Cmd:%04X -- stop advertising", cmd_opcode);
            bt_stop_advertisement();
        }
        else
        {
            APP_HCI_TRACE("Cmd:%04X -- pairing", cmd_opcode);
            bt_enter_pairing();
        }
        break;

    case HCI_CONTROL_HIDD_COMMAND_HID_HOST_ADDR:  // reqesting for host paired host address
        APP_HCI_TRACE("Cmd:%04X -- Host info req.", cmd_opcode);
        hci_control_send_paired_host_info();
        hci_control_send_state_change(BT_TRANSPORT_LE, link_is_connected() ? HID_LINK_CONNECTED :  HID_LINK_DISCONNECTED);
        hci_control_send_advertisement_state_evt( bt_get_advertising_mode() );
        break;

    case HCI_CONTROL_HIDD_COMMAND_CONNECT:
        // We check for link state just in case if Client Control is sending wrong command at wrong state
        if (link_is_connected())
        {
            APP_HCI_TRACE("Cmd:%04X -- Connect: Already connected, disconnecting instead...", cmd_opcode);
            bt_disconnect();
        }
        else
        {
            APP_HCI_TRACE("Cmd:%04X -- Connect", cmd_opcode);
            bt_enter_connect();
        }
        break;

    case HCI_CONTROL_HIDD_COMMAND_DISCONNECT:
        // We check for link state just in case if Client Control is sending wrong command at wrong state
        if (link_is_connected())
        {
            APP_HCI_TRACE("Cmd:%04X -- Disconnect", cmd_opcode);
            bt_disconnect();
        }
        else
        {
            APP_HCI_TRACE("Cmd:%04X -- Disconnect: Already disconnected, connecting instead...", cmd_opcode);
            bt_enter_connect();
        }
        break;

    case HCI_CONTROL_COMMAND_DELETE_NVRAM_DATA:
    case HCI_CONTROL_HIDD_COMMAND_VIRTUAL_UNPLUG:
        APP_HCI_TRACE("Cmd:%04X -- Remove pairing", cmd_opcode);
        app_remove_host_bonding();
        led_blink_stop(LINK_LED);
        led_off(LINK_LED);
        break;

    case HCI_CONTROL_HIDD_COMMAND_SEND_REPORT:
        APP_HCI_TRACE("Cmd:%04X -- Send Report: %A", cmd_opcode, p_data, data_len);
        hci_control_send_report(p_data[0], p_data[1], &p_data[2], (uint16_t)( data_len - 2 ));
        break;

    case HCI_CONTROL_MISC_COMMAND_GET_VERSION:
        APP_HCI_TRACE("Cmd:%04X -- Get Version", cmd_opcode);
        hci_control_misc_handle_get_version();
        break;

    case HCI_CONTROL_HIDD_COMMAND_KEY:
        APP_HCI_TRACE("Cmd:%04X -- key %d %d", cmd_opcode, p_data[0], p_data[1]);
        hci_control_key(p_data, data_len);
        break;

#if defined SUPPORT_AUDIO
    case HCI_CONTROL_HIDD_COMMAND_AUDIO_START_REQ:
        APP_HCI_TRACE("Cmd:%04X -- HCI_CONTROL_HIDD_COMMAND_AUDIO_START_REQ: %A",cmd_opcode, p_data, data_len);
        audio_set_routing_from_hci(TRUE, data_len > 1 ? p_data[1] : 0);
        hci.buffer_cnt = hci.req_cnt = 0;
        audio_start_request();
        break;

    case HCI_CONTROL_HIDD_COMMAND_AUDIO_STOP_REQ:
        APP_HCI_TRACE("Cmd:%04X -- HCI_CONTROL_HIDD_COMMAND_AUDIO_START_REQ: %A",cmd_opcode, p_data, data_len);
        audio_stop_request();
        audio_set_routing_from_hci(FALSE, FALSE);
        break;

    case HCI_CONTROL_HIDD_COMMAND_AUDIO_DATA:
        if (data_len)
        {
            handle_audio_data((int16_t *) p_data, data_len >> 1);
        }
        else
        {
            APP_HCI_TRACE("HCI_CONTROL_HIDD_COMMAND_AUDIO_DATA: length 0, send stop request");
            wiced_stop_timer(&hci.timer);
            audio_stop_request();
        }
        break;

    case HCI_CONTROL_HIDD_COMMAND_AUDIO_SET_COPY:
        break;

    case HCI_CONTROL_HIDD_COMMAND_AUDIO_MIC_START_STOP:
        APP_HCI_TRACE("Cmd:%04X -- HCI_CONTROL_HIDD_COMMAND_AUDIO_MIC_START_STOP: %A",cmd_opcode, p_data, data_len);
        // if we have sample rate parameter
        if (data_len > 1)
        {
            APP_HCI_TRACE("Set Sampling Rate %d",p_data[1]);
            mic_set_codec_sampling_freq(p_data[1] ? HIDD_CODEC_SAMP_FREQ_16K : HIDD_CODEC_SAMP_FREQ_8K);
        }
        if (p_data[0])
        {
            // if we have encoded parameter, pass it down, too.
            audio_set_routing_to_hci(TRUE, data_len > 2 ? p_data[2] : 0);
            audio_start();
        }
        else
        {
            audio_stop();
            audio_set_routing_to_hci(FALSE, 0);
        }
        break;
#endif
    default:
        APP_HCI_TRACE("Cmd:%04X not Handled",cmd_opcode );
        break;
    }
}

uint32_t hci_dev_handle_command( uint8_t * p_data, uint32_t length )
{
    uint16_t opcode;
    uint16_t payload_len;
    uint8_t status = HCI_CONTROL_STATUS_SUCCESS;
    uint8_t * p_rx_buf = p_data;

    if ( !p_rx_buf )
    {
        return HCI_CONTROL_STATUS_INVALID_ARGS;
    }
    //Expected minimum 4 byte as the wiced header
    if( length < 4 )
    {
        APP_HCI_TRACE("invalid params");
        wiced_transport_free_buffer( p_rx_buf );
        return HCI_CONTROL_STATUS_INVALID_ARGS;
    }

    READ_LITTLE_ENDIAN_TO_UINT16(opcode, p_data, length);     // Get opcode
    READ_LITTLE_ENDIAN_TO_UINT16(payload_len, p_data, length); // Get len

    hci_hidd_handle_command( opcode, p_data, payload_len );

    //Freeing the buffer in which data is received
    wiced_transport_free_buffer( p_rx_buf );
    return ( 0 );
}

/*
 *  transfer connection event to uart
 */
void hci_control_send_connect_evt( uint8_t addr_type, BD_ADDR addr, uint16_t con_handle, uint8_t role )
{
    int i;
    uint8_t   tx_buf [30];
    uint8_t   *p = tx_buf;

    *p++ = addr_type;
    for ( i = 0; i < 6; i++ )
        *p++ = addr[5 - i];
    *p++ = con_handle & 0xff;
    *p++ = ( con_handle >> 8 ) & 0xff;
    *p++ = role;

    hci_control_send_data( HCI_CONTROL_LE_EVENT_CONNECTED, tx_buf, ( int )( p - tx_buf ) );
}

/*
 *  send audio data request event
 *
 *  count -- sample count of 16-bit data
 */
void hci_control_send_audio_data_req()
{
    hci.req_cnt += audio_wdata_cnt();

    APP_HCI_TRACE2("---request %d words data from HCI, pending req:%d", audio_wdata_cnt(), hci.req_cnt);
    send_audio_data_req();
    APP_HCI_TRACE2("   start timer for %d ms", link_get_acl_conn_param()->conn_interval);
    wiced_start_timer(&hci.timer, link_get_acl_conn_param()->conn_interval+1);  // use +1 to make it slower to avoid congestion
}

/*
 *  transfer disconnection event to UART
 */
void hci_control_send_disconnect_evt( uint8_t reason, uint16_t con_handle )
{
    uint8_t   tx_buf [3];
    uint8_t   *p = tx_buf;

    *p++ = con_handle & 0xff;
    *p++ = ( con_handle >> 8 ) & 0xff;
    *p++ = reason;

    hci_control_send_data( HCI_CONTROL_LE_EVENT_DISCONNECTED, tx_buf, ( int )( p - tx_buf ) );
}

/*
 * Send notification to the host that pairing has been completed
 */
void hci_control_send_pairing_complete_evt( uint8_t result, uint8_t *p_bda, uint8_t type )
{
    uint8_t tx_buf[12];
    uint8_t *p = tx_buf;

    *p++ = result;

    WICED_BT_TRACE("hci_control_send_pairing_complete_evt: result:%d, bda=%B, type=%d", result, p_bda, type);

    for (int i = 0; i < BD_ADDR_LEN; i++ )
    {
        *p++ = p_bda[BD_ADDR_LEN - 1 - i];
    }

    *p++ = type;

    hci_control_send_data( HCI_CONTROL_EVENT_PAIRING_COMPLETE, tx_buf, ( int ) ( p - tx_buf ) );
}

void hci_control_set_capability(char audio, char mouse, char ir)
{
    dev_capabilities[0] = audio;
    dev_capabilities[1] = mouse;
    dev_capabilities[2] = ir;
}

/*
 * hci_control_transport_status
 * This callback function is called when the MCU opens the Wiced UART
 */
void hci_control_transport_status( wiced_transport_type_t type )
{
    APP_HCI_TRACE("hci_control_transport_status %x", type );

    // Tell Host that App is started
    hci_control_send_data( HCI_CONTROL_EVENT_DEVICE_STARTED, NULL, 0 );
}

void hci_control_enable_trace()
{
    /* Register callback for receiving hci traces */
    wiced_bt_dev_register_hci_trace( hci_hci_trace_cback );
}

void hci_control_init()
{
    APP_HCI_TRACE("hci control init");
    wiced_transport_init( &transport_cfg );
    hci_control_enable_trace();
    wiced_init_timer( &hci.timer, hci_timer_cb, 0, WICED_MILLI_SECONDS_TIMER );
}

void hci_control_register_key_handler(hidd_app_hci_key_callback_t key_handler)
{
    hci.registered_app_key_handler = key_handler;
}

#else
 #ifdef HCI_TRACES_DUMP_TO_PUART
//
// The HCI commands are dumped PUART in byte array.
// User the tool TraceToSpy.exe to read the dump file to btspy to decode the HCI messages
//
void hci_trace_to_puart_cback( wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data )
{
    APP_HCI_TRACE("HCI event type %d len %d\n", type,length );
    wiced_trace_array(  p_data, length );
}

void hci_control_enable_trace()
{
    wiced_bt_dev_register_hci_trace( hci_trace_to_puart_cback );
}

 #endif
#endif
