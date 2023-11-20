/*
 * $ Copyright YEAR Cypress Semiconductor $
 */

/**
 * file sds.c
 *
 * This is module handles SDS functions
 */
#include "sds.h"

#if is_SDS_capable && (SLEEP_ALLOWED > 1)

#include "clock_timer.h"
#include "wiced_timer.h"
#include "wiced_bt_trace.h"
#include "wiced_hal_gpio.h"
#include "emconinfo.h"
#include "spar_utils.h"
#include "app.h"

#if CHIP!=20829 && defined WICED_BT_TRACE_ENABLE
# undef WICED_BT_TRACE
# define WICED_BT_TRACE(format,...) if (!sds.attempted) wiced_printf(NULL, 0, (format"\n"), ##__VA_ARGS__)
#endif

#if SDS_TRACE
 #define APP_SDS_TRACE(format, ...) if (!sds.attempted) wiced_printf(NULL, 0, (format"\n"), ##__VA_ARGS__)
 #if SDS_TRACE>1
  #define APP_SDS_TRACE2(format, ...) if (!sds.attempted) wiced_printf(NULL, 0, (format"\n"), ##__VA_ARGS__)
 #else
  #define APP_SDS_TRACE2(...)
 #endif
#else
 #define APP_SDS_TRACE(...)
 #define APP_SDS_TRACE2(...)
#endif

#ifdef ALLOW_SDS_IN_DISCOVERABLE
 #define MAX_SDS_TIMER 2                  // adv timer & idle timer
#else
 #define MAX_SDS_TIMER 1                  // idle timer only
#endif

#pragma pack(1)
typedef struct
{
    uint64_t  start_clock_in_us;
    uint32_t  timeout_in_ms;
    uint8_t   param;
} aon_timer_data_t;

typedef struct
{
    uint8_t   adv_filter_accept_list_enabled:1;
} aon_flags_t;

typedef struct
{
    EMCONINFO_DEVINFO   emconinfo;
    link_t              link;
    uint16_t            nflags, iflags;
    aon_timer_data_t    timer[MAX_SDS_TIMER];
    uint8_t             timer_running;  // bit encoded: the maximum number of MAX_SDS_TIMER is 8
    uint8_t             led_state;      // bit encoded: the maximum number of WICED_PLATFORM_LED_MAX is 8
    aon_flags_t         flags;
#ifdef FILTER_ACCEPT_LIST_FOR_ADVERTISING
    uint8_t             adv_filter_addr[BD_ADDR_LEN];
#endif
} aon_save_content_t;
#pragma pack()

PLACE_DATA_IN_RETENTION_RAM aon_save_content_t aon_data;

typedef struct
{
    sds_timer_t*            p_timer;
    wiced_timer_callback_t  p_cb;
} sds_timer_data_t;

typedef struct
{
    wiced_timer_t       allow_sds_timer;
    sds_timer_data_t    timer[MAX_SDS_TIMER];

#ifdef ALLOW_SDS_IN_DISCOVERABLE
     ///DISCOVERABLE timer (osapi timer that can be supported in uBCS mode)
    hidd_blelink_timer_t discoverable_timer;

    /// timer to switch from DISCOVERABLE to HIDLINK_LE_ADVERTISING_IN_uBCS_UNDIRECTED
    wiced_timer_t state_switch_timer;

    /// timeout value of state_switch_timer in mili seconds
    uint32_t state_switch_timeout_in_ms;
#endif

// flags
    uint8_t             allow_sds:1;
    uint8_t             attempted:1;

} sds_t;

static sds_t sds = {0};

/////////////////////////////////////////////////////////////////////////////////
/// timeout handler
/// \param arg - don't care
/// \param overTimeInUs - don't care
/////////////////////////////////////////////////////////////////////////////////
static void sds_timer_cb(INT32 index, UINT32 overTimeInUs)
{
    APP_SDS_TRACE2("SDS timer cb");

    if (index < MAX_SDS_TIMER && sds.timer[index].p_cb)
    {
        (sds.timer[index].p_cb)(aon_data.timer[index].param);
    }
}

/////////////////////////////////////////////////////////////////////////////////
/// Mark all timers that is running so when waking up from SDS, we know what to check
/////////////////////////////////////////////////////////////////////////////////
static void sds_timer_save_to_aon()
{
    APP_SDS_TRACE2("sds_timer_save_to_aon");

    // save all running timer info
    aon_data.timer_running = 0;
    for (int i=0; i<MAX_SDS_TIMER; i++)
    {
        if (sds.timer[i].p_timer && sds_timer_in_use(sds.timer[i].p_timer))
        {
            aon_data.timer_running |= (1<<i);
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////
/// for all running timers, check if the timer has expired.
/// if expired, initiate callback and stop the timer.
/// Otherwise, restart the timer again.
/////////////////////////////////////////////////////////////////////////////////
static void sds_timer_restore_from_aon()
{
    APP_SDS_TRACE2("sds_timer_restore_from_aon");

    for (int i=0; i<MAX_SDS_TIMER; i++)
    {
        if (aon_data.timer_running & (i<<1))
        {
            uint64_t time_passed_in_ms = (clock_SystemTimeMicroseconds64() - aon_data.timer[i].start_clock_in_us)/1000;

            // is timer expired?
            if ((uint32_t) time_passed_in_ms >= aon_data.timer[i].timeout_in_ms)
            {
                APP_SDS_TRACE("SDS timer expired");
                sds_timer_cb(i, 0);  // call back
            }
            else
            {
                APP_SDS_TRACE2("Restart SDS timer %d for %d ms", i, aon_data.timer[i].timeout_in_ms - (uint32_t) time_passed_in_ms);
                // restart timer for the time remaining
                sds_timer_start(sds.timer[i].p_timer, aon_data.timer[i].timeout_in_ms - (uint32_t) time_passed_in_ms);
            }
        }
    }
}

/**  Initializes the timer
 *
 *@param[in]    p_timer         :Pointer to the timer structure
 *@param[in]    p_cb            :Timer callback function to be invoked on timer expiry
 *@param[in]    param           :Parameter to be passed to the timer callback function which gets invoked on timer expiry, if any
 *
 * @return   wiced_result_t
 */
wiced_result_t sds_timer_init( sds_timer_t *p_timer, wiced_timer_callback_t p_cb, uint8_t param)
{
    for (int i=0; i<MAX_SDS_TIMER; i++)
    {
        // find an empty spot
        if (!sds.timer[i].p_timer)
        {
            APP_SDS_TRACE("sds_timer_init %d", i);
            sds.timer[i].p_timer = p_timer;
            sds.timer[i].p_cb = p_cb;
            aon_data.timer[i].param = param;
            osapi_createTimer(p_timer, sds_timer_cb, i);
            return WICED_SUCCESS;
        }
    }

    WICED_BT_TRACE("sds_timer_init: Error! Not enough space. Increase MAX_SDS_TIMER");
    return WICED_ERROR;
}

/**  Starts the timer
 * Timer should be initialized before starting the timer. Running the timer interfere with the
 * low power modes of the chip. Time to remain in the low power mode is dependent on the
 * timeout values of the running timers, ie time to sleep is dependent on the time after which
 * the next timer among the active timers expires.
 *
 * @param[in]    wiced_timer_t           ::Pointer to the timer structure
 *
 * @return       wiced_result_t
 */

wiced_result_t sds_timer_start(sds_timer_t *p_timer, uint32_t timeout)
{
    for (int i=0; i<MAX_SDS_TIMER; i++)
    {
        // find an empty spot
        if (sds.timer[i].p_timer == p_timer)
        {
            APP_SDS_TRACE("Start SDS timer %d for %d ms", i, timeout);
            aon_data.timer[i].start_clock_in_us = clock_SystemTimeMicroseconds64();
            aon_data.timer[i].timeout_in_ms = timeout;
            osapi_activateTimer(p_timer, timeout * 1000UL);
            return WICED_SUCCESS;
        }
    }
    WICED_BT_TRACE("sds_timer_start: timer is not initialized");
    return WICED_ERROR;
}

/** Stops the timer
 *
 * @param[in]    wiced_timer_t           :Pointer to the timer structure
 *
 * @return       wiced_result_t
 */
wiced_result_t sds_timer_stop(sds_timer_t *p_timer)
{
    for (int i=0; i<MAX_SDS_TIMER; i++)
    {
        // find an empty spot
        if (sds.timer[i].p_timer == p_timer)
        {
            APP_SDS_TRACE("Stop SDS timer %d", i);
            osapi_deactivateTimer(p_timer);
            return WICED_SUCCESS;
        }
    }
    WICED_BT_TRACE("sds_timer_stop: timer is not initialized");
    return WICED_ERROR;
}

/////////////////////////////////////////////////////////////////////////////////
/// timeout handler
///
/// \param arg - don't care
/////////////////////////////////////////////////////////////////////////////////
static void allow_sds_timer_cb( uint32_t arg )
{
    APP_SDS_TRACE("SDS Allowed");
    sds.allow_sds = TRUE;
}

#ifdef ALLOW_SDS_IN_DISCOVERABLE
/////////////////////////////////////////////////////////////////////////////////
/// timeout handler
///
/// \param arg - don't care
/////////////////////////////////////////////////////////////////////////////////
void sds_stateswitchtimerCb( uint32_t arg)
{
    if (HIDLINK_LE_DISCOVERABLE == blelink.subState)
    {
        hidd_blelink_set_state(HIDLINK_LE_ADVERTISING_IN_uBCS_UNDIRECTED);
    }
}

/////////////////////////////////////////////////////////////////////////////////
/// timeout handler
///
/// \param args - don't care
/// \param overTimeInUs - don't care
/////////////////////////////////////////////////////////////////////////////////
void hidd_blelink_discoverabletimerCb(int32_t args, uint32_t overTimeInUs)
{
    WICED_BT_TRACE("\nhidd_blelink_discoverabletimerCb!!!");
    wiced_bt_start_advertisements(BTM_BLE_ADVERT_OFF, 0, NULL);
}
#endif

/////////////////////////////////////////////////////////////////////////////////
/// save data to aon, prepare for enter SDS
/////////////////////////////////////////////////////////////////////////////////
void sds_save_data_to_aon()
{
    WICED_BT_TRACE("SDS");

    //save embeded controller info to AON
    memcpy(&aon_data.emconinfo, &emConInfo_devInfo, sizeof(EMCONINFO_DEVINFO));

    //save link data
    memcpy(&aon_data.link, &link, sizeof(link_t));

    sds_timer_save_to_aon();

    // save cccd flags
    hidd_get_cccd_flags( &aon_data.nflags, &aon_data.iflags );

    // save LED state
    aon_data.led_state = led_get_states();

    wiced_hal_gpio_slimboot_reenforce_outputpin_value();

    sds.attempted = TRUE;
}

/////////////////////////////////////////////////////////////////////////////////
/// restore data from aon, this function should called right after wake from SDS
/////////////////////////////////////////////////////////////////////////////////
void sds_restore_data_from_aon()
{
    APP_SDS_TRACE2("Restore SDS data from AON");

    //restore embeded controller info for the LE link (peer device info, bonded, encrypted, connection parameters etc.)
    memcpy(&emConInfo_devInfo, &aon_data.emconinfo, sizeof(EMCONINFO_DEVINFO));

    //Restore link data
    memcpy(&link, &aon_data.link, sizeof(link_t));

    led_set_states(aon_data.led_state);

    sds_timer_restore_from_aon();

    // restore cccd flags
    hidd_set_cccd_flags( aon_data.nflags, aon_data.iflags );
}

/////////////////////////////////////////////////////////////////////////////////
/// sds_wake
///   This function handles wake up from SDS
/////////////////////////////////////////////////////////////////////////////////
void sds_wake()
{
    sds_restore_data_from_aon();

#ifdef FILTER_ACCEPT_LIST_FOR_ADVERTISING
    // Restore filter accept list for advertising from AON
    if (aon_data.flags.adv_filter_accept_list_enabled)
    {
        //add to Filter Accept List
        wiced_bt_ble_update_advertising_filter_accept_list(WICED_TRUE, aon_data.adv_filter_addr);

        //update advertising filer policy to use Filter Accept List to filter scan and connect request
        wiced_btm_ble_update_advertisement_filter_policy(BTM_BLE_ADV_POLICY_FILTER_CONN_FILTER_SCAN);
    }
#endif
}

/////////////////////////////////////////////////////////////////////////////////
/// sds_link_up
///   This function is called when link is up
/////////////////////////////////////////////////////////////////////////////////
void sds_link_up()
{
#ifdef ALLOW_SDS_IN_DISCOVERABLE
    //stop discoverable timer
    hidd_blelink_stop_timer(&blelink.discoverable_timer);

    aon_data.osapi_app_timer_running &= ~BLEHIDLINK_ADV_CONNECTABLE_UNDIRECTED_TIMER;
    if ((aon_data.osapi_app_timer_running >> 1) == 0)
    {
        aon_data.osapi_app_timer_running = 0; // no more application osapi timer is running
    }
#endif
}

/////////////////////////////////////////////////////////////////////////////////
/// sds_link_down
///   This function is called when link is down
/////////////////////////////////////////////////////////////////////////////////
void sds_link_down()
{
}

/////////////////////////////////////////////////////////////////////////////////
/// sds_allowed_in_ms
///   After calling this function, SDS is prohibitted until the timer expires
/////////////////////////////////////////////////////////////////////////////////
void sds_allowed_in_ms(uint32_t allow_sds_in_ms)
{
    if (allow_sds_in_ms)
    {
        APP_SDS_TRACE("SDS Allowed in %d ms", allow_sds_in_ms);
        sds.allow_sds = FALSE;
        wiced_start_timer(&sds.allow_sds_timer, allow_sds_in_ms);
    }
    else
    {
        APP_SDS_TRACE("SDS Allowed");
        sds.allow_sds = TRUE;
    }
}

/////////////////////////////////////////////////////////////////////////////////
/// sds_is_allowed
///   return TRUE when SDS is allowed
/////////////////////////////////////////////////////////////////////////////////
void sds_set_allowed(wiced_bool_t enable)
{
    if (enable && wiced_is_timer_in_use(&sds.allow_sds_timer))
    {
        APP_SDS_TRACE("Wait for sleep allowed timer to enable SDS");
    }
    else
    {
        APP_SDS_TRACE("SDS %sAllowed",enable ? "" : "not ");
        sds.allow_sds = enable;
    }
}

/////////////////////////////////////////////////////////////////////////////////
/// sds_is_allowed
///   return TRUE when SDS is allowed
/////////////////////////////////////////////////////////////////////////////////
wiced_bool_t sds_is_allowed()
{
    return sds.allow_sds;
}

/////////////////////////////////////////////////////////////////////////////////
/// sds_attempted
///   return TRUE when SDS is allowed
/////////////////////////////////////////////////////////////////////////////////
wiced_bool_t sds_attempted()
{
    return sds.attempted;
}

/////////////////////////////////////////////////////////////////////////////////
/// sds_init
///   This function is called once and only once at start up
/////////////////////////////////////////////////////////////////////////////////
void sds_init()
{
    wiced_init_timer( &sds.allow_sds_timer, allow_sds_timer_cb, 0, WICED_MILLI_SECONDS_TIMER );

#ifdef ALLOW_SDS_IN_DISCOVERABLE
    app.state_switch_timeout_in_ms = 1000; // 1 seconds

    /// timer to switch from DISCOVERABLE to HIDLINK_LE_ADVERTISING_IN_uBCS_UNDIRECTED
    wiced_init_timer( &app.state_switch_timer, app_stateswitchtimerCb, 0, WICED_MILLI_SECONDS_TIMER );

    ///discoverable timer that can be supported in uBCS mode
    hidd_blelink_init_timer(&app.discoverable_timer, app_discoverabletimerCb, 0);
#endif
}

#ifdef FILTER_ACCEPT_LIST_FOR_ADVERTISING
/////////////////////////////////////////////////////////////////////////////////
/// sds_is_allowed
///   return TRUE when SDS is allowed
/////////////////////////////////////////////////////////////////////////////////
void sds_set_filtering_adv(wiced_bool_t enable, uint8_t * addr)
{
    aon_data.flags.adv_filter_accept_list_enabled = enable;
    if (enable)
    {
        memcpy(aon_data.adv_filter_addr, addr, BD_ADDR_LEN);
    }
}
#endif

#endif // #if is_SDS_capable

/////////////////////////////////////////////////////////////////////////////////
