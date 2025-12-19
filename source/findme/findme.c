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
*/

#ifdef SUPPORT_FINDME

#include "gki_target.h"
#include "wiced_memory.h"
#include "wiced_timer.h"
#include "app.h"

#define ALERT_TIMEOUT_IN_SEC 30    // 30 second to timeout

#if CHIP!=20829 && defined WICED_BT_TRACE_ENABLE
# undef WICED_BT_TRACE
# define WICED_BT_TRACE(format,...)   wiced_printf(NULL, 0, (format"\n"), ##__VA_ARGS__)
#endif

#if FINDME_TRACE==1
# define APP_FINDME_TRACE        WICED_BT_TRACE
# define APP_FINDME_TRACE2(...)
#elif FINDME_TRACE>1
# define APP_FINDME_TRACE        WICED_BT_TRACE
# define APP_FINDME_TRACE2       WICED_BT_TRACE
#else
# define APP_FINDME_TRACE(...)
# define APP_FINDME_TRACE2(...)
#endif

enum ble_findme_alert_level
{
    NO_ALERT                        = 0,
    MILD_ALERT                      = 1,
    HIGH_ALERT                      = 2,
    UNDIRECTED_DISCOVERABLE_ALERT   = 3,
};

wiced_timer_t findme_timer;
static uint8_t activeAlterLevel;

////////////////////////////////////////////////////////////////////////////////
///  This function is the alert BUZ timer timeout handler
////////////////////////////////////////////////////////////////////////////////
static void FINDME_timeout(uint32_t unused)
{
    led_blink_stop(RED_LED);
}

////////////////////////////////////////////////////////////////////////////////
///  This function is the callback function for the find me attribute handle.
/// It controls the LED behavior depending on the value of the write command
////////////////////////////////////////////////////////////////////////////////
int findme_gatts_req_write_handler( uint16_t conn_id, wiced_bt_gatt_write_t * p_data )
{
    if ((HDLC_IAS_ALERT_LEVEL_VALUE == p_data->handle) && (p_data->val_len == 1)) // alert level len is 1 byte
    {

        activeAlterLevel = *(p_data->p_val);

        switch(activeAlterLevel)
        {
        case NO_ALERT:
            WICED_BT_TRACE("Immediate Alert: No Alert");
            wiced_stop_timer(&findme_timer);
            led_blink_stop(RED_LED);
            break;

        case MILD_ALERT:
            WICED_BT_TRACE("Immediate Alert: Mild Alert");
            led_blink(RED_LED, 0, 500);
            wiced_start_timer(&findme_timer,ALERT_TIMEOUT_IN_SEC);
            break;

        case HIGH_ALERT:
            WICED_BT_TRACE("Immediate Alert: High Alert");
            led_blink(RED_LED, 0, 200);
            wiced_start_timer(&findme_timer,ALERT_TIMEOUT_IN_SEC);
            break;
        }
        return WICED_BT_GATT_SUCCESS;
    }
    return WICED_BT_GATT_NOT_FOUND;
}

////////////////////////////////////////////////////////////////////////////////
///  This function is to check if "find me alert" is active.
///   i.e.buz/led active or not.
////////////////////////////////////////////////////////////////////////////////
uint8_t findme_is_active(void)
{
    return wiced_is_timer_in_use(&findme_timer);
}

////////////////////////////////////////////////////////////////////////////////
///  This function is the FIND ME profile initialization
/// - configure LED for find me alert
/// - register write handle cb for find me attribute handle.
////////////////////////////////////////////////////////////////////////////////
void findme_init(void)
{
    APP_FINDME_TRACE("Findme Init");
    wiced_init_timer( &findme_timer, FINDME_timeout, 0, WICED_SECONDS_TIMER );
}


#endif /* SUPPORT_FINDME */
