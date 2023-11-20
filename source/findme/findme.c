/*
 * Copyright 2016-2023, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
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
