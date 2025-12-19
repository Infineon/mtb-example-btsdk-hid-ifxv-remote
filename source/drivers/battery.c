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
 * This file defines the interface of battery report service
 *
 */

#ifdef BATTERY_REPORT_SUPPORT
#include "app.h"
#include "battery.h"

#if CHIP!=20829 && defined WICED_BT_TRACE_ENABLE
# undef WICED_BT_TRACE
# define WICED_BT_TRACE(format,...)   wiced_printf(NULL, 0, (format"\n"), ##__VA_ARGS__)
#endif

BatteryReport batRpt={RPT_ID_IN_BATTERY,{100}};

/********************************************************************************
 * Function Name: Bat_battery_level_change_notification
 ********************************************************************************
 * Summary: send out battery report when battery level changed
 *
 * Parameters:
 *  newLevel -- new battery level
 *
 * Return:
 *  None
 *
 *******************************************************************************/
static void Bat_battery_level_change_notification(uint32_t newLevel)
{
    if (batRpt.level[0] != newLevel)
    {
        WICED_BT_TRACE("battery level changed to %d", newLevel);
        batRpt.level[0] = newLevel;
        hidd_send_report(&batRpt, sizeof(BatteryReport));
        wiced_hal_batmon_set_battery_report_sent_flag(WICED_TRUE);
    }
}

/*******************************************************************************
 * Function Name: void bat_init
 ********************************************************************************
 * Summary: initialize battery report
 *
 * Parameters:
 *  void (shutdown_cb)()  -- shutdown callback function
 *
 * Return:
 *  none
 *
 *******************************************************************************/
void bat_init(void (shutdown_cb)())
{
    //battery monitoring configuraion
    wiced_hal_batmon_config(ADC_INPUT_VDDIO,    // ADC input pin, use ADC_INPUT_P38 for CYW920835REF-RCU-01 if P38 is not used for IR
                            3000,               // Period in millisecs between battery measurements
                            8,                  // Number of measurements averaged for a report, max 16
                            3200,               // The full battery voltage in mili-volts
                            1800,               // The voltage at which the batteries are considered drained (in milli-volts)
                            1700,               // System should shutdown if it detects battery voltage at or below this value (in milli-volts)
                            100,                // battery report max level
                            RPT_ID_IN_BATTERY,  // battery report ID
                            1,                  // battery report length
                            1);                 // Flag indicating that a battery report should be sent when a connection is established

     //register App low battery shut down handler
    wiced_hal_batmon_register_low_battery_shutdown_cb(shutdown_cb);
    wiced_hal_batmon_add_battery_observer(Bat_battery_level_change_notification);

    //Setup Battery Service
    wiced_hal_batmon_init();
}

#endif // BATTERY_REPORT_SUPPORT
