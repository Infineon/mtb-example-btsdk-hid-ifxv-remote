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
 * Keyscan Interface
 *
 */

#ifdef SUPPORT_KEYSCAN

#include "app.h"
#include "wiced_hal_mia.h"

#if CHIP!=20829 && defined WICED_BT_TRACE_ENABLE
# undef WICED_BT_TRACE
# define WICED_BT_TRACE(format,...)   wiced_printf(NULL, 0, (format"\n"), ##__VA_ARGS__)
#endif

#if KEY_TRACE
 #define APP_KEY_TRACE WICED_BT_TRACE
#else
 #define APP_KEY_TRACE(...)
#endif

typedef struct {
    key_detected_callback_t * appCb;
} kscan_data_t;

static kscan_data_t ks = {0};

/////////////////////////////////////////////////////////////////////////////////
/// This function polls for key activity and queues any key events in the
/// FW event queue. Events from the keyscan driver are processed until the driver
/// runs out of events.
/////////////////////////////////////////////////////////////////////////////////
static void kscan_pollEvent(void * userData)
{
    // Poll the hardware for events
    wiced_hal_mia_pollHardware();

    HidEventKey event = {{HID_EVENT_KEY_STATE_CHANGE}};

    while (wiced_hal_keyscan_get_next_event(&event.keyEvent))
    {
        if (ks.appCb)
        {
            (ks.appCb)(event.keyEvent.keyCode, event.keyEvent.upDownFlag == KEY_DOWN);
        }
    }
}

/*******************************************************************************
 * Function Name: void kscan_reset(void)
 ********************************************************************************
 * Summary: Resets keyscan and getting ready key events
 *
 * Parameters:
 *  None
 *
 * Return:
 *  None
 *
 *******************************************************************************/
void kscan_reset(void)
{
    // Reset the keyscan HW
    wiced_hal_keyscan_reset();

    // Configure GPIOs for keyscan operation
    wiced_hal_keyscan_config_gpios();
}

/*******************************************************************************
 * Function Name: void kscan_init
 ********************************************************************************
 * Summary: Initialize keyscan configuration,
 *
 * Parameters:
 *  row, col -- key matrix row & col dimention
 *  poll_callback_t * pcb -- application poll function pointer to poll user activities
 *  key_detected_callback_t * cb -- application callback function pointer to handle key event
 *
 * Return:
 *  None
 *
 *******************************************************************************/
void kscan_init(uint8_t row, uint8_t col, key_detected_callback_t * cb)
{
    //save applicatino callback function pointer
    ks.appCb = cb;

    WICED_BT_TRACE("keyscan %dx%d", row, col);
    //keyscan initialize
    wiced_hal_keyscan_configure(row, col);
    wiced_hal_keyscan_init();
    wiced_hal_keyscan_register_for_event_notification(kscan_pollEvent, NULL);
}

#endif // SUPPORT_KEYSCAN
