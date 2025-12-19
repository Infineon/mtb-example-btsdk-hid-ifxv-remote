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
 * button functions
 *
 */

#ifdef SUPPORT_BUTTON
#include "app.h"
#include "cycfg_pins.h"

#undef WICED_BT_TRACE
#define WICED_BT_TRACE(format,...)   wiced_printf(NULL, 0, (format"\n"), ##__VA_ARGS__)

static void button_interrupt_handler( void *user_data, uint8_t gpio )
{
    int index = (uint32_t) user_data;

#if IR_TRACE==3
    // Use button for IR testing
    ir_button(IR_KEY_INDEX, button_down(index));
#elif (CUSTOM_KEY_MATRIX >= 1)
    static wiced_bool_t last_state;
    wiced_bool_t new_state = button_down(index);

    if (last_state != new_state)
    {
        last_state = new_state;
        WICED_BT_TRACE("button %d (P%d) %s", index, gpio, new_state ? "down" : "up");
    }
#else
    int down = button_down(index);

    if(link_is_connected())
    {
        audio_button(down);
    }
    else
    {
        WICED_BT_TRACE("User button %d (P%d) %s", index, gpio, button_down(index) ? "down" : "up");
        if (down)
        {
            if (host_is_paired())
            {
                WICED_BT_TRACE("Enter reconnect");
                bt_enter_connect();
            }
            else
            {
                WICED_BT_TRACE("Enter pairing");
                bt_enter_pairing();
            }
        }
    }
#endif
}

wiced_bool_t button_down(int index)
{
    return wiced_hal_gpio_get_pin_input_status( WICED_GET_PIN_FOR_BUTTON(index) ) == wiced_platform_get_button_pressed_value(index);
}

void button_init_index(int idx)
{
    WICED_BT_TRACE("Initialized P%d for button, cfg=%04x", *platform_button[idx].gpio, platform_button[idx].config);
    wiced_platform_register_button_callback( idx, button_interrupt_handler, (void *) idx, WICED_PLATFORM_BUTTON_BOTH_EDGE);

    // This will enforce button to work in SDS
    wiced_hal_gpio_slimboot_reenforce_cfg(*platform_button[idx].gpio, platform_button[idx].config);
}

void button_init()
{
    for (int i=0; i<button_count; i++)
    {
        button_init_index(i);
    }
}

wiced_bool_t button_active()
{
    for (int i=0; i<button_count; i++)
    {
        if (button_down(i))
        {
            return TRUE;
        }
    }
    return FALSE;
}

void button_check_boot_action()
{
    for (int i=0; i<button_count; i++)
    {
        if (button_down(i))
        {
            button_interrupt_handler((void *) i, *platform_button[i].gpio); // take action
        }
    }
}

#endif // SUPPORT_BUTTON
