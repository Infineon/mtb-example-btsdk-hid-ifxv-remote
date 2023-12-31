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
 *
 * button functions
 *
 */

#ifdef WICED_EVAL
#include "app.h"
#include "cycfg_pins.h"

static void button_interrupt_handler( void *user_data, uint8_t value )
{
    int down = button_down();

    if(link_is_connected())
    {
        audio_button(down);
    }
    else if (down)
    {
        if (host_is_paired())
        {
            bt_enter_connect();
        }
        else
        {
            bt_enter_pairing();
        }
    }
}

wiced_bool_t button_down()
{
    return wiced_hal_gpio_get_pin_input_status( WICED_GET_PIN_FOR_BUTTON(WICED_PLATFORM_BUTTON_1) ) == wiced_platform_get_button_pressed_value(WICED_PLATFORM_BUTTON_1);
}

void button_init()
{
    wiced_platform_register_button_callback( WICED_PLATFORM_BUTTON_1, button_interrupt_handler, NULL, WICED_PLATFORM_BUTTON_BOTH_EDGE);
}

void button_check_boot_action()
{
    if (button_down())
    {
        button_interrupt_handler(NULL, 0); // take action
    }
}

#endif // WICED_EVAL
