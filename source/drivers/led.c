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

/********************************************************************************
*
* File Name: hidd_lib_led.c
*
* Abstract: This file implements LED functions
* Functions:
*
*******************************************************************************/
#if LED_SUPPORT
#include "wiced_bt_trace.h"
#include "wiced_timer.h"
#include "wiced_hal_mia.h"
#include "app.h"

#define BLINK_CODE_SPEED 500
#define ERROR_CODE_BLINK_BREAK      4000

#if CHIP!=20829 && defined WICED_BT_TRACE_ENABLE
# undef WICED_BT_TRACE
# define WICED_BT_TRACE(format,...)   wiced_printf(NULL, 0, (format"\n"), ##__VA_ARGS__)
#endif

#if LED_TRACE
 #define APP_LED_TRACE WICED_BT_TRACE
#else
 #define APP_LED_TRACE(...)
#endif
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
static struct {
    uint8_t       curr_state:1;
    uint8_t       state:1;
    uint8_t       blinking:1;

    wiced_timer_t blinking_timer;
    uint16_t      blinking_duration;
    uint8_t       blinking_count;
    uint8_t       blinking_repeat_code;

} led[WICED_PLATFORM_LED_MAX]={0};

static struct {
    const wiced_platform_led_config_t * platform;
    uint8_t       count;
} led_cfg = {0};

#define led_initialized()  (led_cfg.count)
#define VALID_LED_IDX(idx) (led_initialized() && idx < led_cfg.count)

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
static void LED_setState(uint8_t idx, wiced_bool_t newState)
{
    if (VALID_LED_IDX(idx))
    {
        uint8_t logic = led_cfg.platform[idx].default_state;  // logic = LED off
        if (newState == LED_ON)
        {
           logic = !logic;   // logic = LED on
        }
        APP_LED_TRACE("LED pin %d %d", *led_cfg.platform[idx].gpio, logic);
        wiced_hal_gpio_set_pin_output(*led_cfg.platform[idx].gpio, logic);
        led[idx].curr_state = newState;
    }
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
static void LED_blink_handler(uint32_t idx)
{
    if (VALID_LED_IDX(idx) && led[idx].blinking)
    {
        LED_setState(idx, !led[idx].curr_state);  // invert LED current state

        // if LED state is back to original state and we are counting, we want to check if we should stop
        if (led[idx].curr_state == led[idx].state && led[idx].blinking_count)
        {
            // if counted to 0, we should either stop or if it is repeating code, we restart the code
            if (!--led[idx].blinking_count)
            {
                // Are we repeating a code?
                if (led[idx].blinking_repeat_code)
                {
                    // blinking the code, reload counter
                    led[idx].blinking_count = led[idx].blinking_repeat_code;
                    // give a long break before start another error code blinking
                    wiced_start_timer(&led[idx].blinking_timer, ERROR_CODE_BLINK_BREAK);
                }
                else // blinking stopped, set LED to original state
                {
                    LED_setState(idx, led[idx].state);
                }
                return;
            }
        }
        wiced_start_timer(&led[idx].blinking_timer, led[idx].blinking_duration);
    }
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
static void LED_set(uint8_t idx, wiced_bool_t on_off)
{
    if (VALID_LED_IDX(idx))
    {
        led[idx].state = on_off;
        if (!led_is_blinking(idx))
        {
            LED_setState(idx, on_off);
        }
    }
}

#if is_SDS_capable && (SLEEP_ALLOWED > 1)
/*******************************************************************************
 * Function Name: led_get_states
 ********************************************************************************
 * Summary:
 *    This function is called before entering SDS. It returns all LED states
 *
 * Parameters:
 *    none
 *
 * Return:
 *    LED states
 *
 *******************************************************************************/
uint8_t led_get_states()
{
    uint8_t state=0;

    // initialize LED based on platform defines
    for (int idx=0;idx<led_cfg.count;idx++)
    {
        if (led[idx].state)
        {
            state |= (1<<idx);
        }
    }
    return state;
}

/*******************************************************************************
 * Function Name: led_set_states
 ********************************************************************************
 * Summary:
 *    This function is called when waking up from SDS. It restores LED states
 *
 * Parameters:
 *    All led states
 *
 * Return:
 *    none
 *
 *******************************************************************************/
void led_set_states(uint8_t state)
{
    // set led states
    for (int idx=0;idx<led_cfg.count;idx++)
    {
        led[idx].state = state & (1<<idx) ? 1 : 0;
        LED_setState(idx, led[idx].state);
    }
}
#endif

/*******************************************************************************
 * Function Name: led_is_blinking
 ********************************************************************************
 * Summary:
 *    Return true if the requested LED is blinking.
 *
 * Parameters:
 *    idx -- LED index
 *
 * Return:
 *    True when LED is blinking.
 *
 *******************************************************************************/
wiced_bool_t led_is_blinking(uint8_t idx)
{
    return VALID_LED_IDX(idx) ? wiced_is_timer_in_use(&led[idx].blinking_timer) : FALSE;
}

/*******************************************************************************
 * Function Name: led_blink_stop
 ********************************************************************************
 * Summary:
 *    Stop LED blinking and set LED back to ON or OFF state when it started to blink.
 *
 * Parameters:
 *    idx -- LED index
 *
 * Return:
 *    None
 *
 *******************************************************************************/
void led_blink_stop(uint8_t idx)
{
    APP_LED_TRACE("stop blink led idx %d", idx);
    if (VALID_LED_IDX(idx))
    {
        if (led_is_blinking(idx))
        {
            led[idx].blinking_repeat_code = 0;
            wiced_stop_timer(&led[idx].blinking_timer);
        }
        led[idx].blinking = 0;
        LED_setState(idx, led[idx].state);
    }
}

/*******************************************************************************
 * Function Name: led_off
 ********************************************************************************
 * Summary:
 *    Turn off LED
 *
 * Parameters:
 *    idx -- LED index
 *
 * Return:
 *    None
 *
 *******************************************************************************/
void led_off(uint8_t idx)
{
    LED_set(idx, LED_OFF);
}

/*******************************************************************************
 * Function Name: led_on
 ********************************************************************************
 * Summary:
 *    Turn on LED
 *
 * Parameters:
 *    idx -- LED index
 *
 * Return:
 *    None
 *
 *******************************************************************************/
void led_on(uint8_t idx)
{
    LED_set(idx, LED_ON);
}

/*******************************************************************************
 * Function Name: led_blink
 ********************************************************************************
 * Summary:
 *    Blink the LED
 *
 * Parameters:
 *    idx -- LED index
 *    count -- how many times to blink, use value 0 to blink forever
 *    how_fast_in_ms -- the LED on/off duration in ms
 *
 * Return:
 *    None
 *
 *******************************************************************************/
void led_blink(uint8_t idx, uint32_t count, uint32_t how_fast_in_ms)
{
    APP_LED_TRACE("START blink led idx %d", idx);
    if (VALID_LED_IDX(idx))
    {
        led_blink_stop(idx);
        led[idx].blinking = 1;
        led[idx].blinking_duration = how_fast_in_ms;
        led[idx].blinking_count = count;
        LED_blink_handler(idx);
    }
}

/*******************************************************************************
 * Function Name: led_blink_code
 ********************************************************************************
 * Summary:
 *    Blink an error code
 *
 * Parameters:
 *    idx -- LED index
 *    code -- error code. The code is the fast blink count followed by long off. This repeats forever.
 *
 * Return:
 *    None
 *
 *******************************************************************************/
void led_blink_code(uint8_t idx, uint8_t code)
{
    if (VALID_LED_IDX(idx))
    {
        led[idx].blinking_repeat_code = code;
        led_blink(idx, code, BLINK_CODE_SPEED);
    }
}

/*******************************************************************************
 * Function Name: led_init
 ********************************************************************************
 * Summary:
 *    Initialize LED module. This function must be called once first before using any of
 *    other LED functions. It uses LED configuration from Platform and initialize hardware
 *    to default state (LED off state)
 *
 *    If this is Power On Reset, it will blink first, index 0, LED 5 times to indicate LED is
 *    initialized and ready to use. For apps, this is also an indicator as system start up.
 *
 * Parameters:
 *    none
 *
 * Return:
 *    none
 *
 *******************************************************************************/
void led_init(uint8_t count, const wiced_platform_led_config_t * cfg)
{
    APP_LED_TRACE("%s count=%d", __FUNCTION__, count);

    // if LED is defined in platform
    if (count && cfg && (count <= WICED_PLATFORM_LED_MAX))
    {
        led_cfg.count = count;
        led_cfg.platform = cfg;

        // initialize LED based on platform defines
        for (int idx=0;idx<count;idx++)
        {
            wiced_init_timer( &led[idx].blinking_timer, LED_blink_handler, idx, WICED_MILLI_SECONDS_TIMER );
            if (wiced_hal_mia_is_reset_reason_por())
            {
                APP_LED_TRACE("LED%d: GPIO:%d CFG:%04x default:%d", idx, *cfg[idx].gpio, cfg[idx].config, cfg[idx].default_state);
                LED_set(idx, LED_OFF); // default to turn LED off
            }
            wiced_hal_gpio_slimboot_reenforce_cfg(*led_cfg.platform[idx].gpio, GPIO_OUTPUT_ENABLE);
        }

        if (wiced_hal_mia_is_reset_reason_por())
        {
            led_blink(HIDD_LED, HIDD_START_UP_BLINK_COUNT, HIDD_START_UP_BLINK_SPEED);    // start up indicator
        }
    }
}

#endif // LED_SUPPORT > 0
