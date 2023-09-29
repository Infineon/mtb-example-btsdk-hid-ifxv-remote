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
 * Keyscan Interface
 *
 */

#ifdef SUPPORT_KEY_REPORT

#include "app.h"
#include "usb_usage.h"

#if CHIP!=20829 && defined WICED_BT_TRACE_ENABLE
# undef WICED_BT_TRACE
# define WICED_BT_TRACE(format,...)   wiced_printf(NULL, 0, (format"\n"), ##__VA_ARGS__)
#endif

#if KEY_TRACE==1
 #define APP_KEY_TRACE WICED_BT_TRACE
 #define APP_KEY_TRACE2(...)
#elif KEY_TRACE==2
 #define APP_KEY_TRACE WICED_BT_TRACE
 #define APP_KEY_TRACE2 WICED_BT_TRACE
#else
 #define APP_KEY_TRACE(...)
 #define APP_KEY_TRACE2(...)
#endif

#define CODE_ROLLOVER 1

/// Keyboard Key Config
typedef PACKED struct
{
    /// Type of key, e.g. std key, modifier key, etc.
    uint8_t    type;

    /// Translation code. The actual value depend on the key type.
    ///     - For modifier keys, it  is a bit mask for the reported key
    ///     - For std key, it is the usage provided in the std key report
    ///     - For bit mapped keys, it is the row/col of the associated bit in the bit mapped report
    uint8_t    translationValue;
}KbKeyConfig;


/// Key types. Used to direct key codes to the relevant key processing function
enum KeyType
{
    /// Represents no key. This should not occur normally
    KEY_TYPE_NONE,

    /// Represents a standard key. The associated translation code represents the reported value
    /// of this key
    KEY_TYPE_STD,

    /// Represents a modifier key. The associated translation value indicates which bit
    /// in the modifier key mask is controlled by this key
    KEY_TYPE_MODIFIER,

    /// Represents a bit mapped key in the bit mapped report. The associated translation value
    /// provides the row col of the bit which represents this key
    KEY_TYPE_BIT_MAPPED,

    /// A user defined key. Interpretation is provided by user code
    KEY_TYPE_USER_0,

    /// A user defined key. Interpretation is provided by user code
    KEY_TYPE_MAX
};

/*****************************************************************************
/// Key translation table. It maps keyscan matrix position to key types and
/// specific usage within the type. For example, row 5, column 6 may be
/// mapped as a standard key with usage "ESCAPE". This means that the key
/// will be reported in the standard report with a USB usage of "ESCAPE"
/// See config documentation for details and the keyboard config for an example.
/// By default this table is initialized for the BCM keyboard
*****************************************************************************/
KbKeyConfig kbKeyConfig[] =
{
    // Matrix 7x3 --> Col:7(0..6) Row:3(0..2)

    // Column 0:  order is row0 -> row2
    {KEY_TYPE_NONE,         0},                     //#0, n/c --> Virtually used for Connect
    {KEY_TYPE_BIT_MAPPED,   BITMAP_AC_SEARCH},      //#1, Voice
    {KEY_TYPE_STD,          USB_USAGE_POWER},       //#2, Power

    // Column 1: order is row0 -> row2
    {KEY_TYPE_NONE,         0},                     //#3, n/c
    {KEY_TYPE_STD,          USB_USAGE_UP_ARROW},    //#4, Up
    {KEY_TYPE_NONE,         0},                     //#5, n/c

    // Column 2: order is row0 -> row2
    {KEY_TYPE_STD,          USB_USAGE_RIGHT_ARROW}, //#6, Right
    {KEY_TYPE_STD,          USB_USAGE_ENTER},       //#7, Select
    {KEY_TYPE_STD,          USB_USAGE_LEFT_ARROW},  //#8, Left

    // Column 3: order is row0 -> row2
    {KEY_TYPE_NONE,         0},                     //#9,  n/c
    {KEY_TYPE_STD,          USB_USAGE_DOWN_ARROW},  //#10, Down
    {KEY_TYPE_NONE,         0},                     //#11, n/c

    // Column 4: order is row0 -> row2
    {KEY_TYPE_BIT_MAPPED,   BITMAP_MENU},           //#12, Menu
    {KEY_TYPE_BIT_MAPPED,   BITMAP_AC_HOME},        //#13, Home
    {KEY_TYPE_BIT_MAPPED,   BITMAP_AC_BACK},        //#14, Back

    // Column 5: order is row0 -> row2
    {KEY_TYPE_BIT_MAPPED,   BITMAP_FAST_FORWD},     //#15, Forward
    {KEY_TYPE_BIT_MAPPED,   BITMAP_PLAY_PAUSE},     //#16, Play/Pause
    {KEY_TYPE_BIT_MAPPED,   BITMAP_REWIND},         //#17, Rewind

    // Column 6: order is row0 -> row2
    {KEY_TYPE_STD,          USB_USAGE_VOL_DOWN},    //#18, Vol Down
    {KEY_TYPE_STD,          USB_USAGE_VOL_UP},      //#19, Vol Up
    {KEY_TYPE_STD,          USB_USAGE_MUTE},        //#20, Mute

};

#define KEY_TABLE_SIZE (sizeof(kbKeyConfig)/sizeof(KbKeyConfig))

//////////////////////////////////////////////////////////////////////////////
typedef struct {

    uint8_t                 stdRpt_changed:1;
    uint8_t                 bitMapped_changed:1;

} kbrpt_t;
static kbrpt_t keyRpt;

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////


uint8_t kbrpt_ledStates;

key_input_rpt_t key_rpts = {
    .stdRpt          = {RPT_ID_IN_STD_KEY},
    .bitMappedReport = {RPT_ID_IN_BIT_MAPPED},
};

/*******************************************************************************
 * Function Name: void KeyRpt_stdRptProcEvtKeyDown(uint8_t translationCode)
 ********************************************************************************
 * Summary: add the key to standard key report
 *
 * Parameters:
 *  key -- defined in USB
 *
 * Return:
 *  none
 *
 *******************************************************************************/
static void KeyRpt_stdRptProcEvtKeyDown(uint8_t key)
{
    uint8_t i;
    uint8_t * keyCodes = key_rpts.stdRpt.keyCodes;

    // Check if the key is already in the report
    for (i=0; keyCodes[i] && (i < KEY_MAX_KEYS_IN_STD_REPORT); i++)
    {
        if (keyCodes[i] == key)
        {
            // Already in the report. Ignore the event
            return;
        }
    }

    // Check if the std report has room
    if (i < KEY_MAX_KEYS_IN_STD_REPORT)
    {
        // Add the new key to the report
        keyCodes[i] = key;

        // Flag that the standard key report has changed
        keyRpt.stdRpt_changed = TRUE;
    }
}

/*******************************************************************************
 * Function Name: void KeyRpt_stdRptProcEvtKeyUp(uint8_t translationCode)
 ********************************************************************************
 * Summary: remove the code from standard key report
 *
 * Parameters:
 *  key -- defined in USB
 *
 * Return:
 *  none
 *
 *******************************************************************************/
static void KeyRpt_stdRptProcEvtKeyUp(uint8_t key)
{
    uint8_t i;
    uint8_t * keyCodes = key_rpts.stdRpt.keyCodes;

    // Find the key in the current standard report
    for (i=0; keyCodes[i] && i < KEY_MAX_KEYS_IN_STD_REPORT; i++)
    {
        if (keyCodes[i] == key)
        {
            // Found it. Remove it by shifting it!
            do
            {
                keyCodes[i] = keyCodes[i+1];
                // over sized? if so, we copied junk
                if (++i == KEY_MAX_KEYS_IN_STD_REPORT)
                {
                    keyCodes[--i] = 0; // replace junk data with 0
                }
            }
            while (keyCodes[i]);
            keyRpt.stdRpt_changed = TRUE;
        }
    }
}

/*******************************************************************************
 * Function Name: wiced_bool_t KeyRpt_updateBit(uint8_t *buf, uint8_t set, uint8_t bitMask)
 ********************************************************************************
 * Summary: Set or Clear the bit identified in bitMask within the byte
 *
 * Parameters:
 *  down -- TRUE when key is down
 *  translationCode -- bit position defined in USB
 *
 * Return:
 *  TRUE -- bit is changed
 *  FALSE -- no change
 *
 *******************************************************************************/
static wiced_bool_t KeyRpt_updateBit(uint8_t *buf, uint8_t set, uint8_t bitMask)
{
    uint8_t bits = *buf;

    if (set)
    {
        *buf |= bitMask;
    }
    else
    {
        *buf &= ~bitMask;
    }
    return bits != *buf;
}

/*******************************************************************************
 * Function Name: void KeyRpt_stdRptProcEvtModKey(uint8_t down, uint8_t translationCode)
 ********************************************************************************
 * Summary: handle modifier keys events
 *
 * Parameters:
 *  down -- TRUE when key is down
 *  translationCode -- bit position defined in USB
 *
 * Return:
 *  none
 *
 *******************************************************************************/
static void KeyRpt_stdRptProcEvtModKey(uint8_t set, uint8_t translationCode)
{
    // set or reset the bit
    if (KeyRpt_updateBit(&key_rpts.stdRpt.modifierKeys, set, translationCode))
    {
        // Flag that the standard key report has changed
        keyRpt.stdRpt_changed = TRUE;
    }
}

/*******************************************************************************
 * Function Name: void KeyRpt_bitRptProcEvtKey(uint8_t down, uint8_t bitPos)
 ********************************************************************************
 * Summary: handle modifier keys events
 *
 * Parameters:
 *  set -- TRUE then set the bit, otherwise, clear the bit
 *  translationCode -- bit position defined in USB
 *
 * Return:
 *  none
 *
 *******************************************************************************/
static void KeyRpt_bitRptProcEvtKey(uint8_t set, uint8_t bitPos)
{
    uint8_t idx = bitPos / 8;
    uint8_t bitMask = (1<< (bitPos % 8));

    APP_KEY_TRACE2("bitMap pos:%d set:%d, idx:%d, mask:%02x", bitPos, set, idx, bitMask);
    // set or reset the bit
    if (KeyRpt_updateBit(&key_rpts.bitMappedReport.bitMappedKeys[idx], set, bitMask))
    {
        APP_KEY_TRACE2("Data changed:%02x", key_rpts.bitMappedReport.bitMappedKeys[0]);
    }
    keyRpt.bitMapped_changed = TRUE;
}

/*******************************************************************************
 * Function Name: void KeyRpt_procEvtUserDefinedKey(void)
 ********************************************************************************
 * Summary: User defined key event handling
 *          Not used.
 *
 * Parameters:
 *  keyCode -- key index
 *  keyDown -- key up or down
 *
 * Return:
 *  none
 *
 *******************************************************************************/
static void KeyRpt_procEvtUserDefinedKey(uint8_t down, uint8_t translationCode)
{
    // User define key not implemented
}

/*******************************************************************************
 * Function Name: void key_keyEvent(void)
 ********************************************************************************
 * Summary: This function will generate HID key report
 *
 * Parameters:
 *  keyCode -- key index
 *  keyDown -- key up or down
 *
 * Return:
 *  TRUE -- handled correctly
 *  FALSE -- error detected
 *
 *******************************************************************************/
wiced_bool_t key_event(uint8_t keyCode, uint8_t keyDown)
{
    WICED_BT_TRACE("%s  code=%d, upDown=%d", __FUNCTION__, keyCode, keyDown);
    // Check if we have a valid key
    if (keyCode < KEY_TABLE_SIZE)
    {
        uint8_t keyValue = kbKeyConfig[keyCode].translationValue;

        // Depending on the key type, call the appropriate function for handling
        // Pass unknown key types to user function
        switch(kbKeyConfig[keyCode].type)
        {
            case KEY_TYPE_STD:
                // Processing depends on whether the event is an up or down event
                keyDown ? KeyRpt_stdRptProcEvtKeyDown(keyValue) : KeyRpt_stdRptProcEvtKeyUp(keyValue);
                break;
            case KEY_TYPE_MODIFIER:
                KeyRpt_stdRptProcEvtModKey(keyDown, keyValue);
                break;
            case KEY_TYPE_BIT_MAPPED:
                KeyRpt_bitRptProcEvtKey(keyDown, keyValue);
                break;
            case KEY_TYPE_NONE:
                // do nothing
                break;
            default:
                KeyRpt_procEvtUserDefinedKey(keyDown, keyValue);
                break;
        }
        key_send();
    }
    // Check if we have an end of scan cycle event
    else if (keyCode != END_OF_SCAN_CYCLE)
    {
        // key index is out of range
        return FALSE;
    }
    return TRUE;
}

/*******************************************************************************
 * Function Name: void key_keyEvent(void)
 ********************************************************************************
 * Summary: key event handling
 *
 * Parameters:
 *  keyCode -- key index
 *  keyDown -- key up or down
 *
 * Return:
 *  TRUE -- handled correctly
 *  FALSE -- error detected
 *
 *******************************************************************************/
wiced_bool_t key_process_event(uint8_t keyCode, uint8_t keyDown)
{
    if (keyCode == AUDIO_KEY_INDEX)
    {
        audio_button(keyDown);
    }
    else
    {
        return key_event(keyCode, keyDown);
    }
    return TRUE;
}

/*******************************************************************************
 * Function Name: void key_send(void)
 ********************************************************************************
 * Summary: Send any pending key reports.
 *
 * Parameters:
 *  none
 *
 * Return:
 *  none
 *
 *******************************************************************************/
void key_send()
{
    if (keyRpt.stdRpt_changed)
    {
        APP_KEY_TRACE2("Send STD key: %A", &key_rpts.stdRpt, sizeof(KeyboardStandardReport));
        hidd_send_report(&key_rpts.stdRpt, sizeof(KeyboardStandardReport));
        keyRpt.stdRpt_changed = FALSE;
    }
    if (keyRpt.bitMapped_changed)
    {
        APP_KEY_TRACE2("Bitmap key: %A", &key_rpts.bitMappedReport, sizeof(KeyboardBitMappedReport));
        hidd_send_report(&key_rpts.bitMappedReport, sizeof(KeyboardBitMappedReport));
        keyRpt.bitMapped_changed = FALSE;
    }
}

/*******************************************************************************
 * Function Name: void key_clear(wiced_bool_t sendRpt)
 ********************************************************************************
 * Summary: Clear all reports to default value
 *
 * Parameters:
 *  sendRpt -- When TRUE, after clear, send standard report (to indicate all keys are up)
 *
 * Return:
 *  none
 *
 *******************************************************************************/
void key_clear(wiced_bool_t sendRpt)
{
    // clear report data
    memset(&key_rpts.stdRpt.modifierKeys, 0, sizeof(KeyboardStandardReport)-1);
    memset(key_rpts.bitMappedReport.bitMappedKeys, 0, sizeof(KeyboardBitMappedReport)-1);

    keyRpt.stdRpt_changed = keyRpt.bitMapped_changed = sendRpt;

    key_send();
}

/*******************************************************************************
 * Function Name: void key_sendRollover()
 ********************************************************************************
 * Summary: Send a rollover packet
 *
 * Parameters:
 *  none
 *
 * Return:
 *  none
 *
 *******************************************************************************/
void key_sendRollover()
{
    KeyboardStandardReport  rolloverRpt = {RPT_ID_IN_STD_KEY, 0, 0, {CODE_ROLLOVER, CODE_ROLLOVER, CODE_ROLLOVER, CODE_ROLLOVER, CODE_ROLLOVER, CODE_ROLLOVER}};
    // Tx rollover report
    WICED_BT_TRACE("RollOverRpt");
    hidd_send_report(&rolloverRpt, sizeof(KeyboardStandardReport));
}

#endif // SUPPORT_KEY_REPORT
