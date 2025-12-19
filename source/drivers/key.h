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
 * Keyscan Interface definitions
 *
 */
#ifndef KEY_H__
#define KEY_H__

#include "wiced.h"
#include "hidevent.h"
#include "kscan.h"

/// Maximum number of keys in a standard key report. Technically the report is
/// limited to 6 keys. An LE ATT can hold 23 bytes. We'll
/// only use 6. The length of a non-boot mode report will be set through the config
/// record
#define KEY_MAX_KEYS_IN_STD_REPORT    6

// Bit defines
enum bitmapped_key
{
    BITMAP_AC_SEARCH,       //#0, Voice
    BITMAP_AC_HOME,         //#1, Home
    BITMAP_AC_BACK,         //#2, Back
    BITMAP_FAST_FORWD,      //#3, Forward
    BITMAP_PLAY_PAUSE,      //#4, Play/Pause
    BITMAP_REWIND,          //#5, Rewind
    BITMAP_MENU,            //#6, Menu
    BITMAP_AL_AUDIO_PLAYER, //#7, Music

    BITMAP_MAX,
};

/// Maximum number of bytes in the bit-mapped key report structure.
#define KEY_NUM_BYTES_IN_BIT_MAPPED_REPORT     (((BITMAP_MAX-1)/8)+1)      // How many bytes to hold all bits defined

//define
#define KEY_NUM_BYTES_IN_USER_DEFINED_REPORT   8

extern uint8_t    kbrpt_ledStates;

///
/// Report structures must be packed
///
#pragma pack(1)
/// Standard key report structure
typedef PACKED struct
{
    /// Set to the value specified in the config record. 1 is recommended for boot-mode support
    uint8_t    reportID;

    /// Modifier keys
    uint8_t    modifierKeys;

    /// Reserved (OEM). Normally set to 0 unless changed by application code.
    uint8_t    reserved;

    /// Key array.
    uint8_t    keyCodes[KEY_MAX_KEYS_IN_STD_REPORT];
}KeyboardStandardReport;

/// Bit mapped key report structure
typedef PACKED struct
{
    /// Set to the value specified in the config record.
    uint8_t    reportID;

    /// Bit mapped keys
    uint8_t    bitMappedKeys[KEY_NUM_BYTES_IN_BIT_MAPPED_REPORT];
}KeyboardBitMappedReport;

/// User defined key report structure
typedef PACKED struct
{
    /// Set to the value specified in the config record.
    uint8_t    reportID;

    /// User defined keys
    uint8_t    userKeys[KEY_NUM_BYTES_IN_USER_DEFINED_REPORT];
}KeyboardUserDefinedReport;


/// Keyboard output report. Sets the LED state
typedef PACKED struct
{
    /// Set to the value specified in the config record.
    uint8_t    reportID;

    /// State of various LEDs
    uint8_t    ledStates;
}KeyboardLedReport;

typedef struct {
    /// Standard key report
    KeyboardStandardReport  stdRpt;

    /// Bit mapped key report
    KeyboardBitMappedReport bitMappedReport;

    /// User defined key report
    KeyboardUserDefinedReport userRpt;

}key_input_rpt_t;
#pragma pack()

extern key_input_rpt_t key_rpts;


#ifdef SUPPORT_KEY_REPORT
/*******************************************************************************
 * Function Name: void key_event(uint8_t keyCode, uint8_t keyDown)
 ********************************************************************************
 * Summary: similar to key_process_event() but not to check for special key function but just
 *          to generate key report
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
wiced_bool_t key_event(uint8_t keyCode, uint8_t keyDown);

/*******************************************************************************
 * Function Name: void key_process_event(uint8_t keyCode, uint8_t keyDown)
 ********************************************************************************
 * Summary: process key event, including check for special function, such as audio button.
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
wiced_bool_t key_process_event(uint8_t keyCode, uint8_t keyDown);

/*******************************************************************************
 * Function Name: void key_init
 ********************************************************************************
 * Summary: initialize key report
 *
 * Parameters:
 *  row, col -- key matrix row & col dimention
 *  poll_callback_t * pcb -- application poll function pointer to poll user activities
 *  key_detected_callback_t * cb -- application callback function pointer to handle key event
 *
 * Return:
 *  none
 *
 *******************************************************************************/
#define key_init(r,c,cb) kscan_init(r,c,cb)

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
void key_send();

/*******************************************************************************
 * Function Name: void key_clear(void)
 ********************************************************************************
 * Summary: clear all key reports
 *
 * Parameters:
 *  none
 *
 * Return:
 *  none
 *
 *******************************************************************************/
void key_clear(wiced_bool_t sendRpt);

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
void key_sendRollover();

#else
 #define key_process_event(c,d) TRUE
 #define key_init()
 #define key_send()
 #define key_clear(s)
 #define key_sendRollover();
 #define key_setReport(t,r,p,s) FALSE

#endif // SUPPORT_KEY_REPORT
#endif // KEY_H__
