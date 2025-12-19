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
#ifndef KEYSCAN_H__
#define KEYSCAN_H__

#ifdef SUPPORT_KEYSCAN
#include "wiced.h"
#include "wiced_hal_keyscan.h"
#include "hidevent.h"

typedef void (key_detected_callback_t) (uint8_t code, uint8_t down);
typedef void (kscan_poll_callback_t) (void *);

#define keyscanActive() (wiced_hal_keyscan_is_any_key_pressed() || wiced_hal_keyscan_events_pending())

/*******************************************************************************
 * Function Name: void kscan_init(uint8_t row, uint8_t col, key_detected_callback_t * cb)
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
void kscan_init(uint8_t row, uint8_t col, key_detected_callback_t * cb);

/*******************************************************************************
 * Function Name: void kscan_shutdown(void)
 ********************************************************************************
 * Summary: disable keyscan and getting ready for shutdown
 *
 * Parameters:
 *  None
 *
 * Return:
 *  None
 *
 *******************************************************************************/
#define kscan_shutdown() wiced_hal_keyscan_turnOff()

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
void kscan_reset(void);

/*******************************************************************************
 * Function Name: void kscan_enable_ghost_detection(void)
 ********************************************************************************
 * Summary: Resets keyscan and getting ready key events
 *
 * Parameters:
 *  wiced_bool_t enable : TRUE enable ghost key detection
 *                      : FALSE disable ghost key dection
 *
 * Return:
 *  None
 *
 *******************************************************************************/
#define kscan_enable_ghost_detection(e) wiced_hal_keyscan_enable_ghost_detection(e)

/*******************************************************************************
 * Function Name: void kscan_is_any_key_pressed(void)
 ********************************************************************************
 * Summary: Return TRUE if any key is pressed down
 *
 * Parameters:
 *  None
 *
 * Return:
 *  TRUE - any key is pressed down
 *  FALSE - keyscan is idle
 *
 *******************************************************************************/
#define kscan_is_any_key_pressed() wiced_hal_keyscan_is_any_key_pressed()

#else
 #define kscan_init(r,c,cb)
 #define kscan_shutdown()
 #define kscan_reset()
 #define kscan_enable_ghost_detection(e)
 #define kscan_is_any_key_pressed() FALSE
 #define keyscanActive() FALSE
#endif // SUPPORT_KEYSCAN
#endif // KEYSCAN_H__
