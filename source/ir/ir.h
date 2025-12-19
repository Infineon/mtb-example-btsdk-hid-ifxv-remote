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
 * This file defines the interface of Infrared (IR) Transmit feature
 *
 */
#ifndef IR_H__
#define IR_H__

#define IR_TX_GPIO           WICED_P38
//#define IR_TX_GPIO           WICED_P15 // P38 not available on 62-pin package

#ifdef SUPPORT_IR
 #include "wiced.h"
 #include "wiced_irtx.h"
 void ir_init(BYTE gpio);
 void ir_start(uint8_t code, uint8_t repeat_count);
 void ir_stop();
 wiced_bool_t ir_button(uint8_t key, wiced_bool_t down);
 #define ir_is_active() (!wiced_irtx_isAvailable())
#else
 #define ir_init(gpio)
 #define ir_start(code, repeat)
 #define ir_stop()
 #define ir_button(key, down) FALSE
 #define ir_is_active() FALSE
#endif //SUPPORT_IR

#endif // IR_H__
