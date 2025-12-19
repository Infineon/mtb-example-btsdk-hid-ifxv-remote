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
#ifndef APP_BTSTACK_V1_H__
#define APP_BTSTACK_V1_H__

#include "wiced_bt_types.h"
#include "cycfg_bt_settings.h"
#include "wiced_hidd_lib.h"

#define MAX_MTU_SIZE                     251

#define WICED_TIMER_PARAM_TYPE TIMER_PARAM_TYPE

#ifdef WICED_EVAL
 #define RED_LED        WICED_PLATFORM_LED_2
 #define LINK_LED       WICED_PLATFORM_LED_1
#else
 #ifdef RED_LED
  #undef RED_LED
 #endif
 #define RED_LED        WICED_PLATFORM_LED_1
 #define LINK_LED       WICED_PLATFORM_LED_2
#endif

#define cfg_sec_mask() ( wiced_bt_cfg_settings.security_requirement_mask )

#define WICED_BTSTACK_VERSION_MAJOR WICED_SDK_MAJOR_VER
#define WICED_BTSTACK_VERSION_MINOR WICED_SDK_MINOR_VER
#define WICED_BTSTACK_VERSION_PATCH WICED_SDK_BUILD_NUMBER

void hidd_enable_interrupt(wiced_bool_t en);

wiced_bt_gatt_status_t app_gatt_read_req_handler( uint16_t conn_id, wiced_bt_gatt_read_t * p_read_data );

#endif // APP_BTSTACK_V1_H__
