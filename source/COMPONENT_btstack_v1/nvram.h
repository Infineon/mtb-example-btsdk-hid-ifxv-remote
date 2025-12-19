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
#ifndef NVRAM_V1_H__
#define NVRAM_V1_H__

#include "wiced_bt_types.h"
#include "wiced_hal_nvram.h"

/******************************************************************************
 * macro defines
 ******************************************************************************/
#if SFI_DEEP_SLEEP
 extern uint8_t pmu_attemptSleepState;
 extern void sfi_enter_deep_power_down(void);
 extern void sfi_exit_deep_power_down(BOOL8 forceExitDeepPowerDown);
 extern void sfi_allow_deep_sleep(void);
 #define nvram_allow_deep_sleep() sfi_allow_deep_sleep();
 #define nvram_enter_deep_sleep() sfi_enter_deep_power_down()
 #define nvram_exit_deep_sleep(t) sfi_exit_deep_power_down(t)
 #define nvram_deep_sleep() {sfi_allow_deep_sleep(); sfi_enter_deep_power_down();}
 #define nvram_init(c) nvram_deep_sleep()
#else
 #define nvram_allow_deep_sleep()
 #define nvram_enter_deep_sleep()
 #define nvram_exit_deep_sleep()
 #define nvram_deep_sleep()
 #define nvram_init(c)
#endif

/******************************************************************************
 * Type Definitions
 ******************************************************************************/
/////////////////////////////////////////////////////////////////////////////////
// NVRAM ID defines
/////////////////////////////////////////////////////////////////////////////////
enum {
    VS_ID_LOCAL_IDENTITY = WICED_NVRAM_VSID_START,
    VS_ID_HIDD_HOST_LIST,
};


/******************************************************************************
 * externs
 ******************************************************************************/


/******************************************************************************
 * functions
 ******************************************************************************/

/** Reads the data from NVRAM
 *
 * @param[in]  vs_id       : Volatile Section Identifier. Application can use
 *                           the VS ids from WICED_NVRAM_VSID_START to
 *                           WICED_NVRAM_VSID_END
 *
 * @param[in]  data_length : Length of the data to be read from NVRAM
 *
 * @param[out] p_data      : Pointer to the buffer to which data will be copied
 *
 * @return  TRUE when when reads okay
 */
wiced_bool_t nvram_read( uint16_t vs_id, uint8_t * p_data, uint16_t data_length);

/**
 * Writes the data to NVRAM,
 * Application can write up to 255 bytes in one VS  id section
 *
 * @param[in] vs_id        : Volatile Section Identifier. Application can use
 *                           the VS ids from WICED_NVRAM_VSID_START to
 *                           WICED_NVRAM_VSID_END
 *
 * @param[in] data_length  : Length of the data to be written to the NVRAM,
 *
 * @param[in] p_data       : Pointer to the data to be written to the NVRAM
 *
 * @return  TRUE when writes okay
 */
wiced_bool_t nvram_write( uint16_t vs_id, uint8_t * p_data, uint16_t data_length);

/*
 * Not used in stack_v1
 */

#endif // NVRAM_V1_H__
