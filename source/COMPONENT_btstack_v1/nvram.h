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
