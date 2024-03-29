/*
 * Copyright 2016-2024, Cypress Semiconductor Corporation (an Infineon company) or
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
 * This file defines the interface of OTA firmware upgrade service
 *
 */
#ifndef OTA_H__
#define OTA_H__

#ifdef OTA_FIRMWARE_UPGRADE
 #include "wiced.h"
 #include "wiced_bt_ota_firmware_upgrade.h"
 #define ota_is_active() wiced_ota_fw_upgrade_is_active()
 extern uint8_t  ota_fw_upgrade_initialized;
 #ifdef OTA_SECURE_FIRMWARE_UPGRADE
  #include "bt_types.h"
  #include "p_256_multprecision.h"
  #include "p_256_ecc_pp.h"

  // If secure version of the OTA firmware upgrade is used, the app should be linked with the ecdsa256_pub.c
  // which exports the public key
  extern Point ecdsa256_public_key;
  #define ECDSA256_PUBLIC_KEY &ecdsa256_public_key
 #else
  #define ECDSA256_PUBLIC_KEY NULL
 #endif // OTA_SECURE_FIRMWARE_UPGRADE
#else
 #define ota_is_active() FALSE
#endif

#endif // OTA_H__
