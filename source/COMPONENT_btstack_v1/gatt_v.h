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
#ifndef GATT_V1_H__
#define GATT_V1_H__

#include "cycfg_gatt_db.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_gatt_util.h"

/******************************************************************************
 * Macro Definitions
 ******************************************************************************/
#define gatt_db_init(data, len) wiced_bt_gatt_db_init( data, len  )

/******************************************************************************
 * Type Definitions
 ******************************************************************************/
typedef wiced_bt_gatt_write_t wiced_bt_gatt_write_req_t;
typedef wiced_bt_gatt_status_t (*hidd_gatt_req_read_callback_t)( uint16_t conn_id, wiced_bt_gatt_read_t * p_data );
typedef wiced_bt_gatt_status_t (*hidd_gatt_req_write_callback_t)( uint16_t conn_id, wiced_bt_gatt_write_req_t * p_data );

/******************************************************************************
 * functions
 ******************************************************************************/
wiced_bt_gatt_status_t gatt_req_cb( wiced_bt_gatt_attribute_request_t *p_data );

wiced_bt_gatt_status_t gatt_v_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t * p_data);

wiced_bt_gatt_status_t gatt_req_read_default_handler( uint16_t conn_id, wiced_bt_gatt_read_t * p_read_data );

#endif // GATT_V1_H__
