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

#ifndef GATT_H__
#define GATT_H__

#include "gatt_v.h"
#include "cycfg_gatt_db.h"

#define HIDD_HANDLE_BEGIN  HDLS_HIDS
#define HIDD_HANDLE_END    HDLD_HIDS_IN_RPT_USER_REPORT_REFERENCE
#define is_hidd_handle(h)  ((h >= HIDD_HANDLE_BEGIN) && (h <= HIDD_HANDLE_END))

#define BAS_HANDLE_BEGIN   HDLS_BAS
#define BAS_HANDLE_END     HDLD_BAS_BATTERY_LEVEL_REPORT_REFERENCE
#define is_bas_handle(h)   ((h >= BAS_HANDLE_BEGIN) && (h <= BAS_HANDLE_END))

typedef void (*gatt_congested_cb_t) (wiced_bool_t congested);

typedef struct {
    uint16_t             peer_mtu;
    wiced_bool_t         congested;
    gatt_congested_cb_t  congested_cb;
} gatt_t;

extern gatt_t gatt;

/***********************************************************
 * Function Name: gatt_write_default_handler
 ***********************************************************
 * Summary:
 *  Default handler to process write request or command from peer device.
 *  The event calls application gatt_read_req_handler first. When it is not
 *  handled in application, this default handler is called.
 *
 * Parameters:
 *  uint16_t conn_id
 *  wiced_bt_gatt_write_req_t * p_wr_data
 *
 * Return:
 *  wiced_bt_gatt_status_t
 ***********************************************************/
wiced_bt_gatt_status_t gatt_write_default_handler( uint16_t conn_id, wiced_bt_gatt_write_req_t * p_wr_data );

/***********************************************************
 * Function Name: gatt_initialize
 ***********************************************************
 * Summary:
 *  This function uses BT configurator generated source to
 *  initialize gatt database.
 *
 * Parameters:
 *  None
 *
 * Return:
 *  wiced_bt_gatt_status_t
 ***********************************************************/
wiced_bt_gatt_status_t gatt_initialize();

/*
 * gatt_get_att_mtu_size
 */
uint16_t gatt_get_att_mtu_size();

/*
 * Process indication confirm. If client wanted us to use indication instead of
 * notifications we have to wait for confirmation after every message sent.
 * For example if user pushed button twice very fast
 * we will send first message, then
 * wait for confirmation, then
 * send second message, then
 * wait for confirmation and
 * if configured start idle timer only after that.
 */
wiced_bt_gatt_status_t gatt_req_conf_handler( uint16_t conn_id, uint16_t handle );

void gatt_register_congested_callback( gatt_congested_cb_t cb );

#endif // GATT_H__
