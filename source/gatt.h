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
