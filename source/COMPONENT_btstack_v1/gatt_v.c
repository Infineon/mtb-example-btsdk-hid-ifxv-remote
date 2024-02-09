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
 * GATT callback function and handlers
 *
 */

#include "app.h"

#if CHIP!=20829 && defined WICED_BT_TRACE_ENABLE
# undef WICED_BT_TRACE
# define WICED_BT_TRACE(format,...)   wiced_printf(NULL, 0, (format"\n"), ##__VA_ARGS__)
#endif

#define APP_GATT_TRACE0        WICED_BT_TRACE
#if GATT_TRACE
# define APP_GATT_TRACE        WICED_BT_TRACE
# if GATT_TRACE>1
#  define APP_GATT_TRACE2      WICED_BT_TRACE
#  if GATT_TRACE>2
#   define APP_GATT_TRACE3     WICED_BT_TRACE
#  else
#   define APP_GATT_TRACE3(...)
#  endif
# else
#  define APP_GATT_TRACE2(...)
#  define APP_GATT_TRACE3(...)
# endif
#else
# define APP_GATT_TRACE(...)
# define APP_GATT_TRACE2(...)
# define APP_GATT_TRACE3(...)
#endif

wiced_bt_gatt_status_t gatt_v_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t * p_data)
{
    APP_GATT_TRACE("GATT: unhandled event %d !!!", event);
    return WICED_BT_GATT_SUCCESS;
}

/*
 * Process Read request or command from peer device
 */
wiced_bt_gatt_status_t gatt_req_read_default_handler( uint16_t conn_id, wiced_bt_gatt_read_t * p_read_data )
{
    const gatt_db_lookup_table_t * p_attribute;
    wiced_bt_gatt_status_t result;
    uint16_t attr_len_to_copy;
    uint8_t *attr_val_ptr = NULL;

    APP_GATT_TRACE3("In gatt_req_read_default_handler");
    if(!p_read_data)
    {
        return WICED_BT_GATT_ERROR;
    }

    // default to invalid handle
    result = WICED_BT_GATT_INVALID_HANDLE;

    APP_GATT_TRACE3("GATT Read - conn %d hdl 0x%x", conn_id, p_read_data->handle );

    p_attribute = hidd_get_attribute(p_read_data->handle);
    if(p_attribute)
    {
        uint16_t mtu;

        APP_GATT_TRACE3("Found handle hdl 0x%x", p_read_data->handle );
        //check if this is read request is for a long attribute value, if so take care of the offset as well
        if(p_read_data->is_long)
        {
            if (p_read_data->offset >= p_attribute->max_len)
            {
                return WICED_BT_GATT_INVALID_OFFSET;
            }
            else
            {
                attr_val_ptr = (uint8_t *) p_attribute->p_data + p_read_data->offset;
                attr_len_to_copy = p_attribute->max_len - p_read_data->offset;
            }
        }
        else
        {
            attr_val_ptr = (uint8_t *) p_attribute->p_data;
            attr_len_to_copy = p_attribute->max_len;
        }

        APP_GATT_TRACE3("attr_len_to_copy: %d offset: %d", attr_len_to_copy, p_read_data->offset);

        if(attr_len_to_copy<*p_read_data->p_val_len)
        {
            // report back our length
            *p_read_data->p_val_len = attr_len_to_copy;
        }

        mtu = wiced_blehidd_get_att_mtu_size(host_addr());
        APP_GATT_TRACE3("attr_len_to_copy: %d offset: %d, mtu is %d", attr_len_to_copy, p_read_data->offset, mtu);

        //make sure copying buff is large enough so it won't corrupt memory
        if(attr_len_to_copy >= mtu)
        {
            APP_GATT_TRACE3("size(%d) > mtu(%d)", attr_len_to_copy, mtu);
            attr_len_to_copy = mtu - 1;
        }
        APP_GATT_TRACE3("attr_len_to_copy: %d offset: %d", attr_len_to_copy, p_read_data->offset);

        // copy over the value to the supplied buffer(entirely if it fits, data worth of MTU size)
        // if we have only sent partial value of an attribute we expect the peer to issue a read blob request to get the
        // rest of the attribute value.
        memcpy( p_read_data->p_val, attr_val_ptr, attr_len_to_copy );
        APP_GATT_TRACE("GATT Read - handle 0x%04x, len=%d, offset=%d -- %A", p_read_data->handle, attr_len_to_copy, p_read_data->offset, attr_val_ptr, attr_len_to_copy);

        result = WICED_BT_GATT_SUCCESS;
    }
    else
    {
        WICED_BT_TRACE("GATT Read - attribute not found, handle: %0x", p_read_data->handle);
    }
    return result;
}

/*
 *
 */
wiced_bt_gatt_status_t gatt_req_cb( wiced_bt_gatt_attribute_request_t *p_data )
{
    wiced_bt_gatt_status_t result = WICED_BT_GATT_SUCCESS;

    switch ( p_data->request_type )
    {
        case GATTS_REQ_TYPE_READ:
            APP_GATT_TRACE2("Req READ handle:%04x len:%d ", p_data->data.read_req.handle, *p_data->data.read_req.p_val_len);
            result = app_gatt_read_req_handler(p_data->conn_id, &(p_data->data.read_req));
            break;

        case GATTS_REQ_TYPE_WRITE:
        case GATTS_REQ_TYPE_PREP_WRITE:
            APP_GATT_TRACE2("Req PREP_WRITE or WRITE handle:%04x len:%d -- %A", p_data->data.write_req.handle, p_data->data.write_req.val_len, p_data->data.write_req.p_val, p_data->data.write_req.val_len);
            result = app_gatt_write_handler(p_data->conn_id, &(p_data->data.write_req));
            break;

        case GATTS_REQ_TYPE_MTU:
            APP_GATT_TRACE("Req MTU to %d bytes", p_data->data.mtu);
            break;

        case GATTS_REQ_TYPE_CONF:
            result = gatt_req_conf_handler( p_data->conn_id, p_data->data.handle );
            break;

        case GATTS_REQ_TYPE_WRITE_EXEC:
             APP_GATT_TRACE2("Req WRITE_EXEC %d - not handled", p_data->data.exec_write);
            break;

        default:
            APP_GATT_TRACE2("Check not handled req type %d", p_data->request_type);
            break;
    }

//    hidd_deep_sleep_not_allowed(1000);// No deep sleep for 1 second.

    return result;
}
