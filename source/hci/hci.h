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
 * HCI hidd handling routines
 *
 */
#ifndef BLE_HID_HCI_H__
#define BLE_HID_HCI_H__

typedef unsigned char uint8_t;
typedef unsigned short uint16_t;

#define DEVICE_CAPABILITY_LEN 3

#ifdef TESTING_USING_HCI

#include "wiced_bt_dev.h"
#include "wiced_transport.h"
#include "hci_v.h"

#define MAX_SEND_HCI_SAMPLE_CNT 126    // max only can send 126 samples each transaction
#define MAX_SEND_HCI_BYTE_CNT (MAX_SEND_HCI_SAMPLE_CNT*2)

typedef void (*hidd_app_hci_key_callback_t ) (uint8_t key, wiced_bool_t pressed);

void hci_control_init();
void hci_control_set_capability(char audio, char mouse, char ir);
void hci_control_send_pairing_complete_evt( uint8_t result, uint8_t *p_bda, uint8_t type );
void hci_control_send_disconnect_evt( uint8_t reason, uint16_t con_handle );
void hci_control_send_connect_evt( uint8_t addr_type, BD_ADDR addr, uint16_t con_handle, uint8_t role );
void hci_control_send_advertisement_state_evt( uint8_t state );
void hci_control_send_paired_host_info();
void hci_control_send_state_change( uint8_t transport, uint8_t state );
void hci_control_enable_trace();
void hci_control_register_key_handler(hidd_app_hci_key_callback_t key_handler);
void hci_control_transport_status( wiced_transport_type_t type );
void hci_control_send_audio_data_req();
uint32_t hci_dev_handle_command( uint8_t * p_data, uint32_t length );

 #define hci_control_send_data( code, buf, len ) wiced_transport_send_data( code, buf, len )
#else
 #define hci_control_init()
 #define hci_control_set_capability(a,m,i)
 #define hci_control_send_pairing_complete_evt( result, p_bda, type )
 #define hci_control_send_disconnect_evt( reason, con_handle )
 #define hci_control_send_connect_evt( addr_type, addr, con_handle, role )
 #define hci_control_send_advertisement_state_evt( state )
 #define hci_control_send_paired_host_info()
 #define hci_control_send_state_change( transport, state )
 #define hci_control_send_data( code, buf, len )
 #define hci_control_register_key_handler( handler )
 #define hci_control_send_audio_data_req()
 #ifdef HCI_TRACES_DUMP_TO_PUART
  void hci_control_enable_trace();
 #else
  #define hci_control_enable_trace()
 #endif
#endif
#endif
