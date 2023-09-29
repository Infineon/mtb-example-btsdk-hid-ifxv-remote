/*
 * $ Copyright YEAR Cypress Semiconductor $
 */

/**
 * file host.h
 *
 * host management header file
 *
 * Abstract: This file defines an interface for managing LE host lists, i.e.
 * device address, link key, client configuration characteristic descriptor value
 */

#pragma once

#include "wiced_bt_dev.h"

#define HIDD_HOST_LIST_MAX 1
#define HOST_INFO_SIZE (1 + (HIDD_HOST_LIST_MAX * (BD_ADDR_LEN+1)))

/********************************************************************
 * Function Name: host_get_info
 ********************************************************************
 * Summary:
 *  get paired host info
 *  This function is used by Client Control for the paired host info
 *
 * Param:
 *  buf : pointer to the buffer to save the data
 *
 * Return
 *  data length
 ********************************************************************/
uint16_t host_get_info(uint8_t * buf);

/********************************************************************
 * Function Name: host_get_link_key
 ********************************************************************
 * Summary:
 *  get a copy of active link key data
 *
 * Return FALSE if bdaddr does not match
 ********************************************************************/
wiced_bool_t host_get_link_key(const wiced_bt_device_address_t bdAddr, wiced_bt_device_link_keys_t * link_key);

/********************************************************************
 * Function Name: host_set_link_key
 ********************************************************************
 * Summary:
 *  This function is called when the host is bonded and data will be
 *  saved into NVRAM database
 *
 *  if host does not exist in database, it creates and add to new host.
 *  if host already exist, replace current host and move to active host.
 *
 * param bdAddr:      host address to activate
 * param link_key:    host link_key
 * param transport:   host tranport type
 *
 ********************************************************************/
void host_set_link_key(const wiced_bt_device_address_t bdAddr, wiced_bt_device_link_keys_t * link_key, uint8_t transport);

/********************************************************************
 * Function Name: host_init
 ********************************************************************
 * Summary:
 *  Read HID host information from NVRAM VS section and initialize
 *  host list database.
 ********************************************************************/
void host_init();

/***********************************************************
 * Function Name: host_set_cccd_flags
 ***********************************************************
 * Summary:
 *  Set and save the CCCD flags to NVRAM. If the provided bdAddr does not exists
 *  in NVRAM, a new entry is created.
 *
 * Parameters:
 *   wiced_bt_device_address_t bdAddr -- device address
 *   uint16_t nflags  -- Notification flags
 *   uint16_t iflags -- Indication flags
 *
 ***********************************************************/
void host_set_cccd_flags(const wiced_bt_device_address_t bdAddr, uint16_t nflags, uint16_t iflags);

/***********************************************************
 * Function Name: host_get_cccd_flags
 ***********************************************************
 * Summary:
 *  This function gets CCCD flags stored in NVRAM. If the CCCD
 *  does not exists for the bdAddr, it returns FALSE and the
 *  nflags/iflags are untouched and undefined.
 *
 * Parameters:
 *  wiced_bt_device_address_t bdAddr -- device address
 *  uint16_t * nflags  -- Notification flags pointer
 *  uint16_t * iflags -- Indication flags pointer
 *
 * Return:
 *  TRUE if the CCCD exists for the bdAddr.
 *
 ***********************************************************/
wiced_bool_t host_get_cccd_flags(const wiced_bt_device_address_t bdAddr,
                                 uint16_t * nflags,
                                 uint16_t * iflags);

/***********************************************************
 * Function Name: host_restore_cccd_flags
 ***********************************************************
 * Summary:
 *  Find the bdAddr's CCCD from NVRAM. If exists, it restores
 *  to CCCD memory buffer. If it doesn't exists, then clears
 *  all memory buffer's CCCD flags.
 *
 * Parameters:
 *  wiced_bt_device_address_t bdAddr -- device address
 *
 * Return:
 *  none
 *
 ***********************************************************/
void host_restore_cccd_flags(const wiced_bt_device_address_t bdAddr);

/***********************************************************
 * Function Name: host_remove
 ***********************************************************
 * Summary:
 *  Delete the active host from the list
 ***********************************************************/
wiced_bool_t host_remove(void);

/***********************************************************
 * Function Name: host_remove_addr
 ***********************************************************
 * Summary:
 *  Delete the bd_addr host from the list
 ***********************************************************/
wiced_bool_t host_remove_addr(wiced_bt_device_address_t host_bd_addr);

/***********************************************************
 * Function Name: host_remove_all
 ***********************************************************
 * Summary:
 *  Delete all hosts from the list
 ***********************************************************/
void host_remove_all(void);

/***********************************************************
 * Function Name: host_addr
 ***********************************************************
 * Summary:
 *  return active host's bd_addr
 *   If no device is paired, it returns a pointer points to an
 *   address with all 00.
 ***********************************************************/
uint8_t * host_addr();

/***********************************************************
 * Function Name: host_exist
 ***********************************************************
 * Summary:
 *  return TRUE if given bd_addr is in the host list
 *
 * param host_bd_addr : The host address to search for
 *
 ***********************************************************/
wiced_bool_t host_exist(const wiced_bt_device_address_t host_bd_addr);

/***********************************************************
 * Function Name: host_count
 ***********************************************************
 * Summary:
 *  returns number of hosts paired
 ***********************************************************/
uint8_t host_count();

/***********************************************************
 * Function Name: host_is_paired
 ***********************************************************
 * Summary:
 *  returns TRUE if the device is paired
 ***********************************************************/
#define host_is_paired() host_count()

/***********************************************************
 * Function Name: host_addr_type
 ***********************************************************
 * Summary:
 *  returns the active host type for LE link.
 *  returns 0 if the link is not LE.
 ***********************************************************/
uint8_t host_addr_type();

/***********************************************************
 * Function Name: host_transport
 ***********************************************************
 * Summary:
 *  returns the active host transport type
 *
 * returns BT_TRANSPORT_LE
 *         BT_TRANSPORT_BR_EDR
 *         or 0 if not paired
 ***********************************************************/
wiced_bt_transport_t host_transport();


/* end of file */
