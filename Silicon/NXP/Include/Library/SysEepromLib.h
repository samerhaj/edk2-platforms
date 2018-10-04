/** @file  SysEepromLib.h

  SystemID Non-Volatile Memory Device

  Copyright 2017 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution. The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/
#ifndef SYS_EEPROM_LIB_H__
#define SYS_EEPROM_LIB_H__

/**
  Read the MAC addresses from EEPROM

  This function reads the MAC addresses from EEPROM.
  This function must be called after relocation to ram.

  @param[in]  MacNo        Mac number whose address to be read from Eeprom
                           (Must be greater than zero)
  @param[out] MacAddress   Buffer of atleast 6 bytes to hold Mac address
                           (Must not be NULL)

  @retval EFI_INVALID_PARAMETER  Invalid MacNo or Null MacAddress
  @retval EFI_OUT_OF_RESOURCES   Not able to allocate memory for SYSTEM_ID
                                 structure.
  @retval EFI_CRC_ERROR          CRC checksum not mached
  @retval EFI_NOT_FOUND          MacAddress for MacNo not found
  @retval EFI_SUCCESS            Retrieved MacAddress successfully
**/
EFI_STATUS
EFIAPI
MacReadFromEeprom (
  IN   UINT8   MacNo,
  OUT  VOID    *MacAddress
  );

/**
  Generate an Ethernet address (MAC) that is not multicast
  and has the local assigned bit set.

  @param[in]  EthernetId   EthernetId for which Mac address is to be generated
                           Last two bytes (5th and 6th octet) of mac address are EthernetId
  @param[out] MacAddress   Buffer of at least 6 bytes to hold Mac address
                           (Must not be NULL)
 **/
EFI_STATUS
GenerateMacAddress (
  IN  UINT16     EthernetId,
  OUT VOID       *MacAddress
  );

#endif  /* SYS_EEPROM_LIB_H__ */

