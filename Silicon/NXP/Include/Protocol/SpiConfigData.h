/** @file

 Copyright 2018 NXP

 This program and the accompanying materials
 are licensed and made available under the terms and conditions of the BSD License
 which accompanies this distribution.  The full text of the license may be found at
 http://opensource.org/licenses/bsd-license.php

 THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
 WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

 **/

#ifndef _SPI_CONFIG_DATA_H_
#define _SPI_CONFIG_DATA_H_

#define SPI_NOR_FLASH_SIGNATURE  SIGNATURE_64('s', 'p', 'i', ' ', 'n', 'o', 'r', ' ')

typedef struct {
  ///
  /// Signature specifying that the Configuration data is for SPI NOR flash type peripheral
  ///
  UINT64                Signature;
  ///
  /// if the SPI flash is memory mapped device, i.e. we can read flash contents like we read ram contents, then
  /// this tells the base address of flash device.
  ///
  EFI_PHYSICAL_ADDRESS  DeviceBaseAddress;

  ///
  /// Page Write size in bytes
  ///
  UINT32                PageSize;

  ///
  /// Flash size in bytes
  ///
  UINT32                FlashSize;
} SPI_FLASH_CONFIGURATION_DATA;

#endif /*_SPI_CONFIG_DATA_H_*/

