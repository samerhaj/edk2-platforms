/** @file

 Copyright 2018 NXP.

 This program and the accompanying materials
 are licensed and made available under the terms and conditions of the BSD License
 which accompanies this distribution.  The full text of the license may be found at
 http://opensource.org/licenses/bsd-license.php

 THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
 WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

 **/

#ifndef _SPI_PLATFORM_CONFIG_LIB_H_
#define _SPI_PLATFORM_CONFIG_LIB_H_

#include <Pi/PiSpi.h>

typedef struct {
  ///
  /// if the SPI flash is memory mapped device, i.e. we can read flash contents like we read ram contents, then
  /// this tells the base address of flash device.
  ///
  EFI_PHYSICAL_ADDRESS  DeviceBaseAddress;

  /// if the SPI flash is memory mapped device, then this tells the start address of region, which we can program (write)
  /// the area between RegionBaseAddress - DeviceBaseAddress is protected region, which we cannot program.
  /// This address should be aligned to block size of flash device.
  EFI_PHYSICAL_ADDRESS  RegionBaseAddress;

  ///
  /// Page Write size in bytes
  ///
  UINT32                PageSize;
} SPI_CONFIGURATION_DATA;

///
/// Key to retrieve the SPI device's configuration data from configuration map
///
typedef struct {
  ///
  /// Device Path of SPI controller to which SPI device is attached
  ///
  EFI_SPI_DEVICE_PATH      ControllerPath;

  ///
  /// Chip select number of SPI device
  ///
  UINT32                   ChipSelect;
} SPI_PERIPHERAL_CONFIG_KEY;

///
/// A map containing key value pair.
///
typedef struct {
  ///
  /// Key to search Value in Map
  ///
  SPI_PERIPHERAL_CONFIG_KEY    Key;

  ///
  /// Value
  ///
  SPI_CONFIGURATION_DATA       Value;
} SPI_CONFIG_MAP;

/**
  Retrieve the Platform specific configuration data for a SPI device.

  @param[in] Key        Pointer to an SPI_PERIPHERAL_CONFIG_KEY data structure.
                        Key contains the SPI controller information as well as SPI device chip select number
  @param[out] Value     Pointer to an SPI_CONFIGURATION_DATA data structure.
                        The SPI_CONFIG_MAP is searched for the Key and corresponding Value is copied in
                        memory pointed to by Value. It is the responsibility of caller of this function to
                        allocate memory for SPI_CONFIGURATION_DATA.

  @retval EFI_SUCCESS             The configuration data was retrieved successfully.
  @retval EFI_NOT_FOUND           no configuration data found for the SPI device
  @retval EFI_INVALID_PARAMETER   either Key or Value is NULL
**/
EFI_STATUS
SpiGetPlatformConfigData (
  IN   SPI_PERIPHERAL_CONFIG_KEY    *Key,
  OUT  SPI_CONFIGURATION_DATA       *Value
);

#endif /*_SPI_PLATFORM_CONFIG_LIB_H_*/

