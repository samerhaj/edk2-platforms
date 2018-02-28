/** @file

 Copyright 2018 NXP

 This program and the accompanying materials
 are licensed and made available under the terms and conditions of the BSD License
 which accompanies this distribution.  The full text of the license may be found at
 http://opensource.org/licenses/bsd-license.php

 THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
 WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

 **/

#include <Library/SpiPlatformConfigLib.h>
#include <Library/BaseMemoryLib.h>

SPI_CONFIG_MAP mSpiConfigMap[] = {
  {
    .Key = {
      .ControllerPath = {
        .Vendor = {
          .Header = {
            .Type = HARDWARE_DEVICE_PATH,
            .SubType = HW_VENDOR_DP,
            .Length = {
              (UINT8)sizeof (VENDOR_DEVICE_PATH),
              (UINT8)((sizeof (VENDOR_DEVICE_PATH)) >> 8)
            }
          },
          .Guid = {0x889788dc, 0x5fde, 0x4dd8, {0xa7, 0x7d, 0x8f, 0xff, 0x11, 0x0d, 0x53, 0xd3}}
        },
        .Controller = {
          .Header = {
            .Type = HARDWARE_DEVICE_PATH,
            .SubType = HW_CONTROLLER_DP,
            .Length = {
              (UINT8) (sizeof (CONTROLLER_DEVICE_PATH)),
              (UINT8) ((sizeof (CONTROLLER_DEVICE_PATH)) >> 8)
            }
          },
          .ControllerNumber = 0  // Qspi0
        },
        .End = {
          .Type = END_DEVICE_PATH_TYPE,
          .SubType = END_ENTIRE_DEVICE_PATH_SUBTYPE,
          .Length = { sizeof (EFI_DEVICE_PATH_PROTOCOL), 0 }
        }
      },
      .ChipSelect = 0
    },
    .Value = {
      .DeviceBaseAddress = 0x40000000,
      .RegionBaseAddress = 0x40000000,
      .PageSize = 0x40
    }
  },
  {
    .Key = {
      .ControllerPath = {
        .Vendor = {
          .Header = {
            .Type = HARDWARE_DEVICE_PATH,
            .SubType = HW_VENDOR_DP,
            .Length = {
              (UINT8)sizeof (VENDOR_DEVICE_PATH),
              (UINT8)((sizeof (VENDOR_DEVICE_PATH)) >> 8)
            }
          },
          .Guid = {0x889788dc, 0x5fde, 0x4dd8, {0xa7, 0x7d, 0x8f, 0xff, 0x11, 0x0d, 0x53, 0xd3}}
        },
        .Controller = {
          .Header = {
            .Type = HARDWARE_DEVICE_PATH,
            .SubType = HW_CONTROLLER_DP,
            .Length = {
              (UINT8) (sizeof (CONTROLLER_DEVICE_PATH)),
              (UINT8) ((sizeof (CONTROLLER_DEVICE_PATH)) >> 8)
            }
          },
          .ControllerNumber = 0  // Qspi0
        },
        .End = {
          .Type = END_DEVICE_PATH_TYPE,
          .SubType = END_ENTIRE_DEVICE_PATH_SUBTYPE,
          .Length = { sizeof (EFI_DEVICE_PATH_PROTOCOL), 0 }
        }
      },
      .ChipSelect = 1
    },
    .Value = {
      .DeviceBaseAddress = 0x48000000,
      .RegionBaseAddress = 0x48000000,
      .PageSize = 0x40
    }
  }
};

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
)
{
  UINTN        Index;
  EFI_STATUS   Status;

  if (Key == NULL || Value == NULL) {
    return EFI_INVALID_PARAMETER;
  }

  Status = EFI_NOT_FOUND;
  for (Index = 0; Index < ARRAY_SIZE(mSpiConfigMap); Index++) {
    if (CompareMem (&(mSpiConfigMap[Index].Key), Key, sizeof (SPI_PERIPHERAL_CONFIG_KEY)) == 0) {
      Status = EFI_SUCCESS;
      CopyMem (Value, &(mSpiConfigMap[Index].Value), sizeof (SPI_CONFIGURATION_DATA));
      break;
    }
  }

  return Status;
}
