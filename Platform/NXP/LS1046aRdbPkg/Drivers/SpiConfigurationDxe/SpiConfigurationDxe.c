/** @file
 file implementing EFI_SPI_CONFIGURATION_PROTOCOL.

 The board layer uses the EFI_SPI_CONFIGURATION_PROTOCOL to expose
 the data tables which describe the board's SPI busses, The SPI bus
 layer uses these tables to configure the clock, chip select and 
 manage the SPI transactions on the SPI controllers.
 The configuration tables describe:
   •The number of SPI busses on the board
   •Which SPI chips are connected to each SPI bus 
   For each SPI chip the configuration describes:
    •The maximum clock frequency for the SPI part
    •The clock polarity needed for the SPI part
    •Whether the SPI controller is a separate clock generator needs to be set up
    •The chip select polarity
    •Whether the SPI controller or a GPIO pin is used for the chip select
    •The data sampling edge for the SPI part

 Copyright 2018 NXP.

 This program and the accompanying materials
 are licensed and made available under the terms and conditions of
 the BSD License which accompanies this distribution. The full
 text of the license may be found at
 http://opensource.org/licenses/bsd-license.php

 THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS"
 BASIS, WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER
 EXPRESS OR IMPLIED.

 @par Specification Reference:
   - PI 1.6, Chapter 18, Spi Protocol Stack
**/
/* Include necessary header files here */
#include <Pi/PiSpi.h>
#include <Protocol/SpiConfiguration.h>
#include <Library/BaseMemoryLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <SpiPlatform.h>

/* Define external, global and module variables here */
#define    MAX_SPI_BUSES    1

//
// Template for SPI controller device path.
//
EFI_SPI_DEVICE_PATH gSpiControllerDevicePathTemplate = {
  .Vendor = {
    .Header = {
      .Type = HARDWARE_DEVICE_PATH,
      .SubType = HW_VENDOR_DP,
      .Length = {
        (UINT8)sizeof(VENDOR_DEVICE_PATH),
        (UINT8)((sizeof(VENDOR_DEVICE_PATH)) >> 8)
      }
    },
    .Guid = EFI_CALLER_ID_GUID
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
    .ControllerNumber = 0
  },
  .End = {
    .Type = END_DEVICE_PATH_TYPE,
    .SubType = END_ENTIRE_DEVICE_PATH_SUBTYPE,
    .Length = { sizeof (EFI_DEVICE_PATH_PROTOCOL), 0 }
  }
};

BOOLEAN            gSpiBusRuntime[MAX_SPI_BUSES];
CONST EFI_SPI_BUS  *gSpiBuses[MAX_SPI_BUSES];

EFI_SPI_CONFIGURATION_PROTOCOL gSpiConfigProtocol = {
  .BusCount = 0,
  .Buslist = gSpiBuses,
};

/* Function Definitions */
/**
 EFI image entry point. 

 @param[in] ImageHandle  The firmware allocated handle for the UEFI image.
 @param[in] SystemTable  A pointer to the EFI System Table.

 @retval EFI_SUCCESS     The operation completed successfully.
 @retval Others          An unexpected error occurred.
**/
EFI_STATUS
EFIAPI
SpiConfigurationDxeEntryPoint (
  IN EFI_HANDLE        ImageHandle,
  IN EFI_SYSTEM_TABLE  *SystemTable
  )
{
  EFI_SPI_BUS                        *SpiBus;
  EFI_SPI_PERIPHERAL                 *SpiPeripheral;
  UINTN                              Index;
  GUID                               *SpiPeripheralDriverGuid;
  EFI_SPI_PART                       *SpiPart;
  EFI_SPI_DEVICE_PATH                *DevicePath;
  SPI_CONFIGURATION_DATA             *SpiConfigData;
  VOID                               *ChipSelectParameter;
  EFI_STATUS                         Status;

  SpiBus = AllocateRuntimeZeroPool (sizeof (EFI_SPI_BUS));
  if (SpiBus == NULL) {
    return EFI_OUT_OF_RESOURCES;
  }
  
  DevicePath = AllocateRuntimeCopyPool (sizeof (EFI_SPI_DEVICE_PATH), &gSpiControllerDevicePathTemplate);
  if (SpiBus == NULL) {
    return EFI_OUT_OF_RESOURCES;
  }
  
  CopyMem (&(DevicePath->Vendor.Guid), &gEfiQspiDriverGuid, sizeof (GUID));
  DevicePath->Controller.ControllerNumber = 0;

  for (Index = 0; Index < 2; Index++) {
    if (Index == 0) {
      SpiPeripheral = AllocateRuntimeZeroPool (sizeof (EFI_SPI_PERIPHERAL));
      SpiPeripheralDriverGuid = AllocateRuntimeCopyPool (sizeof (GUID), &gEfiSpiNorFlashDriverGuid);
      SpiPart = AllocateRuntimeZeroPool (sizeof (EFI_SPI_PART));
      SpiPart->ChipSelectPolarity = TRUE;
      SpiPart->MaxClockHz = 50000000;
      SpiPeripheral->Attributes = SPI_HALF_DUPLEX |
                                  SPI_SUPPORTS_DTR_OPERATIONS |
                                  SPI_SUPPORTS_2_BIT_DATA_BUS_WIDTH |
                                  SPI_SUPPORTS_4_BIT_DATA_BUS_WIDTH;
      SpiConfigData = AllocateRuntimeZeroPool (sizeof (SPI_CONFIGURATION_DATA));
      SpiConfigData->DeviceBaseAddress = 0x40000000;
      SpiConfigData->RegionBaseAddress = 0x40000000;
      SpiConfigData->PageSize = 0x40; 
      ChipSelectParameter = AllocateRuntimePool (sizeof (UINT8));
      *((UINT8 *)ChipSelectParameter) = Index;

      SpiPeripheral->SpiPeripheralDriverGuid = SpiPeripheralDriverGuid;
      SpiPeripheral->SpiPart = SpiPart;
      SpiPeripheral->ConfigurationData = SpiConfigData;
      SpiPeripheral->SpiBus = SpiBus;
      SpiPeripheral->ChipSelectParameter = ChipSelectParameter;
      SpiBus->RuntimePeripherallist = SpiPeripheral;
    } else {
      SpiPeripheral = AllocateZeroPool (sizeof (EFI_SPI_PERIPHERAL));
      SpiPeripheralDriverGuid = AllocateCopyPool (sizeof (GUID), &gEfiSpiNorFlashDriverGuid);
      SpiPart = AllocateZeroPool (sizeof (EFI_SPI_PART));
      SpiPart->ChipSelectPolarity = TRUE;
      SpiPart->MaxClockHz = 133000000;
      SpiPeripheral->Attributes = SPI_HALF_DUPLEX |
                                  SPI_SUPPORTS_DTR_OPERATIONS |
                                  SPI_SUPPORTS_2_BIT_DATA_BUS_WIDTH |
                                  SPI_SUPPORTS_4_BIT_DATA_BUS_WIDTH;
      SpiConfigData = AllocateZeroPool (sizeof (SPI_CONFIGURATION_DATA));
      SpiConfigData->DeviceBaseAddress = 0x48000000;
      SpiConfigData->RegionBaseAddress = 0x48000000;
      SpiConfigData->PageSize = 0x40; 
      ChipSelectParameter = AllocatePool (sizeof (UINT8));
      *((UINT8 *)ChipSelectParameter) = Index;
      
      SpiPeripheral->SpiPeripheralDriverGuid = SpiPeripheralDriverGuid;
      SpiPeripheral->SpiPart = SpiPart;
      SpiPeripheral->ConfigurationData = SpiConfigData;
      SpiPeripheral->SpiBus = SpiBus;
      SpiPeripheral->ChipSelectParameter = ChipSelectParameter;
      SpiBus->Peripherallist = SpiPeripheral;
    }
  }
  
  if (SpiBus->RuntimePeripherallist != NULL) {
    gSpiBusRuntime[0] = TRUE;
  }
  
  SpiBus->ControllerPath = (EFI_DEVICE_PATH_PROTOCOL *)DevicePath;

  gSpiBuses[0] = SpiBus;

  gSpiConfigProtocol.BusCount = MAX_SPI_BUSES;
  
  Status = gBS->InstallProtocolInterface (
                  &ImageHandle,
                  &gEfiSpiConfigurationProtocolGuid,
                  EFI_NATIVE_INTERFACE,
                  &gSpiConfigProtocol
                  );
  if (EFI_ERROR (Status)) {
    return Status;
  }

  return EFI_SUCCESS;
}

/* If this is a protocol definition, the
 * protocol structure is defined and initialized here.
 * */
