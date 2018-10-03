/** @file
 file implementing EFI_SPI_CONFIGURATION_PROTOCOL.

 The board layer uses the EFI_SPI_CONFIGURATION_PROTOCOL to expose
 the data tables which describe the board's SPI busses, The SPI bus
 layer uses these tables to configure the clock, chip select and
 manage the SPI transactions on the SPI controllers.
 The configuration tables describe:
   -The number of SPI busses on the board
   -Which SPI chips are connected to each SPI bus
   For each SPI chip the configuration describes:
    -The maximum clock frequency for the SPI part
    -The clock polarity needed for the SPI part
    -Whether the SPI controller is a separate clock generator needs to be set up
    -The chip select polarity
    -Whether the SPI controller or a GPIO pin is used for the chip select
    -The data sampling edge for the SPI part

 Copyright 2018 NXP

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
#include <libfdt.h>
#include <Pi/PiSpi.h>
#include <Protocol/SpiConfiguration.h>
#include <Library/BaseMemoryLib.h>
#include <Library/DebugLib.h>
#include <Library/DxeServicesLib.h>
#include <Library/DxeServicesTableLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/PcdLib.h>
#include <Library/UefiBootServicesTableLib.h>

/* Define external, global and module variables here */
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

///
/// Mapping between SPI Bus compatible string in device tree and controller drivers' GUID
///
STATIC CONST struct {
  CHAR8*  SpiBusCompatible;
  GUID*   SpiBusDriverGuid;
} gSpiBusIds[] = {
  { FixedPcdGetPtr (PcdQspiFdtCompatible), &gEfiQspiDriverGuid },
  { FixedPcdGetPtr (PcdDspiFdtCompatible), &gEfiDspiDriverGuid },
  { FixedPcdGetPtr (PcdFlexSpiFdtCompatible), &gEfiFlexSpiDriverGuid }
};

///
/// Mapping between SPI Peripheral compatible string in device tree and Peripheral drivers' GUID
///
STATIC CONST struct {
  CHAR8*  SpiPeripheralCompatible;
  GUID*   SpiPeripheralDriverGuid;
} gSpiPeripheralIds[] = {
  { "spansion,m25p80", &gEfiSpiNorFlashDriverGuid },
  { "micron,m25p80", &gEfiSpiNorFlashDriverGuid }
};

CONST EFI_SPI_BUS    *gSpiBuses[FixedPcdGet32 (PcdSpiBusCount)];

EFI_SPI_CONFIGURATION_PROTOCOL gSpiConfigProtocol = {
  .BusCount = 0,
  .Buslist = gSpiBuses,
};

/* Function Definitions */
/**
 Parse the device tree and retrieve the information of SPI bus node and the peripherals' information
 which are children of SPI bus node.
 Allocate and fill the EFI_SPI_BUS structure.
 EFI_SPI_BUS structure also contains the peripherals' information attached to SPI bus.

 @param[in]  Fdt                 Platform's device tree blob
 @param[in]  SpiBusDevicePath    Device path of SPI Controller controlling the SPI bus
 @param[in]  SpiBusNodeOffset    SPI bus node offset in device tree.
 @param[in]  SpiChildNodeOffset  SPI Child node offset in device tree.
 @param[out] SpiPeripheral       pointer to EFI_SPI_PERIPHERAL structure containing information about
                                 peripheral and the Bus to which this peripheral is attached.
 @param[out] Runtime             Weather this peripheral is needed for uefi runtime services or not?
                                 Based on this value, the peripheral node is added to Runtime Peripherals' list.

 @retval EFI_SUCCESS           The operation completed successfully.
 @retval EFI_OUT_OF_RESOURCES  No sufficient memory available.
 @retval EFI_NOT_FOUND         No Peripheral driver assigned to Child node
**/
STATIC
EFI_STATUS
ParseSpiChildNode (
  IN  VOID                    *Fdt,
  IN  EFI_SPI_DEVICE_PATH     *SpiBusDevicePath,
  IN  INT32                   SpiBusNodeOffset,
  IN  INT32                   SpiChildNodeOffset,
  OUT EFI_SPI_PERIPHERAL      **SpiPeripheral,
  OUT BOOLEAN                 *Runtime
  )
{
  CONST fdt32_t                      *Prop;
  INT32                              PropLen;
  UINTN                              Index;
  UINT32                             SpiBusWidth;
  VOID                               *ChipSelectParameter;
  EFI_SPI_PERIPHERAL                 *pSpiPeripheral;
  EFI_SPI_PART                       *SpiPart;
  EFI_STATUS                         Status;

  pSpiPeripheral = NULL;
  SpiPart = NULL;
  ChipSelectParameter = NULL;
  Status = EFI_SUCCESS;
  SpiBusWidth = SPI_TRANSACTION_BUS_WIDTH_1;

  if ((Runtime == NULL) || (SpiPeripheral == NULL) || (Fdt == NULL) || (SpiBusDevicePath == NULL)) {
    Status = EFI_INVALID_PARAMETER;
    goto ErrorExit;
  }

  /* Allocate resources based on runtime property */
  Prop = fdt_getprop(Fdt, SpiChildNodeOffset, "uefi-runtime", &PropLen);
  if (Prop == NULL) {
    pSpiPeripheral = AllocateZeroPool (sizeof (EFI_SPI_PERIPHERAL));
    SpiPart = AllocateZeroPool (sizeof (EFI_SPI_PART));
    ChipSelectParameter = AllocatePool (sizeof (UINT32));
    *Runtime = FALSE;
  } else {
    pSpiPeripheral = AllocateRuntimeZeroPool (sizeof (EFI_SPI_PERIPHERAL));
    SpiPart = AllocateRuntimeZeroPool (sizeof (EFI_SPI_PART));
    ChipSelectParameter = AllocateRuntimePool (sizeof (UINT32));
    *Runtime = TRUE;
  }
  if ((pSpiPeripheral == NULL) || (SpiPart == NULL) || (ChipSelectParameter == NULL)) {
    Status = EFI_OUT_OF_RESOURCES;
    goto ErrorExit;
  }

  // As per Device tree bindings reg node contains chip select number,
  // for now we support only one cell for chip select number
  // and we don't support GPIO chip selects.
  Prop = fdt_getprop(Fdt, SpiChildNodeOffset, "reg", &PropLen);
  if (Prop && PropLen >= sizeof (INT32)) {
    *(UINT32 *)ChipSelectParameter = fdt32_to_cpu(Prop[0]);
  } else {
    Status = EFI_NOT_FOUND;
    goto ErrorExit;
  }

  // sift through supported spi Peripherals' list
  for (Index = 0; Index < ARRAY_SIZE (gSpiPeripheralIds); Index++) {
    if (fdt_node_check_compatible(Fdt, SpiChildNodeOffset, gSpiPeripheralIds[Index].SpiPeripheralCompatible) == 0) {
      pSpiPeripheral->SpiPeripheralDriverGuid = gSpiPeripheralIds[Index].SpiPeripheralDriverGuid;
      if (AsciiStrnCmp (gSpiPeripheralIds[Index].SpiPeripheralCompatible, "spi-flash", AsciiStrLen ("spi-flash")) == 0) {
        pSpiPeripheral->Attributes |= SPI_HALF_DUPLEX; // All Spi flashes are half duplex
      }
    }
  }

  if (pSpiPeripheral->SpiPeripheralDriverGuid == NULL) {
    Status = EFI_NOT_FOUND;
    goto ErrorExit;
  }

  // As per device tree bindings spi-max-frequency is mandatory property
  Prop = fdt_getprop(Fdt, SpiChildNodeOffset, "spi-max-frequency", &PropLen);
  if (Prop && PropLen >= sizeof (INT32)) {
    SpiPart->MaxClockHz = fdt32_to_cpu(Prop[0]);
  } else {
    Status = EFI_NOT_FOUND;
    goto ErrorExit;
  }

  //Empty property indicating device requires chip select active high.
  Prop = fdt_getprop(Fdt, SpiChildNodeOffset, "spi-cs-high", &PropLen);
  if (Prop != NULL) {
    SpiPart->ChipSelectPolarity = TRUE;
  }

  // Empty property indicating device requires inverse clock polarity (CPOL) mode.
  Prop = fdt_getprop(Fdt, SpiChildNodeOffset, "spi-cpol", &PropLen);
  if (Prop != NULL) {
    pSpiPeripheral->ClockPolarity = TRUE;
  }

  // Empty property indicating device requires shifted clock phase (CPHA) mode.
  Prop = fdt_getprop(Fdt, SpiChildNodeOffset, "spi-cpha", &PropLen);
  if (Prop != NULL) {
    pSpiPeripheral->ClockPhase = TRUE;
  }

  // Empty property indicating device requires 3-wire mode.
  // Now the value that spi-tx-bus-width and spi-rx-bus-width can receive is
  // only 1 (SINGLE), 2 (DUAL) and 4 (QUAD) or 8 (OCTAL)
  // Dual/Quad/Octal mode is not allowed when 3-wire mode is used.
  Prop = fdt_getprop(Fdt, SpiChildNodeOffset, "spi-3wire", &PropLen);
  if (Prop != NULL) {
    pSpiPeripheral->Attributes |= SPI_HALF_DUPLEX;
  } else {
    // choose minimum of tx width and rx width as SPI IO width
    // By default IO width one is assumed.
    Prop = fdt_getprop(Fdt, SpiChildNodeOffset, "spi-tx-bus-width", &PropLen);
    if (Prop && PropLen >= sizeof (INT32)) {
      SpiBusWidth = fdt32_to_cpu(Prop[0]);
    }

    Prop = fdt_getprop(Fdt, SpiChildNodeOffset, "spi-rx-bus-width", &PropLen);
    if (Prop && PropLen >= sizeof (INT32)) {
      SpiBusWidth = (SpiBusWidth <= fdt32_to_cpu(Prop[0])) ? SpiBusWidth : fdt32_to_cpu(Prop[0]);
    }

    switch (SpiBusWidth) {
      case SPI_TRANSACTION_BUS_WIDTH_8:
        pSpiPeripheral->Attributes |= SPI_SUPPORTS_8_BIT_DATA_BUS_WIDTH;
        break;
      case SPI_TRANSACTION_BUS_WIDTH_4:
        pSpiPeripheral->Attributes |= SPI_SUPPORTS_4_BIT_DATA_BUS_WIDTH;
        break;
      case SPI_TRANSACTION_BUS_WIDTH_2:
        pSpiPeripheral->Attributes |= SPI_SUPPORTS_2_BIT_DATA_BUS_WIDTH;
        break;
    }
  }

  pSpiPeripheral->ChipSelectParameter = ChipSelectParameter;
  pSpiPeripheral->SpiPart = SpiPart;

  *SpiPeripheral = pSpiPeripheral;

ErrorExit:
  if (EFI_ERROR(Status)) {
    if (pSpiPeripheral != NULL) {
      FreePool (pSpiPeripheral);
    }
    if (SpiPart != NULL) {
      FreePool (SpiPart);
    }
    if (ChipSelectParameter != NULL) {
      FreePool (ChipSelectParameter);
    }
  }

  return Status;
}

/**
 Free SpiPeripheral members, that are dynamically allocated.

 @param[in] SpiPeripheral  Pointer to EFI_SPI_PERIPHERAL whose members are to be freed.

**/
STATIC
VOID
FreeSpiPeripheral (
  IN CONST EFI_SPI_PERIPHERAL       *SpiPeripheral
  )
{
  if (SpiPeripheral == NULL) {
    return;
  }

  FreePool ((VOID *)SpiPeripheral->SpiPart);
  FreePool ((VOID *)SpiPeripheral->ChipSelectParameter);
  if (SpiPeripheral->ConfigurationData != NULL) {
    FreePool ((VOID *)SpiPeripheral->ConfigurationData);
  }

  return;
}

/**
 Free SpiBus members, that are dynamically allocated.

 @param[in] SpiBus  Pointer to EFI_SPI_BUS whose members are to be freed.

**/
STATIC
VOID
FreeSpiBus (
  IN CONST EFI_SPI_BUS       *SpiBus
  )
{
  CONST EFI_SPI_PERIPHERAL           *SpiPeripheral;
  CONST EFI_SPI_PERIPHERAL           *NextSpiPeripheral;

  if (SpiBus == NULL) {
    return;
  }

  SpiPeripheral = SpiBus->RuntimePeripherallist;
  while (SpiPeripheral != NULL) {
    NextSpiPeripheral = SpiPeripheral->NextSpiPeripheral;
    FreeSpiPeripheral (SpiPeripheral);
    FreePool ((VOID *)SpiPeripheral);
    SpiPeripheral = NextSpiPeripheral;
  }

  SpiPeripheral = SpiBus->Peripherallist;
  while (SpiPeripheral != NULL) {
    NextSpiPeripheral = SpiPeripheral->NextSpiPeripheral;
    FreeSpiPeripheral (SpiPeripheral);
    FreePool ((VOID *)SpiPeripheral);
    SpiPeripheral = NextSpiPeripheral;
  }

  if (SpiBus->ControllerPath != NULL) {
    FreePool ((VOID *)SpiBus->ControllerPath);
  }
}

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
  EFI_SPI_DEVICE_PATH                *DevicePath;
  VOID                               *Fdt;
  UINTN                              FdtSize;
  CONST fdt32_t                      *Prop;
  INT32                              PropLen;
  INT32                              SpiBusNodeOffset;
  INT32                              SpiChildNodeOffset;
  UINT32                             SpiBusCount; // Total number of spi buses in system
  UINT32                             SpiBusTypeCount; // Spi buses of one type
  BOOLEAN                            Runtime;
  EFI_STATUS                         Status;

  Runtime = FALSE;
  SpiPeripheral = NULL;

  Status = GetSectionFromAnyFv (
             &gDtPlatformDefaultDtbFileGuid,
             EFI_SECTION_RAW,
             0,
             &Fdt,
             &FdtSize
             );
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "Failed to get device tree\n"));
    return EFI_NOT_FOUND;
  }

  if (fdt_check_header (Fdt)) {
    DEBUG ((DEBUG_ERROR, "Invalid Device Tree\n"));
    return EFI_CRC_ERROR;
  }

  //
  // Make sure the Spi Configuration Protocol has not been installed in the system yet.
  //
  ASSERT_PROTOCOL_ALREADY_INSTALLED (NULL, &gEfiSpiConfigurationProtocolGuid);

  SpiBusCount = 0;
  for (Index = 0; (Index < ARRAY_SIZE (gSpiBusIds)) && (SpiBusCount < FixedPcdGet32 (PcdSpiBusCount)); Index++) {
    if (AsciiStrSize(gSpiBusIds[Index].SpiBusCompatible) == 1) {
      // The compatible string is empty string, therefore skip it.
      continue;
    }
    SpiBusTypeCount = 0;
    for ((SpiBusNodeOffset = fdt_node_offset_by_compatible (Fdt, -1, gSpiBusIds[Index].SpiBusCompatible));
         (SpiBusNodeOffset != -FDT_ERR_NOTFOUND) && ((SpiBusCount + SpiBusTypeCount) < FixedPcdGet32 (PcdSpiBusCount));
         (SpiBusNodeOffset = fdt_node_offset_by_compatible (Fdt, SpiBusNodeOffset, gSpiBusIds[Index].SpiBusCompatible)))
    {
      Prop = fdt_getprop (Fdt, SpiBusNodeOffset, "status", &PropLen);
      if (Prop == NULL) {
        DEBUG ((DEBUG_WARN, "status property not found\n"));
      } else if (PropLen != (AsciiStrLen ("okay") + 1)
               || !fdt_stringlist_contains ( (CHAR8 *)Prop, PropLen, "okay"))
      {

        // It should say "okay", so only allow that. Some fdts use "ok" but
        // this is a bug. Please fix your device tree source file. See here
        // for discussion:
        //
        // http://www.mail-archive.com/u-boot@lists.denx.de/msg71598.html

        continue;
      }

      Prop = fdt_getprop(Fdt, SpiBusNodeOffset, "uefi-runtime", &PropLen);
      if (Prop == NULL) {
        SpiBus = AllocateZeroPool (sizeof (EFI_SPI_BUS));
      } else {
        SpiBus = AllocateRuntimeZeroPool (sizeof (EFI_SPI_BUS));
      }

      DevicePath = AllocateCopyPool (sizeof (EFI_SPI_DEVICE_PATH), &gSpiControllerDevicePathTemplate);
      if (DevicePath == NULL) {
        FreePool (SpiBus);
        Status = EFI_OUT_OF_RESOURCES;
        break;
      }

      CopyMem (&(DevicePath->Vendor.Guid), gSpiBusIds[Index].SpiBusDriverGuid, sizeof (GUID));

      Prop = fdt_getprop(Fdt, SpiBusNodeOffset, "bus-num", &PropLen);
      if (Prop == NULL) {
        DevicePath->Controller.ControllerNumber = SpiBusTypeCount;
      } else {
        DevicePath->Controller.ControllerNumber = fdt32_to_cpu(*Prop);
      }

      // Parse Child nodes of SPI bus
      fdt_for_each_subnode (SpiChildNodeOffset, Fdt, SpiBusNodeOffset) {
        Status = ParseSpiChildNode (
                   Fdt,
                   DevicePath,
                   SpiBusNodeOffset,
                   SpiChildNodeOffset,
                   &SpiPeripheral,
                   &Runtime
                   );

        // if any valid child node info is not found, its not an error for SPI bus
        // Move onto next SPI child node
        if (Status == EFI_NOT_FOUND) {
          Status = EFI_SUCCESS;
          continue;
        }
        if (EFI_ERROR (Status)) {
          break;
        }

        SpiPeripheral->SpiBus = SpiBus;
        if (Runtime == TRUE) {
          SpiPeripheral->NextSpiPeripheral = SpiBus->RuntimePeripherallist;
          SpiBus->RuntimePeripherallist = SpiPeripheral;
        } else {
          SpiPeripheral->NextSpiPeripheral = SpiBus->Peripherallist;
          SpiBus->Peripherallist = SpiPeripheral;
        }
      }
      // if error occurred while parsing SPI child nodes of a SPI bus node
      // free all the resources allocated for that SPI bus
      if (EFI_ERROR (Status)) {
        FreePool (DevicePath);
        FreeSpiBus (SpiBus);
        FreePool (SpiBus);
        break;
      }

      SpiBus->ControllerPath = (EFI_DEVICE_PATH_PROTOCOL *)DevicePath;
      gSpiBuses[SpiBusCount + SpiBusTypeCount] = SpiBus;
      SpiBusTypeCount++;
    }
    if (EFI_ERROR (Status)) {
      break;
    }
    SpiBusCount += SpiBusTypeCount;
  }

  if (EFI_ERROR (Status)) {
    goto ErrorExit;
  }

  gSpiConfigProtocol.BusCount = SpiBusCount;

  Status = gBS->InstallProtocolInterface (
                  &ImageHandle,
                  &gEfiSpiConfigurationProtocolGuid,
                  EFI_NATIVE_INTERFACE,
                  &gSpiConfigProtocol
                  );

ErrorExit:
  if (EFI_ERROR (Status)) {
    // in case of error free all the resources allocated for all SPI buses
    for (Index = 0; Index < SpiBusCount; Index++) {
       FreeSpiBus (gSpiBuses[Index]);
       FreePool ((VOID *)gSpiBuses[Index]);
    }
    SpiBusCount = 0;
    gSpiConfigProtocol.BusCount = SpiBusCount;
  }

  return Status;
}
