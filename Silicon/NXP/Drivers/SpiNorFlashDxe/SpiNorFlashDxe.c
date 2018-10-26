/** @file
 The EFI_SPI_NOR_FLASH_PROTOCOL exists in the SPI peripheral layer.
 This protocol manipulates the SPI NOR flash parts using a common set
 of commands. The board layer provides the interconnection and
 configuration details for the SPI NOR flash part. The SPI NOR flash
 driver uses this configuration data to expose a generic interface
 which provides the following APls:
   •Read manufacture and device ID
   •Read data
   •Read data using low frequency
   •Read status
   •Write data
   •Erase 4 KiB blocks
   •Erase 32 or 64 KiB blocks
   •Write status

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
#include <Library/BaseMemoryLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/DebugLib.h>
#include <Library/TimerLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiLib.h>

#include "SpiNorFlashDxe.h"

/* Define external, global and module variables here */
//
//  EFI_DRIVER_BINDING_PROTOCOL instance
//
EFI_DRIVER_BINDING_PROTOCOL gSpiNorFlashBinding = {
  .Supported = SpiNorFlashSupported,
  .Start = SpiNorFlashStart,
  .Stop = SpiNorFlashStop,
  .Version = 0x10,
  .ImageHandle = NULL,
  .DriverBindingHandle = NULL
};


/* Function Definitions */
EFI_STATUS
SpiNorFlashSupported (
  IN EFI_DRIVER_BINDING_PROTOCOL   *This,
  IN EFI_HANDLE                    ControllerHandle,
  IN EFI_DEVICE_PATH_PROTOCOL      *RemainingDevicePath OPTIONAL
  )
{
  EFI_STATUS            Status;
  EFI_SPI_IO_PROTOCOL   *SpiIo;

  Status = gBS->OpenProtocol (
                  ControllerHandle,
                  &gEfiCallerIdGuid,
                  (VOID **)&SpiIo,
                  This->DriverBindingHandle,
                  ControllerHandle,
                  EFI_OPEN_PROTOCOL_BY_DRIVER
                  );
  if (EFI_ERROR (Status)) {
    return Status;
  }
  gBS->CloseProtocol (
         ControllerHandle,
         &gEfiCallerIdGuid,
         This->DriverBindingHandle,
         ControllerHandle
         );
  return EFI_SUCCESS;
}

VOID
PrintFlashParameters (
  IN  SPI_NOR_PARAMS  *SpiNorParams
  )
{
  SFDP_FLASH_PARAM          *ParamTable;

  ParamTable = SpiNorParams->ParamTable;
  DEBUG ((DEBUG_ERROR, "Flash Size = %u\n", SFDP_PARAM_FLASH_SIZE(ParamTable)));
  DEBUG ((DEBUG_ERROR, "Page Program Size = %u\n", SFDP_PARAM_PAGE_SIZE(ParamTable)));
  DEBUG ((DEBUG_ERROR, "Erase Size = %u\n", SFDP_PARAM_ERASE_SIZE(ParamTable)));
  DEBUG ((DEBUG_ERROR, "Erase Timeout = %u\n", GetMaxTimeout(SpiNorParams, SPI_NOR_REQUEST_TYPE_ERASE)));
  DEBUG ((DEBUG_ERROR, "Page Program Timeout = %u\n", GetMaxTimeout(SpiNorParams, SPI_NOR_REQUEST_TYPE_WRITE)));
}

VOID
FillData (
  IN  UINT8   *Data,
  IN  UINT32  Size
  )
{
  UINT32 Index;

  for (Index = 1; Index <= Size; Index++) {
    Data[Index - 1] = GetPerformanceCounter () % 0xFF;
    MicroSecondDelay (1); // to introduce randomization
  }
}

VOID
PrintData (
  IN  UINT8   *Data,
  IN  UINT32  Size
  )
{
  UINT32 Index;

  for (Index = 1; Index <= Size; Index++) {
    DEBUG ((DEBUG_ERROR, "%02x, ", Data[Index - 1]));
    if (!(Index % 0x10)) DEBUG ((DEBUG_ERROR, "\n"));
  }
  DEBUG ((DEBUG_ERROR, "\n"));
}

EFI_STATUS
SpiNorFlashStart (
  IN EFI_DRIVER_BINDING_PROTOCOL   *This,
  IN EFI_HANDLE                    ControllerHandle,
  IN EFI_DEVICE_PATH_PROTOCOL      *RemainingDevicePath OPTIONAL
  )
{
  EFI_STATUS                  Status;
  EFI_SPI_IO_PROTOCOL         *SpiIo;
  UINT32                      RequestPacketsBufferSize;
  SPI_NOR_PARAMS              *SpiNorParams;
  UINT8                       *ReadData;
  UINT8                       *WriteData;
  UINT32                      FlashOffset = 0x3c0000;
  UINT32                      DataSize = 0x100;
  SFDP_FLASH_PARAM            *ParamTable;

  Status = EFI_SUCCESS;
  SpiIo = NULL;
  SpiNorParams = NULL;
  ReadData = NULL;
  WriteData = NULL;

  //
  // Open the SPI I/O Protocol that this driver consumes
  //
  Status = gBS->OpenProtocol (
                  ControllerHandle,
                  &gEfiCallerIdGuid,
                  (VOID **)&SpiIo,
                  This->DriverBindingHandle,
                  ControllerHandle,
                  EFI_OPEN_PROTOCOL_BY_DRIVER
                  );
  if (EFI_ERROR (Status)) {
    return Status;
  }

  // Read SFDP Request Packet size
  RequestPacketsBufferSize = sizeof (UINTN) + 4 * sizeof (EFI_SPI_BUS_TRANSACTION);
  // Read Request Packet size. Assume that maximum 8 transactions would be sent
  RequestPacketsBufferSize += sizeof (UINTN) + 8 * sizeof (EFI_SPI_BUS_TRANSACTION);
  // Write Request Packet size. Assume that maximum 8 transactions would be sent
  RequestPacketsBufferSize += sizeof (UINTN) + 8 * sizeof (EFI_SPI_BUS_TRANSACTION);
  // Erase Request Packet size. Assume that maximum 8 transactions would be sent
  RequestPacketsBufferSize += sizeof (UINTN) + 8 * sizeof (EFI_SPI_BUS_TRANSACTION);
  // Write Enable Request Packet size.
  RequestPacketsBufferSize += sizeof (UINTN) + sizeof (EFI_SPI_BUS_TRANSACTION);
  // Read Status Request Packet size.
  RequestPacketsBufferSize += sizeof (UINTN) + 2 * sizeof (EFI_SPI_BUS_TRANSACTION);

  // TODO : How to know if allocate runtime memory of not ?
  // Allocate memory
  SpiNorParams = (SPI_NOR_PARAMS *)AllocateZeroPool (sizeof (SPI_NOR_PARAMS));
  if (!SpiNorParams) {
    Status = EFI_OUT_OF_RESOURCES;
    goto ErrorExit;
  }
  SpiNorParams->RequestPackets = (EFI_SPI_REQUEST_PACKET *)AllocateZeroPool (RequestPacketsBufferSize);
  if (!SpiNorParams->RequestPackets) {
    Status = EFI_OUT_OF_RESOURCES;
    goto ErrorExit;
  }

  // Read JEDEC Basic Flash Parameter Header and Table
  Status = ReadSfdpParameterTable (
             SpiIo,
             SpiNorParams,
             PARAMETER_ID (0xFF, 0x00),
             PARAMETER_REV (1, 6),
             &SpiNorParams->ParamHeader,
             (VOID **)&SpiNorParams->ParamTable,
             TRUE
             );
  if (EFI_ERROR (Status)) {
    goto ErrorExit;
  }

  Status = FillFlashParam (SpiIo, SpiNorParams);
  if (EFI_ERROR (Status)) {
    goto ErrorExit;
  }

  ParamTable = SpiNorParams->ParamTable;

  PrintFlashParameters (SpiNorParams);

  // Allocate memory for Reading Data
  ReadData = (UINT8 *)AllocatePool (DataSize);
  if (!ReadData) {
    Status = EFI_OUT_OF_RESOURCES;
    goto ErrorExit;
  }
  // Read Data from Flash
  Status = ReadFlashData (
             SpiIo,
             SpiNorParams,
             FlashOffset,
             DataSize,
             ReadData
             );
  if (EFI_ERROR (Status)) {
    goto ErrorExit;
  }
  DEBUG ((DEBUG_ERROR, "Data Already present in Flash @ Offset 0x%x\n", FlashOffset));
  PrintData (ReadData, DataSize);
  // Allocate memory for Writing Data
  WriteData = (UINT8 *)AllocatePool (DataSize);
  if (!WriteData) {
    Status = EFI_OUT_OF_RESOURCES;
    goto ErrorExit;
  }
  // Erase flash
  Status = EraseFlashBlock (
             SpiIo,
             SpiNorParams,
             FlashOffset
             );
  if (EFI_ERROR (Status)) {
    goto ErrorExit;
  }
  // Fill Write data
  FillData (WriteData, DataSize);
  DEBUG ((DEBUG_ERROR, "Data To be written to  Flash @ Offset 0x%x\n", FlashOffset));
  PrintData (WriteData, DataSize);
  // Write data to Flash
  Status = WriteFlashData (
             SpiIo,
             SpiNorParams,
             FlashOffset,
             DataSize,
             WriteData
             );
  if (EFI_ERROR (Status)) {
    goto ErrorExit;
  }
  // Read Data from Flash
  Status = ReadFlashData (
             SpiIo,
             SpiNorParams,
             FlashOffset,
             DataSize,
             ReadData
             );
  if (EFI_ERROR (Status)) {
    goto ErrorExit;
  }
  DEBUG ((DEBUG_ERROR, "Data Read back from Flash @ Offset 0x%x\n", FlashOffset));
  PrintData (ReadData, DataSize);
  // Compare data written and then read back
  if (!CompareMem (ReadData, WriteData, DataSize)) {
    DEBUG ((DEBUG_ERROR, "Flash Operations are successful\n"));
  } else {
    DEBUG ((DEBUG_ERROR, "Flash Operations failed\n"));
  }

ErrorExit:
  //
  // When there is an error, the private data structures need to be freed and
  // the protocols that were opened need to be closed.
  //
  if (EFI_ERROR(Status)) {
    gBS->CloseProtocol (
           ControllerHandle,
           &gEfiCallerIdGuid,
           This->DriverBindingHandle,
           ControllerHandle
           );
  }

  if (SpiNorParams && SpiNorParams->RequestPackets) {
    FreePool (SpiNorParams->RequestPackets);
  }
  if (SpiNorParams) {
    FreePool (SpiNorParams);
  }
  if (ReadData) {
    FreePool (ReadData);
  }
  if (WriteData) {
    FreePool (WriteData);
  }

  return Status;
}

EFI_STATUS
SpiNorFlashStop (
  IN EFI_DRIVER_BINDING_PROTOCOL   *This,
  IN EFI_HANDLE                    ControllerHandle,
  IN UINTN                         NumberOfChildren,
  IN EFI_HANDLE                    *ChildHandleBuffer OPTIONAL
  )
{
  EFI_STATUS    Status;

  //
  // Close the protocol opened in Start()
  //
  Status = gBS->CloseProtocol (
                  ControllerHandle,
                  &gEfiCallerIdGuid,
                  This->DriverBindingHandle,
                  ControllerHandle
                  );
  return Status;
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
SpiNorFlashDxeEntryPoint (
  IN EFI_HANDLE        ImageHandle,
  IN EFI_SYSTEM_TABLE  *SystemTable
  )
{
  EFI_STATUS              Status;

  //
  // Install driver model protocol(s).
  //
  Status = EfiLibInstallDriverBinding (
             ImageHandle,
             SystemTable,
             &gSpiNorFlashBinding,
             NULL
             );
  ASSERT_EFI_ERROR (Status);

  return Status;
}
