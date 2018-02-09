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
/* Include necessary header files here */
#include <Pi/PiSpi.h>
#include <Protocol/SpiIo.h>
#include <Library/BaseMemoryLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/DebugLib.h>
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

EFI_STATUS
SpiNorFlashStart (
  IN EFI_DRIVER_BINDING_PROTOCOL   *This,
  IN EFI_HANDLE                    ControllerHandle,
  IN EFI_DEVICE_PATH_PROTOCOL      *RemainingDevicePath OPTIONAL
  )
{
  EFI_STATUS                 Status;
  EFI_SPI_IO_PROTOCOL        *SpiIo;
  EFI_SPI_REQUEST_PACKET     *RequestPacket;
  UINT8                      *Buf;
  UINT8                      *Data;
  UINTN                      Index;
  UINT32                     Address;

  Buf = NULL;
  Data = NULL;
  RequestPacket = NULL;
  Status = EFI_SUCCESS;
  SpiIo = NULL;

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

  DEBUG ((DEBUG_ERROR, "%a Attributes = 0x%08x\n",__FUNCTION__, SpiIo->Attributes));

  //
  // Allocate and zero a private data structure for the SpiNorFlash device.
  //
  RequestPacket = (EFI_SPI_REQUEST_PACKET *)AllocateZeroPool (
                                              sizeof (UINTN) +
                                              4 * sizeof (EFI_SPI_BUS_TRANSACTION)
                                              );
  if (RequestPacket == NULL) {
    goto ErrorExit;
  }
  Buf = (UINT8 *)AllocatePool (sizeof (UINT32));
  if (Buf == NULL) {
    goto ErrorExit;
  }
  Data = (UINT8 *)AllocatePool (sizeof (UINT8) * 0x40);
  if (Data == NULL) {
    goto ErrorExit;
  }

  //
  // Initialize the contents of the private data structure for the SpiNorFlash device.
  // This includes the SpiIo protocol instance and other private data fields
  // and the EFI_ABC_IO_PROTOCOL instance that will be installed.
  //
  Buf[0] = 0x0b; // Fast Read
  Address = 0x100000; // first boot loader address
  CopyMem (&Buf[1], &Address, 3); // copy 3 bytes of address

  RequestPacket->TransactionCount = 4;

  // Send Fast read Command
  RequestPacket->Transaction[0].TransactionType = SPI_TRANSACTION_COMMAND;
  RequestPacket->Transaction[0].BusWidth = SPI_TRANSACTION_BUS_WIDTH_1;
  RequestPacket->Transaction[0].Length = 1;
  RequestPacket->Transaction[0].FrameSize = 8;
  RequestPacket->Transaction[0].WriteBuffer = &Buf[0];

  // Send Address
  RequestPacket->Transaction[1].TransactionType = SPI_TRANSACTION_ADDRESS;
  RequestPacket->Transaction[1].BusWidth = SPI_TRANSACTION_BUS_WIDTH_1;
  RequestPacket->Transaction[1].Length = 3;
  RequestPacket->Transaction[1].FrameSize = 8;
  RequestPacket->Transaction[1].WriteBuffer = &Buf[1];

  // Send 8 dummy cycles
  RequestPacket->Transaction[2].TransactionType = SPI_TRANSACTION_DUMMY;
  RequestPacket->Transaction[2].BusWidth = SPI_TRANSACTION_BUS_WIDTH_1;
  RequestPacket->Transaction[2].Length = 8;
  RequestPacket->Transaction[2].FrameSize = 8;

  // Read Data
  RequestPacket->Transaction[3].TransactionType = SPI_TRANSACTION_DATA;
  RequestPacket->Transaction[3].BusWidth = SPI_TRANSACTION_BUS_WIDTH_1;
  RequestPacket->Transaction[3].Length = 0x40;
  RequestPacket->Transaction[3].FrameSize = 8;
  RequestPacket->Transaction[3].ReadBuffer = Data;

  Status = SpiIo->Transaction (
                    SpiIo,
                    RequestPacket,
                    0
                    );
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "%a %d Status = %d\n", __FUNCTION__, __LINE__, Status));
    goto ErrorExit;
  }

  for (Index = 1; Index <= 0x40; Index++) {
    DEBUG ((DEBUG_ERROR, "0x%02x, ", Data[Index - 1]));
    if (!(Index % 0x10)) {
      DEBUG ((DEBUG_ERROR, "\n"));
    }
  }

ErrorExit:
  //
  // When there is an error, the private data structures need to be freed and
  // the protocols that were opened need to be closed.
  //
  if (Data != NULL) {
    gBS->FreePool (Data);
  }
  if (Buf != NULL) {
    gBS->FreePool (Buf);
  }
  if (RequestPacket != NULL) {
    gBS->FreePool (RequestPacket);
  }
  if (EFI_ERROR(Status)) {
    gBS->CloseProtocol (
           ControllerHandle,
           &gEfiCallerIdGuid,
           This->DriverBindingHandle,
           ControllerHandle
           );
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
  EFI_STATUS             Status = EFI_SUCCESS;
  /*EFI_ABC_IO             SpiNorFlashIo;
  EFI_ABC_DEVICE         SpiNorFlashDevice;
  //
  // Get our context back
  //
  Status = gBS->OpenProtocol (
                  ControllerHandle,
                  &gEfiSpiNorFlashIoProtocolGuid,
                  &SpiNorFlashIo,
                  This->DriverBindingHandle,
                  ControllerHandle,
                  EFI_OPEN_PROTOCOL_GET_PROTOCOL
                  );
  if (EFI_ERROR (Status)) {
    return EFI_UNSUPPORTED;
  }
  //
  // Use Containment Record Macro to get SpiNorFlashDevice structure from
  // a pointer to the SpiNorFlashIo structure within the SpiNorFlashDevice structure.
  //
  SpiNorFlashDevice = ABC_IO_PRIVATE_DATA_FROM_THIS (SpiNorFlashIo);
  //
  // Uninstall the protocol installed in Start()
  //
  Status = gBS->UninstallMultipleProtocolInterfaces (
                  ControllerHandle,
                  &gEfiSpiNorFlashIoProtocolGuid, &SpiNorFlashDevice->SpiNorFlashIo,
                  NULL
                  );*/
  if (!EFI_ERROR (Status)) {
    //
    // Close the protocol opened in Start()
    //
    Status = gBS->CloseProtocol (
                    ControllerHandle,
                    &gEfiCallerIdGuid,
                    This->DriverBindingHandle,
                    ControllerHandle
                    );
    //
    // Free the structure allocated in Start().
    //
    // gBS->FreePool (SpiNorFlashDevice);
  }
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