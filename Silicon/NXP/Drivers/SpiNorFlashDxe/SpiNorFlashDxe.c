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
#include <Bitops.h>
#include <Library/BaseMemoryLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/DebugLib.h>
#include <Library/TimerLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiRuntimeLib.h>

#include <Protocol/SpiConfigData.h>

#include "SpiNorFlashDxe.h"

/* Define external, global and module variables here */
SPI_NOR_FLASH_CONTEXT  mSpiNorFlashContextTemplate = {
  .Signature = SPI_NOR_FLASH_SIGNATURE,

  .SpiIo = NULL, // NEED TO BE FILLED
  .SpiNorParams = NULL, // NEED TO BE FILLED

  .FvbProtocol = {
    .GetAttributes = FvbGetAttributes,
    .SetAttributes = FvbSetAttributes,
    .GetPhysicalAddress = FvbGetPhysicalAddress,
    .GetBlockSize = FvbGetBlockSize,
    .Read = FvbRead,
    .Write = FvbWrite,
    .EraseBlocks = FvbEraseBlocks,
    .ParentHandle = NULL,
  },
  .StartLba = 0,
  .LastLba = 0,
  .ShadowBuffer = NULL, // NEED TO BE FILLED

  .SpiNorVirtualAddressEvent = NULL // NEED TO BE FILLED
};

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
  EFI_STATUS                            Status;
  EFI_SPI_IO_PROTOCOL                   *SpiIo;
  CONST SPI_FLASH_CONFIGURATION_DATA    *ConfigData;
  BOOLEAN                               ContainVariableStorage;

  ContainVariableStorage = TRUE;
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

  ConfigData = SpiIo->SpiPeripheral->ConfigurationData;
  if (ConfigData && ConfigData->DeviceBaseAddress) {
    ContainVariableStorage = (ConfigData->DeviceBaseAddress <= PcdGet64 (PcdFlashNvStorageVariableBase64)) &&
      ((PcdGet32 (PcdFlashNvStorageVariableSize) + PcdGet64 (PcdFlashNvStorageVariableBase64)) <=
         (ConfigData->DeviceBaseAddress + ConfigData->FlashSize));
  }
  gBS->CloseProtocol (
         ControllerHandle,
         &gEfiCallerIdGuid,
         This->DriverBindingHandle,
         ControllerHandle
         );
  if (ContainVariableStorage) {
    return EFI_SUCCESS;
  } else {
    return EFI_UNSUPPORTED;
  }
}

/*
   Write a full or portion of a block.
   It must not span block boundaries; that is,
   Offset + NumBytes <= Context->Media.BlockSize.
   */
EFI_STATUS
SpiNorFlashWrite (
  IN        SPI_NOR_FLASH_CONTEXT   *Context,
  IN        EFI_LBA                  Lba,
  IN        UINTN                    Offset,
  IN OUT    UINTN                    NumBytes,
  IN        UINT8                    *Buffer
  )
{
  EFI_STATUS                          Status;
  UINTN                               BlockSize;
  BOOLEAN                             DoErase;
  UINT8*                              Source;
  UINTN                               Alignment;
  UINTN                               IntBytes; // Intermediate Bytes needed to copy for alignment
  SPI_NOR_PARAMS                      *SpiNorParams;
  SFDP_FLASH_PARAM                    *ParamTable;
  CONST SPI_FLASH_CONFIGURATION_DATA  *ConfigData;
  EFI_SPI_IO_PROTOCOL                 *SpiIo;
  UINT32                              Index;

  SpiIo = Context->SpiIo;
  SpiNorParams = Context->SpiNorParams;
  ParamTable = SpiNorParams->ParamTable;
  ConfigData = SpiIo->SpiPeripheral->ConfigurationData;

  DEBUG ((
    DEBUG_BLKIO, "%a(Parameters: Lba=%ld, Offset=0x%x, NumBytes=0x%x, Buffer @ 0x%08x)\n",
    __FUNCTION__,
    Lba,
    Offset,
    NumBytes,
    Buffer
    ));

  // Cache the block size to avoid de-referencing pointers all the time
  BlockSize = SFDP_PARAM_ERASE_SIZE (ParamTable);

  // For the very best performance, programming should be done in full pages
  // of page size aligned on page size boundaries with each Page being programmed only once.
  Alignment = SpiIo->MaximumTransferBytes;
  if (SpiIo->Attributes | SPI_TRANSFER_SIZE_INCLUDES_ADDRESS) {
    Alignment -= sizeof (UINT32);
  }
  if (SpiIo->Attributes | SPI_TRANSFER_SIZE_INCLUDES_OPCODE) {
    Alignment -= sizeof (UINT8);
  }
  Alignment = MIN (ConfigData->PageSize, Alignment);
  DoErase = TRUE;

  // Pick 128bytes as a good start for word operations as opposed to erasing the
  // block and writing the data regardless if an erase is really needed.
  // It looks like most individual NV variable writes are smaller than 128bytes.
  if (NumBytes <= 128) {
    Source = (UINT8 *)Context->ShadowBuffer;
    IntBytes = (Offset % Alignment); // Bytes to read before the offset
    IntBytes += (Alignment - ( (Offset + NumBytes) % Alignment)) % Alignment; // Bytes to read after the numbytes

    //First Read the data into shadow buffer from location where data is to be written
    Status = ReadFlashData (
               SpiIo,
               SpiNorParams,
               GET_BLOCK_OFFSET(Lba) + (Offset - (Offset % Alignment)),
               NumBytes + IntBytes,
               Source
               );
    if (EFI_ERROR (Status)) {
      DEBUG ((
        DEBUG_ERROR,
        "%a: ERROR - Failed to Read @ Offset 0x%x Status=%d\n",
        __FUNCTION__,
        Offset + GET_BLOCK_OFFSET(Lba),
        Status
        ));
      return Status;
    }

    // Check to see if we need to erase before programming the data into QSPI.
    // If the destination bits are only changing from 1s to 0s we can
    // just write. After a block is erased all bits in the block is set to 1.
    // If any byte requires us to erase we just give up and rewrite all of it.
    Source += (Offset % Alignment);
    DoErase = TestBitSetClear (Source, Buffer, NumBytes, TRUE);

    // if we got here then write all the data. Otherwise do the
    // Erase-Write cycle.
    if (!DoErase) {
      // Put the data at the appropriate location inside the buffer area
      CopyMem (Source, Buffer, NumBytes);

      for (Index = 0, Source = Context->ShadowBuffer;
           Index < (NumBytes + IntBytes);
           Index += Alignment, Source += Alignment) {
        Status = WriteFlashData (
                    SpiIo,
                    SpiNorParams,
                    GET_BLOCK_OFFSET(Lba) + (Offset - (Offset % Alignment)) + Index,
                    Alignment,
                    Source
                    );
        if (EFI_ERROR (Status)) {
          DEBUG ((
            DEBUG_ERROR, "%a: ERROR - Failed to Write @ Offset 0x%x Status %r\n",
            __FUNCTION__,
            GET_BLOCK_OFFSET(Lba) + (Offset - (Offset % Alignment)) + Index,
            Status
            ));
          return Status;
        }
      }
      return EFI_SUCCESS;
    }
  }

  // If we are going to write full block, no need to read block and then update bytes in it
  if (NumBytes != BlockSize) {
    // Read QSPI Flash data into shadow buffer
    Status = ReadFlashData (
               SpiIo,
               SpiNorParams,
               GET_BLOCK_OFFSET(Lba),
               BlockSize,
               Context->ShadowBuffer
               );
    if (EFI_ERROR (Status)) {
        DEBUG ((
          DEBUG_ERROR, "%a: ERROR - Failed to Read @ Offset 0x%x Status %r\n",
          __FUNCTION__,
          GET_BLOCK_OFFSET(Lba),
          Status
          ));
      // Return one of the pre-approved error statuses
      return EFI_DEVICE_ERROR;
    }
    // Put the data at the appropriate location inside the buffer area
    CopyMem ((VOID*)((UINTN)Context->ShadowBuffer + Offset), Buffer, NumBytes);
  }

  //Erase Block
  Status = EraseFlashBlock (Context->SpiIo, SpiNorParams, GET_BLOCK_OFFSET(Lba));
  if (EFI_ERROR (Status)) {
    // Return one of the pre-approved error statuses
    return EFI_DEVICE_ERROR;
  }

  if (NumBytes != BlockSize) {
    // Write the modified shadow buffer back to the SpiNorFlash
    for (Index = 0, Source = Context->ShadowBuffer;
         Index < BlockSize;
         Index += Alignment, Source += Alignment) {
      Status = WriteFlashData (
                 SpiIo,
                 SpiNorParams,
                 GET_BLOCK_OFFSET(Lba) + Index,
                 Alignment,
                 Source
                 );
      if (EFI_ERROR (Status)) {
        DEBUG ((
          DEBUG_ERROR, "%a: ERROR - Failed to Write @ Offset 0x%x Status %r\n",
          __FUNCTION__,
          GET_BLOCK_OFFSET(Lba) + Index,
          Status
          ));
        // Return one of the pre-approved error statuses
        return EFI_DEVICE_ERROR;
      }
    }
  } else {
    // Write the Buffer to an entire block in SpiNorFlash
    for (Index = 0, Source = Buffer;
         Index < BlockSize;
         Index += Alignment, Source += Alignment) {
      Status = WriteFlashData (
                 SpiIo,
                 SpiNorParams,
                 GET_BLOCK_OFFSET(Lba) + Index,
                 Alignment,
                 Source
                 );
      if (EFI_ERROR (Status)) {
        DEBUG ((
          DEBUG_ERROR, "%a: ERROR - Failed to Write @ Offset 0x%x Status %r\n",
          __FUNCTION__,
          GET_BLOCK_OFFSET(Lba) + Index,
          Status
          ));
        // Return one of the pre-approved error statuses
        return EFI_DEVICE_ERROR;
      }
    }
  }

  return EFI_SUCCESS;
}

/**
  Fixup internal data so that EFI can be call in virtual mode.
  Call the passed in Child Notify event and convert any pointers in
  lib to virtual mode.

  @param[in]    Event   The Event that is being processed
  @param[in]    Context Event Context
**/
VOID
EFIAPI
SpiNorVirtualNotifyEvent (
  IN EFI_EVENT        Event,
  IN VOID             *Context
  )
{
  SPI_NOR_FLASH_CONTEXT       *SpiNorContext;

  SpiNorContext = (SPI_NOR_FLASH_CONTEXT *)Context;

  // The other elements of SpiIo would be relocated by SPI Bus layer
  EfiConvertPointer (0x0, (VOID**)&SpiNorContext->SpiIo);

  EfiConvertPointer (0x0, (VOID**)&SpiNorContext->SpiNorParams->RequestPackets);
  EfiConvertPointer (0x0, (VOID**)&SpiNorContext->SpiNorParams->ParamTable);
  EfiConvertPointer (0x0, (VOID**)&SpiNorContext->SpiNorParams->ParamHeader);
  EfiConvertPointer (0x0, (VOID**)&SpiNorContext->SpiNorParams);

  // Convert Fvb
  EfiConvertPointer (0x0, (VOID**)&SpiNorContext->FvbProtocol.EraseBlocks);
  EfiConvertPointer (0x0, (VOID**)&SpiNorContext->FvbProtocol.GetAttributes);
  EfiConvertPointer (0x0, (VOID**)&SpiNorContext->FvbProtocol.GetBlockSize);
  EfiConvertPointer (0x0, (VOID**)&SpiNorContext->FvbProtocol.GetPhysicalAddress);
  EfiConvertPointer (0x0, (VOID**)&SpiNorContext->FvbProtocol.Read);
  EfiConvertPointer (0x0, (VOID**)&SpiNorContext->FvbProtocol.SetAttributes);
  EfiConvertPointer (0x0, (VOID**)&SpiNorContext->FvbProtocol.Write);

  EfiConvertPointer (0x0, (VOID**)&SpiNorContext->ShadowBuffer);

  return;
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

EFI_STATUS
SpiNorFlashCreateContext (
  IN  EFI_SPI_IO_PROTOCOL         *SpiIo,
  IN  SPI_NOR_PARAMS              *SpiNorParams,
  OUT SPI_NOR_FLASH_CONTEXT       **SpiNorContext
)
{
  BOOLEAN                               ContainVariableStorage;
  SFDP_FLASH_PARAM                      *ParamTable;
  CONST SPI_FLASH_CONFIGURATION_DATA    *ConfigData;
  EFI_STATUS                            Status;

  Status = EFI_SUCCESS;
  ParamTable = SpiNorParams->ParamTable;
  ConfigData = SpiIo->SpiPeripheral->ConfigurationData;
  ContainVariableStorage = FALSE;
  *SpiNorContext = NULL;

  if (ConfigData->DeviceBaseAddress) {
    ContainVariableStorage = (ConfigData->DeviceBaseAddress <= PcdGet64 (PcdFlashNvStorageVariableBase64)) &&
      ((PcdGet32 (PcdFlashNvStorageVariableSize) + PcdGet64 (PcdFlashNvStorageVariableBase64)) <=
         (ConfigData->DeviceBaseAddress + ConfigData->FlashSize));
  }

  if (!ContainVariableStorage) {
    Status = EFI_UNSUPPORTED;
    goto ErrorExit;
  }

  *SpiNorContext = (SPI_NOR_FLASH_CONTEXT *)AllocateRuntimeCopyPool (
                                              sizeof (SPI_NOR_FLASH_CONTEXT),
                                              &mSpiNorFlashContextTemplate
                                              );
  if (*SpiNorContext == NULL) {
    Status = EFI_OUT_OF_RESOURCES;
    goto ErrorExit;
  }

  (*SpiNorContext)->ShadowBuffer = AllocateRuntimeZeroPool (SFDP_PARAM_ERASE_SIZE(ParamTable));
  if ((*SpiNorContext)->ShadowBuffer == NULL) {
    Status = EFI_OUT_OF_RESOURCES;
    goto ErrorExit;
  }

  (*SpiNorContext)->SpiIo = SpiIo;
  (*SpiNorContext)->SpiNorParams = SpiNorParams;

  Status = SpiNorFlashFvbInitialize (*SpiNorContext);
  if (EFI_ERROR (Status)) {
    goto ErrorExit;
  }

  //
  // Register for the virtual address change event
  //
  Status = gBS->CreateEventEx (
                  EVT_NOTIFY_SIGNAL,
                  TPL_NOTIFY,
                  SpiNorVirtualNotifyEvent,
                  (VOID *)*SpiNorContext,
                  &gEfiEventVirtualAddressChangeGuid,
                  &(*SpiNorContext)->SpiNorVirtualAddressEvent
                  );
  ASSERT_EFI_ERROR (Status);

ErrorExit:
  if (EFI_ERROR (Status)) {
    if (*SpiNorContext) {
      if ((*SpiNorContext)->ShadowBuffer) {
        FreePool ((*SpiNorContext)->ShadowBuffer);
      }

      FreePool (*SpiNorContext);
      *SpiNorContext = NULL;
    }
  }

  return Status;
}

EFI_STATUS
SpiNorFlashStart (
  IN EFI_DRIVER_BINDING_PROTOCOL   *This,
  IN EFI_HANDLE                    ControllerHandle,
  IN EFI_DEVICE_PATH_PROTOCOL      *RemainingDevicePath OPTIONAL
  )
{
  EFI_STATUS                        Status;
  EFI_SPI_IO_PROTOCOL               *SpiIo;
  UINT32                            RequestPacketsBufferSize;
  SPI_NOR_PARAMS                    *SpiNorParams;
  SFDP_FLASH_PARAM                  *ParamTable;
  SPI_FLASH_CONFIGURATION_DATA      *ConfigData;
  EFI_SPI_PERIPHERAL                *UpdatedSpiPeripheral;
  SPI_NOR_FLASH_CONTEXT             *SpiNorContext;

  Status = EFI_SUCCESS;
  SpiIo = NULL;
  SpiNorParams = NULL;
  UpdatedSpiPeripheral = NULL;
  ConfigData = NULL;
  SpiNorContext = NULL;

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
  SpiNorParams = (SPI_NOR_PARAMS *)AllocateRuntimeZeroPool (sizeof (SPI_NOR_PARAMS));
  if (!SpiNorParams) {
    Status = EFI_OUT_OF_RESOURCES;
    goto ErrorExit;
  }
  SpiNorParams->RequestPackets = (EFI_SPI_REQUEST_PACKET *)AllocateRuntimeZeroPool (RequestPacketsBufferSize);
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

  // Now that we know the size of flash call SpiBusUpdateSpiPeripheral
  UpdatedSpiPeripheral = (EFI_SPI_PERIPHERAL *)AllocateRuntimeCopyPool (
                                                 sizeof (EFI_SPI_PERIPHERAL),
                                                 SpiIo->SpiPeripheral
                                                 );
  if (!UpdatedSpiPeripheral) {
    Status = EFI_OUT_OF_RESOURCES;
    goto ErrorExit;
  }

  ConfigData = (SPI_FLASH_CONFIGURATION_DATA *)AllocateRuntimeZeroPool (
                                                 sizeof (SPI_FLASH_CONFIGURATION_DATA)
                                               );
  if (!ConfigData) {
    Status = EFI_OUT_OF_RESOURCES;
    goto ErrorExit;
  }

  ConfigData->Signature = SPI_NOR_FLASH_SIGNATURE;
  ConfigData->PageSize = SFDP_PARAM_PAGE_SIZE(ParamTable);
  ConfigData->FlashSize = SFDP_PARAM_FLASH_SIZE(ParamTable);

  UpdatedSpiPeripheral->ConfigurationData = ConfigData;

  Status = SpiIo->UpdateSpiPeripheral (
                    SpiIo,
                    UpdatedSpiPeripheral
                    );
  if (EFI_ERROR (Status)) {
    goto ErrorExit;
  }

  Status = SpiNorFlashCreateContext (SpiIo, SpiNorParams, &SpiNorContext);
  if (EFI_ERROR (Status)) {
    goto ErrorExit;
  }

  Status = gBS->InstallMultipleProtocolInterfaces (
                  &ControllerHandle,
                  &gEfiFirmwareVolumeBlockProtocolGuid, &SpiNorContext->FvbProtocol,
                  NULL
                  );
  ASSERT_EFI_ERROR (Status);

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
    if (SpiNorParams) {
      if (SpiNorParams->ParamTable) {
        FreePool (SpiNorParams->ParamTable);
      }
      if (SpiNorParams->ParamHeader) {
        FreePool (SpiNorParams->ParamHeader);
      }
      if (SpiNorParams->RequestPackets) {
        FreePool (SpiNorParams->RequestPackets);
      }
      FreePool (SpiNorParams);
    }
    if (SpiNorContext) {
      if (SpiNorContext->ShadowBuffer) {
        FreePool (SpiNorContext->ShadowBuffer);
      }
      FreePool (SpiNorContext);
    }
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
