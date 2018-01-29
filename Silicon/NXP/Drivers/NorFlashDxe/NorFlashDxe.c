/** @file

  Based on NorFlash implementation available in
  ArmPlatformPkg/Drivers/NorFlashDxe/NorFlashDxe.c

  Copyright (c) 2011 - 2014, ARM Ltd. All rights reserved.
  Copyright 2017 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include <Bitops.h>
#include <Library/BaseMemoryLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/NorFlashLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiRuntimeLib.h>

#include "NorFlashDxe.h"

STATIC EFI_EVENT mNorFlashVirtualAddrChangeEvent;

//
// Global variable declarations
//
NOR_FLASH_INSTANCE **mNorFlashInstances;
UINT32               mNorFlashDeviceCount;

NOR_FLASH_INSTANCE  mNorFlashInstanceTemplate = {
  .Signature = NOR_FLASH_SIGNATURE,
  .Initialized = FALSE,
  .Initialize = NULL,
  .StartLba = 0,
  .BlockIoProtocol = {
    .Revision = EFI_BLOCK_IO_PROTOCOL_REVISION2,
    .Reset = NorFlashBlockIoReset,
    .ReadBlocks = NorFlashBlockIoReadBlocks,
    .WriteBlocks = NorFlashBlockIoWriteBlocks,
    .FlushBlocks = NorFlashBlockIoFlushBlocks,
  },

  .Media = {
    .RemovableMedia = FALSE,
    .MediaPresent = TRUE,
    .LogicalPartition = FALSE,
    .ReadOnly = FALSE,
    .WriteCaching = FALSE,
    .IoAlign = 4,
    .LowestAlignedLba = 0,
    .LogicalBlocksPerPhysicalBlock = 1,
  },

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
  .ShadowBuffer = NULL,
  .DevicePath = {
    .Vendor = {
      .Header = {
        .Type = HARDWARE_DEVICE_PATH,
        .SubType = HW_VENDOR_DP,
        .Length = {(UINT8)sizeof (VENDOR_DEVICE_PATH),
            (UINT8)((sizeof (VENDOR_DEVICE_PATH)) >> 8) }
      },
      .Guid = EFI_CALLER_ID_GUID, // GUID ... NEED TO BE FILLED
    },
    .End = {
      .Type = END_DEVICE_PATH_TYPE,
      .SubType = END_ENTIRE_DEVICE_PATH_SUBTYPE,
      .Length = { sizeof (EFI_DEVICE_PATH_PROTOCOL), 0 }
    }
  }
};

EFI_STATUS
NorFlashCreateInstance (
  IN UINTN                  NorFlashDeviceBase,
  IN UINTN                  NorFlashRegionBase,
  IN UINTN                  NorFlashSize,
  IN UINT32                 MediaId,
  IN UINT32                 BlockSize,
  IN BOOLEAN                SupportFvb,
  OUT NOR_FLASH_INSTANCE**  NorFlashInstance
  )
{
  EFI_STATUS               Status;
  NOR_FLASH_INSTANCE*      Instance;

  ASSERT (NorFlashInstance != NULL);

  Instance = AllocateRuntimeCopyPool (sizeof (NOR_FLASH_INSTANCE),
                            &mNorFlashInstanceTemplate);
  if (Instance == NULL) {
    return EFI_OUT_OF_RESOURCES;
  }

  Instance->DeviceBaseAddress = NorFlashDeviceBase;
  Instance->RegionBaseAddress = NorFlashRegionBase;
  Instance->Size = NorFlashSize;

  Instance->BlockIoProtocol.Media = &Instance->Media;
  Instance->Media.MediaId = MediaId;
  Instance->Media.BlockSize = BlockSize;
  Instance->Media.LastBlock = (NorFlashSize / BlockSize)-1;

  Instance->ShadowBuffer = AllocateRuntimePool (BlockSize);
  if (Instance->ShadowBuffer == NULL) {
    FreePool (Instance);
    return EFI_OUT_OF_RESOURCES;
  }

  if (SupportFvb) {
    Instance->SupportFvb = TRUE;
    Instance->Initialize = NorFlashFvbInitialize;

    Status = gBS->InstallMultipleProtocolInterfaces (
             &Instance->Handle,
             &gEfiDevicePathProtocolGuid, &Instance->DevicePath,
             &gEfiBlockIoProtocolGuid,  &Instance->BlockIoProtocol,
             &gEfiFirmwareVolumeBlockProtocolGuid, &Instance->FvbProtocol,
             NULL
             );
    if (EFI_ERROR (Status)) {
       FreePool (Instance->ShadowBuffer);
       FreePool (Instance);
       return Status;
    }
  } else {
    Instance->Initialized = TRUE;

    Status = gBS->InstallMultipleProtocolInterfaces (
          &Instance->Handle,
          &gEfiDevicePathProtocolGuid, &Instance->DevicePath,
          &gEfiBlockIoProtocolGuid,  &Instance->BlockIoProtocol,
          NULL
          );
    if (EFI_ERROR (Status)) {
      FreePool (Instance->ShadowBuffer);
      FreePool (Instance);
      return Status;
    }
  }

  *NorFlashInstance = Instance;

  return Status;
}

/*
   Write a full or portion of a block.
   It must not span block boundaries; that is,
   Offset + NumBytes <= Instance->Media.BlockSize.
   */
EFI_STATUS
NorFlashWrite (
  IN        NOR_FLASH_INSTANCE   *Instance,
  IN        EFI_LBA               Lba,
  IN        UINTN                 Offset,
  IN OUT    UINTN                 *NumBytes,
  IN        UINT8                 *Buffer
)
{
  EFI_STATUS                      Status;
  UINTN                           BlockSize;
  BOOLEAN                         DoErase;
  VOID                            *Source;
  UINTN                           SectorAddress;

  Status = EFI_SUCCESS;
  Source = NULL;

  DEBUG ((DEBUG_BLKIO, "%a(Parameters: Lba=%ld, Offset=0x%x, NumBytes=0x%x, "
                       "Buffer @ 0x%08x)\n", __FUNCTION__,
                       Lba, Offset, *NumBytes, Buffer));

  // The buffer must be valid
  if (Buffer == NULL) {
    return EFI_INVALID_PARAMETER;
  }

  // Detect WriteDisabled state
  if (Instance->Media.ReadOnly == TRUE) {
    DEBUG ((DEBUG_ERROR, "NorFlashWrite: ERROR - Can not write: "
                         "Device is in WriteDisabled state.\n"));
    // It is in WriteDisabled state, return an error right away
    return EFI_ACCESS_DENIED;
  }

  // Cache the block size to avoid de-referencing pointers all the time
  BlockSize = Instance->Media.BlockSize;

  // We must have some bytes to write
  if ((*NumBytes == 0) || (*NumBytes > BlockSize)) {
    DEBUG ((DEBUG_ERROR, "NorFlashWrite: ERROR - EFI_BAD_BUFFER_SIZE: "
                         "(Offset=0x%x + NumBytes=0x%x) > BlockSize=0x%x\n", \
                         Offset, *NumBytes, BlockSize ));
    return EFI_BAD_BUFFER_SIZE;
  }

  if (((Lba * BlockSize) + Offset + *NumBytes) > Instance->Size) {
    DEBUG ((DEBUG_ERROR, "%a: ERROR - Write will exceed device size.\n",
                         __FUNCTION__));
    return EFI_INVALID_PARAMETER;
  }

  // Check we did get some memory. Buffer is BlockSize.
  if (Instance->ShadowBuffer == NULL) {
    DEBUG ((DEBUG_ERROR, "FvbWrite: ERROR - Buffer not ready\n"));
    return EFI_DEVICE_ERROR;
  }

  SectorAddress = GET_NOR_BLOCK_ADDRESS (
                         Instance->RegionBaseAddress,
                         Lba,
                         Instance->Media.BlockSize);

  // Pick 128bytes as a good start for word operations as opposed to erasing the
  // block and writing the data regardless if an erase is really needed.
  // It looks like most individual NV variable writes are smaller than 128bytes.
  if (*NumBytes <= 128) {
    Source = Instance->ShadowBuffer;
    //First Read the data into shadow buffer from location where data is to be written
    Status = NorFlashPlatformRead (
                        Instance,
                        Lba,
                        Offset,
                        *NumBytes,
                        Source);
    if (EFI_ERROR (Status)) {
      DEBUG ((DEBUG_ERROR, "%a: ERROR - Failed to "
                           "Read @ %p Status=%d\n", __FUNCTION__,
                           Offset + SectorAddress, Status));
      return Status;
    }
    // Check to see if we need to erase before programming the data into NorFlash.
    // If the destination bits are only changing from 1s to 0s we can
    // just write. After a block is erased all bits in the block is set to 1.
    // If any byte requires us to erase we just give up and rewrite all of it.
    DoErase = TestBitSetClear (Source, Buffer, *NumBytes, TRUE);

    // if we got here then write all the data. Otherwise do the
    // Erase-Write cycle.
    if (!DoErase) {
      Status = NorFlashPlatformWriteBuffer (
                        Instance,
                        Lba,
                        Offset,
                        NumBytes,
                        Buffer);
      if (EFI_ERROR (Status)) {
        DEBUG ((DEBUG_ERROR, "%a: ERROR - Failed to "
                             "Write @ %p Status=%d\n", __FUNCTION__,
                             Offset + SectorAddress, Status));
        return Status;
      }
      return EFI_SUCCESS;
    }
  }

  // If we are not going to write full block, read block and then update bytes in it
  if (*NumBytes != BlockSize) {
    // Read NorFlash Flash data into shadow buffer
    Status = NorFlashBlockIoReadBlocks (
                        &(Instance->BlockIoProtocol),
                        Instance->Media.MediaId,
                        Lba,
                        BlockSize,
                        Instance->ShadowBuffer);
    if (EFI_ERROR (Status)) {
      // Return one of the pre-approved error statuses
      return EFI_DEVICE_ERROR;
    }
    // Put the data at the appropriate location inside the buffer area
    CopyMem ((VOID *)((UINTN)Instance->ShadowBuffer + Offset), Buffer, *NumBytes);
  }
  //Erase Block
  Status = NorFlashPlatformEraseSector (Instance, SectorAddress);
  if (EFI_ERROR (Status)) {
    // Return one of the pre-approved error statuses
    return EFI_DEVICE_ERROR;
  }
  if (*NumBytes != BlockSize) {
    // Write the modified shadow buffer back to the NorFlash
    Status = NorFlashPlatformWriteBuffer (
                        Instance,
                        Lba,
                        0,
                        &BlockSize,
                        Instance->ShadowBuffer);
  } else {
    // Write the Buffer to an entire block in NorFlash
    Status = NorFlashPlatformWriteBuffer (
                        Instance,
                        Lba,
                        0,
                        &BlockSize,
                        Buffer);
  }
  if (EFI_ERROR (Status)) {
    // Return one of the pre-approved error statuses
    return EFI_DEVICE_ERROR;
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
NorFlashVirtualNotifyEvent (
  IN EFI_EVENT        Event,
  IN VOID             *Context
  )
{
  UINTN Index;

  for (Index = 0; Index < mNorFlashDeviceCount; Index++) {
    EfiConvertPointer (0x0, (VOID**)&mNorFlashInstances[Index]->DeviceBaseAddress);
    EfiConvertPointer (0x0, (VOID**)&mNorFlashInstances[Index]->RegionBaseAddress);

    // Convert BlockIo protocol
    EfiConvertPointer (0x0, (VOID**)&mNorFlashInstances[Index]->BlockIoProtocol.FlushBlocks);
    EfiConvertPointer (0x0, (VOID**)&mNorFlashInstances[Index]->BlockIoProtocol.ReadBlocks);
    EfiConvertPointer (0x0, (VOID**)&mNorFlashInstances[Index]->BlockIoProtocol.Reset);
    EfiConvertPointer (0x0, (VOID**)&mNorFlashInstances[Index]->BlockIoProtocol.WriteBlocks);

    // Convert Fvb
    EfiConvertPointer (0x0, (VOID**)&mNorFlashInstances[Index]->FvbProtocol.EraseBlocks);
    EfiConvertPointer (0x0, (VOID**)&mNorFlashInstances[Index]->FvbProtocol.GetAttributes);
    EfiConvertPointer (0x0, (VOID**)&mNorFlashInstances[Index]->FvbProtocol.GetBlockSize);
    EfiConvertPointer (0x0, (VOID**)&mNorFlashInstances[Index]->FvbProtocol.GetPhysicalAddress);
    EfiConvertPointer (0x0, (VOID**)&mNorFlashInstances[Index]->FvbProtocol.Read);
    EfiConvertPointer (0x0, (VOID**)&mNorFlashInstances[Index]->FvbProtocol.SetAttributes);
    EfiConvertPointer (0x0, (VOID**)&mNorFlashInstances[Index]->FvbProtocol.Write);
    if (mNorFlashInstances[Index]->ShadowBuffer != NULL) {
      EfiConvertPointer (0x0, (VOID **)&mNorFlashInstances[Index]->ShadowBuffer);
    }
  }

  return;
}

EFI_STATUS
EFIAPI
NorFlashInitialise (
  IN EFI_HANDLE       ImageHandle,
  IN EFI_SYSTEM_TABLE *SystemTable
  )
{
  EFI_STATUS           Status;
  UINT32               Index;
  NorFlashDescription* NorFlashDevices;
  BOOLEAN              ContainVariableStorage;

  ContainVariableStorage = 0;

  Status = NorFlashPlatformGetDevices (&NorFlashDevices, &mNorFlashDeviceCount);
  if (EFI_ERROR(Status)) {
    DEBUG((DEBUG_ERROR, "%a : Failed to get Nor devices (0x%x)\n",
                        __FUNCTION__,  Status));
    return Status;
  }

  Status = NorFlashPlatformFlashGetAttributes (NorFlashDevices, mNorFlashDeviceCount);
  if (EFI_ERROR(Status)) {
    DEBUG ((DEBUG_ERROR, "%a : Failed to get NOR device attributes (0x%x)\n",
                         __FUNCTION__, Status));
    ASSERT_EFI_ERROR (Status); // System becomes unusable if NOR flash is not detected
    return Status;
  }

  mNorFlashInstances = AllocateRuntimePool (
                            sizeof(NOR_FLASH_INSTANCE*) * mNorFlashDeviceCount);
  if (mNorFlashInstances == NULL) {
    DEBUG ((DEBUG_ERROR, "%a : Failed to allocate runtime  memory \n"));
    return EFI_OUT_OF_RESOURCES;
  }

  for (Index = 0; Index < mNorFlashDeviceCount; Index++) {
    // Check if this NOR Flash device contain the variable storage region
    ContainVariableStorage =
      (NorFlashDevices[Index].RegionBaseAddress <= PcdGet64 (PcdFlashNvStorageVariableBase64)) &&
      (PcdGet64 (PcdFlashNvStorageVariableBase64) + PcdGet32 (PcdFlashNvStorageVariableSize) <=
       NorFlashDevices[Index].RegionBaseAddress + NorFlashDevices[Index].Size);

    Status = NorFlashCreateInstance (
                        NorFlashDevices[Index].DeviceBaseAddress,
                        NorFlashDevices[Index].RegionBaseAddress,
                        NorFlashDevices[Index].Size,
                        Index,
                        NorFlashDevices[Index].BlockSize,
                        ContainVariableStorage,
                        &mNorFlashInstances[Index]);

    if (EFI_ERROR (Status)) {
      DEBUG ((DEBUG_ERROR, "%a : Failed to create instance for "
                           "NorFlash[%d] (0x%x)\n",Index, Status));
    }
  }

  //
  // Register for the virtual address change event
  //
  Status = gBS->CreateEventEx (
                        EVT_NOTIFY_SIGNAL,
                        TPL_NOTIFY,
                        NorFlashVirtualNotifyEvent,
                        NULL,
                        &gEfiEventVirtualAddressChangeGuid,
                        &mNorFlashVirtualAddrChangeEvent);

  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "Failed to create VirtualAddressChange event 0x%x\n", Status));
  }

  return Status;
}
