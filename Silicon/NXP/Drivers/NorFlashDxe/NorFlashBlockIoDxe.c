/** @NorFlashBlockIoDxe.c

  Based on NorFlash implementation available in
  ArmPlatformPkg/Drivers/NorFlashDxe/NorFlashBlockIoDxe.c

  Copyright (c) 2011 - 2014, ARM Ltd. All rights reserved.
  Copyright 2017 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include <Library/DebugLib.h>
#include <Library/NorFlashLib.h>

#include <NorFlash.h>
#include "NorFlashDxe.h"

//
// BlockIO Protocol function EFI_BLOCK_IO_PROTOCOL.Reset
//
EFI_STATUS
EFIAPI
NorFlashBlockIoReset (
  IN EFI_BLOCK_IO_PROTOCOL  *This,
  IN BOOLEAN                ExtendedVerification
  )
{
  NOR_FLASH_INSTANCE        *Instance;

  Instance = INSTANCE_FROM_BLKIO_THIS (This);

  DEBUG ((DEBUG_INFO, "NorFlashBlockIoReset (MediaId=0x%x)\n",
                            This->Media->MediaId));

  return NorFlashPlatformReset (Instance->DeviceBaseAddress);
}

//
// BlockIO Protocol function EFI_BLOCK_IO_PROTOCOL.ReadBlocks
//
EFI_STATUS
EFIAPI
NorFlashBlockIoReadBlocks (
  IN  EFI_BLOCK_IO_PROTOCOL   *This,
  IN  UINT32                  MediaId,
  IN  EFI_LBA                 Lba,
  IN  UINTN                   BufferSizeInBytes,
  OUT VOID                    *Buffer
  )
{
  NOR_FLASH_INSTANCE          *Instance;
  EFI_STATUS                  Status;
  EFI_BLOCK_IO_MEDIA          *Media;
  UINTN                       NumBlocks;
  UINT8                       *ReadBuffer;
  UINTN                       BlockCount;
  UINTN                       BlockSizeInBytes;
  EFI_LBA                     CurrentBlock;

  Status = EFI_SUCCESS;

  if ((This == NULL) || (Buffer == NULL)) {
    return EFI_INVALID_PARAMETER;
  }

  Instance = INSTANCE_FROM_BLKIO_THIS (This);
  Media = This->Media;

  if (Media  == NULL) {
    DEBUG ((DEBUG_ERROR, "%a : Media is NULL\n", __FUNCTION__));
    return EFI_INVALID_PARAMETER;
  }

  NumBlocks = ((UINTN)BufferSizeInBytes) / Instance->Media.BlockSize ;

  DEBUG ((DEBUG_BLKIO, "%a : (MediaId=0x%x, Lba=%ld, "
                           "BufferSize=0x%x bytes (%d kB)"
                           ", BufferPtr @ 0x%p)\n",
                           __FUNCTION__,MediaId, Lba,
                           BufferSizeInBytes, Buffer));

  if (!Media) {
    Status = EFI_INVALID_PARAMETER;
  }
  else if (!Media->MediaPresent) {
    Status = EFI_NO_MEDIA;
  }
  else if (Media->MediaId != MediaId) {
    Status = EFI_MEDIA_CHANGED;
  }
  else if ((Media->IoAlign >= 2) &&
          (((UINTN)Buffer & (Media->IoAlign - 1)) != 0)) {
    Status = EFI_INVALID_PARAMETER;
  }
  else if (BufferSizeInBytes == 0) {
    // Return if we have not any byte to read
    Status = EFI_SUCCESS;
  }
  else if ((BufferSizeInBytes % Media->BlockSize) != 0) {
    // The size of the buffer must be a multiple of the block size
    DEBUG ((DEBUG_ERROR, "%a : BlockSize in bytes = 0x%x\n",__FUNCTION__,
                     BufferSizeInBytes));
    Status = EFI_INVALID_PARAMETER;
  } else if ((Lba + NumBlocks - 1) > Media->LastBlock) {
    // All blocks must be within the device
    DEBUG ((DEBUG_ERROR, "%a : Read will exceed last block %d, %d, %d \n",
                __FUNCTION__, Lba, NumBlocks, Media->LastBlock));
    Status = EFI_INVALID_PARAMETER;
  } else {
    BlockSizeInBytes = Instance->Media.BlockSize;

    /* Because the target *Buffer is a pointer to VOID,
     * we must put all the data into a pointer
     * to a proper data type, so use *ReadBuffer */
    ReadBuffer = (UINT8 *)Buffer;

    CurrentBlock = Lba;
    // Read data block by Block
    for (BlockCount = 0; BlockCount < NumBlocks; BlockCount++, CurrentBlock++,
            ReadBuffer = ReadBuffer + BlockSizeInBytes) {
      DEBUG ((DEBUG_BLKIO, "%a: Reading block #%d\n",
                  __FUNCTION__,(UINTN)CurrentBlock));

      Status = NorFlashPlatformRead (Instance, CurrentBlock, (UINTN)0 ,
                                   BlockSizeInBytes,ReadBuffer);
      if (EFI_ERROR (Status)) {
        break;
      }
    }
  }
  DEBUG ((DEBUG_BLKIO,"%a: Exit Status = \"%r\".\n",__FUNCTION__,Status));

  return Status;
}

//
// BlockIO Protocol function EFI_BLOCK_IO_PROTOCOL.WriteBlocks
//
EFI_STATUS
EFIAPI
NorFlashBlockIoWriteBlocks (
  IN  EFI_BLOCK_IO_PROTOCOL   *This,
  IN  UINT32                  MediaId,
  IN  EFI_LBA                 Lba,
  IN  UINTN                   BufferSizeInBytes,
  IN  VOID                    *Buffer
  )
{
  NOR_FLASH_INSTANCE          *Instance;
  EFI_STATUS                   Status;
  EFI_BLOCK_IO_MEDIA           *Media;
  UINTN                        NumBlocks;
  EFI_LBA                      CurrentBlock;
  UINTN                        BlockSizeInBytes;
  UINT32                       BlockCount;
  UINTN                        SectorAddress;
  UINT8                        *WriteBuffer;

  Status = EFI_SUCCESS;

  if ((This == NULL) || (Buffer == NULL)) {
    return EFI_INVALID_PARAMETER;
  }

  Instance = INSTANCE_FROM_BLKIO_THIS (This);
  Media = This->Media;

  if (Media  == NULL) {
    DEBUG ((DEBUG_ERROR, "%a : Media is NULL\n",  __FUNCTION__));
    return EFI_INVALID_PARAMETER;
  }

  NumBlocks = ((UINTN)BufferSizeInBytes) / Instance->Media.BlockSize ;

  DEBUG ((DEBUG_BLKIO, "%a : (MediaId=0x%x, Lba=%ld, BufferSize=0x%x "
              "bytes (%d kB) BufferPtr @ 0x%08x)\n",
              __FUNCTION__,MediaId, Lba,BufferSizeInBytes, Buffer));

  if (!Media->MediaPresent) {
    Status = EFI_NO_MEDIA;
  }
  else if (Media->MediaId != MediaId) {
    Status = EFI_MEDIA_CHANGED;
  }
  else if (Media->ReadOnly) {
    Status = EFI_WRITE_PROTECTED;
  }
  else if (BufferSizeInBytes == 0) {
    Status = EFI_BAD_BUFFER_SIZE;
  }
  else if ((BufferSizeInBytes % Media->BlockSize) != 0) {
    // The size of the buffer must be a multiple of the block size
    DEBUG ((DEBUG_ERROR, "%a : BlockSize in bytes = 0x%x\n",__FUNCTION__,
                     BufferSizeInBytes));
    Status = EFI_INVALID_PARAMETER;
  } else if ((Lba + NumBlocks - 1) > Media->LastBlock) {
    // All blocks must be within the device
    DEBUG ((DEBUG_ERROR, "%a: Write will exceed last block %d, %d, %d  \n",
                __FUNCTION__,Lba, NumBlocks, Media->LastBlock));
    Status = EFI_INVALID_PARAMETER;
  } else {
    BlockSizeInBytes = Instance->Media.BlockSize;

    WriteBuffer = (UINT8 *)Buffer;

    CurrentBlock = Lba;
    // Program data block by Block
    for (BlockCount = 0; BlockCount < NumBlocks;
            BlockCount++, CurrentBlock++,
            WriteBuffer = (WriteBuffer + BlockSizeInBytes)) {
      DEBUG ((DEBUG_BLKIO, "%a: Writing block #%d\n",
                  __FUNCTION__,(UINTN)CurrentBlock));
      // Erase the Block(Sector) to be written to
      SectorAddress = GET_NOR_BLOCK_ADDRESS (
                           Instance->RegionBaseAddress,
                           CurrentBlock,
                           Instance->Media.BlockSize
                           );
      Status = NorFlashPlatformEraseSector (Instance, (UINTN)SectorAddress);
      if (EFI_ERROR (Status)) {
        DEBUG ((DEBUG_ERROR, "%a: Failed to erase Target 0x%x (0x%x) \n",
                   __FUNCTION__,SectorAddress, Status));
        break;
      }
      // Program Block(Sector) to be written to
      Status = NorFlashWrite (Instance, CurrentBlock, (UINTN)0,
                     &BlockSizeInBytes, WriteBuffer);
      if (EFI_ERROR (Status)) {
        break;
      }
    }
  }
  DEBUG ((DEBUG_BLKIO, "%a: Exit Status = \"%r\".\n",__FUNCTION__,Status));
  return Status;
}

//
// BlockIO Protocol function EFI_BLOCK_IO_PROTOCOL.FlushBlocks
//
EFI_STATUS
EFIAPI
NorFlashBlockIoFlushBlocks (
  IN EFI_BLOCK_IO_PROTOCOL  *This
  )
{

  DEBUG ((DEBUG_BLKIO, "%a NOT IMPLEMENTED (not required)\n", __FUNCTION__));

  // Nothing to do so just return without error
  return EFI_SUCCESS;
}
