/** @NorFlashLib.c

  Based on NorFlash implementation available in NorFlashDxe.c

  Copyright (c) 2011 - 2014, ARM Ltd. All rights reserved.
  Copyright (c) 2015, Freescale Semiconductor, Inc. All rights reserved.
  Copyright 2017 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include <PiDxe.h>
#include <Library/BaseMemoryLib/MemLibInternals.h>
#include <Library/TimerLib.h>

#include "CfiCommand.h"
#include "CfiNorFlashLib.h"

#define GET_BLOCK_OFFSET(Lba) ((Instance->RegionBaseAddress)-\
                               (Instance->DeviceBaseAddress)+((UINTN)((Lba) * Instance->Media.BlockSize)))

NorFlashDescription mNorFlashDevices[NOR_FLASH_DEVICE_COUNT];

STATIC VOID
UnlockEraseAddress (
  IN  UINTN  DeviceBaseAddress
  )
{  // Issue the Unlock cmds
  SEND_NOR_COMMAND (DeviceBaseAddress, CMD_UNLOCK_1_ADDR,
                   CMD_ERASE_FIRST);

  SEND_NOR_COMMAND (DeviceBaseAddress, CMD_UNLOCK_2_ADDR,
                   CMD_ERASE_SECOND);

  // Issue a setup command
  SEND_NOR_COMMAND (DeviceBaseAddress, CMD_UNLOCK_1_ADDR,
                   CMD_ERASE_THIRD);

  // Issue the Unlock cmds
  SEND_NOR_COMMAND (DeviceBaseAddress, CMD_UNLOCK_1_ADDR,
                   CMD_ERASE_FOURTH);

  SEND_NOR_COMMAND (DeviceBaseAddress, CMD_UNLOCK_2_ADDR,
                   CMD_ERASE_FIFTH);

  return;
}

STATIC
UINT64
ConvertMicroSecondsToTicks (
  IN  UINTN  MicroSeconds
  )
{
  UINT64     TimerTicks64;

  TimerTicks64 = 0;

  // Calculate counter ticks that represent requested delay:
  //  = MicroSeconds x TICKS_PER_MICRO_SEC
  //  = MicroSeconds x Timer Frequency(in Hz) x 10^-6
  // GetPerformanceCounterProperties = Get Arm Timer Frequency in Hz
  TimerTicks64 = DivU64x32 (
                   MultU64x64 (
                     MicroSeconds,
                     GetPerformanceCounterProperties (NULL, NULL)
                     ),
                   1000000U
                   );
  return TimerTicks64;
}

/**
 * The following function erases a NOR flash sector.
 **/
EFI_STATUS
NorFlashPlatformEraseSector (
  IN NOR_FLASH_INSTANCE     *Instance,
  IN UINTN                  SectorAddress
  )
{
  FLASH_DATA                EraseStatus1;
  FLASH_DATA                EraseStatus2;
  UINT64                    Timeout;
  UINT64                    SystemCounterVal;

  EraseStatus1 = 0;
  EraseStatus2 = 0;
  Timeout = 0;

  Timeout = ConvertMicroSecondsToTicks (
                   mNorFlashDevices[Instance->Media.MediaId].BlockEraseTimeOut);
  // Request a sector erase by writing two unlock cycles, followed by a
  // setup command and two additional unlock cycles

  UnlockEraseAddress (Instance->DeviceBaseAddress);

  // Now send the address of the sector to be erased
  SEND_NOR_COMMAND (SectorAddress, 0, CMD_SECTOR_ERASE_SIXTH);

  // Wait for erase to complete
  // Read Sector start address twice to detect bit toggle and to
  // determine ERASE DONE (all bits are 1)
  // Get the maximum timer ticks needed to complete the operation
  // Check if operation is complete or not in continous loop?
  // if complete, exit from loop
  // if not check the ticks that have been passed from the begining of loop
  // if Maximum Ticks allocated for operation has passed exit from loop

  SystemCounterVal = GetPerformanceCounter ();
  Timeout += SystemCounterVal;
  while (SystemCounterVal < Timeout) {
    if ((EraseStatus1 = FLASH_READ (SectorAddress)) ==
            (EraseStatus2 = FLASH_READ (SectorAddress))) {
      if (0xFFFF == FLASH_READ (SectorAddress)) {
        break;
      }
    }
    SystemCounterVal = GetPerformanceCounter ();
  }

  if (SystemCounterVal >= Timeout) {
    DEBUG ((DEBUG_ERROR, "%a :Failed to Erase @ SectorAddress 0x%p, Timeout\n",
                __FUNCTION__, SectorAddress));
    return EFI_DEVICE_ERROR;
  } else {
    return EFI_SUCCESS;
  }
}

EFI_STATUS
NorFlashPlatformWriteWord  (
  IN NOR_FLASH_INSTANCE   *Instance,
  IN UINTN                WordOffset,
  IN FLASH_DATA           Word
  )
{
  UINT64                  Timeout;
  UINTN                   TargetAddress;
  UINT64                  SystemCounterVal;
  FLASH_DATA              Read1;
  FLASH_DATA              Read2;

  Timeout = 0;

  Timeout = ConvertMicroSecondsToTicks (
              mNorFlashDevices[Instance->Media.MediaId].WordWriteTimeOut);

  TargetAddress = CREATE_NOR_ADDRESS (Instance->DeviceBaseAddress,
              CREATE_BYTE_OFFSET (WordOffset));

  // Issue the Unlock cmds
  SEND_NOR_COMMAND (Instance->DeviceBaseAddress, CMD_UNLOCK_1_ADDR,
                   CMD_PROGRAM_FIRST);

  SEND_NOR_COMMAND (Instance->DeviceBaseAddress, CMD_UNLOCK_2_ADDR,
                   CMD_PROGRAM_SECOND);

  SEND_NOR_COMMAND (Instance->DeviceBaseAddress, CMD_UNLOCK_1_ADDR,
                   CMD_PROGRAM_THIRD);

  FLASH_WRITE_DATA (TargetAddress, Word);

  // Wait for Write to Complete
  // Read the last written address twice to detect bit toggle and
  // to determine if date is wriiten successfully or not ?
  // Get the maximum timer ticks needed to complete the operation
  // Check if operation is complete or not in continous loop?
  // if complete, exit from loop
  // if not check the ticks that have been passed from the begining of loop
  // if Maximum Ticks allocated for operation has passed, then exit from loop

  SystemCounterVal = GetPerformanceCounter ();
  Timeout += SystemCounterVal;
  while (SystemCounterVal < Timeout) {
    if ((Read1 = FLASH_READ_DATA (TargetAddress)) ==
            (Read2 = FLASH_READ_DATA (TargetAddress))) {
      if (Word == FLASH_READ_DATA (TargetAddress)) {
        break;
      }
    }
    SystemCounterVal = GetPerformanceCounter ();
  }

  if (SystemCounterVal >= Timeout) {
    DEBUG ((DEBUG_ERROR, "%a: Failed to  Write @ TargetAddress 0x%p, Timeout\n",
                __FUNCTION__, TargetAddress));
    return EFI_DEVICE_ERROR;
  } else {
    return EFI_SUCCESS;
  }
}

EFI_STATUS
NorFlashPlatformWritePageBuffer (
  IN NOR_FLASH_INSTANCE      *Instance,
  IN UINTN                   PageBufferOffset,
  IN UINTN                   NumWords,
  IN FLASH_DATA              *Buffer
  )
{
  UINT64        Timeout;
  UINTN         LastWrittenAddress;
  FLASH_DATA    LastWritenData;
  UINTN         CurrentOffset;
  UINTN         EndOffset;
  UINTN         TargetAddress;
  UINT64        SystemCounterVal;
  FLASH_DATA    Read1;
  FLASH_DATA    Read2;

  // Initialize variables
  Timeout = 0;
  LastWrittenAddress = 0;
  LastWritenData = 0;
  CurrentOffset   = PageBufferOffset;
  EndOffset       = PageBufferOffset + NumWords - 1;
  Timeout   = ConvertMicroSecondsToTicks (
                  mNorFlashDevices[Instance->Media.MediaId].BufferWriteTimeOut);
  TargetAddress = CREATE_NOR_ADDRESS (Instance->DeviceBaseAddress,
                     CREATE_BYTE_OFFSET (CurrentOffset));

  // don't try with a count of zero
  if (!NumWords) {
    return EFI_SUCCESS;
  }
  else if (NumWords == 1) {
    return NorFlashPlatformWriteWord (Instance, PageBufferOffset, *Buffer);
  }

  // Issue the Unlock cmds
  SEND_NOR_COMMAND (Instance->DeviceBaseAddress, CMD_UNLOCK_1_ADDR,
                   CMD_WRITE_TO_BUFFER_FIRST);

  SEND_NOR_COMMAND (Instance->DeviceBaseAddress, CMD_UNLOCK_2_ADDR,
                   CMD_WRITE_TO_BUFFER_SECOND);

  // Write the buffer load
  SEND_NOR_COMMAND (TargetAddress, 0, CMD_WRITE_TO_BUFFER_THIRD);

  // Write # of locations to program
  SEND_NOR_COMMAND (TargetAddress, 0, (NumWords - 1));

  // Load Data into Buffer
  while (CurrentOffset <= EndOffset) {
    LastWrittenAddress = CREATE_NOR_ADDRESS (Instance->DeviceBaseAddress,
                            CREATE_BYTE_OFFSET (CurrentOffset++));
    LastWritenData = *Buffer++;

    // Write Data
    FLASH_WRITE_DATA (LastWrittenAddress,LastWritenData);
  }

  // Issue the Buffered Program Confirm command
  SEND_NOR_COMMAND (TargetAddress, 0, CMD_WRITE_TO_BUFFER_CONFIRM);

  /* Wait for Write to Complete
     Read the last written address twice to detect bit toggle and
     to determine if date is wriiten successfully or not ?
     Get the maximum timer ticks needed to complete the operation
     Check if operation is complete or not in continous loop?
     if complete, exit from loop
     if not check the ticks that have been passed from the begining of loop
     if Maximum Ticks allocated for operation has passed, then exit from loop **/
  SystemCounterVal = GetPerformanceCounter();
  Timeout += SystemCounterVal;
  while (SystemCounterVal < Timeout) {
    if ((Read1 = FLASH_READ_DATA (LastWrittenAddress)) ==
            (Read2 = FLASH_READ_DATA (LastWrittenAddress))) {
      if (LastWritenData == FLASH_READ_DATA (LastWrittenAddress)) {
        break;
      }
    }
    SystemCounterVal = GetPerformanceCounter ();
  }

  if (SystemCounterVal >= Timeout) {
    DEBUG ((DEBUG_ERROR, "%a: Failed to Write @LastWrittenAddress 0x%p, Timeout\n",
                __FUNCTION__, LastWrittenAddress));
    return EFI_DEVICE_ERROR;
  } else {
    return EFI_SUCCESS;
  }
}

EFI_STATUS
NorFlashPlatformWriteWordAlignedAddressBuffer  (
  IN NOR_FLASH_INSTANCE   *Instance,
  IN UINTN                Offset,
  IN UINTN                NumWords,
  IN FLASH_DATA           *Buffer
  )
{
  EFI_STATUS              Status;
  UINTN                   MultiByteWordCount;
  UINTN                   Mask;
  UINTN                   IntWords;

  MultiByteWordCount = mNorFlashDevices[Instance->Media.MediaId].MultiByteWordCount;
  Mask = MultiByteWordCount - 1;
  IntWords = NumWords;
  Status = EFI_SUCCESS;

  if (Offset & Mask) {
    // program only as much as necessary, so pick the lower of the two numbers
    if (NumWords < (MultiByteWordCount - (Offset & Mask))) {
      IntWords = NumWords;
    } else {
      IntWords = MultiByteWordCount - (Offset & Mask);
    }

    // program the first few to get write buffer aligned
    Status = NorFlashPlatformWritePageBuffer (Instance, Offset, IntWords, Buffer);
    if (EFI_ERROR (Status)) {
      return Status;
    }

    Offset   += IntWords; // adjust pointers and counter
    NumWords -= IntWords;
    Buffer += IntWords;

    if (NumWords == 0) {
      return Status;
    }
  }

  while (NumWords >= MultiByteWordCount) {// while big chunks to do
    Status = NorFlashPlatformWritePageBuffer (Instance, Offset,
                            MultiByteWordCount, Buffer);
    if (EFI_ERROR (Status)) {
      return (Status);
    }

    Offset   += MultiByteWordCount; // adjust pointers and counter
    NumWords -= MultiByteWordCount;
    Buffer   += MultiByteWordCount;
  }
  if (NumWords == 0) {
    return (Status);
  }

  Status = NorFlashPlatformWritePageBuffer (Instance, Offset, NumWords, Buffer);
  return (Status);
}

/**
  Writes data to the NOR Flash using the Buffered Programming method.

  Write Buffer Programming allows the system to write a maximum of 32 bytes
  in one programming operation. Therefore this function will only handle
  buffers up to 32 bytes.
  To deal with larger buffers, call this function again.
**/
EFI_STATUS
NorFlashPlatformWriteBuffer (
  IN        NOR_FLASH_INSTANCE     *Instance,
  IN        EFI_LBA                Lba,
  IN        UINTN                  Offset,
  IN OUT    UINTN                  *NumBytes,
  IN        UINT8                  *Buffer
  )
{
  EFI_STATUS                       Status;
  FLASH_DATA                       *SrcBuffer;
  UINTN                            TargetOffsetinBytes;
  UINTN                            WordsToWrite;
  UINTN                            Mask;
  UINTN                            BufferSizeInBytes;
  UINTN                            IntBytes;
  UINT8                            *CopyFrom;
  UINT8                            *CopyTo;
  FLASH_DATA                       TempWrite;

  SrcBuffer = (FLASH_DATA *)Buffer;
  TargetOffsetinBytes = 0;
  WordsToWrite = 0;
  Mask = sizeof (FLASH_DATA) - 1;
  BufferSizeInBytes = *NumBytes;
  IntBytes = BufferSizeInBytes; // Intermediate Bytes needed to copy for alignment
  TempWrite = 0;

  DEBUG ((DEBUG_BLKIO, "%a(Parameters: Lba=%ld, Offset=0x%x, "
                        "*NumBytes=0x%x, Buffer @ 0x%08x)\n",
                        __FUNCTION__, Lba, Offset, *NumBytes, Buffer));

  TargetOffsetinBytes = GET_BLOCK_OFFSET (Lba) + (UINTN)(Offset);

  if (TargetOffsetinBytes & Mask) {
    // Write only as much as necessary, so pick the lower of the two numbers
    // and call it Intermediate bytes to write to make alignment proper
    if (BufferSizeInBytes < (sizeof (FLASH_DATA) - (TargetOffsetinBytes & Mask))) {
      IntBytes = BufferSizeInBytes;
    } else {
      IntBytes = sizeof (FLASH_DATA) - (TargetOffsetinBytes & Mask);
    }

    // Read the first few to get Read buffer aligned
    NorFlashPlatformRead (Instance, Lba, (TargetOffsetinBytes & ~Mask) -
            GET_BLOCK_OFFSET (Lba), sizeof (TempWrite), (UINT8*)&TempWrite);

    CopyTo = (UINT8*)&TempWrite;
    CopyTo += (TargetOffsetinBytes & Mask);
    CopyFrom = (UINT8*)Buffer;

    InternalMemCopyMem (CopyTo, CopyFrom, IntBytes);

    Status = NorFlashPlatformWriteWordAlignedAddressBuffer (
                       Instance,
                       (UINTN)((TargetOffsetinBytes & ~Mask) / sizeof (FLASH_DATA)),
                       1,
                       &TempWrite);
    if (EFI_ERROR (Status)) {
      DEBUG((DEBUG_ERROR, "%a : Failed to Write @TargetOffset 0x%x (0x%x)\n",
                  __FUNCTION__, TargetOffsetinBytes, Status));
      goto EXIT;
    }

    TargetOffsetinBytes += IntBytes; /* adjust pointers and counter */
    BufferSizeInBytes -= IntBytes;
    Buffer += IntBytes;

    if (BufferSizeInBytes == 0) {
      goto EXIT;
    }
  }

  // Write the bytes to CFI width aligned address.
  // Note we can Write number of bytes=CFI width in one operation
  WordsToWrite = BufferSizeInBytes/sizeof (FLASH_DATA);
  SrcBuffer = (FLASH_DATA*)Buffer;

  Status = NorFlashPlatformWriteWordAlignedAddressBuffer (
                     Instance,
                     (UINTN)(TargetOffsetinBytes/sizeof (FLASH_DATA)),
                     WordsToWrite,
                     SrcBuffer);
  if (EFI_ERROR(Status)) {
    DEBUG((DEBUG_ERROR, "%a : Failed to Write @ TargetOffset 0x%x (0x%x)\n",
            __FUNCTION__, TargetOffsetinBytes, Status));
    goto EXIT;
  }

  BufferSizeInBytes -= (WordsToWrite * sizeof (FLASH_DATA));
  Buffer += (WordsToWrite*sizeof (FLASH_DATA));
  TargetOffsetinBytes += (WordsToWrite * sizeof (FLASH_DATA));

  if (BufferSizeInBytes == 0) {
    goto EXIT;
  }

  // Now Write bytes that are remaining and are less than CFI width.
  // Read the first few to get Read buffer aligned
  NorFlashPlatformRead (
          Instance,
          Lba,
          TargetOffsetinBytes - GET_BLOCK_OFFSET (Lba),
          sizeof (TempWrite),
          (UINT8*)&TempWrite);

  CopyFrom = (UINT8*)Buffer;
  CopyTo = (UINT8*)&TempWrite;

  InternalMemCopyMem (CopyTo, CopyFrom, BufferSizeInBytes);

  Status = NorFlashPlatformWriteWordAlignedAddressBuffer (Instance,
                            (UINTN)(TargetOffsetinBytes/sizeof (FLASH_DATA)),
                            1,
                            &TempWrite);
  if (EFI_ERROR(Status)) {
    DEBUG((DEBUG_ERROR, "%a: Failed to Write @TargetOffset 0x%x Status=%d\n",
                __FUNCTION__, TargetOffsetinBytes, Status));
    goto EXIT;
  }

EXIT:
  // Put device back into Read Array mode (via Reset)
  NorFlashPlatformReset (Instance->DeviceBaseAddress);
  return (Status);
}

EFI_STATUS
NorFlashPlatformRead (
  IN  NOR_FLASH_INSTANCE  *Instance,
  IN  EFI_LBA             Lba,
  IN  UINTN               Offset,
  IN  UINTN               BufferSizeInBytes,
  OUT UINT8               *Buffer
  )
{
  UINTN                  IntBytes;
  UINTN                  Mask;
  FLASH_DATA             TempRead;
  UINT8                  *CopyFrom;
  UINT8                  *CopyTo;
  UINTN                  TargetOffsetinBytes;
  FLASH_DATA             *ReadData;
  UINTN                  BlockSize;

  IntBytes = BufferSizeInBytes; //Intermediate Bytes needed to copy for alignment
  Mask = sizeof (FLASH_DATA) - 1;
  TempRead = 0;
  TargetOffsetinBytes = (UINTN)(GET_BLOCK_OFFSET (Lba) + Offset);
  BlockSize = Instance->Media.BlockSize;

  DEBUG ((DEBUG_BLKIO, "%a(Parameters: Lba=%ld, Offset=0x%x,"
              " BufferSizeInBytes=0x%x, Buffer @ 0x%p)\n",
              __FUNCTION__, Lba, Offset, BufferSizeInBytes, Buffer));

  // The buffer must be valid
  if (Buffer == NULL) {
    return EFI_INVALID_PARAMETER;
  }

  // Return if we have not any byte to read
  if (BufferSizeInBytes == 0) {
    return EFI_SUCCESS;
  }

  if (((Lba * BlockSize) + BufferSizeInBytes) > Instance->Size) {
    DEBUG ((DEBUG_ERROR, "%a : Read will exceed device size.\n", __FUNCTION__));
    return EFI_INVALID_PARAMETER;
  }

  // Put device back into Read Array mode (via Reset)
  NorFlashPlatformReset (Instance->DeviceBaseAddress);

  // First Read bytes to make buffer aligned to CFI width
  if (TargetOffsetinBytes & Mask) {
    // Read only as much as necessary, so pick the lower of the two numbers
    if (BufferSizeInBytes < (sizeof (FLASH_DATA) - (TargetOffsetinBytes & Mask))) {
      IntBytes = BufferSizeInBytes;
    } else {
      IntBytes = sizeof (FLASH_DATA) - (TargetOffsetinBytes & Mask);
    }

    // Read the first few to get Read buffer aligned
    TempRead = FLASH_READ_DATA (CREATE_NOR_ADDRESS (
                     Instance->DeviceBaseAddress,
                     CREATE_BYTE_OFFSET ((TargetOffsetinBytes & ~Mask)/sizeof (FLASH_DATA))));

    CopyFrom = (UINT8*)&TempRead;
    CopyFrom += (TargetOffsetinBytes & Mask);
    CopyTo = (UINT8*)Buffer;

    InternalMemCopyMem (CopyTo, CopyFrom, IntBytes);

    TargetOffsetinBytes += IntBytes; // adjust pointers and counter
    BufferSizeInBytes -= IntBytes;
    Buffer += IntBytes;
    if (BufferSizeInBytes == 0) {
      return EFI_SUCCESS;
    }
  }

  ReadData = (FLASH_DATA*)Buffer;

  // Readout the bytes from CFI width aligned address.
  // Note we can read number of bytes=CFI width in one operation
  while (BufferSizeInBytes >= sizeof (FLASH_DATA)) {
    *ReadData = FLASH_READ_DATA (CREATE_NOR_ADDRESS (
                     Instance->DeviceBaseAddress,
                     CREATE_BYTE_OFFSET (TargetOffsetinBytes/sizeof (FLASH_DATA))));
    ReadData += 1;
    BufferSizeInBytes -= sizeof (FLASH_DATA);
    TargetOffsetinBytes += sizeof (FLASH_DATA);
  }

  if (BufferSizeInBytes == 0) {
    return EFI_SUCCESS;
  }

  // Now read bytes that are remaining and are less than CFI width.
  CopyTo = (UINT8*)ReadData;
  // Read the first few to get Read buffer aligned
  TempRead = FLASH_READ_DATA (CREATE_NOR_ADDRESS (
                     Instance->DeviceBaseAddress,
                     CREATE_BYTE_OFFSET (TargetOffsetinBytes/sizeof (FLASH_DATA))));
  CopyFrom = (UINT8*)&TempRead;

  InternalMemCopyMem (CopyTo, CopyFrom, BufferSizeInBytes);

  return EFI_SUCCESS;
}

EFI_STATUS
NorFlashPlatformReset (
  IN  UINTN  DeviceBaseAddress
  )
{
  SEND_NOR_COMMAND (DeviceBaseAddress, CMD_UNLOCK_1_ADDR,
                     CMD_RESET_FIRST);

  SEND_NOR_COMMAND (DeviceBaseAddress, CMD_UNLOCK_2_ADDR,
                     CMD_RESET_SECOND);

  SEND_NOR_COMMAND (DeviceBaseAddress, 0, CMD_RESET);

  return EFI_SUCCESS;
}

EFI_STATUS
NorFlashPlatformGetDevices (
  OUT NorFlashDescription  **NorFlashDevices,
  OUT UINT32               *Count
  )
{
  if ((NorFlashDevices == NULL) || (Count == NULL)) {
    return EFI_INVALID_PARAMETER;
  }

  // Get the number of NOR flash devices supported
  *NorFlashDevices = mNorFlashDevices;
  *Count = NOR_FLASH_DEVICE_COUNT;

  return EFI_SUCCESS;
}

EFI_STATUS
NorFlashPlatformFlashGetAttributes (
  OUT NorFlashDescription  *NorFlashDevices,
  IN UINT32                Count
  )
{
  EFI_STATUS               Status;
  UINT32                   Index;

  if ((NorFlashDevices == NULL) || (Count == 0)) {
    return EFI_INVALID_PARAMETER;
  }

  // Check the attributes of the NOR flash slave we are connected to.
  // Currently we support only CFI flash devices. Bail-out otherwise.
  Status = CfiNorFlashFlashGetAttributes (NorFlashDevices, Count);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  // Limit the Size of Nor Flash that can be programmed
  for (Index = 0; Index < Count; Index++) {
    NorFlashDevices[Index].RegionBaseAddress = PcdGet64 (PcdFlashReservedRegionBase64);

    // flash size available is half of total flash size
    NorFlashDevices[Index].Size >>= 1;
    NorFlashDevices[Index].Size -= (NorFlashDevices[Index].RegionBaseAddress -
                                    NorFlashDevices[Index].DeviceBaseAddress);
    if((NorFlashDevices[Index].RegionBaseAddress - NorFlashDevices[Index].DeviceBaseAddress) %
                NorFlashDevices[Index].BlockSize) {
      DEBUG ((DEBUG_ERROR, "%a : Reserved Region(0x%p) doesn't start "
                  "from block boundry(0x%08x)\n", __FUNCTION__,
                  (UINTN)NorFlashDevices[Index].RegionBaseAddress,
                  (UINT32)NorFlashDevices[Index].BlockSize));
      return EFI_DEVICE_ERROR;
    }
  }
  return Status;
}
