/** @NandFlash.c
#
#  Driver for installing BlockIo protocol over IFC NAND
#
#  Copyright 218 NXP
#
#  This program and the accompanying materials
#  are licensed and made available under the terms and conditions of the BSD License
#  which accompanies this distribution.  The full text of the license may be found at
#  http://opensource.org/licenses/bsd-license.php
#
#  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
#  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.
#
**/

#include <Library/BaseMemoryLib.h>
#include <Library/DebugLib.h>
#include <Library/IoLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/BaseMemoryLib/MemLibInternals.h>
#include <Protocol/BlockIo.h>
#include <IfcNand.h>

//
// Global define
//
UINT32 NandCs;
NAND_FLASH_INFO *gNandFlashInfo;

//
// Static definition for Supported NAND flash meta data
//
NAND_PART_INFO_TABLE gNandPartInfoTable[2] = {
  { 0x2C, 0xAC, 17, 11},
  { 0x2C, 0x48, 17, 11}
};

//
// EFI_BLOCK_IO_MEDIA instance definition
//
STATIC EFI_BLOCK_IO_MEDIA gNandFlashMedia = {
  SIGNATURE_32('n','a','n','d'),            // MediaId
  FALSE,                                    // RemovableMedia
  TRUE,                                     // MediaPresent
  FALSE,                                    // LogicalPartition
  FALSE,                                    // ReadOnly
  FALSE,                                    // WriteCaching
  0,                                        // BlockSize
  2,                                        // IoAlign
  0,                                        // Pad
  0                                         // LastBlock
};

//
// Device path for SemiHosting. It contains our autogened Caller ID GUID.
//
typedef struct {
  VENDOR_DEVICE_PATH       Guid;
  EFI_DEVICE_PATH_PROTOCOL End;
} FLASH_DEVICE_PATH;

FLASH_DEVICE_PATH gDevicePath = {
  {
    { HARDWARE_DEVICE_PATH, HW_VENDOR_DP, { sizeof (VENDOR_DEVICE_PATH), 0 } },
    EFI_CALLER_ID_GUID
  },
  { END_DEVICE_PATH_TYPE, END_ENTIRE_DEVICE_PATH_SUBTYPE,
                { sizeof (EFI_DEVICE_PATH_PROTOCOL), 0} }
};

/**
  Function to return actual page address in NAND flash block

  @param[in]  BlockIndex Block Number of NAND flash
  @param[in]  PageIndex  Page in NAND flash Block
  @param[out] The page address within block

**/
STATIC
UINTN
GetActualPageAddress (
  UINTN BlockIndex,
  UINTN PageIndex
 )
{
  return ((BlockIndex * gNandFlashInfo->NumPagesPerBlock) + PageIndex);
}

/**
   Function implementing getting NAND flash ID

   @retval EFI_SUCCESS   Nand part ID supported
   @retval EFI_NOT_FOUND Nand ID not supported

**/
STATIC
EFI_STATUS
NandDetectPart (
  VOID
  )
{
  UINT8      PartInfo[8];
  UINTN      Index;
  BOOLEAN    Found;
  EFI_STATUS Status;

  Found = FALSE;

  // Send READ ID command
  Status = NandCmdSend (NAND_CMD_READID, 0, 0, NandCs);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  // Read 5-bytes to idenfity code programmed into the NAND flash devices.
  // BYTE 0 = Manufacture ID
  // Byte 1 = Device ID
  // Byte 2, 3, 4 = Nand part specific information (Page size, Block size etc)

  for (Index = 0; Index < sizeof(PartInfo); Index++) {
    PartInfo[Index] = MmioRead8 ((UINTN)gNandFlashInfo->BufBase + Index);
  }

  // Check if the ManufactureId and DeviceId are part of the currently
  // supported nand parts.
  for (Index = 0; Index < sizeof (gNandPartInfoTable)/sizeof (NAND_PART_INFO_TABLE);
       Index++) {
    if (gNandPartInfoTable[Index].ManufactureId == PartInfo[0] &&
        gNandPartInfoTable[Index].DeviceId == PartInfo[1]) {
          gNandFlashInfo->BlockAddressStart =  gNandPartInfoTable[Index].BlockAddressStart;
          gNandFlashInfo->PageAddressStart =  gNandPartInfoTable[Index].PageAddressStart;
          Found = TRUE;
          break;
    }
  }

  if (Found == FALSE) {
    DEBUG ((DEBUG_ERROR, "Nand part is not currently supported.\
                          Manufacture id: %x, Device id: %x\n",
                          PartInfo[0], PartInfo[1]));
    return EFI_NOT_FOUND;
  }

  // Populate NAND_FLASH_INFO based on the result of READ ID command.
  gNandFlashInfo->ManufactureId = PartInfo[0];
  gNandFlashInfo->DeviceId = PartInfo[1];

  // Calculate total number of blocks.
  gNandFlashInfo->NumPagesPerBlock = DivU64x32 (gNandFlashInfo->BlockSize,
                                                gNandFlashInfo->PageSize);

  return EFI_SUCCESS;
}

/**
   Function to read page from NAND flash

  @param[in]  BlockIndex   Block Number of NAND flash
  @param[in]  PageIndex    Page in NAND flash Block
  @param[out] Buffer       Page in buffer

  @retval EFI_SUCCESS      Page read succcessfully
  @retval EFI_DEVICE_ERROR Hardware error during read operation

**/
STATIC
EFI_STATUS
NandReadPage (
  IN  UINTN  BlockIndex,
  IN  UINTN  PageIndex,
  OUT VOID   *Buffer
  )
{
  EFI_STATUS Status;
  UINTN      PageAddr;
  VOID       *SrcAddr;
  UINT8      *SpareArea;

  // Generate device address in bytes to access specific block and page index
  PageAddr = GetActualPageAddress (BlockIndex, PageIndex);

  // Send READ command
  Status = NandCmdSend (NAND_CMD_READ0, 0, PageAddr, NandCs);
  if (EFI_ERROR (Status)) {
    return Status;
  }
  // Calculate source page address from IFC NAND SRAM buffer base
  SrcAddr = (VOID*)(gNandFlashInfo->BufBase +
                   (gNandFlashInfo->PageSize << 1) * (PageIndex & 0x3));
  SpareArea = (UINT8*)SrcAddr + gNandFlashInfo->PageSize;
  if (*SpareArea != 0xff) {
    return EFI_DEVICE_ERROR;
  }
  CopyMem (Buffer, SrcAddr, gNandFlashInfo->PageSize);

  return EFI_SUCCESS;
}

/**
   Function to write page in NAND flash

  @param[in]  BlockIndex Block Number of NAND flash
  @param[in]  PageIndex  Page in NAND flash Block
  @param[in]  Buffer     Data to be written

**/
STATIC
EFI_STATUS
NandWritePage (
  IN  UINTN  BlockIndex,
  IN  UINTN  PageIndex,
  OUT VOID   *Buffer,
  IN  UINT8  *SpareBuffer
  )
{
  UINTN  Address;
  VOID   *DestAddr;

  // Generate device address in bytes to access specific block and page index
  Address = GetActualPageAddress (BlockIndex, PageIndex);

  // Send SERIAL DATA INPUT command
  NandCmdSend (NAND_CMD_SEQIN, 0, Address, NandCs);

  DestAddr = (VOID*)(gNandFlashInfo->BufBase +
                    (gNandFlashInfo->PageSize << 1) * (PageIndex & 0x3));
  // Data input from Buffer
  CopyMem (DestAddr, (VOID*) Buffer, gNandFlashInfo->PageSize);

  // Send PROGRAM command
  return NandCmdSend (NAND_CMD_PAGEPROG, 0, Address, NandCs);
}

/**
   Function to erase block in NAND flash

  @param[in]  BlockIndex Block Number of NAND flash

**/
STATIC
EFI_STATUS
NandEraseBlock (
  IN UINTN BlockIndex
  )
{
  UINTN      Address;

  // Generate device address in bytes to access specific block and page index
  Address = GetActualPageAddress (BlockIndex, 0);

  // Send ERASE command
  return NandCmdSend (NAND_CMD_ERASE1, 0, Address, NandCs);
}

/**
  Function to read block from NAND flash

  @param[in]   BlockIndex  Block Number of NAND flash
  @return[out] Buffer      Page in NAND flash Block

  @retval EFI_SUCCESS      Block read successfully
  @retval EFI_DEVICE_ERROR Hardware error during read operation

**/
STATIC
EFI_STATUS
NandReadBlock (
  IN UINTN  BlockIndex,
  OUT VOID  *Buffer
  )
{
  UINTN      PageIndex;
  EFI_STATUS Status;

  Status = EFI_SUCCESS;

  for (PageIndex = 0; PageIndex < gNandFlashInfo->NumPagesPerBlock;
       PageIndex++) {
    Status = NandReadPage (BlockIndex, PageIndex, Buffer);
    if (EFI_ERROR (Status)) {
      return Status;
    }
    Buffer = ((UINT8 *)Buffer + gNandFlashInfo->PageSize);
  }

  return Status;
}

/**
  Function to write block in NAND flash

  @param[in]  StartBlockIndex  Start Block Number of NAND flash
  @param[in]  EndBlockIndex    End Block Number of NAND flash
  @param[out] Buffer           Data to be written

  @retval EFI_SUCCESS          Data block written successfully
  @retval EFI_DEVICE_ERROR     Hardware error during write or wait operation

**/
STATIC
EFI_STATUS
NandWriteBlock (
  IN UINTN  StartBlockIndex,
  IN UINTN  EndBlockIndex,
  OUT VOID  *Buffer,
  OUT VOID  *SpareBuffer
  )
{
  UINTN      BlockIndex;
  UINTN      PageIndex;
  EFI_STATUS Status;

  Status = EFI_SUCCESS;

  for (BlockIndex = StartBlockIndex; BlockIndex <= EndBlockIndex; BlockIndex++) {
    // Page programming.
    for (PageIndex = 0; PageIndex < gNandFlashInfo->NumPagesPerBlock;
         PageIndex++) {
      Status = NandWritePage (BlockIndex, PageIndex, Buffer, SpareBuffer);
      if (EFI_ERROR (Status)) {
        DEBUG ((DEBUG_ERROR,"NandWritePage Failed\n"));
        return Status;
      }
      Status = Wait (NandCs);
      if (EFI_ERROR (Status)) {
        DEBUG ((DEBUG_ERROR,"IfcWait Failed\n"));
        return Status;
      }
      Buffer = ((UINT8 *)Buffer + gNandFlashInfo->PageSize);
    }
  }

  return Status;
}
/**
  Function for NAND flash reset

  @param[in] This                 EFI_BLOCK_IO_PROTOCOL instance of calling context
  @param[in] ExtendedVerification Boolean for extended verification

**/
EFI_STATUS
EFIAPI
NandFlashReset (
  IN EFI_BLOCK_IO_PROTOCOL *This,
  IN BOOLEAN               ExtendedVerification
  )
{
  return NandCmdSend (NAND_CMD_RESET, 0, 0, NandCs);
}

/**
   Function for NAND flash block read

  @param[in]  This              EFI_BLOCK_IO_PROTOCOL instance of calling context
  @param[in]  MediaId           MediaId of Nand flash
  @param[in]  Lba               Starting logical block index from which to read
  @param[in]  BufferSize        Size of buffer to read
  @param[out] Buffer            Page in NAND flash Block

  @retval  EFI_SUCCESS           Block read successfully
  @retval  EFI_DEVICE_ERROR      Hardware error during read operation
  @retval  EFI_INVALID_PARAMETER Caller allocated Buffer is NULL
  @retval  EFI_BAD_BUFFER_SIZE   Buffer size is invalid as per block size

**/
EFI_STATUS
EFIAPI
NandFlashReadBlocks (
  IN EFI_BLOCK_IO_PROTOCOL *This,
  IN UINT32                MediaId,
  IN EFI_LBA               Lba,
  IN UINTN                 BufferSize,
  OUT VOID                 *Buffer
  )
{
  UINTN       NumBlocks;
  UINTN       Index;
  UINTN       BlockCount;
  VOID        *TBuf;
  EFI_STATUS  Status;

  Status = EFI_SUCCESS;

  if (Buffer == NULL) {
    Status = EFI_INVALID_PARAMETER;
    goto exit;
  }

  if (Lba > gNandFlashInfo->LastBlock) {
    Status = EFI_INVALID_PARAMETER;
    goto exit;
  }

  if ((BufferSize % gNandFlashInfo->BlockSize) != 0) {
    Status = EFI_BAD_BUFFER_SIZE;
    goto exit;
  }

  NumBlocks = DivU64x32 (BufferSize, gNandFlashInfo->BlockSize);

  TBuf = Buffer;
  Index = Lba;
  BlockCount = 0;
  while (BlockCount < NumBlocks) {
    Status = NandReadBlock ((UINTN)Index, TBuf);
    if (EFI_ERROR (Status)) {
      DEBUG ((DEBUG_ERROR, "Read block failed, skipping %x\n", Status));
      Index++;
      continue;
    }
    TBuf+=gNandFlashInfo->BlockSize;
    BlockCount++;
    Index++;
  }

  exit:
    return Status;
}

/**
  Function for NAND flash block write

  @param[in]  This              EFI_BLOCK_IO_PROTOCOL instance of calling context
  @param[in]  MediaId           MediaId of Nand flash
  @param[in]  Lba               Starting logical block index from which to write
  @param[in]  BufferSize        Size of buffer to write
  @param[out] Buffer            Page in NAND flash Block

  @retval EFI_SUCCESS           Block write successfully
  @retval EFI_DEVICE_ERROR      Hardware error during read operation
  @retval EFI_INVALID_PARAMETER Caller allocated Buffer is NULL
  @retval EFI_BAD_BUFFER_SIZE   Buffer size is invalid as per block size

**/
EFI_STATUS
EFIAPI
NandFlashWriteBlocks (
  IN EFI_BLOCK_IO_PROTOCOL *This,
  IN UINT32                MediaId,
  IN EFI_LBA               Lba,
  IN UINTN                 BufferSize,
  IN VOID                  *Buffer
  )
{
  UINTN      BlockIndex;
  UINTN      NumBlocks;
  UINTN      EndBlockIndex;
  EFI_STATUS Status;
  UINT8      *SpareBuffer;

  SpareBuffer = NULL;

  if (Buffer == NULL) {
    Status = EFI_INVALID_PARAMETER;
    goto exit;
  }

  if (Lba > gNandFlashInfo->LastBlock) {
    Status = EFI_INVALID_PARAMETER;
    goto exit;
  }

  if ((BufferSize % gNandFlashInfo->BlockSize) != 0) {
    Status = EFI_BAD_BUFFER_SIZE;
    goto exit;
  }

  NumBlocks = DivU64x32 (BufferSize, gNandFlashInfo->BlockSize);
  EndBlockIndex = ((UINTN)Lba + NumBlocks) - 1;

  // Erase block
  for (BlockIndex = (UINTN)Lba; BlockIndex <= EndBlockIndex; BlockIndex++) {
    Status = NandEraseBlock (BlockIndex);
    if (EFI_ERROR (Status)) {
      DEBUG ((DEBUG_ERROR, "Erase block failed. Status: %x\n", Status));
      goto exit;
    }
    Status = Wait (NandCs);
    if (EFI_ERROR (Status)) {
      DEBUG ((DEBUG_ERROR, "Wait on Erase block failed. Status: %x\n", Status));
      goto exit;
    }
  }

  // Program data
  Status = NandWriteBlock ((UINTN)Lba, EndBlockIndex, Buffer, SpareBuffer);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "Block write fails: %x\n", Status));
  }

  exit:
    if (SpareBuffer != NULL) {
      FreePool (SpareBuffer);
    }

  return Status;
}

/**
   Function for NAND flash block flush
**/
EFI_STATUS
EFIAPI
NandFlashFlushBlocks (
  IN EFI_BLOCK_IO_PROTOCOL  *This
  )
{
  return EFI_SUCCESS;
}

/**
  Function for NAND flash initialization
   Detect Nand flash manufacture ID
   Fill Nand flash structure
   Flash reset

  @param[out] gNandFlashMedia Pointer to EFI_BLOCK_IO_MEDIA instance
                              of calling context

  @retval EFI_SUCCESS         Flash inialization successful
  @retval EFI_NOT_FOUND       Nand flash ID not found

**/
EFI_STATUS
IfcNandFlashInit (
  OUT EFI_BLOCK_IO_MEDIA *gNandFlashMedia
  )
{
  EFI_STATUS  Status;

  if (!gNandFlashInfo) {
    gNandFlashInfo = (NAND_FLASH_INFO *)AllocateZeroPool (sizeof (NAND_FLASH_INFO));
    ASSERT(gNandFlashInfo != NULL);
    GetIfcNandFlashInfo (gNandFlashInfo);
    NandCs = gNandFlashInfo->ChipSelect;
  }

  // Reset NAND part
  NandFlashReset (NULL, FALSE);

  // Detect NAND part and populate gNandFlashInfo structure
  Status = NandDetectPart ();
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "Nand part id detection failure: Status: %x\n", Status));
    return Status;
  }

  // Patch EFI_BLOCK_IO_MEDIA structure.
  if (gNandFlashMedia) {
    gNandFlashMedia->BlockSize = gNandFlashInfo->BlockSize;
    gNandFlashMedia->LastBlock = gNandFlashInfo->LastBlock;
  }

  return Status;
}

EFI_BLOCK_IO_PROTOCOL BlockIo =
{
  EFI_BLOCK_IO_INTERFACE_REVISION,  // Revision
  &gNandFlashMedia,                 // Media
  NandFlashReset,                   // Reset
  NandFlashReadBlocks,              // ReadBlocks
  NandFlashWriteBlocks,             // WriteBlocks
  NandFlashFlushBlocks              // FlushBlocks
};

/**
  The user Entry Point for module NandFlasjDxe. The user code starts with this function.

  @param[in] ImageHandle    The firmware allocated handle for the EFI image.
  @param[in] SystemTable    A pointer to the EFI System Table.

  @retval EFI_SUCCESS       The entry point is executed successfully.
  @retval other             Some error occurs when executing this entry point.

**/
EFI_STATUS
NandFlashInitialize (
  IN EFI_HANDLE       ImageHandle,
  IN EFI_SYSTEM_TABLE *SystemTable
  )
{
  EFI_STATUS Status;

  Status = IfcNandFlashInit (&gNandFlashMedia);

  //
  // Install driver model protocol(s).
  //
  if (!EFI_ERROR (Status)) {
    Status = gBS->InstallMultipleProtocolInterfaces (&ImageHandle,
                                                     &gEfiDevicePathProtocolGuid,
                                                     &gDevicePath,
                                                     &gEfiBlockIoProtocolGuid,
                                                     &BlockIo,
                                                     NULL);
  }

  return Status;
}
