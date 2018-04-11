/** @IfcNand.h

  This file define the NAND flash data structures and interfaces

  Copyright 2018 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#ifndef __IFC_NAND_H__
#define __IFC_NAND_H__

//
// List of commands
//
#define NAND_CMD_RESET      0xFF
#define NAND_CMD_READID     0x90
#define NAND_CMD_STATUS     0x70
#define NAND_CMD_READ0      0x00
#define NAND_CMD_READSTART  0x30
#define NAND_CMD_ERASE1     0x60
#define NAND_CMD_ERASE2     0xD0
#define NAND_CMD_SEQIN      0x80
#define NAND_CMD_PAGEPROG   0x10

typedef struct {
  UINT8 ManufactureId;     // Manufacture ID of NAND flash
  UINT8 DeviceId;          // Device ID of NAND flash
  UINT8 BlockAddressStart; // Start of the Block address in actual NAND
  UINT8 PageAddressStart;  // Start of the Page address in actual NAND
} NAND_PART_INFO_TABLE;

//
// NAND flash structure
//
typedef struct {
  UINT8     ChipSelect;
  UINT8     DeviceId;
  UINT8     Organization;
  UINT32    PageSize;
  UINT32    SparePageSize;
  UINT32    BlockSize;
  UINT32    LastBlock;
  UINT8     ManufactureId;
  UINT32    NumPagesPerBlock;
  UINT8     BlockAddressStart;
  UINT8     PageAddressStart;
  VOID*     BufBase;
} NAND_FLASH_INFO;

/**
  Function to get NAND flash structure
**/
VOID
GetIfcNandFlashInfo (
  IN NAND_FLASH_INFO *NandFlashInfo
  );

/**
  Function for sending command to NAND flash
**/
EFI_STATUS
NandCmdSend (
  UINTN,
  INTN,
  INTN,
  INTN
  );

/**
  Function to implement wait operation during NAND block write
  Writing instruction and command register
**/
EFI_STATUS
Wait (
  INTN
  );

#endif //__IFC_NAND_H__
