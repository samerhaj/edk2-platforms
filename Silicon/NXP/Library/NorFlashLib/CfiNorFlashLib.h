/** @CfiNorFlashLib.h

  Copyright (c) 2015, Freescale Semiconductor, Inc. All rights reserved.
  Copyright 2017 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#ifndef __CFI_NOR_FLASH_LIB_H__
#define __CFI_NOR_FLASH_LIB_H__

#include <Library/DebugLib.h>
#include <Library/NorFlashLib.h>

/*
 * Values for the width of the port
 */
#define FLASH_CFI_8BIT               0x01
#define FLASH_CFI_16BIT              0x02
#define FLASH_CFI_32BIT              0x04
#define FLASH_CFI_64BIT              0x08

#define CREATE_BYTE_OFFSET(OffsetAddr)               ((sizeof (FLASH_DATA)) * (OffsetAddr))
#define CREATE_NOR_ADDRESS(BaseAddr, OffsetAddr)     ((BaseAddr) + (OffsetAddr))
#define FLASH_READ(Addr)                             FlashRead ((Addr))
#define FLASH_WRITE(Addr, Val)                       FlashWrite ((Val), (Addr))
#define FLASH_READ_DATA(Addr)                        FlashReadData ((Addr))
#define FLASH_WRITE_DATA(Addr, Val)                  FlashWriteData ((Val), (Addr))
#define SEND_NOR_COMMAND(BaseAddr, Offset, Cmd)      FLASH_WRITE (CREATE_NOR_ADDRESS (BaseAddr, CREATE_BYTE_OFFSET (Offset)), (Cmd))

typedef UINT16 FLASH_DATA;

VOID
FlashWrite (
  IN  FLASH_DATA  Val,
  IN  UINTN       Addr
  );

FLASH_DATA
FlashRead (
  IN  UINTN       Addr
  );

VOID
FlashWriteData (
  IN  FLASH_DATA  Val,
  IN  UINTN       Addr
  );

FLASH_DATA
FlashReadData (
  IN  UINTN      Addr
  );

EFI_STATUS
CfiNorFlashFlashGetAttributes (
  OUT NorFlashDescription *NorFlashDevices,
  IN UINT32               Index
  );

#endif //__CFI_NOR_FLASH_LIB_H__
