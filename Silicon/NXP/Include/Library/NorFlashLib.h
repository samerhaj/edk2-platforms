/** @file

 Copyright (c) 2011-2012, ARM Ltd. All rights reserved.
 Copyright (c) 2016, Freescale Semiconductor. All rights reserved.
 Copyright 2017 NXP

This program and the accompanying materials
 are licensed and made available under the terms and conditions of the BSD License
 which accompanies this distribution.  The full text of the license may be found at
 http://opensource.org/licenses/bsd-license.php

 THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
 WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

 **/

#ifndef _NOR_FLASH_LIB_H_
#define _NOR_FLASH_LIB_H_

#include <NorFlash.h>

#define NOR_FLASH_DEVICE_COUNT      1

typedef struct {
  UINTN  DeviceBaseAddress;   // Start address of the Device Base Address (DBA)
  UINTN  RegionBaseAddress;   // Start address of one single region
  UINTN  Size;
  UINTN  BlockSize;
  UINTN  MultiByteWordCount;  // Maximum Word count that can be written to Nor Flash in multi byte write
  UINTN  WordWriteTimeOut;    // single byte/word timeout usec
  UINTN  BufferWriteTimeOut;  // buffer write timeout usec
  UINTN  BlockEraseTimeOut;   // block erase timeout usec
  UINTN  ChipEraseTimeOut;    // chip erase timeout usec
} NorFlashDescription;

EFI_STATUS
NorFlashPlatformGetDevices (
  OUT NorFlashDescription **NorFlashDevices,
  OUT UINT32              *Count
  );

EFI_STATUS
NorFlashPlatformFlashGetAttributes (
  OUT NorFlashDescription *NorFlashDevices,
  IN  UINT32              Count
  );

EFI_STATUS
NorFlashPlatformWriteBuffer (
  IN NOR_FLASH_INSTANCE     *Instance,
  IN EFI_LBA                Lba,
  IN        UINTN           Offset,
  IN OUT    UINTN           *NumBytes,
  IN        UINT8           *Buffer
  );

EFI_STATUS
NorFlashPlatformEraseSector (
  IN NOR_FLASH_INSTANCE     *Instance,
  IN UINTN                  SectorAddress
  );

EFI_STATUS
NorFlashPlatformRead (
  IN NOR_FLASH_INSTANCE   *Instance,
  IN EFI_LBA              Lba,
  IN UINTN                Offset,
  IN UINTN                BufferSizeInBytes,
  OUT UINT8               *Buffer
  );

EFI_STATUS
NorFlashPlatformReset (
  IN UINTN Instance
  );

#endif /* _NOR_FLASH_LIB_H_ */
