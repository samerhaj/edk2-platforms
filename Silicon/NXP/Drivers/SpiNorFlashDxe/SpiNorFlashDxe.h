/** @file
  Copyright 2018 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#ifndef _SPI_NOR_FLASH_DXE_H_
#define _SPI_NOR_FLASH_DXE_H_

#include <Guid/EventGroup.h>

#include <Pi/PiSpi.h>
#include <Protocol/FirmwareVolumeBlock.h>
#include <Protocol/SpiIo.h>

#include <Library/UefiLib.h>

#include "SpiNorFlashOps.h"

#define CONTEXT_FROM_FVB_THIS(a)                 CR(a, SPI_NOR_FLASH_CONTEXT, FvbProtocol, SPI_NOR_FLASH_SIGNATURE)

typedef struct _SPI_NOR_FLASH_CONTEXT {
  UINT64                                Signature;
  EFI_SPI_IO_PROTOCOL                   *SpiIo;

  SPI_NOR_PARAMS                        *SpiNorParams;

  EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL   FvbProtocol;
  EFI_LBA                               StartLba;
  EFI_LBA                               LastLba;
  VOID*                                 ShadowBuffer;

  EFI_EVENT                             SpiNorVirtualAddressEvent;
} SPI_NOR_FLASH_CONTEXT;

EFI_STATUS
SpiNorFlashSupported (
  IN EFI_DRIVER_BINDING_PROTOCOL   *This,
  IN EFI_HANDLE                    ControllerHandle,
  IN EFI_DEVICE_PATH_PROTOCOL      *RemainingDevicePath OPTIONAL
  );

EFI_STATUS
SpiNorFlashStart (
  IN EFI_DRIVER_BINDING_PROTOCOL   *This,
  IN EFI_HANDLE                    ControllerHandle,
  IN EFI_DEVICE_PATH_PROTOCOL      *RemainingDevicePath OPTIONAL
  );

EFI_STATUS
SpiNorFlashStop (
  IN EFI_DRIVER_BINDING_PROTOCOL   *This,
  IN EFI_HANDLE                    ControllerHandle,
  IN UINTN                         NumberOfChildren,
  IN EFI_HANDLE                    *ChildHandleBuffer OPTIONAL
  );

EFIAPI
FvbGetAttributes(
  IN CONST  EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL    *This,
  OUT       EFI_FVB_ATTRIBUTES_2                   *Attributes
  );

EFI_STATUS
EFIAPI
FvbSetAttributes(
  IN CONST  EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL  *This,
  IN OUT    EFI_FVB_ATTRIBUTES_2                 *Attributes
  );

EFI_STATUS
EFIAPI
FvbGetPhysicalAddress (
  IN CONST  EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL  *This,
  OUT       EFI_PHYSICAL_ADDRESS                 *Address
  );

EFI_STATUS
EFIAPI
FvbGetBlockSize (
  IN CONST  EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL  *This,
  IN        EFI_LBA                              Lba,
  OUT       UINTN                                *BlockSize,
  OUT       UINTN                                *NumberOfBlocks
  );

EFI_STATUS
EFIAPI
FvbRead (
  IN CONST  EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL   *This,
  IN        EFI_LBA                               Lba,
  IN        UINTN                                 Offset,
  IN OUT    UINTN                                 *NumBytes,
  IN OUT    UINT8                                 *Buffer
  );

EFI_STATUS
EFIAPI
FvbWrite (
  IN CONST  EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL   *This,
  IN        EFI_LBA                               Lba,
  IN        UINTN                                 Offset,
  IN OUT    UINTN                                 *NumBytes,
  IN        UINT8                                 *Buffer
  );

EFI_STATUS
EFIAPI
FvbEraseBlocks (
  IN CONST EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL *This,
  ...
  );

EFI_STATUS
EFIAPI
SpiNorFlashFvbInitialize (
  IN SPI_NOR_FLASH_CONTEXT *Context
  );

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
  );

#endif /*_SPI_NOR_FLASH_DXE_H_*/
