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

#endif /*_SPI_NOR_FLASH_DXE_H_*/