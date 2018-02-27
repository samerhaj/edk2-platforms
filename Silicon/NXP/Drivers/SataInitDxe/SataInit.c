/** @file
  This driver module adds SATA controller support.

  Copyright 2017-2018 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution. The full text of the license may be found
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

 **/

#include <IndustryStandard/Pci.h>
#include <Library/BeIoLib.h>
#include <Library/DebugLib.h>
#include <Library/IoLib.h>
#include <Library/NonDiscoverableDeviceRegistrationLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiLib.h>
#include <Protocol/PciIo.h>

#include "SataInit.h"

/**
  The Entry Point of module. It follows the standard UEFI driver model.

  @param[in] ImageHandle              The firmware allocated handle for the EFI image.
  @param[in] SystemTable              A pointer to the EFI System Table.

  @retval EFI_SUCCESS                 The entry point is executed successfully.
  @retval EFI_INVALID_PARAMETER       An invalid argument was given to RegisterNonDiscoverableMmioDevice
  @retval Other                       Registration of Sata controller failed
                                      as Non discoverable Mmio device.

**/
EFI_STATUS
EFIAPI
InitializeSataController (
  IN EFI_HANDLE            ImageHandle,
  IN EFI_SYSTEM_TABLE      *SystemTable
  )
{
  EFI_STATUS               Status;
  EFI_HANDLE               LocalHandle;
  UINT32                   NumSataController;
  UINT32                   Data;
  UINTN                    ControllerAddr;

  ASSERT ((PcdGet32 (PcdNumSataController) != 0) || (PcdGet32 (PcdSataBaseAddr) != 0));

  NumSataController = PcdGet32 (PcdNumSataController);

  //
  // Impact : The SATA controller does not detect some hard drives reliably with
  // the default SerDes register setting.
  // Workaround : write value 0x80104e20 to 0x1eb1300 (serdes 2)
  //
  if (PcdGetBool (PcdSataErratumA010554)) {
    BeMmioWrite32 ((UINTN)SERDES2_SATA_ERRATA, 0x80104e20);
  }

  //
  // Impact : Device may see false CRC errors causing unreliable SATA operation.
  // Workaround : write 0x80000000 to the address 0x20140520 (dcsr).
  //
  if (PcdGetBool (PcdSataErratumA010635)) {
    BeMmioWrite32 ((UINTN)DCSR_SATA_ERRATA, 0x80000000);
  }

  while (NumSataController) {
    LocalHandle = NULL;
    NumSataController--;
    ControllerAddr = PcdGet32 (PcdSataBaseAddr) +
                     (NumSataController * PcdGet32 (PcdSataSize));

    //
    // configuring Physical Control Layer parameters for Port 0
    //
    MmioWrite32 ((UINTN)(ControllerAddr + SATA_PPCFG), PORT_PHYSICAL);

    //
    // This register controls the configuration of the
    // Transport Layer for  Port 0
    // Errata Description : The default Rx watermark value may be insufficient for some
    // hard drives and result in a false CRC or internal errors.
    // Workaround: Change PTC[RXWM] field at offset 0xC8 to 0x29. Do not change
    // the other reserved fields of the register.
    //

    Data = MmioRead32 ((UINTN)(ControllerAddr + SATA_PTC));
    if (PcdGetBool (PcdSataErratumA009185)) {
      Data |= PORT_RXWM;
    } else {
      Data |= PORT_TRANSPORT;
    }
    MmioWrite32 ((UINTN)(ControllerAddr + SATA_PTC), Data);

    // Enable Non-Zero 4 MB PRD entries.
    MmioOr32 ((UINTN)(ControllerAddr + SATA_PAXIC), ENABLE_NONZERO_4MB_PRD);

    Status = RegisterNonDiscoverableMmioDevice (
               NonDiscoverableDeviceTypeAhci,
               NonDiscoverableDeviceDmaTypeNonCoherent,
               NULL,
               &LocalHandle,
               1,
               ControllerAddr, PcdGet32 (PcdSataSize)
             );

    if (EFI_ERROR (Status)) {
      DEBUG ((DEBUG_ERROR, "Failed to register SATA device (0x%x) with error 0x%x \n",
                           ControllerAddr, Status));
      return Status;
    }
  }

  return EFI_SUCCESS;
}
