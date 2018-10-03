/** @file
*
*  Copyright (c) 2017, Linaro, Ltd. All rights reserved.
*  Copyright 2018 NXP
*
*  This program and the accompanying materials
*  are licensed and made available under the terms and conditions of the BSD License
*  which accompanies this distribution.  The full text of the license may be found at
*  http://opensource.org/licenses/bsd-license.php
*
*  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
*  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.
*
**/

#include <libfdt.h>
#include <Library/BaseLib.h>
#include <Library/DebugLib.h>
#include <Library/DevicePathLib.h>
#include <Library/DxeServicesLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiDriverEntryPoint.h>

#include "DtPlatformDxe.h"

/**
  The entry point for DtPlatformDxe driver.

  @param[in] ImageHandle     The image handle of the driver.
  @param[in] SystemTable     The system table.

  @retval EFI_NOT_FOUND           No suitable DTB image could be located
  @retval EFI_OUT_OF_RESOURCES    Fail to execute entry point due to lack of
                                  resources.
  @retval EFI_BAD_BUFFER_SIZE     Dtb could not be copied to allocated pool
  @retval EFI_SUCCES              All the related protocols are installed on
                                  the driver.

**/
EFI_STATUS
EFIAPI
DtPlatformDxeEntryPoint (
  IN EFI_HANDLE                   ImageHandle,
  IN EFI_SYSTEM_TABLE             *SystemTable
  )
{
  EFI_STATUS                      Status;
  VOID                            *Dtb;
  VOID                            *OrigDtb;
  UINTN                           DtbSize;

  Dtb = NULL;
  OrigDtb = NULL;
  Status = EFI_SUCCESS;

  Status = GetSectionFromAnyFv (
             &gDtPlatformDefaultDtbFileGuid,
             EFI_SECTION_RAW,
             0,
             &OrigDtb,
             &DtbSize
             );
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "Failed to get device tree\n"));
    return EFI_NOT_FOUND;
  }

  if (fdt_check_header (OrigDtb)) {
    DEBUG ((DEBUG_ERROR, "Invalid Device tree Header\n"));
    return EFI_NOT_FOUND;
  }

  // Assign extra memory for fixups
  DtbSize = fdt_totalsize (OrigDtb) + SIZE_512KB;

  // copy the blob into newly allocated memory
  Dtb = AllocateCopyPool (DtbSize, OrigDtb);
  if (Dtb == NULL) {
    return EFI_OUT_OF_RESOURCES;
  }

  // TODO: Verify the signed dtb image and then copy
  if (fdt_open_into (Dtb, Dtb, DtbSize)) {
    Status = EFI_BAD_BUFFER_SIZE;
    goto FreeDtb;
  }

  //
  // install a reference to it as the FDT configuration table.
  //
  Status = gBS->InstallConfigurationTable (&gFdtTableGuid, Dtb);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "%a: failed to install FDT configuration table\n",
      __FUNCTION__));
    goto FreeDtb;
  }

  return EFI_SUCCESS;

FreeDtb:
  if (Dtb != NULL) {
    FreePool (Dtb);
  }

  return Status;
}

