/** @file
*
*  Copyright (c) 2017, Linaro, Ltd. All rights reserved.
*  Copyright 2018-2019 NXP
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
#include <Chassis.h>
#include <Library/BaseLib.h>
#include <Library/DebugLib.h>
#include <Library/DevicePathLib.h>
#include <Library/DtPlatformDtbLoaderLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiDriverEntryPoint.h>
#include <Library/UefiRuntimeServicesTableLib.h>

#include "DtInitDxe.h"

STATIC EFI_EVENT mDtInitDxeBootEvent;


STATIC
VOID
EFIAPI
DtInitDxeReadyToBootHandler (
  EFI_EVENT       Event,
  VOID            *Context
  )
{
  EFI_STATUS                      Status;
  DT_ACPI_VARSTORE_DATA           DtAcpiPref;
  UINTN                           BufferSize;

  BufferSize = sizeof (DtAcpiPref);
  Status = gRT->GetVariable(DT_ACPI_VARIABLE_NAME, &gDtPlatformFormSetGuid,
                  NULL, &BufferSize, &DtAcpiPref);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_WARN, "%a: no DT/ACPI preference found, defaulting to %a\n",
      __FUNCTION__, PcdGetBool (PcdDefaultDtPref) ? "DT" : "ACPI"));
    DtAcpiPref.Pref = PcdGetBool (PcdDefaultDtPref) ? DT_ACPI_SELECT_DT
                                                    : DT_ACPI_SELECT_ACPI;
  }

  if (DtAcpiPref.Pref != DT_ACPI_SELECT_DT) {
    Status = gBS->InstallConfigurationTable (&gFdtTableGuid, NULL);
  }

  gBS->CloseEvent (Event);
}

/**
  The entry point for DtInitDxe driver.

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
DtInitDxeEntryPoint (
  IN EFI_HANDLE                   ImageHandle,
  IN EFI_SYSTEM_TABLE             *SystemTable
  )
{
  EFI_STATUS                      Status;
  VOID                            *Dtb;
  UINTN                           DtbSize;

  Dtb = NULL;
  Status = DtPlatformLoadDtb (&Dtb, &DtbSize);
  if (EFI_ERROR (Status)) {
    goto Error;
  }
    

  //
  // install a reference to it as the FDT configuration table.
  //
  Status = gBS->InstallConfigurationTable (&gFdtTableGuid, Dtb);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "%a: failed to install FDT configuration table %r\n",
      __FUNCTION__, Status));
    goto Error;
  }

  //
  // Expose the Device Tree.
  //
  Status = gBS->InstallProtocolInterface (
                  &ImageHandle,
                  &gEdkiiPlatformHasDeviceTreeGuid,
                  EFI_NATIVE_INTERFACE,
                  NULL
                  );
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "%a: failed to installFDT configuration guid %r\n",
      __FUNCTION__, Status));
    goto Error;
  }

  Status = gBS->CreateEventEx (
                  EVT_NOTIFY_SIGNAL,
                  TPL_CALLBACK,
                  DtInitDxeReadyToBootHandler,
                  NULL,
                  &gEfiEventReadyToBootGuid,
                  &mDtInitDxeBootEvent
                  );
  ASSERT_EFI_ERROR (Status);

  return EFI_SUCCESS;

Error:
  ASSERT_EFI_ERROR (Status);
  CpuDeadLoop ();
  //
  // Keep compilers happy.
  //

  return Status;
}

