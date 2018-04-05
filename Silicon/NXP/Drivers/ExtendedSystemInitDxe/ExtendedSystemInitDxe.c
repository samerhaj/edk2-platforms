/** @ExtendedSystemInitDxe.c
 *
 *  Extended System Memory Init
 *
 *  Copyright 2017 NXP
 *
 *  This program and the accompanying materials
 *  are licensed and made available under the terms and conditions of the bsd
 *  license which accompanies this distribution. the full text of the license may
 *  be found at http://opensource.org/licenses/bsd-license.php
 *
 *  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.
 *
 *  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.
 */


#include <Library/DebugLib.h>
#include <Library/DxeServicesTableLib.h>

EFI_STATUS
ExtendedSystemInitialize (
  IN EFI_HANDLE         ImageHandle,
  IN EFI_SYSTEM_TABLE   *SystemTable
  )
{
  EFI_STATUS  Status;
  UINTN       SystemMemoryExSize;

#if FixedPcdGetBool(PcdMcHighMemSupport)
    SystemMemoryExSize = FixedPcdGet64 (PcdSystemMemoryExSize) - FixedPcdGet64 (PcdDpaa2McRamSize);
#else
    SystemMemoryExSize = FixedPcdGet64 (PcdSystemMemoryExSize);
#endif

  // Extended System Memory
  Status = gDS->AddMemorySpace (
                  EfiGcdMemoryTypeSystemMemory,
                  FixedPcdGet64 (PcdSystemMemoryExBase),
                  SystemMemoryExSize,
                  EFI_MEMORY_WC | EFI_MEMORY_WT |
                  EFI_MEMORY_WB
                  );

  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "AddMemorySpace Failed for Extended System Memory, Status=0x%llx ",Status));
    return Status;
  }

  // Set Memory Space Attributes
  Status = gDS->SetMemorySpaceAttributes (
                  FixedPcdGet64 (PcdSystemMemoryExBase),
                  SystemMemoryExSize,
                  EFI_MEMORY_WC | EFI_MEMORY_WT |
                  EFI_MEMORY_WB
                  );

  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "SetMemorySpaceAttributes Failed for Extended System "
                         "Memory, Status=0x%llx ",Status));
  }

  return Status;
}
