/** @file
*
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
#include <Library/MemoryAllocationLib.h>
#include <Library/PcdLib.h>
#include <Library/SocFixupLib.h>
#include <Library/UefiLib.h>

/**
  The functions fixes the cpu nodes present under cpus node in device tree with the
  cpu enable-method as psci.

  for more details please refer https://www.kernel.org/doc/Documentation/devicetree/bindings/arm/cpus.txt

  @param[in] Dtb        Device Tree to fix up.

  @retval EFI_NOT_FOUND     "cpus" node not found or cpus node doesn't contain any "cpu" subnode
  @retval EFI_DEVICE_ERROR  Failed to set values in device tree
**/

STATIC
EFI_STATUS
FdtCpuFixup (
  IN  VOID *Dtb
  )
{
  INTN   ParentOffset;
  INTN   NodeOffset;
  INTN   FdtStatus;

  ParentOffset = fdt_subnode_offset (Dtb, 0, "cpus");
  if (ParentOffset < 0) {
    DEBUG ((DEBUG_ERROR, "Fdt: No cpus node found!!\n\n"));
    return EFI_NOT_FOUND;
  }

  NodeOffset = fdt_first_subnode (Dtb, ParentOffset);
  if (NodeOffset < 0) {
    DEBUG ((DEBUG_ERROR, "No cpu nodes found!!!\n"));
    return EFI_NOT_FOUND;
  }

  while (NodeOffset >= 0) {
    FdtStatus = fdt_setprop_string (Dtb, NodeOffset, "enable-method", "psci");
    if (FdtStatus) {
      DEBUG ((DEBUG_ERROR, "fdt_setprop/psci: Could not add enable-method, %a!!\n", fdt_strerror (FdtStatus)));
      return EFI_DEVICE_ERROR;
    }

    NodeOffset = fdt_next_subnode (Dtb, NodeOffset);
  }

  // Create the /psci node if it doesn't exist
  NodeOffset = fdt_subnode_offset (Dtb, 0, "psci");
  if (NodeOffset < 0) {
    NodeOffset = fdt_add_subnode (Dtb, 0, "psci");
    if (NodeOffset < 0) {
      DEBUG ((DEBUG_ERROR, "fdt_add_subnode: Could not add psci!!, %a\n", fdt_strerror (NodeOffset)));
      return EFI_DEVICE_ERROR;
    }

    FdtStatus = fdt_setprop_string (Dtb, NodeOffset, "compatible", "arm,psci-0.2");
    if (FdtStatus) {
      DEBUG ((DEBUG_ERROR, "fdt_setprop/psci: Could not add compatiblity, %a!!\n", fdt_strerror (FdtStatus)));
      return EFI_DEVICE_ERROR;
    }

    FdtStatus = fdt_setprop_string (Dtb, NodeOffset, "method", "smc");
    if (FdtStatus) {
      DEBUG ((DEBUG_ERROR, "fdt_setprop/psci: Could not add method, %a!!\n", fdt_strerror (FdtStatus)));
      return EFI_DEVICE_ERROR;
    }
  }

  DEBUG ((DEBUG_INFO, "PSCI fixup done!!!!\n"));

  return EFI_SUCCESS;
}

/**
  Return a pool allocated copy of the DTB image that is appropriate for
  booting the current platform via DT.

  @param[out]   Dtb                   Pointer to the DTB copy
  @param[out]   DtbSize               Size of the DTB copy

  @retval       EFI_SUCCESS           Operation completed successfully
  @retval       EFI_NOT_FOUND         No suitable DTB image could be located
  @retval       EFI_OUT_OF_RESOURCES  No pool memory available
  @retval       EFI_BAD_BUFFER_SIZE   Dtb could not be copied to allocated pool

**/
EFI_STATUS
EFIAPI
DtPlatformLoadDtb (
  OUT   VOID        **Dtb,
  OUT   UINTN       *DtbSize
  )
{
  EFI_STATUS      Status;
  VOID            *OrigDtb;

  Status = EFI_SUCCESS;

  OrigDtb = (VOID *)FixedPcdGet64 (PcdFdtAddress);
  if (fdt_check_header (OrigDtb)) {
    DEBUG((EFI_D_ERROR, "Invalid Device Tree\n"));
    return EFI_NOT_FOUND;
  }

  // Assign extra memory for fixups
  *DtbSize = fdt_totalsize (OrigDtb) + SIZE_512KB;

  *Dtb = AllocateCopyPool (*DtbSize, OrigDtb);
  if (*Dtb == NULL) {
    return EFI_OUT_OF_RESOURCES;
  }

  // TODO: Verify the signed dtb image and then copy
  if (fdt_open_into (*Dtb, *Dtb, *DtbSize)) {
    Status = EFI_BAD_BUFFER_SIZE;
    goto FreeDtb;
  }

  // we use PSCI boot method for all platforms
  Status = FdtCpuFixup (*Dtb);
  if (EFI_ERROR (Status)) {
    goto FreeDtb;
  };

  Status = FdtSocFixup (*Dtb);
  if (EFI_ERROR (Status)) {
    goto FreeDtb;
  };

  return Status;

FreeDtb:
  if (*Dtb != NULL) {
    FreePool (*Dtb);
  }

  *Dtb = NULL;
  *DtbSize = 0;

  return Status;
}
