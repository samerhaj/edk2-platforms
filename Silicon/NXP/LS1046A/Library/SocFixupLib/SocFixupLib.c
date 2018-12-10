/** @file

 Copyright 2018 NXP

 This program and the accompanying materials
 are licensed and made available under the terms and conditions of the BSD License
 which accompanies this distribution.  The full text of the license may be found at
 http://opensource.org/licenses/bsd-license.php

 THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
 WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include <libfdt.h>
#include <Chassis.h>
#include <Soc.h>
#include <Library/DebugLib.h>
#include <Library/ItbParse.h>
#include <Library/SocFixupLib.h>

/**
  Fixup the device tree based on running SOC's properties.

  @param[in]  Dtb   The device tree to fixup.

  @return EFI_SUCCESS       Successfully fix up the device tree
  @return EFI_DEVICE_ERROR  Could not fixup the device tree
  @return EFI_NOT_FOUND     Could not found the device tree overlay file
  @retval EFI_CRC_ERROR     Device tree overlay file is not correct.
**/
EFI_STATUS
FdtSocFixup (
  IN  VOID  *Dtb
  )
{
  INTN        NodeOffset;
  INTN        SubNodeOffset;
  UINT64      ChipSelect;
  EFI_STATUS  Status;
  INT32       FdtStatus;

  // Add uefi-runtime property in first Qspi node and the Chip Select 0 (if not already present)
  // TODO: Add this to the boot source node, which can be determined from RCW
  NodeOffset = fdt_node_offset_by_compatible (Dtb, -1, (VOID *)(PcdGetPtr (PcdQspiFdtCompatible)));
  if (NodeOffset == FDT_ERR_NOTFOUND) {
    DEBUG ((DEBUG_ERROR, "Error: can't find node %a in Dtb\n", (CHAR8* )PcdGetPtr (PcdQspiFdtCompatible)));
    return EFI_NOT_FOUND;
  }

  FdtStatus = fdt_setprop_empty (Dtb, NodeOffset, "uefi-runtime");
  if (FdtStatus) {
    DEBUG ((DEBUG_ERROR, "Error: can't set uefi-runtime %a\n", fdt_strerror (FdtStatus)));
    return EFI_DEVICE_ERROR;
  }

  fdt_for_each_subnode (SubNodeOffset, Dtb, NodeOffset) {
    Status = FdtGetAddressSize (Dtb, SubNodeOffset, "reg", 0, &ChipSelect, NULL);
    if (EFI_ERROR (Status)) {
      DEBUG ((DEBUG_ERROR, "Error: can't get ChipSelect (Status = %r)!\n", Status));
      return EFI_DEVICE_ERROR;
    }

    if (ChipSelect == 0) {
      FdtStatus = fdt_setprop_empty (Dtb, SubNodeOffset, "uefi-runtime");
      if (FdtStatus) {
        DEBUG ((DEBUG_ERROR, "Error: can't set uefi-runtime %a\n", fdt_strerror (FdtStatus)));
        return EFI_DEVICE_ERROR;
      }

      break;
    }
  }

  return EFI_SUCCESS;
}

/**
  Retrive the System Version from System Version Register (SVR)

  @return  0  could not read SVR register or invalid value in SVR register
  @return     SVR register value
**/
UINT32
SocGetSvr (
  )
{
  CCSR_GUR     *GurBase;

  GurBase = (VOID *)PcdGet64 (PcdGutsBaseAddr);
  ASSERT (GurBase != NULL);

  return GurRead ( (UINTN)&GurBase->Svr);
}
