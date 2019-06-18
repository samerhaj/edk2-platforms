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
#include <PiDxe.h>
#include <Soc.h>
#include <Library/DebugLib.h>
#include <Library/DxeServicesLib.h>
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
  CCSR_GUR        *GurBase;
  CONST fdt32_t   *Prop;
  EFI_STATUS      Status;
  INT32           PropLen;
  INTN            FdtStatus;
  UINTN           DtbOverlaySize;
  UINTN           Svr;
  UINTN           Index;
  VOID            *DtbOverlay;
  INT32           NodeOffset;

  GurBase = (VOID *)PcdGet64 (PcdGutsBaseAddr);
  ASSERT (GurBase != NULL);

  Svr = GurRead ( (UINTN)&GurBase->Svr);

  Index = 1;
  // Apply the device tree overlays one by one
  while (TRUE) {
    Status = GetSectionFromAnyFv (
               &gDtPlatformDefaultDtbFileGuid,
               EFI_SECTION_RAW,
               Index,
               &DtbOverlay,
               &DtbOverlaySize
               );
    // If no section found, that means all overlay files have been processed
    if (Status == EFI_NOT_FOUND) {
      Status = EFI_SUCCESS;
      break;
    }

    if (EFI_ERROR (Status)) {
      DEBUG ((DEBUG_ERROR, "Failed to get device tree overlay Index %d Status = %r\n", Index, Status));
      break;
    }

    if (fdt_check_header (DtbOverlay)) {
      DEBUG ((DEBUG_ERROR, "Invalid Device Tree Overlay Index %d\n", Index));
      Status = EFI_CRC_ERROR;
      break;
    }

    // Test if overlay is for symbols
    if (!fdt_node_check_compatible (DtbOverlay, 0, "fsl,ls1043a-symbols")) {
      NodeOffset = fdt_path_offset (Dtb, "/__symbols__");
      if (NodeOffset < 0) {
        // only apply if the device tree doesn't contain auto generated symbols
        FdtStatus = fdt_overlay_apply (Dtb, DtbOverlay);
        if (FdtStatus) {
          DEBUG ((
            EFI_D_ERROR,
            "fdt_overlay_apply/failed to apply overlay file @ index %d: %a\n",
            Index, fdt_strerror (FdtStatus)
            ));
          Status = EFI_DEVICE_ERROR;
          break;
        }
      }
    }

    // Test if overlay is for soc revision
    if (!fdt_node_check_compatible (DtbOverlay, 0, "fsl,ls1043a-rev")) {
      // Get SOC revision from overlay and compare with SOC revision present read from SOC
      Prop = fdt_getprop (DtbOverlay, 0, "reg", &PropLen);
      if (Prop == NULL) {
        DEBUG ((DEBUG_ERROR, "Device Tree Overlay Index %d has no reg property\n", Index));
        Status = EFI_NOT_FOUND;
        break;
      }

      if (SVR_MINOR (Svr) == fdt32_to_cpu (*Prop)) {
        FdtStatus = fdt_overlay_apply (Dtb, DtbOverlay);
        if (FdtStatus) {
          DEBUG ((
            EFI_D_ERROR,
            "fdt_overlay_apply/failed to apply overlay file @ index %d: %a\n",
            Index, fdt_strerror (FdtStatus)
            ));
          Status = EFI_DEVICE_ERROR;
          break;
        }
      }
    }

    // Move onto next Overlay File
    Index++;
  }

  if (Svr & BIT11) {
    // 2 core variant of LS1043A SOC : LS1023A present. disable cpu 2 and 3
    NodeOffset = fdt_path_offset (Dtb, "/cpus/cpu@2");
    if (NodeOffset >= 0) {
      FdtStatus = fdt_del_node (Dtb, NodeOffset);
      if (FdtStatus) {
        DEBUG ((
          EFI_D_ERROR,
          "fdt_del_node/failed to delete node /cpus/cpu@2: %a\n",
          fdt_strerror (FdtStatus)
          ));
        return EFI_DEVICE_ERROR;
      }
    }

    NodeOffset = fdt_path_offset (Dtb, "/cpus/cpu@3");
    if (NodeOffset >= 0) {
      FdtStatus = fdt_del_node (Dtb, NodeOffset);
      if (FdtStatus) {
        DEBUG ((
          EFI_D_ERROR,
          "fdt_del_node/failed to delete node /cpus/cpu@3: %a\n",
          fdt_strerror (FdtStatus)
          ));
        return EFI_DEVICE_ERROR;
      }
    }
  }

  //
  // if overlay is applied, check if gic address are changed too
  //
  if (Index > 1) {
    CONST UINT64  *RegProp;
    INTN Offset;
    UINT64 GicDistributorBase;
    UINT64 GicInterruptInterfaceBase;

    //
    // Check for Gic node , DT must have gic
    // 
    Offset =  fdt_node_offset_by_compatible (Dtb, -1, "arm,gic-400");

    ASSERT (Offset != -FDT_ERR_NOTFOUND);

    RegProp = fdt_getprop (Dtb, Offset, "reg", &PropLen);
    ASSERT (PropLen == 64);

    //
    // Get the Gic Distributor Base
    //
    GicDistributorBase = fdt64_to_cpu (ReadUnaligned64 (RegProp));

    //
    // Move to Interrupt Interface Base
    //
    RegProp +=2;
    //
    // Get the Gic Distributor Base
    //
    GicInterruptInterfaceBase = fdt64_to_cpu (ReadUnaligned64 (RegProp));
    DEBUG ((DEBUG_INFO, "Gic DistBase %x Interrupt Base %x \n", GicDistributorBase, GicInterruptInterfaceBase));

    //
    // Changed GIC base addresses
    //
    PcdSet64S (PcdGicDistributorBase, GicDistributorBase);
    PcdSet64S (PcdGicInterruptInterfaceBase, GicInterruptInterfaceBase);
  }
  return Status;
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
  UINT32        Svr;

  GurBase = (VOID *)PcdGet64 (PcdGutsBaseAddr);
  ASSERT (GurBase != NULL);

  Svr = GurRead ((UINTN)&GurBase->Svr);

  /* Store the SVR value */
  PcdSet32(PcdSocSvr, (UINT32)Svr);

  return Svr;
}
