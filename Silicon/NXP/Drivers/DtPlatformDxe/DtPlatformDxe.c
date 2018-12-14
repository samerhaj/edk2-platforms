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
#include <Chassis.h>
#include <Library/BaseLib.h>
#include <Library/BeIoLib.h>
#include <Library/DebugLib.h>
#include <Library/DevicePathLib.h>
#include <Library/DxeServicesLib.h>
#include <Library/IoLib.h>
#include <Library/ItbParse.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/SocClockLib.h>
#include <Library/SocFixupLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiDriverEntryPoint.h>

#include "DtPlatformDxe.h"

/**
  The functions fixes the cpu nodes present under cpus node in device tree with the
  cpu enable-method as psci.

  for more details please refer https://www.kernel.org/doc/Documentation/devicetree/bindings/arm/cpus.txt
  // TODO: delete the CPU nodes that are not present in SOC depending on SOC flavor

  @param[in] Dtb        Device Tree to fix up.

  @retval EFI_SUCCESS       enable method fixed successfully in cpu nodes.
  @retval EFI_NOT_FOUND     "cpus" node not found or cpus node doesn't contain any "cpu" subnode
  @retval EFI_DEVICE_ERROR  Failed to set values in device tree
**/

STATIC
EFI_STATUS
FdtCpuFixup (
  IN  VOID *Dtb
  )
{
  INTN                        ParentOffset;
  INTN                        NodeOffset;
  INTN                        FdtStatus;
  CONST struct fdt_property   *Prop;
  INT32                       PropLen;

  ParentOffset = fdt_subnode_offset (Dtb, 0, "cpus");
  if (ParentOffset < 0) {
    DEBUG ((DEBUG_ERROR, "Fdt: No cpus node found!!\n\n"));
    return EFI_NOT_FOUND;
  }

  fdt_for_each_subnode (NodeOffset, Dtb, ParentOffset) {
    Prop = fdt_get_property(Dtb, NodeOffset, "device_type", &PropLen);
    if (!Prop) {
      continue;
    }
    if (PropLen < 4) {
      continue;
    }
    if (AsciiStrCmp (Prop->data, "cpu")) {
      continue;
    }

    FdtStatus = fdt_setprop_string (Dtb, NodeOffset, "enable-method", "psci");
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
  The functions fixes the Input system clock frequency (SYSCLK) in device tree

  for more details please refer https://www.kernel.org/doc/Documentation/devicetree/bindings/clock/qoriq-clock.txt

  @param[in] Dtb        Device Tree to fix up.

  @retval EFI_SUCCESS       system's SYSCLK value updated in sysclk node in device tree
  @retval EFI_NOT_FOUND     Either could not get the valid SYSCLK value or
                            the sysclk node not found in device tree
  @retval EFI_DEVICE_ERROR  Failed to set values in device tree
**/
STATIC
EFI_STATUS
FdtSysClockFixup (
  IN  VOID *Dtb
  )
{
  UINT32  SysClk;
  INTN    NodeOffset;
  INTN    FdtStatus;

  SysClk = SocGetClock (IP_SYSCLK, 0);
  if (SysClk == 0) {
    DEBUG ((DEBUG_ERROR, "Invalid System Clock\n"));
    return EFI_NOT_FOUND;
  }

  NodeOffset = fdt_path_offset (Dtb, "/sysclk");
  if (NodeOffset < 0) {
    DEBUG ((DEBUG_ERROR, "No sysclk nodes found!!!\n"));
    return EFI_NOT_FOUND;
  }

  FdtStatus = fdt_setprop_u32 (Dtb, NodeOffset, "clock-frequency", SysClk);
  if (FdtStatus) {
    DEBUG ((DEBUG_ERROR, "fdt_setprop/sysclk: Could not add property, %a!!\n", fdt_strerror (FdtStatus)));
    return EFI_DEVICE_ERROR;
  }

  return EFI_SUCCESS;
}

/**
  Get the Crypto block's era property based on its ID and Rev

  @param[in] CryptoAddress  Address of crypto block
  @param[in] BigEndian      If the crypto block is big endian or not?

  @retval   0   No valid Era found for Crypto block
  @retval       The era property
**/
STATIC
UINT8
GetCryptoEra (
 IN UINT64   CryptoAddress,
 IN BOOLEAN  BigEndian
 )
{
  UINT32 SecVidMs;
  UINT32 CcbVid;
  UINT16 Id;
  UINT8 Rev;
  UINT8 Era;
  UINT32 Index;

  if (BigEndian) {
    SecVidMs = BeMmioRead32 (CryptoAddress + VIDMS_OFFSET);
    CcbVid = BeMmioRead32 (CryptoAddress + CCBVID_OFFSET);
  } else {
    SecVidMs = MmioRead32 (CryptoAddress + VIDMS_OFFSET);
    CcbVid = MmioRead32 (CryptoAddress + CCBVID_OFFSET);
  }

  Id = (SecVidMs & SECVID_MS_IPID_MASK) >> SECVID_MS_IPID_SHIFT;
  Rev = (SecVidMs & SECVID_MS_MAJ_REV_MASK) >> SECVID_MS_MAJ_REV_SHIFT;
  Era = (CcbVid & CCBVID_ERA_MASK) >> CCBVID_ERA_SHIFT;

  if (Era) {
    return Era;
  }

  for (Index = 0; Index < ARRAY_SIZE(Eras); Index++) {
    if (Eras[Index].Id == Id && Eras[Index].Rev == Rev) {
      return Eras[Index].Era;
    }
  }

  DEBUG((DEBUG_WARN, "Unable to get ERA for CAAM rev: %u\n", SecVidMs));

  return 0;
}

/**
  either delete Crypto node or fixup the era property in crypto node.

  @param[in]  Dtb           Dtb node to fixup
  @param[in]  DeleteCrypto  Weather to delete the crypto node ot to fixup version info in it?

  @retval EFI_SUCCESS       deleted the crypto node in device tree or fixup version info in it
                            or crypto node doesn't exist in device tree
  @retval EFI_DEVICE_ERROR  Failed to delete node in device tree or failed to fixup version info in it
                            or failed to delete the node reference from aliases
**/
STATIC
EFI_STATUS
FdtFixupCrypto (
  IN  VOID     *Dtb,
  IN  BOOLEAN  DeleteCrypto
  )
{
  INTN       NodeOffset;
  INTN       FdtStatus;
  UINT64     CryptoAddress;
  BOOLEAN    CryptoBigEndian;
  UINT64     JobRingOffset;
  UINT8      Era;
  EFI_STATUS Status;

  NodeOffset = fdt_path_offset (Dtb, "crypto");
  if (NodeOffset < 0) {
    return EFI_SUCCESS;
  }

  if (DeleteCrypto) {
    FdtStatus = fdt_del_node(Dtb, NodeOffset);
    if (FdtStatus) {
      DEBUG ((DEBUG_ERROR, "fdt_del_node/crypto: Could not delete node, %a!!\n", fdt_strerror (FdtStatus)));
      return EFI_DEVICE_ERROR;
    }

    NodeOffset = fdt_path_offset (Dtb, "/aliases");
    if (NodeOffset >= 0) {
      FdtStatus = fdt_delprop (Dtb, NodeOffset, "crypto");
      if (FdtStatus && FdtStatus != -FDT_ERR_NOTFOUND) {
        DEBUG ((DEBUG_ERROR, "fdt_delprop/crypto: Could not delete alias, %a!!\n", fdt_strerror (FdtStatus)));
      }
    }
  } else {
    Status = FdtGetAddressSize (Dtb, NodeOffset, "reg", 0, &CryptoAddress, NULL);
    if (EFI_ERROR (Status)) {
      DEBUG ((DEBUG_ERROR, "Error: can't get regs base address(Status = %r)!\n", Status));
      return EFI_SUCCESS;
    }

    CryptoBigEndian = !!(BeMmioRead32 (CryptoAddress + SSTA_OFFSET) & (SSTA_PLEND | SSTA_ALT_PLEND));
    Era = GetCryptoEra (CryptoAddress, CryptoBigEndian);

    if (Era) {
      FdtStatus = fdt_setprop_u32 (Dtb, NodeOffset, "fsl,sec-era", Era);
      if (FdtStatus) {
        DEBUG ((DEBUG_ERROR, "fdt_setprop_u32/sec-era could not set property %a\n", fdt_strerror (FdtStatus)));
        return EFI_DEVICE_ERROR;
      }
    }

    for (NodeOffset = fdt_node_offset_by_compatible (Dtb, NodeOffset, "fsl,sec-v4.0-job-ring");
         NodeOffset != -FDT_ERR_NOTFOUND;
         NodeOffset = fdt_node_offset_by_compatible (Dtb, NodeOffset, "fsl,sec-v4.0-job-ring")) {
      Status = FdtGetAddressSize (Dtb, NodeOffset, "reg", 0, &JobRingOffset, NULL);
      if (EFI_ERROR (Status)) {
        DEBUG ((DEBUG_ERROR, "Error: can't get regs base address(Status = %r)!\n", Status));
        return EFI_SUCCESS;
      }

      if (JobRingOffset == JR3_OFFSET) {
        FdtStatus = fdt_del_node (Dtb, NodeOffset);
        if (FdtStatus) {
          DEBUG ((DEBUG_ERROR, "fdt_del_node/crypto: Could not delete node, %a!!\n", fdt_strerror (FdtStatus)));
          return EFI_DEVICE_ERROR;
        }

        break;
      }
    }
  }

  return EFI_SUCCESS;
}

/**
  Add/fix "clock-frequency" property in DUART(ns16550) node.

  @retval EFI_SUCCESS       system's DUART clock value updated in DUART node in device tree
  @retval EFI_DEVICE_ERROR  Failed to set values in device tree
**/
STATIC
EFI_STATUS
FdtFixupDuart (
  IN  VOID *Dtb
  )
{
  INTN     NodeOffset;
  INTN     FdtStatus;
  UINT32   DuartClk;

  DuartClk = SocGetClock (IP_DUART, 0);
  if (DuartClk == 0) {
    DEBUG ((DEBUG_WARN, "Invalid Duart Clock\n"));
    return EFI_SUCCESS;
  }

  for (NodeOffset = fdt_node_offset_by_compatible (Dtb, -1, "fsl,ns16550");
       NodeOffset >= 0;
       NodeOffset = fdt_node_offset_by_compatible (Dtb, NodeOffset, "fsl,ns16550")) {
    FdtStatus = fdt_setprop_u32(Dtb, NodeOffset, "clock-frequency", DuartClk);
    if (FdtStatus) {
      DEBUG ((DEBUG_ERROR, "fdt_setprop_u32/Duart: Could not set clock-frequency, %a!!\n", fdt_strerror (FdtStatus)));
      return EFI_DEVICE_ERROR;
    }
  }

  return EFI_SUCCESS;
}

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
  UINT32                          Svr;

  Dtb = NULL;
  OrigDtb = NULL;
  Status = EFI_SUCCESS;

  Svr = SocGetSvr ();
  ASSERT (Svr != 0);

  if (PcdGet64 (PcdFdtAddress)) {
    OrigDtb = (VOID *)PcdGet64 (PcdFdtAddress);

    if (!fdt_check_header (OrigDtb)) {
      goto FdtLoadedFromAddress;
    }
  }

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

FdtLoadedFromAddress:
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

  // we use PSCI boot method for all platforms
  Status = FdtCpuFixup (Dtb);
  if (EFI_ERROR (Status)) {
    goto FreeDtb;
  };

  // All Platforms need SYS clock frequency
  Status = FdtSysClockFixup (Dtb);
  if (EFI_ERROR (Status)) {
    goto FreeDtb;
  }

  if (!IS_E_PROCESSOR (Svr)) {
    // delete crypto node if not on an E-processor
    Status = FdtFixupCrypto (Dtb, TRUE);
  } else {
    // fix the era version in crypto node
    Status = FdtFixupCrypto (Dtb, FALSE);
  }
  if (EFI_ERROR (Status)) {
    return EFI_DEVICE_ERROR;
  }

  // Fixup clock-frequency in DUART node
  // TODO: Remove this fixup when linux driver has added clockgen support
  Status = FdtFixupDuart (Dtb);
  if (EFI_ERROR (Status)) {
    goto FreeDtb;
  }

  Status = FdtSocFixup (Dtb);
  if (EFI_ERROR (Status)) {
    goto FreeDtb;
  }

  //
  // install a reference to it as the FDT configuration table.
  //
  Status = gBS->InstallConfigurationTable (&gFdtTableGuid, Dtb);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "%a: failed to install FDT configuration table %r\n",
      __FUNCTION__, Status));
    goto FreeDtb;
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
    goto FreeDtb;
  }

  return EFI_SUCCESS;

FreeDtb:
  if (Dtb != NULL) {
    FreePool (Dtb);
  }

  ASSERT_EFI_ERROR (Status);
  CpuDeadLoop ();
  //
  // Keep compilers happy.
  //

  return Status;
}

