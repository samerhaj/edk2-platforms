/** @file

  Fixup the ICIDs of legacy devices in DCFG space and create
  iommus property in corresponding device's Device tree node.

  Copyright 2019 NXP

  SPDX-License-Identifier: BSD-2-Clause

**/
#include <Library/DebugLib.h>
#include <Library/ItbParse.h>
#include <Library/UefiLib.h>
#include <Soc.h>
#include <Uefi.h>
#include <libfdt.h>

#include "Platform.h"

/**
  Get the Phandle of SMMU Node.
  if no phandle exists for SMMU node create one and return that phandle

  @param  Fdt      Pointer to Flattened device tree.
  @param  Phandle  Phandle of SMMU Node.

  @return EFI_SUCCESS       Phandle of SMMU Node.
  @retval EFI_NOT_FOUND     No node found.
  @return EFI_DEVICE_ERROR  No Phandle present in SMMU node and
                            could not create phandle for SMMU node.
**/
STATIC
EFI_STATUS
FdtGetSmmuPhandle (
  IN  VOID  *Fdt,
  OUT INTN  *Phandle
  )
{
  INTN          NodeOffset;
  INTN          SmmuPhandle;
  INTN          FdtStatus;

  NodeOffset = fdt_node_offset_by_compatible (Fdt, -1, "arm,mmu-500");
  if (NodeOffset == -FDT_ERR_NOTFOUND) {
    DEBUG ( (DEBUG_WARN, "failed to get smmu node\n"));
    return EFI_NOT_FOUND;
  }

  SmmuPhandle = fdt_get_phandle (Fdt, NodeOffset);
  if (SmmuPhandle == 0) {
    SmmuPhandle = fdt_get_max_phandle (Fdt);
    if (SmmuPhandle > 0 && SmmuPhandle != (UINT32)-1) {
      SmmuPhandle += 1;
      FdtStatus = fdt_setprop_u32 (Fdt, NodeOffset, "phandle", SmmuPhandle);
      if (FdtStatus < 0) {
        DEBUG ((DEBUG_ERROR, "Could not add phandle property : %s\n", fdt_strerror (FdtStatus)));
        return EFI_DEVICE_ERROR;
      }

      FdtStatus = fdt_setprop_u32 (Fdt, NodeOffset, "linux,phandle", SmmuPhandle);
      if (FdtStatus < 0) {
        DEBUG ((DEBUG_ERROR, "Could not add phandle property : %s\n", fdt_strerror (FdtStatus)));
        return EFI_DEVICE_ERROR;
      }
    } else {
      DEBUG ((DEBUG_ERROR, "Could not get maximum phandle\n"));
      return EFI_DEVICE_ERROR;
    }
  }

  *Phandle = SmmuPhandle;

  return EFI_SUCCESS;
}

/**
  Get the Node offset in Flattened device tree based on Compatible property
  and Address.

  @param  Fdt          Pointer to Flattened device tree.
  @param  Compatible   Compatible string to find device node in device tree.
  @param  RegName      Name of address of devices' registers in reg-name property.
                       if this String is NULL then "regs" is searched for address
                       in reg-names. if that is not found then it's assumed that first
                       address in "regs" is device's address.
  @param  Address      The reg property in node is searched to find
                       the address of device. That address is compared with
                       Address to find the correct device node.

  @return                    Node Offset in device tree
  @retval -FDT_ERR_NOTFOUND  No node found.
**/
STATIC
INTN
FdtGetNodeOffset (
  IN VOID           *Fdt,
  IN CHAR8          *Compatible,
  IN CHAR8          *RegName,
  IN UINTN          Address
  )
{
  INTN              NodeOffset;
  UINT64            DeviceAddress;
  INT32             RegIndex;
  EFI_STATUS        Status;

  NodeOffset = fdt_node_offset_by_compatible (Fdt, -1, Compatible);
  while (NodeOffset != -FDT_ERR_NOTFOUND) {
    // Get the Index of device's registers
    if (RegName) {
      RegIndex = fdt_stringlist_search (Fdt, NodeOffset, "reg-names", RegName);
    } else {
      RegIndex = fdt_stringlist_search (Fdt, NodeOffset, "reg-names", "regs");
    }
    if (RegIndex < 0) {
      RegIndex = 0;
    }
    // Get the device's registers' address from node.
    Status = FdtGetAddressSize (Fdt, NodeOffset, "reg", RegIndex, &DeviceAddress, NULL);
    if (EFI_ERROR (Status)) {
      DEBUG ((DEBUG_ERROR, "Error: can't get regs base address(Status = %r)!\n", Status));
      continue;
    }
    if (Address == DeviceAddress) {
      break;
    }

    NodeOffset = fdt_node_offset_by_compatible (Fdt, NodeOffset, Compatible);
  }

  return NodeOffset;
}

/**
  Fixup the ICIDs of legacy devices in Flattened Device Tree

  @param  Fdt          Pointer to Flattened device tree.
  @param  NodeOffset   NodeOffset in which ICIDs are to be fixed.
  @param  SmmuPhandle  Phandle of SMMU Node.
  @param  Icids        Array of ICID value to fix in device node.
  @param  NumIcids     Size of Icids array

  @return EFI_SUCCESS       Icid value fixed successfully.
  @return EFI_DEVICE_ERROR  Could not fix ICID value in device tree
**/
EFI_STATUS
EFIAPI
FdtFixupIcids (
  IN VOID           *Fdt,
  IN INTN           NodeOffset,
  IN INTN           SmmuPhandle,
  IN UINT32         *Icids,
  IN UINT32         NumIcids
  )
{
  UINT32        Prop[2];
  UINT32        Index;
  INTN          FdtStatus;

  for (Index = 0; Index < NumIcids; Index++) {
    Prop[0] = cpu_to_fdt32(SmmuPhandle);
    Prop[1] = cpu_to_fdt32(Icids[Index]);

    if (Index == 0) {
      FdtStatus = fdt_setprop(Fdt, NodeOffset, "iommus", Prop, sizeof(Prop));
    } else {
      FdtStatus = fdt_appendprop(Fdt, NodeOffset, "iommus", Prop, sizeof(Prop));
    }

    if (FdtStatus) {
      DEBUG ((
        DEBUG_ERROR, "error %a setting iommus for %a\n",
        fdt_strerror (FdtStatus), fdt_get_name (Fdt, NodeOffset, NULL)
        ));
      return EFI_DEVICE_ERROR;
    }
  }

  return EFI_SUCCESS;
}

/**
  Entry point of the Icid Application.
  Fixup the ICIDs of legacy devices in DCFG space

  @param  ImageHandle  A handle for the image that is initializing this driver.
  @param  SystemTable  A pointer to the EFI system table.

  @return EFI_SUCCESS  Driver initialized successfully.

**/
EFI_STATUS
EFIAPI
FixupIcid (
  IN EFI_HANDLE           ImageHandle,
  IN EFI_SYSTEM_TABLE     *SystemTable
  )
{
  VOID                  *Fdt;
  CCSR_GUR              *GurBase;
  INTN                  SmmuPhandle;
  INTN                  NodeOffset;
  UINT32                Icid;
  EFI_STATUS            Status;

  GurBase = (VOID *)PcdGet64 (PcdGutsBaseAddr);

  Status = EfiGetSystemConfigurationTable (&gFdtTableGuid, &Fdt);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "%a Did not find the Fdt Blob.\n", __FUNCTION__));
    return Status;
  }

  SmmuPhandle = -1;
  Status = FdtGetSmmuPhandle (Fdt, &SmmuPhandle);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  Icid = NXP_USB0_STREAM_ID;
  GurWrite ( (UINTN)&GurBase->Usb1Amqr, Icid);
  NodeOffset = FdtGetNodeOffset (Fdt, "snps,dwc3", NULL, USB0_BASE);
  if (NodeOffset != -FDT_ERR_NOTFOUND) {
    Status = FdtFixupIcids (Fdt, NodeOffset, SmmuPhandle, &Icid, 1);
    if (EFI_ERROR (Status)) {
      return Status;
    }
  }

  Icid = NXP_USB1_STREAM_ID;
  GurWrite ( (UINTN)&GurBase->Usb2Amqr, Icid);
  NodeOffset = FdtGetNodeOffset (Fdt, "snps,dwc3", NULL, USB1_BASE);
  if (NodeOffset != -FDT_ERR_NOTFOUND) {
    Status = FdtFixupIcids (Fdt, NodeOffset, SmmuPhandle, &Icid, 1);
    if (EFI_ERROR (Status)) {
      return Status;
    }
  }

  Icid = NXP_SDMMC0_STREAM_ID;
  GurWrite ( (UINTN)&GurBase->Sdmm1Amqr, Icid);
  NodeOffset = FdtGetNodeOffset (Fdt, "fsl,esdhc", NULL, SDC0_BASE);
  if (NodeOffset != -FDT_ERR_NOTFOUND) {
    Status = FdtFixupIcids (Fdt, NodeOffset, SmmuPhandle, &Icid, 1);
    if (EFI_ERROR (Status)) {
      return Status;
    }
  }

  Icid = NXP_SDMMC1_STREAM_ID;
  GurWrite ( (UINTN)&GurBase->Sdmm2Amqr, Icid);
  NodeOffset = FdtGetNodeOffset (Fdt, "fsl,esdhc", NULL, SDC1_BASE);
  if (NodeOffset != -FDT_ERR_NOTFOUND) {
    Status = FdtFixupIcids (Fdt, NodeOffset, SmmuPhandle, &Icid, 1);
    if (EFI_ERROR (Status)) {
      return Status;
    }
  }

  return EFI_SUCCESS;
}
