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
#include <Library/BeIoLib.h>
#include <Library/DebugLib.h>
#include <Library/DevicePathLib.h>
#include <Library/DxeServicesLib.h>
#include <Library/IoLib.h>
#include <Library/ItbParse.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/PrintLib.h>
#include <Library/SocClockLib.h>
#include <Library/SysEepromLib.h>
#include <Library/UefiLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiDriverEntryPoint.h>

#include "Dpaa1EthernetDxe.h"

/**
  Frees the Fman buffer allocated by GetFmanFirmware

  @param[in]  FmanFw   Pointer to QeFirmware structure returned by GetFmanFirmware
**/
VOID
FreeFmanFirmware (
  IN  QeFirmware *FmanFw
  )
{
  // memory for Preset Value 0xffffffff was allocated by GetFmanFirmware
  FreePool ((VOID *)FmanFw - sizeof (UINT32));
}

/**
  Allocate the Memory for Fman Firmware and load the Fman Firmware from Flash and validate it
  If the firmware is correct, return its pointer in DDR memory

  To free the Fman memory allocated by this function, FreeFmanFirmware function should be called

  @param[out]  Length    if not null and retval is not null Length of Fman firmware
                         if not null and retval is null then containes the error code
  @retval                Null if no valid fman firmware found, otherwise fman firmware address in memory
**/
QeFirmware *
GetFmanFirmware (
  OUT  UINT32  *Length
  )
{
  QeFirmware  *FmanFw, *FmanFwFlash;
  UINT32      LengthFound;
  UINT32      Crc;
  UINT32      CrcCalculated;
  EFI_STATUS  Status;

  FmanFw = NULL;
  FmanFwFlash = (QeFirmware *)FixedPcdGet64 (PcdFmanFwFlashAddr);

  // If firmware not found, then exit silently
  if (FmanFwFlash == NULL) {
    Status = EFI_NOT_FOUND;
    goto Error;
  }

  if ((FmanFwFlash->Header.Magic[0] != 'Q') ||
      (FmanFwFlash->Header.Magic[1] != 'E') ||
      (FmanFwFlash->Header.Magic[2] != 'F')) {
    DEBUG ((DEBUG_ERROR, "Data at %p is not Fman Firmware\n", FmanFwFlash));
    Status = EFI_NOT_FOUND;
    goto Error;
  }

  LengthFound = fdt32_to_cpu (FmanFwFlash->Header.Length);
  LengthFound += sizeof (UINT32); // for Preset value
  FmanFw = (QeFirmware *)AllocatePool (LengthFound);
  if (FmanFw == NULL) {
    DEBUG ((DEBUG_ERROR, "%a could not allocate memory %d for Fman Firmware\n", __FUNCTION__, LengthFound));
    Status = EFI_OUT_OF_RESOURCES;
    goto Error;
  }

  *(UINT32 *)FmanFw = MAX_UINT32; // Add preset value 0xffffffff to FmanFirmware
  CopyMem ((VOID *)FmanFw + sizeof (UINT32), FmanFwFlash, LengthFound - sizeof (UINT32));
  LengthFound -= sizeof (UINT32);  /* Subtract the size of the CRC */
  Crc = fdt32_to_cpu (*(UINT32 *)( (VOID *)FmanFw + LengthFound));
  Status = gBS->CalculateCrc32 ( (VOID *)FmanFw, LengthFound, &CrcCalculated);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "Error while calculating Fman CRC %r\n", Status));
    goto Error;
  }

  CrcCalculated ^= MAX_UINT32;
  if (Crc != CrcCalculated) {
    DEBUG ((DEBUG_ERROR,
      "Fman firmware at %p has invalid CRC expected = 0x%x, calculated = 0x%x\n",
      FmanFwFlash, Crc, CrcCalculated
    ));
    Status = EFI_CRC_ERROR;
    goto Error;
  }

  if (Length) {
    *Length = LengthFound;
  }

  return ((VOID *)FmanFw + sizeof (UINT32));

Error:
  if (FmanFw) {
    FreePool (FmanFw);
  }
  if (Length) {
    *Length = Status;
  }

  return NULL;
}

/**
  insert the Fman firmware into the device tree

  The binding for an Fman firmware node is documented in
  https://www.kernel.org/doc/Documentation/devicetree/bindings/net/fsl-fman.txt.  This node contains
  the actual Fman firmware binary data.  The operating system is expected to
  be able to parse the binary data to determine any attributes it needs.

  @param[in] Dtb     Dtb Image into which firmware is to be inserted.

  @retval EFI_CRC_ERROR       Fman firmware CRC check failed
  @retval EFI_DEVICE_ERROR    Fail to add firmware to Device tree.
  @retval EFI_SUCCESS         Fman Firmware inserted into Device tree.
**/
EFI_STATUS
FdtFixupFmanFirmware (
  IN VOID *Dtb
  )
{
  INTN  FmanNode, FirmwareNode;
  QeFirmware  *FmanFw;
  UINT32      Length;
  UINT32      PHandle;
  INTN        FdtStatus;
  EFI_STATUS  Status;

  // The first Fman we find will contain the actual firmware.
  FmanNode = fdt_node_offset_by_compatible (Dtb, -1, "fsl,fman");
  if (FmanNode < 0) {
    // Exit silently if there are no Fman devices
    return EFI_SUCCESS;
  }

  // If we already have a firmware node, then also exit silently.
  if (fdt_node_offset_by_compatible (Dtb, -1, "fsl,fman-firmware") > 0) {
    return EFI_SUCCESS;
  }

  FmanFw = (QeFirmware *)GetFmanFirmware (&Length);
  if (FmanFw == NULL) {
    Status = Length;
    if (Status == EFI_NOT_FOUND) {
      // If firmware not found, then exit silently
      return EFI_SUCCESS;
    } else {
      return Status;
    }
  }

  /* Create the firmware node. */
  FirmwareNode = fdt_add_subnode (Dtb, FmanNode, "fman-firmware");
  if (FirmwareNode < 0) {
    DEBUG ((DEBUG_ERROR, "Could not add firmware node : %s\n", fdt_strerror (FirmwareNode)));
    return EFI_DEVICE_ERROR;
  }

  FdtStatus = fdt_setprop_string (Dtb, FirmwareNode, "compatible", "fsl,fman-firmware");
  if (FdtStatus < 0) {
    DEBUG ((DEBUG_ERROR, "Could not add compatible property : %s\n", fdt_strerror (FdtStatus)));
    return EFI_DEVICE_ERROR;
  }

  PHandle = fdt_get_max_phandle (Dtb);
  if (PHandle > 0 && PHandle != (UINT32)-1) {
    PHandle += 1;
    FdtStatus = fdt_setprop_u32 (Dtb, FirmwareNode, "phandle", PHandle);
    if (FdtStatus < 0) {
      DEBUG ((DEBUG_ERROR, "Could not add phandle property : %s\n", fdt_strerror (FdtStatus)));
      return EFI_DEVICE_ERROR;
    }

    FdtStatus = fdt_setprop_u32 (Dtb, FirmwareNode, "linux,phandle", PHandle);
    if (FdtStatus < 0) {
      DEBUG ((DEBUG_ERROR, "Could not add phandle property : %s\n", fdt_strerror (FdtStatus)));
      return EFI_DEVICE_ERROR;
    }
  } else {
    DEBUG ((DEBUG_ERROR, "Could not get maximum phandle\n"));
    return EFI_DEVICE_ERROR;
  }

  FdtStatus = fdt_setprop (Dtb, FirmwareNode, "fsl,firmware", FmanFw, Length);
  if (FdtStatus < 0) {
    DEBUG ((DEBUG_ERROR, "Could not add firmware property : %s\n", fdt_strerror (FdtStatus)));
    return EFI_DEVICE_ERROR;
  }

  /* Find all other Fman nodes and point them to the firmware node. */
  while ((FmanNode = fdt_node_offset_by_compatible (Dtb, FmanNode, "fsl,fman")) > 0) {
    FdtStatus = fdt_setprop_cell (Dtb, FmanNode, "fsl,firmware-phandle", PHandle);
    if (FdtStatus < 0) {
      DEBUG ((DEBUG_ERROR, "Could not add pointer property : %s\n", fdt_strerror (FdtStatus)));
      return EFI_DEVICE_ERROR;
    }
  }

  // before return free the Fman Firmware memory if no further use
  FreeFmanFirmware (FmanFw);

  return EFI_SUCCESS;
}

/**
  insert the Qman version info into the device tree
  and set the clock-frequency property in device tree with QMAN clock

  @param[in] Dtb     Dtb Image into which Qman version info is to be inserted.

  @retval EFI_DEVICE_ERROR    Fail to add Qman version info to Device tree.
  @retval EFI_SUCCESS         Qman version info inserted into Device tree.
**/
EFI_STATUS
FdtFixupQman (
  IN VOID *Dtb
  )
{
  EFI_STATUS Status;
  INT32      NodeOffset;
  UINT64     QmanAddress;
  INT32      FdtStatus;
  UINT8      Major, Minor, IpCfg;
  UINT32     QmanRev1, QmanRev2;
  CHAR8      CompatStr[64];
  UINT64     QmanClk;

  NodeOffset = fdt_node_offset_by_compatible (Dtb, -1, "fsl,qman");
  if (NodeOffset < 0) {
    // Exit silently if there are no Qman devices
    return EFI_SUCCESS;
  }

  QmanClk = SocGetClock (IP_QMAN, 0);
  if (!QmanClk) {
    DEBUG ((DEBUG_ERROR, "Invalid Qman clock\n"));
    return EFI_SUCCESS;
  }

  FdtStatus = fdt_setprop_u32 (Dtb, NodeOffset, "clock-frequency", QmanClk);
  if (FdtStatus) {
    DEBUG ((DEBUG_ERROR, "fdt_setprop/qman: Could not add property, %a!!\n", fdt_strerror (FdtStatus)));
    return EFI_DEVICE_ERROR;
  }

  Status = FdtGetAddressSize (Dtb, NodeOffset, "reg", 0, &QmanAddress, NULL);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "Error: can't get regs base address(Status = %r)!\n", Status));
    return EFI_SUCCESS;
  }

  if (fdt_getprop (Dtb, NodeOffset, "big-endian", NULL) != NULL) {
    QmanRev1 = BeMmioRead32 (QmanAddress + QMAN_IP_REV_1);
    QmanRev2 = BeMmioRead32 (QmanAddress + QMAN_IP_REV_2);
  } else {
    QmanRev1 = MmioRead32 (QmanAddress + QMAN_IP_REV_1);
    QmanRev2 = MmioRead32 (QmanAddress + QMAN_IP_REV_2);
  }

  Major = (QmanRev1 >> 8) & 0xff;
  Minor = QmanRev1 & 0xff;
  IpCfg = QmanRev2 & 0xff;

  AsciiSPrint (CompatStr, sizeof(CompatStr), "fsl,qman-portal-%u.%u.%u", Major, Minor, IpCfg);

  for (NodeOffset = fdt_node_offset_by_compatible (Dtb, -1, "fsl,qman-portal");
       NodeOffset >= 0;
       NodeOffset = fdt_node_offset_by_compatible (Dtb, NodeOffset, "fsl,qman-portal")) {
    FdtStatus = fdt_setprop_string (Dtb, NodeOffset, "compatible", CompatStr);
    if (FdtStatus < 0) {
      DEBUG ((DEBUG_ERROR, "fdt_appendprop/Qman : unable to set compatible %s", fdt_strerror (FdtStatus)));
      Status = EFI_DEVICE_ERROR;
      break;
    }

    FdtStatus = fdt_appendprop_string (Dtb, NodeOffset, "compatible", "fsl,qman-portal");
    if (FdtStatus < 0) {
      DEBUG ((DEBUG_ERROR, "fdt_appendprop/Qman : unable to set compatible %s", fdt_strerror (FdtStatus)));
      Status = EFI_DEVICE_ERROR;
      break;
    }
  }

  return Status;
}

/**
  insert the Bman version info into the device tree

  @param[in] Dtb     Dtb Image into which Bman version info is to be inserted.

  @retval EFI_DEVICE_ERROR    Fail to add Bman version info to Device tree.
  @retval EFI_SUCCES          Bman version info inserted into Device tree.
**/
EFI_STATUS
FdtFixupBmanVersion (
  IN VOID *Dtb
  )
{
  EFI_STATUS Status;
  INT32      NodeOffset;
  UINT64     BmanAddress;
  INT32      FdtStatus;
  UINT8      Major, Minor, IpCfg;
  UINT32     BmanRev1, BmanRev2;
  CHAR8      CompatStr[64];

  NodeOffset = fdt_node_offset_by_compatible (Dtb, -1, "fsl,bman");
  if (NodeOffset < 0) {
    // Exit silently if there are no Qman devices
    return EFI_SUCCESS;
  }

  Status = FdtGetAddressSize (Dtb, NodeOffset, "reg", 0, &BmanAddress, NULL);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "Error: can't get regs base address(Status = %r)!\n", Status));
    return EFI_SUCCESS;
  }

  if (fdt_getprop (Dtb, NodeOffset, "big-endian", NULL) != NULL) {
    BmanRev1 = BeMmioRead32 (BmanAddress + BMAN_IP_REV_1);
    BmanRev2 = BeMmioRead32 (BmanAddress + BMAN_IP_REV_2);
  } else {
    BmanRev1 = MmioRead32 (BmanAddress + BMAN_IP_REV_1);
    BmanRev2 = MmioRead32 (BmanAddress + BMAN_IP_REV_2);
  }

  Major = (BmanRev1 >> 8) & 0xff;
  Minor = BmanRev1 & 0xff;
  IpCfg = BmanRev2 & 0xff;

  AsciiSPrint (CompatStr, sizeof(CompatStr), "fsl,bman-portal-%u.%u.%u", Major, Minor, IpCfg);

  for (NodeOffset = fdt_node_offset_by_compatible (Dtb, -1, "fsl,bman-portal");
       NodeOffset >= 0;
       NodeOffset = fdt_node_offset_by_compatible (Dtb, NodeOffset, "fsl,bman-portal")) {
    FdtStatus = fdt_setprop_string (Dtb, NodeOffset, "compatible", CompatStr);
    if (FdtStatus < 0) {
      DEBUG ((DEBUG_ERROR, "fdt_appendprop/Bman : unable to set compatible %s", fdt_strerror (FdtStatus)));
      Status = EFI_DEVICE_ERROR;
      break;
    }

    FdtStatus = fdt_appendprop_string (Dtb, NodeOffset, "compatible", "fsl,bman-portal");
    if (FdtStatus < 0) {
      DEBUG ((DEBUG_ERROR, "fdt_appendprop/Bman : unable to set compatible %s", fdt_strerror (FdtStatus)));
      Status = EFI_DEVICE_ERROR;
      break;
    }
  }

  return Status;
}

/**
  insert the mac addresses into the device tree
  NOTE : the ethernet node enable/disable depending on serdes protocol
  should have been done before calling this function

  @param[in] Dtb     Dtb Image into which mac addresses are to be inserted.

  @retval EFI_DEVICE_ERROR    Fail to add mac addresses to Device tree.
  @retval EFI_SUCCES          mac addresses info inserted into Device tree.
**/
EFI_STATUS
FdtFixupMacAddresses (
  IN   VOID    *Dtb
  )
{
  UINT32             I, Prop;
  CONST CHAR8        *Path, *Name;
  UINT8              MacAddr[6];
  INT32              NodeOffset;
  EFI_STATUS         Status;
  INT32              FdtStatus;
  UINT32             EthernetId;
  INT32              PropLen;
  CONST fdt32_t      *PropCompatible;

  EthernetId = 0;

  if (fdt_path_offset(Dtb, "/aliases") < 0) {
    return EFI_SUCCESS;
  }

  /* Cycle through all aliases */
  for (Prop = 0; ; Prop++) {
    /* FDT might have been changed, recompute the offset */
    NodeOffset = fdt_first_property_offset(Dtb, fdt_path_offset(Dtb, "/aliases"));
    /* Select property number 'prop' */
    for (I = 0; I < Prop; I++) {
      NodeOffset = fdt_next_property_offset(Dtb, NodeOffset);
    }

    // No property left, get out of loop
    if (NodeOffset < 0) {
      break;
    }

    Path = fdt_getprop_by_offset(Dtb, NodeOffset, &Name, NULL);
    // Check if tha alias points to an ethernet node
    if (!AsciiStrnCmp(Name, "ethernet", AsciiStrLen("ethernet")) ) {
      // check the node in device tree
      NodeOffset = fdt_path_offset (Dtb, Path);
      if (NodeOffset < 0) {
        DEBUG ((DEBUG_ERROR, "Did not find path %a for alias %a\n", Path, Name));
      }
      // check if the ethernet is enabled or not?
      // if not enable, no need to add mac address
      PropCompatible = fdt_getprop (Dtb, NodeOffset, "status", &PropLen);
      if (PropCompatible == NULL) {
        DEBUG ((DEBUG_WARN, "status property not found\n"));
      } else if (PropLen != (AsciiStrLen ("okay") + 1)
               || !fdt_stringlist_contains ( (CHAR8 *)PropCompatible, PropLen, "okay"))
      {

        // It should say "okay", so only allow that. Some fdts use "ok" but
        // this is a bug. Please fix your device tree source file. See here
        // for discussion:
        //
        // http://www.mail-archive.com/u-boot@lists.denx.de/msg71598.html

        continue;
      }
      Status = MacReadFromEeprom (EthernetId, MacAddr);
      if (EFI_ERROR (Status)) {
        Status = GenerateMacAddress (EthernetId, MacAddr);
      }
      if (EFI_ERROR (Status)) {
        DEBUG ((DEBUG_ERROR, "Error getting mac address status %r\n", Status));
        return EFI_DEVICE_ERROR;
      }
      EthernetId++;

      if (fdt_getprop (Dtb, NodeOffset, "mac-address", NULL) != NULL) {
        FdtStatus = fdt_setprop (Dtb, NodeOffset, "mac-address", MacAddr, sizeof(MacAddr));
        if (FdtStatus) {
          DEBUG ((DEBUG_ERROR, "Not able to set mac-address %a\n", fdt_strerror (FdtStatus)));
          return EFI_DEVICE_ERROR;
        }
      }

      FdtStatus = fdt_setprop (Dtb, NodeOffset, "local-mac-address", MacAddr, sizeof(MacAddr));
      if (FdtStatus) {
        DEBUG ((DEBUG_ERROR, "Not able to set local-mac-address %a\n", fdt_strerror (FdtStatus)));
        return EFI_DEVICE_ERROR;
      }
    }
  }

  return EFI_SUCCESS;
}

EFI_STATUS
FdtFixupDpaa1 (
  IN  VOID  *Dtb
  )
{
  EFI_STATUS      Status;

  Status = FdtFixupFmanFirmware (Dtb);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  Status = FdtFixupQman (Dtb);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  Status = FdtFixupBmanVersion (Dtb);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  Status = FdtFixupMacAddresses (Dtb);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  return Status;
}

/**
  The entry point for Dpaa1EthernetDxe driver.

  @param[in] ImageHandle     The image handle of the driver.
  @param[in] SystemTable     The system table.

  @retval EFI_NOT_FOUND           No suitable DTB image could be located
  @retval EFI_OUT_OF_RESOURCES    Fail to execute entry point due to lack of
                                  resources.
  @retval EFI_SUCCES              All the related protocols are installed on
                                  the driver.

**/
EFI_STATUS
EFIAPI
Dpaa1EthernetDxeEntryPoint (
  IN EFI_HANDLE                   ImageHandle,
  IN EFI_SYSTEM_TABLE             *SystemTable
  )
{
  EFI_STATUS                      Status;
  VOID                            *Dtb;

  Status = EfiGetSystemConfigurationTable (&gFdtTableGuid, &Dtb);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "Did not find the Dtb Blob.\n"));
    return EFI_NOT_FOUND;
  }

  Status = FdtFixupDpaa1 (Dtb);

  return Status;
}
