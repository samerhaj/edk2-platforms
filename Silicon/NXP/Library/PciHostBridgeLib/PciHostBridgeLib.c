/** @file
  PCI Host Bridge Library instance for NXP SoCs

  Copyright 2018 NXP

  This program and the accompanying materials are licensed and made available
  under the terms and conditions of the BSD License which accompanies this
  distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php.

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS, WITHOUT
  WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include <PiDxe.h>
#include <IndustryStandard/Pci22.h>
#include <Library/BeIoLib.h>
#include <Library/DebugLib.h>
#include <Library/DevicePathLib.h>
#include <Library/IoLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/PcdLib.h>
#include <Library/PciHostBridgeLib.h>
#include <Pcie.h>
#include <Protocol/PciHostBridgeResourceAllocation.h>
#include <Protocol/PciRootBridgeIo.h>

#pragma pack(1)
typedef struct {
  ACPI_HID_DEVICE_PATH     AcpiDevicePath;
  EFI_DEVICE_PATH_PROTOCOL EndDevicePath;
} EFI_PCI_ROOT_BRIDGE_DEVICE_PATH;
#pragma pack ()

STATIC CONST EFI_PCI_ROOT_BRIDGE_DEVICE_PATH mEfiPciRootBridgeDevicePath[] = {
  {
    {
      {
        ACPI_DEVICE_PATH,
        ACPI_DP,
        {
          (UINT8)(sizeof (ACPI_HID_DEVICE_PATH)),
          (UINT8)(sizeof (ACPI_HID_DEVICE_PATH) >> 8)
        }
      },
      EISA_PNP_ID (0x0A08), // PCI Express
      PCI_SEG0_NUM
    },

    {
      END_DEVICE_PATH_TYPE,
      END_ENTIRE_DEVICE_PATH_SUBTYPE,
      {
        END_DEVICE_PATH_LENGTH,
        0
      }
    }
  },
  {
    {
      {
        ACPI_DEVICE_PATH,
        ACPI_DP,
        {
          (UINT8)(sizeof(ACPI_HID_DEVICE_PATH)),
          (UINT8)(sizeof(ACPI_HID_DEVICE_PATH) >> 8)
        }
      },
      EISA_PNP_ID (0x0A08), // PCI Express
      PCI_SEG1_NUM
    },

    {
      END_DEVICE_PATH_TYPE,
      END_ENTIRE_DEVICE_PATH_SUBTYPE,
      {
        END_DEVICE_PATH_LENGTH,
        0
      }
    }
  },
  {
    {
      {
        ACPI_DEVICE_PATH,
        ACPI_DP,
        {
          (UINT8)(sizeof(ACPI_HID_DEVICE_PATH)),
          (UINT8)(sizeof(ACPI_HID_DEVICE_PATH) >> 8)
        }
      },
      EISA_PNP_ID (0x0A08), // PCI Express
      PCI_SEG2_NUM
    },

    {
      END_DEVICE_PATH_TYPE,
      END_ENTIRE_DEVICE_PATH_SUBTYPE,
      {
        END_DEVICE_PATH_LENGTH,
        0
      }
    }
  }
};

GLOBAL_REMOVE_IF_UNREFERENCED
CHAR16 *mPciHostBridgeLibAcpiAddressSpaceTypeStr[] = {
  L"Mem", L"I/O", L"Bus"
};

#define PCI_ALLOCATION_ATTRIBUTES       EFI_PCI_HOST_BRIDGE_COMBINE_MEM_PMEM | \
                                        EFI_PCI_HOST_BRIDGE_MEM64_DECODE

#define PCI_SUPPORT_ATTRIBUTES          EFI_PCI_ATTRIBUTE_ISA_IO_16 | \
                                        EFI_PCI_ATTRIBUTE_ISA_MOTHERBOARD_IO | \
                                        EFI_PCI_ATTRIBUTE_VGA_MEMORY | \
                                        EFI_PCI_ATTRIBUTE_VGA_IO_16  | \
                                        EFI_PCI_ATTRIBUTE_VGA_PALETTE_IO_16

PCI_ROOT_BRIDGE mPciRootBridges[] = {
  {
    PCI_SEG0_NUM,                           // Segment
    PCI_SUPPORT_ATTRIBUTES,                 // Supports
    PCI_SUPPORT_ATTRIBUTES,                 // Attributes
    FALSE,                                  // DmaAbove4G
    FALSE,                                  // NoExtendedConfigSpace
    FALSE,                                  // ResourceAssigned
    PCI_ALLOCATION_ATTRIBUTES,              // AllocationAttributes
    { PCI_SEG0_BUSNUM_MIN,
      PCI_SEG0_BUSNUM_MAX },                // Bus
    { PCI_SEG0_PORTIO_MIN,
      PCI_SEG0_PORTIO_MAX },                // Io
    { PCI_SEG0_MMIO32_MIN,
      PCI_SEG0_MMIO32_MAX },                // Mem
    { PCI_SEG0_MMIO64_MIN,
      PCI_SEG0_MMIO64_MAX },                // MemAbove4G
    { MAX_UINT64, 0x0 },                    // PMem
    { MAX_UINT64, 0x0 },                    // PMemAbove4G
    (EFI_DEVICE_PATH_PROTOCOL *)&mEfiPciRootBridgeDevicePath[PCI_SEG0_NUM]
  }, {
    PCI_SEG1_NUM,                           // Segment
    PCI_SUPPORT_ATTRIBUTES,                 // Supports
    PCI_SUPPORT_ATTRIBUTES,                 // Attributes
    FALSE,                                  // DmaAbove4G
    FALSE,                                  // NoExtendedConfigSpace
    FALSE,                                  // ResourceAssigned
    PCI_ALLOCATION_ATTRIBUTES,              // AllocationAttributes
    { PCI_SEG1_BUSNUM_MIN,
      PCI_SEG1_BUSNUM_MAX },                // Bus
    { PCI_SEG1_PORTIO_MIN,
      PCI_SEG1_PORTIO_MAX },                // Io
    { PCI_SEG1_MMIO32_MIN,
      PCI_SEG1_MMIO32_MAX },                // Mem
    { PCI_SEG1_MMIO64_MIN,
      PCI_SEG1_MMIO64_MAX },                // MemAbove4G
    { MAX_UINT64, 0x0 },                    // PMem
    { MAX_UINT64, 0x0 },                    // PMemAbove4G
    (EFI_DEVICE_PATH_PROTOCOL *)&mEfiPciRootBridgeDevicePath[PCI_SEG1_NUM]
  }, {
    PCI_SEG2_NUM,                           // Segment
    PCI_SUPPORT_ATTRIBUTES,                 // Supports
    PCI_SUPPORT_ATTRIBUTES,                 // Attributes
    FALSE,                                  // DmaAbove4G
    FALSE,                                  // NoExtendedConfigSpace
    FALSE,                                  // ResourceAssigned
    PCI_ALLOCATION_ATTRIBUTES,              // AllocationAttributes
    { PCI_SEG2_BUSNUM_MIN,
      PCI_SEG2_BUSNUM_MAX },                // Bus
    { PCI_SEG2_PORTIO_MIN,
      PCI_SEG2_PORTIO_MAX },                // Io
    { PCI_SEG2_MMIO32_MIN,
      PCI_SEG2_MMIO32_MAX },                // Mem
    { PCI_SEG2_MMIO64_MIN,
      PCI_SEG2_MMIO64_MAX },                // MemAbove4G
    { MAX_UINT64, 0x0 },                    // PMem
    { MAX_UINT64, 0x0 },                    // PMemAbove4G
    (EFI_DEVICE_PATH_PROTOCOL *)&mEfiPciRootBridgeDevicePath[PCI_SEG2_NUM]
  }
};

/**
  Function to set-up iATU outbound window for PCIe controller

  @param Dbi     Address of PCIe host controller.
  @param Idx     Index of iATU outbound window.
  @param Type    Type(Cfg0/Cfg1/Mem/IO) of iATU outbound window.
  @param Phys    PCIe controller phy address for outbound window.
  @param BusAdr  PCIe controller bus address for outbound window.
  @param Pcie    Size of PCIe controller space(Cfg0/Cfg1/Mem/IO).

**/
STATIC
VOID
PcieIatuOutboundSet (
  IN EFI_PHYSICAL_ADDRESS Dbi,
  IN UINT32 Idx,
  IN UINT32 Type,
  IN UINT64 Phys,
  IN UINT64 BusAddr,
  IN UINT64 Size
  )
{
  MmioWrite32 (Dbi + IATU_VIEWPORT_OFF,
              (UINT32)(IATU_VIEWPORT_OUTBOUND | Idx));
  MmioWrite32 (Dbi + IATU_LWR_BASE_ADDR_OFF_OUTBOUND_0,
              (UINT32)Phys);
  MmioWrite32 (Dbi + IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_0,
              (UINT32)(Phys >> 32));
  MmioWrite32 (Dbi + IATU_LIMIT_ADDR_OFF_OUTBOUND_0,
              (UINT32)(Phys + Size - BIT0));
  MmioWrite32 (Dbi + IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_0,
              (UINT32)BusAddr);
  MmioWrite32 (Dbi + IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_0,
              (UINT32)(BusAddr >> 32));
  MmioWrite32 (Dbi + IATU_REGION_CTRL_1_OFF_OUTBOUND_0,
              (UINT32)Type);
  MmioWrite32 (Dbi + IATU_REGION_CTRL_2_OFF_OUTBOUND_0,
              IATU_REGION_CTRL_2_OFF_OUTBOUND_0_REGION_EN);
}

/**
   Function to check PCIe controller LTSSM state

   @param Pcie Address of PCIe host controller.

**/
STATIC
INTN
PcieLinkState (
  IN EFI_PHYSICAL_ADDRESS Pcie
  )
{
  UINT32 State;

  //
  // Reading PCIe controller LTSSM state
  //
  if (FeaturePcdGet (PcdPciLutBigEndian)) {
    State = BeMmioRead32 ((UINTN)Pcie + PCI_LUT_BASE + PCI_LUT_DBG) &
            LTSSM_STATE_MASK;
  } else {
   State = MmioRead32 ((UINTN)Pcie + PCI_LUT_BASE + PCI_LUT_DBG) &
           LTSSM_STATE_MASK;
  }

  if (State < LTSSM_PCIE_L0) {
    DEBUG ((DEBUG_INFO," Pcie Link error. LTSSM=0x%2x\n", State));
    return EFI_SUCCESS;
  }

  return EFI_UNSUPPORTED;
}

/**
   Helper function to check PCIe link state

   @param Pcie Address of PCIe host controller.

**/
STATIC
INTN
PcieLinkUp (
  IN EFI_PHYSICAL_ADDRESS Pcie
  )
{
  INTN State;
  UINT32 Cap;

  State = PcieLinkState (Pcie);
  if (State) {
    return State;
  }

  //
  // Try to download speed to gen1
  //
  Cap = MmioRead32 ((UINTN)Pcie + PCI_LINK_CAP);
  MmioWrite32 ((UINTN)Pcie + PCI_LINK_CAP, (UINT32)(Cap & (~PCI_LINK_SPEED_MASK)) | BIT0);
  State = PcieLinkState (Pcie);
  if (State) {
    return State;
  }

  MmioWrite32 ((UINTN)Pcie + PCI_LINK_CAP, Cap);

  return EFI_SUCCESS;
}

/**
   This function checks whether PCIe is enabled or not
   depending upon SoC serdes protocol map

   @param  PcieNum PCIe number.

   @return The     PCIe number enabled in map.
   @return FALSE   PCIe number is disabled in map.

**/
STATIC
BOOLEAN
IsPcieNumEnabled(
  IN UINTN PcieNum
  )
{
  UINT64 SerDes1ProtocolMap;

  SerDes1ProtocolMap = 0x0;

  //
  // Reading serdes map
  //
  GetSerdesProtocolMaps (&SerDes1ProtocolMap);

  //
  // Verify serdes line is configured in the map
  //
  if (PcieNum < NUM_PCIE_CONTROLLER) {
    return IsSerDesLaneProtocolConfigured (SerDes1ProtocolMap, (PcieNum + BIT0));
  } else {
    DEBUG ((DEBUG_ERROR, "Device not supported\n"));
  }

  return FALSE;
}

/**
  Function to set-up iATU outbound window for PCIe controller

  @param Pcie     Address of PCIe host controller
  @param Cfg0Base PCIe controller phy address Type0 Configuration Space.
  @param Cfg1Base PCIe controller phy address Type1 Configuration Space.
  @param MemBase  PCIe controller phy address Memory Space.
  @param IoBase   PCIe controller phy address IO Space.
**/
STATIC
VOID
PcieSetupAtu (
  IN EFI_PHYSICAL_ADDRESS Pcie,
  IN EFI_PHYSICAL_ADDRESS Cfg0Base,
  IN EFI_PHYSICAL_ADDRESS Cfg1Base,
  IN EFI_PHYSICAL_ADDRESS MemBase,
  IN EFI_PHYSICAL_ADDRESS IoBase
  )
{

  //
  // iATU : OUTBOUND WINDOW 0 : CFG0
  //
  PcieIatuOutboundSet (Pcie, IATU_REGION_INDEX0,
                            IATU_REGION_CTRL_1_OFF_OUTBOUND_0_TYPE_CFG0,
                            Cfg0Base,
                            SEG_CFG_BUS,
                            SEG_CFG_SIZE);

  //
  // iATU : OUTBOUND WINDOW 1 : CFG1
  PcieIatuOutboundSet (Pcie, IATU_REGION_INDEX1,
                            IATU_REGION_CTRL_1_OFF_OUTBOUND_0_TYPE_CFG1,
                            Cfg1Base,
                            SEG_CFG_BUS,
                            SEG_CFG_SIZE);
  //
  // iATU 2 : OUTBOUND WINDOW 2 : MEM
  //
  PcieIatuOutboundSet (Pcie, IATU_REGION_INDEX2,
                            IATU_REGION_CTRL_1_OFF_OUTBOUND_0_TYPE_MEM,
                            MemBase,
                            SEG_MEM_BUS,
                            SEG_MEM_SIZE);

  //
  // iATU 3 : OUTBOUND WINDOW 3: IO
  //
  PcieIatuOutboundSet (Pcie, IATU_REGION_INDEX3,
                            IATU_REGION_CTRL_1_OFF_OUTBOUND_0_TYPE_IO,
                            IoBase,
                            SEG_IO_BUS,
                            SEG_IO_SIZE);

}

/**
  Helper function to set-up PCIe controller

  @param Pcie     Address of PCIe host controller
  @param Cfg0Base PCIe controller phy address Type0 Configuration Space.
  @param Cfg1Base PCIe controller phy address Type1 Configuration Space.
  @param MemBase  PCIe controller phy address Memory Space.
  @param IoBase   PCIe controller phy address IO Space.

**/
STATIC
VOID
PcieSetupCntrl (
  IN EFI_PHYSICAL_ADDRESS Pcie,
  IN EFI_PHYSICAL_ADDRESS Cfg0Base,
  IN EFI_PHYSICAL_ADDRESS Cfg1Base,
  IN EFI_PHYSICAL_ADDRESS MemBase,
  IN EFI_PHYSICAL_ADDRESS IoBase
  )
{
  //
  // iATU outbound set-up
  //
  PcieSetupAtu (Pcie, Cfg0Base, Cfg1Base, MemBase, IoBase);

  //
  // program correct class for RC
  //
  MmioWrite32 ((UINTN)Pcie + PCI_BASE_ADDRESS_0, (BIT0 - BIT0));
  MmioWrite32 ((UINTN)Pcie + PCI_DBI_RO_WR_EN, (UINT32)BIT0);
  MmioWrite32 ((UINTN)Pcie + PCI_CLASS_DEVICE, (UINT32)PCI_CLASS_BRIDGE_PCI);
  MmioWrite32 ((UINTN)Pcie + PCI_DBI_RO_WR_EN, (UINT32)(BIT0 - BIT0));
}

/**
  Return all the root bridge instances in an array.

  @param Count  Return the count of root bridge instances.

  @return All the root bridge instances in an array.

**/
PCI_ROOT_BRIDGE *
EFIAPI
PciHostBridgeGetRootBridges (
  OUT UINTN     *Count
  )
{
  UINTN  Idx;
  INTN   LinkUp;
  UINT64 PciPhyMemAddr[NUM_PCIE_CONTROLLER];
  UINT64 PciPhyCfg0Addr[NUM_PCIE_CONTROLLER];
  UINT64 PciPhyCfg1Addr[NUM_PCIE_CONTROLLER];
  UINT64 PciPhyIoAddr[NUM_PCIE_CONTROLLER];
  UINT64 Regs[NUM_PCIE_CONTROLLER];

  *Count = 0;

  //
  // Filling local array for
  // PCIe controller Physical address space for Cfg0,Cfg1,Mem,IO
  // Host Contoller address
  //
  for  (Idx = 0; Idx < NUM_PCIE_CONTROLLER; Idx++) {
    PciPhyMemAddr[Idx] = PCI_SEG0_PHY_MEM_BASE + (PCI_BASE_DIFF * Idx);
    PciPhyCfg0Addr[Idx] = PCI_SEG0_PHY_CFG0_BASE + (PCI_BASE_DIFF * Idx);
    PciPhyCfg1Addr[Idx] = PCI_SEG0_PHY_CFG1_BASE + (PCI_BASE_DIFF * Idx);
    PciPhyIoAddr [Idx] =  PCI_SEG0_PHY_IO_BASE + (PCI_BASE_DIFF * Idx);
    Regs[Idx] =  PCI_SEG0_DBI_BASE + (PCI_DBI_SIZE_DIFF * Idx);
  }

  for (Idx = 0; Idx < NUM_PCIE_CONTROLLER; Idx++) {
    //
    // Verify PCIe controller is enabled in Soc Serdes Map
    //
    if (!IsPcieNumEnabled (Idx)) {
      DEBUG ((DEBUG_ERROR, "PCIE%d is disabled\n", (Idx + BIT0)));
      //
      // Continue with other PCIe controller
      //
      continue;
    }
    DEBUG ((DEBUG_INFO, "PCIE%d is Enabled\n", Idx + BIT0));

    //
    // Verify PCIe controller LTSSM state
    //
    LinkUp = PcieLinkUp(Regs[Idx]);
    if (!LinkUp) {
      //
      // Let the user know there's no PCIe link
      //
      DEBUG ((DEBUG_INFO,"no link, regs @ 0x%lx\n", Regs[Idx]));
      //
      // Continue with other PCIe controller
      //
      continue;
    }
    DEBUG ((DEBUG_INFO, "PCIE%d Passed Linkup Phase\n", Idx + BIT0));

    //
    // Function to set up address translation unit outbound window for
    // PCIe Controller
    //
    PcieSetupCntrl (Regs[Idx],
                    PciPhyCfg0Addr[Idx],
                    PciPhyCfg1Addr[Idx],
                    PciPhyMemAddr[Idx],
                    PciPhyIoAddr[Idx]);
    *Count += BIT0;
    break;
  }

  return &mPciRootBridges[Idx];
}

/**
  Free the root bridge instances array returned from PciHostBridgeGetRootBridges().

  @param Bridges The root bridge instances array.
  @param Count   The count of the array.
**/
VOID
EFIAPI
PciHostBridgeFreeRootBridges (
  PCI_ROOT_BRIDGE *Bridges,
  UINTN           Count
  )
{
}

/**
  Inform the platform that the resource conflict happens.

  @param HostBridgeHandle Handle of the Host Bridge.
  @param Configuration    Pointer to PCI I/O and PCI memory resource
                          descriptors. The Configuration contains the resources
                          for all the root bridges. The resource for each root
                          bridge is terminated with END descriptor and an
                          additional END is appended indicating the end of the
                          entire resources. The resource descriptor field
                          values follow the description in
                          EFI_PCI_HOST_BRIDGE_RESOURCE_ALLOCATION_PROTOCOL
                          .SubmitResources().

**/
VOID
EFIAPI
PciHostBridgeResourceConflict (
  EFI_HANDLE                        HostBridgeHandle,
  VOID                              *Configuration
  )
{
  EFI_ACPI_ADDRESS_SPACE_DESCRIPTOR *Descriptor;
  UINTN                             RootBridgeIndex;
  DEBUG ((DEBUG_ERROR, "PciHostBridge: Resource conflict happens!\n"));

  RootBridgeIndex = 0;
  Descriptor = (EFI_ACPI_ADDRESS_SPACE_DESCRIPTOR *) Configuration;
  while (Descriptor->Desc == ACPI_ADDRESS_SPACE_DESCRIPTOR) {
    DEBUG ((DEBUG_ERROR, "RootBridge[%d]:\n", RootBridgeIndex++));
    for (; Descriptor->Desc == ACPI_ADDRESS_SPACE_DESCRIPTOR; Descriptor++) {
      ASSERT (Descriptor->ResType <
              ARRAY_SIZE (mPciHostBridgeLibAcpiAddressSpaceTypeStr));
      DEBUG ((DEBUG_ERROR, " %s: Length/Alignment = 0x%lx / 0x%lx\n",
              mPciHostBridgeLibAcpiAddressSpaceTypeStr[Descriptor->ResType],
              Descriptor->AddrLen, Descriptor->AddrRangeMax
              ));
      if (Descriptor->ResType == ACPI_ADDRESS_SPACE_TYPE_MEM) {
        DEBUG ((DEBUG_ERROR, "     Granularity/SpecificFlag = %ld / %02x%s\n",
                Descriptor->AddrSpaceGranularity, Descriptor->SpecificFlag,
                ((Descriptor->SpecificFlag &
                  EFI_ACPI_MEMORY_RESOURCE_SPECIFIC_FLAG_CACHEABLE_PREFETCHABLE
                  ) != 0) ? L" (Prefetchable)" : L""
                ));
      }
    }
    //
    // Skip the END descriptor for root bridge
    //
    ASSERT (Descriptor->Desc == ACPI_END_TAG_DESCRIPTOR);
    Descriptor = (EFI_ACPI_ADDRESS_SPACE_DESCRIPTOR *)(
                   (EFI_ACPI_END_TAG_DESCRIPTOR *)Descriptor + 1
                   );
  }

  return;
}
