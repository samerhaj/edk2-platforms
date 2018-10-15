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
#include <libfdt.h>
#include <IndustryStandard/Pci22.h>
#include <Library/BeIoLib.h>
#include <Library/DebugLib.h>
#include <Library/DevicePathLib.h>
#include <Library/IoLib.h>
#include <Library/ItbParse.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/PcdLib.h>
#include <Library/PciHostBridgeLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiLib.h>
#include <Pcie.h>
#include <Protocol/PciHostBridgeResourceAllocation.h>
#include <Protocol/PciRootBridgeIo.h>
#include <Protocol/PciIo.h>

#include "PciHostBridgeLib.h"

#pragma pack(1)
typedef struct {
  ACPI_HID_DEVICE_PATH     AcpiDevicePath;
  EFI_DEVICE_PATH_PROTOCOL EndDevicePath;
} EFI_PCI_ROOT_BRIDGE_DEVICE_PATH;
#pragma pack ()

//
// Protocol notify related globals
//
VOID          *PlatformHasPciIoNotifyReg;
EFI_EVENT     PlatformHasPciIoEvent;

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
          (UINT8)(sizeof (ACPI_HID_DEVICE_PATH)),
          (UINT8)(sizeof (ACPI_HID_DEVICE_PATH) >> 8)
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
          (UINT8)(sizeof (ACPI_HID_DEVICE_PATH)),
          (UINT8)(sizeof (ACPI_HID_DEVICE_PATH) >> 8)
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
  },
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
      PCI_SEG3_NUM
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
          (UINT8)(sizeof (ACPI_HID_DEVICE_PATH)),
          (UINT8)(sizeof (ACPI_HID_DEVICE_PATH) >> 8)
        }
      },
      EISA_PNP_ID (0x0A08), // PCI Express
      PCI_SEG4_NUM
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
          (UINT8)(sizeof (ACPI_HID_DEVICE_PATH)),
          (UINT8)(sizeof (ACPI_HID_DEVICE_PATH) >> 8)
        }
      },
      EISA_PNP_ID (0x0A08), // PCI Express
      PCI_SEG5_NUM
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

STATIC
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

PCI_ROOT_BRIDGE mPciRootBridges[NUM_PCIE_CONTROLLER];

/**
  Function to select page among the 48 1KB pages for
  AXI access

  @param  Dbi    GPEX host controller address.
  @param  PgIdx  The page index to select

**/
VOID
CcsrSetPg (
  IN EFI_PHYSICAL_ADDRESS Dbi,
  IN UINT8 PgIdx
  )
{
  UINT32 Val;
  Val = MmioRead32 (Dbi + PAB_CTRL);
  // Bit 18:13 of Bridge Control Register(PAB) denotes page select
  // Mask is 6 bits and shift is 13 to select page
  Val &= ~(PAB_CTRL_PAGE_SEL_MASK << PAB_CTRL_PAGE_SEL_SHIFT);
  Val |= (PgIdx & PAB_CTRL_PAGE_SEL_MASK) << PAB_CTRL_PAGE_SEL_SHIFT;

  MmioWrite32 (Dbi + PAB_CTRL, Val);
}

/**
  Function to read PCIe config address

  @param  Dbi     GPEX host controller address.
  @param  Offset  Offset to read from

**/
INTN
CcsrRead32 (
 IN EFI_PHYSICAL_ADDRESS Dbi,
 IN UINT32 Offset
  )
{

  // If Offset < 3KB direct addressing
  // is used
  if (Offset < INDIRECT_ADDR_BNDRY) {
    CcsrSetPg (Dbi, 0);
    return MmioRead32 (Dbi + Offset);
  } else {
    // If Offset > 3KB than paging mechanism is used
    // with indirect address mechanism
    // Select page index and offset within the page
    CcsrSetPg (Dbi, OFFSET_TO_PAGE_IDX (Offset));
    return MmioRead32 (Dbi + OFFSET_TO_PAGE_ADDR (Offset));
  }
}

/**
  Function to write PCIe Controller config address

  @param  Dbi     GPEX host controller address
  @param  Offset  Offset to read from

**/
VOID
CcsrWrite32 (
 IN EFI_PHYSICAL_ADDRESS Dbi,
 IN UINT32 Offset,
 IN UINT32 Value
  )
{

  if (Offset < INDIRECT_ADDR_BNDRY) {
    CcsrSetPg (Dbi, 0);
    MmioWrite32 (Dbi + Offset, Value);
  } else {
    CcsrSetPg (Dbi, OFFSET_TO_PAGE_IDX (Offset));
    MmioWrite32 (Dbi + OFFSET_TO_PAGE_ADDR (Offset), Value);
  }
}

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
PcieOutboundSet (
  IN EFI_PHYSICAL_ADDRESS Dbi,
  IN UINT32 Idx,
  IN UINT32 Type,
  IN UINT64 Phys,
  IN UINT64 BusAddr,
  IN UINT64 Size
  )
{
  if (PcdGetBool (PcdPcieConfigurePex)) {
    UINT32 Val = 0;

    // Program AXI window base with appropriate PCIe physical address space
    // values in Bridge Address mapping window Register
    CcsrWrite32 ((UINTN)Dbi, PAB_AXI_AMAP_AXI_WIN (Idx), (UINT32)Phys);
    CcsrWrite32 ((UINTN)Dbi, PAB_EXT_AXI_AMAP_AXI_WIN (Idx), Phys >> 32);
    // Program PEX Address base with appropriate Bus Address values in
    // Bridge address mapping window
    CcsrWrite32 ((UINTN)Dbi, PAB_AXI_AMAP_PEX_WIN_L (Idx), (UINT32)BusAddr);
    CcsrWrite32 ((UINTN)Dbi, PAB_AXI_AMAP_PEX_WIN_H (Idx), BusAddr >> 32);
    // Program the size of window
    CcsrWrite32 ((UINTN)Dbi, PAB_EXT_AXI_AMAP_SIZE (Idx), (UINT64)Size >> 32);

    // Mapping AXI transactions to PEX address
    Val = CcsrRead32 ((UINTN)Dbi, PAB_AXI_AMAP_CTRL (Idx));
    Val &= ~((AXI_AMAP_CTRL_TYPE_MASK << AXI_AMAP_CTRL_TYPE_SHIFT) |
             (AXI_AMAP_CTRL_SIZE_MASK << AXI_AMAP_CTRL_SIZE_SHIFT) |
              AXI_AMAP_CTRL_EN);
    // Type indicates the type of AXI transaction to the window address is mapped
    // to CFG/IO/MEM
    Val |= ((Type & AXI_AMAP_CTRL_TYPE_MASK) << AXI_AMAP_CTRL_TYPE_SHIFT) |
           (((UINT32)Size >> AXI_AMAP_CTRL_SIZE_SHIFT) <<
             AXI_AMAP_CTRL_SIZE_SHIFT) | AXI_AMAP_CTRL_EN;
    // Program Address mapping enable, other fields with desired values in Bridge
    // control Register
    // Contols the mapping of address for AXI transactions to PEX address
    CcsrWrite32 ((UINTN)Dbi, PAB_AXI_AMAP_CTRL (Idx), Val);
  } else {
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

STATIC
VOID
PcieSetupWindow (
  IN EFI_PHYSICAL_ADDRESS Pcie,
  IN EFI_PHYSICAL_ADDRESS Cfg0Base,
  IN EFI_PHYSICAL_ADDRESS Cfg1Base,
  IN EFI_PHYSICAL_ADDRESS MemBase,
  IN EFI_PHYSICAL_ADDRESS IoBase
  )
{
  IoBase = Cfg0Base + LX_PEX_CFG_SIZE;

  // ATU 0 : OUTBOUND : CFG0
  PcieOutboundSet (Pcie, IATU_REGION_INDEX0,
                         PAB_AXI_TYPE_CFG,
                         Cfg0Base,
                         SEG_CFG_BUS,
                         LX_PEX_CFG_SIZE);
  // ATU 2 : OUTBOUND : IO
  PcieOutboundSet (Pcie, IATU_REGION_INDEX1,
                         PAB_AXI_TYPE_IO,
                         IoBase,
                         SEG_CFG_BUS,
                         SEG_CFG_SIZE);

  // ATU 3 : OUTBOUND : MEM
  PcieOutboundSet (Pcie, IATU_REGION_INDEX2,
                         PAB_AXI_TYPE_MEM,
                         MemBase,
                         SEG_MEM_BUS,
                         SEG_MEM_SIZE);

  if (FeaturePcdGet (PcdPciDebug) == TRUE) {
    INTN Cnt;
    for (Cnt = 0; Cnt <= IATU_REGION_INDEX2; Cnt++) {
      DEBUG ((DEBUG_INFO,"AMAP WINDOW%d:\n", Cnt));
      DEBUG ((DEBUG_INFO,"\tLOWER PHYS 0x%08x\n",
              CcsrRead32 ((UINTN)Pcie, PAB_AXI_AMAP_AXI_WIN (Cnt))));
      DEBUG ((DEBUG_INFO,"\tUPPER PHYS 0x%08x\n",
              CcsrRead32 ((UINTN)Pcie, PAB_EXT_AXI_AMAP_AXI_WIN (Cnt))));
      DEBUG ((DEBUG_INFO,"\tLOWER BUS  0x%08x\n",
              CcsrRead32 ((UINTN)Pcie, PAB_AXI_AMAP_PEX_WIN_L (Cnt))));
      DEBUG ((DEBUG_INFO,"\tUPPER BUS  0x%08x\n",
              CcsrRead32 ((UINTN)Pcie, PAB_AXI_AMAP_PEX_WIN_H (Cnt))));
      DEBUG ((DEBUG_INFO,"\tSIZE      0x%08x\n",
              CcsrRead32 ((UINTN)Pcie, PAB_AXI_AMAP_CTRL (Cnt)) & (AXI_AMAP_CTRL_SIZE_MASK << AXI_AMAP_CTRL_SIZE_SHIFT)));
      DEBUG ((DEBUG_INFO,"\tEXT_SIZE        0x%08x\n",
              CcsrRead32 ((UINTN)Pcie, PAB_EXT_AXI_AMAP_SIZE (Cnt))));
      DEBUG ((DEBUG_INFO,"\tCTRL:        0x%08x\n",
              CcsrRead32 ((UINTN)Pcie, PAB_AXI_AMAP_CTRL (Cnt))));
    }
  }
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
  PcieOutboundSet (Pcie, IATU_REGION_INDEX0,
                            IATU_REGION_CTRL_1_OFF_OUTBOUND_0_TYPE_CFG0,
                            Cfg0Base,
                            SEG_CFG_BUS,
                            SEG_CFG_SIZE);

  //
  // iATU : OUTBOUND WINDOW 1 : CFG1
  PcieOutboundSet (Pcie, IATU_REGION_INDEX1,
                            IATU_REGION_CTRL_1_OFF_OUTBOUND_0_TYPE_CFG1,
                            Cfg1Base,
                            SEG_CFG_BUS,
                            SEG_CFG_SIZE);
  //
  // iATU 2 : OUTBOUND WINDOW 2 : MEM
  //
  PcieOutboundSet (Pcie, IATU_REGION_INDEX2,
                            IATU_REGION_CTRL_1_OFF_OUTBOUND_0_TYPE_MEM,
                            MemBase,
                            SEG_MEM_BUS,
                            SEG_MEM_SIZE);

  //
  // iATU 3 : OUTBOUND WINDOW 3: IO
  //
  PcieOutboundSet (Pcie, IATU_REGION_INDEX3,
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
  if (PcdGetBool (PcdPcieConfigurePex)) {
    UINT32 Val;

    // Enable AMBA & PEX PIO
    // PEX PIO is used to generate PIO traffic from PCIe Link to AXI
    Val = CcsrRead32 ((UINTN)Pcie, PAB_CTRL);
    // Enable AXI-PIO/PEX PIO
    Val |= PAB_CTRL_APIO_EN | PAB_CTRL_PPIO_EN;
    // Programm APIO/PPIO in Bridge control register
    CcsrWrite32 ((UINTN)Pcie, PAB_CTRL, Val);

    // Enable APIO and Memory/IO/CFG Wins
    Val = CcsrRead32 ((UINTN)Pcie, PAB_AXI_PIO_CTRL (0));
    // AXI PIO programming
    // AXI PIO is used to generate PIO traffic from AXI to PCIe Link
    // Enable AXI-PIO/MEM/IO/CFG
    Val |= APIO_EN | MEM_WIN_EN | IO_WIN_EN | CFG_WIN_EN;
    // Programm MEM/IO/CFG window in bridge control register
    CcsrWrite32 ((UINTN)Pcie, PAB_AXI_PIO_CTRL (0), Val);

    if (FeaturePcdGet (PcdPciDebug) == TRUE) {
      DEBUG ((DEBUG_INFO, "Going to SetUp PCIe Space Windows\n\n"));
    }

    PcieSetupWindow (Pcie, Cfg0Base, Cfg1Base, MemBase, IoBase);
  } else {
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
}

/**
  Find the Pcie Node Offset in device tree, whose address matches with the given address

  @param[in] Dtb         Device tree to fixup
  @param[in] Address     addresses of registers of pcie controller which is to be matched

  @retval  Pcie Node Offset in device tree. if no node found then -FDT_ERR_NOTFOUND
**/
INT32
FdtFindPcie (
  IN  VOID    *Dtb,
  IN  UINTN   Address
  )
{
  INTN    NodeOffset;
  UINT64  PcieAddress;
  UINT8   RegIndex;
  EFI_STATUS Status;

  /* find pci controller node */
  for (NodeOffset = fdt_node_offset_by_compatible (Dtb, -1, (VOID *)(PcdGetPtr (PcdPciFdtCompatible)));
       NodeOffset != -FDT_ERR_NOTFOUND;
       NodeOffset = fdt_node_offset_by_compatible (Dtb, NodeOffset, (VOID *)(PcdGetPtr (PcdPciFdtCompatible)))) {
    // Get the Index of Controllers' registers
    RegIndex = fdt_stringlist_search (Dtb, NodeOffset, "reg-names", "regs");
    if (RegIndex < 0) {
      RegIndex = 0;
    }
    // Get the controller's registers' address from node.
    Status = FdtGetAddressSize (Dtb, NodeOffset, "reg", RegIndex, &PcieAddress, NULL);
    if (EFI_ERROR (Status)) {
      DEBUG ((DEBUG_ERROR, "Error: can't get regs base address(Status = %r)!\n", Status));
      continue;
    }
    // Search the address in received array from PciHostBridgeLib
    if (Address == PcieAddress) {
      break;
    }
  }

  return NodeOffset;
}

/**
  Program a single LUT entry
 **/
STATIC
VOID
PcieLutSetMapping (
  IN   LS_PCIE   *LsPcie,
  IN   INT32     Index,
  IN   UINT32    BusDevFuc,
  IN   UINT32    StreamId
  )
{
  /* leave mask as all zeroes, want to match all bits */
  if (FeaturePcdGet (PcdPciLutBigEndian)) {
    BeMmioWrite32 (LsPcie->LsPcieLut->PexLut[Index].PexLudr, BusDevFuc << 16);
    BeMmioWrite32 (LsPcie->LsPcieLut->PexLut[Index].PexLldr, StreamId | PCIE_LUT_ENABLE);
  } else {
    MmioWrite32 (LsPcie->LsPcieLut->PexLut[Index].PexLudr, BusDevFuc << 16);
    MmioWrite32 (LsPcie->LsPcieLut->PexLut[Index].PexLldr, StreamId | PCIE_LUT_ENABLE);
  }
}

/**
  Return next available LUT index.
 **/
STATIC
INT32
PcieNextLutIndex (
 IN  LS_PCIE   *LsPcie
 )
{
  if (LsPcie->NextLutIndex < ARRAY_SIZE (LsPcie->LsPcieLut->PexLut)) {
    return LsPcie->NextLutIndex++;
  } else {
    return -1;;  /* LUT is full */
  }
}

/**
  returns the next available streamid for pcie, OUT_OF_RESOURCES if failed
 **/
STATIC
INT32
PcieNextStreamId ()
{
  STATIC INT32 NextStreamid = FixedPcdGet32 (PcdPcieStreamIdStart);

  if (NextStreamid > FixedPcdGet32 (PcdPcieStreamIdEnd)) {
    return -1;
  }

  return NextStreamid++;
}

/**
  An msi-map is a property to be added to the pci controller
  node.  It is a table, where each entry consists of 4 fields
  e.g.:

       msi-map = <[devid] [phandle-to-msi-ctrl] [stream-id] [count]
                  [devid] [phandle-to-msi-ctrl] [stream-id] [count]>;
 **/
EFI_STATUS
FdtPcieSetMsiMapEntry (
  VOID      *Dtb,
  INT32     PcieNodeOffset,
  UINT32    BusDevFuc,
  UINT32    StreamId
  )
{
  INT32          FdtStatus;
  CONST fdt32_t  *Property;
  UINT32         PHandle;

  /* get phandle to MSI controller */
  Property = fdt_getprop (Dtb, PcieNodeOffset, "msi-parent", 0);
  if (Property == NULL) {
    DEBUG ((DEBUG_WARN, "missing msi-parent for %a\n", fdt_get_name (Dtb, PcieNodeOffset, NULL)));
    return EFI_NOT_FOUND;
  }
  PHandle = fdt32_to_cpu (*Property);

  /* set one msi-map row */
  FdtStatus = fdt_appendprop_u32 (Dtb, PcieNodeOffset, "msi-map", BusDevFuc);
  if (FdtStatus) {
    goto Error;
  }
  FdtStatus = fdt_appendprop_u32 (Dtb, PcieNodeOffset, "msi-map", PHandle);
  if (FdtStatus) {
    goto Error;
  }
  FdtStatus = fdt_appendprop_u32 (Dtb, PcieNodeOffset, "msi-map", StreamId);
  if (FdtStatus) {
    goto Error;
  }
  FdtStatus = fdt_appendprop_u32 (Dtb, PcieNodeOffset, "msi-map", 1);
  if (FdtStatus) {
    goto Error;
  }

  return EFI_SUCCESS;

Error:
    DEBUG ((
      DEBUG_ERROR, "error %a setting msi-map for %a\n",
      fdt_strerror (FdtStatus), fdt_get_name (Dtb, PcieNodeOffset, NULL)
      ));
    return EFI_DEVICE_ERROR;
}

/**
  An iommu-map is a property to be added to the pci controller
  node.  It is a table, where each entry consists of 4 fields
  e.g.:

       iommu-map = <[devid] [phandle-to-iommu-ctrl] [stream-id] [count]
                  [devid] [phandle-to-iommu-ctrl] [stream-id] [count]>;
 **/
EFI_STATUS
FdtPcieSetIommuMapEntry (
  VOID    *Dtb,
  INT32   PcieNodeOffset,
  UINT32  BusDevFuc,
  UINT32  StreamId
  )
{
  INT32          FdtStatus;
  CONST fdt32_t  *Property;
  UINT32         IommuMap[4];

  /* get phandle to iommu controller */
  Property = fdt_getprop (Dtb, PcieNodeOffset, "iommu-map", NULL);
  if (Property == NULL) {
    DEBUG ((DEBUG_WARN, "missing iommu-map for %a\n", fdt_get_name (Dtb, PcieNodeOffset, NULL)));
    return EFI_NOT_FOUND;
  }

  /* set iommu-map row */
  IommuMap[0] = cpu_to_fdt32 (BusDevFuc);
  IommuMap[1] = *++Property;
  IommuMap[2] = cpu_to_fdt32 (StreamId);
  IommuMap[3] = cpu_to_fdt32 (1);

  if (BusDevFuc == 0) {
    FdtStatus = fdt_setprop_inplace (Dtb, PcieNodeOffset, "iommu-map", IommuMap, 16);
  } else {
    FdtStatus = fdt_appendprop (Dtb, PcieNodeOffset, "iommu-map", IommuMap, 16);
  }

  if (FdtStatus) {
    DEBUG ((
      DEBUG_ERROR, "error %a setting iommu-map for %a\n",
      fdt_strerror (FdtStatus), fdt_get_name (Dtb, PcieNodeOffset, NULL)
      ));
    return EFI_DEVICE_ERROR;
  }

  return EFI_SUCCESS;
}

/**
  This notification function is invoked when an instance of the
  EFI_PCI_IO_PROTOCOL is produced.  It searches the devices on the IO
  Protocol and fixes the device tree with msi-map and iommu-map property
  corresponding to that device.

  @param  Event                 The event that occured
  @param  Context               The address of PCIE Registers

**/
VOID
EFIAPI
OnPlatformHasPciIo (
  IN  EFI_EVENT Event,
  IN  VOID      *Context
  )
{
  EFI_HANDLE            Handle;
  EFI_STATUS            Status;
  UINTN                 BufferSize;
  EFI_PCI_IO_PROTOCOL   *PciIo;
  UINTN                 SegmentNumber;
  UINTN                 BusNumber;
  UINTN                 DeviceNumber;
  UINTN                 FunctionNumber;
  UINTN                 BusDevFuc;
  INTN                  PcieNodeOffset;
  VOID                  *Dtb;
  UINT32                StreamId;
  UINT32                LutIndex;
  LS_PCIE               *LsPcie;

  LsPcie = (LS_PCIE *)Context;

  Status = EfiGetSystemConfigurationTable (&gFdtTableGuid, &Dtb);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "Did not find the Dtb Blob.\n"));
    return;
  }

  //
  // Examine all new handles
  //
  for (;;) {
    //
    // Get the next handle
    //
    BufferSize = sizeof (Handle);
    Status = gBS->LocateHandle (
                    ByRegisterNotify,
                    NULL,
                    PlatformHasPciIoNotifyReg,
                    &BufferSize,
                    &Handle
                    );

    //
    // If not found, we're done
    //
    if (EFI_NOT_FOUND == Status) {
      break;
    }

    if (EFI_ERROR (Status)) {
      continue;
    }

    //
    // Get the PciIo protocol on that handle
    //
    Status = gBS->HandleProtocol (Handle, &gEfiPciIoProtocolGuid, (VOID **)&PciIo);
    ASSERT_EFI_ERROR (Status);
    ASSERT (PciIo != NULL);

    //
    // Get the Bus Device and Function
    //
    Status = PciIo->GetLocation (
                      PciIo,
                      &SegmentNumber,
                      &BusNumber,
                      &DeviceNumber,
                      &FunctionNumber
                      );

    // Combine Bus, Device and Function number
    // Bus       PCI Bus number. Range 0..255.
    // Device    PCI Device number. Range 0..31.
    // Function  PCI Function number. Range 0..7.
    BusDevFuc = ((BusNumber & 0xff) << 8) | ((DeviceNumber & 0x1f) << 3) | (FunctionNumber & 0x07);

    LutIndex = PcieNextLutIndex (&LsPcie[SegmentNumber]);
    if (LutIndex < 0) {
      DEBUG ((DEBUG_WARN, "No free Lut Index for Pcie\n"));
      continue;
    }

    // Get a free StreamId from pool, if not found we are done
    StreamId = PcieNextStreamId ();
    if (StreamId < 0) {
      DEBUG ((DEBUG_WARN, "No free StreamId for Pcie\n"));
      break;
    }

    // Segment Number denotes the controller number
    // Find the controller node offset in Device tree based on this
    PcieNodeOffset = FdtFindPcie (Dtb, LsPcie[SegmentNumber].ControllerAddress);
    if (PcieNodeOffset < 0) {
      DEBUG ((
        DEBUG_WARN,
        "Pcie node with regs address %p node found in Dtb\n",
        LsPcie[SegmentNumber].ControllerAddress
        ));
      continue;
    }

    Status = FdtPcieSetIommuMapEntry (Dtb, PcieNodeOffset, BusDevFuc, StreamId);
    if (Status != EFI_NOT_FOUND) {
      break;
    }

    Status = FdtPcieSetMsiMapEntry (Dtb, PcieNodeOffset, BusDevFuc, StreamId);
    if (Status != EFI_NOT_FOUND) {
      break;
    }

    PcieLutSetMapping (&LsPcie[SegmentNumber], LutIndex, BusDevFuc, StreamId);
  }

  return;
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
  UINTN  Loop;
  INTN   LinkUp;
  UINT64 PciPhyMemAddr[NUM_PCIE_CONTROLLER];
  UINT64 PciPhyCfg0Addr[NUM_PCIE_CONTROLLER];
  UINT64 PciPhyCfg1Addr[NUM_PCIE_CONTROLLER];
  UINT64 PciPhyIoAddr[NUM_PCIE_CONTROLLER];
  UINT64 Regs[NUM_PCIE_CONTROLLER];
  UINT8  PciEnabled[NUM_PCIE_CONTROLLER];
  LS_PCIE *LsPcie;

  *Count = 0;

  LsPcie = AllocateZeroPool (sizeof (LS_PCIE) * NUM_PCIE_CONTROLLER);
  if (!LsPcie) {
    DEBUG ((DEBUG_ERROR, "Failed to allocate memory for Pcie controller data\n"));
    return NULL;
  }

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
    LsPcie[Idx].ControllerAddress = Regs[Idx];
    LsPcie[Idx].NextLutIndex = 0;
    LsPcie[Idx].LsPcieLut = (LS_PCIE_LUT *)(LsPcie[Idx].ControllerAddress + PCI_LUT_BASE);
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

    //
    // Local array to index all enable PCIe controllers
    //
    PciEnabled[*Count] = Idx;

    *Count += BIT0;
  }

  if (!PlatformHasPciIoEvent) {
    PlatformHasPciIoEvent = EfiCreateProtocolNotifyEvent (
                            &gEfiPciIoProtocolGuid,
                            TPL_CALLBACK,
                            OnPlatformHasPciIo,
                            LsPcie,
                            &PlatformHasPciIoNotifyReg
                            );
  }

  if (*Count == 0) {
     return NULL;
  } else {
     for (Loop = 0; Loop < *Count; Loop++) {
        mPciRootBridges[Loop].Segment               = PciEnabled[Loop];
        mPciRootBridges[Loop].Supports              = PCI_SUPPORT_ATTRIBUTES;
        mPciRootBridges[Loop].Attributes            = PCI_SUPPORT_ATTRIBUTES;
        mPciRootBridges[Loop].DmaAbove4G            = FALSE;
        mPciRootBridges[Loop].NoExtendedConfigSpace = FALSE;
        mPciRootBridges[Loop].ResourceAssigned      = FALSE;
        mPciRootBridges[Loop].AllocationAttributes  = PCI_ALLOCATION_ATTRIBUTES;

        mPciRootBridges[Loop].Bus.Base              = PCI_SEG_BUSNUM_MIN;
        mPciRootBridges[Loop].Bus.Limit             = PCI_SEG_BUSNUM_MAX;
        mPciRootBridges[Loop].Io.Base               = PciEnabled[Loop] *
                                                      SEG_IO_SIZE;
        mPciRootBridges[Loop].Io.Limit              = PCI_SEG_PORTIO_MAX +
                                                      (PciEnabled[Loop] *
                                                       SEG_IO_SIZE);
        mPciRootBridges[Loop].Mem.Base              = PCI_SEG_MMIO32_MIN +
                                                      (PciEnabled[Loop] *
                                                       PCI_SEG_MMIO32_DIFF);
        mPciRootBridges[Loop].Mem.Limit             = PCI_SEG_MMIO32_MAX +
                                                      (PciEnabled[Loop] *
                                                      PCI_SEG_MMIO32_DIFF);
        mPciRootBridges[Loop].MemAbove4G.Base       = PciPhyMemAddr[PciEnabled[Loop]];
        mPciRootBridges[Loop].MemAbove4G.Limit      = PciPhyMemAddr[PciEnabled[Loop]] +
                                                      PCI_SEG_MMIO64_MAX_DIFF;

        //
        // No separate ranges for prefetchable and non-prefetchable BARs
        //
        mPciRootBridges[Loop].PMem.Base             = MAX_UINT64;
        mPciRootBridges[Loop].PMem.Limit            = 0;
        mPciRootBridges[Loop].PMemAbove4G.Base      = MAX_UINT64;
        mPciRootBridges[Loop].PMemAbove4G.Limit     = 0;
        mPciRootBridges[Loop].DevicePath            = (EFI_DEVICE_PATH_PROTOCOL *)&mEfiPciRootBridgeDevicePath[PciEnabled[Loop]];
     }

     return mPciRootBridges;
  }
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
