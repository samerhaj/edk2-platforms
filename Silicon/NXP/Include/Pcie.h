/** @file
  PCI memory configuration for NXP

  Copyright 2018 NXP

  This program and the accompanying materials are licensed and made available
  under the terms and conditions of the BSD License which accompanies this
  distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php.

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS, WITHOUT
  WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#ifndef __PCI_H__
#define __PCI_H__

// Segment 0
#define PCI_SEG0_NUM              0
#define PCI_SEG0_MMIO32_MIN       0x40000000
#define PCI_SEG0_MMIO32_MAX       0x4fffffff
#define PCI_SEG0_MMIO64_MIN       PCI_SEG0_MMIO_MEMBASE + SEG_MEM_SIZE
#define PCI_SEG0_MMIO64_MAX       PCI_SEG0_MMIO_MEMBASE + SEG_MEM_LIMIT
#define PCI_SEG0_MMIO_MEMBASE     FixedPcdGet64 (PcdPciExp1BaseAddr)
#define PCI_SEG0_DBI_BASE         0x03400000

// Segment 1
#define PCI_SEG1_NUM              1
#define PCI_SEG1_MMIO32_MIN       0x50000000
#define PCI_SEG1_MMIO32_MAX       0x5fffffff
#define PCI_SEG1_MMIO64_MIN       PCI_SEG1_MMIO_MEMBASE + SEG_MEM_SIZE
#define PCI_SEG1_MMIO64_MAX       PCI_SEG1_MMIO_MEMBASE + SEG_MEM_LIMIT
#define PCI_SEG1_MMIO_MEMBASE     FixedPcdGet64 (PcdPciExp2BaseAddr)
#define PCI_SEG1_DBI_BASE         0x03500000

// Segment 2
#define PCI_SEG2_NUM              2
#define PCI_SEG2_MMIO32_MIN       0x60000000
#define PCI_SEG2_MMIO32_MAX       0x6fffffff
#define PCI_SEG2_MMIO64_MIN       PCI_SEG2_MMIO_MEMBASE + SEG_MEM_SIZE
#define PCI_SEG2_MMIO64_MAX       PCI_SEG2_MMIO_MEMBASE + SEG_MEM_LIMIT
#define PCI_SEG2_MMIO_MEMBASE     FixedPcdGet64 (PcdPciExp3BaseAddr)
#define PCI_SEG2_DBI_BASE         0x03600000

// Segment 3
#define PCI_SEG3_NUM              3
#define PCI_SEG3_MMIO32_MIN       0x70000000
#define PCI_SEG3_MMIO32_MAX       0x7fffffff
#define PCI_SEG3_MMIO64_MIN       PCI_SEG3_MMIO_MEMBASE + SEG_MEM_SIZE
#define PCI_SEG3_MMIO64_MAX       PCI_SEG3_MMIO_MEMBASE + SEG_MEM_LIMIT
#define PCI_SEG3_MMIO_MEMBASE     FixedPcdGet64 (PcdPciExp4BaseAddr)
#define PCI_SEG3_DBI_BASE         0x03700000

// Segment 4
#define PCI_SEG4_NUM              4
#define PCI_SEG4_MMIO32_MIN       0x70000000
#define PCI_SEG4_MMIO32_MAX       0x7fffffff
#define PCI_SEG4_MMIO64_MIN       PCI_SEG4_MMIO_MEMBASE + SEG_MEM_SIZE
#define PCI_SEG4_MMIO64_MAX       PCI_SEG4_MMIO_MEMBASE + SEG_MEM_LIMIT
#define PCI_SEG4_MMIO_MEMBASE     FixedPcdGet64 (PcdPciExp5BaseAddr)
#define PCI_SEG4_DBI_BASE         0x03800000

// Segment 5
#define PCI_SEG5_NUM              5
#define PCI_SEG5_MMIO32_MIN       0x70000000
#define PCI_SEG5_MMIO32_MAX       0x7fffffff
#define PCI_SEG5_MMIO64_MIN       PCI_SEG5_MMIO_MEMBASE + SEG_MEM_SIZE
#define PCI_SEG5_MMIO64_MAX       PCI_SEG5_MMIO_MEMBASE + SEG_MEM_LIMIT
#define PCI_SEG5_MMIO_MEMBASE     FixedPcdGet64 (PcdPciExp6BaseAddr)
#define PCI_SEG5_DBI_BASE         0x03900000

// Segment configuration
#define PCI_SEG_BUSNUM_MIN        0x0
#define PCI_SEG_BUSNUM_MAX        0xff
#define PCI_SEG_PORTIO_MAX        0xffff
#define PCI_SEG_MMIO32_MIN        0x40000000
#define PCI_SEG_MMIO32_MAX        0x4fffffff
#define PCI_SEG_MMIO32_DIFF       0x10000000
#define PCI_SEG_MMIO64_MAX_DIFF   0x3fffffff
#define SEG_CFG_SIZE              0x00001000
#define SEG_CFG_BUS               0x00000000
#define SEG_MEM_SIZE              0x40000000
#define SEG_MEM_LIMIT             0x7fffffff
#define SEG_MEM_BUS               0x40000000
#define SEG_IO_SIZE               0x00010000
#define SEG_IO_BUS                0x00000000
#define PCI_SEG_PORTIO_LIMIT      (NUM_PCIE_CONTROLLER * SEG_IO_SIZE) + \
                                  PCI_SEG_PORTIO_MAX
#define PCI_BASE_DIFF             0x800000000
#define PCI_DBI_SIZE_DIFF         0x100000
#define PCI_SEG0_PHY_CFG0_BASE    PCI_SEG0_MMIO_MEMBASE
#define PCI_SEG0_PHY_CFG1_BASE    PCI_SEG0_PHY_CFG0_BASE + SEG_CFG_SIZE
#define PCI_SEG0_PHY_MEM_BASE     PCI_SEG0_MMIO64_MIN
#define PCI_SEG0_PHY_IO_BASE      PCI_SEG0_MMIO_MEMBASE + SEG_IO_SIZE

#define IATU_REGION_INDEX0        0x0
#define IATU_REGION_INDEX1        0x1
#define IATU_REGION_INDEX2        0x2
#define IATU_REGION_INDEX3        0x3

// PCIe Controller configuration
#define NUM_PCIE_CONTROLLER       FixedPcdGet32 (PcdNumPciController)
#define PCI_LUT_DBG               FixedPcdGet32 (PcdPcieLutDbg)
#define PCI_LUT_BASE              FixedPcdGet32 (PcdPcieLutBase)
#define LTSSM_PCIE_L0             0x11
#define PCI_LINK_CAP              0x7c
#define PCI_LINK_SPEED_MASK       0xf


#define IATU_VIEWPORT_OFF                            0x900
#define IATU_VIEWPORT_OUTBOUND                       0

#define IATU_REGION_CTRL_1_OFF_OUTBOUND_0            0x904
#define IATU_REGION_CTRL_1_OFF_OUTBOUND_0_TYPE_MEM   0x0
#define IATU_REGION_CTRL_1_OFF_OUTBOUND_0_TYPE_IO    0x2
#define IATU_REGION_CTRL_1_OFF_OUTBOUND_0_TYPE_CFG0  0x4
#define IATU_REGION_CTRL_1_OFF_OUTBOUND_0_TYPE_CFG1  0x5

#define IATU_REGION_CTRL_2_OFF_OUTBOUND_0            0x908
#define IATU_REGION_CTRL_2_OFF_OUTBOUND_0_REGION_EN  BIT31

#define IATU_LWR_BASE_ADDR_OFF_OUTBOUND_0            0x90C
#define IATU_UPPER_BASE_ADDR_OFF_OUTBOUND_0          0x910
#define IATU_LIMIT_ADDR_OFF_OUTBOUND_0               0x914
#define IATU_LWR_TARGET_ADDR_OFF_OUTBOUND_0          0x918
#define IATU_UPPER_TARGET_ADDR_OFF_OUTBOUND_0        0x91C

#define PCI_CLASS_BRIDGE_PCI                         0x6040010
#define PCI_CLASS_DEVICE                             0x8
#define PCI_DBI_RO_WR_EN                             0x8bc
#define PCI_BASE_ADDRESS_0                           0x10

// PAB CSR
#define PAB_CTRL                                     0x808
#define PAB_CTRL_APIO_EN                             0x1
#define PAB_CTRL_PPIO_EN                             0x1 << 1
#define PAB_CTRL_PAGE_SEL_SHIFT                      13
#define PAB_CTRL_PAGE_SEL_MASK                       0x3f

// AXI PIO
#define PAB_AXI_PIO_CTRL(Idx)                        (0x840 + 0x10 * Idx)
#define APIO_EN                                      0x1
#define MEM_WIN_EN                                   0x1 << 1
#define IO_WIN_EN                                    0x1 << 2
#define CFG_WIN_EN                                   0x1 << 3

#define INDIRECT_ADDR_BNDRY                          0xc00
#define PAGE_IDX_SHIFT                               10
#define PAGE_ADDR_MASK                               0x3ff

#define OFFSET_TO_PAGE_IDX(Off)                \
       ((Off >> PAGE_IDX_SHIFT) & PAB_CTRL_PAGE_SEL_MASK)

#define OFFSET_TO_PAGE_ADDR(Off)                \
       ((Off & PAGE_ADDR_MASK) | INDIRECT_ADDR_BNDRY)

// APIO WINDOW
#define PAB_AXI_AMAP_CTRL(Idx)                       (0xba0 + 0x10 * Idx)
#define PAB_EXT_AXI_AMAP_SIZE(Idx)                   (0xbaf0 + 0x4 * Idx)
#define PAB_AXI_AMAP_AXI_WIN(Idx)                    (0xba4 + 0x10 * Idx)
#define PAB_EXT_AXI_AMAP_AXI_WIN(Idx)                (0x80a0 + 0x4 * Idx)
#define PAB_AXI_AMAP_PEX_WIN_L(Idx)                  (0xba8 + 0x10 * Idx)
#define PAB_AXI_AMAP_PEX_WIN_H(Idx)                  (0xbac + 0x10 * Idx)

#define AXI_AMAP_CTRL_EN                             0x1
#define AXI_AMAP_CTRL_TYPE_SHIFT                     1
#define AXI_AMAP_CTRL_TYPE_MASK                      0x3
#define AXI_AMAP_CTRL_SIZE_SHIFT                     10
#define AXI_AMAP_CTRL_SIZE_MASK                      0x3fffff

#define PAB_AXI_TYPE_CFG                             0x00
#define PAB_AXI_TYPE_IO                              0x01
#define PAB_AXI_TYPE_MEM                             0x02

#define LX_PEX_CFG_SIZE                              0x00020000

// CCSR registers
#define PCIE_LINK_CTRL_STA                           0x5c
#define PCIE_LINK_SPEED_SHIFT                        16
#define PCIE_LINK_SPEED_MASK                         0x0f
#define PCIE_LINK_WIDTH_SHIFT                        20
#define PCIE_LINK_WIDTH_MASK                         0x3f

// PF control registers
#define PCIE_LTSSM_STA                               0x7fc

#if (FixedPcdGetBool (PcdPcieConfigurePex))
#define LTSSM_STATE_MASK                             0x7f
#else
#define LTSSM_STATE_MASK                             0x3f
#endif

VOID GetSerdesProtocolMaps (UINT64 *);

BOOLEAN IsSerDesLaneProtocolConfigured (UINT64, UINT16);

#endif
