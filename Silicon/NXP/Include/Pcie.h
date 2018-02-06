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

#ifndef _PCI_H_
#define _PCI_H_

// Segment 0
#define PCI_SEG0_NUM              0

#define PCI_SEG0_BUSNUM_MIN       0x0
#define PCI_SEG0_BUSNUM_MAX       0xff

#define PCI_SEG0_PORTIO_MIN       0x0
#define PCI_SEG0_PORTIO_MAX       0xffff

#define PCI_SEG0_MMIO32_MIN       0x40000000
#define PCI_SEG0_MMIO32_MAX       0x4fffffff
#define PCI_SEG0_MMIO64_MIN       0x4040000000
#define PCI_SEG0_MMIO64_MAX       0x407fffffff
#define PCI_SEG0_MMIO_MEMBASE     0x4000000000

#define PCI_SEG0_DBI_BASE         0x03400000

// Segment 1
#define PCI_SEG1_NUM              1

#define PCI_SEG1_BUSNUM_MIN       0x0
#define PCI_SEG1_BUSNUM_MAX       0xff

#define PCI_SEG1_PORTIO_MIN       0x10000
#define PCI_SEG1_PORTIO_MAX       0x1ffff

#define PCI_SEG1_MMIO32_MIN       0x50000000
#define PCI_SEG1_MMIO32_MAX       0x5fffffff
#define PCI_SEG1_MMIO64_MIN       0x4840000000
#define PCI_SEG1_MMIO64_MAX       0x487fffffff
#define PCI_SEG1_MMIO_MEMBASE     0x4800000000

#define PCI_SEG1_DBI_BASE         0x03500000

// Segment 2
#define PCI_SEG2_NUM              2

#define PCI_SEG2_BUSNUM_MIN       0x0
#define PCI_SEG2_BUSNUM_MAX       0xff

#define PCI_SEG2_PORTIO_MIN       0x20000
#define PCI_SEG2_PORTIO_MAX       0x2ffff

#define PCI_SEG2_MMIO32_MIN       0x60000000
#define PCI_SEG2_MMIO32_MAX       0x6fffffff
#define PCI_SEG2_MMIO64_MIN       0x5040000000
#define PCI_SEG2_MMIO64_MAX       0x507fffffff
#define PCI_SEG2_MMIO_MEMBASE     0x5000000000

#define PCI_SEG2_DBI_BASE         0x03600000

// Segment configuration
#define SEG_CFG_SIZE              0x00001000
#define SEG_CFG_BUS               0x00000000
#define SEG_MEM_SIZE              0x40000000
#define SEG_MEM_BUS               0x40000000
#define SEG_IO_SIZE               0x00010000
#define SEG_IO_BUS                0x00000000
#define PCI_BASE_DIFF             0x800000000
#define PCI_DBI_SIZE_DIFF         0x100000
#define PCI_SEG0_PHY_CFG0_BASE    PCI_SEG0_MMIO_MEMBASE
#define PCI_SEG0_PHY_CFG1_BASE    0x4000001000
#define PCI_SEG0_PHY_MEM_BASE     PCI_SEG0_MMIO64_MIN
#define PCI_SEG0_PHY_IO_BASE      0x4000010000

// iATU configuration
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

#define IATU_REGION_INDEX0                           0x0
#define IATU_REGION_INDEX1                           0x1
#define IATU_REGION_INDEX2                           0x2
#define IATU_REGION_INDEX3                           0x3

// PCIe Controller configuration
#define NUM_PCIE_CONTROLLER  FixedPcdGet32 (PcdNumPciController)
#define PCI_LUT_DBG          FixedPcdGet32 (PcdPcieLutDbg)
#define PCI_LUT_BASE         FixedPcdGet32 (PcdPcieLutBase)
#define LTSSM_STATE_MASK     0x3f
#define LTSSM_PCIE_L0        0x11
#define PCI_LINK_CAP         0x7c
#define PCI_LINK_SPEED_MASK  0xf
#define PCI_CLASS_BRIDGE_PCI 0x6040010
#define PCI_CLASS_DEVICE     0x8
#define PCI_DBI_RO_WR_EN     0x8bc
#define PCI_BASE_ADDRESS_0   0x10

extern VOID GetSerdesProtocolMaps (UINT64 *);

extern BOOLEAN IsSerDesLaneProtocolConfigured (UINT64, UINT16);

#endif
