/** Dpaa2EthernetPhyLib.h
  DPAA2 Ethernet PHY library interface

  Copyright 2017 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution. The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#ifndef __DPAA2_ETHERTNET_PHY_LIB_H__
#define __DPAA2_ETHERTNET_PHY_LIB_H__

#include <Uefi.h>

/**
 * MDIO bus controller registers
 */
typedef struct _MEMAC_MDIO_BUS_REGS {
  UINT32 Reserved[0xc];
  /**
   * MDIO configuration and status
   */
  UINT32 MdioStat;
# define MDIO_STAT_CLKDIV(_x)   ((((_x) >> 1) & 0xff) << 8)
# define MDIO_STAT_BSY          (1 << 0)
# define MDIO_STAT_RD_ER        (1 << 1)
# define MDIO_STAT_PRE          (1 << 5)
# define MDIO_STAT_ENC          (1 << 6)
# define MDIO_STAT_HOLD_15_CLK  (7 << 2)
# define MDIO_STAT_NEG          (1 << 23)

  /**
   * MDIO control
   */
  UINT32 MdioCtl;
# define MDIO_CTL_DEV_ADDR(_x)   ((_x) & 0x1f)
# define MDIO_CTL_PORT_ADDR(_x)  (((_x) & 0x1f) << 5)
# define MDIO_CTL_PRE_DIS        (1 << 10)
# define MDIO_CTL_SCAN_EN        (1 << 11)
# define MDIO_CTL_POST_INC       (1 << 14)
# define MDIO_CTL_READ           (1 << 15)

  /**
   * MDIO data
   */
  UINT32 MdioData;
# define MDIO_DATA(_x)    ((_x) & 0xffff)
# define MDIO_DATA_BSY    (1 << 31)

  /**
   * MDIO address
   */
  UINT32 MdioAddr;
} MEMAC_MDIO_BUS_REGS;

/**
 * Phy Id for QC phy
 */
#define QC_PHY 0x5A

/**
 * Control block kept for a PHY MDIO bus
 */
typedef struct _DPAA2_PHY_MDIO_BUS {
  /**
   * Signature for run-time type checking
   */
  UINT32 Signature;
# define DPAA2_PHY_MDIO_BUS_SIGNATURE SIGNATURE_32('M', 'D', 'I', 'O')

  /**
   * Pointer to MMIO registers for the MDIO bus
   */
  MEMAC_MDIO_BUS_REGS *CONST IoRegs;
} DPAA2_PHY_MDIO_BUS;


typedef enum _PHY_INTERFACE_TYPE {
  PHY_INTERFACE_XGMII = 0,
  PHY_INTERFACE_SGMII,
  PHY_INTERFACE_QSGMII,
  PHY_INTERFACE_RGMII,

  /*
   * New values must added above this entry
   */
  NUM_PHY_INTERFACE_TYPES,
  PHY_INTERFACE_NONE
} PHY_INTERFACE_TYPE;

typedef enum _PHY_MEDIA_TYPE {
  COPPER_PHY = 0,
  OPTICAL_PHY,

  /*
   * New values must added above this entry
   */
  NUM_PHY_MEDIA_TYPES,
} PHY_MEDIA_TYPE;

/**
 * State information kept for a DPAA2 PHY
 */
typedef struct _DPAA2_PHY {
  /**
   * Signature for run-time type checking
   */
  UINT32 Signature;
# define DPAA2_PHY_SIGNATURE SIGNATURE_32('D', 'P', 'H', 'Y')

  /**
   * PHY interface type
   */
  PHY_INTERFACE_TYPE PhyInterfaceType;

  /**
   * PHY media type
   */
  PHY_MEDIA_TYPE PhyMediaType;

  /**
   * PHY address on the MDIO bus for this PHY
   */
  UINT8 PhyAddress;

  /**
   * PHY Id,
   */
  UINT8 PhyId;

  /**
   * Boolean flag indicating if the physical link is up (TRUE) or down (FALSE)
   */
  BOOLEAN LinkUp;

  /**
   * Boolean flag indicating if auto-negotiation is supported (TRUE) or not (FALSE)
   */
  BOOLEAN AutoNegotiation;

  /**
   * Boolean flag indicating if full-duplex communication is supported (TRUE) or
   * not (FALSE)
   */
  BOOLEAN FullDuplex;

  /**
   * Speed in MBits/s. Valid values: 10000, 1000, 100, 10
   */
  UINT16 Speed;

  /**
   * Pointer to the MDIO bus that connects this PHY to a DPMAC
   */
  DPAA2_PHY_MDIO_BUS *MdioBus;

  /**
   * PHY Flags
   */
  UINT32 Flags;
# define PHY_BROKEN_RESET  (1 << 0) /* Soft reset not supported */
} DPAA2_PHY;


EFI_STATUS
Dpaa2PhyMdioBusesInit (
  DPAA2_PHY_MDIO_BUS MdioBuses[],
  UINT8              MdioBusesCount
  );

EFI_STATUS
Dpaa2PhyInit (
  DPAA2_PHY *Dpaa2Phy
  );

EFI_STATUS
Dpaa2PhyStartup (
  DPAA2_PHY *Dpaa2Phy
  );

VOID
Dpaa2PhyShutdown (
  DPAA2_PHY *Dpaa2Phy
  );

#endif /* __DPAA2_ETHERTNET_PHY_LIB_H__ */
