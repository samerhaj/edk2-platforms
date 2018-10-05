/** Dpaa1EthernetPhyLib.h
  DPAA1 Ethernet PHY library interface

  Copyright 2017 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution. The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#ifndef __DPAA1_ETHERTNET_PHY_LIB_H__
#define __DPAA1_ETHERTNET_PHY_LIB_H__

#include <Uefi.h>

/**
 * MDIO bus controller registers
 */
typedef volatile struct _MEMAC_MDIO_BUS_REGS {
  UINT32 Reserved[0xc];
  /**
   * MDIO configuration and status
   */
  UINT32 MdioStat;
# define MDIO_STAT_CLKDIV(_x)	((((_x) >> 1) & 0xff) << 8)
# define MDIO_STAT_BSY		BIT(0)
# define MDIO_STAT_RD_ER		BIT(1)
# define MDIO_STAT_PRE		BIT(5)
# define MDIO_STAT_ENC		BIT(6)
# define MDIO_STAT_HOLD_15_CLK	(7 << 2)
# define MDIO_STAT_NEG		BIT(23)

  /**
   * MDIO control
   */
  UINT32 MdioCtl;
# define MDIO_CTL_DEV_ADDR(_x)	((_x) & 0x1f)
# define MDIO_CTL_PORT_ADDR(_x)	(((_x) & 0x1f) << 5)
# define MDIO_CTL_PRE_DIS		BIT(10)
# define MDIO_CTL_SCAN_EN		BIT(11)
# define MDIO_CTL_POST_INC		BIT(14)
# define MDIO_CTL_READ		BIT(15)

  /**
   * MDIO data
   */
  UINT32 MdioData;
# define MDIO_DATA(_x)		((_x) & 0xffff)
# define MDIO_DATA_BSY		BIT(31)

  /**
   * MDIO address
   */
  UINT32 MdioAddr;
} MEMAC_MDIO_BUS_REGS;


/**
 * Control block kept for a PHY MDIO bus
 */
typedef struct _DPAA1_PHY_MDIO_BUS {
  /**
   * Signature for run-time type checking
   */
  UINT32 Signature;
# define DPAA1_PHY_MDIO_BUS_SIGNATURE SIGNATURE_32('M', 'D', 'I', 'O')

  /**
   * Pointer to MMIO registers for the MDIO bus
   */
  MEMAC_MDIO_BUS_REGS *CONST IoRegs;
} DPAA1_PHY_MDIO_BUS;

/* IF_MODE - Interface Mode Register */
#define IF_MODE_EN_AUTO     0x00008000 /* 1 - Enable automatic speed selection */
#define IF_MODE_SETSP_100M  0x00000000 /* 00 - 100Mbps RGMII */
#define IF_MODE_SETSP_10M   0x00002000 /* 01 - 10Mbps RGMII */
#define IF_MODE_SETSP_1000M 0x00004000 /* 10 - 1000Mbps RGMII */
#define IF_MODE_SETSP_MASK  0x00006000 /* setsp mask bits */
#define IF_MODE_XFI       0x00000000 /* 00- XFI(10) interface mode */
#define IF_MODE_GMII        0x00000002 /* 10- GMII interface mode */
#define IF_MODE_MASK 	0x00000003 /* mask for mode interface mode */
#define IF_MODE_RGMII       0x00000004 /* 1- RGMII */

#define IF_DEFAULT   (IF_GMII)

typedef enum _PHY_INTERFACE_TYPE {
  PHY_INTERFACE_XFI = 0,
  PHY_INTERFACE_SGMII,
  PHY_INTERFACE_SGMII_2500,
  PHY_INTERFACE_QSGMII,
  PHY_INTERFACE_RGMII,

  /*
   * New values must added above this entry
   */
  NUM_PHY_INTERFACE_TYPES,
  PHY_INTERFACE_NONE
} PHY_INTERFACE_TYPE;

/**
 * State information kept for a DPAA1 PHY
 */
typedef struct _DPAA1_PHY {
  /**
   * Signature for run-time type checking
   */
  UINT32 Signature;
# define DPAA1_PHY_SIGNATURE SIGNATURE_32('D', 'P', 'H', 'Y')

  /**
   * PHY interface type
   */
  PHY_INTERFACE_TYPE PhyInterfaceType;

  /**
   * PHY address on the MDIO bus for this PHY
   */
  UINT8 PhyAddress;

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
   * Pointer to the MDIO bus that connects this PHY to a MEMAC
   */
  DPAA1_PHY_MDIO_BUS *MdioBus;

  /**
   * PHY Flags
   */
  UINT32 Flags;
# define PHY_BROKEN_RESET   BIT(0) /* Soft reset not supported */

  UINT32 PhyIdentifier;            /*IEEE 802.3 clause 22.2.4.3.1*/

  EFI_STATUS (*Dpaa1PhyStartup)(struct _DPAA1_PHY *);

  EFI_STATUS (*Dpaa1PhyConfig)(struct _DPAA1_PHY *);

} DPAA1_PHY;


EFI_STATUS
Dpaa1PhyMdioBusesInit(DPAA1_PHY_MDIO_BUS MdioBuses[], UINT8 MdioBusesCount);

EFI_STATUS
Dpaa1PhyInit(DPAA1_PHY *Dpaa1Phy);

EFI_STATUS
Dpaa1PhyStartup(DPAA1_PHY *Dpaa1Phy);

VOID
Dpaa1PhyShutdown(DPAA1_PHY *Dpaa1Phy);

BOOLEAN
Dpaa1PhyStatus (
  IN  DPAA1_PHY *Dpaa1Phy
  );

VOID
DtsecInitPhy (
    IN  DPAA1_PHY_MDIO_BUS *MdioBus,
    IN  DPAA1_PHY *Dpaa1Phy
    );

#endif /* __DPAA1_ETHERTNET_PHY_LIB_H__ */
