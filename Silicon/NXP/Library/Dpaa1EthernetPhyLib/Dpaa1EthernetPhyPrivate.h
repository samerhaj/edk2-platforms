/** Dpaa1EthernetPhyPrivate.h
  DPAA1 Ethernet PHY private common declarations

  Copyright (c) 2016, Freescale Semiconductor, Inc. All rights reserved.

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution. The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#ifndef __DPAA1_ETHERNET_PHY_PRIVATE_H__
#define __DPAA1_ETHERNET_PHY_PRIVATE_H__

#include <Library/Dpaa1EthernetPhyLib.h>
#include <Uefi.h>

/*
 * MDIO Manageable Device addresses IEEE 802.3 clause 45.2
 */

/**
 * Physical Medium Attachment/Physical Medium Dependent
 */
# define MDIO_CTL_DEV_PMAPMD             1

/**
 * Auto-Negotiation
 */
# define MDIO_CTL_DEV_AUTO_NEGOTIATION   7
/**
 * For Older Devices using Clause 22 access
 */
# define MDIO_CTL_DEV_NONE               (-1)

/*
 * PHY registers Refer IEEE 802.3 Clause 22.2.4
 * Table 22.96.97 MII management register set
 */

/**
 * PHY control register
 */
#define PHY_CONTROL_REG	         0x00

/*
* PHY Control Register (PHY_CONTROL_REG) bits
*/
#define PHY_CONTROL_DUPLEX            BIT8
#define PHY_CONTROL_AUTONEG_RESTART   BIT9
#define PHY_CONTROL_AUTONEG_ENABLE    BIT12
#define PHY_CONTROL_RESET             BIT15
#define PHY_CONTROL_LOOPBACK          BIT14

/**
 * PHY status register
 */
#define PHY_STATUS_REG	0x01

/*
 * Fields of the PHY status register
 */
#define PHY_STATUS_LINK_STATUS        BIT2
#define PHY_STATUS_AUTONEG_ABILITY    BIT3
#define PHY_STATUS_AUTONEG_COMPLETE   BIT5
/**
 * PHY autonegotiation timeout (milliseconds)
 */
#define PHY_AUTO_NEGOTIATION_TIMEOUT	5000

/*
 * PHY Identifier Registers
 */
#define PHY_IDENTIFIER_HIGH_WORD   0x0002
#define PHY_IDENTIFIER_LOW_WORD    0x0003

/**
 * Organizationally Unique Identifier (OUI) Mask in PHY Identifier ID
 * and Supported PHYs OUI
 */
#define PHY_IDENTIFIER_OUI_MASK    0x3FFFFFC0
#define REALTEK_OUI                0x000732
#define AQUANTIA_OUI               0x00E86D
#define VITESSE_OUI                0x0001C1

/* Internal PHY Registers - SGMII */
#define PHY_SGMII_IF_MODE_SGMII     0x0001
#define PHY_SGMII_IF_MODE_AN        0x0002
#define PHY_SGMII_DEV_ABILITY_SGMII 0x4001
#define PHY_SGMII_CR_DEF_VAL        0x1140
#define PHY_SGMII_CR_RESET_AN       0x0200
#define PHY_SGMII_IF_SPEED_GIGABIT  0x0008
#define PHY_SGMII_CR_PHY_RESET      0x8000

/* MDIO SGMII REGISTERS */
#define SGMII_DEV_ABIL              0x04
#define SGMII_LINK_TMR_L            0x12
#define SGMII_LINK_TMR_H            0x13
#define SGMII_IF_MODE               0x14

VOID
Dpaa1PhyRegisterWrite (
  IN DPAA1_PHY *Dpaa1Phy,
  IN INT8 MdioCtlDevAddr,
  IN UINT16 PhyRegNum,
  IN UINT16 Value
  );

UINT16
Dpaa1PhyRegisterRead (
  IN DPAA1_PHY *Dpaa1Phy,
  IN INT8 MdioCtlDevAddr,
  IN UINT16 PhyRegNum
  );

EFI_STATUS
Dpaa1PhyRestartAutoNeg (
  IN  DPAA1_PHY *Dpaa1Phy
  );

BOOLEAN
Dpaa1PhyStatus (
  IN  DPAA1_PHY *Dpaa1Phy
  );

EFI_STATUS
AquantiaPhyConfig(DPAA1_PHY *Dpaa1Phy);

EFI_STATUS
AquantiaPhyStartup(DPAA1_PHY *Dpaa1Phy);

EFI_STATUS
RealtekPhyConfig(DPAA1_PHY *Dpaa1Phy);

EFI_STATUS
RealtekPhyStartup(DPAA1_PHY *Dpaa1Phy);

EFI_STATUS
VitessePhyConfig(DPAA1_PHY *Dpaa1Phy);

EFI_STATUS
VitessePhyStartup(DPAA1_PHY *Dpaa1Phy);

#endif /* __DPAA1_ETHERNET_PHY_PRIVATE_H__ */
