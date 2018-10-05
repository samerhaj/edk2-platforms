/** Realtek.c
  Realtek PHY services implementation

  Copyright (c) 2016, Freescale Semiconductor, Inc. All rights reserved.

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution. The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include <Library/Dpaa1DebugLib.h>
#include <Library/IoLib.h>
#include <Library/TimerLib.h>

#include"Dpaa1EthernetPhyPrivate.h"

/*
* Realtek PHY specific pages which contain different registers
*/
#define REALTEK_PAGE_LEDCR       0xd04 // LED Registers' page
#define REALTEK_PAGE_PHY         0xa43 // PHY Specific Control and Status Register
#define REALTEK_PAGE_MIICR       0xd08 // MII Control Register
/*
* Realtek PHY specific registers
*/
#define REALTEK_REG_PAGSR        0x1f  // Page Select Register
#define REALTEK_REG_LCR          0x10  // LED Control Register
#define REALTEK_REG_EEELCR       0x11  // EEE LED Control Register
#define REALTEK_REG_FLCR         0x12  // Fiber LED Control Register
#define REALTEK_REG_PHYSR        0x1A  // PHY specific Status Register
#define REALTEK_REG_RGMII_TXDLY  0x11
/*
* PHY specific Status Register (PHYSR) bits
*/
#define REALTEK_AUTONEG_ENABLE   BIT12
#define REALTEK_PHYSTAT_SPEED    (BIT4|BIT5)
#define REALTEK_PHYSTAT_GBIT     BIT5
#define REALTEK_PHYSTAT_100      BIT4
#define REALTEK_PHYSTAT_DUPLEX   BIT3
#define REALTEK_PHYSTAT_SPDDONE  BIT11
#define REALTEK_PHYSTAT_LINK     BIT2
/*
 * LED Control Register (LCR) Bits
*/
#define REALTEK_LED0_LINK_10     BIT0 // LED0 Link Indication : 10Mbps
#define REALTEK_LED0_LINK_100    BIT1 // LED0 Link Indication : 100Mbps
#define REALTEK_LED0_RSRVD       BIT2 // Reserved
#define REALTEK_LED0_LINK_1000   BIT3 // LED0 Link Indication : 1000Mbps
#define REALTEK_LED0_ACT         BIT4 // LED0 Active (Transmitting or Receiving) Indication
#define REALTEK_LED1_LINK_10     BIT5 // LED1 Link Indication : 10Mbps
#define REALTEK_LED1_LINK_100    BIT6 // LED1 Link Indication : 100Mbps
#define REALTEK_LED1_RSRVD       BIT7 // Reserved
#define REALTEK_LED1_LINK_1000   BIT8 // LED1 Link Indication : 1000Mbps
#define REALTEK_LED1_ACT         BIT9 // LED1 Active (Transmitting or Receiving) Indication
#define REALTEK_LED2_LINK_10     BIT11 // LED2 Link Indication : 10Mbps
#define REALTEK_LED2_LINK_100    BIT11 // LED2 Link Indication : 100Mbps
#define REALTEK_LED2_RSRVD       BIT12 // Reserved
#define REALTEK_LED2_LINK_1000   BIT13 // LED2 Link Indication : 1000Mbps
#define REALTEK_LED2_ACT         BIT14 // LED2 Active (Transmitting or Receiving) Indication
#define REALTEK_LED_RSRVD        BIT15 // Reserved

// Enabling of TXDLY is via register setting Page 0xd08, Reg 17, Bit[8]=1
#define REALTEK_RGMII_TX_DELAY   0x100

/**
   Configures Realtek PHY

   @param[in] Dpaa1Phy          Pointer to PHY object

   @retval EFI_SUCCESS, on success
   @retval error code, on failure

 **/
EFI_STATUS
RealtekPhyConfig (DPAA1_PHY *Dpaa1Phy)
{
  UINT16 PhyRegValue;
  DPAA1_DEBUG_MSG("***** RealtekPhyConfig\n");
  ASSERT(Dpaa1Phy->Signature == DPAA1_PHY_SIGNATURE);

  Dpaa1PhyRegisterWrite(Dpaa1Phy, MDIO_CTL_DEV_NONE,
                          PHY_CONTROL_REG, PHY_CONTROL_RESET);

  if (Dpaa1Phy->PhyInterfaceType == PHY_INTERFACE_RGMII) {
      Dpaa1PhyRegisterWrite(Dpaa1Phy, MDIO_CTL_DEV_NONE,
                      REALTEK_REG_PAGSR, REALTEK_PAGE_MIICR);
      PhyRegValue = Dpaa1PhyRegisterRead(Dpaa1Phy,
                                       MDIO_CTL_DEV_NONE,
                                       REALTEK_REG_RGMII_TXDLY);
      PhyRegValue |= REALTEK_RGMII_TX_DELAY;
      Dpaa1PhyRegisterWrite(Dpaa1Phy, MDIO_CTL_DEV_NONE,
                      REALTEK_REG_RGMII_TXDLY, PhyRegValue);
      Dpaa1PhyRegisterWrite(Dpaa1Phy, MDIO_CTL_DEV_NONE,
                      REALTEK_REG_PAGSR, PHY_CONTROL_REG);
  }

  /* Set green LED for Link, yellow LED for Active */
  PhyRegValue = REALTEK_LED0_LINK_10 | REALTEK_LED0_LINK_100 |
                REALTEK_LED0_RSRVD | REALTEK_LED0_LINK_1000 |
                REALTEK_LED0_ACT | REALTEK_LED1_LINK_10 |
                REALTEK_LED1_LINK_100 | REALTEK_LED1_LINK_1000 |
                REALTEK_LED2_LINK_1000 | REALTEK_LED2_ACT;

  Dpaa1PhyRegisterWrite(Dpaa1Phy, MDIO_CTL_DEV_NONE,
                  REALTEK_REG_PAGSR, REALTEK_PAGE_LEDCR);
  Dpaa1PhyRegisterWrite(Dpaa1Phy, MDIO_CTL_DEV_NONE,
                  REALTEK_REG_LCR, PhyRegValue);
  Dpaa1PhyRegisterWrite(Dpaa1Phy, MDIO_CTL_DEV_NONE,
                  REALTEK_REG_PAGSR, PHY_CONTROL_REG);
  return EFI_SUCCESS;
}

/**
   Start Realtek PHY

   @param[in] Dpaa1Phy          Pointer to PHY object

   @retval EFI_SUCCESS, on success
   @retval error code, on failure

 **/
EFI_STATUS
RealtekPhyStartup (DPAA1_PHY *Dpaa1Phy)
{
  UINT16       PhyRegValue;
  EFI_STATUS   Status;

  DPAA1_DEBUG_MSG("***** RealtekPhyStartup *******\n");

  ASSERT(Dpaa1Phy->Signature == DPAA1_PHY_SIGNATURE);

  Dpaa1PhyRegisterWrite(Dpaa1Phy, MDIO_CTL_DEV_NONE,
        REALTEK_REG_PAGSR, PHY_CONTROL_REG);

  Status = Dpaa1PhyRestartAutoNeg(Dpaa1Phy);
  if (EFI_ERROR(Status))
    return Status;

  Dpaa1PhyRegisterWrite(Dpaa1Phy, MDIO_CTL_DEV_NONE,
        REALTEK_REG_PAGSR, PHY_STATUS_REG);

  //Check Link Status
  Dpaa1PhyStatus(Dpaa1Phy);

  DPAA1_INFO_MSG("PHY link is %a\n", Dpaa1Phy->LinkUp ? "up" : "down");

  Dpaa1PhyRegisterWrite(Dpaa1Phy, MDIO_CTL_DEV_NONE,
        REALTEK_REG_PAGSR, REALTEK_PAGE_PHY);

  PhyRegValue = Dpaa1PhyRegisterRead(Dpaa1Phy,
                                MDIO_CTL_DEV_NONE,
                                REALTEK_REG_PHYSR);

  if (PhyRegValue & REALTEK_PHYSTAT_DUPLEX)
    Dpaa1Phy->FullDuplex = TRUE;
  else
    Dpaa1Phy->FullDuplex = FALSE;

  switch (PhyRegValue & REALTEK_PHYSTAT_SPEED) {
    case REALTEK_PHYSTAT_GBIT:
        Dpaa1Phy->Speed = 1000;
        break;
    case REALTEK_PHYSTAT_100:
        Dpaa1Phy->Speed = 100;
        break;
    default:
        Dpaa1Phy->Speed = 10;
  }
  Dpaa1PhyRegisterWrite(Dpaa1Phy, MDIO_CTL_DEV_NONE,
                        REALTEK_REG_PAGSR, PHY_CONTROL_REG);
  DPAA1_DEBUG_MSG("PHY speed is %d\n", Dpaa1Phy->Speed);
  return EFI_SUCCESS;
}
