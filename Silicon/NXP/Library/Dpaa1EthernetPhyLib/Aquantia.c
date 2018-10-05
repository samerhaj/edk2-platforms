/** Aquantia.c
  Aquantia PHY services implementation

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

#define AQUNTIA_10G_CTL		0x20
#define AQUNTIA_VENDOR_P1		0xc400

#define AQUNTIA_SPEED_LSB_MASK	0x2000
#define AQUNTIA_SPEED_MSB_MASK	0x40

/**
   Configures Aquantia PHY

   @param[in] Dpaa1Phy          Pointer to PHY object

   @retval EFI_SUCCESS, on success
   @retval error code, on failure

 **/
EFI_STATUS
AquantiaPhyConfig(DPAA1_PHY *Dpaa1Phy)
{
  UINT16 PhyRegValue;

  DPAA1_DEBUG_MSG("***** AquantiaPhyConfig\n");
  ASSERT(Dpaa1Phy->Signature == DPAA1_PHY_SIGNATURE);
  ASSERT(Dpaa1Phy->PhyInterfaceType == PHY_INTERFACE_XFI); /* 10GBASE-T */
  
  PhyRegValue = Dpaa1PhyRegisterRead(Dpaa1Phy, MDIO_CTL_DEV_PMAPMD,
                                     PHY_CONTROL_REG);

  if (!(PhyRegValue & AQUNTIA_SPEED_LSB_MASK) ||
      !(PhyRegValue & AQUNTIA_SPEED_MSB_MASK)) {
    Dpaa1PhyRegisterWrite(Dpaa1Phy, MDIO_CTL_DEV_PMAPMD,
                          PHY_CONTROL_REG,
                          AQUNTIA_SPEED_LSB_MASK | AQUNTIA_SPEED_MSB_MASK);
  }

  return EFI_SUCCESS;
}


/**
   Start Aquantia PHY

   @param[in] Dpaa1Phy          Pointer to PHY object

   @retval EFI_SUCCESS, on success
   @retval error code, on failure

 **/
EFI_STATUS
AquantiaPhyStartup(DPAA1_PHY *Dpaa1Phy)
{
  UINTN I;
  UINT16 PhyRegValue;

  DPAA1_DEBUG_MSG("***** AquantiaPhyStartup *******\n");
                    
  ASSERT(Dpaa1Phy->Signature == DPAA1_PHY_SIGNATURE);

  Dpaa1Phy->FullDuplex = TRUE;
  Dpaa1Phy->AutoNegotiation = TRUE;

  /*
   * If the auto-negotiation is still in progress, wait:
   *
   * NOTE: Read twice because link state is latched and a
   * read moves the current state into the register
   */
  (VOID)Dpaa1PhyRegisterRead(Dpaa1Phy,
                             MDIO_CTL_DEV_AUTO_NEGOTIATION,
                             PHY_STATUS_REG);
  PhyRegValue = Dpaa1PhyRegisterRead(Dpaa1Phy,
                                     MDIO_CTL_DEV_AUTO_NEGOTIATION,
                                     PHY_STATUS_REG);
  if (!(PhyRegValue & PHY_STATUS_AUTONEG_COMPLETE)) {
    DPAA1_DEBUG_MSG("Waiting for PHY (PHY address: 0x%x) auto negotiation to complete ",
                    Dpaa1Phy->PhyAddress);
    for (I = 0; I < PHY_AUTO_NEGOTIATION_TIMEOUT; I ++) {
      MicroSecondDelay(1000);
      PhyRegValue = Dpaa1PhyRegisterRead(Dpaa1Phy,
                                         MDIO_CTL_DEV_AUTO_NEGOTIATION,
                                         PHY_STATUS_REG);
      if (I % 500 == 0) {
        DPAA1_DEBUG_MSG_NO_PREFIX(".");
      }

      if (PhyRegValue & PHY_STATUS_AUTONEG_COMPLETE) {
        break;
      }
    }

    if (I == PHY_AUTO_NEGOTIATION_TIMEOUT) {
      DPAA1_DEBUG_MSG_NO_PREFIX("TIMEOUT!\n");
      DPAA1_ERROR_MSG("PHY auto-negotiation failed\n");
      Dpaa1Phy->AutoNegotiation = FALSE;
      return EFI_TIMEOUT;
    }

    DPAA1_DEBUG_MSG_NO_PREFIX("\n");
  }

  /*
   * Read twice because link state is latched and a
   * read moves the current state into the register
   */
  (VOID)Dpaa1PhyRegisterRead(Dpaa1Phy,
                             MDIO_CTL_DEV_AUTO_NEGOTIATION,
                             PHY_STATUS_REG);
  PhyRegValue = Dpaa1PhyRegisterRead(Dpaa1Phy,
                                     MDIO_CTL_DEV_AUTO_NEGOTIATION,
                                     PHY_STATUS_REG);

  if (PhyRegValue == (UINT16)-1 ||
      !(PhyRegValue & PHY_STATUS_LINK_STATUS)) {
    Dpaa1Phy->LinkUp = FALSE;
  } else {
    Dpaa1Phy->LinkUp = TRUE;
  }

  DPAA1_INFO_MSG("PHY link is %a\n", Dpaa1Phy->LinkUp ? "up" : "down");

  PhyRegValue = Dpaa1PhyRegisterRead(Dpaa1Phy, MDIO_CTL_DEV_PMAPMD,
                                     PHY_CONTROL_REG);
  if (PhyRegValue & AQUNTIA_SPEED_MSB_MASK) {
    if (PhyRegValue & AQUNTIA_SPEED_LSB_MASK) {
      Dpaa1Phy->Speed = 10000;
    } else {
      Dpaa1Phy->Speed = 1000;
    }
  } else {
    if (PhyRegValue & AQUNTIA_SPEED_LSB_MASK) {
      Dpaa1Phy->Speed = 100;
    } else {
      Dpaa1Phy->Speed = 10;
    }
  }
  DPAA1_DEBUG_MSG("PHY speed is %d\n", Dpaa1Phy->Speed);

  return EFI_SUCCESS;
}
