/** Aquantia.c
  Aquantia PHY services implementation

  Copyright 2017 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution. The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include <Library/DpaaDebugLib.h>
#include <Library/IoLib.h>
#include <Library/TimerLib.h>

#include "Dpaa2EthernetPhyPrivate.h"

#define AQUNTIA_10G_CTL    0x20
#define AQUNTIA_VENDOR_P1  0xc400

#define AQUNTIA_SPEED_LSB_MASK  0x2000
#define AQUNTIA_SPEED_MSB_MASK  0x40

#define SI_SR                   0xe812
#define VENDOR_PROVISIONING_REG 0xC441

#define USX_AUTONEG_CONTROL_ENA 0x0008
#define SI_IN_USE_MASK          0x0078
#define SI_USXGMII              0x0018


/**
   Configures Aquantia PHY

   @param[in] Dpaa2Phy          Pointer to PHY object

   @retval EFI_SUCCESS, on success
   @retval error code, on failure

 **/
EFI_STATUS
AquantiaPhyConfig (
  DPAA2_PHY *Dpaa2Phy
  )
{
  UINT16 PhyRegValue;

  ASSERT (Dpaa2Phy->Signature == DPAA2_PHY_SIGNATURE);
  ASSERT (Dpaa2Phy->PhyInterfaceType == PHY_INTERFACE_XGMII); /* 10GBASE-T */

  PhyRegValue = Dpaa2PhyRegisterRead (Dpaa2Phy, MDIO_CTL_DEV_PMAPMD,
                                     PHY_CONTROL_REG);

  if (!(PhyRegValue & AQUNTIA_SPEED_LSB_MASK) ||
      !(PhyRegValue & AQUNTIA_SPEED_MSB_MASK)) {
    Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_CTL_DEV_PMAPMD,
                          PHY_CONTROL_REG,
                          AQUNTIA_SPEED_LSB_MASK | AQUNTIA_SPEED_MSB_MASK);
  }
  /* If SI is USXGMII then start USXGMII autoneg*/
  PhyRegValue  = Dpaa2PhyRegisterRead (Dpaa2Phy, MDIO_MMD_PHYXS, SI_SR);

  if ((PhyRegValue & SI_IN_USE_MASK) == SI_USXGMII) {
    Dpaa2PhyRegisterWrite (
      Dpaa2Phy,
      MDIO_MMD_PHYXS,
      VENDOR_PROVISIONING_REG,
      USX_AUTONEG_CONTROL_ENA
      );
  }

  return EFI_SUCCESS;
}


/**
   Start Aquantia PHY

   @param[in] Dpaa2Phy          Pointer to PHY object

   @retval EFI_SUCCESS, on success
   @retval error code, on failure

 **/
EFI_STATUS
AquantiaPhyStartup (
  DPAA2_PHY *Dpaa2Phy
  )
{
  UINTN I;
  UINT16 PhyRegValue;

  ASSERT (Dpaa2Phy->Signature == DPAA2_PHY_SIGNATURE);

  Dpaa2Phy->FullDuplex = TRUE;
  Dpaa2Phy->AutoNegotiation = TRUE;

  if (FixedPcdGetBool (PcdDpaa2PhyAutoNegWait)) {
    /*
     * NOTE: Read twice because link state is latched and a
     * read moves the current state into the register
     */
    (VOID)Dpaa2PhyRegisterRead (Dpaa2Phy,
        MDIO_CTL_DEV_AUTO_NEGOTIATION,
        PHY_STATUS_REG);

    PhyRegValue = Dpaa2PhyRegisterRead (Dpaa2Phy,
        MDIO_CTL_DEV_AUTO_NEGOTIATION,
        PHY_STATUS_REG);

    if (!(PhyRegValue & PHY_STATUS_AUTO_NEGOTIATION_COMPLETE)) {
      DPAA_DEBUG_MSG ("Waiting for PHY (PHY address: 0x%x) auto negotiation to complete ",
          Dpaa2Phy->PhyAddress);
      for (I = 0; I < PHY_AUTO_NEGOTIATION_TIMEOUT; I ++) {
        MicroSecondDelay (1000);
        PhyRegValue = Dpaa2PhyRegisterRead (Dpaa2Phy,
            MDIO_CTL_DEV_AUTO_NEGOTIATION,
            PHY_STATUS_REG);
        if (I % 500 == 0) {
          DPAA_DEBUG_MSG_NO_PREFIX (".");
        }

        if (PhyRegValue & PHY_STATUS_AUTO_NEGOTIATION_COMPLETE) {
          break;
        }
      }

      if (I == PHY_AUTO_NEGOTIATION_TIMEOUT) {
        DPAA_DEBUG_MSG_NO_PREFIX ("TIMEOUT!\n");
        DPAA_ERROR_MSG ("PHY auto-negotiation failed\n");
        Dpaa2Phy->AutoNegotiation = FALSE;
        return EFI_TIMEOUT;
      }

      DPAA_DEBUG_MSG_NO_PREFIX ("\n");
    }
  }

  /*
   * Read twice because link state is latched and a
   * read moves the current state into the register
   */
  (VOID)Dpaa2PhyRegisterRead (Dpaa2Phy,
                             MDIO_CTL_DEV_AUTO_NEGOTIATION,
                             PHY_STATUS_REG);
  PhyRegValue = Dpaa2PhyRegisterRead (Dpaa2Phy,
                                     MDIO_CTL_DEV_AUTO_NEGOTIATION,
                                     PHY_STATUS_REG);

  if (PhyRegValue == (UINT16)-1 ||
      !(PhyRegValue & PHY_STATUS_LINK_STATUS)) {
    Dpaa2Phy->LinkUp = FALSE;
  } else {
    Dpaa2Phy->LinkUp = TRUE;
  }

  DPAA_DEBUG_MSG ("PHY link is %a\n", Dpaa2Phy->LinkUp ? "up" : "down");

  PhyRegValue = Dpaa2PhyRegisterRead (Dpaa2Phy, MDIO_CTL_DEV_PMAPMD,
                                     PHY_CONTROL_REG);
  if (PhyRegValue & AQUNTIA_SPEED_MSB_MASK) {
    if (PhyRegValue & AQUNTIA_SPEED_LSB_MASK) {
      Dpaa2Phy->Speed = 10000;
    } else {
      Dpaa2Phy->Speed = 1000;
    }
  } else {
    if (PhyRegValue & AQUNTIA_SPEED_LSB_MASK) {
      Dpaa2Phy->Speed = 100;
    } else {
      Dpaa2Phy->Speed = 10;
    }
  }

  return EFI_SUCCESS;
}
