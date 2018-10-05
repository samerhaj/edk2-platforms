/** Vitesse.c
  Vitesse PHY services implementation

  Copyright 2016- 2017 NXP

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

/**
* Vitesse PHY specific pages which contain different registers
*/
// IEEE Clause 22 device registers with addresses from 0 to 31
#define VITESSE_PAGE_MAINREG     0x0000
// Three Pages of Extended Registers
#define VITESSE_PAGE_EXTREG1     0x0001 // 16E1-30E1
#define VITESSE_PAGE_EXTREG2     0x0002 // 16E2-30E2
#define VITESSE_PAGE_EXTREG3     0x0003 // 16E2-30E3
// General Purpose Registers' Page
#define VITESSE_PAGE_GENPURPOSE  0x0010 // 0G to 30G

#define VITESSE_PAGE_CHANGE(PAGE) do { \
  if(PAGE >= VITESSE_PAGE_MAINREG && \
    (PAGE <= VITESSE_PAGE_EXTREG3 || PAGE == VITESSE_PAGE_GENPURPOSE)) \
      Dpaa1PhyRegisterWrite(Dpaa1Phy, MDIO_CTL_DEV_NONE, 0x1F, PAGE); \
} while (0)

/*
* Vitesse Vendor Specific Registers (16-31) as specified by Clause 22
*/
#define VITESSE_REG_EXTENDED_PHY_CNTL_SET1  23 // Extended PHY Control Set 1
#define VITESSE_REG_AUX_CTL_STATUS          28 // Auxiliary Control and Status
/*
* Vitesse Extended Page3 Registers
*/
#define VITESSE_E3REG_MAC_SERDES_PCS_CTL    16 // MAC SerDes PCS Control
/*
* Vitesse Extended Page3 Registers' Bits
*/
#define VITESSE_E3REG_MAC_SERDES_PCS_CTL_AUTONEG_ENABLE    BIT7 // MAC SerDes ANEG Enable
/*
* Vitesse General Purpose Registers on Page Gen Purpose Page (0x0010)
*/
#define VITESSE_GENREG_MICRO_CMD            18  // Micro Command Register
#define VITESSE_GENREG_MACMODE_FASTLINK     19  // Mac Mode and Fast Link Configuration
/*
* Vitesse General Purpose Registers' Values
*/
#define VITESSE_GENREG_MICRO_CMD_QSGMII    0x80E0

/**
   Configures Vitesse PHY

   @param[in] Dpaa1Phy          Pointer to PHY object

   @retval EFI_SUCCESS, on success
   @retval error code, on failure

 **/
EFI_STATUS
VitessePhyConfig (DPAA1_PHY *Dpaa1Phy)
{
  UINT16 PhyRegValue;
  UINT32 TimeoutMsCount = 250;
  DPAA1_DEBUG_MSG("***** VitessePhyConfig\n");
  ASSERT(Dpaa1Phy->Signature == DPAA1_PHY_SIGNATURE);

  VITESSE_PAGE_CHANGE(VITESSE_PAGE_GENPURPOSE);

  PhyRegValue = Dpaa1PhyRegisterRead(Dpaa1Phy, MDIO_CTL_DEV_NONE,
                                         VITESSE_GENREG_MACMODE_FASTLINK);
  /* bit 15:14 MAC Configuration *
   *      0 0  Reserved          *
   *      0 1  QSGMII            *
   *      1 0  Reserved          *
   *      1 1  Reserved          */
  if (Dpaa1Phy->PhyInterfaceType == PHY_INTERFACE_QSGMII) {
    PhyRegValue |= BIT14;
    PhyRegValue &= ~BIT15;
    Dpaa1PhyRegisterWrite(Dpaa1Phy, MDIO_CTL_DEV_NONE,
                          VITESSE_GENREG_MACMODE_FASTLINK, PhyRegValue);
    /* Enable 4 ports MAC QSGMII */
    Dpaa1PhyRegisterWrite(Dpaa1Phy, MDIO_CTL_DEV_NONE, VITESSE_GENREG_MICRO_CMD,
                          VITESSE_GENREG_MICRO_CMD_QSGMII);
  }

  /* Register 18 is command register. Bit 15 tells internal processot to execute command.    *
  * when bit 15 is cleared the command has completed. software needs to wait until bit 15=0. *
  * before proceeding with the next PHY register access. Bit 14 typically indicates error.   *
  * commands may take upto 25msec to complete before bit 15 changes to zero                  */
  for ( ; ; ) {
    PhyRegValue = Dpaa1PhyRegisterRead(Dpaa1Phy, MDIO_CTL_DEV_NONE,
                                           VITESSE_GENREG_MICRO_CMD);
    if (PhyRegValue == (UINT16)(-1))
      return EFI_DEVICE_ERROR;

    if (!(PhyRegValue & BIT15))
      break;

    if (PhyRegValue & BIT14)
      return EFI_DEVICE_ERROR;

    if (TimeoutMsCount == 0)
      return EFI_TIMEOUT;

    MicroSecondDelay(1000);
    TimeoutMsCount --;
  }

  VITESSE_PAGE_CHANGE(VITESSE_PAGE_MAINREG);

  PhyRegValue = Dpaa1PhyRegisterRead(Dpaa1Phy, MDIO_CTL_DEV_NONE,
                                         VITESSE_REG_EXTENDED_PHY_CNTL_SET1);
  /* bit 10:8 Media Operation Mode *
   *          Super Sticky Bits    *
   *     000  Cat5 Copper Only     */
  PhyRegValue &= ~(BIT8|BIT9|BIT10);
  Dpaa1PhyRegisterWrite(Dpaa1Phy, MDIO_CTL_DEV_NONE,
                              VITESSE_REG_EXTENDED_PHY_CNTL_SET1, PhyRegValue);

  /* Enable Serdes Auto-negotiation */
  VITESSE_PAGE_CHANGE(VITESSE_PAGE_EXTREG3);

  PhyRegValue = Dpaa1PhyRegisterRead(Dpaa1Phy, MDIO_CTL_DEV_NONE,
                                       VITESSE_E3REG_MAC_SERDES_PCS_CTL);
  PhyRegValue |= VITESSE_E3REG_MAC_SERDES_PCS_CTL_AUTONEG_ENABLE;
  Dpaa1PhyRegisterWrite(Dpaa1Phy, MDIO_CTL_DEV_NONE,
                             VITESSE_E3REG_MAC_SERDES_PCS_CTL, PhyRegValue);

  // Set Main Clause 22 Registers' Page
  VITESSE_PAGE_CHANGE(VITESSE_PAGE_MAINREG);

  return EFI_SUCCESS;
}

/**
   Start Vitesse PHY

   @param[in] Dpaa1Phy          Pointer to PHY object

   @retval EFI_SUCCESS, on success
   @retval error code, on failure

 **/
EFI_STATUS
VitessePhyStartup (DPAA1_PHY *Dpaa1Phy)
{
  UINT16 PhyRegValue;
  EFI_STATUS Status;

  DPAA1_DEBUG_MSG("***** VitessePhyStartup *******\n");

  ASSERT(Dpaa1Phy->Signature == DPAA1_PHY_SIGNATURE);

  VITESSE_PAGE_CHANGE(VITESSE_PAGE_MAINREG);

  Status = Dpaa1PhyRestartAutoNeg(Dpaa1Phy);
  if (EFI_ERROR(Status))
    return Status;

  //Check Link Status
  Dpaa1PhyStatus(Dpaa1Phy);

  DPAA1_INFO_MSG("PHY link is %a\n", Dpaa1Phy->LinkUp ? "up" : "down");

  PhyRegValue = Dpaa1PhyRegisterRead(Dpaa1Phy, MDIO_CTL_DEV_NONE,
                                         VITESSE_REG_AUX_CTL_STATUS);
  /* bit 5 FDX Status   *
   *     1 Full Duplex  *
   *     0 Half Duplex  */
  if (PhyRegValue & BIT5)
    Dpaa1Phy->FullDuplex = TRUE;
  else
    Dpaa1Phy->FullDuplex = FALSE;

  /* bit 4:3 Speed Status                        *
   *     0 0  Speed is 10BASE-T                  *
   *     0 1  Speed is 100BASE-TX or 100BASE-FX  *
   *     1 0  Speed is 1000BASE-T or 1000BASE-X  *
   *     1 1  Reserved                           */
  switch ((PhyRegValue & (BIT4|BIT3))>>3) {
    case 0:
        Dpaa1Phy->Speed = 10;
        break;
    case 1:
        Dpaa1Phy->Speed = 100;
        break;
    case 2:
        Dpaa1Phy->Speed = 1000;
        break;
    default:
        DPAA1_ERROR_MSG("Could not determine PHY speed\n");
        return EFI_DEVICE_ERROR;
        break;
  }

  DPAA1_DEBUG_MSG("PHY speed is %d\n", Dpaa1Phy->Speed);

  return EFI_SUCCESS;
}
