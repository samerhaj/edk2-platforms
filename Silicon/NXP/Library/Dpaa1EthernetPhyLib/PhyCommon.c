/** PhyCommon.c
  DPAA1 Ethernet PHY common services implementation

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

#include <Bitops.h>
#include"Dpaa1EthernetPhyPrivate.h"

#define MDIO_CLOCK_DIVIDER  258
#define MDIO_PHY_DEV_ADDR   0x0

/**
   Initializes the given DPAA1 PHY MDIO bus

   @param[in] MdioBus   Pointer to MDIO bus object

   @retval EFI_SUCCESS, on success
   @retval error code, on failure

 **/
STATIC EFI_STATUS
Dpaa1PhyMdioBusInit (
  IN  DPAA1_PHY_MDIO_BUS *MdioBus
  )
{
  MEMAC_MDIO_BUS_REGS *CONST MdioBusRegs = MdioBus->IoRegs;

  UINT32 RegValue;

  RegValue = MmioReadBe32((UINTN)&MdioBusRegs->MdioStat);

  RegValue |= MDIO_STAT_CLKDIV(MDIO_CLOCK_DIVIDER) | MDIO_STAT_NEG;
  DPAA1_DEBUG_MSG(" MDIO Init : 0x%x : 0x%x \n",
                                &MdioBusRegs->MdioStat, RegValue);
  MmioWriteBe32((UINTN)&MdioBusRegs->MdioStat, RegValue);

  return EFI_SUCCESS;
}

/**
   Initializes the MDIO buses used by DPAA1 mEMACs to reach their
   corresponding PHYs.

   @param[in] MdioBuses         Pointer to array of MDIO buses to initialize
   @param[in] MdioBusesCount    Number of entries in MdioBuses[]

   @retval EFI_SUCCESS, on success
   @retval error code, on failure

 **/
EFI_STATUS
Dpaa1PhyMdioBusesInit (
  IN  DPAA1_PHY_MDIO_BUS MdioBuses[],
  UINT8 MdioBusesCount
  )
{
  EFI_STATUS Status;
  UINT8 I;

  for (I = 0; I < MdioBusesCount; I ++) {
    Status = Dpaa1PhyMdioBusInit(&MdioBuses[I]);
    if (EFI_ERROR(Status)) {
      return Status;
    }
  }

  return EFI_SUCCESS;
}

STATIC
VOID
Dpaa1PhyMdioBusWrite (
  IN DPAA1_PHY_MDIO_BUS *MdioBus,
  IN UINT8 PhyAddress,
  IN INT8 MdioCtlDevAddr,
  IN UINT16 PhyRegNum,
  IN UINT16 Value
  )
{
  UINT32 RegValue;
  BOOLEAN Clause45 = TRUE; // Clause 45 is used for 10 GigPHYs and MACs
  MEMAC_MDIO_BUS_REGS *CONST MdioBusRegs = MdioBus->IoRegs;
  DPAA1_DEBUG_MSG("MDIO bus WRITE for PHY addr 0x%x, dev addr %d, "
                   "reg num 0x%x (MDIO stat reg: 0x%x)\n",
                   PhyAddress, MdioCtlDevAddr, PhyRegNum, Value);

  ASSERT(MdioBus->Signature == DPAA1_PHY_MDIO_BUS_SIGNATURE);

  RegValue = MmioReadBe32((UINTN)&MdioBusRegs->MdioStat);
  if (MDIO_CTL_DEV_NONE == MdioCtlDevAddr) {
    Clause45 = FALSE; // Clause 22
    MdioCtlDevAddr = MDIO_CTL_DEV_ADDR(PhyRegNum);
    RegValue &= ~MDIO_STAT_ENC;
  }
  else
    RegValue |= MDIO_STAT_ENC;

  MmioWriteBe32((UINTN)&MdioBusRegs->MdioStat, RegValue);

  /*
   * Wait until the MDIO bus is not busy
   */
  do {
    RegValue = MmioReadBe32((UINTN)&MdioBusRegs->MdioStat);
  } while (RegValue & MDIO_STAT_BSY);

  /*
   * Specify the target PHY and PHY control device:
   */
  RegValue = MDIO_CTL_PORT_ADDR(PhyAddress) |
             MDIO_CTL_DEV_ADDR(MdioCtlDevAddr);
  MmioWriteBe32((UINTN)&MdioBusRegs->MdioCtl, RegValue);

  /*
   * Specify the target PHY register:
   */
  if (TRUE == Clause45)
    MmioWriteBe32((UINTN)&MdioBusRegs->MdioAddr, PhyRegNum);

  /*
   * Wait until the MDIO bus is not busy:
   */
  do {
    RegValue = MmioReadBe32((UINTN)&MdioBusRegs->MdioStat);
  } while (RegValue & MDIO_STAT_BSY);

  /*
   * Write the value to the PHY register
   */
  MmioWriteBe32((UINTN)&MdioBusRegs->MdioData, MDIO_DATA(Value));

  /*
   * Wait until the MDIO write is complete
   */
  do {
    RegValue = MmioReadBe32((UINTN)&MdioBusRegs->MdioData);
  } while (RegValue & MDIO_DATA_BSY);
}

/**
   Writes a value to a PHY register

   @param[in] Dpaa1Phy          Pointer to PHY object
   @param[in] MdioCtlDevAddr    MDIO control device selector
   @param[in] PhyRegNum         Phy register index
   @param[in] Value             Value to write to the PHY register

   @retval EFI_SUCCESS, on success
   @retval error code, on failure

 **/
VOID
Dpaa1PhyRegisterWrite (
  IN DPAA1_PHY *Dpaa1Phy,
  IN INT8 MdioCtlDevAddr,
  IN UINT16 PhyRegNum,
  IN UINT16 Value
  )
{
  ASSERT(Dpaa1Phy->Signature == DPAA1_PHY_SIGNATURE);
  ASSERT(MdioCtlDevAddr == MDIO_PHY_DEV_ADDR ||
         MdioCtlDevAddr == MDIO_CTL_DEV_NONE ||
         MdioCtlDevAddr == MDIO_CTL_DEV_PMAPMD ||
         MdioCtlDevAddr == MDIO_CTL_DEV_AUTO_NEGOTIATION);

  Dpaa1PhyMdioBusWrite(Dpaa1Phy->MdioBus,
                       Dpaa1Phy->PhyAddress,
                       MdioCtlDevAddr,
                       PhyRegNum,
                       Value);
}

STATIC
UINT16
Dpaa1PhyMdioBusRead (
  IN DPAA1_PHY_MDIO_BUS *MdioBus,
  IN UINT8 PhyAddress,
  IN INT8 MdioCtlDevAddr,
  IN UINT16 PhyRegNum
  )
{
  UINT32 RegValue;
  MEMAC_MDIO_BUS_REGS *CONST MdioBusRegs = MdioBus->IoRegs;
  BOOLEAN Clause45 = TRUE; // Clause 45 is used for 10 GigPHYs and MACs

  ASSERT(MdioBus->Signature == DPAA1_PHY_MDIO_BUS_SIGNATURE);

  RegValue = MmioReadBe32((UINTN)&MdioBusRegs->MdioStat);
  if (MDIO_CTL_DEV_NONE == MdioCtlDevAddr) {
    Clause45 = FALSE; // Clause 22
    MdioCtlDevAddr = MDIO_CTL_DEV_ADDR(PhyRegNum);
    RegValue &= ~MDIO_STAT_ENC;
  }
  else
    RegValue |= MDIO_STAT_ENC;

  MmioWriteBe32((UINTN)&MdioBusRegs->MdioStat, RegValue);

  /*
   * Wait until the MDIO bus is not busy
   */
  do {
    RegValue = MmioReadBe32((UINTN)&MdioBusRegs->MdioStat);
  } while (RegValue & MDIO_STAT_BSY);

  /*
   * Specify the target PHY and PHY control device:
   */
  RegValue = MDIO_CTL_PORT_ADDR(PhyAddress) |
             MDIO_CTL_DEV_ADDR(MdioCtlDevAddr);
  MmioWriteBe32((UINTN)&MdioBusRegs->MdioCtl, RegValue);

  /*
   * Specify the target PHY register:
   */
  if (TRUE == Clause45)
    MmioWriteBe32((UINTN)&MdioBusRegs->MdioAddr, PhyRegNum & 0xFFFF);

  /*
   * Wait until the MDIO bus is not busy:
   */
  do {
    RegValue = MmioReadBe32((UINTN)&MdioBusRegs->MdioStat);
  } while (RegValue & MDIO_STAT_BSY);

  /*
   * Initiate the MDIO read:
   */
  RegValue = MDIO_CTL_PORT_ADDR(PhyAddress) |
             MDIO_CTL_DEV_ADDR(MdioCtlDevAddr) |
             MDIO_CTL_READ;
  MmioWriteBe32((UINTN)&MdioBusRegs->MdioCtl, RegValue);

  /*
   * Wait until the MDIO read is complete
   */
  do {
    RegValue = MmioReadBe32((UINTN)&MdioBusRegs->MdioData);
  } while (RegValue & MDIO_DATA_BSY);

  RegValue = MmioReadBe32((UINTN)&MdioBusRegs->MdioStat);

  if (RegValue & MDIO_STAT_RD_ER) {
    /*
     * If read failed, return -1:
     */
    DPAA1_ERROR_MSG("MDIO bus read failed for PHY addr 0x%x, dev addr %d, "
                    "reg num 0x%x (MDIO stat reg: 0x%x)\n",
                    PhyAddress, MdioCtlDevAddr, PhyRegNum, RegValue);
    return (UINT16)(-1);
  }
  RegValue = MmioReadBe32((UINTN)&MdioBusRegs->MdioData);
  DPAA1_DEBUG_MSG("MDIO bus read for PHY addr 0x%x, dev addr %d, "
                    "reg num 0x%x (MDIO stat reg: 0x%x)\n",
                    PhyAddress, MdioCtlDevAddr, PhyRegNum, RegValue);

  return (UINT16)RegValue;
}


/**
   Reads the value of a PHY register

   @param[in] Dpaa1Phy          Pointer to PHY object
   @param[in] MdioCtlDevAddr    MDIO control device selector
   @param[in] PhyRegNum         Phy register index

   @retval Value of the PHY register, on success
   @retval -1, on failure

 **/
UINT16
Dpaa1PhyRegisterRead (
  IN DPAA1_PHY *Dpaa1Phy,
  IN INT8 MdioCtlDevAddr,
  IN UINT16 PhyRegNum
  )
{
  ASSERT(Dpaa1Phy->Signature == DPAA1_PHY_SIGNATURE);
  ASSERT(MdioCtlDevAddr == MDIO_PHY_DEV_ADDR ||
         MdioCtlDevAddr == MDIO_CTL_DEV_NONE ||
         MdioCtlDevAddr == MDIO_CTL_DEV_PMAPMD ||
         MdioCtlDevAddr == MDIO_CTL_DEV_AUTO_NEGOTIATION);

  return Dpaa1PhyMdioBusRead(Dpaa1Phy->MdioBus,
                             Dpaa1Phy->PhyAddress,
                             MdioCtlDevAddr,
                             PhyRegNum);
}

/**
   if Auto Negotiaition is supported by PHY;
   restarts the PHY Auto Negotiation for a given DPAA1 MEMAC.

   else sets the various PHY parametes manually.

   @param[in] Memac Pointer to MEMAC

   @retval EFI_SUCCESS, on success
   @retval error code, on failure

 **/

EFI_STATUS
Dpaa1PhyRestartAutoNeg (
  IN  DPAA1_PHY *Dpaa1Phy
  )
{
  UINT16       PhyRegValue;
  UINT32       TimeoutMsCount = PHY_AUTO_NEGOTIATION_TIMEOUT;
  INT8         DevAddr = MDIO_CTL_DEV_NONE;

  DPAA1_DEBUG_MSG("****Restart PHY Auto Negotiation (PHY address: 0x%x) ...\n",
                  Dpaa1Phy->PhyAddress);
  // Currently Only Clause 22 Auto Negotiation Restart is common for all PHY
  ASSERT(Dpaa1Phy->PhyInterfaceType != PHY_INTERFACE_XFI);

  PhyRegValue = Dpaa1PhyRegisterRead(Dpaa1Phy,
                        DevAddr, PHY_STATUS_REG);
  if (PhyRegValue == (UINT16)(-1))
    return EFI_DEVICE_ERROR;

  if (PhyRegValue & PHY_STATUS_AUTONEG_ABILITY)
    Dpaa1Phy->AutoNegotiation = TRUE;
  else
    Dpaa1Phy->AutoNegotiation = FALSE;

  if(TRUE == Dpaa1Phy->AutoNegotiation) {
    /*Auto Negotiaition is supported by PHY. Enable it and restart it*/
    PhyRegValue = Dpaa1PhyRegisterRead(Dpaa1Phy,
                        DevAddr, PHY_CONTROL_REG);
    PhyRegValue |= (PHY_CONTROL_AUTONEG_RESTART|PHY_CONTROL_AUTONEG_ENABLE);
    Dpaa1PhyRegisterWrite(Dpaa1Phy,
                  DevAddr, PHY_CONTROL_REG, PhyRegValue);
    DPAA1_DEBUG_MSG("Waiting for PHY (PHY address: 0x%x) auto negotiation to complete ",
                    Dpaa1Phy->PhyAddress);
    for ( ; ; ) {
      PhyRegValue = Dpaa1PhyRegisterRead(Dpaa1Phy,
                          DevAddr, PHY_STATUS_REG);
      if (PhyRegValue == (UINT16)(-1))
        return EFI_DEVICE_ERROR;

      if (TimeoutMsCount % 500 == 0)
        DPAA1_DEBUG_MSG_NO_PREFIX(".");

      if (PhyRegValue & PHY_STATUS_AUTONEG_COMPLETE)
        break;

      if (TimeoutMsCount == 0) {
        DPAA1_DEBUG_MSG_NO_PREFIX("TIMEOUT!\n");
        DPAA1_ERROR_MSG("PHY auto-negotiation failed\n");
        Dpaa1Phy->AutoNegotiation = FALSE;
        return EFI_TIMEOUT;
      }

      MicroSecondDelay(1000);
      TimeoutMsCount --;
    }
    DPAA1_DEBUG_MSG_NO_PREFIX("\n");
  } else {
    /* To Do : PHY not able to perform Auto Negotiaition *
    * Set the PHY attributes Manully                     */
  }

  return EFI_SUCCESS;
}

/**
   Resets the PHY for a given DPAA1 MEMAC

   @param[in] Memac Pointer to MEMAC

   @retval EFI_SUCCESS, on success
   @retval error code, on failure

 **/

STATIC
EFI_STATUS
Dpaa1PhyReset (
  IN  DPAA1_PHY *Dpaa1Phy
  )
{
  UINT16 PhyRegValue;
  UINT32 TimeoutMsCount = 500;
  INT8   DevAddr = MDIO_CTL_DEV_NONE;

  if (Dpaa1Phy->Flags & PHY_BROKEN_RESET) {
    return EFI_SUCCESS;
  }

  DPAA1_DEBUG_MSG("****Resetting PHY (PHY address: 0x%x) ...\n",
                  Dpaa1Phy->PhyAddress);

  if(Dpaa1Phy->PhyInterfaceType == PHY_INTERFACE_XFI)
    DevAddr = MDIO_PHY_DEV_ADDR;
  else
    DevAddr = MDIO_CTL_DEV_NONE;

  PhyRegValue = Dpaa1PhyRegisterRead(Dpaa1Phy,
                        DevAddr, PHY_CONTROL_REG);
  if (PhyRegValue == (UINT16)(-1)) {
    return EFI_DEVICE_ERROR;
  }

  PhyRegValue |= PHY_CONTROL_RESET;
  Dpaa1PhyRegisterWrite(Dpaa1Phy,
                DevAddr, PHY_CONTROL_REG, PhyRegValue);

  /*
   * Poll the control register for the reset bit to go to 0 (it is
   * auto-clearing). This should happen within 0.5 seconds per the
   * IEEE spec.
   */
  for ( ; ; ) {
    PhyRegValue = Dpaa1PhyRegisterRead(Dpaa1Phy,
                        DevAddr, PHY_CONTROL_REG);
    if (PhyRegValue == (UINT16)(-1)) {
      return EFI_DEVICE_ERROR;
    }

    if ((PhyRegValue & PHY_CONTROL_RESET) == 0) {
      break;
    }

    if (TimeoutMsCount == 0) {
      DPAA1_ERROR_MSG("Timeout resetting PHY (PHY address: 0x%x)\n",
                      Dpaa1Phy->PhyAddress);
      return EFI_NOT_READY;
    }

    MicroSecondDelay(1000);
    TimeoutMsCount --;
  }

  return EFI_SUCCESS;
}

/**
   Configures the PHY for a given DPAA1 MEMAC

   @param[in] Memac Pointer to MEMAC

   @retval EFI_SUCCESS, on success
   @retval error code, on failure

 **/
STATIC
EFI_STATUS
Dpaa1PhyConfig (
  IN  DPAA1_PHY *Dpaa1Phy
  )
{
  DPAA1_DEBUG_MSG("Configuring PHY (PHY address: 0x%x) ...\n",
                  Dpaa1Phy->PhyAddress);

  if (NULL != Dpaa1Phy->Dpaa1PhyConfig)
    return Dpaa1Phy->Dpaa1PhyConfig(Dpaa1Phy);
  else
    DPAA1_INFO_MSG("PhyIdentifier 0x%08x configuration is not supported yet",
                                  Dpaa1Phy->PhyIdentifier);

  return EFI_UNSUPPORTED;
}

STATIC
EFI_STATUS
Dpaa1PhyIdentify (
  IN  DPAA1_PHY *Dpaa1Phy
  )
{
  INT8   DevAddr = MDIO_CTL_DEV_NONE;

  if(Dpaa1Phy->PhyInterfaceType == PHY_INTERFACE_XFI)
    DevAddr = MDIO_CTL_DEV_PMAPMD;
  else
    DevAddr = MDIO_CTL_DEV_NONE;

  Dpaa1Phy->PhyIdentifier = \
     (Dpaa1PhyRegisterRead(Dpaa1Phy,DevAddr, PHY_IDENTIFIER_HIGH_WORD) << 16) |
      Dpaa1PhyRegisterRead(Dpaa1Phy,DevAddr, PHY_IDENTIFIER_LOW_WORD);

  DPAA1_DEBUG_MSG("%a PhyIdentifier = 0x%08x\n",
                    __FUNCTION__, Dpaa1Phy->PhyIdentifier);
  /* The PHY Identifier shall be composed of the third through 24th bits
   * of the Organizationally Unique Identifier (OUI) assigned to the PHY
   * manufacturer by the IEEE,1 plus a six-bit manufacturer model
   * number, plus a four-bit manufacturer revision number.*/

  switch((Dpaa1Phy->PhyIdentifier & PHY_IDENTIFIER_OUI_MASK)>>0xA) // Extract OUI
  {
    case REALTEK_OUI:
                 Dpaa1Phy->Dpaa1PhyStartup = RealtekPhyStartup;
                 Dpaa1Phy->Dpaa1PhyConfig = RealtekPhyConfig;
                 break;
    case AQUANTIA_OUI:
                 Dpaa1Phy->Dpaa1PhyStartup = AquantiaPhyStartup;
                 Dpaa1Phy->Dpaa1PhyConfig = AquantiaPhyConfig;
                 break;
    case VITESSE_OUI:
                 Dpaa1Phy->Dpaa1PhyStartup = VitessePhyStartup;
                 Dpaa1Phy->Dpaa1PhyConfig = VitessePhyConfig;
                 break;
    default:
           DPAA1_ERROR_MSG("PhyIdentifier 0x%08x is not supported yet\n",
                                        Dpaa1Phy->PhyIdentifier);
           break;
  }

  if(NULL != Dpaa1Phy->Dpaa1PhyStartup)
    return EFI_SUCCESS;
  else
    return EFI_DEVICE_ERROR;
}
/**
   Initializes the PHY for a given DPAA1 MEMAC

   @param[in] Memac Pointer to MEMAC

   @retval EFI_SUCCESS, on success
   @retval error code, on failure

 **/
EFI_STATUS
Dpaa1PhyInit (
  IN  DPAA1_PHY *Dpaa1Phy
  )
{
  EFI_STATUS Status;

  DPAA1_DEBUG_MSG("Initializing PHY (PHY type: %d, address: 0x%x) ...\n",
                    Dpaa1Phy->PhyInterfaceType, Dpaa1Phy->PhyAddress);

  Dpaa1Phy->Flags = 0;

  Status = Dpaa1PhyIdentify(Dpaa1Phy);
  if (EFI_ERROR(Status))
    return Status;

  Status = Dpaa1PhyReset(Dpaa1Phy);
  if (EFI_ERROR(Status))
    return Status;

  Status = Dpaa1PhyConfig(Dpaa1Phy);
  if (EFI_ERROR(Status))
    return Status;

  return EFI_SUCCESS;
}

/**
   Start the PHY for a given DPAA1 MEMAC

   @param[in] Memac Pointer to MEMAC

   @retval EFI_SUCCESS, on success
   @retval error code, on failure

 **/
EFI_STATUS
Dpaa1PhyStartup (
  IN  DPAA1_PHY *Dpaa1Phy
  )
{
  DPAA1_DEBUG_MSG("Starting up PHY (PHY address: 0x%x) ...\n",
                 Dpaa1Phy->PhyAddress);

  if (NULL != Dpaa1Phy->Dpaa1PhyStartup)
    return Dpaa1Phy->Dpaa1PhyStartup(Dpaa1Phy);
  else
    DPAA1_INFO_MSG("PhyIdentifier 0x%08x Startup is not supported yet\n",
                                Dpaa1Phy->PhyIdentifier);

  return EFI_SUCCESS;
}

/**
   Stop the PHY for a given DPAA1 MEMAC

   @param[in] Memac Pointer to MEMAC

 **/
VOID
Dpaa1PhyShutdown (
  IN  DPAA1_PHY *Dpaa1Phy
  )
{
  /*
   * For the Aquantia PHY there is no shutdown
   */
}

/**
   Get status of PHY for a given DPAA1 MEMAC

   @param[in] Memac Pointer to MEMAC

   @retval 	TRUE if link is up
		FALSE if link is down 
 **/
BOOLEAN
Dpaa1PhyStatus (
  IN  DPAA1_PHY *Dpaa1Phy
  )
{
  UINT16 PhyRegValue;
  INT8   DevAddr = MDIO_CTL_DEV_NONE;

  if(Dpaa1Phy->PhyInterfaceType == PHY_INTERFACE_XFI)
    DevAddr = MDIO_CTL_DEV_AUTO_NEGOTIATION;
  else
    DevAddr = MDIO_CTL_DEV_NONE;

  /*
  * Read twice because link state is latched and a
  * read moves the current state into the register
  */
  (VOID)Dpaa1PhyRegisterRead(Dpaa1Phy,
                             DevAddr,
                             PHY_STATUS_REG);
  PhyRegValue = Dpaa1PhyRegisterRead(Dpaa1Phy,
                                     DevAddr,
                                     PHY_STATUS_REG);

  if (PhyRegValue == (UINT16)-1 ||
      !(PhyRegValue & PHY_STATUS_LINK_STATUS)) {
    Dpaa1Phy->LinkUp = FALSE;
  } else {
    Dpaa1Phy->LinkUp = TRUE;
  }
  return Dpaa1Phy->LinkUp;

}

VOID
DtsecInitPhy (
    IN  DPAA1_PHY_MDIO_BUS *MdioBus,
    IN  DPAA1_PHY *Dpaa1Phy
    )
{
  BOOLEAN Sgmii2500;
  UINT32 Value;

  Sgmii2500 = (Dpaa1Phy->PhyInterfaceType == PHY_INTERFACE_SGMII_2500) ? TRUE : FALSE;
  Value = PHY_SGMII_IF_MODE_SGMII | PHY_SGMII_IF_MODE_AN;

  if (Sgmii2500)
    Value = PHY_SGMII_CR_PHY_RESET | PHY_SGMII_IF_SPEED_GIGABIT | PHY_SGMII_IF_MODE_SGMII;

  Dpaa1PhyMdioBusWrite(MdioBus, 0, MDIO_CTL_DEV_NONE, SGMII_IF_MODE, Value);

  Value = PHY_SGMII_DEV_ABILITY_SGMII;
  Dpaa1PhyMdioBusWrite(MdioBus, 0, MDIO_CTL_DEV_NONE, SGMII_DEV_ABIL, Value);

  if (Sgmii2500) {
   /* Adjust link timer for 2.5G SGMII,
    * 1.6 ms in units of 3.2 ns:
    * 1.6ms / 3.2ns = 5 * 10^5 = 0x7a120.
    */
    Dpaa1PhyMdioBusWrite(MdioBus, 0, MDIO_CTL_DEV_NONE, SGMII_LINK_TMR_H, 0x0007);
    Dpaa1PhyMdioBusWrite(MdioBus, 0, MDIO_CTL_DEV_NONE, SGMII_LINK_TMR_L, 0xa120);
  } else {
    /* Adjust link timer for SGMII,
     * 1.6 ms in units of 8 ns
     * 1.6ms / 8ns = 2 * 10^5 = 0x30d40.
     */
    Dpaa1PhyMdioBusWrite(MdioBus, 0, MDIO_CTL_DEV_NONE, SGMII_LINK_TMR_H, 0x0003);
    Dpaa1PhyMdioBusWrite(MdioBus, 0, MDIO_CTL_DEV_NONE, SGMII_LINK_TMR_L, 0x0d40);
  }
  Value = PHY_SGMII_CR_DEF_VAL | PHY_SGMII_CR_RESET_AN;
  Dpaa1PhyMdioBusWrite(MdioBus, 0, MDIO_CTL_DEV_NONE, 0x0, Value);
}
