/** PhyCommon.c
  DPAA2 Ethernet PHY common services implementation

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

#define MDIO_CLOCK_DIVIDER  258

STATIC CONST CHAR8 *CONST gPhyMediaTypeStrings[] = {
  [COPPER_PHY] = "Copper PHY",
  [OPTICAL_PHY] = "Optical PHY",
};

#define IS_SEPARATOR(_Char) \
  ((_Char) == ' ' || (_Char) == '\t' || (_Char) == '\n')

CHAR8 *
SkipSeparators (
  CHAR8 *Str
  )
{
  CHAR8 *Ptr;

  for (Ptr = Str; *Ptr != '\0'; Ptr ++) {
    if (!IS_SEPARATOR (*Ptr)) {
        break;
    }
  }

  return Ptr;
}

CHAR8 *
FindNextSeparator (
  CHAR8 *Str
  )
{
  CHAR8 *Ptr;

  for (Ptr = Str; *Ptr != '\0'; Ptr ++) {
    if (IS_SEPARATOR (*Ptr)) {
        break;
    }
  }

  return Ptr;
}


/**
   Initializes the given DPAA2 PHY MDIO bus

   @param[in] MdioBus   Pointer to MDIO bus object

   @retval EFI_SUCCESS

 **/
STATIC EFI_STATUS
Dpaa2PhyMdioBusInit (
  DPAA2_PHY_MDIO_BUS *MdioBus
  )
{
  MEMAC_MDIO_BUS_REGS *CONST MdioBusRegs = MdioBus->IoRegs;
  UINT32 RegValue;

  RegValue = MmioRead32 ((UINTN)&MdioBusRegs->MdioStat);
  RegValue |= MDIO_STAT_CLKDIV (MDIO_CLOCK_DIVIDER) | MDIO_STAT_NEG;
  MmioWrite32 ((UINTN)&MdioBusRegs->MdioStat, RegValue);
  return EFI_SUCCESS;
}


/**
   Initializes the MDIO buses used by DPAA2 DPMACs to reach their
   corresponding PHYs.

   @param[in] MdioBuses         Pointer to array of MDIO buses to initialize
   @param[in] MdioBusesCount    Number of entries in MdioBuses[]

   @retval EFI_SUCCESS, on success
   @retval error code, on failure

 **/
EFI_STATUS
Dpaa2PhyMdioBusesInit (
  DPAA2_PHY_MDIO_BUS MdioBuses[],
  UINT8              MdioBusesCount
  )
{
  EFI_STATUS Status;
  UINT8 I;

  for (I = 0; I < MdioBusesCount; I ++) {
    Status = Dpaa2PhyMdioBusInit (&MdioBuses[I]);
    if (EFI_ERROR (Status)) {
      return Status;
    }
  }

  return EFI_SUCCESS;
}


STATIC
VOID
Dpaa2PhyMdioBusWrite (
  IN DPAA2_PHY_MDIO_BUS *MdioBus,
  IN UINT8              PhyAddress,
  IN INT8               MdioCtlDevAddr,
  IN UINT16             PhyRegNum,
  IN UINT16             Value
  )
{
  UINT32 RegValue;
  BOOLEAN Clause45;

  Clause45 = TRUE; // Clause 45 is used for 10 GigPHYs and MACs
  MEMAC_MDIO_BUS_REGS *CONST MdioBusRegs = MdioBus->IoRegs;

  DPAA_DEBUG_MSG("MDIO bus WRITE for PHY addr 0x%x, dev addr %d, "
                   "reg num 0x%x (MDIO stat reg: 0x%x)\n",
                   PhyAddress, MdioCtlDevAddr, PhyRegNum, Value);

  ASSERT (MdioBus->Signature == DPAA2_PHY_MDIO_BUS_SIGNATURE);

  RegValue = MmioRead32 ((UINTN)&MdioBusRegs->MdioStat);

  if (MDIO_CTL_DEVAD_NONE == MdioCtlDevAddr) {
    Clause45 = FALSE; // Clause 22
    MdioCtlDevAddr = MDIO_CTL_DEV_ADDR (PhyRegNum);
    RegValue &= ~MDIO_STAT_ENC;
  } else {
    RegValue |= MDIO_STAT_ENC;
  }

  MmioWrite32 ((UINTN)&MdioBusRegs->MdioStat, RegValue);

  /*
   * Wait until the MDIO bus is not busy
   */
  do {
    RegValue = MmioRead32 ((UINTN)&MdioBusRegs->MdioStat);
  } while (RegValue & MDIO_STAT_BSY);

  /*
   * Specify the target PHY and PHY control device:
   */
  RegValue = MDIO_CTL_PORT_ADDR (PhyAddress) |
             MDIO_CTL_DEV_ADDR (MdioCtlDevAddr);
  MmioWrite32 ((UINTN)&MdioBusRegs->MdioCtl, RegValue);

  /*
   * Specify the target PHY register:
   */
  if (TRUE == Clause45) {
    MmioWrite32 ((UINTN)&MdioBusRegs->MdioAddr, PhyRegNum);
  }

  /*
   * Wait until the MDIO bus is not busy:
   */
  do {
    RegValue = MmioRead32 ((UINTN)&MdioBusRegs->MdioStat);
  } while (RegValue & MDIO_STAT_BSY);

  /*
   * Write the value to the PHY register
   */
  MmioWrite32 ((UINTN)&MdioBusRegs->MdioData, MDIO_DATA (Value));

  /*
   * Wait until the MDIO write is complete
   */
  do {
    RegValue = MmioRead32 ((UINTN)&MdioBusRegs->MdioData);
  } while (RegValue & MDIO_DATA_BSY);
}


/**
   Writes a value to a PHY register

   @param[in] Dpaa2Phy          Pointer to PHY object
   @param[in] MdioCtlDevAddr    MDIO control device selector
   @param[in] PhyRegNum         Phy register index
   @param[in] Value             Value to write to the PHY register

   @retval EFI_SUCCESS, on success
   @retval error code, on failure

 **/
VOID
Dpaa2PhyRegisterWrite (
  IN DPAA2_PHY *Dpaa2Phy,
  IN INT8      MdioCtlDevAddr,
  IN UINT16    PhyRegNum,
  IN UINT16    Value
  )
{
  ASSERT (Dpaa2Phy->Signature == DPAA2_PHY_SIGNATURE);
  ASSERT (MdioCtlDevAddr == MDIO_PHY_DEV_ADDR ||
         MdioCtlDevAddr == MDIO_CTL_DEVAD_NONE ||
         MdioCtlDevAddr == MDIO_CTL_DEV_PMAPMD ||
         MdioCtlDevAddr == MDIO_CTL_DEV_AUTO_NEGOTIATION ||
         MdioCtlDevAddr == MDIO_MMD_VEND1 ||
         MdioCtlDevAddr == MDIO_MMD_PHYXS);

  Dpaa2PhyMdioBusWrite (Dpaa2Phy->MdioBus,
                       Dpaa2Phy->PhyAddress,
                       MdioCtlDevAddr,
                       PhyRegNum,
                       Value);
}


STATIC
UINT16
Dpaa2PhyMdioBusRead (
  IN DPAA2_PHY_MDIO_BUS *MdioBus,
  IN UINT8              PhyAddress,
  IN INT8               MdioCtlDevAddr,
  IN UINT16             PhyRegNum
  )
{
  UINT32 RegValue;
  MEMAC_MDIO_BUS_REGS *CONST MdioBusRegs = MdioBus->IoRegs;
  BOOLEAN Clause45;

  Clause45 = TRUE; // Clause 45 is used for 10 GigPHYs and MACs

  ASSERT (MdioBus->Signature == DPAA2_PHY_MDIO_BUS_SIGNATURE);

  RegValue = MmioRead32 ((UINTN)&MdioBusRegs->MdioStat);
  if (MdioCtlDevAddr == MDIO_CTL_DEVAD_NONE) {
    Clause45 = FALSE; // Clause 22
    MdioCtlDevAddr = MDIO_CTL_DEV_ADDR (PhyRegNum);
    RegValue &= ~MDIO_STAT_ENC;
  } else {
    RegValue |= MDIO_STAT_ENC;
  }

  MmioWrite32 ((UINTN)&MdioBusRegs->MdioStat, RegValue);

  /*
   * Wait until the MDIO bus is not busy
   */
  do {
    RegValue = MmioRead32 ((UINTN)&MdioBusRegs->MdioStat);
  } while (RegValue & MDIO_STAT_BSY);

  /*
   * Specify the target PHY and PHY control device:
   */
  RegValue = MDIO_CTL_PORT_ADDR (PhyAddress) |
             MDIO_CTL_DEV_ADDR (MdioCtlDevAddr);
  MmioWrite32 ((UINTN)&MdioBusRegs->MdioCtl, RegValue);

  /*
   * Specify the target PHY register:
   */
  if (TRUE == Clause45) {
    MmioWrite32 ((UINTN)&MdioBusRegs->MdioAddr, PhyRegNum);
  }

  /*
   * Wait until the MDIO bus is not busy:
   */
  do {
    RegValue = MmioRead32 ((UINTN)&MdioBusRegs->MdioStat);
  } while (RegValue & MDIO_STAT_BSY);


  /*
   * Initiate the MDIO read:
   */
  RegValue = MDIO_CTL_PORT_ADDR (PhyAddress) |
             MDIO_CTL_DEV_ADDR (MdioCtlDevAddr) |
             MDIO_CTL_READ;
  MmioWrite32 ((UINTN)&MdioBusRegs->MdioCtl, RegValue);

  /*
   * Wait until the MDIO read is complete
   */
  do {
    RegValue = MmioRead32 ((UINTN)&MdioBusRegs->MdioData);
  } while (RegValue & MDIO_DATA_BSY);

  RegValue = MmioRead32 ((UINTN)&MdioBusRegs->MdioStat);

  if (RegValue & MDIO_STAT_RD_ER) {
    /*
     * If read failed, return -1:
     */
    DPAA_ERROR_MSG ("MDIO bus read failed for PHY addr 0x%x, dev addr 0x%x, "
                    "reg num 0x%x (MDIO stat reg: 0x%x)\n",
                    PhyAddress, MdioCtlDevAddr, PhyRegNum, RegValue);
    return (UINT16)-1;
  }

  RegValue = MmioRead32 ((UINTN)&MdioBusRegs->MdioData);

  DPAA_DEBUG_MSG("MDIO bus read for PHY addr 0x%x, dev addr %d, "
                    "reg num 0x%x (MDIO stat reg: 0x%x)\n",
                    PhyAddress, MdioCtlDevAddr, PhyRegNum, RegValue);

  return (UINT16)RegValue;
}


/**
   Reads the value of a PHY register

   @param[in] Dpaa2Phy          Pointer to PHY object
   @param[in] MdioCtlDevAddr    MDIO control device selector
   @param[in] PhyRegNum         Phy register index

   @retval Value of the PHY register, on success
   @retval -1, on failure

 **/
UINT16
Dpaa2PhyRegisterRead (
  IN DPAA2_PHY *Dpaa2Phy,
  IN INT8      MdioCtlDevAddr,
  IN UINT16    PhyRegNum
  )
{
  ASSERT (Dpaa2Phy->Signature == DPAA2_PHY_SIGNATURE);
  ASSERT (MdioCtlDevAddr == MDIO_PHY_DEV_ADDR ||
         MdioCtlDevAddr == MDIO_CTL_DEVAD_NONE ||
         MdioCtlDevAddr == MDIO_CTL_DEV_PMAPMD ||
         MdioCtlDevAddr == MDIO_CTL_DEV_AUTO_NEGOTIATION ||
         MdioCtlDevAddr == MDIO_MMD_VEND1 ||
         MdioCtlDevAddr == MDIO_MMD_PHYXS);

  return Dpaa2PhyMdioBusRead (Dpaa2Phy->MdioBus,
                             Dpaa2Phy->PhyAddress,
                             MdioCtlDevAddr,
                             PhyRegNum);
}


/**
   Resets the PHY for a given DPAA2 DPMAC

   @param[in] Dpmac Pointer to DPMAC

   @retval EFI_SUCCESS, on success
   @retval error code, on failure

 **/
STATIC
EFI_STATUS
Dpaa2PhyReset (
  DPAA2_PHY *Dpaa2Phy
  )
{
  UINT16 PhyRegValue;
  UINT32 TimeoutMsCount;
  INT8   DevAddr;

  TimeoutMsCount = 500;

  if (Dpaa2Phy->Flags & PHY_BROKEN_RESET) {
    return EFI_SUCCESS;
  }

  DPAA_DEBUG_MSG ("Resetting PHY (PHY address: 0x%x) ...\n",
                  Dpaa2Phy->PhyAddress);

  if (Dpaa2Phy->PhyInterfaceType == PHY_INTERFACE_XGMII) {
    DevAddr = MDIO_PHY_DEV_ADDR;
  } else {
    DevAddr = MDIO_CTL_DEVAD_NONE;
  }

  PhyRegValue = Dpaa2PhyRegisterRead (Dpaa2Phy, DevAddr, PHY_CONTROL_REG);
  if (PhyRegValue == (UINT16)-1) {
    return EFI_DEVICE_ERROR;
  }

  PhyRegValue |= PHY_CONTROL_RESET;
  Dpaa2PhyRegisterWrite (Dpaa2Phy, DevAddr, PHY_CONTROL_REG, PhyRegValue);

  /*
   * Poll the control register for the reset bit to go to 0 (it is
   * auto-clearing). This should happen within 0.5 seconds per the
   * IEEE spec.
   */
  while (1) {
    PhyRegValue = Dpaa2PhyRegisterRead (Dpaa2Phy, DevAddr, PHY_CONTROL_REG);
    if (PhyRegValue == (UINT16)-1) {
      return EFI_DEVICE_ERROR;
    }

    if ((PhyRegValue & PHY_CONTROL_RESET) == 0) {
      break;
    }

    if (TimeoutMsCount == 0) {
      DPAA_ERROR_MSG ("Timeout resetting PHY (PHY address: 0x%x)\n",
                      Dpaa2Phy->PhyAddress);
      return EFI_NOT_READY;
    }

    MicroSecondDelay (1000);
    TimeoutMsCount --;
  }

  return EFI_SUCCESS;
}


/**
   Configures the PHY for a given DPAA2 DPMAC

   @param[in] Dpmac Pointer to DPMAC

   @retval EFI_SUCCESS, on success
   @retval error code, on failure

 **/
STATIC
EFI_STATUS
Dpaa2PhyConfig (
  DPAA2_PHY *Dpaa2Phy
  )
{
  DPAA_DEBUG_MSG ("Configuring PHY (PHY address: 0x%x) ... Media %d \n",
                  Dpaa2Phy->PhyAddress, Dpaa2Phy->PhyMediaType);

  if (Dpaa2Phy->PhyId == QC_PHY) {
    return Ar8035PhyConfig( Dpaa2Phy);
  } else if (Dpaa2Phy->PhyMediaType == COPPER_PHY) {
    return AquantiaPhyConfig (Dpaa2Phy);
  } else if (Dpaa2Phy->PhyMediaType == OPTICAL_PHY) {
    if (Dpaa2Phy->PhyAddress == INPHI_PHY_ADDR) {
      return In112525S03PhyConfig (Dpaa2Phy);
    } else {
      return CortinaPhyConfig (Dpaa2Phy);
    }
  } else {
    DPAA_ERROR_MSG ("PHY media type not supported: 0x%x\n",
                    Dpaa2Phy->PhyMediaType);
    return EFI_UNSUPPORTED;
  }
}

/**
   Initializes the PHY for a given DPAA2 DPMAC

   @param[in] Dpmac Pointer to DPMAC

   @retval EFI_SUCCESS, on success
   @retval error code, on failure

 **/
EFI_STATUS
Dpaa2PhyInit (
  DPAA2_PHY *Dpaa2Phy
  )
{
  EFI_STATUS Status;

  DPAA_DEBUG_MSG ("Initializing PHY (PHY address: 0x%x) ...\n",
                  Dpaa2Phy->PhyAddress);

  if (Dpaa2Phy->PhyMediaType == OPTICAL_PHY) {
    Dpaa2Phy->Flags = PHY_BROKEN_RESET;
  } else {
    Dpaa2Phy->Flags = 0;
  }

  Status = Dpaa2PhyReset (Dpaa2Phy);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  Status = Dpaa2PhyConfig (Dpaa2Phy);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  return EFI_SUCCESS;
}


/**
   Start the PHY for a given DPAA2 DPMAC

   @param[in] Dpmac Pointer to DPMAC

   @retval EFI_SUCCESS, on success
   @retval error code, on failure

 **/
EFI_STATUS
Dpaa2PhyStartup (
  DPAA2_PHY *Dpaa2Phy
  )
{
  DPAA_INFO_MSG ("Starting up PHY (PHY address: 0x%x) %x ...\n",
                 Dpaa2Phy->PhyAddress, Dpaa2Phy);

  if (Dpaa2Phy->PhyId == QC_PHY) {
    return Ar8035PhyStartup (Dpaa2Phy);
  } else if (Dpaa2Phy->PhyMediaType == COPPER_PHY) {
    return AquantiaPhyStartup (Dpaa2Phy);
  } else if (Dpaa2Phy->PhyMediaType == OPTICAL_PHY) {
    return In112525S03PhyStartup (Dpaa2Phy);
  } else {
    DPAA_ERROR_MSG ("PHY media type not supported: 0x%x (%a)\n",
                    Dpaa2Phy->PhyMediaType,
                    gPhyMediaTypeStrings[Dpaa2Phy->PhyMediaType]);
    return EFI_UNSUPPORTED;
  }
}


/**
   Stop the PHY for a given DPAA2 DPMAC

   @param[in] Dpmac Pointer to DPMAC

 **/
VOID
Dpaa2PhyShutdown (
  DPAA2_PHY *Dpaa2Phy
  )
{
  /*
   * Shutdown not supported currently
   */
}

UINT32
Dpaa2PhyReadMmdIndirect (
  IN  DPAA2_PHY *Dpaa2Phy,
  IN  UINT32    PortAddr,
  IN  UINT32    DevAddr,
  IN  INT8      Addr
  )
{
  /* Write the desired MMD Devad */
  Dpaa2PhyRegisterWrite (Dpaa2Phy, Addr, MII_MMD_CTRL, DevAddr);

  /* Write the desired MMD register Address */
  Dpaa2PhyRegisterWrite (Dpaa2Phy, Addr, MII_MMD_DATA, PortAddr);

  /* Select the Function : DATA with no post increment */
  Dpaa2PhyRegisterWrite (Dpaa2Phy, Addr, MII_MMD_CTRL, (DevAddr | MII_MMD_CTRL_NOINCR));

  /* Read the content of the MMD's selected register */
  return Dpaa2PhyRegisterRead (Dpaa2Phy, Addr, MII_MMD_DATA);
}

VOID
Dpaa2PhyWriteMmdIndirect (
  IN  DPAA2_PHY *Dpaa2Phy,
  IN  UINT32    PortAddr,
  IN  UINT32    DevAddr,
  IN  INT8      Addr,
  IN  UINT32    Data
  )
{
  /* Write the desired MMD Devad */
  Dpaa2PhyRegisterWrite (Dpaa2Phy, Addr, MII_MMD_CTRL, DevAddr);

  /* Write the desired MMD register Address */
  Dpaa2PhyRegisterWrite (Dpaa2Phy, Addr, MII_MMD_DATA, PortAddr);

  /* Select the Function : DATA with no post increment */
  Dpaa2PhyRegisterWrite (Dpaa2Phy, Addr, MII_MMD_CTRL, (DevAddr | MII_MMD_CTRL_NOINCR));

  /* Write the data into MMD's selected register */
  Dpaa2PhyRegisterWrite (Dpaa2Phy, Addr, MII_MMD_DATA, Data);
}

/**
  Function to update the speed and duplex of phy.
  If autonegotiation is enabled, it uses the AND of the link
  partner's advertised capabilities and our advertised
  capabilities.  If autonegotiation is disabled, we use the
  appropriate bits in the control register.

 **/
VOID
ParsePhyLink (
  DPAA2_PHY *Dpaa2Phy
  )
{
  UINT32    PhyRegValue;
  INT32     GigLpa;
  UINT32    Lpa;
  UINT32    ExtendedStatus;
  UINT32    Bmcr;

  ExtendedStatus = 0;

  /* Set the default values, set them if they're different */
  Dpaa2Phy->Speed = 10;
  Dpaa2Phy->FullDuplex = FALSE;

  PhyRegValue = Dpaa2PhyRegisterRead (Dpaa2Phy, MDIO_CTL_DEVAD_NONE, MII_BMSR);

  /* We're using autonegotiation */
  if (Dpaa2Phy->AutoNegotiation == TRUE) {
    GigLpa = 0;

    /* Check for gigabit capability */
    if (Dpaa2Phy->Support &
        (SUPPORT_1000BaseT_FULL | SUPPORT_1000BaseT_HALF)) {
      /* We want a list of states supported by both PHYs in the link */
      GigLpa = Dpaa2PhyRegisterRead (Dpaa2Phy, MDIO_CTL_DEVAD_NONE, MII_STAT1000);
      if (GigLpa < 0) {
        DPAA_ERROR_MSG ("Cant read 1000 BaseT Status, Ignoring gigabit capability\n");
        GigLpa = 0;
      }
      GigLpa &= (Dpaa2PhyRegisterRead (Dpaa2Phy, MDIO_CTL_DEVAD_NONE, MII_CTRL1000) << 2);
    }

    /* Check the gigabit fields */
    if (GigLpa & (PHY_1000BTSR_1000FD | PHY_1000BTSR_1000HD)) {
      Dpaa2Phy->Speed = 1000;

      if (GigLpa & PHY_1000BTSR_1000FD) {
        Dpaa2Phy->FullDuplex = TRUE;
      }

      return;
    }

    Lpa = Dpaa2PhyRegisterRead (Dpaa2Phy, MDIO_CTL_DEVAD_NONE, MII_ADVERTISE);
    Lpa &= Dpaa2PhyRegisterRead (Dpaa2Phy, MDIO_CTL_DEVAD_NONE, MII_LPA);

    if (Lpa & (PHY_LPA_100FULL | PHY_LPA_100HALF)) {
      Dpaa2Phy->Speed = 100;

      if (Lpa & PHY_LPA_100FULL) {
        Dpaa2Phy->FullDuplex = TRUE;
      }
    } else if (Lpa & PHY_LPA_10FULL) {
      Dpaa2Phy->FullDuplex = TRUE;
    }

    /*
     * Extended status may indicate that the PHY supports 1000BASE-T/X even
     * though the 1000BASE-T registers are missing.
     * In this case we can't tell whether the peer also supports it, so we
     * only check extended status if 1000BASE-T registers are missing.
     */
    if ((PhyRegValue & PHY_BMSR_ESTATEN) && !(PhyRegValue & PHY_BMSR_ERCAP)) {
      ExtendedStatus = Dpaa2PhyRegisterRead (Dpaa2Phy, MDIO_CTL_DEVAD_NONE, MII_ESTATUS);
    }

    if (ExtendedStatus &
        (PHY_ESTATUS_1000_XFULL | PHY_ESTATUS_1000_XHALF |
         PHY_ESTATUS_1000_TFULL | PHY_ESTATUS_1000_THALF)) {

      Dpaa2Phy->Speed = 1000;

      if (ExtendedStatus & (PHY_ESTATUS_1000_XFULL | PHY_ESTATUS_1000_TFULL)) {
        Dpaa2Phy->FullDuplex = TRUE;
      }
    }
  } else {
    Bmcr = Dpaa2PhyRegisterRead (Dpaa2Phy, MDIO_CTL_DEVAD_NONE, MII_BMCR);

    if (Bmcr & PHY_BMCR_FULLDPLX) {
      Dpaa2Phy->FullDuplex = TRUE;
    }

    if (Bmcr & PHY_BMCR_SPEED1000) {
      Dpaa2Phy->Speed = 1000;
    } else if (Bmcr & PHY_BMCR_SPEED100) {
      Dpaa2Phy->Speed = 100;
    }
  }

  return;
}

/**
  UpdatePhyLink - Update the value in Dpaa2Phy->Link to reflect the
  current link value.  In order to do this, read the status register twice,
  keeping the second value.

  @Dpaa2Phy            : target DPAA2_PHY structure

  @return EFI_TIMEOUT  : Autnegotiation Timed Out.
  @return EFI_SUCCESS  : Phy Link UPdated Successfully.

 **/
EFI_STATUS
UpdatePhyLink (
  DPAA2_PHY *Dpaa2Phy
  )
{
  UINTN I;
  UINT32    PhyRegValue;

  Dpaa2Phy->AutoNegotiation = TRUE;

  /*
   * If the auto-negotiation is still in progress, wait:
   */
  PhyRegValue = Dpaa2PhyRegisterRead (Dpaa2Phy, MDIO_CTL_DEVAD_NONE, MII_BMSR);

  /*
   * If  link is up then no need to wait for autoneg again
   */
  if (Dpaa2Phy->LinkUp && PhyRegValue & PHY_BMSR_LSTATUS) {
    return EFI_SUCCESS;
  }

  if ((Dpaa2Phy->AutoNegotiation) && !(PhyRegValue & PHY_BMSR_ANEGCOMPLETE)) {
    DPAA_DEBUG_MSG ("Waiting for PHY (PHY address: 0x%x) auto negotiation to complete ",
                    Dpaa2Phy->PhyAddress);

    for (I = 0; I < PHY_AUTO_NEGOTIATION_TIMEOUT; I ++) {
      MicroSecondDelay (1000);
      PhyRegValue = Dpaa2PhyRegisterRead (Dpaa2Phy,
                                         MDIO_CTL_DEVAD_NONE,
                                         MII_BMSR);
      if (I % 500 == 0) {
        DPAA_DEBUG_MSG_NO_PREFIX (".");
      }

      if (PhyRegValue & PHY_BMSR_ANEGCOMPLETE) {
         break;
      }
    }

    if (I == PHY_AUTO_NEGOTIATION_TIMEOUT) {
      DPAA_DEBUG_MSG_NO_PREFIX ("TIMEOUT!\n");
      DPAA_ERROR_MSG ("PHY auto-negotiation failed\n");
      Dpaa2Phy->AutoNegotiation = FALSE;
      Dpaa2Phy->LinkUp = FALSE;

      return EFI_TIMEOUT;
    }

    Dpaa2Phy->LinkUp = TRUE;
  } else {
    /* Read the link a second time to clear the latched state */
    PhyRegValue = Dpaa2PhyRegisterRead (Dpaa2Phy, MDIO_CTL_DEVAD_NONE, MII_BMSR);

    if (PhyRegValue & PHY_BMSR_LSTATUS) {
      Dpaa2Phy->LinkUp = TRUE;
    } else {
      Dpaa2Phy->LinkUp = FALSE;
    }
  }

  DPAA_DEBUG_MSG ("PHY link is %a\n", Dpaa2Phy->LinkUp ? "up" : "down");

  return EFI_SUCCESS;
}

/**
 * ConfigAdvertising - sanitize and advertise auto-negotation parameters
 *
 * @Dpaa2Phy         : target DPAA2_PHY struct
 *
 * Description       : Writes MII_ADVERTISE with the appropriate values,
 **/
STATIC
BOOLEAN
ConfigAdvertising (
  DPAA2_PHY *Dpaa2Phy
  )
{
  UINT32    PhyAdvertise;
  UINT32    OrigAdvertise;
  UINT32    Advertise;
  UINT32    Bmsr;
  BOOLEAN   NeedUpdate;

  NeedUpdate = FALSE;

  /* Only allow advertising what this PHY supports */
  PhyAdvertise = Dpaa2Phy->Support;

  /* Setup standard advertisement */
  Advertise =   Dpaa2PhyRegisterRead (Dpaa2Phy, MDIO_CTL_DEVAD_NONE, MII_ADVERTISE);
  OrigAdvertise = Advertise;

  Advertise &= ~(PHY_ADVERTISE_ALL | PHY_ADVERTISE_100BASE4 | PHY_ADVERTISE_PAUSE_CAP |
                 PHY_ADVERTISE_PAUSE_ASYM);

  if (PhyAdvertise & PHY_ADVERTISED_10BaseT_HALF) {
    Advertise |= PHY_ADVERTISE_10HALF;
  }
  if (PhyAdvertise & PHY_ADVERTISED_10BaseT_FULL) {
    Advertise |= PHY_ADVERTISE_10FULL;
  }
  if (PhyAdvertise & PHY_ADVERTISED_100BaseT_HALF) {
    Advertise |= PHY_ADVERTISE_100HALF;
  }
  if (PhyAdvertise & PHY_ADVERTISED_100BaseT_FULL) {
    Advertise |= PHY_ADVERTISE_100FULL;
  }
  if (PhyAdvertise & PHY_ADVERTISED_Pause) {
    Advertise |= PHY_ADVERTISE_PAUSE_CAP;
  }
  if (PhyAdvertise & PHY_ADVERTISED_Asym_Pause) {
    Advertise |= PHY_ADVERTISE_PAUSE_ASYM;
  }
  if (PhyAdvertise & PHY_ADVERTISED_1000BaseX_HALF) {
    Advertise |= PHY_ADVERTISE_1000XHALF;
  }
  if (PhyAdvertise & PHY_ADVERTISED_1000BaseX_FULL) {
    Advertise |= PHY_ADVERTISE_1000XFULL;
  }

  if (Advertise != OrigAdvertise) {
    Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_CTL_DEVAD_NONE, MII_ADVERTISE, Advertise);

    NeedUpdate = TRUE;
  }

  Bmsr = Dpaa2PhyRegisterRead (Dpaa2Phy, MDIO_CTL_DEVAD_NONE, MII_BMSR);

  ///
  // As Per 802.3-2008, Section 22.2.4.2.16 Extended status,
  // all 1000Mbits/sec capable PHYs shall have the BMSR_ESTATEN
  // bit set to a logical 1
  ///
  if (!(Bmsr & PHY_BMSR_ESTATEN)) {
    return NeedUpdate;
  }

  /* Configure gigabit if it's supported */
  Advertise = Dpaa2PhyRegisterRead (Dpaa2Phy, MDIO_CTL_DEVAD_NONE, MII_CTRL1000);
  OrigAdvertise = Advertise;

  Advertise &= ~(PHY_ADVERTISE_1000FULL | PHY_ADVERTISE_1000HALF);

  if (Dpaa2Phy->Support & (SUPPORT_1000BaseT_HALF | SUPPORT_1000BaseT_FULL)) {
     if (PhyAdvertise & SUPPORT_1000BaseT_HALF) {
       Advertise |= PHY_ADVERTISE_1000HALF;
     }
     if (PhyAdvertise & SUPPORT_1000BaseT_FULL) {
       Advertise |= PHY_ADVERTISE_1000FULL;
     }
  }

  if (Advertise != OrigAdvertise) {
    NeedUpdate = TRUE;
  }

  Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_CTL_DEVAD_NONE, MII_CTRL1000, Advertise);

  return NeedUpdate;
}

/**
 * RestartAneg - Enable and Restart Autonegotiation
 * @Dpaa2Phy   : target DPAA2_PHY struct
 **/
VOID
RestartAneg (
  DPAA2_PHY *Dpaa2Phy
  )
{
  UINT32    Bmcr;

  Bmcr = Dpaa2PhyRegisterRead (Dpaa2Phy, MDIO_CTL_DEVAD_NONE, MII_BMCR);

  Bmcr |= (PHY_BMCR_ANENABLE | PHY_BMCR_ANRESTART);

  /* Don't isolate the PHY if we're negotiating */
  Bmcr &= ~(PHY_BMCR_ISOLATE);

  Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_CTL_DEVAD_NONE, MII_BMCR, Bmcr);
}

 /**
  * ConfigAneg  - restart auto-negotiation or write BMCR
  * @Dpaa2Phy   : target DPAA2_PHY struct
  *
  * Description : If auto-negotiation is enabled, we configure the
  *               advertising, and then restart auto-negotiation.
  *               If it is not, then we write the BMCR.
  **/
VOID
ConfigAneg (
  DPAA2_PHY *Dpaa2Phy
  )
{
  BOOLEAN   Updated;
  UINT32    Bmcr;

  Updated = ConfigAdvertising (Dpaa2Phy);

  if (Updated == FALSE) {
    Bmcr = Dpaa2PhyRegisterRead (Dpaa2Phy, MDIO_CTL_DEVAD_NONE, MII_BMCR);

    if (!(Bmcr & PHY_BMCR_ANENABLE) || (Bmcr & PHY_BMCR_ISOLATE)) {
      Updated = TRUE; /* do restart aneg */
    }
  }

  //
  // Only restart aneg if we are advertising something different
  // than we were before
  //
  if (Updated == TRUE) {
    RestartAneg (Dpaa2Phy);
  }
}
