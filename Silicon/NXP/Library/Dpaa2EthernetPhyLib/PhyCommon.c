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
  IN UINT8              MdioCtlDevAddr,
  IN UINT16             PhyRegNum,
  IN UINT16             Value
  )
{
  UINT32 RegValue;
  MEMAC_MDIO_BUS_REGS *CONST MdioBusRegs = MdioBus->IoRegs;

  ASSERT (MdioBus->Signature == DPAA2_PHY_MDIO_BUS_SIGNATURE);

  RegValue = MmioRead32 ((UINTN)&MdioBusRegs->MdioStat);
  RegValue |= MDIO_STAT_ENC;
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
  MmioWrite32 ((UINTN)&MdioBusRegs->MdioAddr, PhyRegNum);

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
  IN UINT8     MdioCtlDevAddr,
  IN UINT16    PhyRegNum,
  IN UINT16    Value
  )
{
  ASSERT (Dpaa2Phy->Signature == DPAA2_PHY_SIGNATURE);
  ASSERT (MdioCtlDevAddr == 0x0 ||
         MdioCtlDevAddr == MDIO_CTL_DEV_PMAPMD ||
         MdioCtlDevAddr == MDIO_CTL_DEV_AUTO_NEGOTIATION);
  if (Dpaa2Phy->PhyMediaType == COPPER_PHY) {
    ASSERT (PhyRegNum == PHY_CONTROL_REG || PhyRegNum == PHY_STATUS_REG);
  }

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
  IN UINT8              MdioCtlDevAddr,
  IN UINT16             PhyRegNum
  )
{
  UINT32 RegValue;
  MEMAC_MDIO_BUS_REGS *CONST MdioBusRegs = MdioBus->IoRegs;

  ASSERT (MdioBus->Signature == DPAA2_PHY_MDIO_BUS_SIGNATURE);

  RegValue = MmioRead32 ((UINTN)&MdioBusRegs->MdioStat);
  RegValue |= MDIO_STAT_ENC;
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
  MmioWrite32 ((UINTN)&MdioBusRegs->MdioAddr, PhyRegNum);

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
  IN UINT8     MdioCtlDevAddr,
  IN UINT16    PhyRegNum
  )
{
  ASSERT (Dpaa2Phy->Signature == DPAA2_PHY_SIGNATURE);
  ASSERT (MdioCtlDevAddr == 0x0 ||
         MdioCtlDevAddr == MDIO_CTL_DEV_PMAPMD ||
         MdioCtlDevAddr == MDIO_CTL_DEV_AUTO_NEGOTIATION);
  if (Dpaa2Phy->PhyMediaType == COPPER_PHY) {
    ASSERT (PhyRegNum == PHY_CONTROL_REG || PhyRegNum == PHY_STATUS_REG);
  }

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

  TimeoutMsCount = 500;

  if (Dpaa2Phy->Flags & PHY_BROKEN_RESET) {
    return EFI_SUCCESS;
  }

  DPAA_DEBUG_MSG ("Resetting PHY (PHY address: 0x%x) ...\n",
                  Dpaa2Phy->PhyAddress);

  ASSERT (Dpaa2Phy->PhyInterfaceType == PHY_INTERFACE_XGMII);

  PhyRegValue = Dpaa2PhyRegisterRead (Dpaa2Phy, 0x0, PHY_CONTROL_REG);
  if (PhyRegValue == (UINT16)-1) {
    return EFI_DEVICE_ERROR;
  }

  PhyRegValue |= PHY_CONTROL_RESET;
  Dpaa2PhyRegisterWrite (Dpaa2Phy, 0x0, PHY_CONTROL_REG, PhyRegValue);

  /*
   * Poll the control register for the reset bit to go to 0 (it is
   * auto-clearing). This should happen within 0.5 seconds per the
   * IEEE spec.
   */
  while (1) {
    PhyRegValue = Dpaa2PhyRegisterRead (Dpaa2Phy, 0x0, PHY_CONTROL_REG);
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

  /*
   * For now we just support Aquantia PHY
   */
  if (Dpaa2Phy->PhyId == QC_PHY) {
    return QC8035PhyConfig( Dpaa2Phy);
  } else if (Dpaa2Phy->PhyMediaType == COPPER_PHY) {
    return AquantiaPhyConfig (Dpaa2Phy);
  } else if (Dpaa2Phy->PhyMediaType == OPTICAL_PHY) {
    return CortinaPhyConfig (Dpaa2Phy);
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

  /*
   * For now we just support the Aquantia PHY
   */
  if (Dpaa2Phy->PhyMediaType == COPPER_PHY) {
    return AquantiaPhyStartup (Dpaa2Phy);
  } else if (Dpaa2Phy->PhyMediaType == OPTICAL_PHY) {
    return CortinaPhyStartup (Dpaa2Phy);
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
