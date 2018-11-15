/**@file

  Copyright 2017 NXP

  This Program And The Accompanying Materials
  Are Licensed And Made Available Under The Terms And Conditions Of The BSD License
  Which Accompanies This Distribution.  The Full Text Of The License May Be Found At
  Http://Opensource.Org/Licenses/Bsd-License.Php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include <Library/BeIoLib.h>
#include <Library/IoLib.h>
#include <Library/MmcLib.h>
#include <Library/TimerLib.h>
#include "MmcInternal.h"

extern MMC *mMmc;

/**
  Function to read from MMC depending upon pcd

  @param   Address  MMC register to read

  @retval           Read Value from register

**/
UINT32
EFIAPI
MmcRead (
  IN  UINTN         Address
  )
{
  if (FixedPcdGetBool (PcdMmcBigEndian)) {
    return BeMmioRead32(Address);
  } else {
    return MmioRead32 (Address);
  }
}

/**
  Function to write on MMC depeding upon pcd

  @param   Address  MMC register to write

**/
UINT32
EFIAPI
MmcWrite (
  IN  UINTN         Address,
  IN  UINT32        Value
  )
{
  if (FixedPcdGetBool (PcdMmcBigEndian)) {
    return BeMmioWrite32 (Address, Value);
  } else {
    return MmioWrite32 (Address, Value);
  }
}

/**
  Function to call MmioAndThenOr32 depending upon pcd

  @param   Address  MMC register
  @param   AndData  The value to AND with the read value from the MMC register
  @param   OrData   The value to OR with the result of the AND operation.

  @retval           Value written back to register

**/
UINT32
EFIAPI
MmcAndThenOr (
  IN  UINTN         Address,
  IN  UINT32        AndData,
  IN  UINT32        OrData
  )
{
  if (FixedPcdGetBool (PcdMmcBigEndian)) {
    return BeMmioAndThenOr32 (Address, AndData, OrData);
  } else {
    return MmioAndThenOr32 (Address, AndData, OrData);
  }
}

/**
  Function to call MmioOr32 depending upon pcd

  @param   Address  MMC register
  @param   OrData   The value to OR with the result of the AND operation.

  @retval           Value written back to register

**/
UINT32
EFIAPI
MmcOr (
  IN  UINTN         Address,
  IN  UINT32        OrData
  )
{
  if (FixedPcdGetBool (PcdMmcBigEndian)) {
    return BeMmioOr32 (Address, OrData);
  } else {
    return MmioOr32 (Address, OrData);
  }
}

/**
  Function to call MmioAnd32 depending upon pcd

  @param   Address  MMC register
  @param   AndData  The value to AND with the read value from the MMC register

  @retval           Value written back to register

**/
UINT32
EFIAPI
MmcAnd (
  IN  UINTN         Address,
  IN  UINT32        AndData
  )
{
  if (FixedPcdGetBool (PcdMmcBigEndian)) {
    return BeMmioAnd32 (Address, AndData);
  } else {
    return MmioAnd32 (Address, AndData);
  }
}

/**
  Function to Dump MMC Controller register
**/
VOID
DumpMmcRegs (
  IN VOID   *BaseAddress
  )
{
  SDXC_REGS *Regs;

  Regs = BaseAddress;

  DEBUG ((DEBUG_ERROR, "Dsaddr : 0x%x \n", Regs->Dsaddr));
  DEBUG ((DEBUG_ERROR, "Blkattr : 0x%x \n", Regs->Blkattr));
  DEBUG ((DEBUG_ERROR, "CmdArg : 0x%x \n", Regs->CmdArg));
  DEBUG ((DEBUG_ERROR, "Xfertype : 0x%x \n", Regs->Xfertype));
  DEBUG ((DEBUG_ERROR, "Rspns0 : 0x%x \n", Regs->Rspns0));
  DEBUG ((DEBUG_ERROR, "Rspns1 : 0x%x \n", Regs->Rspns1));
  DEBUG ((DEBUG_ERROR, "Rspns1 : 0x%x \n", Regs->Rspns1));
  DEBUG ((DEBUG_ERROR, "Rspns3 : 0x%x \n", Regs->Rspns3));
  DEBUG ((DEBUG_ERROR, "Datport : 0x%x \n", Regs->Datport));
  DEBUG ((DEBUG_ERROR, "Prsstat : 0x%x \n", Regs->Prsstat));
  DEBUG ((DEBUG_ERROR, "Proctl : 0x%x \n", Regs->Proctl));
  DEBUG ((DEBUG_ERROR, "Sysctl : 0x%x \n", Regs->Sysctl));
  DEBUG ((DEBUG_ERROR, "Irqstat : 0x%x \n", Regs->Irqstat));
  DEBUG ((DEBUG_ERROR, "Irqstaten : 0x%x \n", Regs->Irqstaten));
  DEBUG ((DEBUG_ERROR, "Irqsigen : 0x%x \n", Regs->Irqsigen));
  DEBUG ((DEBUG_ERROR, "Autoc12err : 0x%x \n", Regs->Autoc12err));
  DEBUG ((DEBUG_ERROR, "Hostcapblt : 0x%x \n", Regs->Hostcapblt));
  DEBUG ((DEBUG_ERROR, "Wml : 0x%x \n", Regs->Wml));
  DEBUG ((DEBUG_ERROR, "Mixctrl : 0x%x \n", Regs->Mixctrl));
  DEBUG ((DEBUG_ERROR, "Fevt : 0x%x \n", Regs->Fevt));
  DEBUG ((DEBUG_ERROR, "Admaes : 0x%x \n", Regs->Admaes));
  DEBUG ((DEBUG_ERROR, "Adsaddr : 0x%x \n", Regs->Adsaddr));
  DEBUG ((DEBUG_ERROR, "Hostver : 0x%x \n", Regs->Hostver));
  DEBUG ((DEBUG_ERROR, "Dmaerraddr : 0x%x \n", Regs->Dmaerraddr));
  DEBUG ((DEBUG_ERROR, "Dmaerrattr : 0x%x \n", Regs->Dmaerrattr));
  DEBUG ((DEBUG_ERROR, "Hostcapblt2 : 0x%x \n", Regs->Hostcapblt2));
  DEBUG ((DEBUG_ERROR, "Tcr : 0x%x \n", Regs->Tcr));
  DEBUG ((DEBUG_ERROR, "Sddirctl : 0x%x \n", Regs->Sddirctl));
  DEBUG ((DEBUG_ERROR, "Scr : 0x%x \n", Regs->Scr));
}

/**
  Function to create dma map for read/write operation

  @param   DmaData  Pointer to Dma data Structure

  @retval           Address of dma map

**/
VOID *
GetDmaBuffer (
  IN  DMA_DATA      *DmaData
  )
{
  EFI_STATUS        Status;
  EFI_PHYSICAL_ADDRESS PhyAddr;

  Status = DmaAllocateBuffer (EfiBootServicesData,
                              EFI_SIZE_TO_PAGES (DmaData->Bytes),
                              &(DmaData->DmaAddr));
  if (Status) {
    DEBUG ((DEBUG_ERROR,"DmaAllocateBuffer failed\n"));
    return NULL;
  }

  Status = DmaMap (DmaData->MapOperation, DmaData->DmaAddr,
                  &DmaData->Bytes, &PhyAddr, &DmaData->Mapping);
  if (Status) {
    DEBUG ((DEBUG_ERROR,"DmaMap failed %d \n", Status));

    DmaFreeBuffer (EFI_SIZE_TO_PAGES (DmaData->Bytes), DmaData->DmaAddr);

    return NULL;
  }
  return (VOID *)PhyAddr;
}

/**
  Function to free dma map

  @param   DmaData  Pointer to Dma data Structure

  @retval           Address of dma map

**/
EFI_STATUS
FreeDmaBuffer (
  IN  DMA_DATA      *DmaData
  )
{
  EFI_STATUS        Status;

  Status = DmaUnmap (DmaData->Mapping);
  if (Status) {
    DEBUG ((DEBUG_ERROR,"DmaUnmap failed 0x%x\n", Status));
  }

  Status = DmaFreeBuffer (EFI_SIZE_TO_PAGES (DmaData->Bytes), DmaData->DmaAddr);
  if (Status) {
    DEBUG ((DEBUG_ERROR,"DmaFreeBuffer failed 0x%x\n", Status));
    return EFI_OUT_OF_RESOURCES;
  }

  return EFI_SUCCESS;
}

/**
  Function to select the transfer type flags depending upon given
  command and data packet

  @param   Cmd      Pointer to MMC command
  @param   Data     Pointer to MMC data

  @retval           Returns the XFERTYPE flags

**/
UINT32
SdxcXfertype (
  IN  MMC_CMD_INFO  *Cmd,
  IN  MMC_DATA      *Data
  )
{
  UINT32            Xfertype;

  Xfertype = 0;

  if (Data) {
    Xfertype |= XFERTYPE_DPSEL;
    Xfertype |= XFERTYPE_DMAEN; //DMA Support

    if (Data->Blocks > 1) {
      Xfertype |= XFERTYPE_MSBSEL;
      Xfertype |= XFERTYPE_BCEN;
    }

    if (Data->Flags & MMC_DATA_READ) {
      Xfertype |= XFERTYPE_DTDSEL;
    }
  }

  if (Cmd->RespType & MMC_RSP_CRC) {
    Xfertype |= XFERTYPE_CCCEN;
  }
  if (Cmd->RespType & MMC_RSP_OPCODE) {
    Xfertype |= XFERTYPE_CICEN;
  }
  if (Cmd->RespType & MMC_RSP_136) {
    Xfertype |= XFERTYPE_RSPTYP_136;
  }
  else if (Cmd->RespType & MMC_RSP_BUSY) {
    Xfertype |= XFERTYPE_RSPTYP_48_BUSY;
  }
  else if (Cmd->RespType & MMC_RSP_PRESENT) {
    Xfertype |= XFERTYPE_RSPTYP_48;
  }

  if (Cmd->CmdIdx == CMD_STOP_TRANSMISSION) {
    Xfertype |= XFERTYPE_CMDTYP_ABORT;
  }

  return XFERTYPE_CMD (Cmd->CmdIdx) | Xfertype;
}

/**
  Function to set up MMC data (timeout value,watermark level,
  system address,block attributes etc.)

  @param   Data     Pointer to MMC data

**/
EFI_STATUS
SdxcSetupData (
  IN  SDXC_REGS     *Regs,
  IN  MMC_DATA      *Data
  )
{
  INT32             Timeout;
  UINT32            WmlVal;

  EFI_PHYSICAL_ADDRESS Addr;

  Timeout = 0;
  WmlVal = 0;

  WmlVal = Data->Blocksize/4;

  if (Data->Flags & MMC_DATA_READ) {

    if (WmlVal > WML_RD_MAX) {
      WmlVal = WML_RD_MAX_VAL;
    }

    MmcAndThenOr ((UINTN)&Regs->Wml, ~WML_RD_MASK, WmlVal);

  } else {
    if (WmlVal > WML_WR_MAX) {
      WmlVal = WML_WR_MAX_VAL;
    }

    if ((MmcRead ((UINTN)&Regs->Prsstat) & PRSSTATE_WPSPL) == 0) {
      DEBUG ((DEBUG_ERROR, "The SD card is locked. Can not write to a locked card.\n"));
      return EFI_ACCESS_DENIED;
    }

    MmcAndThenOr ((UINTN)&Regs->Wml, ~WML_WR_MASK, WmlVal << 16);
  }

  Addr = (EFI_PHYSICAL_ADDRESS)Data->Addr;
  MmcWrite ((UINTN)&Regs->Dsaddr, Addr);

  MmcWrite ((UINTN)&Regs->Blkattr, Data->Blocks << 16 | Data->Blocksize);

  // Calculate the timeout period for data transactions
  Timeout = GenericFls (mMmc->Clock/4) - 13;

  if (Timeout > 14) {
    Timeout = 14;
  }

  if (Timeout < 0) {
    Timeout = 0;
  }

  MmcAndThenOr ((UINTN)&Regs->Sysctl, ~SYSCTL_TIMEOUT_MASK, Timeout << 16);

  return EFI_SUCCESS;
}

/**
  Function to peform reset of MMC command and data

**/
VOID
ResetCmdFailedData (
  IN  SDXC_REGS     *Regs,
  IN  UINT8         Data
  )
{
  INT32             Timeout;

  Timeout = TIMEOUT;

  // Reset CMD and DATA portions on error
  MmcWrite ((UINTN)&Regs->Sysctl, MmcRead ((UINTN)&Regs->Sysctl) |
           SYSCTL_RSTC);

  while ((MmcRead ((UINTN)&Regs->Sysctl) & SYSCTL_RSTC) && Timeout--);
  if (Timeout < 0) {
    DEBUG ((DEBUG_ERROR, "Failed to reset CMD portion on error\n"));
    return;
  }

  Timeout = TIMEOUT;
  if (Data) {
    MmcWrite ((UINTN)&Regs->Sysctl,
           MmcRead ((UINTN)&Regs->Sysctl) | SYSCTL_RSTD);
    while ((MmcRead ((UINTN)&Regs->Sysctl) & SYSCTL_RSTD) && Timeout--);
    if (Timeout < 0) {
      DEBUG ((DEBUG_ERROR, "Failed to reset DATA portion on error\n"));
    }
  }

  MmcWrite ((UINTN)&Regs->Irqstat, 0xFFFFFFFF);
}

/**
  Function to do MMC read/write transfer using DMA and checks
  whether transfer is completed or not

**/
EFI_STATUS
Transfer (
  IN  VOID         *BaseAddress
  )
{
  UINT32            Irqstat;
  UINT32            Timeout;
  SDXC_REGS         *Regs;

  Regs = BaseAddress;
  Timeout = TRANSFER_TIMEOUT;

  do {
    Irqstat = MmcRead ((UINTN)&Regs->Irqstat);

    if (Irqstat & IRQSTATE_DTOE) {
      DumpMmcRegs (BaseAddress);
      ResetCmdFailedData (Regs, 1);
      return EFI_TIMEOUT;
    }

    if (Irqstat & DATA_ERR) {
      ResetCmdFailedData (Regs, 1);
      return EFI_DEVICE_ERROR;
    }

   MicroSecondDelay (10);

  } while ((!(Irqstat & DATA_COMPLETE)) && Timeout--);

  if (Timeout <= 0) {
    DEBUG ((DEBUG_ERROR, "Timeout Waiting for DATA_COMPLETE to set\n"));
    ResetCmdFailedData (Regs, 1);
    return EFI_TIMEOUT;
  }

  MmcWrite ((UINTN)&Regs->Irqstat, 0xFFFFFFFF);
  return EFI_SUCCESS;
}

/**
  Function to set MMC host controller system control register

  @param  Clock     Clock value for setting the register

**/
VOID
SetSysctl (
  IN  SDXC_REGS     *Regs,
  IN  UINT32        Clock
  )
{
  INT32             Div;
  INT32             PreDiv;
  INT32             SdhcClk;
  UINT32            Clk;

  SdhcClk = mMmc->SdhcClk;

  if (Clock < mMmc->FMin) {
    Clock = mMmc->FMin;
  } else if (Clock > mMmc->FMax) {
    Clock = mMmc->FMax;
  }

  mMmc->Clock = Clock;

  if (SdhcClk / 16 > Clock) {
    for (PreDiv = 2; PreDiv < 256; PreDiv *= 2)
      if ((SdhcClk / PreDiv) <= (Clock * 16)) {
        break;
      }
  } else {
    PreDiv = 2;
  }

  for (Div = 1; Div <= 16; Div++)
    if ((SdhcClk / (Div * PreDiv)) <= Clock) {
      break;
    }

  PreDiv >>= mMmc->DdrMode ? DIV_2 : DIV_1;
  Div -= 1;

  Clk = (PreDiv << 8) | (Div << 4);

  MmcAnd ((UINTN)&Regs->Sysctl, ~SYSCTL_CKEN);

  MmcAndThenOr ((UINTN)&Regs->Sysctl, ~SYSCTL_CLOCK_MASK, Clk);

  MicroSecondDelay (100);

  Clk = SYSCTL_PEREN | SYSCTL_CKEN;

  MmcOr ((UINTN)&Regs->Sysctl, Clk);
}

/**
  Function to set MMC host controller bus width

  @param  Mmc       Pointer to MMC data structure
  @param  BWidth    Bus width to be set

**/
VOID
SdxcSetBusWidth (
  IN  SDXC_REGS     *Regs,
  IN  MMC           *Mmc,
  IN  UINT32        BWidth
  )
{
  Mmc->BusWidth = BWidth;

  SetIos ((VOID *)Regs, Mmc->Clock, Mmc->BusWidth, 0);
}

/**
  Function to Initialize MMC host controller

  @param  Mmc       Pointer to MMC data structure

**/
EFI_STATUS
SdxcInit (
  IN  SDXC_REGS     *Regs,
  IN  MMC           *Mmc
  )
{
  INT32             Timeout;

  Timeout = TIMEOUT;

  // Reset the entire host controller
  MmcOr ((UINTN)&Regs->Sysctl, SYSCTL_RSTA);

  // Wait until the controller is available
  while ((MmcRead ((UINTN)&Regs->Sysctl) & SYSCTL_RSTA) && --Timeout) {
    MicroSecondDelay (1000);
  }

  if (Timeout <= 0) {
    DEBUG ((DEBUG_ERROR, "Host controller failed to reset \n"));
    return EFI_DEVICE_ERROR;
  }

  // Enable cache snooping
  MmcWrite ((UINTN)&Regs->Scr, ENABLE_CACHE_SNOOPING);

  MmcOr ((UINTN)&Regs->Sysctl, SYSCTL_HCKEN | SYSCTL_IPGEN);

  // Set the initial clock speed
  SetIos ((VOID *)Regs, MIN_CLK_FREQUENCY, Mmc->BusWidth, 0);

  // Disable the BRR and BWR bits in IRQSTAT
  MmcAnd ((UINTN)&Regs->Irqstaten,
          ~(IRQSTATE_EN_BRR | IRQSTATE_EN_BWR));

  // Set Little Endian mode for data Buffer
  MmcWrite ((UINTN)&Regs->Proctl, PRCTL_INIT);

  // Set timeout to the maximum value
  MmcAndThenOr ((UINTN)&Regs->Sysctl,
          ~SYSCTL_TIMEOUT_MASK, 14 << 16);

  return EFI_SUCCESS;
}

/**
  Function to reset MMC host controller

  @param   Regs     Pointer to MMC host Controller

**/
VOID
SdxcReset (
  IN  SDXC_REGS     *Regs
  )
{
  UINT64            Timeout;

  Timeout = TIMEOUT;

  // Reset the controller
  MmcWrite ((UINTN)&Regs->Sysctl, SYSCTL_RSTA);

  // Hardware clears the Bit when it is done
  while ((MmcRead ((UINTN)&Regs->Sysctl) & SYSCTL_RSTA) && --Timeout) {
    MicroSecondDelay (10);
  }

  if (!Timeout) {
    DEBUG ((DEBUG_ERROR, "MMC/SD: Reset Never Completed.\n"));
  }
}
