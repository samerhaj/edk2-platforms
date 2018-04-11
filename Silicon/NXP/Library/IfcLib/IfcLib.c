/** @IfcLib.c

  Copyright 2018 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include <Library/BeIoLib.h>
#include <Library/BaseLib.h>
#include <Library/DebugLib.h>
#include <Library/IoLib.h>
#include <Library/TimerLib.h>
#include "IfcLib.h"

UINT8 mNandCS;
UINT8 mNorCS;
UINT8 mFpgaCS;

/**
  Function implements IFC read to devices connected
  via IFC controller

  @param[in]  Address IFC address to read
  @param[out] Value   Value read

**/
UINT32
EFIAPI
IfcRead (
  IN UINT32 Address
  )
{
  UINT32 Value;

  if (FixedPcdGetBool (PcdIfcBigEndian)) {
    Value = BeMmioRead32 (Address);
  } else {
    Value = MmioRead32 (Address);
  }

  return Value;
}

/**
  Function implements IFC write to devices connected
  via IFC controller

  @param[in] Address IFC address to write
  @param[in] Value   Value to be written

**/
UINT32
EFIAPI
IfcWrite (
  IN  UINTN  Address,
  IN  UINT32 Value
  )
{
  if (FixedPcdGetBool(PcdIfcBigEndian)) {
    return BeMmioWrite32 (Address, Value);
  } else {
    return MmioWrite32 (Address, Value);
  }
}

VOID
SetTimings (
  IN  UINT8        CS,
  IN  IFC_TIMINGS  IfcTimings
  )
{
  IFC_REGS*        IfcRegs;

  IfcRegs = (IFC_REGS*) PcdGet64 (PcdIfcBaseAddr);

  // Configure Extended chip select property registers
  IfcWrite ((UINTN)&IfcRegs->CsprCs[CS].CsprExt, IfcTimings.CsprExt);

  // Configure Fpga timing registers
  IfcWrite ((UINTN)&IfcRegs->FtimCs[CS].Ftim[IFC_FTIM0], IfcTimings.Ftim[0]);
  IfcWrite ((UINTN)&IfcRegs->FtimCs[CS].Ftim[IFC_FTIM1], IfcTimings.Ftim[1]);
  IfcWrite ((UINTN)&IfcRegs->FtimCs[CS].Ftim[IFC_FTIM2], IfcTimings.Ftim[2]);
  IfcWrite ((UINTN)&IfcRegs->FtimCs[CS].Ftim[IFC_FTIM3], IfcTimings.Ftim[3]);

  // Configure chip select option registers
  IfcWrite ((UINTN)&IfcRegs->CsprCs[CS].Cspr, IfcTimings.Cspr);

  // Configure address mask registers
  IfcWrite ((UINTN)&IfcRegs->AmaskCs[CS].Amask, IfcTimings.Amask);

  // Configure chip select property registers
  IfcWrite ((UINTN)&IfcRegs->CsorCs[CS].Csor, IfcTimings.Csor);

  return;
}

VOID
NandInit(
  VOID
  )
{
  IFC_REGS*       IfcRegs;
  IFC_TIMINGS     NandIfcTimings;

  IfcRegs = (IFC_REGS*) PcdGet64 (PcdIfcBaseAddr);

  // Get Nand Flash Timings
  GetIfcNandFlashTimings (&NandIfcTimings);

  mNandCS = NandIfcTimings.CS;

  // clear event registers
  IfcWrite ((UINTN)&IfcRegs->IfcNand.PgrdcmplEvtStat, ~0U);

  IfcWrite ((UINTN)&IfcRegs->IfcNand.NandEvterStat, ~0U);

  // Enable error and event for any detected errors
  IfcWrite ((UINTN)&IfcRegs->IfcNand.NandEvterEn,
                  IFC_NAND_EVTER_EN_OPC_EN |
                  IFC_NAND_EVTER_EN_PGRDCMPL_EN |
                  IFC_NAND_EVTER_EN_FTOER_EN |
                  IFC_NAND_EVTER_EN_WPER_EN);
  IfcWrite ((UINTN)&IfcRegs->IfcNand.Ncfgr, 0x0);

  SetTimings (mNandCS, NandIfcTimings);

  return;
}

VOID
FpgaInit (
  VOID
  )
{
  IFC_TIMINGS     FpgaIfcTimings;

  // Get Fpga Flash Timings
  GetIfcFpgaTimings (&FpgaIfcTimings);

  mFpgaCS = FpgaIfcTimings.CS;

  SetTimings (mFpgaCS, FpgaIfcTimings);

  return;
}

VOID
NorInit (
  VOID
  )
{
  IFC_TIMINGS     NorIfcTimings;

  // Get NOR Flash Timings
  GetIfcNorFlashTimings (&NorIfcTimings);

  // Validate chip select
  if (NorIfcTimings.CS <= IFC_CS_MAX) {
    mNorCS = NorIfcTimings.CS;
    SetTimings (mNorCS, NorIfcTimings);
  }

  return;
}

//
// IFC has NOR , NAND and FPGA
//
VOID
IfcInit (
  VOID
  )
{
  NorInit();
  NandInit();
  FpgaInit();

  return;
}

/**
  Execute IFC NAND command and wait for it to complete
 **/
STATIC
INTN
IfcRunCmd (
  INTN NandCs
  )
{
  UINT32    Status;
  UINT32    Count;
  IFC_REGS* IfcRegs;

  IfcRegs = (IFC_REGS*) PcdGet64 (PcdIfcBaseAddr);
  Status = 0;

  // Set the chip select for NAND Transaction
  IfcWrite ((UINTN)&IfcRegs->IfcNand.NandCsel, (NandCs << 26));

  // Start read/write seq
  IfcWrite ((UINTN)&IfcRegs->IfcNand.NandSeqStrt, IFC_NAND_SEQ_STRT_FIR_STRT);

  // Wait for NAND Machine complete flag or timeout
  Count = 0;
  while (Count++ < MAX_RETRY_COUNT) {
    Status = IfcRead ((UINTN)&IfcRegs->IfcNand.NandEvterStat);

    if (Status & IFC_NAND_EVTER_STAT_OPC) {
      break;
    }

    MicroSecondDelay (100);
  }

  IfcWrite ((UINTN)&IfcRegs->IfcNand.NandEvterStat, Status);

  if (Status & IFC_NAND_EVTER_STAT_FTOER) {
    DEBUG ((DEBUG_ERROR, "Flash Time Out Error %x \n", Status));
  }

  if (Status & IFC_NAND_EVTER_STAT_WPER) {
    DEBUG ((DEBUG_ERROR, "Write Protect Error %x \n", Status));
  }

  // returns 0 on success otherwise non-zero
  return Status == IFC_NAND_EVTER_STAT_OPC ? EFI_SUCCESS : EFI_DEVICE_ERROR;
}

/**
  Set up the IFC hardware block and page address fields, and the ifc nand
  structure addr field to point to the correct IFC buffer in memory
 **/
STATIC
VOID
SetAddressRegs (
  INTN Column,
  INTN PageAddr
  )
{
  IFC_REGS* IfcRegs;

  IfcRegs = (IFC_REGS*) PcdGet64 (PcdIfcBaseAddr);

  // Program ROW0/COL0
  IfcWrite ((UINTN)&IfcRegs->IfcNand.Row0, PageAddr);
  IfcWrite ((UINTN)&IfcRegs->IfcNand.Col0, Column);
}

/**
  Function to implement wait operation during NAND block write
  Writing instruction and command register
**/
EFI_STATUS
Wait (
  INTN NandCs
  )
{
  EFI_STATUS Status;
  IFC_REGS*  IfcRegs;

  IfcRegs = (IFC_REGS*) PcdGet64 (PcdIfcBaseAddr);

  IfcWrite ((UINTN)&IfcRegs->IfcNand.NandFir0,
            (IFC_FIR_OP_CW0 << IFC_NAND_FIR0_OP0_SHIFT) |
            (IFC_FIR_OP_RDSTAT << IFC_NAND_FIR0_OP1_SHIFT));
  IfcWrite ((UINTN)&IfcRegs->IfcNand.NandFcr0, IFC_NAND_CMD_STATUS <<
            IFC_NAND_FCR0_CMD0_SHIFT);
  IfcWrite ((UINTN)&IfcRegs->IfcNand.NandFbcr, 1);
  SetAddressRegs (0, 0);

  Status = IfcRunCmd (NandCs);

  return Status;
}

/**
  Function to write instruction and command register during NAND flash
  read operation
**/
STATIC
VOID
IfcSetRegister (
  VOID
  )
{
  IFC_REGS* IfcRegs;

  IfcRegs = (IFC_REGS*) PcdGet64 (PcdIfcBaseAddr);

  IfcWrite ((UINTN)&IfcRegs->IfcNand.NandFir0,
            (IFC_FIR_OP_CW0 << IFC_NAND_FIR0_OP0_SHIFT) |
            (IFC_FIR_OP_CA0 << IFC_NAND_FIR0_OP1_SHIFT) |
            (IFC_FIR_OP_RA0 << IFC_NAND_FIR0_OP2_SHIFT) |
            (IFC_FIR_OP_CMD1 << IFC_NAND_FIR0_OP3_SHIFT) |
            (IFC_FIR_OP_BTRD << IFC_NAND_FIR0_OP4_SHIFT));
  IfcWrite ((UINTN)&IfcRegs->IfcNand.NandFir1, 0x0);
  IfcWrite ((UINTN)&IfcRegs->IfcNand.NandFcr0,
            (IFC_NAND_CMD_READ0 << IFC_NAND_FCR0_CMD0_SHIFT) |
            (IFC_NAND_CMD_READSTART << IFC_NAND_FCR0_CMD1_SHIFT));
}

/**
  Function to send commands to the IFC NAND Machine
 **/
EFI_STATUS
NandCmdSend (
  UINTN Cmd,
  INTN  Column,
  INTN  PgAddr,
  INTN  NandCs
  )
{
  EFI_STATUS Status;
  UINT32     NandFcr0;
  VOID*      BufBase;
  IFC_REGS*  IfcRegs;

  IfcRegs = (IFC_REGS*) PcdGet64 (PcdIfcBaseAddr);

  switch (Cmd) {
    // NAND flash page read operation
    case IFC_NAND_CMD_READ0:
         IfcWrite ((UINTN)&IfcRegs->IfcNand.NandFbcr, 0);
         SetAddressRegs (0, PgAddr);
         IfcSetRegister ();
         return IfcRunCmd (NandCs);
    // NAND flash meta data read operation
    case IFC_NAND_CMD_READID:
         IfcWrite ((UINTN)&IfcRegs->IfcNand.NandFir0,
                   (IFC_FIR_OP_CW0 << IFC_NAND_FIR0_OP0_SHIFT) |
                   (IFC_FIR_OP_UA << IFC_NAND_FIR0_OP1_SHIFT) |
                   (IFC_FIR_OP_RB << IFC_NAND_FIR0_OP2_SHIFT));
         IfcWrite ((UINTN)&IfcRegs->IfcNand.NandFcr0,
                    Cmd << IFC_NAND_FCR0_CMD0_SHIFT);
         IfcWrite ((UINTN)&IfcRegs->IfcNand.Row3, Column);
         IfcWrite ((UINTN)&IfcRegs->IfcNand.NandFbcr, 256);
         SetAddressRegs (0, 0);
         return IfcRunCmd (NandCs);
    // NAND flash erase operation
    case IFC_NAND_CMD_ERASE1:
         SetAddressRegs (0, PgAddr);
         IfcWrite ((UINTN)&IfcRegs->IfcNand.NandFir0,
                   (IFC_FIR_OP_CW0 << IFC_NAND_FIR0_OP0_SHIFT) |
                   (IFC_FIR_OP_RA0 << IFC_NAND_FIR0_OP1_SHIFT) |
                   (IFC_FIR_OP_CMD1 << IFC_NAND_FIR0_OP2_SHIFT));
         IfcWrite ((UINTN)&IfcRegs->IfcNand.NandFcr0,
                   (IFC_NAND_CMD_ERASE1 << IFC_NAND_FCR0_CMD0_SHIFT) |
                   (IFC_NAND_CMD_ERASE2 << IFC_NAND_FCR0_CMD1_SHIFT));
         IfcWrite ((UINTN)&IfcRegs->IfcNand.NandFbcr, 0);
         return IfcRunCmd (NandCs);
    // NAND flash command and instruction register write operation
    case IFC_NAND_CMD_SEQIN:
         NandFcr0 =(IFC_NAND_CMD_SEQIN << IFC_NAND_FCR0_CMD0_SHIFT) |
                   (IFC_NAND_CMD_STATUS << IFC_NAND_FCR0_CMD1_SHIFT) |
                   (IFC_NAND_CMD_PAGEPROG << IFC_NAND_FCR0_CMD2_SHIFT);

         IfcWrite ((UINTN)&IfcRegs->IfcNand.NandFir0,
                   (IFC_FIR_OP_CW0 << IFC_NAND_FIR0_OP0_SHIFT) |
                   (IFC_FIR_OP_CA0 << IFC_NAND_FIR0_OP1_SHIFT) |
                   (IFC_FIR_OP_RA0 << IFC_NAND_FIR0_OP2_SHIFT) |
                   (IFC_FIR_OP_WBCD << IFC_NAND_FIR0_OP3_SHIFT) |
                   (IFC_FIR_OP_CMD2 << IFC_NAND_FIR0_OP4_SHIFT));
         IfcWrite ((UINTN)&IfcRegs->IfcNand.NandFir1,
                   (IFC_FIR_OP_CW1 << IFC_NAND_FIR1_OP5_SHIFT) |
                   (IFC_FIR_OP_RDSTAT << IFC_NAND_FIR1_OP6_SHIFT) |
                   (IFC_FIR_OP_NOP << IFC_NAND_FIR1_OP7_SHIFT));
         IfcWrite ((UINTN)&IfcRegs->IfcNand.NandFcr0, NandFcr0);
         SetAddressRegs (Column, PgAddr);
         return EFI_SUCCESS;
    // NAND flash page write operation
    case IFC_NAND_CMD_PAGEPROG:
         IfcWrite ((UINTN)&IfcRegs->IfcNand.NandFbcr, 0);
         return IfcRunCmd (NandCs);
    // NAND flash status operation
    case IFC_NAND_CMD_STATUS:
         GetIfcNandBufBase (&BufBase);
         IfcWrite ((UINTN)&IfcRegs->IfcNand.NandFir0,
                   (IFC_FIR_OP_CW0 << IFC_NAND_FIR0_OP0_SHIFT) |
                   (IFC_FIR_OP_RB << IFC_NAND_FIR0_OP1_SHIFT));
         IfcWrite ((UINTN)&IfcRegs->IfcNand.NandFcr0,
                   IFC_NAND_CMD_STATUS << IFC_NAND_FCR0_CMD0_SHIFT);
         IfcWrite ((UINTN)&IfcRegs->IfcNand.NandFbcr, 1);
         SetAddressRegs (0, 0);
         Status  = IfcRunCmd (NandCs);
         MmioWrite8 ((UINTN)BufBase,
                     MmioRead8((UINTN)BufBase) | NAND_STATUS_WP);
         return Status;
    // NAND flash reset operation
    case IFC_NAND_CMD_RESET:
         IfcWrite ((UINTN)&IfcRegs->IfcNand.NandFir0,
                         IFC_FIR_OP_CW0 << IFC_NAND_FIR0_OP0_SHIFT);
         IfcWrite ((UINTN)&IfcRegs->IfcNand.NandFcr0,
                         IFC_NAND_CMD_RESET << IFC_NAND_FCR0_CMD0_SHIFT);
         return IfcRunCmd (NandCs);
    default:
         DEBUG ((DEBUG_ERROR, "Unsupported Command 0x%x.\n", Cmd));
         return EFI_UNSUPPORTED;
  }
}
