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
#include <Library/IoLib.h>
#include "IfcLib.h"

UINT8 mNandCS;
UINT8 mNorCS;
UINT8 mFpgaCS;

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
  if (NorIfcTimings.CS >= IFC_CS0) {
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
