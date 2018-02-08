/** @file

  Copyright 2018 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include <IfcBoardSpecific.h>

VOID
GetIfcNorFlashTimings (
  IN IFC_TIMINGS * NorIfcTimings
  )
{
  NorIfcTimings->CS = IFC_NOR_CS;

  return ;
}

VOID
GetIfcFpgaTimings (
  IN IFC_TIMINGS  *FpgaIfcTimings
  )
{
  FpgaIfcTimings->Ftim[0] = FPGA_FTIM0;
  FpgaIfcTimings->Ftim[1] = FPGA_FTIM1;
  FpgaIfcTimings->Ftim[2] = FPGA_FTIM2;
  FpgaIfcTimings->Ftim[3] = FPGA_FTIM3;
  FpgaIfcTimings->Cspr = FPGA_CSPR;
  FpgaIfcTimings->CsprExt = FPGA_CSPR_EXT;
  FpgaIfcTimings->Amask = FPGA_AMASK;
  FpgaIfcTimings->Csor = FPGA_CSOR;
  FpgaIfcTimings->CS = IFC_FPGA_CS;

  return;
}

VOID
GetIfcNandFlashTimings (
  IN IFC_TIMINGS * NandIfcTimings
  )
{
  NandIfcTimings->Ftim[0] = NAND_FTIM0;
  NandIfcTimings->Ftim[1] = NAND_FTIM1;
  NandIfcTimings->Ftim[2] = NAND_FTIM2;
  NandIfcTimings->Ftim[3] = NAND_FTIM3;
  NandIfcTimings->Cspr = NAND_CSPR;
  NandIfcTimings->CsprExt = NAND_CSPR_EXT;
  NandIfcTimings->Amask = NAND_AMASK;
  NandIfcTimings->Csor = NAND_CSOR;
  NandIfcTimings->CS = IFC_NAND_CS;

  return;
}
