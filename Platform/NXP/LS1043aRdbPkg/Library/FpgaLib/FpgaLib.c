/** @FpgaLib.c
  Fpga Library for LS1043A-RDB board, containing functions to
  program and read the Fpga registers.

  FPGA is connected to IFC Controller and so MMIO APIs are used
  to read/write FPGA registers

  Copyright 2017 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution. The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include <Base.h>
#include <Library/BaseLib.h>
#include <Library/DebugLib.h>
#include <Library/FpgaLib.h>
#include <Library/IoLib.h>

#include "FpgaLibInternal.h"

UINT8
FpgaRead (
  IN  UINTN  Reg
  )
{
  VOID       *Base;

  Base = (VOID *)FPGA_BASE_PHYS;

  return MmioRead8 ((UINTN)(Base + Reg));
}

VOID
FpgaWrite (
  IN  UINTN  Reg,
  IN  UINT8  Value
  )
{
  VOID       *Base;

  Base = (VOID *)FPGA_BASE_PHYS;

  MmioWrite8 ((UINTN)(Base + Reg), Value);
}

VOID
FpgaRevBit (
  OUT UINT8  *Value
  )
{
  UINT8      Rev;
  UINT8      Val;
  UINTN      Index;

  Val = *Value;
  Rev = Val & 1;
  for (Index = 1; Index <= 7; Index++) {
    Val >>= 1;
    Rev <<= 1;
    Rev |= Val & 1;
  }

  *Value = Rev;
}

VOID
PrintBoardPersonality (
  VOID
  )
{
  UINT8  RcwSrc1;
  UINT8  RcwSrc2;
  UINT32 RcwSrc;
  UINT32 Sd1RefClkSel;

  RcwSrc1 = FPGA_READ(RcwSource1);
  RcwSrc2 = FPGA_READ(RcwSource2);
  FpgaRevBit (&RcwSrc1);
  RcwSrc = RcwSrc1;
  RcwSrc = (RcwSrc << 1) | RcwSrc2;

  switch (RcwSrc) {
    case SRC_VBANK:
      DEBUG ((DEBUG_INFO, "vBank: %d\n", FPGA_READ(Vbank)));
      break;
    case SRC_NAND:
      DEBUG ((DEBUG_INFO, "NAND\n"));
      break;
    case SRC_QSPI:
      DEBUG ((DEBUG_INFO, "QSPI vBank %d\n", FPGA_READ(Vbank)));
      break;
    case SRC_SD:
      DEBUG ((DEBUG_INFO, "SD\n"));
      break;
    default:
      DEBUG ((DEBUG_INFO, "Invalid setting of SW5\n"));
      break;
  }

  DEBUG ((DEBUG_INFO, "FPGA:  V%x.%x\nPCBA:  V%x.0\n",
              FPGA_READ(FpgaVersionMajor),
              FPGA_READ(FpgaVersionMinor),
              FPGA_READ(PcbaVersion)));

  DEBUG ((DEBUG_INFO, "SERDES Reference Clocks:\n"));

  Sd1RefClkSel = FPGA_READ(Sd1RefClkSel);
  DEBUG((DEBUG_INFO, "SD1_CLK1 = %a, SD1_CLK2 = %a\n",
              Sd1RefClkSel ? SERDES_FREQ2 : SERDES_FREQ1, SERDES_FREQ1));

  return;
}

/**
   Function to get system clock frequency.
**/
UINTN
GetBoardSysClk (
  VOID
  )
{
  // TODO: read SYSCLK from clock generator using I2C
  return 100000000;
}

