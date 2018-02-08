/** @FpgaLib.c
  Fpga Library for LS2088A-RDB board, containing functions to
  program and read the Fpga registers.

  FPGA is connected to IFC Controller and so MMIO APIs are used
  to read/write FPGA registers

  Copyright 2017-2018 NXP

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

/**
   Function to read FPGA register.

   @param  Reg  Register offset of FPGA to read.

**/
UINT8
FpgaRead (
  IN  UINTN  Reg
  )
{
  VOID       *Base;

  Base = (VOID *)FPGA_BASE_PHYS;

  return MmioRead8 ((UINTN)(Base + Reg));
}

/**
   Function to write FPGA register.

   @param  Reg   Register offset of FPGA to write.
   @param  Value Value to be written.

**/
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

/**
   Function to get board system clock frequency.

**/
UINTN
GetBoardSysClk (
  VOID
  )
{
  UINT8 SysclkConf;
  SysclkConf = FPGA_READ (BrdCfg[1]);
  switch (SysclkConf & FPGA_CLK_MASK) {
    case CLK_66:
      return SYSCLK_66_MHZ;
    case CLK_83:
      return SYSCLK_83_MHZ;
    case CLK_100:
      return SYSCLK_100_MHZ;
    case CLK_125:
      return SYSCLK_125_MHZ;
    case CLK_133:
      return SYSCLK_133_MHZ;
  }
  return SYSCLK_100_MHZ;
}

/**
   Function to print board personality.

**/
VOID
PrintBoardPersonality (
  VOID
  )
{
  UINT8 SwitchConf;
  SwitchConf = FPGA_READ (Arch);

  DEBUG ((DEBUG_INFO, "Board Arch: V%d, ", SwitchConf >> 4));
  DEBUG ((DEBUG_INFO, "Board version: %c, boot from ",
        (SwitchConf & 0xf) + 'A'));

  SwitchConf = FPGA_READ (BrdCfg[0]);

  if (SwitchConf & FPGA_CS_MASK)
    DEBUG ((DEBUG_INFO, "NAND\n"));
  else
    DEBUG ((DEBUG_INFO,  "vBank: %d\n", (SwitchConf & FPGA_VBANK_MASK)));

  DEBUG ((DEBUG_INFO, "FPGA: v%d.%d\n", FPGA_READ (Ver),
        FPGA_READ (Minor)));
}
