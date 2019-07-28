/** @FpgaLib.c
  Fpga Library for LX2160A-CEX7 module, containing functions to
  program and read the Fpga registers.

  FPGA is connected to IFC Controller and so MMIO APIs are used
  to read/write FPGA registers

  Copyright 2018 NXP

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
#include <Library/I2c.h>

/**
   Function to read FPGA register.

   @param  Reg  Register offset of FPGA to read.

**/
UINT8
FpgaRead (
  IN  UINTN  Reg
  )
{
  return 0;
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
}

/**
   Function to get board system clock frequency.

**/
UINTN
GetBoardSysClk (
  VOID
  )
{
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
  DEBUG ((DEBUG_INFO, "LX2160A COM express type 7 module\n"));
}
