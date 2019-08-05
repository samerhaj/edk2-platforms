/** @FpgaLib.c
  Fpga Library for LS1028A-RDB board, containing functions to
  program and read the Fpga registers.

  MMIO APIs are used to read/write FPGA registers

  Copyright 2019 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution. The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include <Base.h>
#include <Library/BaseLib.h>
#include <Library/FpgaLib.h>
#include <Library/IoLib.h>

/**
   Function to print board personality.

**/
VOID
PrintBoardPersonality (
  VOID
  )
{
// TO DO
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
  // TODO : read SYSCLK from clock generator using i2c bus
  return 100000000;
}
