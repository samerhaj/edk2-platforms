/** @file
  This file containa all erratas need to be applied on different SoCs.

  Copyright 2018 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution. The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include <Base.h>
#include <Library/ArmLib.h>
#include <Library/BaseLib.h>
#include <Library/BeIoLib.h>
#include <Library/DebugLib.h>
#include <Library/IoLib.h>
#include <Library/PcdLib.h>

#include "Soc.h"

#define I2C_DEBUG_REG 0x6
#define I2C_GLITCH_EN 0x8
/*
 * I2cErratumA009203 
 * This erratum improves clock stability by setting glitch_en bit
 * which enable digital glitch filter.
*/

VOID
 I2cErratumA009203 (
  VOID
  )
{

  UINTN I2cBaseAddr;
  UINT32 I2cSize;
  UINT8 Count;

  I2cBaseAddr = PcdGet64 (PcdI2c0BaseAddr);
  I2cSize = PcdGet32 (PcdI2cSize);

  for (Count = 0; Count < PcdGet32 (PcdNumI2cController) ; Count++) {
    MmioWrite8 ((I2cBaseAddr + I2cSize*Count + I2C_DEBUG_REG), I2C_GLITCH_EN);
  }
  return;
}


VOID
  ApplyErratum (
    VOID
   )
{

  I2cErratumA009203();
}
