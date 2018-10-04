/** SecureMonRngLib.c
  RNG Library to provide get random number from SEC.

  Copyright 2017-2018 NXP

  This program and the accompanying materials are licensed and made available
  under the terms and conditions of the BSD License which accompanies this
  distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include <IndustryStandard/ArmStdSmc.h>
#include <Library/ArmSmcLib.h>
#include <Library/BaseMemoryLib/MemLibInternals.h>
#include <Library/SecureMonRngLib.h>

EFI_STATUS
getRawRng (
    IN  UINTN    RNGLength,
    OUT UINT8   *RNGValue
  )
{
  UINT8 I;
  ARM_SMC_ARGS ArmSmcArgs;

  I = 0;

  // Send command to get RNG value
  if (SIZE_4_BYTE == RNGLength) {
    ArmSmcArgs.Arg0 = SIP_RNG_32;
    ArmSmcArgs.Arg1 = GET_32BIT_VALUE;
  } else if (SIZE_8_BYTE == RNGLength) {
    ArmSmcArgs.Arg0 = SIP_RNG_64;
    ArmSmcArgs.Arg1 = GET_64BIT_VALUE;
  }

  ArmCallSmc (&ArmSmcArgs);

  if (ArmSmcArgs.Arg0) {
    DEBUG ((DEBUG_ERROR, "Failed to get Random number \n"));
    return EFI_DEVICE_ERROR;
  }

  CopyMem (RNGValue,(UINT8 *)&ArmSmcArgs.Arg1, RNGLength);

  for (I = 0; I < RNGLength; I++) {
    DEBUG ((DEBUG_INFO, "RNGValue[%d] 0x%x \n", I, *(RNGValue+I)));
  }

  return EFI_SUCCESS;
}
