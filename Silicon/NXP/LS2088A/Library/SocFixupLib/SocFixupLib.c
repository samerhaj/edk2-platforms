/** @file

 Copyright 2018 NXP

 This program and the accompanying materials
 are licensed and made available under the terms and conditions of the BSD License
 which accompanies this distribution.  The full text of the license may be found at
 http://opensource.org/licenses/bsd-license.php

 THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
 WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include <libfdt.h>
#include <Chassis.h>
#include <Soc.h>
#include <Library/DebugLib.h>
#include <Library/SocFixupLib.h>

/**
  Fixup the device tree based on running SOC's properties.

  @param[in]  Dtb   The device tree to fixup.

  @return EFI_SUCCESS       Successfully fix up the device tree
  @return EFI_DEVICE_ERROR  Could not fixup the device tree
  @return EFI_NOT_FOUND     Could not found the device tree overlay file
  @retval EFI_CRC_ERROR     Device tree overlay file is not correct.
**/
EFI_STATUS
FdtSocFixup (
  IN  VOID  *Dtb
  )
{
  return EFI_SUCCESS;
}

/**
  Retrive the System Version from System Version Register (SVR)

  @return  0  could not read SVR register or invalid value in SVR register
  @return     SVR register value
**/
UINT32
SocGetSvr (
  )
{
  CCSR_GUR     *GurBase;

  GurBase = (VOID *)PcdGet64 (PcdGutsBaseAddr);
  ASSERT (GurBase != NULL);

  return GurRead ( (UINTN)&GurBase->Svr);
}
