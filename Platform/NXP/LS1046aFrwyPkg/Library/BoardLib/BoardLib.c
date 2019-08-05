/** @file

  Copyright 2019 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include <IfcBoardSpecific.h>
#include <IfcNand.h>

VOID
GetIfcNorFlashTimings (
  IN IFC_TIMINGS * NorIfcTimings
  )
{

  return ;
}

VOID
GetIfcFpgaTimings (
  IN IFC_TIMINGS  *FpgaIfcTimings
  )
{

  return;
}

VOID
GetIfcNandFlashTimings (
  IN IFC_TIMINGS * NandIfcTimings
  )
{
  return;
}

VOID
GetIfcNandFlashInfo (
  IN NAND_FLASH_INFO *NandFlashInfo
  )
{

  return;
}

VOID
GetIfcNandBufBase (
 VOID* BufBase
 )
{
  return;
}
