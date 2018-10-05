/** @file
*  Header defining the structure for Dram Information
*
*  Copyright 2017 NXP
*
*  This program and the accompanying materials
*  are licensed and made available under the terms and conditions of the BSD License
*  which accompanies this distribution.  The full text of the license may be found at
*  http://opensource.org/licenses/bsd-license.php
*
*  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
*  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.
*
**/

#ifndef __DRAM_INFO_H__
#define __DRAM_INFO_H__

#include <Uefi/UefiBaseType.h>

#define SMC_DRAM_BANK_INFO          (0xC200FF12)

typedef struct {
  UINTN            BaseAddress;
  UINTN            Size;
} DRAM_REGION_INFO;

typedef struct {
  UINT32            NumOfDrams;
  UINT32            Reserved;
  DRAM_REGION_INFO  DramRegion[3];
} DRAM_INFO;

EFI_STATUS
GetDramBankInfo (
  IN OUT DRAM_INFO *DramInfo
  );

VOID
UpdateDpaaDram (
  IN OUT DRAM_INFO *DramInfo
  );

#endif /* __DRAM_INFO_H__ */
