/** @file
*
*  Copyright 2017-2018 NXP
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

#ifndef SOC_CLOCK_INTERNAL_LIB_H_
#define SOC_CLOCK_INTERNAL_LIB_H_

/**
  1024 bits Reset Configuration Word (RCW) layout

**/
typedef struct  {
  UINT8     SysPllCfg : 2;  // Bits 1-0
  UINT8     SysPllRat : 5;  // Bits 6-2
  UINT8     Reserved0 : 1;
  UINT8     Reserved1[35];
  UINT8     Reserved2 : 4;
  UINT8     SysClkFreqL : 4; // Bits 295-292
  UINT8     SysClkFreqH : 6; // Bits 301-296
  UINT8     Reserved3 : 2;
  UINT8     Reserved4[90];
} RCW_FIELDS;

#endif // SOC_CLOCK_INTERNAL_LIB_H_
