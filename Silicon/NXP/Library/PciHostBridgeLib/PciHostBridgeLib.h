/** @file
  Include file for PCI Host Bridge Library instance for NXP SoCs.

  Copyright 2018 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#ifndef __PCIE_HOST_BRIDGE_LIB_H_
#define __PCIE_HOST_BRIDGE_LIB_H_

/* LUT registers */
#define PCIE_LUT_ENABLE		BIT31

typedef struct __LS_PCIE_LUT {
  UINT8        Reserved0[0x20];
  UINT32       PexLsr;   // PEX LUT Status Register
  UINT32       PexLcr;   // PEX LUT Control Register
  UINT8        Reserved28[0x800 - 0x28];
  struct {
    UINT32     PexLudr; // PEX LUT Entry Upper Data Register
    UINT32     PexLldr; // PEX LUT Entry Lower Data Register
  } PexLut[32];
} LS_PCIE_LUT;

typedef struct __LS_PCIE {
  ///
  /// Base Address of Controller registers
  ///
  UINTN          ControllerAddress;
  ///
  /// Next Available Look Up Table entry Index
  ///
  INT32          NextLutIndex;
  ///
  /// Look Up Table
  ///
  LS_PCIE_LUT    *LsPcieLut;
} LS_PCIE;

#endif // __PCIE_HOST_BRIDGE_LIB_H_
