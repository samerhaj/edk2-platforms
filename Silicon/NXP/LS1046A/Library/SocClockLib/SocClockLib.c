/** @file

 Copyright 2017-2018 NXP

 This program and the accompanying materials
 are licensed and made available under the terms and conditions of the BSD License
 which accompanies this distribution.  The full text of the license may be found at
 http://opensource.org/licenses/bsd-license.php

 THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
 WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

 **/

#include <Soc.h>
#include <Library/BeIoLib.h>
#include <Library/DebugLib.h>
#include <Library/SocClockLib.h>

#include "SocClockInternalLib.h"

/**
  Return the input clock frequency to an IP Module.
  If a module is disabled or doesn't exist on platform, then return zero.

  @param[in]  IpModule   The IP module whose input clock frequency is needed.
  @param[in]  Instance   The Instance of IP module whose input clock frequency is needed.
                         if there are multiple modules of same type then this value tells the
                         instance of module for which clock is to be retrieved.
                         (e.g. if there are four i2c controllers in SOC, then this value can be 1, 2, 3, 4)
                         for IP modules which have only single instance in SOC (e.g. one QSPI controller)
                         this value should be 0.

  @return      > 0       Return the input clock frequency to an IP Module
                0        either IP module doesn't exist in SOC
                         or IP module instance doesn't exist in SOC
                         or IP module instance is disabled. i.e. no input clock is provided to IP module instance.
**/
UINT64
SocGetClock (
  IN  IP_MODULES  IpModule,
  IN  UINT32      Instance
  )
{
  CCSR_GUR     *GurBase;
  CCSR_SCFG    *Scfg;
  RCW_FIELDS   *Rcw;
  UINT64       ClusterGroupA;
  UINT64       ReturnValue;
  UINT16       SysClkFreqMultiplier;
  UINT32       ConfigRegister; // device configuration register. can be used for any device

  if (IpModule >= IP_MAX) {
    return 0;
  }

  GurBase = (VOID *)PcdGet64 (PcdGutsBaseAddr);
  Scfg    = (VOID *)PcdGet64 (PcdScfgBaseAddr);
  ASSERT ((GurBase != NULL) && (Scfg != NULL));

  Rcw = (RCW_FIELDS *)GurBase->RcwSr;
  ReturnValue = 0;

  // SysClkFreq comprises of 10 bits. 8 bits SysClkFreqH and 2 bits SysClkFreqL
  SysClkFreqMultiplier = ((UINT16)Rcw->SysClkFreqH) << 2 | Rcw->SysClkFreqL;

  switch (IpModule) {
    case IP_QSPI:
      ConfigRegister = BeMmioRead32 ( (UINTN)&Scfg->QspiCfg);
      if (ConfigRegister & QSPI_CLOCK_DISABLE) {
        break;
      }

      switch (Rcw->HwaCgaM2ClkSel) {
        case 1:
        case 2:
        case 3:
          ClusterGroupA = ((UINT64)Rcw->CgaPll2Rat * (UINT64)SysClkFreqMultiplier * 250000 * 2) /
                          (Rcw->HwaCgaM2ClkSel * 3);
          break;
        case 6:
          ClusterGroupA = (UINT64)Rcw->CgaPll1Rat * (UINT64)SysClkFreqMultiplier * 250000 / 3;
          break;
        default:
          ClusterGroupA = 0;
          break;
      }

      if (ClusterGroupA) {
        switch ((ConfigRegister & 0xF0000000) >> 28) { // CLK_SEL bits in SCFG_QSPI_CFG
          case 0:
            ReturnValue = ClusterGroupA >> 8; // Divide by 256
            break;
          case 1:
            ReturnValue = ClusterGroupA >> 6; // Divide by 64
            break;
          case 2:
            ReturnValue = ClusterGroupA >> 5; // Divide by 32
            break;
          case 3:
            ReturnValue = (UINT64)ClusterGroupA / 24; // Divide by 24
            break;
          case 4:
            ReturnValue = (UINT64)ClusterGroupA / 20; // Divide by 20
            break;
          case 5:
            ReturnValue = ClusterGroupA >> 4; // Divide by 16
            break;
          case 6:
            ReturnValue = (UINT64)ClusterGroupA / 12; // Divide by 12
            break;
          case 7:
            ReturnValue = ClusterGroupA >> 3; // Divide by 8
            break;
          default:
            break;
        }
      }
      break;
    default:
      break;
  }

  return ReturnValue;
}
