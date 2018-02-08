/** @Soc.c
  SoC specific Library containg functions to initialize various SoC components

  Copyright 2017 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution. The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include <Base.h>
#include <Chassis.h>
#include <Library/BaseLib.h>
#include <Library/BaseMemoryLib/MemLibInternals.h>
#include <Library/DebugLib.h>
#include <Library/IfcLib.h>
#include <Library/IoLib.h>
#include <Library/PcdLib.h>
#include <Library/PrintLib.h>
#include <Library/SerialPortLib.h>

#include "Soc.h"

extern VOID PrintBoardPersonality (VOID);
extern UINTN GetBoardSysClk (VOID);

VOID
GetSysInfo (
  OUT SYS_INFO *PtrSysInfo
  )
{
  UINT32 Index;
  CCSR_GUR *GurBase;
  CCSR_CLT_CTRL *ClkBase;
  CCSR_CLK_CLUSTER  *ClkGrp[2] = {
    (VOID *) (FSL_CLK_GRPA_ADDR),
    (VOID *) (FSL_CLK_GRPB_ADDR)
  };

  const UINT8 CoreCplxPll[16] = {
    [0] = 0,        // CC1 PPL / 1
    [1] = 0,        // CC1 PPL / 2
    [2] = 0,        // CC1 PPL / 4
    [4] = 1,        // CC2 PPL / 1
    [5] = 1,        // CC2 PPL / 2
    [6] = 1,        // CC2 PPL / 4
    [8] = 2,        // CC3 PPL / 1
    [9] = 2,        // CC3 PPL / 2
    [10] = 2,       // CC3 PPL / 4
    [12] = 3,       // CC4 PPL / 1
    [13] = 3,       // CC4 PPL / 2
    [14] = 3,       // CC4 PPL / 4
  };

  const UINT8 CoreCplxPllDivisor[16] = {
    [0] = 1,        // CC1 PPL / 1
    [1] = 2,        // CC1 PPL / 2
    [2] = 4,        // CC1 PPL / 4
    [4] = 1,        // CC2 PPL / 1
    [5] = 2,        // CC2 PPL / 2
    [6] = 4,        // CC2 PPL / 4
    [8] = 1,        // CC3 PPL / 1
    [9] = 2,        // CC3 PPL / 2
    [10] = 4,       // CC3 PPL / 4
    [12] = 1,       // CC4 PPL / 1
    [13] = 2,       // CC4 PPL / 2
    [14] = 4,       // CC4 PPL / 4
  };

  INT32 CcGroup[12] = FSL_CLUSTER_CLOCKS;
  UINTN PllCount;
  UINTN Cluster;
  UINTN FreqCPll[NUM_CC_PLLS];
  UINTN PllRatio[NUM_CC_PLLS];
  UINTN SysClk;
  UINT32 Cpu;
  UINT32 CPllSel;
  UINT32 CplxPll;
  VOID  *Offset;

  SetMem (PtrSysInfo, sizeof (SYS_INFO), 0);

  GurBase = (VOID *)PcdGet64 (PcdGutsBaseAddr);
  ClkBase = (VOID *)PcdGet64 (PcdClkBaseAddr);
  SysClk = GetBoardSysClk ();

  PtrSysInfo->FreqSystemBus = SysClk;
  PtrSysInfo->FreqDdrBus = PcdGet64 (PcdDdrClk);
  PtrSysInfo->FreqDdrBus2 = PcdGet64 (PcdDdrClk);

  //
  // selects the platform clock:SYSCLK ratio and calculate
  // system frequency
  //
  PtrSysInfo->FreqSystemBus *= (GurRead ((UINTN)&GurBase->RcwSr[0]) >>
      CHASSIS3_RCWSR_0_SYS_PLL_RAT_SHIFT) &
      CHASSIS3_RCWSR_0_SYS_PLL_RAT_MASK;

  //
  // Platform clock is half of platform PLL
  //
  PtrSysInfo->FreqSystemBus /= PcdGet32 (PcdPlatformFreqDiv);

  //
  // selects the DDR PLL:SYSCLK Ratio and calculate DDR frequency
  //
  PtrSysInfo->FreqDdrBus *= (GurRead ((UINTN)&GurBase->RcwSr[0]) >>
      CHASSIS3_RCWSR_0_MEM_PLL_RAT_SHIFT) &
      CHASSIS3_RCWSR_0_MEM_PLL_RAT_MASK;
  PtrSysInfo->FreqDdrBus2 *= (GurRead ((UINTN)&GurBase->RcwSr[0]) >>
      CHASSIS3_RCWSR_0_MEM2_PLL_RAT_SHIFT) &
      CHASSIS3_RCWSR_0_MEM2_PLL_RAT_MASK;

  for (PllCount = 0; PllCount < NUM_CC_PLLS; PllCount++) {
    Offset = (VOID *)((UINTN)ClkGrp[PllCount/3] +
        __builtin_offsetof (CCSR_CLK_CLUSTER, PllnGsr[PllCount%3].Gsr));
    PllRatio[PllCount] = (GurRead ((UINTN)Offset) >> 1) & 0x3f;
    FreqCPll[PllCount] = SysClk * PllRatio[PllCount];
  }

  //
  // Calculate Core frequency
  //
  ForEachCpu (Index, Cpu, CpuNumCores (), CpuMask ()) {
    Cluster = QoriqCoreToCluster (Cpu);
    ASSERT_EFI_ERROR (Cluster);
    CPllSel = (GurRead ((UINTN)&ClkBase->ClkCnCsr[Cluster].Csr) >> 27) & 0xf;
    CplxPll = CoreCplxPll[CPllSel];
    CplxPll += CcGroup[Cluster] - 1;
    PtrSysInfo->FreqProcessor[Cpu] = FreqCPll[CplxPll] / CoreCplxPllDivisor[CPllSel];
  }
  PtrSysInfo->FreqSdhc = PtrSysInfo->FreqSystemBus/PcdGet32 (PcdPlatformFreqDiv);
}

/**
  Perform the early initialization.
  This function is called by the ArmPlatformPkg/Pei or ArmPlatformPkg/Pei/PlatformPeim

**/
VOID
SocInit (
  VOID
  )
{
  CHAR8 Buffer[100];
  UINTN CharCount;

  //
  // Initialize SMMU
  //
  SmmuInit ();

  //
  // Perform IFC Initialization.
  // Early IFC initialization is required to set timings required for fpga initilzation to
  // get system clock frequency, board info etc.
  //
  IfcInit ();

  //
  //  Initialize the Serial Port.
  //  Early serial port initialization is required to print RCW, Soc and CPU infomation at
  //  the begining of UEFI boot.
  //
  SerialPortInitialize ();

  CharCount = AsciiSPrint (Buffer, sizeof (Buffer), "\nUEFI firmware (version %s built at %a on %a)\n\r",
      (CHAR16*)PcdGetPtr (PcdFirmwareVersionString), __TIME__, __DATE__);
  SerialPortWrite ((UINT8 *) Buffer, CharCount);

  //
  // Print CPU information
  //
  PrintCpuInfo ();

  //
  // Print Reset Controll Word
  //
  PrintRCW ();

  //
  // Print Soc Personality information
  //
  PrintSoc ();

  //
  // Print Board Personality information
  //
  PrintBoardPersonality ();
}

