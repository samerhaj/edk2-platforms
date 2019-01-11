/** @Soc.c
  SoC specific Library containg functions to initialize various SoC components

  Copyright 2018 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution. The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include <Base.h>
#include <Chassis.h>
#include <DramInfo.h>
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
    (void *)(FSL_CLK_GRPA_ADDR),
    (void *)(FSL_CLK_GRPB_ADDR)
  };

  const UINT8 CoreCplxPll[8] = {
    [0] = 0,        /* CC1 PLL / 1 */
    [1] = 0,        /* CC1 PLL / 2 */
    [2] = 0,        /* CC1 PLL / 4 */
    [4] = 1,        /* CC2 PLL / 1 */
    [5] = 1,        /* CC2 PLL / 2 */
    [6] = 1,        /* CC2 PLL / 4 */
  };

  const UINT8 CoreCplxPllDivisor[8] = {
    [0] = 1,        /* CC1 PLL Sync Mode */
    [1] = 2,        /* CC1 PLL / 1       */
    [2] = 4,        /* CC1 PLL / 2       */
    [4] = 1,        /* CC2 PLL / 1       */
    [5] = 2,        /* CC2 PLL / 2       */
    [6] = 4,        /* CC2 PLL / 4       */
  };

  UINTN PllCount, Cluster;
  UINTN FreqCPll[NUM_CC_PLLS];
  UINTN PllRatio[NUM_CC_PLLS];
  UINTN SysClk;

  INT32 CcGroup[8] = FSL_CLUSTER_CLOCKS;
  UINT32 Cpu;
  UINT32 CPllSel;
  UINT32 CplxPll;
  VOID *OffSet;

  GurBase = (VOID *)PcdGet64 (PcdGutsBaseAddr);
  ClkBase = (VOID *)PcdGet64 (PcdClkBaseAddr);
  InternalMemZeroMem (PtrSysInfo, sizeof (SYS_INFO));

  SysClk = GetBoardSysClk ();

  PtrSysInfo->FreqSystemBus = SysClk;
  PtrSysInfo->FreqDdrBus = PcdGet64 (PcdDdrClk);
  PtrSysInfo->FreqDdrBus2 = PcdGet64 (PcdDdrClk);

  PtrSysInfo->FreqSystemBus *= (MmioRead32 ((UINTN)&GurBase->RcwSr[0]) >>
      CHASSIS3_RCWSR_0_SYS_PLL_RAT_SHIFT) &
      CHASSIS3_RCWSR_0_SYS_PLL_RAT_MASK;

  /* Platform clock is half of platform PLL */
  PtrSysInfo->FreqSystemBus /= 2;

  PtrSysInfo->FreqDdrBus *= (MmioRead32 ((UINTN)&GurBase->RcwSr[0]) >>
      CHASSIS3_RCWSR_0_MEM_PLL_RAT_SHIFT) &
      CHASSIS3_RCWSR_0_MEM_PLL_RAT_MASK;

  /* For GEN2 DDR */
  PtrSysInfo->FreqDdrBus *= DDR_GEN2_PHY_MULTIPLIER /
    (((MmioRead32 ((UINTN)&GurBase->RcwSr[0]) >>
       CHASSIS3_RCWSR_0_MEM_PLL_CFG_SHIFT) &
       CHASSIS3_RCWSR_0_MEM_PLL_CFG_MASK) + 1);

  PtrSysInfo->FreqDdrBus2 *= (MmioRead32 ((UINTN)&GurBase->RcwSr[0]) >>
      CHASSIS3_RCWSR_0_MEM2_PLL_RAT_SHIFT) &
      CHASSIS3_RCWSR_0_MEM2_PLL_RAT_MASK;

  /// CPU clocking has two clock groups A and B
  /// Each group has 2 PLL1. Therefore we got 4 PLL in total
  /// Each PLL has fixed outout x, x/2 and x/4
  /// Any PLL of given group (A/B) can be connected to given
  /// Cluster. So each cluster can select clock from possible 6
  /// Clock sources
  /// Clock groups A can fed Cluster 1-4 whereas clock groups B
  /// Fed Cluster 4-8.
  /*
            Clock groups
              ----
             | P  | --> PLL1
    sysclk ->| L  | --> PLL1/2
             | L  | --> PLL1/4
             | 1  |
              ----

              ----
             | P  | --> PLL2
    sysclk ->| L  | --> PLL2/2
             | L  | --> PLL2/4
             | 2  |
              ----

      Clock group A/B follow same structure as above

    PLL1 and PLL2 clocks are fed to Mux
    csr register will determine output clock to Cluster
              ----
   PLL1  ->  |    |
   PLL1/2 -> | M  | --> To Cluster
   PLL1/4 -> | U  |
   PLL2   -> | X  |
   PLL2/2 -> |    |
   PLL2/4 -> |    |
              ----
   */

  /// First get the PLL outputs PLL1 and PLL2 for each
  /// clock group
  /// i.e FreqCPll[0] and FreqCPll[1] are PLL1 and PLL2 freq for clock group A
  /// FreqCPll[2] and FreqCPll[3] are PLL1 and PLL2 freq for clock group B
  for (PllCount = 0; PllCount < NUM_CC_PLLS; PllCount++) {
    OffSet = (VOID *)((UINTN)ClkGrp[PllCount/2] +
        OFFSET_OF (CCSR_CLK_CLUSTER, PllnGsr[PllCount%2].Gsr));
    PllRatio[PllCount] = (MmioRead32 ((UINTN)OffSet) >> 1) & 0x3f;
    FreqCPll[PllCount] = SysClk * PllRatio[PllCount];
  }

  ForEachCpu(Index, Cpu, CpuNumCores(), CpuMask()) {
    Cluster = QoriqCoreToCluster(Cpu);
    ASSERT_EFI_ERROR (Cluster);
    /// Read the mux to check which clock is selected for this Cluster
    // CPllSel 0, 1, 2 means PLL-1 /2 or /4
    // CPllSel 4, 5, 6 menas PLL-2 /2 or /4
    CPllSel = (MmioRead32 ((UINTN)&ClkBase->ClkCnCsr[Cluster].Csr) >> 27) & 0xf;

    // Get if this CPU is clocked from PLL-1 or PLL-2
    CplxPll = CoreCplxPll[CPllSel];

    // Get the CCG A or  B
    // Meaning Cluster 1-4 are sourced from CCG A (final value 0 or 1)
    // and Cluster 5-8 are sourced from CCG B     (Final value 2 or 3)
    CplxPll += CcGroup[Cluster];
    PtrSysInfo->FreqProcessor[Cpu] = FreqCPll[CplxPll] / CoreCplxPllDivisor[CPllSel];
  }
  PtrSysInfo->FreqSdhc = PtrSysInfo->FreqSystemBus/2;
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

  // Apply Erratum
  ApplyErratum();

  //
  // Initialize SMMU
  //
  SmmuInit ();

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

VOID
UpdateDpaaDram (
  IN OUT DRAM_INFO *DramInfo
  )
{
  UINT8            I;

  if (FixedPcdGet64 (PcdDpaa2McLowRamSize)) {
    DramInfo->DramRegion[0].Size -= FixedPcdGet64 (PcdDpaa2McLowRamSize);
    //
    // To align address boundary with DPAA2 requirements
    // 
    DramInfo->DramRegion[0].Size &= 0xE0000000;
  }

  if (FixedPcdGet64 (PcdDpaa2McHighRamSize)) {
      DramInfo->DramRegion[1].Size -= FixedPcdGet64 (PcdDpaa2McHighRamSize);
  }

  for (I = 0;  I < DramInfo->NumOfDrams; I++) {
    DEBUG ((DEBUG_INFO, "BANK[%d]: start 0x%lx, size 0x%lx\n",
      I, DramInfo->DramRegion[I].BaseAddress, DramInfo->DramRegion[I].Size));
  }

  return;
}
