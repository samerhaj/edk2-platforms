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
#include <DramInfo.h>
#include <Library/BaseLib.h>
#include <Library/BaseMemoryLib/MemLibInternals.h>
#include <Library/BeIoLib.h>
#include <Library/DebugLib.h>
#include <Library/FpgaLib.h>
#include <Library/IfcLib.h>
#include <Library/IoLib.h>
#include <Library/PcdLib.h>
#include <Library/PrintLib.h>
#include <Library/SerialPortLib.h>

#include "Soc.h"

/**
  Calculate the frequency of various controllers and
  populate the passed structure with frequuencies.

  @param  PtrSysInfo            Input structure to populate with
                                frequencies.
**/
VOID
GetSysInfo (
  OUT SYS_INFO *PtrSysInfo
  )
{
  CCSR_GUR     *GurBase;
  CCSR_CLOCK   *ClkBase;
  UINTN        CpuIndex;
  UINT32       TempRcw;
  UINT32       CPllSel;
  UINT32       CplxPll;
  CONST UINT8  CoreCplxPll[8] = {
    [0] = 0,    /* CC1 PPL / 1 */
    [1] = 0,    /* CC1 PPL / 2 */
    [4] = 1,    /* CC2 PPL / 1 */
    [5] = 1,    /* CC2 PPL / 2 */
  };

  CONST UINT8  CoreCplxPllDivisor[8] = {
    [0] = 1,    /* CC1 PPL / 1 */
    [1] = 2,    /* CC1 PPL / 2 */
    [4] = 1,    /* CC2 PPL / 1 */
    [5] = 2,    /* CC2 PPL / 2 */
  };

  UINTN        PllCount;
  UINTN        FreqCPll[NUM_CC_PLLS];
  UINTN        PllRatio[NUM_CC_PLLS];
  UINTN        SysClk;

  GurBase = (VOID *)PcdGet64 (PcdGutsBaseAddr);
  ClkBase = (VOID *)PcdGet64 (PcdClkBaseAddr);
  SysClk = CLK_FREQ;

  SetMem (PtrSysInfo, sizeof (SYS_INFO), 0);

  PtrSysInfo->FreqSystemBus = SysClk;
  PtrSysInfo->FreqDdrBus = SysClk;

  //
  // selects the platform clock:SYSCLK ratio and calculate
  // system frequency
  //
  PtrSysInfo->FreqSystemBus *= (GurRead ((UINTN)&GurBase->RcwSr[0]) >>
                CHASSIS2_RCWSR0_SYS_PLL_RAT_SHIFT) &
                CHASSIS2_RCWSR0_SYS_PLL_RAT_MASK;
  //
  // selects the DDR PLL:SYSCLK Ratio and calculate DDR frequency
  //
  PtrSysInfo->FreqDdrBus *= (GurRead ((UINTN)&GurBase->RcwSr[0]) >>
                CHASSIS2_RCWSR0_MEM_PLL_RAT_SHIFT) &
                CHASSIS2_RCWSR0_MEM_PLL_RAT_MASK;

  for (PllCount = 0; PllCount < NUM_CC_PLLS; PllCount++) {
    PllRatio[PllCount] = (GurRead ((UINTN)&ClkBase->PllCgSr[PllCount].PllCnGSr) >> 1) & 0xff;
    if (PllRatio[PllCount] > 4) {
      FreqCPll[PllCount] = SysClk * PllRatio[PllCount];
    } else {
      FreqCPll[PllCount] = PtrSysInfo->FreqSystemBus * PllRatio[PllCount];
    }
  }

  //
  // Calculate Core frequency
  //
  for (CpuIndex = 0; CpuIndex < MAX_CPUS; CpuIndex++) {
    CPllSel = (GurRead ((UINTN)&ClkBase->ClkcSr[CpuIndex].ClkCnCSr) >> 27) & 0xf;
    CplxPll = CoreCplxPll[CPllSel];

    PtrSysInfo->FreqProcessor[CpuIndex] = FreqCPll[CplxPll] / CoreCplxPllDivisor[CPllSel];
  }

  //
  // Calculate FMAN frequency
  //
  TempRcw = GurRead ((UINTN)&GurBase->RcwSr[7]);
  switch ((TempRcw & HWA_CGA_M1_CLK_SEL) >> HWA_CGA_M1_CLK_SHIFT) {
  case 2:
    PtrSysInfo->FreqFman[0] = FreqCPll[0] / 2;
    break;
  case 3:
    PtrSysInfo->FreqFman[0] = FreqCPll[0] / 3;
    break;
  case 4:
    PtrSysInfo->FreqFman[0] = FreqCPll[0] / 4;
    break;
  case 5:
    PtrSysInfo->FreqFman[0] = PtrSysInfo->FreqSystemBus;
    break;
  case 6:
    PtrSysInfo->FreqFman[0] = FreqCPll[1] / 2;
    break;
  case 7:
    PtrSysInfo->FreqFman[0] = FreqCPll[1] / 3;
    break;
  default:
    DEBUG ((DEBUG_WARN, "Error: Unknown FMan1 clock select!\n"));
    break;
  }
  PtrSysInfo->FreqSdhc = PtrSysInfo->FreqSystemBus/PcdGet32 (PcdPlatformFreqDiv);
  PtrSysInfo->FreqQman = PtrSysInfo->FreqSystemBus/PcdGet32 (PcdPlatformFreqDiv);
}

/**
   Function to select pins depending upon pcd using supplemental
   configuration unit(SCFG) extended RCW controlled pinmux control
   register which contains the bits to provide pin multiplexing control.
   This register is reset on HRESET.
 **/
VOID
ConfigScfgMux (VOID)
{
  CCSR_SCFG *Scfg;
  UINT32 UsbPwrFault;

  Scfg = (VOID *)PcdGet64 (PcdScfgBaseAddr);
  // Configures functionality of the IIC3_SCL to USB2_DRVVBUS
  // Configures functionality of the IIC3_SDA to USB2_PWRFAULT

  // LS1043A
  // Configures functionality of the IIC4_SCL to USB3_DRVVBUS
  // Configures functionality of the IIC4_SDA to USB3_PWRFAULT

  // LS1046A
  // USB3 is not used, configure mux to IIC4_SCL/IIC4_SDA
  if (PcdGetBool (PcdMuxToUsb3)) {
    BeMmioWrite32 ((UINTN)&Scfg->RcwPMuxCr0, CCSR_SCFG_RCWPMUXCRO_SELCR_USB);
  } else {
    BeMmioWrite32 ((UINTN)&Scfg->RcwPMuxCr0, CCSR_SCFG_RCWPMUXCRO_NOT_SELCR_USB);
  }
  BeMmioWrite32 ((UINTN)&Scfg->UsbDrvVBusSelCr, CCSR_SCFG_USBDRVVBUS_SELCR_USB1);
  UsbPwrFault = (CCSR_SCFG_USBPWRFAULT_DEDICATED <<
                CCSR_SCFG_USBPWRFAULT_USB3_SHIFT) |
                (CCSR_SCFG_USBPWRFAULT_DEDICATED <<
                CCSR_SCFG_USBPWRFAULT_USB2_SHIFT) |
                (CCSR_SCFG_USBPWRFAULT_SHARED <<
                CCSR_SCFG_USBPWRFAULT_USB1_SHIFT);
  BeMmioWrite32 ((UINTN)&Scfg->UsbPwrFaultSelCr, UsbPwrFault);
  BeMmioWrite32 ((UINTN)&Scfg->UsbPwrFaultSelCr, UsbPwrFault);
}

/**
  Function to initialize SoC specific constructs
  CPU Info
  SoC Personality
  Board Personality
  RCW prints
 **/
VOID
SocInit (
  VOID
  )
{
  CHAR8 Buffer[100];
  UINTN CharCount;

  SmmuInit ();

  //
  // Early init serial Port to get board information.
  //
  SerialPortInitialize ();
  CharCount = AsciiSPrint (Buffer, sizeof (Buffer), "\nUEFI firmware (version %s built at %a on %a)\n\r",
    (CHAR16*)PcdGetPtr (PcdFirmwareVersionString), __TIME__, __DATE__);
  SerialPortWrite ((UINT8 *) Buffer, CharCount);

  PrintCpuInfo ();

  //
  // Print Reset control Word
  //
  PrintRCW ();
  PrintSoc ();
  IfcInit ();
  PrintBoardPersonality ();
  //
  // Due to the extensive functionality present on the chip and the limited number of external
  // signals available, several functional blocks share signal resources through multiplexing.
  // In this case when there is alternate functionality between multiple functional blocks,
  // the signal's function is determined at the chip level (rather than at the block level)
  // typically by a reset configuration word (RCW) option. Some of the signals' function are
  // determined externel to RCW at Power-on Reset Sequence.
  //
  ConfigScfgMux ();


  return;
}

VOID
UpdateDpaaDram (
  IN OUT DRAM_INFO *DramInfo
  )
{
  return;
}

