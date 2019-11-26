/** @file
  SoC specific Library containg functions to initialize various SoC components

  Copyright 2017-2019 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution. The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include <Base.h>
#include <Library/ArmSmcLib.h>
#include <Library/BaseLib.h>
#include <Library/BeIoLib.h>
#include <Library/DebugLib.h>
#include <Library/IoLib.h>
#include <Library/PcdLib.h>
#include <Library/PrintLib.h>
#include <Library/SerialPortLib.h>

#include <Soc.h>

#include "Chassis.h"
#include "DramInfo.h"

UINT32
EFIAPI
GurRead (
  IN  UINTN     Address
  )
{
  if (FixedPcdGetBool (PcdGurBigEndian)) {
    return BeMmioRead32 (Address);
  } else {
    return MmioRead32 (Address);
  }
}

VOID
EFIAPI
GurWrite (
  IN  UINTN    Address,
  IN  UINT32   Value
  )
{
  if (FixedPcdGetBool (PcdGurBigEndian)) {
    BeMmioWrite32 (Address, Value);
  } else {
    MmioWrite32 (Address, Value);
  }
}

/*
 *  Structure to list available SOCs.
 */
STATIC CPU_TYPE CpuTypeList[] = {
  CPU_TYPE_ENTRY (LS1043A, LS1043A, 4),
  CPU_TYPE_ENTRY (LS1046A, LS1046A, 4),
  CPU_TYPE_ENTRY (LS2088A, LS2088A, 8),
  CPU_TYPE_ENTRY (LX2160A, LX2160A, 16),
  CPU_TYPE_ENTRY (LS1028A, LS1028A, 2),
};

/*
 * Return the number of bits set
 */
STATIC
inline
UINTN
CountSetBits (
  IN  UINTN  Num
  )
{
  UINTN Count;

  Count = 0;

  while (Num) {
    Count += Num & 1;
    Num >>= 1;
  }

  return Count;
}

/*
 * Return the type of initiator (core or hardware accelerator)
 */
UINT32
InitiatorType (
  IN UINT32 Cluster,
  IN UINTN  InitId
  )
{
  CCSR_GUR *GurBase;
  UINT32   Idx;
  UINT32   Type;

  GurBase = (VOID *)PcdGet64 (PcdGutsBaseAddr);
  Idx = (Cluster >> (InitId * 8)) & TP_CLUSTER_INIT_MASK;
  Type = GurRead ((UINTN)&GurBase->TpItyp[Idx]);

  if (Type & TP_ITYP_AV_MASK) {
    return Type;
  }

  return 0;
}

/*
 *  Return the mask for number of cores on this SOC.
 */
UINT32
CpuMask (
  VOID
  )
{
  CCSR_GUR  *GurBase;
  UINTN     ClusterIndex;
  UINTN     Count;
  UINT32    Cluster;
  UINT32    Type;
  UINT32    Mask;
  UINTN     InitiatorIndex;

  GurBase = (VOID *)PcdGet64 (PcdGutsBaseAddr);
  ClusterIndex = 0;
  Count = 0;
  Mask = 0;

  do {
    Cluster = GurRead ((UINTN)&GurBase->TpCluster[ClusterIndex].Lower);
    for (InitiatorIndex = 0; InitiatorIndex < TP_INIT_PER_CLUSTER; InitiatorIndex++) {
      Type = InitiatorType (Cluster, InitiatorIndex);
      if (Type) {
        if (TP_ITYP_TYPE_MASK (Type) == TP_ITYP_TYPE_ARM)
          Mask |= 1 << Count;
        Count++;
      }
    }
    ClusterIndex++;
  } while (CHECK_CLUSTER (Cluster));

  return Mask;
}

/*
 *  Return the number of cores on this SOC.
 */
UINTN
CpuNumCores (
  VOID
  )
{
    return CountSetBits (CpuMask ());
}

/*
 *  Return core's cluster
 */
UINT32
QoriqCoreToCluster (
  IN UINTN Core
  )
{
  CCSR_GUR  *GurBase;
  UINTN     ClusterIndex;
  UINTN     Count;
  UINT32    Cluster;
  UINT32    Type;
  UINTN     InitiatorIndex;

  GurBase = (VOID *)PcdGet64 (PcdGutsBaseAddr);
  ClusterIndex = 0;
  Count = 0;
  do {
    Cluster = GurRead ((UINTN)&GurBase->TpCluster[ClusterIndex].Lower);
    for (InitiatorIndex = 0; InitiatorIndex < TP_INIT_PER_CLUSTER; InitiatorIndex++) {
      Type = InitiatorType (Cluster, InitiatorIndex);
      if (Type) {
        if (Count == Core)
          return ClusterIndex;
        Count++;
      }
    }
    ClusterIndex++;
  } while (CHECK_CLUSTER (Cluster));

  return -1;      // cannot identify the cluster
}

/*
 *  Return the type of core i.e. A53, A57 etc of inputted
 *  core number.
 */
UINT32
QoriqCoreToType (
  IN UINTN Core
  )
{
  CCSR_GUR  *GurBase;
  UINTN     ClusterIndex;
  UINTN     Count;
  UINT32    Cluster;
  UINT32    Type;
  UINTN     InitiatorIndex;

  GurBase = (VOID *)PcdGet64 (PcdGutsBaseAddr);
  ClusterIndex = 0;
  Count = 0;

  do {
    Cluster = GurRead ((UINTN)&GurBase->TpCluster[ClusterIndex].Lower);
    for (InitiatorIndex = 0; InitiatorIndex < TP_INIT_PER_CLUSTER; InitiatorIndex++) {
      Type = InitiatorType (Cluster, InitiatorIndex);
      if (Type) {
        if (Count == Core)
          return Type;
        Count++;
      }
    }
    ClusterIndex++;
  } while (CHECK_CLUSTER (Cluster));

  return -1;      /* cannot identify the cluster */
}

/*
 * Print CPU information
 */
VOID
PrintCpuInfo (
  VOID
  )
{
  SYS_INFO SysInfo;
  UINTN    CoreIndex;
  UINTN    Core;
  UINT32   Type;
  CHAR8    Buffer[100];
  UINTN    CharCount;

  GetSysInfo (&SysInfo);
  CharCount = AsciiSPrint (Buffer, sizeof (Buffer), "Clock Configuration:");
  SerialPortWrite ((UINT8 *) Buffer, CharCount);

  ForEachCpu (CoreIndex, Core, CpuNumCores (), CpuMask ()) {
    if (!(CoreIndex % 3)) {
      CharCount = AsciiSPrint (Buffer, sizeof (Buffer), "\n      ");
      SerialPortWrite ((UINT8 *) Buffer, CharCount);
    }

    Type = TP_ITYP_VERSION (QoriqCoreToType (Core));
    CharCount = AsciiSPrint (Buffer, sizeof (Buffer), "CPU%d(%a):%-4d MHz  ", Core,
        Type == TY_ITYP_VERSION_A7 ? "A7 " :
        (Type == TY_ITYP_VERSION_A53 ? "A53" :
         (Type == TY_ITYP_VERSION_A57 ? "A57" :
          (Type == TY_ITYP_VERSION_A72 ? "A72" : " Unknown Core "))),
        SysInfo.FreqProcessor[Core] / MEGA_HZ);
    SerialPortWrite ((UINT8 *) Buffer, CharCount);
  }

  CharCount = AsciiSPrint (Buffer, sizeof (Buffer), "\n      Bus:      %-4d MHz  ",
                           SysInfo.FreqSystemBus / MEGA_HZ);
  SerialPortWrite ((UINT8 *) Buffer, CharCount);
  CharCount = AsciiSPrint (Buffer, sizeof (Buffer), "DDR:      %-4d MT/s",
                           SysInfo.FreqDdrBus / MEGA_HZ);
  SerialPortWrite ((UINT8 *) Buffer, CharCount);

  if (SysInfo.FreqFman[0] != 0) {
    CharCount = AsciiSPrint (Buffer, sizeof (Buffer), "\n      FMAN:     %-4d MHz  ",
                             SysInfo.FreqFman[0] / MEGA_HZ);
    SerialPortWrite ((UINT8 *) Buffer, CharCount);
  }

  CharCount = AsciiSPrint (Buffer, sizeof (Buffer), "\n");
  SerialPortWrite ((UINT8 *) Buffer, CharCount);
}

/*
 * Print Soc information
 */
VOID
PrintSoc (
  VOID
  )
{
  CHAR8    Buf[64];
  CCSR_GUR *GurBase;
  UINTN    Count;
  UINTN    Svr;
  UINTN    Ver;

  GurBase = (VOID *)PcdGet64 (PcdGutsBaseAddr);

  Buf[0] = L'\0';
  Svr = GurRead ((UINTN)&GurBase->Svr);
  Ver = SVR_SOC_VER (Svr);
  AsciiStrCpy (Buf, "SOC: ");

  for (Count = 0; Count < ARRAY_SIZE (CpuTypeList); Count++) {
    if ((CpuTypeList[Count].SocVer & SVR_WO_E) == Ver) {
      AsciiStrCat (Buf, (CONST CHAR8 *)CpuTypeList[Count].Name);

      if (Ver == (SVR_LX2160A & SVR_WO_E)) {
        if (!((Svr >> 12) & 0x1)) {
          AsciiStrCat (Buf, (CONST CHAR8 *)"C");
        }
      }

      if (IS_E_PROCESSOR (Svr)) {
        AsciiStrCat (Buf, (CONST CHAR8 *)"E");
      }

      AsciiSPrint (Buf + AsciiStrLen(Buf), sizeof (Buf) - AsciiStrLen(Buf),
                   " Rev%d.%d", SVR_MAJOR (Svr), SVR_MINOR (Svr));
      break;
    }
  }

  if (Count == ARRAY_SIZE (CpuTypeList)) {
    AsciiStrCat (Buf, (CONST CHAR8 *)"unknown");
  }

  AsciiSPrint (Buf + AsciiStrLen(Buf), sizeof (Buf) - AsciiStrLen(Buf), " (0x%x)\n", Svr);
  SerialPortWrite ((UINT8 *) Buf, AsciiStrLen(Buf));

  return;
}

/*
 * Dump RCW (Reset Control Word) on console
 */
VOID
PrintRCW (
  VOID
  )
{
  CCSR_GUR *Base;
  UINTN    Count;
  CHAR8    Buffer[100];
  UINTN    CharCount;

  Base = (VOID *)PcdGet64 (PcdGutsBaseAddr);

  /*
   * Display the RCW, so that no one gets confused as to what RCW
   * we're actually using for this boot.
   */

  CharCount = AsciiSPrint (Buffer, sizeof (Buffer),
               "Reset Configuration Word (RCW):");
  SerialPortWrite ((UINT8 *) Buffer, CharCount);
  for (Count = 0; Count < ARRAY_SIZE(Base->RcwSr); Count++) {
    UINT32 Rcw = BeMmioRead32((UINTN)&Base->RcwSr[Count]);

    if ((Count % 4) == 0) {
      CharCount = AsciiSPrint (Buffer, sizeof (Buffer),
                   "\n       %08x:", Count * 4);
      SerialPortWrite ((UINT8 *) Buffer, CharCount);
    }

    CharCount = AsciiSPrint (Buffer, sizeof (Buffer), " %08x", Rcw);
    SerialPortWrite ((UINT8 *) Buffer, CharCount);
  }

  CharCount = AsciiSPrint (Buffer, sizeof (Buffer), "\n");
  SerialPortWrite ((UINT8 *) Buffer, CharCount);
}

/*
 * Setup SMMU in bypass mode
 * and also set its pagesize
 */
VOID
SmmuInit (
  VOID
  )
{
  UINT32 Value;

  /* set pagesize as 64K and ssmu-500 in bypass mode */
  Value = (MmioRead32 ((UINTN)SMMU_REG_SACR) | SACR_PAGESIZE_MASK);
  MmioWrite32 ((UINTN)SMMU_REG_SACR, Value);

  Value = (MmioRead32 ((UINTN)SMMU_REG_SCR0) | SCR0_CLIENTPD_MASK) & ~SCR0_USFCFG_MASK;
  MmioWrite32 ((UINTN)SMMU_REG_SCR0, Value);

  Value = (MmioRead32 ((UINTN)SMMU_REG_NSCR0) | SCR0_CLIENTPD_MASK) & ~SCR0_USFCFG_MASK;
  MmioWrite32 ((UINTN)SMMU_REG_NSCR0, Value);
}

/*
 * Return current Soc Name form CpuTypeList
 */
CHAR8 *
GetSocName (
  VOID
  )
{
  UINT8     Count;
  UINTN     Svr;
  UINTN     Ver;
  CCSR_GUR  *GurBase;

  GurBase = (VOID *)PcdGet64 (PcdGutsBaseAddr);

  Svr = GurRead ((UINTN)&GurBase->Svr);
  Ver = SVR_SOC_VER (Svr);

  for (Count = 0; Count < ARRAY_SIZE (CpuTypeList); Count++) {
    if ((CpuTypeList[Count].SocVer & SVR_WO_E) == Ver) {
      return (CHAR8 *)CpuTypeList[Count].Name;
    }
  }

  return NULL;
}

UINTN
GetDramSize (
  IN VOID
  )
{
  ARM_SMC_ARGS  ArmSmcArgs;

  ArmSmcArgs.Arg0 = SMC_DRAM_BANK_INFO;
  ArmSmcArgs.Arg1 = -1;

  ArmCallSmc (&ArmSmcArgs);

  if (ArmSmcArgs.Arg0) {
    return 0;
  } else {
    return ArmSmcArgs.Arg1;
  }
}

EFI_STATUS
GetDramBankInfo (
  IN OUT DRAM_INFO *DramInfo
  )
{
  ARM_SMC_ARGS  ArmSmcArgs;
  UINT32        I;
  UINTN         DramSize;

  DramSize = GetDramSize ();
  DEBUG ((DEBUG_INFO, "DRAM Total Size 0x%lx \n", DramSize));

  // Ensure DramSize has been set
  ASSERT (DramSize != 0);

  I = 0;

  do {
    ArmSmcArgs.Arg0 = SMC_DRAM_BANK_INFO;
    ArmSmcArgs.Arg1 = I;

    ArmCallSmc (&ArmSmcArgs);
    if (ArmSmcArgs.Arg0) {
      if (I > 0) {
        break;
      } else {
        ASSERT (ArmSmcArgs.Arg0 == 0);
      }
    }

    DramInfo->DramRegion[I].BaseAddress = ArmSmcArgs.Arg1;
    DramInfo->DramRegion[I].Size = ArmSmcArgs.Arg2;

    DramSize -= DramInfo->DramRegion[I].Size;

    DEBUG ((DEBUG_INFO, "bank[%d]: start 0x%lx, size 0x%lx\n",
      I, DramInfo->DramRegion[I].BaseAddress, DramInfo->DramRegion[I].Size));

    I++;
  } while (DramSize);

  DramInfo->NumOfDrams = I;

  DEBUG ((DEBUG_INFO, "Number Of DRAM in system %d \n", DramInfo->NumOfDrams));

  UpdateDpaaDram (DramInfo);

  return EFI_SUCCESS;
}
