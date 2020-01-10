/*
 * Copyright 2018 INPHI
 * Copyright 2019 NXP
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must Statusain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Inphi is a Registered trademark of Inphi Corporation
 *
 */

#include <Uefi.h>
#include <Library/UefiLib.h>
#include <Library/DpaaDebugLib.h>
#include <Library/BaseMemoryLib/MemLibInternals.h>
#include <Library/IoLib.h>
#include <Library/TimerLib.h>
#include <Library/UefiBootServicesTableLib.h>

#include "Dpaa2EthernetPhyPrivate.h"
#include "Inphi112525.h"

BOOLEAN gLaneRecovery;
IN112525_S03_VCO_CODES S03VcoCodes;
STATIC EFI_EVENT Dpaa2PhyPeriodicTimer;

IN112525_CONFIG  InphiS03Config = {
  .EnableOtuProtocol = 0,
  .EnableExternalRefclk = 0,
  .EnablePrescaler = 0,
  .TxPllMpyRatio = 10,
  .EnableHalfRate = 0,
  .EnableExtendedRange = 0,
  .TxPllRefclkSource = RECOV_CLK,
  .CtleMode = MODE_25_25_10,
  .RxCommonMode = 3,
  .RxOdtOverride = 0,
  .L0PhaseAdjustVal = IN112525_PHASE_ADJUST_VAL,
  .L1PhaseAdjustVal = IN112525_PHASE_ADJUST_VAL,
  .L2PhaseAdjustVal = IN112525_PHASE_ADJUST_VAL,
  .L3PhaseAdjustVal = IN112525_PHASE_ADJUST_VAL
};

/* lookup table to map the multiply options and MUX selects to the bit Values */
UINT8 TxPllMpyMap[][2] = {
  /* MPY  MS, LS */
  [10] = {0,  0},
  [20] = {1,  0},
  [40] = {2,  0},
  [8]  = {0,  1},
  [16] = {1,  1},
  [32] = {2,  1},
  [33] = {1,  6},
  [66] = {2,  6},
  [15] = {0,  7},
  [30] = {1,  7},
  [60] = {2,  7}
};

INT32
BitTest (
  IN  INT32  Value,
  IN  INT32  BitField
  )
{
  INT32 BitMask;

  BitMask = (1 << BitField);
  INT32 Result;

  Result = ((Value & BitMask) == BitMask);
  return Result;
}

EFI_STATUS
In112525UploadFirmware (
  IN  DPAA2_PHY       *Dpaa2Phy
  )
{
  IN112525_REG_CONFIG FwRegConfig;
  CHAR8               LineTemp[0x50];
  CHAR8               RegAddr[0x50];
  CHAR8               Data[0x50];
  INT32               I;
  INT32               LineCount;
  INT32               ColumnCount;
  CHAR8               *Addr;
  CHAR8               *TokenStartPtr;
  CHAR8               *TokenEndPtr;
  UINT8               TokenLength;

  LineCount = 0;
  ColumnCount = 0;
  Addr = (CHAR8 *)PcdGet64 (PcdIn112525FwNorBaseAddr);;

  DPAA_INFO_MSG ("Loading Inphi firmware from NOR flash address"
                 " 0x%p for PHY address 0x%x ...\n", Addr,
                 Dpaa2Phy->PhyAddress);
  while (*Addr != 'Q') {
    I = 0;
    while (*Addr != 0xa) {
      LineTemp[I++] = *Addr++;
      if (I > 0x50) {
        DPAA_ERROR_MSG ("Line %d in Inphi Firmware is too long\n",
                        LineCount + 1);
        return EFI_INVALID_PARAMETER;
      }
    }

    Addr++;  // skip '\n'
    LineCount++;
    ColumnCount = I;
    LineTemp[ColumnCount] = '\0';

    if (LineCount > PcdGet64 (PcdIn112525FwSize)) {
      DPAA_ERROR_MSG ("Inphi Firmware larger than expected\n");
      return EFI_INVALID_PARAMETER;

    }

    TokenStartPtr = SkipSeparators (LineTemp);
    if (*TokenStartPtr == '\0') {
      DPAA_ERROR_MSG ("InValid blank line found in Inphi Firmware\n");
      return EFI_INVALID_PARAMETER;
    }

    TokenEndPtr = FindNextSeparator (TokenStartPtr);
    TokenLength = TokenEndPtr - TokenStartPtr;
    CopyMem (RegAddr, TokenStartPtr, TokenLength);
    RegAddr[TokenLength] = '\0';

    TokenStartPtr = SkipSeparators (TokenEndPtr);
    if (*TokenStartPtr == '\0') {
      DPAA_ERROR_MSG ("Incomplete line found in Inphi Firmware\n");
      return EFI_INVALID_PARAMETER;
    }

    TokenEndPtr = FindNextSeparator (TokenStartPtr);
    TokenLength = TokenEndPtr - TokenStartPtr;
    CopyMem (Data, TokenStartPtr, TokenLength);
    Data[TokenLength] = '\0';

    FwRegConfig.Addr = AsciiStrHexToUintn (RegAddr);
    FwRegConfig.Value = AsciiStrHexToUintn (Data);
    Dpaa2PhyRegisterWrite (Dpaa2Phy,
      MDIO_MMD_VEND1, FwRegConfig.Addr, FwRegConfig.Value);
  }

  return EFI_SUCCESS;
}

VOID
RxResetAssert (
  IN  DPAA2_PHY *Dpaa2Phy,
  IN  INT32     Lane
  )
{
  INT32         Mask;
  INT32         Val;

  if (Lane == NUMBER_OF_LANES) {
    Val = Dpaa2PhyRegisterRead (Dpaa2Phy, MDIO_MMD_VEND1, PHYMISC_REG2);
    Mask = (1 << 15);
    Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, PHYMISC_REG2, Val + Mask);
  } else {
    Val = Dpaa2PhyRegisterRead (Dpaa2Phy, MDIO_MMD_VEND1,
      Lane * 0x100 + PHYCTRL_REG8);
    Mask = (1 << 6);
    Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1,
      Lane * 0x100 + PHYCTRL_REG8, Val + Mask);
  }
}

VOID
RxResetDeAssert  (
  IN  DPAA2_PHY *Dpaa2Phy,
  IN  INT32     Lane
  )
{
  INT32         Mask;
  INT32         Val;

  if (Lane == NUMBER_OF_LANES) {
    Val = Dpaa2PhyRegisterRead (Dpaa2Phy, MDIO_MMD_VEND1, PHYMISC_REG2);
    Mask = 0xffff - (1 << 15);
    Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, PHYMISC_REG2, Val & Mask);
  } else {
    Val = Dpaa2PhyRegisterRead (Dpaa2Phy, MDIO_MMD_VEND1,
            Lane * 0x100 + PHYCTRL_REG8);
    Mask = 0xffff - (1 << 6);
    Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1,
      Lane * 0x100 + PHYCTRL_REG8, Val & Mask);
  }
}

INT32
AzCompleteTest (
  IN  DPAA2_PHY *Dpaa2Phy,
  IN  INT32     Lane
  )
{
  INT32         Success;
  INT32         Value;

  Success = 1;

  if (Lane == 0 || Lane == NUMBER_OF_LANES) {
    Value = Dpaa2PhyRegisterRead (Dpaa2Phy, MDIO_MMD_VEND1, PHYCTRL_REG5);
    Success = Success & BitTest(Value, 2);
  }
  if (Lane == 1 || Lane == NUMBER_OF_LANES) {
    Value = Dpaa2PhyRegisterRead (Dpaa2Phy, MDIO_MMD_VEND1, PHYCTRL_REG5 + 0x100);
    Success = Success & BitTest(Value, 2);
  }
  if (Lane == 2 || Lane == NUMBER_OF_LANES) {
    Value = Dpaa2PhyRegisterRead (Dpaa2Phy, MDIO_MMD_VEND1, PHYCTRL_REG5 + 0x200);
    Success = Success & BitTest(Value, 2);
  }
  if (Lane == 3 || Lane == NUMBER_OF_LANES) {
    Value = Dpaa2PhyRegisterRead (Dpaa2Phy, MDIO_MMD_VEND1, PHYCTRL_REG5 + 0x300);
    Success = Success & BitTest(Value, 2);
  }

  return Success;
}

VOID
SaveAzOffsets (
  IN  DPAA2_PHY *Dpaa2Phy,
  IN  INT32     Lane
  )
{
  INT32         I;

#define AZ_OFFSET_LANE_UPDATE(Reg, Lane) \
  Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, (Reg) + (Lane) * 0x100,  \
    (Dpaa2PhyRegisterRead (Dpaa2Phy, MDIO_MMD_VEND1, (Reg) + (Lane) * 0x100) >> 8))

  if (Lane == NUMBER_OF_LANES) {
    for (I = 0; I < NUMBER_OF_LANES; I++) {
      AZ_OFFSET_LANE_UPDATE (PHYMISC_REG20, I);
      AZ_OFFSET_LANE_UPDATE (PHYMISC_REG20 + 1, I);
      AZ_OFFSET_LANE_UPDATE (PHYMISC_REG20 + 2, I);
      AZ_OFFSET_LANE_UPDATE (PHYMISC_REG20 + 3, I);
      AZ_OFFSET_LANE_UPDATE (PHYMISC_REG21, I);
      AZ_OFFSET_LANE_UPDATE (PHYMISC_REG21 + 1, I);
      AZ_OFFSET_LANE_UPDATE (PHYMISC_REG21 + 2, I);
      AZ_OFFSET_LANE_UPDATE (PHYMISC_REG21 + 3, I);
      AZ_OFFSET_LANE_UPDATE (PHYMISC_REG22, I);
    }
  } else {
    AZ_OFFSET_LANE_UPDATE (PHYMISC_REG20, Lane);
    AZ_OFFSET_LANE_UPDATE (PHYMISC_REG20 + 1, Lane);
    AZ_OFFSET_LANE_UPDATE (PHYMISC_REG20 + 2, Lane);
    AZ_OFFSET_LANE_UPDATE (PHYMISC_REG20 + 3, Lane);
    AZ_OFFSET_LANE_UPDATE (PHYMISC_REG21, Lane);
    AZ_OFFSET_LANE_UPDATE (PHYMISC_REG21 + 1, Lane);
    AZ_OFFSET_LANE_UPDATE (PHYMISC_REG21 + 2, Lane);
    AZ_OFFSET_LANE_UPDATE (PHYMISC_REG21 + 3, Lane);
    AZ_OFFSET_LANE_UPDATE (PHYMISC_REG22, Lane);
  }

  Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, PHYCTRL_REG7, 0x0001);
}

VOID
TxPllAssert (
  IN  DPAA2_PHY *Dpaa2Phy,
  IN  INT32     Lane
  )
{
  INT32         Val;
  INT32         Recal;

  if (Lane == NUMBER_OF_LANES) {
    Val = Dpaa2PhyRegisterRead (Dpaa2Phy, MDIO_MMD_VEND1, PHYMISC_REG2);
    Recal = (1 << 12);
    Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, PHYMISC_REG2, Val | Recal);
  } else {
    Val = Dpaa2PhyRegisterRead (Dpaa2Phy, MDIO_MMD_VEND1, Lane * 0x100 + PHYCTRL_REG4);
    Recal = (1 << 15);
    Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, Lane * 0x100 + PHYCTRL_REG4, Val | Recal);
  }
}

VOID
TxPllDeAssert (
  IN  DPAA2_PHY *Dpaa2Phy,
  IN  INT32     Lane
  )
{
  INT32         Val;
  INT32         Recal;

  if (Lane == NUMBER_OF_LANES) {
    Val = Dpaa2PhyRegisterRead (Dpaa2Phy, MDIO_MMD_VEND1, PHYMISC_REG2);
    Recal = 0xefff;
    Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, PHYMISC_REG2, Val & Recal);
  } else {
    Val = Dpaa2PhyRegisterRead (Dpaa2Phy, MDIO_MMD_VEND1, Lane * 0x100 + PHYCTRL_REG4);
    Recal = 0x7fff;
    Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, Lane * 0x100 + PHYCTRL_REG4, Val & Recal);
  }
}

VOID
TxCoreAssert (
  IN  DPAA2_PHY *Dpaa2Phy,
  IN  INT32     Lane
  )
{
  INT32         Recal;
  INT32         Val;
  INT32         Val2;
  INT32         CoreReset;

  if (Lane == 4) {
    Val = Dpaa2PhyRegisterRead (Dpaa2Phy, MDIO_MMD_VEND1, PHYMISC_REG2);
    Recal = 1 << 10;
    Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, PHYMISC_REG2, Val | Recal);
  } else {
    Val2 = Dpaa2PhyRegisterRead (Dpaa2Phy, MDIO_MMD_VEND1, PHYMISC_REG3);
    CoreReset = (1 << (Lane + 8));
    Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, PHYMISC_REG3, Val2 | CoreReset);
  }
}

VOID
TxCoreDeAssert (
  IN  DPAA2_PHY *Dpaa2Phy,
  IN  INT32     Lane
  )
{
  INT32         Recal;
  INT32         Val;
  INT32         Val2;
  INT32         CoreReset;

  if (Lane == NUMBER_OF_LANES) {
    Val = Dpaa2PhyRegisterRead (Dpaa2Phy, MDIO_MMD_VEND1, PHYMISC_REG2);
    Recal = 0xffff - (1 << 10);
    Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, PHYMISC_REG2, Val & Recal);
  } else {
    Val2 = Dpaa2PhyRegisterRead (Dpaa2Phy, MDIO_MMD_VEND1, PHYMISC_REG3);
    CoreReset = 0xffff - (1 << (Lane + 8));
    Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, PHYMISC_REG3, Val2 & CoreReset);
  }
}

VOID
TxRestart (
  IN  DPAA2_PHY *Dpaa2Phy,
  IN  INT32     Lane
  )
{
  TxCoreAssert (Dpaa2Phy, Lane);
  TxPllAssert (Dpaa2Phy, Lane);
  TxPllDeAssert (Dpaa2Phy, Lane);

  MicroSecondDelay (1500);
  TxCoreDeAssert(Dpaa2Phy, Lane);
}

INT32
TxPllLockTest (
  IN  DPAA2_PHY *Dpaa2Phy,
  IN  INT32     Lane
  )
{
  INT32         I;
  INT32         Val;
  INT32         Locked;

  Locked = 1;

  if (Lane == NUMBER_OF_LANES) {
    for (I = 0; I < NUMBER_OF_LANES; I++) {
      Val = Dpaa2PhyRegisterRead (Dpaa2Phy, MDIO_MMD_VEND1, I * 0x100 + PHYSTAT_REG3);
      Locked = Locked & BitTest(Val, 15);
    }
  } else {
    Val = Dpaa2PhyRegisterRead (Dpaa2Phy, MDIO_MMD_VEND1, Lane * 0x100 + PHYSTAT_REG3);
    Locked = Locked & BitTest(Val, 15);
  }

  return Locked;
}

VOID
SaveVcoCodes (
  IN  DPAA2_PHY *Dpaa2Phy,
  IN  INT32     Lane
  )
{
  INT32         Value0;
  INT32         Value1;
  INT32         Value2;
  INT32         Value3;

  if (Lane == 0 || Lane == NUMBER_OF_LANES) {
    Value0 = Dpaa2PhyRegisterRead (Dpaa2Phy, MDIO_MMD_VEND1, PHYMISC_REG5);
    Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1,
      PHYMISC_REG7, Value0 + IN112525_RX_VCO_CODE_OFFSET);
    S03VcoCodes.L0VcoCode = Value0;
  }
  if (Lane == 1 || Lane == NUMBER_OF_LANES) {
    Value1 = Dpaa2PhyRegisterRead (Dpaa2Phy, MDIO_MMD_VEND1, PHYMISC_REG5 + 0x100);
    Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, PHYMISC_REG7 + 0x100,
      Value1 + IN112525_RX_VCO_CODE_OFFSET);
    S03VcoCodes.L1VcoCode = Value1;
  }
  if (Lane == 2 || Lane == NUMBER_OF_LANES) {
    Value2 = Dpaa2PhyRegisterRead (Dpaa2Phy, MDIO_MMD_VEND1, PHYMISC_REG5 + 0x200);
    Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, PHYMISC_REG7 + 0x200,
      Value2 + IN112525_RX_VCO_CODE_OFFSET);
    S03VcoCodes.L2VcoCode = Value2;
  }
  if (Lane == 3 || Lane == NUMBER_OF_LANES) {
    Value3 = Dpaa2PhyRegisterRead (Dpaa2Phy, MDIO_MMD_VEND1, PHYMISC_REG5 + 0x300);
    Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, PHYMISC_REG7 + 0x300,
      Value3 + IN112525_RX_VCO_CODE_OFFSET);
    S03VcoCodes.L3VcoCode = Value3;
  }
}

EFI_STATUS
In112525S03LaneRecovery (
  IN  DPAA2_PHY *Dpaa2Phy,
  IN  INT32     Lane
  )
{
  INT32         I;
  INT32         Value;
  INT32         AzPass;

  switch (Lane) {
  case 0:
  case 1:
  case 2:
  case 3:
    RxResetAssert (Dpaa2Phy, Lane);
    MicroSecondDelay (20000);
    break;
  case NUMBER_OF_LANES:
    Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, PHYMISC_REG2, 0x9C00);
    MicroSecondDelay (20000);
    while (1) {
      Value = Dpaa2PhyRegisterRead (Dpaa2Phy, MDIO_MMD_VEND1, PHYMISC_REG2);
      if (BitTest(Value, 4))
        break;
    }
    break;
  default:
    DPAA_ERROR_MSG ("Incorrect usage of APIs in Inphi driver\n");
    break;
  }

  if (Lane == 0 || Lane == NUMBER_OF_LANES ) {
    Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, PHYMISC_REG7, L0_VCO_CODE_trim);
  }
  if (Lane == 1 || Lane == NUMBER_OF_LANES) {
    Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, PHYMISC_REG7 + 0x100, L1_VCO_CODE_trim);
  }
  if (Lane == 2 || Lane == NUMBER_OF_LANES) {
    Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, PHYMISC_REG7 + 0x200, L2_VCO_CODE_trim);
  }
  if (Lane == 3 || Lane == NUMBER_OF_LANES) {
    Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, PHYMISC_REG7 + 0x300, L3_VCO_CODE_trim);
  }

  if (Lane == 0 || Lane == 4) {
    Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, PHYCTRL_REG5, 0x0418);
  }
  if (Lane == 1 || Lane == 4) {
    Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, PHYCTRL_REG5 + 0x100, 0x0418);
  }
  if (Lane == 2 || Lane == 4) {
    Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, PHYCTRL_REG5 + 0x200, 0x0418);
  }
  if (Lane == 3 || Lane == 4) {
    Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, PHYCTRL_REG5 + 0x300, 0x0418);
  }
  Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, PHYCTRL_REG7, 0x0000);
  RxResetDeAssert (Dpaa2Phy, Lane);

  if (Lane == 0 || Lane == 4) {
    Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, PHYCTRL_REG5, 0x0410);
    Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, PHYCTRL_REG5, 0x0412);
  }
  if (Lane == 1 || Lane == 4) {
    Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, PHYCTRL_REG5 + 0x100, 0x0410);
    Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, PHYCTRL_REG5 + 0x100, 0x0412);
  }
  if (Lane == 2 || Lane == 4) {
    Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, PHYCTRL_REG5 + 0x200, 0x0410);
    Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, PHYCTRL_REG5 + 0x200, 0x0412);
  }
  if (Lane == 3 || Lane == 4) {
    Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, PHYCTRL_REG5 + 0x300, 0x0410);
    Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, PHYCTRL_REG5 + 0x300, 0x0412);
  }

  for (I = 0; I < 64; I++) {
    /* wait 1000 times 10us */
    MicroSecondDelay (100000);
    AzPass = AzCompleteTest (Dpaa2Phy, Lane);
    if (AzPass) {
      SaveAzOffsets (Dpaa2Phy, Lane);
      break;
    }
  }

  if (!AzPass) {
    DPAA_ERROR_MSG ("auto-zero calibration timed out for Lane %d\n", Lane);
    return EFI_SUCCESS;
  }

  Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, Lane * 0x100 + PHYMISC_REG4, 0x0002);
  Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, Lane * 0x100 + PHYMISC_REG6, 0x2028);
  Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, Lane * 0x100 + PHYCTRL_REG5, 0x0010);
  MicroSecondDelay (1000);
  Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, Lane * 0x100 + PHYCTRL_REG5, 0x0110);
  MicroSecondDelay (30000);
  Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, Lane * 0x100 + PHYMISC_REG6, 0x3020);

  if (Lane == NUMBER_OF_LANES) {
    Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, PHYMISC_REG2, 0x1C00);
    Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, PHYMISC_REG2, 0x0C00);
  } else {
    TxRestart(Dpaa2Phy, Lane);
    /* delay > 10ms is required */
    MicroSecondDelay (11000);
  }

  if (Lane == NUMBER_OF_LANES) {
    if (BitTest(Dpaa2PhyRegisterRead (Dpaa2Phy, MDIO_MMD_VEND1, PHYMISC_REG2), 6) == 0) {
      return EFI_DEVICE_ERROR;
    }
  } else {
    if (TxPllLockTest (Dpaa2Phy,Lane) == 0) {
      return EFI_DEVICE_ERROR;
    }
  }

  SaveVcoCodes (Dpaa2Phy, Lane);

  if (Lane == NUMBER_OF_LANES) {
    Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, PHYMISC_REG2, 0x0400);
    Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, PHYMISC_REG2, 0x0000);
    Value = Dpaa2PhyRegisterRead (Dpaa2Phy, MDIO_MMD_VEND1, PHYCTRL_REG1);
    Value = Value & 0xffbf;
    Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, PHYCTRL_REG2, Value);
  } else {
    TxCoreDeAssert (Dpaa2Phy, Lane);
  }

  if (Lane == NUMBER_OF_LANES) {
    Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, PHYMISC_REG1, 0x8000);
    Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, PHYMISC_REG1, 0x0000);
  }

  Dpaa2PhyRegisterRead (Dpaa2Phy, MDIO_MMD_VEND1, PHYMISC_REG1);
  Dpaa2PhyRegisterRead (Dpaa2Phy, MDIO_MMD_VEND1, PHYMISC_REG1);

  MicroSecondDelay (1000);
  Dpaa2PhyRegisterRead (Dpaa2Phy, MDIO_MMD_VEND1, PHYSTAT_REG1);
  Dpaa2PhyRegisterRead (Dpaa2Phy, MDIO_MMD_VEND1, PHYSTAT_REG2);

  DEBUG ((DEBUG_ERROR, "TX PLL Lock Success for lane %d \n", Lane));

  if (TxPllLockTest (Dpaa2Phy, NUMBER_OF_LANES)) {
    DEBUG ((DEBUG_INFO, "All PLLs Locked \n"));
    gLaneRecovery = TRUE;
  }

  return EFI_SUCCESS;
}

VOID
Dpaa2PhyTimerHandler (
    IN EFI_EVENT  Event,
    IN VOID       *Context
    )
{
  if (!gLaneRecovery) {
    In112525S03LaneRecovery ((DPAA2_PHY *)Context, NUMBER_OF_LANES);
  }
}

EFI_STATUS
In112525S03PhyConfig (
  IN  DPAA2_PHY *Dpaa2Phy
  )
{
  EFI_STATUS    Status;
  UINT32        I;
  UINT32        RegValue;
  UINT32        Reg;
  INT32         TxPllMsdivValue;
  INT32         TxPllLsdivValue;
  INT32         TxPllIqDiv;
  INT32         TxPllCtrl2Value;

  /* put the chip in hw/sw  and MDIO reset */
  Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, PHYCTRL_REG0,
    IN112525_HRESET | IN112525_SRESET | IN112525_MDIOINIT);

  /* de-assert MDIO init */
  Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, PHYCTRL_REG0,
    IN112525_HRESET | IN112525_SRESET);

  /* apply configuration */
  if (InphiS03Config.EnablePrescaler) {
    Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, IN112525_PRESCALE_20M, 0x0001);
  }

  if (InphiS03Config.EnableExternalRefclk) {
    Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, PHYMISC_REG3, (1 << 15));
  } else {
    Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, PHYMISC_REG3, 0x0);
  }

  Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, PHYCTRL_REG0, IN112525_SRESET);

  MicroSecondDelay(10000);

  RegValue = Dpaa2PhyRegisterRead (Dpaa2Phy, MDIO_MMD_VEND1, IN112525_EFUSE_REG);
  if (!(RegValue & IN112525_EFUSE_DONE)) {
    DPAA_ERROR_MSG ("IN112525_s03 init failed: EFUSE Done not set\n");
    return EFI_DEVICE_ERROR;
  }
  MicroSecondDelay(10000);

  RegValue = Dpaa2PhyRegisterRead (Dpaa2Phy, MDIO_MMD_VEND1, PHYMISC_REG2);
  if (!(RegValue & IN112525_CALIBRATION_DONE)) {
    DPAA_ERROR_MSG ("IN112525_s03 init failed: CALIBRATION_DONE not set\n");
    return EFI_DEVICE_ERROR;
  }

  Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, PHYMISC_REG2,
    IN112525_RX_PLL_RESET |
    IN112525_TX_PLL_RESET |
    IN112525_TX_SERDES_RESET |
    IN112525_CORE_DATAPATH_RESET);

  if (InphiS03Config.EnableOtuProtocol) {
    Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, PHYCTRL_REG0, 0x8C00);
  } else {
    Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, PHYCTRL_REG0, 0x8200);
  }

  if (InphiS03Config.EnableExtendedRange) {
    Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, PHYCTRL_REG10, 0x2032);
    DPAA_INFO_MSG ("IN112525_s03 possible misconfig [ext range]\n");
    Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, PHYCTRL_REG12, 0x0007);
  } else {
    Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, PHYCTRL_REG10, 0xA02D);
    Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, PHYCTRL_REG12, 0x0005);
  }

  Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, PHYCTRL_REG18, 0x00ff);
  Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, PHYCTRL_REG19, 0x002d);
  Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, PHYCTRL_REG19, 0x802d);
  Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, PHYCTRL_REG19, 0x0000);
  Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, PHYCTRL_REG18, 0x00e9);
  Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, PHYCTRL_REG19, 0x0008);
  Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, PHYCTRL_REG19, 0x8008);
  Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, PHYCTRL_REG19, 0x0000);

  TxPllMsdivValue = TxPllMpyMap [InphiS03Config.TxPllMpyRatio][0];
  TxPllLsdivValue = TxPllMpyMap [InphiS03Config.TxPllMpyRatio][1];
  TxPllIqDiv = (InphiS03Config.EnableHalfRate) ? 1 : 0;
  TxPllCtrl2Value =
    (InphiS03Config.TxPllRefclkSource << 11) +
    (TxPllIqDiv << 8) +
    (TxPllMsdivValue << 4) +
    TxPllLsdivValue;

  Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, PHYCTRL_REG11, TxPllCtrl2Value);

  if (InphiS03Config.EnableHalfRate) {
    Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, PHYCTRL_REG14, 0x0020);
  }

  /* set the CTLE mode (bw on the front end stages)
   * for example '25:25:10', '10:10:10' etc.
   */
  for (I = 0; I < NUMBER_OF_LANES; I++) {
    Reg = I * 0x100 + PHYCTRL_REG8;
    RegValue = Dpaa2PhyRegisterRead (Dpaa2Phy, MDIO_MMD_VEND1, Reg);
    RegValue = RegValue & 0xFF7C;
    /* put bits 7,1,0 for EQ */
    RegValue = RegValue | InphiS03Config.CtleMode;
    Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, Reg, RegValue);
  }

  /* rx common code settings */
  Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1,
    PHYMISC_REG32, InphiS03Config.RxCommonMode);
  Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1,
    PHYMISC_REG32 + 0x100, InphiS03Config.RxCommonMode);
  Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1,
    PHYMISC_REG32 + 0x200, InphiS03Config.RxCommonMode - 1);
  Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1,
    PHYMISC_REG32 + 0x300, InphiS03Config.RxCommonMode - 1);

  if (InphiS03Config.RxOdtOverride) {
    Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1,
      PHYMISC_REG31, InphiS03Config.RxOdtOverride);
  }

  S03VcoCodes.L0VcoCode = Dpaa2PhyRegisterRead (Dpaa2Phy, MDIO_MMD_VEND1,
                            PHYMISC_REG7);
  S03VcoCodes.L1VcoCode = Dpaa2PhyRegisterRead (Dpaa2Phy, MDIO_MMD_VEND1,
                            PHYMISC_REG7 + 0x100);
  S03VcoCodes.L2VcoCode = Dpaa2PhyRegisterRead (Dpaa2Phy, MDIO_MMD_VEND1,
                            PHYMISC_REG7 + 0x200);
  S03VcoCodes.L3VcoCode = Dpaa2PhyRegisterRead (Dpaa2Phy, MDIO_MMD_VEND1,
                            PHYMISC_REG7 + 0x300);

  if (InphiS03Config.EnableExtendedRange) {
    S03VcoCodes.L0VcoCode = (INT32)((S03VcoCodes.L0VcoCode * 8 / 10));
    S03VcoCodes.L1VcoCode = (INT32)((S03VcoCodes.L1VcoCode * 8) / 10);
    S03VcoCodes.L2VcoCode = (INT32)((S03VcoCodes.L2VcoCode * 8) / 10);
    S03VcoCodes.L3VcoCode = (INT32)((S03VcoCodes.L3VcoCode * 8) / 10);
  }

  Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, PHYMISC_REG2, 0x0);
  MicroSecondDelay (100000);

  /* start fresh */
  In112525S03LaneRecovery (Dpaa2Phy, NUMBER_OF_LANES);

  Status = gBS->CreateEvent (
                  EVT_TIMER | EVT_NOTIFY_SIGNAL,
                  TPL_NOTIFY,
                  Dpaa2PhyTimerHandler,
                  Dpaa2Phy,
                  &Dpaa2PhyPeriodicTimer
                  );
  if (EFI_ERROR (Status)) {
    DPAA_ERROR_MSG ("Failed to create Inphi PeriodicTimer event (error %u)\n", Status);
    return Status;
  }

  Status = gBS->SetTimer (
                  Dpaa2PhyPeriodicTimer,
                  TimerPeriodic,
                  EFI_TIMER_PERIOD_SECONDS(5)
                  );
  if (EFI_ERROR (Status)) {
    DPAA_ERROR_MSG ("Failed to set Inphi PeriodicTimer (error %u)\n", Status);
    return Status;
  }

  return EFI_SUCCESS;
}

EFI_STATUS
In112525S05PhyConfig (
  IN  DPAA2_PHY *Dpaa2Phy
  )
{
  EFI_STATUS    Status;
  UINT32        Val;
  UINT32        Addr;
  UINT32        L0Vc0Code;
  UINT32        L1Vc0Code;
  UINT32        L2Vc0Code;
  UINT32        L3Vc0Code;

  Dpaa2PhyRegisterWrite (Dpaa2Phy,
    MDIO_MMD_VEND1, PHYMISC_REG11, 0);

  Dpaa2PhyRegisterWrite (Dpaa2Phy,
    MDIO_MMD_VEND1, PHYMISC_REG11, IN112525_FORCE_PC);

  Dpaa2PhyRegisterWrite (Dpaa2Phy,
    MDIO_MMD_VEND1, PHYMISC_REG11, IN112525_LOL_CTRL);

  Dpaa2PhyRegisterWrite (Dpaa2Phy,
    MDIO_MMD_VEND1, PHYCTRL_REG0,
    (IN112525_MDIOINIT | IN112525_HRESET | IN112525_SRESET));

  Dpaa2PhyRegisterWrite (Dpaa2Phy,
    MDIO_MMD_VEND1, PHYCTRL_REG0,
    (IN112525_HRESET | IN112525_SRESET));

  Dpaa2PhyRegisterWrite (Dpaa2Phy,
    MDIO_MMD_VEND1, PHYMISC_REG3,
    (IN112525_TX_RF_EN | IN112525_CTLE_10G));

  Dpaa2PhyRegisterWrite (Dpaa2Phy,
    MDIO_MMD_VEND1, PHYCTRL_REG0, (IN112525_SRESET));

  MicroSecondDelay(10);

  Val = Dpaa2PhyRegisterRead (Dpaa2Phy, MDIO_MMD_VEND1, IN112525_EFUSE_REG);
  if (!(Val & IN112525_EFUSE_DONE)) {
    DPAA_ERROR_MSG ("IN112525 phy init failed: EFUSE Done not set\n");
    return EFI_DEVICE_ERROR;
  }

  MicroSecondDelay(10);

  Val = Dpaa2PhyRegisterRead (Dpaa2Phy, MDIO_MMD_VEND1, PHYMISC_REG2);
  if (!(Val & IN112525_CALIBRATION_DONE)) {
    DPAA_ERROR_MSG ("IN112525 phy init failed: CAL DONE not set\n");
    return EFI_DEVICE_ERROR;
  }

  Dpaa2PhyRegisterWrite (Dpaa2Phy,
    MDIO_MMD_VEND1, PHYMISC_REG2,
    (IN112525_RX_PLL_RESET | IN112525_TX_PLL_RESET |
     IN112525_CORE_DATAPATH_RESET | IN112525_TX_SERDES_RESET));

  Dpaa2PhyRegisterWrite (Dpaa2Phy,
    MDIO_MMD_VEND1, PHYCTRL_REG0,
    (IN112525_MANUALRESET_SELECT | IN112525_SRESET));

  Dpaa2PhyRegisterWrite (Dpaa2Phy,
    MDIO_MMD_VEND1, PHYCTRL_REG15, PHYCTRL_REG15_VAL);

  Dpaa2PhyRegisterWrite (Dpaa2Phy,
    MDIO_MMD_VEND1, PHYCTRL_REG17, PHYCTRL_REG17_VAL);

  Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, PHYCTRL_REG18, 0xff);
  Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, PHYCTRL_REG19, 0x2d);
  Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, PHYCTRL_REG19, 0x802d);
  Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, PHYCTRL_REG19, 0x0);

  Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, PHYCTRL_REG18, 0xe9);
  Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, PHYCTRL_REG19, 0x8);
  Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, PHYCTRL_REG19, 0x8008);
  Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, PHYCTRL_REG19, 0x0);

  Dpaa2PhyRegisterWrite (Dpaa2Phy,
    MDIO_MMD_VEND1, PHYCTRL_REG16,
    (IN112525_TXPLL_MSDIV | IN112525_TXPLL_IQDIV));

  Dpaa2PhyRegisterWrite (Dpaa2Phy,
    MDIO_MMD_VEND1, PHYCTRL_REG13,
    PHYCTRL_REG13_VAL | IN112525_LOSD_HYSTERESIS_EN);

  Dpaa2PhyRegisterWrite (Dpaa2Phy,
    MDIO_MMD_VEND1, PHYCTRL_REG20,
    (IN112525_RX_LOS_EN | IN112525_RX_LOS_100G_THRESHOLD));

  Dpaa2PhyRegisterWrite (Dpaa2Phy,
    MDIO_MMD_VEND1, PHYMISC_REG8, IN112525_FA_WIN_SIZE);

  Dpaa2PhyRegisterWrite (Dpaa2Phy,
    MDIO_MMD_VEND1, PHYCTRL_REG2, 0x5000);
  Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, 0X501, 0x0200);
  Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, 0X510, 0x001F);
  Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, 0X517, 0x803F);

  L0Vc0Code = Dpaa2PhyRegisterRead (Dpaa2Phy,
                MDIO_MMD_VEND1, PHYMISC_REG7);
  L1Vc0Code = Dpaa2PhyRegisterRead (Dpaa2Phy,
                MDIO_MMD_VEND1, PHYMISC_REG7 + 0x100);
  L2Vc0Code = Dpaa2PhyRegisterRead (Dpaa2Phy,
                MDIO_MMD_VEND1, PHYMISC_REG7 + 0x200);
  L3Vc0Code = Dpaa2PhyRegisterRead (Dpaa2Phy,
                MDIO_MMD_VEND1, PHYMISC_REG7 + 0x300);

  Status = In112525UploadFirmware (Dpaa2Phy);
  if (Status) {
    DPAA_ERROR_MSG ("IN112525 phy init failed: upload firmware failed\n");
    return EFI_DEVICE_ERROR;
  }

  Addr = IN112525_US_DATA_MEM_ADDR;

  Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, Addr, L3Vc0Code);
  Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, (Addr - 1), L2Vc0Code);
  Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, (Addr - 2), L1Vc0Code);
  Dpaa2PhyRegisterWrite (Dpaa2Phy, MDIO_MMD_VEND1, (Addr - 3), L0Vc0Code);

  Dpaa2PhyRegisterWrite (Dpaa2Phy,
    MDIO_MMD_VEND1, (Addr - 4), IN112525_PHASE_ADJUST_VAL);

  Dpaa2PhyRegisterWrite (Dpaa2Phy,
    MDIO_MMD_VEND1, (Addr - 5), IN112525_PHASE_ADJUST_VAL);

  Dpaa2PhyRegisterWrite (Dpaa2Phy,
    MDIO_MMD_VEND1, (Addr - 6), IN112525_PHASE_ADJUST_VAL);

  Dpaa2PhyRegisterWrite (Dpaa2Phy,
    MDIO_MMD_VEND1, (Addr - 7), IN112525_PHASE_ADJUST_VAL);

  Dpaa2PhyRegisterWrite (Dpaa2Phy,
    MDIO_MMD_VEND1, PHYMISC_REG12, IN112525_USEQ_FL);

  Dpaa2PhyRegisterWrite (Dpaa2Phy,
    MDIO_MMD_VEND1, PHYMISC_REG11,
    (IN112525_LOL_CTRL | IN112525_USEQ_EN));

  return EFI_SUCCESS;
}

/**
   Start Inphi PHY

   @param[in] Dpaa2Phy    Pointer to PHY object

   @Status EFI_SUCCESS    on Success
   @Status error code     on failure

 **/
EFI_STATUS
In112525S05PhyStartup (
  IN  OUT  DPAA2_PHY *Dpaa2Phy
  )
{
  Dpaa2Phy->LinkUp = TRUE;
  Dpaa2Phy->Speed = 10000;
  Dpaa2Phy->FullDuplex = TRUE;

  return EFI_SUCCESS;
}

/**
   Start Inphi PHY

   @param[in] Dpaa2Phy    Pointer to PHY object

   @Status EFI_SUCCESS    on Success
   @Status error code     on failure

 **/
EFI_STATUS
In112525S03PhyStartup (
  IN  OUT  DPAA2_PHY *Dpaa2Phy
  )
{
  Dpaa2Phy->LinkUp = TRUE;
  Dpaa2Phy->Speed = 10000;
  Dpaa2Phy->FullDuplex = TRUE;

  /* start fresh */
  if (!gLaneRecovery) {
    In112525S03LaneRecovery (Dpaa2Phy, NUMBER_OF_LANES);
  }

  return EFI_SUCCESS;
}
