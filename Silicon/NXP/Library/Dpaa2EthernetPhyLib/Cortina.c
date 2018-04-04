/** Cortina.c
  Cortina PHY services implementation

  Copyright 2017 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution. The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include <Bitops.h>
#include <Library/BaseLib.h>
#include <Library/BaseMemoryLib/MemLibInternals.h>
#include <Library/DpaaDebugLib.h>
#include <Library/IoLib.h>
#include <Library/PcdLib.h>
#include <Library/TimerLib.h>

#include "Dpaa2EthernetPhyPrivate.h"

#define VILLA_GLOBAL_CHIP_ID_LSB           0x000
#define VILLA_GLOBAL_CHIP_ID_MSB           0x001
#define VILLA_GLOBAL_BIST_CONTROL          0x002
#define VILLA_GLOBAL_BIST_STATUS           0x003
#define VILLA_GLOBAL_LINE_SOFT_RESET       0x007
#define VILLA_GLOBAL_HOST_SOFT_RESET       0x008
#define VILLA_GLOBAL_DWNLD_CHECKSUM_CTRL   0x00A
#define VILLA_GLOBAL_DWNLD_CHECKSUM_STATUS 0x00B
#define VILLA_GLOBAL_MSEQCLKCTRL           0x00E

#define MSEQ_EDC_BIST_DONE  BIT(0)
#define MSEQ_EDC_BIST_FAIL  BIT(8)

#define IS_SEPARATOR(_Char) \
  ((_Char) == ' ' || (_Char) == '\t' || (_Char) == '\n')

typedef struct _CORTINA_REG_CONFIG {
  UINT16 RegAddr;
  UINT16 RegValue;
} CORTINA_REG_CONFIG;


STATIC
CHAR8 *
SkipSeparators (
  CHAR8 *Str
  )
{
  CHAR8 *Ptr;

  for (Ptr = Str; *Ptr != '\0'; Ptr ++) {
    if (!IS_SEPARATOR (*Ptr)) {
        break;
    }
  }

  return Ptr;
}

STATIC
CHAR8 *
FindNextSeparator (
  CHAR8 *Str
  )
{
  CHAR8 *Ptr;

  for (Ptr = Str; *Ptr != '\0'; Ptr ++) {
    if (IS_SEPARATOR (*Ptr)) {
        break;
    }
  }

  return Ptr;
}

STATIC
EFI_STATUS
LoadPhyFirmware (
  DPAA2_PHY *Dpaa2Phy
  )
{
# define TEXT_LINE_SIZE  80

  EFI_PHYSICAL_ADDRESS FwFlashAddr;
  UINT32 FwMaxLines;
  CHAR8 LineTemp[TEXT_LINE_SIZE];
  CHAR8 RegAddr[TEXT_LINE_SIZE];
  CHAR8 RegData[TEXT_LINE_SIZE];
  UINT8 I;
  UINT8 LineCount;
  CORTINA_REG_CONFIG FwRegConfig;
  const CHAR8 *FwFlashPtr;
  CHAR8 *TokenStartPtr;
  CHAR8 *TokenEndPtr;
  UINT8 TokenLength;

  FwMaxLines = FixedPcdGet32 (PcdDpaa2CortinaFwMaxLen);
  FwFlashAddr = FixedPcdGet64 (PcdDpaa2CortinaFwNorAddr);
  DPAA_INFO_MSG ("Loading Cortina firmware from NOR flash address"
                 " 0x%p for PHY address 0x%x ...\n", FwFlashAddr,
                 Dpaa2Phy->PhyAddress);

  FwFlashPtr = (CHAR8 *) FwFlashAddr;
  LineCount = 0;
  while (*FwFlashPtr != 'Q') {
    I = 0;
    while (*FwFlashPtr != '\n') {
      if (I >= TEXT_LINE_SIZE - 1) {
          DPAA_ERROR_MSG ("Line %d in Cortina Firmware is too long\n",
                          LineCount + 1);
          return EFI_INVALID_PARAMETER;
      }

      LineTemp[I++] = *FwFlashPtr++;
    }

    ASSERT (I < TEXT_LINE_SIZE);
    LineTemp[I] = '\0';
    FwFlashPtr++;  /* skip '\n' */
    LineCount++;
    if (LineCount > FwMaxLines) {
      DPAA_ERROR_MSG ("Cortina Firmware larger than expected\n");
      return EFI_INVALID_PARAMETER;
    }

    TokenStartPtr = SkipSeparators (LineTemp);
    if (*TokenStartPtr == '\0') {
      DPAA_ERROR_MSG ("Invalid blank line found in Cortina Firmware\n");
      return EFI_INVALID_PARAMETER;
    }

    TokenEndPtr = FindNextSeparator (TokenStartPtr);
    TokenLength = TokenEndPtr - TokenStartPtr;
    CopyMem (RegAddr, TokenStartPtr, TokenLength);
    RegAddr[TokenLength] = '\0';

    TokenStartPtr = SkipSeparators (TokenEndPtr);
    if (*TokenStartPtr == '\0') {
      DPAA_ERROR_MSG ("Incomplete line found in Cortina Firmware\n");
      return EFI_INVALID_PARAMETER;
    }

    TokenEndPtr = FindNextSeparator (TokenStartPtr);
    TokenLength = TokenEndPtr - TokenStartPtr;
    CopyMem (RegData, TokenStartPtr, TokenLength);
    RegData[TokenLength] = '\0';

    FwRegConfig.RegAddr = AsciiStrHexToUintn (RegAddr);
    FwRegConfig.RegValue = AsciiStrHexToUintn (RegData);
    Dpaa2PhyRegisterWrite (Dpaa2Phy, 0x00, FwRegConfig.RegAddr,
                          FwRegConfig.RegValue);
  }

  return EFI_SUCCESS;
}

/**
   Configures Cortina PHY

   @param[in] Dpaa2Phy          Pointer to PHY object

   @retval EFI_SUCCESS, on success
   @retval error code, on failure

 **/
EFI_STATUS
CortinaPhyConfig (
  DPAA2_PHY *Dpaa2Phy
  )
{
  /**
   * Timeout for successfully completing the BIST test (in milliseconds)
   */
# define BIST_TEST_DONE_TIMEOUT_MS    100

  UINT32 TimeoutCount;
  UINT16 PhyRegValue;
  EFI_STATUS Status;

  ASSERT (Dpaa2Phy->Signature == DPAA2_PHY_SIGNATURE);
  ASSERT (Dpaa2Phy->PhyInterfaceType == PHY_INTERFACE_XGMII); /* 10GBASE-T */

  /*
   * Do BIST test:
   */
  Dpaa2PhyRegisterWrite (Dpaa2Phy, 0x00, VILLA_GLOBAL_MSEQCLKCTRL, 0x0004);
  Dpaa2PhyRegisterWrite (Dpaa2Phy, 0x00, VILLA_GLOBAL_LINE_SOFT_RESET, 0x0000);
  Dpaa2PhyRegisterWrite (Dpaa2Phy, 0x00, VILLA_GLOBAL_BIST_CONTROL, 0x0001);

  TimeoutCount = BIST_TEST_DONE_TIMEOUT_MS;
  for ( ; ; ) {
    PhyRegValue = Dpaa2PhyRegisterRead (Dpaa2Phy, 0x00,
                                       VILLA_GLOBAL_BIST_STATUS);
    if (PhyRegValue & MSEQ_EDC_BIST_DONE) {
      if ((PhyRegValue & MSEQ_EDC_BIST_FAIL) == 0) {
        break;
      }
    }

    if (TimeoutCount == 0) {
      DPAA_ERROR_MSG ("BIST mseq_edc_bist_done timeout for Cortina PHY 0x%p\n",
                      Dpaa2Phy);
      return EFI_NOT_READY;
    }

    MicroSecondDelay (1000);
    TimeoutCount --;
  }

  /*
   * Load PHY firmware:
   */
  Status = LoadPhyFirmware (Dpaa2Phy);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  PhyRegValue = Dpaa2PhyRegisterRead (Dpaa2Phy, 0x00,
                                     VILLA_GLOBAL_DWNLD_CHECKSUM_STATUS);
  if (PhyRegValue) {
      DPAA_ERROR_MSG ("Firmware checksum status failed (0x%x) for Cortina PHY 0x%p\n",
                      PhyRegValue, Dpaa2Phy);
      return EFI_DEVICE_ERROR;
  }

  return EFI_SUCCESS;
}


/**
   Start Cortina PHY

   @param[in] Dpaa2Phy          Pointer to PHY object

   @retval EFI_SUCCESS, on success
   @retval error code, on failure

 **/
EFI_STATUS
CortinaPhyStartup (
  DPAA2_PHY *Dpaa2Phy
  )
{
  ASSERT (Dpaa2Phy->Signature == DPAA2_PHY_SIGNATURE);

  Dpaa2Phy->LinkUp = TRUE;
  Dpaa2Phy->Speed = 10000;
  Dpaa2Phy->FullDuplex = TRUE;

  return EFI_SUCCESS;
}
