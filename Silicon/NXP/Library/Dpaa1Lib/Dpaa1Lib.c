/** @file
  DPAA library implementation

  Copyright (c) 2016, Freescale Semiconductor, Inc. All rights Reserved.

  This Program And The Accompanying Materials
  Are Licensed And Made Available Under The Terms And Conditions Of The BSD
  License Which Accompanies This Distribution. The Full Text Of The License
  May Be Found At
  http://Opensource.Org/Licenses/Bsd-License.Php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include <Library/Dpaa1DebugLib.h>
#include <Library/Dpaa1Lib.h>
#include <Library/FrameManager.h>
#include <Library/IoLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/NetLib.h>
#include <Library/UefiRuntimeServicesTableLib.h>

STATIC CONST CHAR16           mUniqueMacVariableName[] = L"MacUniqueId";

/**
   Initializes the DPAA Frame Manager (FMan)

   @retval EFI_SUCCESS   on success
   @retval !EFI_SUCCESS  on failure.

 **/

VOID BmiRxPortInit (
  IN  BMI_RX_PORT *Port
  )
{
       /* set BMI to independent mode, Rx port disable */
       MmioWriteBe32((UINTN)&Port->FmanBmRcfg, FMAN_BM_RCFG_IM);
       /* clear FOF in IM case */
       MmioWriteBe32((UINTN)&Port->FmanBmRim, 0);
       /* Rx frame next engine -RISC */
       MmioWriteBe32((UINTN)&Port->FmanBmRfne, NIA_ENG_RISC | NIA_RISC_AC_IM_RX);
       /* Rx command attribute - no order, MR[3] = 1 */
       MmioClearBitsBe32((UINTN)&Port->FmanBmRfca, FMAN_BM_RFCA_ORDER | FMAN_BM_RFCA_MR_MASK);
       MmioSetBitsBe32((UINTN)&Port->FmanBmRfca, FMAN_BM_RFCA_MR(4));
       /* enable Rx statistic counters */
       MmioWriteBe32((UINTN)&Port->FmanBmRstc, FMAN_BM_RSTC_EN);
       /* disable Rx performance counters */
       MmioWriteBe32((UINTN)&Port->FmanBmRpc, 0);
}

VOID BmiTxPortInit (
  IN  BMI_TX_PORT *Port
  )
{
       /* set BMI to independent mode, Tx port disable */
       MmioWriteBe32((UINTN)&Port->FmanBmTcfg, FMAN_BM_TCFG_IM);
       /* Tx frame next engine -RISC */
       MmioWriteBe32((UINTN)&Port->FmanBmTfne, NIA_ENG_RISC | NIA_RISC_AC_IM_TX);
       MmioWriteBe32((UINTN)&Port->FmanBmTfene, NIA_ENG_RISC | NIA_RISC_AC_IM_TX);
       /* Tx command attribute - no order, MR[3] = 1 */
       MmioClearBitsBe32((UINTN)&Port->FmanBmTfca, FMAN_BM_TFCA_ORDER | FMAN_BM_TFCA_MR_MASK);
       MmioSetBitsBe32((UINTN)&Port->FmanBmTfca, FMAN_BM_TFCA_MR(4));
       /* enable Tx statistic counters */
       MmioWriteBe32((UINTN)&Port->FmanBmTstc, FMAN_BM_TSTC_EN);
       /* disable Tx performance counters */
       MmioWriteBe32((UINTN)&Port->FmanBmTpc, 0);
}

EFI_STATUS
DpaaFrameManagerInit (VOID)
{
  return FmanInit(0, (FMAN_CCSR *)PcdGet64(PcdDpaa1FmanAddr));
}

/**
 * Retrieve the SoC unique ID
 */
UINT32
GetSocUniqueId(VOID)
{
  /*
   * TODO: We need to retrieve a SoC unique ID here.
   * A possiblity is to read the Fresscale Unique ID register (FUIDR) register
   * in the Security Fuse Processor (SFP)
   *
   * For now we just generate a pseudo-randmom number.
   */
  STATIC UINT32 SocUniqueId = 0;

  if (SocUniqueId == 0) {
    SocUniqueId = NET_RANDOM(NetRandomInitSeed());
  }

  return SocUniqueId;
}

/**
 * Generate an Ethernet address (MAC) that is not multicast
 * and has the local assigned bit set.
 */
VOID
GenerateMacAddress(
  IN  UINT32 SocUniqueId,
  IN  FMAN_MEMAC_ID MemacId,
  OUT EFI_MAC_ADDRESS *MacAddrBuf
  )
{
  /*
   * Bit masks for first byte of a MAC address
   */
# define MAC_MULTICAST_ADDRESS_MASK 0x1
# define MAC_PRIVATE_ADDRESS_MASK   0x2

  /*
   * Build MAC address from SoC's unique hardware identifier:
   */
  CopyMem(MacAddrBuf->Addr, &SocUniqueId, sizeof(UINT32));
  MacAddrBuf->Addr[4] = ((UINT16)MemacId + 1) >> 8;
  MacAddrBuf->Addr[5] = (UINT8)MemacId + 1;

  /*
   * Ensure special bits of first byte of the MAC address are properly
   * set:
   */
  MacAddrBuf->Addr[0] &= ~MAC_MULTICAST_ADDRESS_MASK;
  MacAddrBuf->Addr[0] |= MAC_PRIVATE_ADDRESS_MASK;

  DEBUG((EFI_D_INFO, "MAC addr: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  MacAddrBuf->Addr[0],
                  MacAddrBuf->Addr[1],
                  MacAddrBuf->Addr[2],
                  MacAddrBuf->Addr[3],
                  MacAddrBuf->Addr[4],
                  MacAddrBuf->Addr[5]));
}

EFI_STATUS
GetNVSocUniqueId (
    OUT UINT32 *UniqueId
    )
{
  EFI_STATUS Status = EFI_SUCCESS;
  UINT32 SocUniqueId = {0};
  UINTN       Size;

  /* Get the UniqueID required for MAC address generation */
  Size = sizeof (UINT32);
  Status = gRT->GetVariable ((CHAR16 *)mUniqueMacVariableName, 
                            &gEfiCallerIdGuid, 
                            NULL, &Size, (VOID *)UniqueId);

  if (EFI_ERROR (Status)) {
    ASSERT(Status != EFI_INVALID_PARAMETER);
    ASSERT(Status != EFI_BUFFER_TOO_SMALL);
    
    if (Status != EFI_NOT_FOUND)
      return Status;
    
    /* The Unique Mac variable does not exist in non-volatile storage, 
     * so create it. 
     */
    SocUniqueId = GetSocUniqueId();
    Status = gRT->SetVariable ((CHAR16 *)mUniqueMacVariableName, 
                    &gEfiCallerIdGuid, 
                    EFI_VARIABLE_NON_VOLATILE | 
                    EFI_VARIABLE_BOOTSERVICE_ACCESS | 
                    EFI_VARIABLE_RUNTIME_ACCESS, Size, 
                    &SocUniqueId);

    if (EFI_ERROR (Status)) {
      DPAA1_ERROR_MSG("SetVariable Failed, Status=0x%lx \n",Status);
      return Status;
    }
    *UniqueId = SocUniqueId;
  }

  return Status;
}

