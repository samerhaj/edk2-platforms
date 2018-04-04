/** QBman.c
  DPAA2 QBman services implementation

  Copyright 2017 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution. The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include <Library/ArmLib.h>
#include <Library/BaseMemoryLib/MemLibInternals.h>
#include <Library/DpaaDebugLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/TimerLib.h>

#include "QBman.h"

STATIC CONST EFI_LOCK gEfiLockInitializer = EFI_INITIALIZE_LOCK_VARIABLE (TPL_HIGH_LEVEL);
/**
 * Cache enabled mapping is not working right now.
 */
#define QBMAN_CACHE_INHIBTED_ONLY

/**
 * Location of a sub-field within a QBMan portal (CENA) cacheline which
 * is either serving as a configuration command or a query result. The
 * representation is inherently little-endian, as the indexing of the words is
 * itself little-endian in nature and layerscape is little endian for anything
 * that crosses a word boundary too (64-bit fields are the obvious examples).
 */
typedef struct _QBMAN_ATTR_LOCATION {
  /**
   * Index of the attribute  in a cache line seen as a UINT32 array
   */
  UINT32 WordIndex;

  /**
   * Offset (shift) of the LSB bit of the attribute
   */
  UINT32 Shift;

  /**
   * Width in bits of the attribute
   */
  UINT32 Width;
} QBMAN_ATTR_LOCATION;

/**
 * Macro to encode QBMAN attributes
 */
#define QBMAN_ATTR_LOCATION_INIT(_WordIndex, _Shift, _Width) \
        { .WordIndex = _WordIndex, .Shift = _Shift, .Width = _Width}

STATIC CONST QBMAN_ATTR_LOCATION gSdqcrDctLocation = QBMAN_ATTR_LOCATION_INIT (0, 24, 2);
STATIC CONST QBMAN_ATTR_LOCATION gSdqcrFcLocation = QBMAN_ATTR_LOCATION_INIT (0, 29, 1);
STATIC CONST QBMAN_ATTR_LOCATION gSdqcrTokLocation = QBMAN_ATTR_LOCATION_INIT (0, 16, 8);

/**
 * Decode a field from a cacheline
 */
STATIC
inline
UINT32
QbmanAttrDecode (
  CONST QBMAN_ATTR_LOCATION *Location,
  CONST UINT32 *Cacheline
  )
{
  return GET_BIT_FIELD32 (Cacheline[Location->WordIndex],
                         Location->Shift,
                         Location->Width);
}


/**
 * Encode a field to a cacheline
 */
STATIC
inline
VOID
QbmanAttrEncode (
  CONST QBMAN_ATTR_LOCATION *Location,
  UINT32 *Cacheline,
  UINT32 Value
  )
{
    SET_BIT_FIELD32 (Cacheline[Location->WordIndex],
                    Location->Shift,
                    Location->Width,
                    Value);
}


/**
 * Write a cache-inhibited register in a QBman software portal
 */
STATIC
inline
VOID
QbmanCacheInhibitedWrite (
  DPAA2_QBMAN_PORTAL *DpioQbmanPortal,
  UINT32 RegOffset,
  UINT32 Value
  )
{
  MmioWrite32 (DpioQbmanPortal->CacheInhibitedBaseAddr + RegOffset,
        Value);
}


/**
 * Read a cache-inhibited register in a QBman software portal
 */
STATIC
inline
UINT32
QbmanCacheInhibitedRead (
  DPAA2_QBMAN_PORTAL *DpioQbmanPortal,
  UINT32 RegOffset
  )
{
  return MmioRead32 (DpioQbmanPortal->CacheInhibitedBaseAddr + RegOffset);
}

STATIC
inline
VOID ZerorCacheLine (
  UINT32 *Ptr
  )
{
  ASSERT (((UINTN)Ptr & 63) == 0);
  Ptr[0] = 0;
  Ptr[1] = 0;
  Ptr[2] = 0;
  Ptr[3] = 0;
  Ptr[4] = 0;
  Ptr[5] = 0;
  Ptr[6] = 0;
  Ptr[7] = 0;
  Ptr[8] = 0;
  Ptr[9] = 0;
  Ptr[10] = 0;
  Ptr[11] = 0;
  Ptr[12] = 0;
  Ptr[13] = 0;
  Ptr[14] = 0;
  Ptr[15] = 0;
}


/**
 * Start a write to a cache-enabled register in a QBman software portal
 */
STATIC
inline
VOID *
QbmanCacheEnabledWriteStart (
  DPAA2_QBMAN_PORTAL *QbmanPortal,
  UINT32 Offset
  )
{
  UINT32 *Shadow;

  Shadow = (UINT32 *)((UINT8 *)QbmanPortal->CacheEnabledRegsShadow + Offset);
  ZerorCacheLine (Shadow);
  return Shadow;
}


/**
 * Completes a write a cache-enabled register in a QBman software portal
 */
STATIC
inline
VOID
QbmanCacheEnabledWriteComplete (
  DPAA2_QBMAN_PORTAL *QbmanPortal,
  UINT32 Offset,
  VOID *Cmd
  )
{
  CONST UINT32 *Shadow;
  INT8 I;

  Shadow = Cmd;
  for (I = (CACHELINE_SIZE / sizeof (UINT32)) - 1; I >= 0; I --) {
# ifdef QBMAN_CACHE_INHIBTED_ONLY
    MmioWrite32 (QbmanPortal->CacheInhibitedBaseAddr + Offset +
                I * sizeof (UINT32),
                Shadow[I]);
# else
    MmioWrite32 (QbmanPortal->CacheEnabledBaseAddr + Offset +
                I * sizeof (UINT32),
                Shadow[I]);
# endif
  }
}


/**
 * Read a cache-enabled register in a QBman software portal
 */
STATIC
inline
VOID *
QbmanCacheEnabledRead (
  DPAA2_QBMAN_PORTAL *QbmanPortal,
  UINT32 Offset
  )
{
  UINT8 I;
  UINT32 *Shadow;

  Shadow = (UINT32 *)((UINT8 *)QbmanPortal->CacheEnabledRegsShadow + Offset);

  for (I = 0; I < CACHELINE_SIZE / sizeof (UINT32); I ++) {
#ifdef QBMAN_CACHE_INHIBTED_ONLY
    Shadow[I] = MmioRead32 (QbmanPortal->CacheInhibitedBaseAddr + Offset +
                           I * sizeof (UINT32));
#else
    shadow[loop] = __raw_readl (s->addr_cena + offset
    Shadow[I] = MmioRead32 (QbmanPortal->CacheEnabledBaseAddr + Offset +
                           I * sizeof (UINT32));
#endif
  }

  return Shadow;
}


STATIC
inline
VOID QbmanCacheEnabledIinvalidatePrefetch (
  DPAA2_QBMAN_PORTAL *QbmanPortal,
  UINT32 Offset
  )
{
}


/**
 * Encode value for cache-inhibited CFG register of a QBman software portal
 */
STATIC
inline
UINT32
QbmanPortalEncodeCfg (
  UINT8 MaxFill,
  UINT8 Wn,
  UINT8 Est,
  UINT8 Rpm,
  UINT8 Dcm,
  UINT8 Epm,
  INT32 Sd,
  INT32 Sp,
  INT32 Se,
  INT32 Dp,
  INT32 De,
  INT32 Ep
  )
{
  UINT32 RegValue;

  RegValue = 0;
  SET_BIT_FIELD32 (RegValue, 20, 3 + (MaxFill >> 3), MaxFill);
  SET_BIT_FIELD32 (RegValue, 16, 3, Est);
  SET_BIT_FIELD32 (RegValue, 12, 2, Rpm);
  SET_BIT_FIELD32 (RegValue, 10, 2, Dcm);
  SET_BIT_FIELD32 (RegValue, 8, 2, Epm);
  SET_BIT_FIELD32 (RegValue, 5, 1, Sd);
  SET_BIT_FIELD32 (RegValue, 4, 1, Sp);
  SET_BIT_FIELD32 (RegValue, 3, 1, Se);
  SET_BIT_FIELD32 (RegValue, 2, 1, Dp);
  SET_BIT_FIELD32 (RegValue, 1, 1, De);
  SET_BIT_FIELD32 (RegValue, 0, 1, Ep);
  SET_BIT_FIELD32 (RegValue, 14, 1, Wn);

  return RegValue;
}


/**
   Initializes a QBman software portal

   @param QbmanPortalDesc Pointer to QBman software portal descriptor

   @retval EFI_SUCCESS, on success
   @retval error code, on failure
 **/
EFI_STATUS
QbmanSwPortalInit (
  DPAA2_QBMAN_PORTAL *DpioQbmanPortal,
  UINT64 qbman_portal_ce_offset,
  UINT64 qbman_portal_ci_offset
  )
{
  EFI_STATUS Status;
  UINT32 RegValue;

  DpioQbmanPortal->QbmanPortalLock = gEfiLockInitializer;
  DpioQbmanPortal->MgmtCmd.State = QBMAN_PORTAL_CMD_CAN_START;
  DpioQbmanPortal->MgmtCmd.ValidBit = QBMAN_VALID_BIT;
  DpioQbmanPortal->Sdq = 0;
  QbmanAttrEncode (&gSdqcrDctLocation, &DpioQbmanPortal->Sdq,
                  QBMAN_SDQCR_DCT_PRIO_ICS);
  QbmanAttrEncode (&gSdqcrFcLocation, &DpioQbmanPortal->Sdq,
                      QBMAN_SDQCR_FC_UP_TO_3);
  QbmanAttrEncode (&gSdqcrTokLocation, &DpioQbmanPortal->Sdq,
                  0xbb);

  DpioQbmanPortal->Vdq.Busy = 1;
  ArmDataMemoryBarrier ();

  DpioQbmanPortal->Vdq.ValidBit = QBMAN_VALID_BIT;
  DpioQbmanPortal->Dqrr.NextIdx = 0;
  DpioQbmanPortal->Dqrr.ValidBit = QBMAN_VALID_BIT;

  DpioQbmanPortal->CacheEnabledBaseAddr =
    FixedPcdGet64 (PcdDpaa2QBmanPortalsBaseAddr) + qbman_portal_ce_offset;
  DpioQbmanPortal->CacheInhibitedBaseAddr =
    FixedPcdGet64 (PcdDpaa2QBmanPortalsBaseAddr) + qbman_portal_ci_offset;

  /*
   * TODO: Is this the rightr way of allocating the Shadow are?
   * (We just need to allocate a chunk of virtual address space,
   *  not all these physical pages).
   */
  DpioQbmanPortal->CacheEnabledRegsShadow =
    AllocatePages (EFI_SIZE_TO_PAGES (QBMAN_CACHE_ENABLED_SHADOW_SIZE));

  if (DpioQbmanPortal->CacheEnabledRegsShadow == NULL) {
    DPAA_ERROR_MSG ("Failed to allocate QBman's portal cache-enabled registers shadow\n");
    return EFI_OUT_OF_RESOURCES;
  }

  ZeroMem (DpioQbmanPortal->CacheEnabledRegsShadow,
          QBMAN_CACHE_ENABLED_SHADOW_SIZE);

  RegValue = QbmanCacheInhibitedRead (DpioQbmanPortal,
                                     QBMAN_PORTAL_CACHE_INHIBITED_CFG);
  if (RegValue != 0) {
    DPAA_ERROR_MSG ("QBman software portal already configured\n");
    Status = EFI_DEVICE_ERROR;
    goto ErrorExitFreeShadow;
  }

#ifdef QBMAN_CACHE_INHIBTED_ONLY
  RegValue = QbmanPortalEncodeCfg (QBMAN_DQRR_SIZE, 1, 0, 3, 2, 3, 0, 1, 0, 1, 0, 0);
#else
  RegValue = QbmanPortalEncodeCfg (QBMAN_DQRR_SIZE, 0, 0, 3, 2, 3, 0, 1, 0, 1, 0, 0);
#endif

  QbmanCacheInhibitedWrite (DpioQbmanPortal,
                           QBMAN_PORTAL_CACHE_INHIBITED_CFG,
                           RegValue);

  RegValue = QbmanCacheInhibitedRead (DpioQbmanPortal,
                                     QBMAN_PORTAL_CACHE_INHIBITED_CFG);
  if (RegValue == 0) {
    DPAA_ERROR_MSG ("QBman software portal could not be configured\n");
    Status = EFI_DEVICE_ERROR;
    goto ErrorExitFreeShadow;
  }

  QbmanCacheInhibitedWrite (DpioQbmanPortal,
                           QBMAN_PORTAL_CACHE_INHIBITED_SDQCR,
                           DpioQbmanPortal->Sdq);

  return EFI_SUCCESS;

ErrorExitFreeShadow:
  FreePages (DpioQbmanPortal->CacheEnabledRegsShadow,
            EFI_SIZE_TO_PAGES (QBMAN_CACHE_ENABLED_SHADOW_SIZE));
  return Status;
}


STATIC CONST QBMAN_ATTR_LOCATION gReleaseSetMeLocation = QBMAN_ATTR_LOCATION_INIT (0, 5, 1);
STATIC CONST QBMAN_ATTR_LOCATION gReleaseBpidLocation = QBMAN_ATTR_LOCATION_INIT (0, 16, 16);


VOID
QbmanReleaseDescClear (
  QBMAN_RELEASE_DESC *QbmanReleaseDesc
  )
{
  ZeroMem (QbmanReleaseDesc, sizeof (*QbmanReleaseDesc));
  QbmanAttrEncode (&gReleaseSetMeLocation, (UINT32 *)QbmanReleaseDesc, 1);
}


VOID
QbmanReleaseDescSetBpid (
  QBMAN_RELEASE_DESC *QbmanReleaseDesc,
  UINT16 HwBufferPoolbId
  )
{
  QbmanAttrEncode (&gReleaseBpidLocation, (UINT32 *)QbmanReleaseDesc, HwBufferPoolbId);
}


VOID
QbmanEnqueueDescClear (
  QBMAN_ENQUEUE_DESC *QbmanEnqueueDesc
  )
{
  ZeroMem (QbmanEnqueueDesc, sizeof (*QbmanEnqueueDesc));
}


STATIC CONST QBMAN_ATTR_LOCATION gEnqueueCmdLocation = QBMAN_ATTR_LOCATION_INIT (0, 0, 2);
STATIC CONST QBMAN_ATTR_LOCATION gEnqueueOrpEnLocation = QBMAN_ATTR_LOCATION_INIT (0, 2, 1);
STATIC CONST QBMAN_ATTR_LOCATION gEnqueueTgtIdLocation = QBMAN_ATTR_LOCATION_INIT (2, 0, 24);
STATIC CONST QBMAN_ATTR_LOCATION gEnqueueQdEnLocation = QBMAN_ATTR_LOCATION_INIT (0, 4, 1);
STATIC CONST QBMAN_ATTR_LOCATION gEnqueueQdBinLocation = QBMAN_ATTR_LOCATION_INIT (4, 0, 16);
STATIC CONST QBMAN_ATTR_LOCATION gEnqueueQdPriLocation = QBMAN_ATTR_LOCATION_INIT (4, 16, 4);
STATIC CONST QBMAN_ATTR_LOCATION gEnqueueRspStashLocation = QBMAN_ATTR_LOCATION_INIT (5, 16, 1);
STATIC CONST QBMAN_ATTR_LOCATION gEnqueueRspLoLocation = QBMAN_ATTR_LOCATION_INIT (6, 0, 32);

VOID
QbmanEnqueueDescSetNoOrp (
  QBMAN_ENQUEUE_DESC *QbmanEnqueueDesc,
  BOOLEAN RespondSuccess
  )
{
  QbmanAttrEncode (&gEnqueueOrpEnLocation, (UINT32 *)QbmanEnqueueDesc, 0);
  QbmanAttrEncode (&gEnqueueCmdLocation, (UINT32 *)QbmanEnqueueDesc,
      RespondSuccess ? QBMAN_EQ_CMD_RESPOND :
           QBMAN_EQ_CMD_RESPOND_REJECT);
}


VOID
QbmanEnqueueDescSetQueueDest (
  QBMAN_ENQUEUE_DESC *QbmanEnqueueDesc,
  UINT32 QueueDestId,
  UINT32 QueueDestBin,
  UINT32 QueueDestPrio
  )
{
  QbmanAttrEncode (&gEnqueueQdEnLocation, (UINT32 *)QbmanEnqueueDesc, 1);
  QbmanAttrEncode (&gEnqueueTgtIdLocation, (UINT32 *)QbmanEnqueueDesc, QueueDestId);
  QbmanAttrEncode (&gEnqueueQdBinLocation, (UINT32 *)QbmanEnqueueDesc, QueueDestBin);
  QbmanAttrEncode (&gEnqueueQdPriLocation, (UINT32 *)QbmanEnqueueDesc, QueueDestPrio);
}


/**
  Releases a set of buffers to QBman

   @param QbmanPortal Pointer to DPIO QBman portal
   @param QbmanReleaseDesc Pointer to QBman release descriptor

   @retval TRUE, on success
   @retval FALSE, if QBman portal is busy
**/
BOOLEAN
QbmanSwPortalReleaseBuffers (
  DPAA2_QBMAN_PORTAL *QbmanPortal,
  CONST QBMAN_RELEASE_DESC *QbmanReleaseDesc,
  CONST UINT64 *Buffers,
  UINT32 NumBuffers
  )
{
# define RAR_IDX(rar)     ((rar) & 0x7)
# define RAR_VB(rar)      ((rar) & 0x80)
# define RAR_SUCCESS(rar) ((rar) & 0x100)

  UINT32 *CmdBuf;
  UINT32 *CacheLine;
  UINT32 Rar = QbmanCacheInhibitedRead (QbmanPortal,
                                       QBMAN_PORTAL_CACHE_INHIBITED_RAR);

  CacheLine = (UINT32 *)QbmanReleaseDesc;
  if (!RAR_SUCCESS (Rar)) {
    return FALSE;
  }

  ASSERT (NumBuffers > 0 && NumBuffers <= 7);
  EfiAcquireLock (&QbmanPortal->QbmanPortalLock);

  CmdBuf = QbmanCacheEnabledWriteStart (QbmanPortal,
                                       QBMAN_PORTAL_CACHE_ENABLED_RCR (RAR_IDX (Rar)));

  /*
   * Copy the buffer pointers from input array to the command:
   *
   * NOTE: We are assuming here that this code only gets executed in
   * little-endian
   */
  CopyMem (&CmdBuf[2], Buffers, NumBuffers * sizeof (Buffers[0]));
  ArmDataMemoryBarrier ();

  /*
   * Set the verb byte, have to substitute in the valid-bit and the number
   * of buffers:
   */
  CmdBuf[0] = CacheLine[0] | RAR_VB (Rar) | NumBuffers;

  QbmanCacheEnabledWriteComplete (QbmanPortal,
         QBMAN_PORTAL_CACHE_ENABLED_RCR (RAR_IDX (Rar)),
         CmdBuf);

  EfiReleaseLock (&QbmanPortal->QbmanPortalLock);
  return TRUE;
}


/**
 * Returns a pointer to where the caller should fill in its
 * QBman command (though they should ignore the verb byte)
 */
STATIC
UINT32 *
QbmanSwPortalCmdStart (
  DPAA2_QBMAN_PORTAL *QbmanPortal
  )
{
  UINT32 *CmdBuf;

  ASSERT (QbmanPortal->MgmtCmd.State == QBMAN_PORTAL_CMD_CAN_START);
  CmdBuf = QbmanCacheEnabledWriteStart (QbmanPortal, QBMAN_PORTAL_CACHE_ENABLED_CR);
  if (*CmdBuf == 0) {
    QbmanPortal->MgmtCmd.State = QBMAN_PORTAL_CMD_CAN_SUBMIT;
  }

  return CmdBuf;
}


/**
 * Commits merges in the caller-supplied command verb (which should not include
 * the valid-bit) and submits the command to the hardware.
 */
STATIC
VOID
QbmanSwPortalCmdSubmit (
  DPAA2_QBMAN_PORTAL *QbmanPortal,
  UINT32 *CmdBuf,
  UINT32 CmdVerb
  )
{
  ASSERT (QbmanPortal->MgmtCmd.State == QBMAN_PORTAL_CMD_CAN_SUBMIT);
  ArmDataMemoryBarrier ();

  ASSERT ((*CmdBuf & CmdVerb) == *CmdBuf);
  *CmdBuf = CmdVerb | QbmanPortal->MgmtCmd.ValidBit;
  QbmanCacheEnabledWriteComplete (QbmanPortal, QBMAN_PORTAL_CACHE_ENABLED_CR,
                                 CmdBuf);
  /* TODO: add prefetch support for GPP */

  QbmanPortal->MgmtCmd.State = QBMAN_PORTAL_CMD_CAN_POLL_RESULT;
}


/**
 * Checks for a completed response (returns non-NULL only if the response is
 * complete)
 */
STATIC
UINT32 *
QbmanSwPortalCmdResult (
  DPAA2_QBMAN_PORTAL *QbmanPortal
  )
{
  UINT32 *ResultBuf;
  UINT32 Verb;

  ASSERT (QbmanPortal->MgmtCmd.State == QBMAN_PORTAL_CMD_CAN_POLL_RESULT);
  ResultBuf = QbmanCacheEnabledRead (
                QbmanPortal,
                QBMAN_PORTAL_CACHE_ENABLED_RR (QbmanPortal->MgmtCmd.ValidBit));

  /*
   * Check if the command had completed:
   */
  Verb = ResultBuf[0] & ~QBMAN_VALID_BIT;
  if (Verb == 0) {
    return NULL;
  }

  QbmanPortal->MgmtCmd.State = QBMAN_PORTAL_CMD_CAN_START;
  QbmanPortal->MgmtCmd.ValidBit ^= QBMAN_VALID_BIT;
  return ResultBuf;
}


/**
 * Submits a management command to a QBMAn portal and waits for its response
 */
STATIC
inline
UINT32 *
QbmanSwPortalCmdSubmitAndWait (
  DPAA2_QBMAN_PORTAL *QbmanPortal,
  UINT32 *Cmd,
  UINT32 CmdVerb
  )
{
  /**
   * Timeout for waiting for a QBman command result (in microseconds)
   */
# define QBMAN_WAIT_CMD_RESULT_TIMEOUT_US    2000

  UINT32 TimeoutCount;
  UINT32 *ResultBuf;

  QbmanSwPortalCmdSubmit (QbmanPortal, Cmd, CmdVerb);
  TimeoutCount = QBMAN_WAIT_CMD_RESULT_TIMEOUT_US;
  for ( ; ; ) {
    ResultBuf = QbmanSwPortalCmdResult (QbmanPortal);
    if (ResultBuf != NULL) {
      break;
    }

    if (TimeoutCount == 0) {
      DPAA_ERROR_MSG ("Timeout waiting for command result for QBman portal 0x%p\n",
                      QbmanPortal);
      QbmanPortal->MgmtCmd.State = QBMAN_PORTAL_CMD_CAN_START;
      break;
    }

    MicroSecondDelay (100);
    TimeoutCount -= 100;
  }

  ASSERT (QbmanPortal->MgmtCmd.State == QBMAN_PORTAL_CMD_CAN_START);
  return ResultBuf;
}

STATIC CONST QBMAN_ATTR_LOCATION gAcquireBpidLocation = QBMAN_ATTR_LOCATION_INIT (0, 16, 16);
STATIC CONST QBMAN_ATTR_LOCATION gAcquireNumLocation = QBMAN_ATTR_LOCATION_INIT (1, 0, 3);
STATIC CONST QBMAN_ATTR_LOCATION gAcquireResultNumLocation = QBMAN_ATTR_LOCATION_INIT (1, 0, 3);
STATIC CONST QBMAN_ATTR_LOCATION gGenericVerbLocation = QBMAN_ATTR_LOCATION_INIT (0, 0, 7);
STATIC CONST QBMAN_ATTR_LOCATION gGenericResultLocation = QBMAN_ATTR_LOCATION_INIT (0, 8, 8);

/**
   Acquires a set of buffers from QBman

   @param[in] QbmanPortal       Pointer to QBman software portal
   @param[in] HwBufferPoolId    Hardware buffer pool Id
   @param[out] Buffers          Array where pointers to acquired buffers are to
                                be returned
   @param[out] NumBuffers       Maximum number of buffers requested

   @retval > 0, Number of buffers acquired (<= NumBuffers)
   @retval 0, if no buffers could be acquired

**/
UINT32
QbmanSwPortalAcquireBuffers (
  DPAA2_QBMAN_PORTAL *QbmanPortal,
  UINT16 HwBufferPoolId,
  UINT64 *Buffers,
  UINT32 NumBuffers
  )
{
  UINT32 *CmdBuf;
  UINT32 *ResultBuf;
  UINT32 Verb;
  UINT32 Result;
  UINT32 NumAcquiredBuffers;

  ASSERT (NumBuffers != 0 && NumBuffers <= 7);

  EfiAcquireLock (&QbmanPortal->QbmanPortalLock);

  CmdBuf = QbmanSwPortalCmdStart (QbmanPortal);
  ASSERT (CmdBuf != NULL);

  QbmanAttrEncode (&gAcquireBpidLocation, CmdBuf, HwBufferPoolId);
  QbmanAttrEncode (&gAcquireNumLocation, CmdBuf, NumBuffers);

  ResultBuf = QbmanSwPortalCmdSubmitAndWait (QbmanPortal,
                                            CmdBuf,
                                            CmdBuf[0] | QBMAN_CMD_ACQUIRE);
  EfiReleaseLock (&QbmanPortal->QbmanPortalLock);

  if (ResultBuf == NULL) {
    return 0;
  }

  Verb = QbmanAttrDecode (&gGenericVerbLocation, ResultBuf);
  Result = QbmanAttrDecode (&gGenericResultLocation, ResultBuf);
  NumAcquiredBuffers = QbmanAttrDecode (&gAcquireResultNumLocation, ResultBuf);
  ASSERT (Verb == QBMAN_CMD_ACQUIRE);

  if (Result != QBMAN_CMD_RESULT_OK) {
    DPAA_ERROR_MSG (
      "QBman acquire command failed with error 0x%02x for BPID 0x%x\n",
      Result,  HwBufferPoolId);

    return 0;
  }

  if (NumAcquiredBuffers != 0) {
    ASSERT (NumAcquiredBuffers <= NumBuffers);

    /*
     * Store pointers to acquired buffers to output array:
     */
    CopyMem (Buffers, &ResultBuf[2], NumAcquiredBuffers * sizeof (Buffers[0]));
  }

  return NumAcquiredBuffers;
}


/**
  Enqueue a frame descriptor to QBman

   @param QbmanPortal Pointer to DPIO QBman portal
   @param QbmanEnqueueDesc Pointer to QBman enqueue descriptor
   @param QbmanFrameDesc Pointer to QBman frame descriptor

   @retval TRUE, on success
   @retval FALSE, if QBman portal is busy
**/
BOOLEAN
QbmanSwPortalEnqueue (
  DPAA2_QBMAN_PORTAL *QbmanPortal,
  CONST QBMAN_ENQUEUE_DESC *QbmanEnqueueDesc,
  CONST QBMAN_FRAME_DESC *QbmanFrameDesc
  )
{
# define EQAR_IDX(eqar)     ((eqar) & 0x7)
# define EQAR_VB(eqar)      ((eqar) & 0x80)
# define EQAR_SUCCESS(eqar) ((eqar) & 0x100)

  UINT32 *CmdBuf;
  UINT32 *CacheLine;
  UINT32 Eqar;

  CacheLine = (UINT32 *)QbmanEnqueueDesc;
  Eqar = QbmanCacheInhibitedRead (QbmanPortal, QBMAN_PORTAL_CACHE_INHIBITED_EQAR);
  if (!EQAR_SUCCESS (Eqar)) {
    return FALSE;
  }

  EfiAcquireLock (&QbmanPortal->QbmanPortalLock);

  /*
   * Start the enqueue command:
   */
  CmdBuf = QbmanCacheEnabledWriteStart (
       QbmanPortal,
       QBMAN_PORTAL_CACHE_ENABLED_EQCR (EQAR_IDX (Eqar)));

  CopyMem (&CmdBuf[1], &CacheLine[1], 7*sizeof (UINT32));
  CopyMem (&CmdBuf[8], QbmanFrameDesc, sizeof (*QbmanFrameDesc));
  ArmDataMemoryBarrier ();

  /*
   * Set the verb byte, have to substitute in the valid-bit:
   */
  CmdBuf[0] = CacheLine[0] | EQAR_VB (Eqar);
  QbmanCacheEnabledWriteComplete (QbmanPortal,
                            QBMAN_PORTAL_CACHE_ENABLED_EQCR (EQAR_IDX (Eqar)),
                            CmdBuf);

  EfiReleaseLock (&QbmanPortal->QbmanPortalLock);
  return TRUE;
}


STATIC
VOID
AtomicInt32Increment (
  volatile INT32 *Var
  )
{
  UINTN SavedInterruptState;

  SavedInterruptState = ArmGetInterruptState ();
  ArmDisableInterrupts ();

  (*Var) ++;

  if (SavedInterruptState) {
    ArmEnableInterrupts ();
  }
}

STATIC
BOOLEAN
AtomicInt32DecrementAndTest (
  volatile INT32 *Var
  )
{
  UINTN SavedInterruptState;
  INT32 Value;

  SavedInterruptState = ArmGetInterruptState ();
  ArmDisableInterrupts ();

  Value = *Var;
  Value --;
  *Var = Value;

  if (SavedInterruptState) {
    ArmEnableInterrupts ();
  }

  return Value == 0;
}


STATIC CONST QBMAN_ATTR_LOCATION gDequeueResponseRingVerbLocation = QBMAN_ATTR_LOCATION_INIT (0, 0, 8);
STATIC CONST QBMAN_ATTR_LOCATION gDequeueResponseRingRespLocation = QBMAN_ATTR_LOCATION_INIT (0, 0, 7);
STATIC CONST QBMAN_ATTR_LOCATION gDequeueResponseRingStatLocation = QBMAN_ATTR_LOCATION_INIT (0, 8, 8);

/**
 * Returns a DQRR (Dequeue Response Ring) entry only once, so repeated calls can
 * return a sequence of DQRR entries, without requiring they be consumed
 * immediately or in any particular order. Returns NULL if there are no
 * unconsumed DQRR entries.
 */
CONST
QBMAN_DEQUEUE_ENTRY *
QbmanSwPortalGetNextDqrrEntry (
  DPAA2_QBMAN_PORTAL *QbmanPortal
  )
{
# define QBMAN_DQRR_RESPONSE_DQ        0x60
# define QBMAN_DQRR_RESPONSE_FQRN      0x21
# define QBMAN_DQRR_RESPONSE_FQRNI     0x22
# define QBMAN_DQRR_RESPONSE_FQPN      0x24
# define QBMAN_DQRR_RESPONSE_FQDAN     0x25
# define QBMAN_DQRR_RESPONSE_CDAN      0x26
# define QBMAN_DQRR_RESPONSE_CSCN_MEM  0x27
# define QBMAN_DQRR_RESPONSE_CGCU      0x28
# define QBMAN_DQRR_RESPONSE_BPSCN     0x29
# define QBMAN_DQRR_RESPONSE_CSCN_WQ   0x2a

  UINT32 Verb;
  UINT32 ResponseVerb;
  UINT32 Flags;
  CONST QBMAN_DEQUEUE_ENTRY *DequeueEntry;
  CONST UINT32 *ResultBuf;

  EfiAcquireLock (&QbmanPortal->QbmanPortalLock);
  DequeueEntry = QbmanCacheEnabledRead (
                  QbmanPortal,
                  QBMAN_PORTAL_CACHE_ENABLED_DQRR (QbmanPortal->Dqrr.NextIdx));

  ResultBuf = (UINT32 *)DequeueEntry;
  Verb = QbmanAttrDecode (&gDequeueResponseRingVerbLocation, ResultBuf);

  /*
   * If the valid-bit isn't of the expected polarity, nothing there. Note,
   * in the DQRR reset bug workaround, we shouldn't need to skip these
   * check, because we've already determined that a new entry is available
   * and we've invalidated the cacheline before reading it, so the
   * valid-bit behaviour is repaired and should tell us what we already
   * knew from reading PI.
   */
  if ((Verb & QBMAN_VALID_BIT) != QbmanPortal->Dqrr.ValidBit) {
    QbmanCacheEnabledIinvalidatePrefetch (
      QbmanPortal,
      QBMAN_PORTAL_CACHE_ENABLED_DQRR (QbmanPortal->Dqrr.NextIdx));

    EfiReleaseLock (&QbmanPortal->QbmanPortalLock);
    return NULL;
  }

  /*
   * There's something there. Move "next_idx" attention to the next ring
   * entry (and prefetch it) before returning what we found.
   */
  QbmanPortal->Dqrr.NextIdx ++;
  QbmanPortal->Dqrr.NextIdx &= QBMAN_DQRR_SIZE - 1; /* Wrap around at 4 */
  if (QbmanPortal->Dqrr.NextIdx == 0) {
    QbmanPortal->Dqrr.ValidBit ^= QBMAN_VALID_BIT;
  }

  /*
   * If this is the final response to a volatile dequeue command
   * indicate that the vdq is no longer busy:
   */
  Flags = QbmanAttrDecode (&gDequeueResponseRingStatLocation, ResultBuf);
  ResponseVerb = QbmanAttrDecode (&gDequeueResponseRingRespLocation, &Verb);
  if (ResponseVerb == QBMAN_DQRR_RESPONSE_DQ &&
      (Flags & QBMAN_DEQUEUE_STAT_VOLATILE) != 0 &&
      (Flags & QBMAN_DEQUEUE_STAT_EXPIRED) != 0) {
    AtomicInt32Increment (&QbmanPortal->Vdq.Busy);
  }

  QbmanCacheEnabledIinvalidatePrefetch (
    QbmanPortal,
    QBMAN_PORTAL_CACHE_ENABLED_DQRR (QbmanPortal->Dqrr.NextIdx));

  EfiReleaseLock (&QbmanPortal->QbmanPortalLock);
  return DequeueEntry;
}


/**
 * Consume DQRR entries previously returned from QbmanSwPortalGetNextDqrrEntry ()
 */
VOID
QbmanSwPortalConsumeDqrrEntry (
  DPAA2_QBMAN_PORTAL *QbmanPortal,
  CONST QBMAN_DEQUEUE_ENTRY *DequeueEntry
  )
{
  QbmanCacheInhibitedWrite (QbmanPortal,
                           QBMAN_PORTAL_CACHE_INHIBITED_DCAP,
                           QBMAN_DEQUEUE_IDX (DequeueEntry));
}


UINT32
QbmanGetFlagsFromDequeueEntry (
  CONST QBMAN_DEQUEUE_ENTRY *DequeueEntry
  )
{
  CONST UINT32 *CacheLine = (UINT32 *)DequeueEntry;

  return QbmanAttrDecode (&gDequeueResponseRingStatLocation, CacheLine);
}


/**
 * Return frame descriptor from a dequeue entry
 */
CONST QBMAN_FRAME_DESC *
QbmanGetFrameDescFromDequeueEntry (
  CONST QBMAN_DEQUEUE_ENTRY *DequeueEntry
  )
{
  CONST UINT32 *CacheLine = (UINT32 *)DequeueEntry;

  return (CONST QBMAN_FRAME_DESC *)&CacheLine[8];
}


STATIC CONST QBMAN_ATTR_LOCATION gPullDctLocation = QBMAN_ATTR_LOCATION_INIT (0, 0, 2);
STATIC CONST QBMAN_ATTR_LOCATION gPullDtLocation = QBMAN_ATTR_LOCATION_INIT (0, 2, 2);
STATIC CONST QBMAN_ATTR_LOCATION gPullRlsLocation = QBMAN_ATTR_LOCATION_INIT (0, 4, 1);
STATIC CONST QBMAN_ATTR_LOCATION gPullStashLocation = QBMAN_ATTR_LOCATION_INIT (0, 5, 1);
STATIC CONST QBMAN_ATTR_LOCATION gPullNumFramesLocation = QBMAN_ATTR_LOCATION_INIT (0, 8, 4);
STATIC CONST QBMAN_ATTR_LOCATION gPullTokenLocation = QBMAN_ATTR_LOCATION_INIT (0, 16, 8);
STATIC CONST QBMAN_ATTR_LOCATION gPullDqSourceLocation = QBMAN_ATTR_LOCATION_INIT (1, 0, 24);


VOID
QbmanPullDescClear (
  QBMAN_PULL_DESC *QbmanPullDesc)
{
  ZeroMem (QbmanPullDesc, sizeof (*QbmanPullDesc));
}


VOID
QbmanPullDescSetNumframes (
  QBMAN_PULL_DESC *QbmanPullDesc,
  UINT8 NumFrames
  )
{
  ASSERT (NumFrames != 0 && NumFrames <= 16);
  QbmanAttrEncode (&gPullNumFramesLocation,
                 (UINT32 *)QbmanPullDesc,
                 (UINT32)(NumFrames - 1));
}


VOID
QbmanPullDescSetFrameDesc (
  QBMAN_PULL_DESC *QbmanPullDesc,
  UINT32 FrameQueueId
  )
{
  UINT32 *CacheLine = (UINT32 *)QbmanPullDesc;

  QbmanAttrEncode (&gPullDctLocation, CacheLine, 1);
  QbmanAttrEncode (&gPullDtLocation, CacheLine, QBMAN_PULL_DT_FRAMEQUEUE);
  QbmanAttrEncode (&gPullDqSourceLocation, CacheLine, FrameQueueId);
}


/**
  Send a Pull command to QBman

   @param QbmanPortal Pointer to DPIO QBman portal
   @param QbmanPullDesc Pointer to QBman pull descriptor

   @retval TRUE, on success
   @retval FALSE, if QBman portal is busy
**/
BOOLEAN
QbmanSwPortalPull (
  DPAA2_QBMAN_PORTAL *QbmanPortal,
  QBMAN_PULL_DESC *QbmanPullDesc
  )
{
  UINT32 *CmdBuf;
  UINT32 *CacheLine = (UINT32 *)QbmanPullDesc;

  EfiAcquireLock (&QbmanPortal->QbmanPortalLock);
  if (!AtomicInt32DecrementAndTest (&QbmanPortal->Vdq.Busy)) {
    AtomicInt32Increment (&QbmanPortal->Vdq.Busy);
    EfiReleaseLock (&QbmanPortal->QbmanPortalLock);
    return FALSE;
  }

  QbmanPortal->Vdq.Storage = *((void **)&CacheLine[4]);
  QbmanPortal->Vdq.Token = QbmanAttrDecode (&gPullTokenLocation, CacheLine);
  CmdBuf = QbmanCacheEnabledWriteStart (QbmanPortal,
                                       QBMAN_PORTAL_CACHE_ENABLED_VDQCR);
  CopyMem (&CmdBuf[1], &CacheLine[1], 3*sizeof (UINT32));
  ArmDataMemoryBarrier ();

  /*
   * Set the verb byte, have to substitute in the valid-bit:
   */
  CmdBuf[0] = CacheLine[0] | QbmanPortal->Vdq.ValidBit;
  QbmanPortal->Vdq.ValidBit ^= QBMAN_VALID_BIT;
  QbmanCacheEnabledWriteComplete (QbmanPortal,
                                 QBMAN_PORTAL_CACHE_ENABLED_VDQCR,
                                 CmdBuf);
  EfiReleaseLock (&QbmanPortal->QbmanPortalLock);
  return TRUE;
}
