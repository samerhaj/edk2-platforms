/** QBman.h
  DPAA2 QBman services interface

  Copyright 2017 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution. The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#ifndef __QBMAN_H__
#define __QBMAN_H__

#include <Library/Dpaa2ManagementComplexLib.h>
#include <Library/UefiLib.h>
#include <Uefi.h>

/**
 * Hardware requires alignment for buffer address and length: 256-byte
 * for ingress, 64-byte for egress. Using 256 for both.
 */
#define DPAA2_ETH_FRAME_BUFFER_ALIGN        256

/**
 * Frame's software annotation: the hardware options are either 0 or 64.
 */
#define DPAA2_ETH_FRAME_SW_ANNOTATION_SIZE  64

#define DPAA2_ETH_NUM_FRAME_BUFFERS  (7 * 7)
#define DPAA2_ETH_REFILL_THRESHOLD  (DPAA2_ETH_NUM_FRAME_BUFFERS / 2)

/**
 * All QBMan command and result structures use this "valid bit" encoding
 */
#define QBMAN_VALID_BIT ((uint32_t)0x80)

/**
 * QBman management command result codes
 */
#define QBMAN_CMD_RESULT_OK      0xf0

/**
 * QBman DQRR size
 * TBD: as of QBMan 4.1, DQRR will be 8 rather than 4!
 */
#define QBMAN_DQRR_SIZE 8


/**
 * Cache Line Size
 */
#define CACHELINE_SIZE  64

/**
 * Size (in bytes) of the shadow copy of a QBMAN portal's
 * Cache-enabled MMIO registers
 */
#define QBMAN_CACHE_ENABLED_SHADOW_SIZE     0x10000

/*
 * QBMan portal management command codes
 */
#define QBMAN_CMD_ACQUIRE           0x30
#define QBMAN_CMD_WQCHAN_CONFIGURE  0x46

/*
 * Offsets of QBman portal cache-inhibited registers
 */
#define QBMAN_PORTAL_CACHE_INHIBITED_EQAR   0x8c0
#define QBMAN_PORTAL_CACHE_INHIBITED_DCAP   0xac0
#define QBMAN_PORTAL_CACHE_INHIBITED_SDQCR  0xb00
#define QBMAN_PORTAL_CACHE_INHIBITED_RAR    0xcc0
#define QBMAN_PORTAL_CACHE_INHIBITED_CFG    0xd00

/*
 * Offsets of QBman portal cache-enabled registers
 */
#define QBMAN_PORTAL_CACHE_ENABLED_EQCR(_N) (0x000 + ((UINT32)(_N) << 6))
#define QBMAN_PORTAL_CACHE_ENABLED_DQRR(_N) (0x200 + ((uint32_t)(_N) << 6))
#define QBMAN_PORTAL_CACHE_ENABLED_RCR(_N)  (0x400 + ((uint32_t)(_N) << 6))
#define QBMAN_PORTAL_CACHE_ENABLED_CR       0x600
#define QBMAN_PORTAL_CACHE_ENABLED_RR(_Vb)  (0x700 + ((uint32_t)(_Vb) >> 1))
#define QBMAN_PORTAL_CACHE_ENABLED_VDQCR    0x780

/*
 * Timeout for successfully releasing a buffer to QBman (in microseconds)
 */
#define QBMAN_BUFFER_RELEASE_TIMEOUT_US    2000

/*
 * Timeout for successfully dequeueing a Rx frame or Tx confirmation
 * from QBman (in microseconds)
 */
#define QBMAN_DEQUEUE_TIMEOUT_US 2000

/**
 * Descriptor for a QBMan instance on the SoC. On partitions/targets that do not
 * control this QBMan instance, these values may simply be place-holders. The
 * idea is simply that we be able to distinguish between them, eg. so that SWP
 * descriptors can identify which QBMan instance they belong to.
 */
typedef struct _QBMAN_BLOCK_DESC {
    /**
     * CCSR register map
     */
    VOID *CcsrRegBar;

    /**
     * Recoverable error interrupt
     */
    INT32 IrqRerr;

    /**
     * Non-recoverable error interrupt
     */
    INT32 IrqNerr;
} QBMAN_BLOCK_DESC;

/**
 * QBman release deascriptor
 */
typedef struct _QBMAN_RELEASE_DESC {
  UINT32 DontManipulateDirectly[1];
} QBMAN_RELEASE_DESC;

/**
 * Enqueue descriptor
 */
typedef struct _QBMAN_ENQUEUE_DESC {
  UINT32 DontManipulateDirectly[8];
} QBMAN_ENQUEUE_DESC;

/**
 * Pull descriptor
 */
typedef struct _QBMAN_PULL_DESC {
  UINT32 dont_manipulate_directly[6];
} QBMAN_PULL_DESC;

/**
 * Place-holder for FDs, we represent it via the simplest form that we need for
 * now. Different overlays may be needed to support different options, etc.
 *
 * The 32-bit words contained within this structure are represented in
 * host-endianness, even if hardware always treats them as little-endian.
 */
typedef struct _QBMAN_FRAME_DESC {
  union {
    UINT32 Words[8];
    struct _QBMAN_FD_SIMPLE {
      UINT32 AddressLowWord;
      UINT32 AddressHighWord;
      UINT32 Length;
      UINT32 BpidOffset; /* Offset is high 16 bits, BPID is low 16 bits */
      UINT32 FrameContext;
#     define QBMAN_FD_FRC_FASV    0x8000
#     define QBMAN_FD_FRC_FAEADV  0x4000
#     define QBMAN_FD_FRC_FAPRV   0x2000
#     define QBMAN_FD_FRC_FAIADV  0x1000
#     define QBMAN_FD_FRC_FASWOV  0x0800
#     define QBMAN_FD_FRC_FAICFDV 0x0400

      UINT32 Control; /* "err", "va", "cbmt", "asal", [...] */
#     define QBMAN_FD_CTRL_ASAL    0x00020000
#     define QBMAN_FD_CTRL_PTA     0x00800000
#     define QBMAN_FD_CTRL_PTV1    0x00400000

      UINT32 FlowContextLowWord;
      UINT32 FlowContextHighWord;
    } Simple;
  };
} QBMAN_FRAME_DESC;

/**
 * When frames are dequeued, the FDs show up inside "dequeue" result structures
 * (if at all, not all dequeue results contain valid FDs). This structure type
 * is intentionally defined without internal detail, and the only reason it
 * isn't declared opaquely (without size) is to allow the user to provide
 * suitably-sized (and aligned) memory for these entries.
 */
typedef struct _QBMAN_DEQUEUE_ENTRY {
  UINT32 DontManipulateDirectly[16];

# define QBMAN_DEQUEUE_STAT_FQEMPTY       0x80
# define QBMAN_DEQUEUE_STAT_HELDACTIVE    0x40
# define QBMAN_DEQUEUE_STAT_FORCEELIGIBLE 0x20
# define QBMAN_DEQUEUE_STAT_VALIDFRAME    0x10
# define QBMAN_DEQUEUE_STAT_ODPVALID      0x04
# define QBMAN_DEQUEUE_STAT_VOLATILE      0x02
# define QBMAN_DEQUEUE_STAT_EXPIRED       0x01
# define QBMAN_DEQUEUE_IDX(_Dq)           (((unsigned long)(_Dq) & 0x1ff) >> 6)
} QBMAN_DEQUEUE_ENTRY;

/**
 * Frame annotation status
 */
typedef struct _QBMAN_FRAME_ANNOTATION_STATUS {
  UINT8 Reserved;
  UINT8 Ppid;
  UINT16 Ifpid;
  UINT32 Status;
# define QBMAN_ETH_FAS_DISC    0x80000000
# define QBMAN_ETH_FAS_MS      0x40000000  /* MACSEC frame */
# define QBMAN_ETH_FAS_PTP     0x08000000
# define QBMAN_ETH_FAS_MC      0x04000000  /* multicast frame */
# define QBMAN_ETH_FAS_BC      0x02000000  /* broadcast frame */
# define QBMAN_ETH_FAS_KSE     0x00040000
# define QBMAN_ETH_FAS_EOFHE   0x00020000
# define QBMAN_ETH_FAS_MNLE    0x00010000
# define QBMAN_ETH_FAS_TIDE    0x00008000
# define QBMAN_ETH_FAS_PIEE    0x00004000
# define QBMAN_ETH_FAS_FLE     0x00002000  /* Frame length error */
# define QBMAN_ETH_FAS_FPE     0x00001000  /* Frame physical error */
# define QBMAN_ETH_FAS_PTE     0x00000080
# define QBMAN_ETH_FAS_ISP     0x00000040
# define QBMAN_ETH_FAS_PHE     0x00000020
# define QBMAN_ETH_FAS_BLE     0x00000010
# define QBMAN_ETH_FAS_L3CV    0x00000008  /* L3 csum validation performed */
# define QBMAN_ETH_FAS_L3CE    0x00000004  /* L3 csum error */
# define QBMAN_ETH_FAS_L4CV    0x00000002  /* L4 csum validation performed */
# define QBMAN_ETH_FAS_L4CE    0x00000001  /* L4 csum error */
# define QBMAN_ETH_RX_ERROR_MASK  (QBMAN_ETH_FAS_DISC  | \
           QBMAN_ETH_FAS_KSE  | \
           QBMAN_ETH_FAS_EOFHE  | \
           QBMAN_ETH_FAS_MNLE  | \
           QBMAN_ETH_FAS_TIDE  | \
           QBMAN_ETH_FAS_PIEE  | \
           QBMAN_ETH_FAS_FLE  | \
           QBMAN_ETH_FAS_FPE  | \
           QBMAN_ETH_FAS_PTE  | \
           QBMAN_ETH_FAS_ISP  | \
           QBMAN_ETH_FAS_PHE  | \
           QBMAN_ETH_FAS_BLE  | \
           QBMAN_ETH_FAS_L3CE  | \
           QBMAN_ETH_FAS_L4CE)

# define QBMAN_ETH_RX_UNSUPPORTED_MASK  QBMAN_ETH_FAS_MS

# define QBMAN_ETH_TXCONF_ERROR_MASK  (QBMAN_ETH_FAS_KSE  | \
           QBMAN_ETH_FAS_EOFHE  | \
           QBMAN_ETH_FAS_MNLE  | \
           QBMAN_ETH_FAS_TIDE)
} QBMAN_FRAME_ANNOTATION_STATUS;

enum _QBMAN_SDQCR_DCT {
  QBMAN_SDQCR_DCT_NULL = 0,
  QBMAN_SDQCR_DCT_PRIO_ICS,
  QBMAN_SDQCR_DCT_ACTIVE_ICS,
  QBMAN_SDQCR_DCT_ACTIVE
};

enum _QBMAN_SDQCR_FC {
  QBMAN_SDQCR_FC_ONE = 0,
  QBMAN_SDQCR_FC_UP_TO_3 = 1
};

enum _QBMAN_ENQUEUE_CMD {
  /* No enqueue, primarily for plugging ORP gaps for dropped frames */
  QBMAN_EQ_CMD_EMPTY,
  /* DMA an enqueue response once complete */
  QBMAN_EQ_CMD_RESPOND,
  /* DMA an enqueue response only if the enqueue fails */
  QBMAN_EQ_CMD_RESPOND_REJECT
};

enum _QBMAN_PULL_DT {
  QBMAN_PULL_DT_CHANNEL,
  QBMAN_PULL_DT_WORKQUEUE,
  QBMAN_PULL_DT_FRAMEQUEUE
};

EFI_STATUS
QbmanSwPortalInit (
  DPAA2_QBMAN_PORTAL *DpioQbmanPortal,
  UINT64 qbman_portal_ce_offset,
  UINT64 qbman_portal_ci_offset
  );

VOID
QbmanReleaseDescClear (
  QBMAN_RELEASE_DESC *QbmanReleaseDesc
  );

VOID
QbmanReleaseDescSetBpid (
  QBMAN_RELEASE_DESC *QbmanReleaseDesc,
  UINT16 HwBufferPoolbId
  );

VOID
QbmanEnqueueDescClear (
  QBMAN_ENQUEUE_DESC *QbmanEnqueueDesc
  );

VOID
QbmanEnqueueDescSetNoOrp (
  QBMAN_ENQUEUE_DESC *QbmanEnqueueDesc,
  BOOLEAN RespondSuccess
  );

VOID
QbmanEnqueueDescSetQueueDest (
  QBMAN_ENQUEUE_DESC *QbmanEnqueueDesc,
  UINT32 QueueDestId,
  UINT32 QueueDestBin,
  UINT32 QueueDestPrio
  );

BOOLEAN
QbmanSwPortalReleaseBuffers (
  DPAA2_QBMAN_PORTAL *QbmanPortal,
  const QBMAN_RELEASE_DESC *QbmanReleaseDesc,
  const UINT64 *Buffers,
  UINT32 NumBuffers
  );

UINT32
QbmanSwPortalAcquireBuffers (
  DPAA2_QBMAN_PORTAL *QbmanPortal,
  UINT16 HwBufferPoolId,
  UINT64 *Buffers,
  UINT32 NumBuffers
  );

BOOLEAN
QbmanSwPortalEnqueue (
  DPAA2_QBMAN_PORTAL *QbmanPortal,
  const QBMAN_ENQUEUE_DESC *QbmanEnqueueDesc,
  const QBMAN_FRAME_DESC *QbmanFrameDesc
  );

const QBMAN_DEQUEUE_ENTRY *
QbmanSwPortalGetNextDqrrEntry (
  DPAA2_QBMAN_PORTAL *QbmanPortal
  );

VOID
QbmanSwPortalConsumeDqrrEntry (
  DPAA2_QBMAN_PORTAL *QbmanPortal,
  const QBMAN_DEQUEUE_ENTRY *DequeueEntry
  );

UINT32
QbmanGetFlagsFromDequeueEntry (
  const QBMAN_DEQUEUE_ENTRY *DequeueEntry
  );

const
QBMAN_FRAME_DESC *
QbmanGetFrameDescFromDequeueEntry (
  const QBMAN_DEQUEUE_ENTRY *DequeueEntry
  );

VOID
QbmanPullDescClear (
  QBMAN_PULL_DESC *QbmanPullDesc
  );

VOID
QbmanPullDescSetNumframes (
  QBMAN_PULL_DESC *QbmanPullDesc, UINT8 NumFrames
  );

VOID
QbmanPullDescSetFrameDesc (
  QBMAN_PULL_DESC *QbmanPullDesc,
  UINT32 FrameQueueId
  );

BOOLEAN
QbmanSwPortalPull (
  DPAA2_QBMAN_PORTAL *QbmanPortal,
  QBMAN_PULL_DESC *QbmanPullDesc
  );

#endif /* __QBMAN_H__ */
