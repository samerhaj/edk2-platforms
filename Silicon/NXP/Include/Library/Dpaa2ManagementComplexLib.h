/** Dpaa2ManagementComplexLib.h
  DPAA2 Management Complex library interface

  Copyright 2017 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution. The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#ifndef __DPAA2_MANAGEMENT_COMPLEX_LIB_H__
#define __DPAA2_MANAGEMENT_COMPLEX_LIB_H__

#include <Library/Dpaa2McIo.h>
#include <Library/Dpaa2McInterfaceLib/fsl_dprc.h>
#include <Library/Dpaa2EthernetMacLib.h>
#include <Uefi.h>

/**
 * Rx buffer size in bytes
 */
#define DPAA2_ETH_RX_FRAME_BUFFER_SIZE      2048
#define DPAA2_ETHERNET_HEADER_SIZE          36
#define MC_VERSION(Major, Minor)            (Major*10 + Minor)

/**
 * DPAA2 QBman software portal
 */
typedef struct _DPAA2_QBMAN_PORTAL {
  /**
   * Lock to serialize access to the QBman portal
   */
  EFI_LOCK QbmanPortalLock;

  /**
   * Pointer to shadow copy of Cache-enabled MMIO registers
   * (must be cache-line aligned)
   */
  VOID *CacheEnabledRegsShadow;

  /**
   * Address of Cache-enabled MMIO registers
   */
  EFI_PHYSICAL_ADDRESS CacheEnabledBaseAddr;

  /**
   * Address of Cache-inhibited MMIO registers
   */
  EFI_PHYSICAL_ADDRESS CacheInhibitedBaseAddr;

  /**
   * Management command states
   */
  struct {
    enum _QBMAN_PORTAL_CMD_STATES {
      QBMAN_PORTAL_CMD_CAN_START,
      QBMAN_PORTAL_CMD_CAN_SUBMIT,
      QBMAN_PORTAL_CMD_CAN_POLL_RESULT,
    } State;

    UINT32 ValidBit; /* 0x00 or 0x80 */
  } MgmtCmd;

  /**
   * Push dequeues
   */
  UINT32 Sdq;

  /**
   * Volatile dequeues
   */
  struct {
    volatile INT32 Busy;
    UINT32 ValidBit; /* QBMAN_VALID_BIT */
    struct _DPAA2_DQ *Storage; /* set to NULL for DQRRs */
    UINT32 Token;
  } Vdq;

  /*
   * DQRR (Dequeue Response Ring)
   */
  struct {
    UINT32 NextIdx;
    UINT32 ValidBit;
  } Dqrr;
} DPAA2_QBMAN_PORTAL;

/**
 * State kept for MC objects that make up a DPAA2 Ethernet network interface
 */
typedef struct _DPAA2_NETWORK_INTERFACE {
  /**
   * Signature for run-time type checking
   */
  UINT32 Signature;
# define DPAA2_NETWORK_INTERFACE_SIGNATURE SIGNATURE_32('D', 'P', 'N', 'I')

  /**
   * Flag indicating the MC objects for this network interface have been
   * created in the MC
   */
  volatile BOOLEAN CreatedInMc;

  /**
   * Flag indicating the Network interface has been started in the MC
   */
  volatile BOOLEAN StartedInMc;

  /**
   * DPRC handle in the MC
   */
  UINT16 DprcHandle;

  /**
   * DPRC object Id
   */
  INT32 DprcId;

  /**
   * McIo object that uses the MC portal associated with the 'DprcId' DPRC
   */
  DPAA2_MC_IO DprcMcIo;

  /**
   * DPBP handle in the MC
   */
  UINT16 DpbpHandle;

  /**
   * Hardware buffer pool Id associated with the the DPBP object
   */
  UINT16 DpbpHwBufferPoolId;

  /**
   * DPBP object Id
   */
  INT32 DpbpId;

  /**
   * DPIO handle in the MC
   */
  UINT16 DpioHandle;

  /**
   * DPIO object Id
   */
  INT32 DpioId;

  /**
   * QBman software portal associated with the DPIO
   */
  DPAA2_QBMAN_PORTAL DpioQbmanPortal;

  /**
   * DPNI handle in the MC
   */
  UINT16 DpniHandle;

  /**
   * DPNI object Id
   */
  INT32 DpniId;

  /**
   * DPNI extended configuration DMA buffer
   * (It must be 256 bytes long and aligned on a 64-bit boundary)
   */
  UINT64 DpniExtCfgDmaBuffer[256 / sizeof (UINT64)];

  /**
   * DPMAC handle in the MC
   */
  UINT16 DpmacHandle;

  /**
   * DPMAC Id
   */
  INT32 DpmacId;

  /**
   * Ethernet frame private data size
   */
  UINT16 FramePrivateDataSize;

  /**
   * Tx payload data offset in an Ethernet frame
   */
  UINT16 TxDataOffset;

  /**
   * Tx flow Id
   */
  UINT16 TxFlowId;

  /**
   * Tx queue destination Id
   */
  UINT16 TxQueueDestinationId;

  /**
   * Rx default frame queue Id
   */
  UINT32 RxDefaultFrameQueueId;

  /**
   * Tx confirmations (Tx completions) queue Id
   */
  UINT32 TxConfirmationsQueueId;

  /**
   * DPMAC end-point of connection DPMAC<->DPNI
   */
  struct dprc_endpoint DpmacEndpoint;

  /**
   * DPNI end-point of connection DPMAC<->DPNI
   */
  struct dprc_endpoint DpniEndpoint;

} DPAA2_NETWORK_INTERFACE;


EFI_STATUS
Dpaa2McInit (
  VOID
  );

VOID
Dpaa2McExit (
  VOID
  );

EFI_STATUS
Dpaa2McDeployDpl (
  VOID
  );

EFI_STATUS
Dpaa2McCreateNetworkInterface (
  DPAA2_NETWORK_INTERFACE *Dpaa2NetInterface
  );

VOID
Dpaa2McDestroyNetworkInterface (
  DPAA2_NETWORK_INTERFACE *Dpaa2NetInterface
  );

EFI_STATUS
Dpaa2McInitializeNetworkInterface (
  DPAA2_NETWORK_INTERFACE *Dpaa2NetInterface,
  EFI_MAC_ADDRESS         *MacAddr,
  UINT32                  *Mtu,
  WRIOP_DPMAC             *WriopDpmac
  );

VOID
Dpaa2McShutdownNetworkInterface (
  DPAA2_NETWORK_INTERFACE *Dpaa2NetInterface
  );

VOID
Dpaa2McResetNetworkInterface (
  DPAA2_NETWORK_INTERFACE *Dpaa2NetInterface
  );

BOOLEAN
Dpaa2McCheckDpniLink (
  DPAA2_NETWORK_INTERFACE *Dpaa2NetInterface
  );

EFI_STATUS
Dpaa2McNetworkInterfaceTransmit (
  DPAA2_NETWORK_INTERFACE *Dpaa2NetInterface,
  WRIOP_DPMAC             *WriopDpmac,
  UINTN                   HdrSize,
  UINTN                   BuffSize,
  VOID                    *Data,
  EFI_MAC_ADDRESS         *SrcAddr,
  EFI_MAC_ADDRESS         *DstAddr,
  UINT16                  *Protocol,
  UINT64                  *QBmanTxBufferAddrOut
  );

EFI_STATUS
Dpaa2McNetworkInterfaceCheckTxCompletion (
  DPAA2_NETWORK_INTERFACE *Dpaa2NetInterface,
  UINT64                  *QBmanTxBufferAddrOut
  );

EFI_STATUS
Dpaa2McNetworkInterfaceReceive (
  DPAA2_NETWORK_INTERFACE *Dpaa2NetInterface,
  WRIOP_DPMAC             *WriopDpmac,
  UINTN                   *BuffSize,
  VOID                    *Data,
  EFI_MAC_ADDRESS         *SrcAddr,
  EFI_MAC_ADDRESS         *DstAddr,
  UINT16                  *Protocol
  );

EFI_STATUS
Dpaa2McAddMulticastMacAddress (
  DPAA2_NETWORK_INTERFACE *Dpaa2NetInterface,
  EFI_MAC_ADDRESS         *MulticastMacAddr
  );

EFI_STATUS
Dpaa2McRemoveMulticastMacAddress (
  DPAA2_NETWORK_INTERFACE *Dpaa2NetInterface,
  EFI_MAC_ADDRESS         *MulticastMacAddr
  );

#endif /* __DPAA2_MANAGEMENT_COMPLEX_LIB_H__ */
