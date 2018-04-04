/** @file

  DPAA2 Ethernet DXE driver private declarations

  Copyright 2017 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#ifndef __DPAA2_ETHERTNET_DXE_H__
#define __DPAA2_ETHERTNET_DXE_H__

#include <Library/Dpaa2EthernetMacLib.h>
#include <Library/Dpaa2ManagementComplexLib.h>
#include <Library/UefiLib.h>
#include <Protocol/SimpleNetwork.h>

#define DPAA2_ETHERNET_DRIVER_VERSION   0x1

/**
 * Bit masks for first byte of a MAC address
 */
# define MAC_MULTICAST_ADDRESS_MASK 0x1
# define MAC_PRIVATE_ADDRESS_MASK   0x2

/**
 * DPAA2 Ethernet Device Path
 */
typedef struct _DPAA2_DEVICE_PATH {
  MAC_ADDR_DEVICE_PATH      MacAddrDevicePath;
  EFI_DEVICE_PATH_PROTOCOL  EndMarker;
} DPAA2_DEVICE_PATH;

/**
 * DPAA2 Ethernet Device Instance
 */
typedef struct _DPAA2_ETHERNET_DEVICE {
  /**
   * Signature for run-time type checking
   */
  UINT32 Signature;
# define DPAA2_ETHERNET_DEVICE_SIGNATURE SIGNATURE_32('D', 'P', 'A', 'A')

  /**
   * UEFI driver model handle associated with this device instance
   */
  EFI_HANDLE ControllerHandle;

  /**
   * SNP protocol instance
   */
  EFI_SIMPLE_NETWORK_PROTOCOL Snp;

  /**
   * SNP mode instance
   */
  EFI_SIMPLE_NETWORK_MODE SnpMode;

  /**
   * SNP statistics instance
   */
  EFI_NETWORK_STATISTICS Stats;

  /**
   * Device path protocol instance
   */
  DPAA2_DEVICE_PATH DevicePath;

  /**
   * Pointer to corresponding WRIOP DPMAC
   */
  WRIOP_DPMAC *WriopDpmac;

  /**
   * Flag indicating if the PHY for this Ethernet device has
   * already been in initialized.
   */
  BOOLEAN PhyInitialized;

  /**
   * DPAA2 network interface object made of
   * MC objects
   */
  DPAA2_NETWORK_INTERFACE Dpaa2NetInterface;

  /**
   * Node in the global list of DPAA2 Ethernet devices
   */
  LIST_ENTRY ListNode;

  /**
   * Head of the linked list of Tx frames in flight
   */
  LIST_ENTRY TxFramesInFlightList;

  /**
   * TPL-based lock to serialize access to TxFramesInFlightList
   */
  EFI_LOCK TxFramesInFlightLock;

} DPAA2_ETHERNET_DEVICE;

/**
 * Information kept for a Tx frame that is in flight
 */
typedef struct _DPAA2_TX_FRAME_IN_FLIGHT {
  /**
   * Signature for run-time type checking
   */
  UINT32 Signature;
# define DPAA2_TX_FRAME_IN_FLIGHT_SIGNATURE SIGNATURE_32('D', 'P', 'T', 'X')

  /**
   * Pointer to the UEFI networking stack's Tx buffer associated with the frame
   */
  VOID *UefiTxBuffer;

  /**
   * Address of QBMAN buffer ssociated with the frame
   */
  UINT64 QBmanTxBufferAddr;

  /**
   * Node in the list of Tx frames in flight for the corresponding Ethernet device
   */
  LIST_ENTRY ListNode;
} DPAA2_TX_FRAME_IN_FLIGHT;

/**
 * DPAA2 Ethernet driver global control block
 */
typedef struct _DPAA2_ETHERNET_DRIVER {
  /**
   * Head of the linked list of enabled DPMACs
   */
  LIST_ENTRY DpmacsList;

  /**
   * Head of the linked list of DPAA2 Ethernet devices
   */
  LIST_ENTRY Dpaa2EthernetDevicesList;

  /**
   * Exit boot services event
   */
  EFI_EVENT ExitBootServicesEvent;

  /**
   * MC final status to be communicated to the OS
   */
  EFI_STATUS McStatus;

} DPAA2_ETHERNET_DRIVER;

STATIC
EFI_STATUS
EFIAPI
Dpaa2SnpStart (
  IN  EFI_SIMPLE_NETWORK_PROTOCOL  *Snp
  );


STATIC
EFI_STATUS
EFIAPI
Dpaa2SnpStop (
  IN        EFI_SIMPLE_NETWORK_PROTOCOL* Snp
  );

STATIC
EFI_STATUS
EFIAPI
Dpaa2SnpInitialize (
  IN        EFI_SIMPLE_NETWORK_PROTOCOL* Snp,
  IN        UINTN                        RxBufferSize    OPTIONAL,
  IN        UINTN                        TxBufferSize    OPTIONAL
  );

STATIC
EFI_STATUS
EFIAPI
Dpaa2SnpReset (
  IN        EFI_SIMPLE_NETWORK_PROTOCOL* Snp,
  IN        BOOLEAN Verification
  );

STATIC
EFI_STATUS
EFIAPI
Dpaa2SnpShutdown (
  IN        EFI_SIMPLE_NETWORK_PROTOCOL *Snp
  );

STATIC
EFI_STATUS
EFIAPI
Dpaa2SnpReceiveFilters (
  IN  EFI_SIMPLE_NETWORK_PROTOCOL  *Snp,
  IN  UINT32                       Enable,
  IN  UINT32                       Disable,
  IN  BOOLEAN                      ResetMCastFilter,
  IN  UINTN                        MCastFilterCnt  OPTIONAL,
  IN  EFI_MAC_ADDRESS              *MCastFilter  OPTIONAL
  );

STATIC
EFI_STATUS
EFIAPI
Dpaa2SnpStationAddress (
  IN  EFI_SIMPLE_NETWORK_PROTOCOL  *Snp,
  IN  BOOLEAN                      Reset,
  IN  EFI_MAC_ADDRESS              *New
  );

STATIC
EFI_STATUS
EFIAPI
Dpaa2SnpStatistics (
  IN        EFI_SIMPLE_NETWORK_PROTOCOL* Snp,
  IN        BOOLEAN Reset,
  IN  OUT   UINTN *StatSize,
  OUT   EFI_NETWORK_STATISTICS *Statistics
  );

STATIC
EFI_STATUS
EFIAPI
Dpaa2SnpMcastIptoMac (
  IN        EFI_SIMPLE_NETWORK_PROTOCOL* Snp,
  IN        BOOLEAN IsIpv6,
  IN        EFI_IP_ADDRESS *Ip,
  OUT       EFI_MAC_ADDRESS *McastMac
  );

STATIC
EFI_STATUS
EFIAPI
Dpaa2SnpNvData (
  IN        EFI_SIMPLE_NETWORK_PROTOCOL *Snp,
  IN        BOOLEAN read_write,
  IN        UINTN offset,
  IN        UINTN buff_size,
  IN  OUT   VOID *data
  );

STATIC
EFI_STATUS
EFIAPI
Dpaa2SnpGetStatus (
  IN   EFI_SIMPLE_NETWORK_PROTOCOL  *Snp,
  OUT  UINT32                       *IrqStat  OPTIONAL,
  OUT  VOID                         **TxBuff  OPTIONAL
  );

STATIC
EFI_STATUS
EFIAPI
Dpaa2SnpTransmit (
  IN  EFI_SIMPLE_NETWORK_PROTOCOL  *Snp,
  IN  UINTN                        HdrSize,
  IN  UINTN                        BuffSize,
  IN  VOID*                        Data,
  IN  EFI_MAC_ADDRESS              *SrcAddr  OPTIONAL,
  IN  EFI_MAC_ADDRESS              *DstAddr  OPTIONAL,
  IN  UINT16                       *Protocol OPTIONAL
  );

STATIC
EFI_STATUS
EFIAPI
Dpaa2SnpReceive (
  IN        EFI_SIMPLE_NETWORK_PROTOCOL   *Snp,
  OUT       UINTN *HdrSize                OPTIONAL,
  IN        OUT   UINTN                   *BuffSize,
  OUT       VOID *                        Data,
  OUT       EFI_MAC_ADDRESS *SrcAddr      OPTIONAL,
  OUT       EFI_MAC_ADDRESS *DstAddr      OPTIONAL,
  OUT       UINT16 *Protocol              OPTIONAL
  );

extern DPAA2_PHY_MDIO_BUS gDpaa2MdioBuses[];
extern VOID ProbeDpaaLanes (VOID *Arg);

#define SNP_TO_DPAA2_DEV(_SnpPtr) \
        CR(_SnpPtr, DPAA2_ETHERNET_DEVICE, Snp, DPAA2_ETHERNET_DEVICE_SIGNATURE)

#endif /* __DPAA2_ETHERTNET_DXE_H__ */
