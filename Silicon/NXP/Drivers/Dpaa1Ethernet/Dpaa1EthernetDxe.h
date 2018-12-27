/** @file  Dpaa1EthernetDxe.h

  DPAA1 Ethernet DXE driver private declarations

  Copyright (c) 2016, Freescale Semiconductor, Inc. All rights reserved.

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#ifndef __DPAA1_ETHERTNET_DXE_H__
#define __DPAA1_ETHERTNET_DXE_H__

#include <Library/Dpaa1EthernetMacLib.h>
#include <Protocol/SimpleNetwork.h>
#include <Library/UefiLib.h>

/* QBMAN Definitions */
#define BMAN_SWP_ISDR_REG          0x3E80
#define BMAN_NUM_PORTALS           10
#define BMAN_MEM_BASE              0x508000000
#define BMAN_MEM_SIZE              0x08000000
#define BMAN_CINH_SIZE             (BMAN_MEM_SIZE >> 1)
#define BMAN_SP_CINH_SIZE          0x10000
#define BMAN_CENA_SIZE             (BMAN_MEM_SIZE >> 1)
#define BMAN_CINH_BASE             (BMAN_MEM_BASE + BMAN_CENA_SIZE)

#define QMAN_SWP_ISDR_REG          0x3680
#define QMAN_NUM_PORTALS           10
#define QMAN_MEM_BASE              0x500000000
#define QMAN_MEM_SIZE              0x08000000
#define QMAN_CINH_SIZE             (QMAN_MEM_SIZE >> 1)
#define QMAN_SP_CINH_SIZE          0x10000
#define QMAN_CENA_SIZE             (QMAN_MEM_SIZE >> 1)
#define QMAN_CINH_BASE             (QMAN_MEM_BASE + QMAN_CENA_SIZE)

#define MAX_BMAN_PORTALS           (BMAN_CINH_SIZE / BMAN_SP_CINH_SIZE)
#define MAX_QMAN_PORTALS           (QMAN_CINH_SIZE / QMAN_SP_CINH_SIZE)

typedef VOID
SerDesProbeLanesCallBack(
  UINT8 LaneProtocol,
  VOID *Arg
);

VOID
SerDesProbeLanes(
  SerDesProbeLanesCallBack *SerDesLaneProbeCallback,
  VOID *Arg
);

VOID
Dpaa1ParseSerDes(UINT8 LaneProtocol,
                 VOID *Arg);
VOID
Dpaa1DiscoverFmanMemac(FMAN_MEMAC_ID MemacId,
                       PHY_INTERFACE_TYPE PhyInterfaceType,
                       VOID *Arg);

#define DPAA1_MDIO_BUSES_COUNT          2
/**
 * DPAA1 Ethernet Device Path
 */
typedef struct _DPAA1_DEVICE_PATH {
  MAC_ADDR_DEVICE_PATH      MacAddrDevicePath;
  EFI_DEVICE_PATH_PROTOCOL  EndMarker;
} DPAA1_DEVICE_PATH;

/**
 * DPAA1 Ethernet Device Instance
 */
typedef struct _DPAA1_ETHERNET_DEVICE {
  /**
   * Signature for run-time type checking
   */
  UINT32 Signature;
# define DPAA1_ETHERNET_DEVICE_SIGNATURE SIGNATURE_32('D', 'P', 'A', 'A')

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
  DPAA1_DEVICE_PATH DevicePath;

  /**
   * Pointer to corresponding FMAN MEMAC
   */
  FMAN_MEMAC *FmanMemac;

  ETH_DEVICE *FmanEthDevice;

  /**
   * Flag indicating if the PHY for this Ethernet device has
   * already been in initialized.
   */
  BOOLEAN PhyInitialized;

  /**
   * Node in the global list of DPAA1 Ethernet devices
   */
  LIST_ENTRY ListNode;
} DPAA1_ETHERNET_DEVICE;

VOID
Dpaa1StopNetworkInterface (
  IN  DPAA1_ETHERNET_DEVICE *EthDev
  );

VOID
Dpaa1StartNetworkInterface (
  IN  DPAA1_ETHERNET_DEVICE *EthDev
  );

#define SNP_TO_DPAA1_DEV(_SnpPtr) \
        CR(_SnpPtr, DPAA1_ETHERNET_DEVICE, Snp, DPAA1_ETHERNET_DEVICE_SIGNATURE)

EFI_STATUS
Dpaa1Transmit (
  IN  DPAA1_ETHERNET_DEVICE *Dpaa1EthDev,
  UINTN HdrSize,
  UINTN BuffSize,
  VOID *Data,
  EFI_MAC_ADDRESS *SrcAddr,
  EFI_MAC_ADDRESS *DstAddr,
  UINT16 *Protocol
  );

EFI_STATUS
Dpaa1Receive(
  IN  DPAA1_ETHERNET_DEVICE *Dpaa1EthDev,
  UINTN *BuffSize,
  VOID *Data,
  EFI_MAC_ADDRESS *SrcAddr,
  EFI_MAC_ADDRESS *DstAddr,
  UINT16 *Protocol
);

#endif /* __DPAA1_ETHERTNET_DXE_H__ */
