/** @file
  DPAA library Interface

  Copyright 2017 NXP

  This Program And The Accompanying Materials
  Are Licensed And Made Available Under The Terms And Conditions Of The BSD
  License Which Accompanies This Distribution. The Full Text Of The License
  May Be Found At
  http://Opensource.Org/Licenses/Bsd-License.Php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#ifndef __DPAA_LIB_H__
#define __DPAA_LIB_H__

#include <Library/Dpaa1EthernetMacLib.h>
#include <Uefi.h>

/* Common BD flags */
#define BD_LAST                    0x0800

/* Rx status flags */
#define Rx_EMPTY          		0x8000
#define Rx_LAST           		BD_LAST
#define Rx_FIRST          		0x0400
#define Rx_PHYS_ERR              	0x0008
#define Rx_SIZE_ERR              	0x0004
#define Rx_ERROR          		(Rx_PHYS_ERR | Rx_SIZE_ERR)

/* Tx status flags */
#define Tx_READY          		0x8000
#define Tx_LAST           		BD_LAST

#define NUM_FMAN                        1
#define FMAN_MEM_SIZE                   0x60000
/**
 Wrapper function to call Fman init for initializing given
 FMAN instance
*/
EFI_STATUS
DpaaFrameManagerInit (VOID);

/**
 Function to initialize Rx Port
 Allocate global port structure from FMAN Memory
 Set Rx queue descriptor pointer
 Initialize Rx ring descriptor
*/
EFI_STATUS FmRxPortParamInit (
  IN  ETH_DEVICE *FmanEthDevice
  );

/**
 Function to initialize Tx Port
 Allocate global port structure from FMAN Memory
 Set Tx queue descriptor pointer
 Initialize Rx ring descriptor
*/
EFI_STATUS FmTxPortParamInit (
  IN  ETH_DEVICE *FmanEthDevice
  );

/**
 Function to initialize bmi Rx Port
 Set BMI port to independent mode
 Enable Rx statistics
*/
VOID BmiRxPortInit (
  IN  BMI_RX_PORT *Port
  );

/**
 Function to initialize bmi Tx Port
 Set BMI port to independent mode
 Enable Tx statistics
*/
VOID BmiTxPortInit (
  IN  BMI_TX_PORT *Port
  );

/**
 Function to populate Ethernet Device Rx/Tx Port,Number,
 Type,1G or 10G
*/
VOID PopulateEthDev (
  IN  ETH_DEVICE *FmanEthDevice,
  IN  FMAN_MEMAC_ID Id
  );

/**
 Function to initialize Mac Controller Structure(Base Address,Regsiters)
*/
EFI_STATUS EthDevInitMac (
  IN  ETH_DEVICE *FmEth
  );

VOID MemWriteMasked (
  IN  UINT16 *Addr, 
  IN  UINT16 Data
  );

UINT16 MemReadMasked (
  IN  UINT16 *Addr
  );

/**
 Function to transmit ethernet frame on DPAA1 network interface
 Enqueue buffer to TxBD circular queue for transmitting
*/
EFI_STATUS
TransmitFrame (
  IN  ETH_DEVICE *FmanEthDevice,
  IN VOID * Data, 
  IN UINTN BuffSize
  );

EFI_STATUS
GetTransmitStatus (
  IN  ETH_DEVICE *FmanEthDevice,
  IN VOID        **TxBuf
  );

/**
 Function to receive ethernet frame from DPAA1 network interface
*/
EFI_STATUS
ReceiveFrame(
  ETH_DEVICE *FmanEthDevice,
  UINTN *BuffSize,
  VOID *Data
  );

/**
 Function to set MAC address for ethernet device
*/
VOID
GenerateMacAddress(
  IN  UINT32 SocUniqueId,
  IN  FMAN_MEMAC_ID MemacId,
  OUT EFI_MAC_ADDRESS *MacAddrBuf
  );

/**
 Function to get Unique for MAC address
*/
EFI_STATUS
GetNVSocUniqueId (
  OUT UINT32 *UniqueId
  );

UINT32
GetSocUniqueId(VOID);

#endif /* __DPAA_LIB_H__ */
