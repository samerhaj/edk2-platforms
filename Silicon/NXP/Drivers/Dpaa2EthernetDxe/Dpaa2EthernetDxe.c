/** @file

  DPAA2 Ethernet DXE driver

  Copyright 2017 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution. The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include <Library/BaseLib.h>
#include <Library/DevicePathLib.h>
#include <Library/DpaaDebugLib.h>
#include <Library/Dpaa2ManagementComplexLib.h>
#include <Library/Dpaa2EthernetMacLib.h>
#include <Library/Dpaa2EthernetPhyLib.h>
#include <Library/IoLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/NetLib.h>
#include <Library/PcdLib.h>
#include <Library/SysEepromLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiRuntimeServicesTableLib.h>
#include <Protocol/ComponentName2.h>
#include <Protocol/DevicePath.h>
#include <Protocol/PxeBaseCode.h>

#include "Dpaa2EthernetDxe.h"

/**
 * Debug mode flags for DPAA2 code
 */
UINT32 gDpaaDebugFlags = 0x0;
CHAR8  * CONST gDpaaDebugString = "DPAA2";

STATIC DPAA2_ETHERNET_DRIVER gDpaa2Driver = {
  .DpmacsList = INITIALIZE_LIST_HEAD_VARIABLE (gDpaa2Driver.DpmacsList),
  .Dpaa2EthernetDevicesList = INITIALIZE_LIST_HEAD_VARIABLE (gDpaa2Driver.Dpaa2EthernetDevicesList),
  .ExitBootServicesEvent = NULL
};

/**
 * Initializer template for a DPAA2 Ethernet device instance
 */
STATIC const DPAA2_ETHERNET_DEVICE gDpaa2EthernetDeviceInitTemplate = {
  .Signature = DPAA2_ETHERNET_DEVICE_SIGNATURE,
  .ControllerHandle = NULL,
  .Snp = {
    .Revision = EFI_SIMPLE_NETWORK_PROTOCOL_REVISION,
    .Start = Dpaa2SnpStart,
    .Stop = Dpaa2SnpStop,
    .Initialize = Dpaa2SnpInitialize,
    .Reset = Dpaa2SnpReset,
    .Shutdown = Dpaa2SnpShutdown,
    .ReceiveFilters = Dpaa2SnpReceiveFilters,
    .StationAddress = Dpaa2SnpStationAddress,
    .Statistics = Dpaa2SnpStatistics,
    .MCastIpToMac = Dpaa2SnpMcastIptoMac,
    .NvData = Dpaa2SnpNvData,
    .GetStatus = Dpaa2SnpGetStatus,
    .Transmit = Dpaa2SnpTransmit,
    .Receive = Dpaa2SnpReceive,
    .WaitForPacket = NULL,
    .Mode = NULL,
  },

  .SnpMode = {
    .State = EfiSimpleNetworkStopped,
    .HwAddressSize = NET_ETHER_ADDR_LEN,
    .MediaHeaderSize = sizeof (ETHER_HEAD),
    .MaxPacketSize = DPAA2_ETH_RX_FRAME_BUFFER_SIZE,
    .NvRamSize = 0,                 // No NVRAM
    .NvRamAccessSize = 0,           // No NVRAM
    .ReceiveFilterMask = EFI_SIMPLE_NETWORK_RECEIVE_UNICAST |
                         EFI_SIMPLE_NETWORK_RECEIVE_MULTICAST |
                         EFI_SIMPLE_NETWORK_RECEIVE_BROADCAST |
                         EFI_SIMPLE_NETWORK_RECEIVE_PROMISCUOUS |
                         EFI_SIMPLE_NETWORK_RECEIVE_PROMISCUOUS_MULTICAST,
    .ReceiveFilterSetting = 0,
    .MaxMCastFilterCount = MAX_MCAST_FILTER_CNT,
    .MCastFilterCount = 0,
    .MCastFilter = { { .Addr = { 0 } } },
    .IfType = NET_IFTYPE_ETHERNET,
    .MacAddressChangeable = TRUE,
    .MultipleTxSupported = TRUE,
    .MediaPresentSupported = TRUE,
    .MediaPresent = FALSE,
    .BroadcastAddress = { .Addr = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF } },
    .CurrentAddress = { .Addr = { 0 } },
    .PermanentAddress = { .Addr = { 0 } },
  },

  .Stats = { 0 },

  .DevicePath = {
    .MacAddrDevicePath = {
      .Header = {
        .Type = MESSAGING_DEVICE_PATH,
        .SubType = MSG_MAC_ADDR_DP,
        .Length = { (UINT8)sizeof (MAC_ADDR_DEVICE_PATH),
                    (UINT8)(sizeof (MAC_ADDR_DEVICE_PATH) >> 8) },
      },

      .MacAddress = { .Addr = { 0 } },
      .IfType = NET_IFTYPE_ETHERNET,
    },

    .EndMarker = {
      .Type = END_DEVICE_PATH_TYPE,
      .SubType = END_ENTIRE_DEVICE_PATH_SUBTYPE,
      .Length = { (UINT8)sizeof (EFI_DEVICE_PATH_PROTOCOL), 0 }
    }
  },

  .WriopDpmac = NULL,
  .PhyInitialized = FALSE,
  .Dpaa2NetInterface = {
    .Signature = DPAA2_NETWORK_INTERFACE_SIGNATURE,
    .CreatedInMc = FALSE,
    .StartedInMc = FALSE
  },

  .TxFramesInFlightLock = EFI_INITIALIZE_LOCK_VARIABLE (TPL_CALLBACK)
};

STATIC
EFI_STATUS
EFIAPI
Dpaa2SnpValidateParameters (
  IN  EFI_SIMPLE_NETWORK_PROTOCOL  *Snp
  )
{
  EFI_SIMPLE_NETWORK_MODE *SnpMode;
  DPAA2_ETHERNET_DEVICE *Dpaa2EthDev;
  WRIOP_DPMAC *WriopDpmac;

  ASSERT (Snp != NULL);
  if (!Snp)
    return EFI_INVALID_PARAMETER;

  SnpMode = Snp->Mode;
  ASSERT (SnpMode != NULL);
  if (!SnpMode)
    return EFI_INVALID_PARAMETER;

  Dpaa2EthDev = SNP_TO_DPAA2_DEV (Snp);
  WriopDpmac = Dpaa2EthDev->WriopDpmac;
  if (!WriopDpmac) {
    ASSERT (WriopDpmac != NULL);
    return EFI_INVALID_PARAMETER;
  }
  return EFI_SUCCESS;
}

/**
   SNP protocol Start () function

   @param Snp   A pointer to the EFI_SIMPLE_NETWORK_PROTOCOL instance.

   @retval EFI_SUCCESS, on success
   @retval error code, on failure
 **/
STATIC
EFI_STATUS
EFIAPI
Dpaa2SnpStart (
  IN  EFI_SIMPLE_NETWORK_PROTOCOL  *Snp
 )
{
  EFI_STATUS Status;
  EFI_SIMPLE_NETWORK_MODE *SnpMode;
  DPAA2_ETHERNET_DEVICE *Dpaa2EthDev;
  WRIOP_DPMAC *WriopDpmac;

  Status = Dpaa2SnpValidateParameters (Snp);
  if (EFI_ERROR (Status)) {
    DPAA_ERROR_MSG ("%a (): Failed to Validate Snp Parameters (error %u)\n", __FUNCTION__, Status);
    return Status;
  }

  Dpaa2EthDev = SNP_TO_DPAA2_DEV (Snp);
  DPAA_DEBUG_MSG ("%a () called for DPAA2 Ethernet device 0x%p (%a) \n", __FUNCTION__,
                  Dpaa2EthDev, gWriopDpmacStrings[Dpaa2EthDev->WriopDpmac->Id]);

  SnpMode = Snp->Mode;
  WriopDpmac = Dpaa2EthDev->WriopDpmac;

  if (SnpMode->State == EfiSimpleNetworkStarted ||
      SnpMode->State == EfiSimpleNetworkInitialized) {
    DPAA_ERROR_MSG ("DPAA2 Ethernet device 0x%p (%a) already started\n",
                   Dpaa2EthDev, gWriopDpmacStrings[WriopDpmac->Id]);
    return EFI_ALREADY_STARTED;
  }

  ASSERT (SnpMode->State == EfiSimpleNetworkStopped);

  if (!Dpaa2EthDev->PhyInitialized) {
    /*
     * Initialize PHY:
     */
    Status = Dpaa2PhyInit (&WriopDpmac->Phy);
    if (EFI_ERROR (Status)) {
      DPAA_ERROR_MSG ("Failed to initialize PHY for DPAA2 Ethernet device 0x%p (%a) (error %u)\n",
                      Dpaa2EthDev, gWriopDpmacStrings[WriopDpmac->Id], Status);
      return Status;
    }

    Dpaa2EthDev->PhyInitialized = TRUE;
  }

  /*
   * Create DPAA2 network interface in the MC:
   */
  DPAA_INFO_MSG ("Creating DPAA2 Ethernet physical device for %a "
                 "(MAC address %02x:%02x:%02x:%02x:%02x:%02x) ...\n",
                 gWriopDpmacStrings[WriopDpmac->Id],
                 SnpMode->CurrentAddress.Addr[0],
                 SnpMode->CurrentAddress.Addr[1],
                 SnpMode->CurrentAddress.Addr[2],
                 SnpMode->CurrentAddress.Addr[3],
                 SnpMode->CurrentAddress.Addr[4],
                 SnpMode->CurrentAddress.Addr[5]);

  Status = Dpaa2McCreateNetworkInterface (&Dpaa2EthDev->Dpaa2NetInterface);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  SnpMode->State = EfiSimpleNetworkStarted;
  return EFI_SUCCESS;
}

/**
   SNP protocol Stop () function

   @param Snp   A pointer to the EFI_SIMPLE_NETWORK_PROTOCOL instance.

   @retval EFI_SUCCESS, on success
   @retval error code, on failure
 **/
STATIC
EFI_STATUS
EFIAPI
Dpaa2SnpStop (
  IN        EFI_SIMPLE_NETWORK_PROTOCOL* Snp
  )
{
  EFI_SIMPLE_NETWORK_MODE *SnpMode;
  DPAA2_ETHERNET_DEVICE *Dpaa2EthDev;
  WRIOP_DPMAC *WriopDpmac;
  EFI_STATUS Status;

  Status = Dpaa2SnpValidateParameters (Snp);
  if (EFI_ERROR (Status)) {
    DPAA_ERROR_MSG ("%a (): Failed to Validate Snp Parameters (error %u)\n", __FUNCTION__, Status);
    return Status;
  }

  Dpaa2EthDev = SNP_TO_DPAA2_DEV (Snp);
  DPAA_DEBUG_MSG ("%a () called for DPAA2 Ethernet device 0x%p (%a) \n", __FUNCTION__,
                  Dpaa2EthDev, gWriopDpmacStrings[Dpaa2EthDev->WriopDpmac->Id]);

  SnpMode = Snp->Mode;
  WriopDpmac = Dpaa2EthDev->WriopDpmac;

  if (SnpMode->State == EfiSimpleNetworkStopped) {
    DPAA_ERROR_MSG ("DPAA2 Ethernet device 0x%p (%a) not started\n",
                   Dpaa2EthDev, gWriopDpmacStrings[WriopDpmac->Id]);
    return EFI_NOT_STARTED;
  }

  if (SnpMode->State == EfiSimpleNetworkInitialized) {
    Dpaa2McShutdownNetworkInterface (&Dpaa2EthDev->Dpaa2NetInterface);
  } else {
    ASSERT (SnpMode->State == EfiSimpleNetworkStarted);
  }

  if (Dpaa2EthDev->Dpaa2NetInterface.CreatedInMc) {
    /*
     * Destroy DPAA2 network interface in the MC:
     */
    DPAA_INFO_MSG ("Destroying DPAA2 Ethernet physical device for %a ...\n",
                   gWriopDpmacStrings[WriopDpmac->Id]);
    Dpaa2McDestroyNetworkInterface (&Dpaa2EthDev->Dpaa2NetInterface);
  }

  EfiAcquireLock (&Dpaa2EthDev->TxFramesInFlightLock);

  if (!IsListEmpty (&Dpaa2EthDev->TxFramesInFlightList)) {
    /*
     * Destroy outstanding TxFrameInFlight objects:
     */
    DPAA2_TX_FRAME_IN_FLIGHT *TxFrameInFlight;
    LIST_ENTRY *ListNode;
    LIST_ENTRY *NextNode;

    DPAA_WARN_MSG ("WARNING: There were pending transmits for %a\n",
                   gWriopDpmacStrings[WriopDpmac->Id]);

    for (ListNode = GetFirstNode (&Dpaa2EthDev->TxFramesInFlightList);
         ListNode != &Dpaa2EthDev->TxFramesInFlightList;
         ListNode = NextNode) {
      TxFrameInFlight = CR (ListNode, DPAA2_TX_FRAME_IN_FLIGHT, ListNode,
                           DPAA2_TX_FRAME_IN_FLIGHT_SIGNATURE);
      NextNode = RemoveEntryList (&TxFrameInFlight->ListNode);
      FreePool (TxFrameInFlight);
    }
  }

  EfiReleaseLock (&Dpaa2EthDev->TxFramesInFlightLock);
  SnpMode->State = EfiSimpleNetworkStopped;
  return EFI_SUCCESS;
}

/**
   SNP protocol Initialize () function

   @param Snp           A pointer to the EFI_SIMPLE_NETWORK_PROTOCOL instance.
   @param RxBufferSize  (Optional)Rx buffer size
   @param TxBufferSize  (Optional)Tx buffer size

   @retval EFI_SUCCESS, on success
   @retval error code, on failure
 **/
STATIC
EFI_STATUS
EFIAPI
Dpaa2SnpInitialize (
  IN  EFI_SIMPLE_NETWORK_PROTOCOL*  Snp,
  IN  UINTN  RxBufferSize    OPTIONAL,
  IN  UINTN  TxBufferSize    OPTIONAL
  )
{
  EFI_STATUS Status;
  EFI_SIMPLE_NETWORK_MODE *SnpMode;
  DPAA2_ETHERNET_DEVICE *Dpaa2EthDev;
  WRIOP_DPMAC *WriopDpmac;

  Status = Dpaa2SnpValidateParameters (Snp);
  if (EFI_ERROR (Status)) {
    DPAA_ERROR_MSG ("%a (): Failed to Validate Snp Parameters (error %u)\n", __FUNCTION__, Status);
    return Status;
  }

  Dpaa2EthDev = SNP_TO_DPAA2_DEV (Snp);
  DPAA_DEBUG_MSG ("%a () called for DPAA2 Ethernet device 0x%p (%a) \n", __FUNCTION__,
                  Dpaa2EthDev, gWriopDpmacStrings[Dpaa2EthDev->WriopDpmac->Id]);

  SnpMode = Snp->Mode;
  WriopDpmac = Dpaa2EthDev->WriopDpmac;

  if (SnpMode->State == EfiSimpleNetworkInitialized) {
    DPAA_WARN_MSG ("DPAA2 Ethernet device 0x%p (%a) already initialized\n",
                   Dpaa2EthDev, gWriopDpmacStrings[WriopDpmac->Id]);
    return EFI_SUCCESS;
  }

  if (SnpMode->State == EfiSimpleNetworkStopped) {
    DPAA_ERROR_MSG ("DPAA2 Ethernet device 0x%p (%a) not started\n", Dpaa2EthDev,
                    gWriopDpmacStrings[WriopDpmac->Id]);
    return EFI_NOT_STARTED;
  }

  ASSERT (SnpMode->State == EfiSimpleNetworkStarted);

  /*
   * Start up PHY:
   */
  Status = Dpaa2PhyStartup (&WriopDpmac->Phy);
  if (Status == EFI_TIMEOUT) {
    DPAA_WARN_MSG ("Link not ready for %a after Timeout. Check Ethernet cable\n",
                   gWriopDpmacStrings[WriopDpmac->Id]);
  } else if (EFI_ERROR (Status)) {
    DPAA_ERROR_MSG ("Failed to start PHY for DPAA2 Ethernet device 0x%p (%a) (error %u)\n",
                    Dpaa2EthDev, gWriopDpmacStrings[WriopDpmac->Id], Status);
    return Status;
  }

  Status = Dpaa2McInitializeNetworkInterface (&Dpaa2EthDev->Dpaa2NetInterface,
                                             &SnpMode->CurrentAddress,
                                             &SnpMode->MaxPacketSize,
                                             WriopDpmac);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  SnpMode->State = EfiSimpleNetworkInitialized;
  return EFI_SUCCESS;
}

/**
   SNP protocol Reset () function

   @param Snp           A pointer to the EFI_SIMPLE_NETWORK_PROTOCOL instance.
   @param Verification  Flag indicating that verification is to be done or not

   @retval EFI_SUCCESS, on success
   @retval error code, on failure
 **/
STATIC
EFI_STATUS
EFIAPI
Dpaa2SnpReset (
  IN  EFI_SIMPLE_NETWORK_PROTOCOL* Snp,
  IN  BOOLEAN Verification
  )
{
  EFI_SIMPLE_NETWORK_MODE *SnpMode;
  DPAA2_ETHERNET_DEVICE *Dpaa2EthDev;
  WRIOP_DPMAC *WriopDpmac;
  EFI_STATUS  Status;

  Status = Dpaa2SnpValidateParameters (Snp);
  if (EFI_ERROR (Status)) {
    DPAA_ERROR_MSG ("%a (): Failed to Validate Snp Parameters (error %u)\n", __FUNCTION__, Status);
    return Status;
  }

  Dpaa2EthDev = SNP_TO_DPAA2_DEV (Snp);
  DPAA_DEBUG_MSG ("%a () called for DPAA2 Ethernet device 0x%p (%a) \n", __FUNCTION__,
                  Dpaa2EthDev, gWriopDpmacStrings[Dpaa2EthDev->WriopDpmac->Id]);

  SnpMode = Snp->Mode;
  WriopDpmac = Dpaa2EthDev->WriopDpmac;

  if (SnpMode->State == EfiSimpleNetworkStarted) {
    DPAA_ERROR_MSG ("DPAA2 Ethernet device 0x%p (%a) not initialized\n",
                   Dpaa2EthDev, gWriopDpmacStrings[WriopDpmac->Id]);
    return EFI_DEVICE_ERROR;
  } else if (SnpMode->State == EfiSimpleNetworkStopped) {
    DPAA_ERROR_MSG ("DPAA2 Ethernet device 0x%p (%a) not started\n",
                   Dpaa2EthDev, gWriopDpmacStrings[WriopDpmac->Id]);
    return EFI_NOT_STARTED;
  }

  ASSERT (SnpMode->State == EfiSimpleNetworkInitialized);

  Dpaa2McShutdownNetworkInterface (&Dpaa2EthDev->Dpaa2NetInterface);
  Dpaa2McResetNetworkInterface (&Dpaa2EthDev->Dpaa2NetInterface);
  return Dpaa2McInitializeNetworkInterface (&Dpaa2EthDev->Dpaa2NetInterface,
                                           &SnpMode->CurrentAddress,
                                           &SnpMode->MaxPacketSize,
                                           Dpaa2EthDev->WriopDpmac);
}

/**
   SNP protocol SnpShutdown () function

   @param Snp   A pointer to the EFI_SIMPLE_NETWORK_PROTOCOL instance.

   @retval EFI_SUCCESS, on success
   @retval error code, on failure
 **/
STATIC
EFI_STATUS
EFIAPI
Dpaa2SnpShutdown (
  IN        EFI_SIMPLE_NETWORK_PROTOCOL *Snp
  )
{
  EFI_SIMPLE_NETWORK_MODE *SnpMode;
  DPAA2_ETHERNET_DEVICE *Dpaa2EthDev;
  WRIOP_DPMAC *WriopDpmac;
  EFI_STATUS  Status;

  Status = Dpaa2SnpValidateParameters (Snp);
  if (EFI_ERROR (Status)) {
    DPAA_ERROR_MSG ("%a (): Failed to Validate Snp Parameters (error %u)\n", __FUNCTION__, Status);
    return Status;
  }

  Dpaa2EthDev = SNP_TO_DPAA2_DEV (Snp);
  DPAA_DEBUG_MSG ("%a () called for DPAA2 Ethernet device 0x%p (%a) \n", __FUNCTION__,
                  Dpaa2EthDev, gWriopDpmacStrings[Dpaa2EthDev->WriopDpmac->Id]);

  SnpMode = Snp->Mode;
  WriopDpmac = Dpaa2EthDev->WriopDpmac;

  if (SnpMode->State == EfiSimpleNetworkStarted) {
    DPAA_ERROR_MSG ("DPAA2 Ethernet device 0x%p (%a) not initialized\n",
                   Dpaa2EthDev, gWriopDpmacStrings[WriopDpmac->Id]);
    return EFI_DEVICE_ERROR;
  } else if (SnpMode->State == EfiSimpleNetworkStopped) {
    DPAA_ERROR_MSG ("DPAA2 Ethernet device 0x%p (%a) not started\n",
                   Dpaa2EthDev, gWriopDpmacStrings[WriopDpmac->Id]);
    return EFI_NOT_STARTED;
  }

  ASSERT (SnpMode->State == EfiSimpleNetworkInitialized);

  /*
   * Shutdown and reset DPAA2 network interface in the MC:
   */
  if (Dpaa2EthDev->Dpaa2NetInterface.StartedInMc) {
    Dpaa2McShutdownNetworkInterface (&Dpaa2EthDev->Dpaa2NetInterface);
    Dpaa2McResetNetworkInterface (&Dpaa2EthDev->Dpaa2NetInterface);
    SnpMode->MCastFilterCount = 0;
  }

  Dpaa2PhyShutdown (&WriopDpmac->Phy);
  SnpMode->State = EfiSimpleNetworkStarted;
  return EFI_SUCCESS;
}

/**
  Enable and/or disable receive MAC filters

  @param[in]  Snp               Pointer to the EFI_SIMPLE_NETWORK_PROTOCOL
                                instance.
  @param[in]  Enable            Bit mask of receive filters to enable.
  @param[in]  Disable           Bit mask of receive filters to disable.
  @param[in]  ResetMCastFilter  Flag to request that the multicast receive filters
                                on the network interface be reset to default values.
  @param[in]  MCastFilterCnt    Number of multicast HW MAC addresses in the new
                                MCastFilter list. This arg is meaningful only if
                                ResetMCastFilter is FALSE.
  @param[in]  MCastFilter       A pointer to a list of new multicast receive
                                filter HW MAC addresses. This list will replace
                                any existing multicast HW MAC address list. This
                                arg is meaningful only if ResetMCastFilter is
                                FALSE.

  @retval  EFI_SUCCESS, on success
  @retval  error code, on failure

**/
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
  )
{
  EFI_SIMPLE_NETWORK_MODE *SnpMode;
  DPAA2_ETHERNET_DEVICE *Dpaa2EthDev;
  WRIOP_DPMAC *WriopDpmac;
  EFI_STATUS  Status;
  UINTN I;

  Status = Dpaa2SnpValidateParameters (Snp);
  if (EFI_ERROR (Status)) {
    DPAA_ERROR_MSG ("%a (): Failed to Validate Snp Parameters (error %u)\n", __FUNCTION__, Status);
    return Status;
  }

  Dpaa2EthDev = SNP_TO_DPAA2_DEV (Snp);
  DPAA_DEBUG_MSG ("%a () called for DPAA2 Ethernet device 0x%p (%a) \n", __FUNCTION__,
                  Dpaa2EthDev, gWriopDpmacStrings[Dpaa2EthDev->WriopDpmac->Id]);

  SnpMode = Snp->Mode;
  WriopDpmac = Dpaa2EthDev->WriopDpmac;

  if (SnpMode->State == EfiSimpleNetworkStarted) {
    DPAA_ERROR_MSG ("DPAA2 Ethernet device 0x%p (%a) not initialized\n",
                    Dpaa2EthDev, gWriopDpmacStrings[WriopDpmac->Id]);
    return EFI_DEVICE_ERROR;
  } else if (SnpMode->State == EfiSimpleNetworkStopped) {
    DPAA_ERROR_MSG ("DPAA2 Ethernet device 0x%p (%a) in stopped state\n",
                    Dpaa2EthDev, gWriopDpmacStrings[WriopDpmac->Id]);
    return EFI_NOT_STARTED;
  }

  /*
   * Validate parameters:
   */
  if (Enable & (~SnpMode->ReceiveFilterMask)) {
    DPAA_ERROR_MSG (
      "Dpaa2SnpReceiveFilter 'Enable' mask has invalid bits: 0x%x (valid bits 0x%x)\n",
      Enable, SnpMode->ReceiveFilterMask);

    return EFI_INVALID_PARAMETER;
  }

  if (Disable & (~SnpMode->ReceiveFilterMask)) {
    DPAA_ERROR_MSG (
      "Dpaa2SnpReceiveFilter 'Disable' mask has invalid bits: 0x%x (valid bits 0x%x)\n",
      Enable, SnpMode->ReceiveFilterMask);

    return EFI_INVALID_PARAMETER;
  }

  /*
   * As per the UEFI Specification, If the same bits are set in the Enable and
   * Disable parameters, then the bits in the Disable parameter takes precedence
   */
  if (Enable & Disable)
    Enable = Enable & ~Disable;

  if (!ResetMCastFilter) {
    if (MCastFilterCnt > SnpMode->MaxMCastFilterCount) {
      DPAA_ERROR_MSG (
        "Dpaa2SnpReceiveFilter 'MCastFilterCnt' (%u) is out of range\n",
        MCastFilterCnt);

      return EFI_INVALID_PARAMETER;
    }

    if (MCastFilterCnt != 0) {
      if (MCastFilter == NULL) {
        DPAA_ERROR_MSG ("Dpaa2SnpReceiveFilter 'MCastFilter' cannot be NULL\n");
        return EFI_INVALID_PARAMETER;
      }

      if (!(Enable & EFI_SIMPLE_NETWORK_RECEIVE_MULTICAST)) {
        DPAA_ERROR_MSG (
          "Dpaa2SnpReceiveFilter 'Enable' mask (0x%x) does not have 'Multicast' bit set\n",
          Enable);

        return EFI_INVALID_PARAMETER;
      }
    }
  }

  /*
   * Update Receive MAC filters:
   */
  if (ResetMCastFilter) {
    /*
     * Remove all entries from the hardware MAC address filter table:
     */
    for (I = 0; I < SnpMode->MCastFilterCount; I++) {
      Dpaa2McRemoveMulticastMacAddress (&Dpaa2EthDev->Dpaa2NetInterface,
                                       &SnpMode->MCastFilter[I]);
    }

    SnpMode->MCastFilterCount = 0;
  } else if (MCastFilterCnt != 0) {
    /*
     * Remove all entries from the hardware MAC address filter table:
     */
    for (I = 0; I < SnpMode->MCastFilterCount; I++) {
      Dpaa2McRemoveMulticastMacAddress (&Dpaa2EthDev->Dpaa2NetInterface,
                                       &SnpMode->MCastFilter[I]);
    }

    /*
     * Add new entries to the hardware MAC address filter table:
     */
    for (I = 0; I < MCastFilterCnt; I++) {
      SnpMode->MCastFilter[I] = MCastFilter[I];
      Dpaa2McAddMulticastMacAddress (&Dpaa2EthDev->Dpaa2NetInterface,
                                    &MCastFilter[I]);
    }

    SnpMode->MCastFilterCount = MCastFilterCnt;
  }

  SnpMode->ReceiveFilterSetting |= Enable;
  SnpMode->ReceiveFilterSetting &= ~Disable;

  return EFI_SUCCESS;
}

/**
  Modify or reset the current station address (MAC address)

  @param[in]  Snp               A pointer to the EFI_SIMPLE_NETWORK_PROTOCOL
                                instance.
  @param[in]  Reset             Flag used to reset the current MAC address to the
                                permanent MAC address.
  @param[in]  New               New MAC address to be used for the network interface.

  @retval  EFI_SUCCESS, on success
  @retval  Error code, on failure

**/
STATIC
EFI_STATUS
EFIAPI
Dpaa2SnpStationAddress (
  IN  EFI_SIMPLE_NETWORK_PROTOCOL  *Snp,
  IN  BOOLEAN                      Reset,
  IN  EFI_MAC_ADDRESS              *New
)
{
  EFI_SIMPLE_NETWORK_MODE *SnpMode;
  DPAA2_ETHERNET_DEVICE *Dpaa2EthDev;
  WRIOP_DPMAC *WriopDpmac;
  EFI_STATUS Status;

  Status = Dpaa2SnpValidateParameters (Snp);
  if (EFI_ERROR (Status)) {
    DPAA_ERROR_MSG ("%a (): Failed to Validate Snp Parameters (error %u)\n", __FUNCTION__, Status);
    return Status;
  }

  Dpaa2EthDev = SNP_TO_DPAA2_DEV (Snp);
  DPAA_DEBUG_MSG ("%a () called for DPAA2 Ethernet device 0x%p (%a) \n", __FUNCTION__,
                  Dpaa2EthDev, gWriopDpmacStrings[Dpaa2EthDev->WriopDpmac->Id]);

  SnpMode = Snp->Mode;
  WriopDpmac = Dpaa2EthDev->WriopDpmac;

  if (SnpMode->State == EfiSimpleNetworkStarted) {
    DPAA_ERROR_MSG ("DPAA2 Ethernet device 0x%p (%a) not initialized\n",
                   Dpaa2EthDev, gWriopDpmacStrings[WriopDpmac->Id]);
    return EFI_DEVICE_ERROR;
  } else if (SnpMode->State == EfiSimpleNetworkStopped) {
    DPAA_ERROR_MSG ("DPAA2 Ethernet device 0x%p (%a) in stopped state\n",
                   Dpaa2EthDev, gWriopDpmacStrings[WriopDpmac->Id]);
    return EFI_NOT_STARTED;
  }

  /*
   * TODO:  DPAA2-specific code here
   */
  DPAA_ERROR_MSG ("%a () called for 0x%p not implemented yet\n", __FUNCTION__, Snp);

  return EFI_UNSUPPORTED;
}

/**
   SNP protocol Statistics () function

   @param Snp   A pointer to the EFI_SIMPLE_NETWORK_PROTOCOL instance.

   @retval EFI_SUCCESS, on success
   @retval error code, on failure
 **/
STATIC
EFI_STATUS
EFIAPI
Dpaa2SnpStatistics (
  IN        EFI_SIMPLE_NETWORK_PROTOCOL* Snp,
  IN        BOOLEAN Reset,
  IN  OUT   UINTN *StatSize,
      OUT   EFI_NETWORK_STATISTICS *Statistics
  )
{
  EFI_SIMPLE_NETWORK_MODE *SnpMode;
  DPAA2_ETHERNET_DEVICE *Dpaa2EthDev;
  WRIOP_DPMAC *WriopDpmac;
  EFI_STATUS Status;

  Status = Dpaa2SnpValidateParameters (Snp);
  if (EFI_ERROR (Status)) {
    DPAA_ERROR_MSG ("%a (): Failed to Validate Snp Parameters (error %u)\n", __FUNCTION__, Status);
    return Status;
  }

  Dpaa2EthDev = SNP_TO_DPAA2_DEV (Snp);
  DPAA_DEBUG_MSG ("%a () called for DPAA2 Ethernet device 0x%p (%a) \n", __FUNCTION__,
                  Dpaa2EthDev, gWriopDpmacStrings[Dpaa2EthDev->WriopDpmac->Id]);

  SnpMode = Snp->Mode;
  WriopDpmac = Dpaa2EthDev->WriopDpmac;

  if (SnpMode->State == EfiSimpleNetworkStarted) {
    DPAA_ERROR_MSG ("DPAA2 Ethernet device 0x%p (%a) not initialized\n",
                   Dpaa2EthDev, gWriopDpmacStrings[WriopDpmac->Id]);
    return EFI_DEVICE_ERROR;
  } else if (Snp->Mode->State == EfiSimpleNetworkStopped) {
    DPAA_ERROR_MSG ("DPAA2 Ethernet device 0x%p (%a) in stopped state\n",
                   Dpaa2EthDev, gWriopDpmacStrings[WriopDpmac->Id]);
    return EFI_NOT_STARTED;
  }

  /*
   * TODO:  DPAA2-specific code here
   */
  DPAA_ERROR_MSG ("%a () called for 0x%p not implemented yet\n", __FUNCTION__, Snp);

  return EFI_UNSUPPORTED;
}

/**
   SNP protocol MCastIPtoMAC () function

   @param Snp   A pointer to the EFI_SIMPLE_NETWORK_PROTOCOL instance.

   @retval EFI_SUCCESS, on success
   @retval error code, on failure
 **/
STATIC
EFI_STATUS
EFIAPI
Dpaa2SnpMcastIptoMac (
  IN        EFI_SIMPLE_NETWORK_PROTOCOL* Snp,
  IN        BOOLEAN IsIpv6,
  IN        EFI_IP_ADDRESS *Ip,
      OUT   EFI_MAC_ADDRESS *McastMac
  )
{
  EFI_SIMPLE_NETWORK_MODE *SnpMode;
  DPAA2_ETHERNET_DEVICE *Dpaa2EthDev;
  WRIOP_DPMAC *WriopDpmac;
  EFI_STATUS Status;

  Status = Dpaa2SnpValidateParameters (Snp);
  if (EFI_ERROR (Status)) {
    DPAA_ERROR_MSG ("%a (): Failed to Validate Snp Parameters (error %u)\n", __FUNCTION__, Status);
    return Status;
  }

  Dpaa2EthDev = SNP_TO_DPAA2_DEV (Snp);
  DPAA_DEBUG_MSG ("%a () called for DPAA2 Ethernet device 0x%p (%a) \n", __FUNCTION__,
                  Dpaa2EthDev, gWriopDpmacStrings[Dpaa2EthDev->WriopDpmac->Id]);

  SnpMode = Snp->Mode;
  WriopDpmac = Dpaa2EthDev->WriopDpmac;

  if (SnpMode->State == EfiSimpleNetworkStarted) {
    DPAA_ERROR_MSG ("DPAA2 Ethernet device 0x%p (%a) not initialized\n",
                   Dpaa2EthDev, gWriopDpmacStrings[WriopDpmac->Id]);
    return EFI_DEVICE_ERROR;
  } else if (SnpMode->State == EfiSimpleNetworkStopped) {
    DPAA_ERROR_MSG ("DPAA2 Ethernet device 0x%p (%a) in stopped state\n",
                   Dpaa2EthDev, gWriopDpmacStrings[WriopDpmac->Id]);
    return EFI_NOT_STARTED;
  }

  /*
   * TODO:  DPAA2-specific code here
   */
  DPAA_ERROR_MSG ("%a () called for 0x%p not implemented yet\n", __FUNCTION__, Snp);

  return EFI_UNSUPPORTED;
}

/**
   SNP protocol NvData () function

   @param Snp   A pointer to the EFI_SIMPLE_NETWORK_PROTOCOL instance.

   @retval EFI_SUCCESS, on success
   @retval error code, on failure
 **/
STATIC
EFI_STATUS
EFIAPI
Dpaa2SnpNvData (
  IN        EFI_SIMPLE_NETWORK_PROTOCOL *Snp,
  IN        BOOLEAN read_write,
  IN        UINTN offset,
  IN        UINTN buff_size,
  IN  OUT   VOID *data
  )
{
  DPAA2_ETHERNET_DEVICE *Dpaa2EthDev;

  ASSERT (Snp != NULL);
  Dpaa2EthDev = SNP_TO_DPAA2_DEV (Snp);
  DPAA_DEBUG_MSG ("%a () called for DPAA2 Ethernet device 0x%p\n", __FUNCTION__,
                  Dpaa2EthDev);

  DPAA_WARN_MSG ("SNP NvData not supported for DPAA2 Ethernet devices\n");
  return EFI_UNSUPPORTED;
}

/**
 * Check arrival of Tx completions for Tx frames in flight.
 * If a Tx completion was received from the hardware, free corresponding
 * TxFrameInFlight object
 */
STATIC
VOID
Dpaa2CheckTxCompletion (
  IN DPAA2_ETHERNET_DEVICE *Dpaa2EthDev,
  OUT VOID **TxBuff
  )
{
  UINT64 QBmanTxBufferAddr;
  EFI_STATUS Status;
  LIST_ENTRY *ListNode;
  BOOLEAN TxBufferFound = FALSE;
  DPAA2_TX_FRAME_IN_FLIGHT *TxFrameInFlight = NULL;

  if (IsListEmpty (&Dpaa2EthDev->TxFramesInFlightList)) {
    return;
  }

  Status = Dpaa2McNetworkInterfaceCheckTxCompletion (&Dpaa2EthDev->Dpaa2NetInterface,
                                                    &QBmanTxBufferAddr);
  if (EFI_ERROR (Status)) {
    return;
  }

  EfiAcquireLock (&Dpaa2EthDev->TxFramesInFlightLock);

  /*
   * Find corresponding UEFI networking stack's Tx data buffer:
   */
  for (ListNode = GetFirstNode (&Dpaa2EthDev->TxFramesInFlightList);
       ListNode != &Dpaa2EthDev->TxFramesInFlightList;
       ListNode = GetNextNode (&Dpaa2EthDev->TxFramesInFlightList, ListNode)) {
    TxFrameInFlight = CR (ListNode, DPAA2_TX_FRAME_IN_FLIGHT, ListNode,
                         DPAA2_TX_FRAME_IN_FLIGHT_SIGNATURE);
    if (TxFrameInFlight->QBmanTxBufferAddr == QBmanTxBufferAddr) {
      TxBufferFound = TRUE;
      break;
    }
  }

  if (!TxBufferFound) {
    DPAA_ERROR_MSG ("Frame in flight not found for Ethernet device 0x%p (%a) \n",
                    Dpaa2EthDev, gWriopDpmacStrings[Dpaa2EthDev->WriopDpmac->Id]);
    EfiReleaseLock (&Dpaa2EthDev->TxFramesInFlightLock);
    return;
  }

  ASSERT (TxFrameInFlight != NULL);
  ASSERT (TxFrameInFlight->QBmanTxBufferAddr == QBmanTxBufferAddr);

  if (TxBuff != NULL) {
    *TxBuff = TxFrameInFlight->UefiTxBuffer;
  }

  (VOID)RemoveEntryList (&TxFrameInFlight->ListNode);
  EfiReleaseLock (&Dpaa2EthDev->TxFramesInFlightLock);
  FreePool (TxFrameInFlight);
}

/**
   SNP protocol GetStatus () function

   @param Snp   A pointer to the EFI_SIMPLE_NETWORK_PROTOCOL instance.

   @retval EFI_SUCCESS, on success
   @retval error code, on failure
 **/
STATIC
EFI_STATUS
EFIAPI
Dpaa2SnpGetStatus (
  IN   EFI_SIMPLE_NETWORK_PROTOCOL  *Snp,
  OUT  UINT32                       *IrqStat  OPTIONAL,
  OUT  VOID                         **TxBuff  OPTIONAL
  )
{
  EFI_SIMPLE_NETWORK_MODE *SnpMode;
  DPAA2_ETHERNET_DEVICE *Dpaa2EthDev;
  EFI_STATUS Status;

  Status = Dpaa2SnpValidateParameters (Snp);
  if (EFI_ERROR (Status)) {
    DPAA_ERROR_MSG ("%a (): Failed to Validate Snp Parameters (error %u)\n", __FUNCTION__, Status);
    return Status;
  }

  Dpaa2EthDev = SNP_TO_DPAA2_DEV (Snp);

  SnpMode = Snp->Mode;

  if (SnpMode->State == EfiSimpleNetworkStarted) {
    return EFI_DEVICE_ERROR;
  } else if (SnpMode->State == EfiSimpleNetworkStopped) {
    return EFI_NOT_STARTED;
  }

  ASSERT (SnpMode->State == EfiSimpleNetworkInitialized);
  if (IrqStat != NULL) {
    *IrqStat = 0;
  }

  if (TxBuff != NULL) {
    *TxBuff = NULL;
  }

  if (Dpaa2EthDev->Dpaa2NetInterface.StartedInMc) {
    /*
     * Check physical link status:
     */
    SnpMode->MediaPresent = Dpaa2McCheckDpniLink (&Dpaa2EthDev->Dpaa2NetInterface);

    /*
     * Check arrival of Tx completions for Tx frames in flight:
     */
    Dpaa2CheckTxCompletion (Dpaa2EthDev, TxBuff);
  } else {
    SnpMode->MediaPresent = FALSE;
  }

  return EFI_SUCCESS;
}

/**
   SNP protocol Transmit () function

   @param Snp   A pointer to the EFI_SIMPLE_NETWORK_PROTOCOL instance.

   @retval EFI_SUCCESS, on success
   @retval error code, on failure
 **/
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
  )
{
  EFI_STATUS Status;
  EFI_SIMPLE_NETWORK_MODE *SnpMode;
  DPAA2_ETHERNET_DEVICE *Dpaa2EthDev;
  WRIOP_DPMAC *WriopDpmac;
  DPAA2_TX_FRAME_IN_FLIGHT *TxFrameInFlight;
  UINT64 QBmanTxBufferAddr;

  Status = Dpaa2SnpValidateParameters (Snp);
  if (EFI_ERROR (Status)) {
    DPAA_ERROR_MSG ("%a (): Failed to Validate Snp Parameters (error %u)\n", __FUNCTION__, Status);
    return Status;
  }

  ASSERT (HdrSize <= BuffSize);
  Dpaa2EthDev = SNP_TO_DPAA2_DEV (Snp);

  SnpMode = Snp->Mode;
  WriopDpmac = Dpaa2EthDev->WriopDpmac;

  if (Data == NULL) {
    DPAA_ERROR_MSG ("%a () called with invalid Data parameter\n", __FUNCTION__);
    return EFI_INVALID_PARAMETER;
  }

  if (SnpMode->State == EfiSimpleNetworkStarted) {
    DPAA_ERROR_MSG ("DPAA2 Ethernet device 0x%p (%a) not initialized\n",
                   Dpaa2EthDev, gWriopDpmacStrings[WriopDpmac->Id]);
    return EFI_DEVICE_ERROR;
  } else if (Snp->Mode->State == EfiSimpleNetworkStopped) {
    DPAA_ERROR_MSG ("DPAA2 Ethernet device 0x%p (%a) in stopped state\n",
                   Dpaa2EthDev, gWriopDpmacStrings[WriopDpmac->Id]);
    return EFI_NOT_STARTED;
  }

  if (HdrSize != 0) {
    if (HdrSize != SnpMode->MediaHeaderSize) {
      DPAA_ERROR_MSG ("%a () called with invalid HdrSize parameter: %lu\n",
                      __FUNCTION__, HdrSize);
      return EFI_INVALID_PARAMETER;
    }

    if (DstAddr == NULL) {
      DPAA_ERROR_MSG ("%a () called with invalid DstAddr parameter\n", __FUNCTION__);
      return EFI_INVALID_PARAMETER;
    }

    if (Protocol == NULL) {
      DPAA_ERROR_MSG ("%a () called with invalid Protocol parameter\n", __FUNCTION__);
      return EFI_INVALID_PARAMETER;
    }
  }

  if (BuffSize < SnpMode->MediaHeaderSize ||
      BuffSize > DPAA2_ETH_RX_FRAME_BUFFER_SIZE) {
      DPAA_ERROR_MSG ("%a () called with invalid BuffSize parameter: %lu\n",
                      __FUNCTION__, BuffSize);
      return EFI_BUFFER_TOO_SMALL;
  }

  TxFrameInFlight = AllocateZeroPool (sizeof (*TxFrameInFlight));
  if (TxFrameInFlight == NULL) {
    DPAA_ERROR_MSG ("Could not allocate DPAA2 TxFrameInFlight object\n");
    return EFI_OUT_OF_RESOURCES;
  }

  Status = Dpaa2McNetworkInterfaceTransmit (&Dpaa2EthDev->Dpaa2NetInterface,
                                           Dpaa2EthDev->WriopDpmac,
                                           HdrSize,
                                           BuffSize,
                                           Data,
                                           SrcAddr,
                                           DstAddr,
                                           Protocol,
                                           &QBmanTxBufferAddr);
  if (EFI_ERROR (Status)) {
    goto ErrorExit;
  }

  TxFrameInFlight->Signature = DPAA2_TX_FRAME_IN_FLIGHT_SIGNATURE;
  TxFrameInFlight->UefiTxBuffer = Data;
  TxFrameInFlight->QBmanTxBufferAddr = QBmanTxBufferAddr;
  InitializeListHead (&TxFrameInFlight->ListNode);
  EfiAcquireLock (&Dpaa2EthDev->TxFramesInFlightLock);
  InsertTailList (&Dpaa2EthDev->TxFramesInFlightList, &TxFrameInFlight->ListNode);
  EfiReleaseLock (&Dpaa2EthDev->TxFramesInFlightLock);

  return EFI_SUCCESS;

ErrorExit:
  FreePool (TxFrameInFlight);
  return Status;
}

/**
   SNP protocol Receive () function

   @param Snp   A pointer to the EFI_SIMPLE_NETWORK_PROTOCOL instance.

   @retval EFI_SUCCESS, on success
   @retval error code, on failure
 **/
STATIC
EFI_STATUS
EFIAPI
Dpaa2SnpReceive (
  IN        EFI_SIMPLE_NETWORK_PROTOCOL *Snp,
      OUT   UINTN *HdrSize                OPTIONAL,
  IN  OUT   UINTN *BuffSize,
      OUT   VOID *Data,
      OUT   EFI_MAC_ADDRESS *SrcAddr      OPTIONAL,
      OUT   EFI_MAC_ADDRESS *DstAddr      OPTIONAL,
      OUT   UINT16 *Protocol              OPTIONAL
  )
{
  EFI_STATUS Status;
  EFI_SIMPLE_NETWORK_MODE *SnpMode;
  DPAA2_ETHERNET_DEVICE *Dpaa2EthDev;
  WRIOP_DPMAC *WriopDpmac;

  Status = Dpaa2SnpValidateParameters (Snp);
  if (EFI_ERROR (Status)) {
    DPAA_ERROR_MSG ("%a (): Failed to Validate Snp Parameters (error %u)\n", __FUNCTION__, Status);
    return Status;
  }

  Dpaa2EthDev = SNP_TO_DPAA2_DEV (Snp);

  SnpMode = Snp->Mode;
  WriopDpmac = Dpaa2EthDev->WriopDpmac;

  if (Data == NULL) {
    DPAA_ERROR_MSG ("%a () called with invalid Data parameter\n", __FUNCTION__);
    return EFI_INVALID_PARAMETER;
  }

  if (BuffSize == NULL) {
    DPAA_ERROR_MSG ("%a () called with Invalid BuffSize parameter\n", __FUNCTION__);
    return EFI_INVALID_PARAMETER;
  }

  if (SnpMode->State == EfiSimpleNetworkStarted) {
    DPAA_ERROR_MSG ("DPAA2 Ethernet device 0x%p (%a) not initialized\n",
                   Dpaa2EthDev, gWriopDpmacStrings[WriopDpmac->Id]);
    return EFI_DEVICE_ERROR;
  } else if (Snp->Mode->State == EfiSimpleNetworkStopped) {
    DPAA_ERROR_MSG ("DPAA2 Ethernet device 0x%p (%a) in stopped state\n",
                   Dpaa2EthDev, gWriopDpmacStrings[WriopDpmac->Id]);
    return EFI_NOT_STARTED;
  }

   if (*BuffSize < SnpMode->MediaHeaderSize) {
      DPAA_ERROR_MSG ("%a () called with invalid *BuffSize parameter: %lu\n",
                      __FUNCTION__, *BuffSize);
      return EFI_BUFFER_TOO_SMALL;
  }

  if (HdrSize != NULL) {
    ASSERT (SnpMode->MediaHeaderSize == sizeof (ETHER_HEAD));
    *HdrSize = SnpMode->MediaHeaderSize;
  }

  Status = Dpaa2McNetworkInterfaceReceive (&Dpaa2EthDev->Dpaa2NetInterface,
                                          Dpaa2EthDev->WriopDpmac,
                                          BuffSize,
                                          Data,
                                          SrcAddr,
                                          DstAddr,
                                          Protocol);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  return EFI_SUCCESS;
 }

STATIC
EFI_STATUS
EFIAPI
CreateDpaa2EthernetDevice (
  IN WRIOP_DPMAC *Dpmac,
  OUT DPAA2_ETHERNET_DEVICE **Dpaa2EthDevPtr
  )
{
  EFI_STATUS Status;
  DPAA2_ETHERNET_DEVICE *Dpaa2EthDev;

  DPAA_INFO_MSG ("Creating DPAA2 Ethernet device object for %a ...\n",
                 gWriopDpmacStrings[Dpmac->Id]);

  Dpaa2EthDev = AllocateCopyPool (sizeof (*Dpaa2EthDev),
                                 &gDpaa2EthernetDeviceInitTemplate);
  if (Dpaa2EthDev == NULL) {
    DPAA_ERROR_MSG ("Could not allocate DPAA2 Ethernet device\n");
    return EFI_OUT_OF_RESOURCES;
  }

  ASSERT (Dpaa2EthDev->Signature == DPAA2_ETHERNET_DEVICE_SIGNATURE);

  Dpaa2EthDev->Snp.Mode = &Dpaa2EthDev->SnpMode;
  Dpaa2EthDev->WriopDpmac = Dpmac;
  InitializeListHead (&Dpaa2EthDev->ListNode);
  InitializeListHead (&Dpaa2EthDev->TxFramesInFlightList);

  /*
   * Set MAC address for the DPAA2 Ethernet device:
   */
  Status = MacReadFromEeprom (Dpmac->Id, &Dpaa2EthDev->SnpMode.CurrentAddress.Addr);
  if (EFI_ERROR (Status)) {
    Status = GenerateMacAddress (Dpmac->Id, &Dpaa2EthDev->SnpMode.CurrentAddress.Addr);
  }
  if (EFI_ERROR (Status)) {
    DPAA_ERROR_MSG ("Error getting mac address for dpmac id %d status %r\n", Dpmac->Id, Status);
    return EFI_DEVICE_ERROR;
  } else {
    DPAA_DEBUG_MSG ("MAC addr: %02X:%02X:%02X:%02X:%02X:%02X\n",
                    MacAddrBuf->Addr[0],
                    MacAddrBuf->Addr[1],
                    MacAddrBuf->Addr[2],
                    MacAddrBuf->Addr[3],
                    MacAddrBuf->Addr[4],
                    MacAddrBuf->Addr[5]);
  }

  /*
   * Copy MAC address to the UEFI device path for the DPAA2 Ethernet device:
   */
  CopyMem (&Dpaa2EthDev->DevicePath.MacAddrDevicePath.MacAddress,
          &Dpaa2EthDev->SnpMode.CurrentAddress,
          NET_ETHER_ADDR_LEN);

  /*
   * Install driver model protocols:
   */

  DPAA_DEBUG_MSG ("Installing UEFI protocol interfaces for DPAA2 Ethernet device 0x%p (%a)...\n",
                  Dpaa2EthDev,gWriopDpmacStrings[Dpmac->Id]);

  ASSERT (Dpaa2EthDev->ControllerHandle == NULL);
  Status = gBS->InstallMultipleProtocolInterfaces (
                  &Dpaa2EthDev->ControllerHandle,
                  &gEfiSimpleNetworkProtocolGuid, &Dpaa2EthDev->Snp,
                  &gEfiDevicePathProtocolGuid, &Dpaa2EthDev->DevicePath,
                  NULL
                  );
  if (EFI_ERROR (Status)) {
    DPAA_ERROR_MSG ("Failed to install UEFI protocols for DPAA2 Ethernet device 0x%p (%a) (error %u)\n",
                    Dpaa2EthDev, gWriopDpmacStrings[Dpmac->Id], Status);
    goto ErrorExitFreeDpaa2EthDevice;
  }

  ASSERT (!Dpaa2EthDev->Dpaa2NetInterface.CreatedInMc);

  *Dpaa2EthDevPtr = Dpaa2EthDev;
  return EFI_SUCCESS;

ErrorExitFreeDpaa2EthDevice:
  FreePool (Dpaa2EthDev);
  return Status;
}


STATIC
VOID
EFIAPI
DestroyDpaa2EthernetDevice (
  IN DPAA2_ETHERNET_DEVICE *Dpaa2EthDev
  )
{
  EFI_STATUS Status;

  ASSERT (Dpaa2EthDev->Signature == DPAA2_ETHERNET_DEVICE_SIGNATURE);
  DPAA_DEBUG_MSG ("Destroying DPAA2 Ethernet device object 0x%p (%a) ...\n",
                  Dpaa2EthDev, gWriopDpmacStrings[Dpaa2EthDev->WriopDpmac->Id]);

  if (Dpaa2EthDev->ControllerHandle != NULL) {
    Status = gBS->UninstallMultipleProtocolInterfaces (
                   Dpaa2EthDev->ControllerHandle,
                   &gEfiSimpleNetworkProtocolGuid, &Dpaa2EthDev->Snp,
                   &gEfiDevicePathProtocolGuid, &Dpaa2EthDev->DevicePath,
                   NULL
                   );

    if (EFI_ERROR (Status)) {
      DPAA_ERROR_MSG ("Failed to uninstall UEFI driver model protocols for DPAA2 device 0x%p (%a) (error %u)\n",
                      Dpaa2EthDev, gWriopDpmacStrings[Dpaa2EthDev->WriopDpmac->Id], Status);
    }
  }

  ASSERT (!Dpaa2EthDev->Dpaa2NetInterface.CreatedInMc);
  ASSERT (IsListEmpty (&Dpaa2EthDev->TxFramesInFlightList));
  FreePool (Dpaa2EthDev);
}

STATIC
VOID
DestroyAllDpaa2NetworkInterfacesInMc (VOID)
{
  LIST_ENTRY *ListNode;
  DPAA2_ETHERNET_DEVICE *Dpaa2EthDev;
  WRIOP_DPMAC *WriopDpmac;

  for (ListNode = GetFirstNode (&gDpaa2Driver.Dpaa2EthernetDevicesList);
       ListNode != &gDpaa2Driver.Dpaa2EthernetDevicesList;
       ListNode = GetNextNode (&gDpaa2Driver.Dpaa2EthernetDevicesList,
                              ListNode)) {
    Dpaa2EthDev = CR (ListNode, DPAA2_ETHERNET_DEVICE, ListNode,
                     DPAA2_ETHERNET_DEVICE_SIGNATURE);

    WriopDpmac = Dpaa2EthDev->WriopDpmac;
    ASSERT (WriopDpmac != NULL);
    if (Dpaa2EthDev->Dpaa2NetInterface.StartedInMc) {
      /*
       * Shutdown DPAA2 network interface in the MC:
       */
      Dpaa2McShutdownNetworkInterface (&Dpaa2EthDev->Dpaa2NetInterface);
    }

    if (Dpaa2EthDev->Dpaa2NetInterface.CreatedInMc) {
      /*
       * Destroy DPAA2 network interface in the MC:
       */
      DPAA_INFO_MSG ("Destroying DPAA2 Ethernet physical device for %a ...\n",
                     gWriopDpmacStrings[WriopDpmac->Id]);

      Dpaa2McDestroyNetworkInterface (&Dpaa2EthDev->Dpaa2NetInterface);
    }
  }
}

/**
   Exit boot services callback

   @param Event         UEFI event
   @param Context       calback argument

 */
STATIC
VOID
EFIAPI
Dpaa2NotifyExitBootServices (
  EFI_EVENT Event,
  VOID      *Context
  )
{
  ASSERT (Event == gDpaa2Driver.ExitBootServicesEvent);
  ASSERT (gDpaa2Driver.ExitBootServicesEvent != NULL);

  DPAA_DEBUG_MSG ("%a () called (Event: 0x%x, Context: 0x%p)\n",
                  __FUNCTION__, Event, Context);

  if (gDpaa2Driver.McStatus == EFI_SUCCESS) {
    DestroyAllDpaa2NetworkInterfacesInMc ();

    /*
     * Cleanup MC state before booting OS:
     */
    Dpaa2McExit ();

    /*
     * Deploy DPL:
     */
    (VOID)Dpaa2McDeployDpl ();
  }
}

/**
   DPAA2 Ethernet Driver unload entry point

   @param ImageHandle   Driver image handle

   @retval EFI_SUCCESS, on success
   @retval error code, on failure
 */

EFI_STATUS
EFIAPI
Dpaa2EthernetUnload (
  IN EFI_HANDLE ImageHandle
  )
{
  EFI_STATUS Status;
  LIST_ENTRY *ListNode;
  LIST_ENTRY *NextNode;
  DPAA2_ETHERNET_DEVICE *Dpaa2EthDev;

  /*
   * Destroy all DPAA2 Ethernet devices:
   */
  for (ListNode = GetFirstNode (&gDpaa2Driver.Dpaa2EthernetDevicesList);
       ListNode != &gDpaa2Driver.Dpaa2EthernetDevicesList;
       ListNode = NextNode) {
    Dpaa2EthDev = CR (ListNode, DPAA2_ETHERNET_DEVICE, ListNode,
                     DPAA2_ETHERNET_DEVICE_SIGNATURE);

    NextNode = RemoveEntryList (&Dpaa2EthDev->ListNode);
    DestroyDpaa2EthernetDevice (Dpaa2EthDev);
  }

  ASSERT (IsListEmpty (&gDpaa2Driver.Dpaa2EthernetDevicesList));
  InitializeListHead (&gDpaa2Driver.DpmacsList);

  Status = gBS->CloseEvent (gDpaa2Driver.ExitBootServicesEvent);
  if (EFI_ERROR (Status)) {
    DPAA_ERROR_MSG ("Failed to close UEFI event x0x%p (error %u)\n",
                    gDpaa2Driver.ExitBootServicesEvent, Status);
    return Status;
  }

  return EFI_SUCCESS;
}

/**
   DPAA2 Ethernet Driver initialization entry point

   @param ImageHandle   Driver image handle
   @param SystemTable   Pointer to the UEFI system table

   @retval EFI_SUCCESS, on success
   @retval error code, on failure
 */
EFI_STATUS
EFIAPI
Dpaa2EthernetInitialize (
  IN EFI_HANDLE         ImageHandle,
  IN EFI_SYSTEM_TABLE   *SystemTable
  )
{
  EFI_STATUS Status;
  LIST_ENTRY *ListNode;
  LIST_ENTRY *NextNode;
  DPAA2_ETHERNET_DEVICE *Dpaa2EthDev;
  BOOLEAN Dpaa2Enabled;
  UINT64 Dpaa2UsedDpmacsMask;

  Dpaa2EthDev = NULL;
  Dpaa2Enabled = FixedPcdGetBool (PcdDpaa2Initialize);
  Dpaa2UsedDpmacsMask = FixedPcdGet64(PcdDpaa2UsedDpmacsMask);

  gDpaaDebugFlags = FixedPcdGet32(PcdDpaaDebugFlags);
  DPAA_DEBUG_MSG ("%a () %a %a\n", __FUNCTION__, __DATE__, __TIME__);

  if (!Dpaa2Enabled) {
    DPAA_DEBUG_MSG ("DPAA2 disabled\n");
    return EFI_SUCCESS;
  }

  Status = Dpaa2PhyMdioBusesInit (gDpaa2MdioBuses, FixedPcdGet8(PcdMdioBustCount));
  if (EFI_ERROR (Status)) {
    DPAA_ERROR_MSG ("%a (): Mdio bus Initialization Failed (error %u)\n", __FUNCTION__, Status);
    return Status;
  }

  /*
   * Discover enabled DPMACs in the RCW:
   */
  ASSERT (IsListEmpty (&gDpaa2Driver.DpmacsList));
  //SerDesProbeLanes (Dpaa2DiscoverWriopDpmac, &gDpaa2Driver.DpmacsList);
  ProbeDpaaLanes (&gDpaa2Driver.DpmacsList);
  ASSERT (!IsListEmpty (&gDpaa2Driver.DpmacsList));

  /*
   * Load and boot DPAA2 MC firmware:
   */
  gDpaa2Driver.McStatus = Dpaa2McInit ();
  if (EFI_ERROR (gDpaa2Driver.McStatus)) {
    goto ErroExitCleanupDpmacsList;
  }

  /*
   * Traverse list of discovered DPMACs and create corresponding DPAA2
   * Ethernet devices, for the DPMACs actually enabled by the user:
   */
  ASSERT (IsListEmpty (&gDpaa2Driver.Dpaa2EthernetDevicesList));
  for (ListNode = GetFirstNode (&gDpaa2Driver.DpmacsList);
       ListNode != &gDpaa2Driver.DpmacsList;
       ListNode = GetNextNode (&gDpaa2Driver.DpmacsList, ListNode)) {
    WRIOP_DPMAC *Dpmac = CR (ListNode, WRIOP_DPMAC, ListNode,
                            WRIOP_DPMAC_SIGNATURE);

    if ((BIT (Dpmac->Id - 1) & Dpaa2UsedDpmacsMask) == 0) {
      continue;
    }

    Status = CreateDpaa2EthernetDevice (Dpmac, &Dpaa2EthDev);
    if (EFI_ERROR (Status)) {
      continue;
    }

    InsertTailList (&gDpaa2Driver.Dpaa2EthernetDevicesList, &Dpaa2EthDev->ListNode);
  }

  if (IsListEmpty (&gDpaa2Driver.Dpaa2EthernetDevicesList)) {
    DPAA_ERROR_MSG ("No usable DPAA2 devices\n");
    Status = EFI_DEVICE_ERROR;
    goto ErroExitCleanupDpmacsList;
  }

  Status = gBS->CreateEvent (
              EVT_SIGNAL_EXIT_BOOT_SERVICES,
              TPL_CALLBACK,
              Dpaa2NotifyExitBootServices,
              NULL,
              &gDpaa2Driver.ExitBootServicesEvent
              );
  if (EFI_ERROR (Status)) {
    DPAA_ERROR_MSG ("Failed to create UEFI event (error %u)\n", Status);
    goto ErrorExitCleanupDpaa2EthDevices;
  }

  return EFI_SUCCESS;

ErrorExitCleanupDpaa2EthDevices:
  for (ListNode = GetFirstNode (&gDpaa2Driver.Dpaa2EthernetDevicesList);
       ListNode != &gDpaa2Driver.Dpaa2EthernetDevicesList;
       ListNode = NextNode) {
    Dpaa2EthDev = CR (ListNode, DPAA2_ETHERNET_DEVICE, ListNode,
                      DPAA2_ETHERNET_DEVICE_SIGNATURE);

    NextNode = RemoveEntryList (&Dpaa2EthDev->ListNode);
    DestroyDpaa2EthernetDevice (Dpaa2EthDev);
  }

  ASSERT (IsListEmpty (&gDpaa2Driver.Dpaa2EthernetDevicesList));

ErroExitCleanupDpmacsList:
  InitializeListHead (&gDpaa2Driver.DpmacsList);
  return Status;
}
