/** @file  Dpaa1EthernetDxe.c

  DPAA1 Ethernet DXE driver

  Copyright (c) 2016, Freescale Semiconductor, Inc. All rights reserved.

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution. The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include <Bitops.h>
#include <Library/BaseLib.h>
#include <Library/BeIoLib.h>
#include <Library/DevicePathLib.h>
#include <Library/Dpaa1DebugLib.h>
#include <Library/Dpaa1Lib.h>
#include <Library/Dpaa1EthernetPhyLib.h>
#include <Library/IoLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/NetLib.h>
#include <Library/PcdLib.h>
//#include <Library/SysEepromLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Protocol/ComponentName2.h>
#include <Protocol/DevicePath.h>
#include <Protocol/PxeBaseCode.h>

#include "Dpaa1EthernetDxe.h"

/**
 * Debug mode flags for DPAA1 code
 */
UINT32 gDpaa1DebugFlags = 0x0;
extern DPAA1_PHY_MDIO_BUS gDpaa1MdioBuses[];
/**
 * DPAA1 Ethernet driver global control block
 */
typedef struct _DPAA1_ETHERNET_DRIVER {
  /**
   * Head of the linked list of enabled MEMACs
   */
  LIST_ENTRY MemacsList;

  /**
   * Head of the linked list of DPAA1 Ethernet devices
   */
  LIST_ENTRY Dpaa1EthernetDevicesList;

  /**
   * Exit boot services event
   */
  EFI_EVENT ExitBootServicesEvent;

  /**
   * FMAN final status to be communicated to the OS
   */
  EFI_STATUS FmanStatus;

} DPAA1_ETHERNET_DRIVER;

STATIC DPAA1_ETHERNET_DRIVER gDpaa1Driver = {
  .MemacsList = INITIALIZE_LIST_HEAD_VARIABLE(gDpaa1Driver.MemacsList),
  .Dpaa1EthernetDevicesList = INITIALIZE_LIST_HEAD_VARIABLE(gDpaa1Driver.Dpaa1EthernetDevicesList),
  .ExitBootServicesEvent = NULL
};

STATIC
EFI_STATUS
EFIAPI
Dpaa1SnpValidateParameters (
  IN  EFI_SIMPLE_NETWORK_PROTOCOL  *Snp
 )
{
  EFI_SIMPLE_NETWORK_MODE *SnpMode;
  DPAA1_ETHERNET_DEVICE *Dpaa1EthDev;
  FMAN_MEMAC *FmanMemac;

  SnpMode = NULL;

  if (Snp) {
    SnpMode = Snp->Mode;
  }

  if (!Snp || !SnpMode) {
    ASSERT(Snp != NULL);
    ASSERT(SnpMode != NULL);
    return EFI_INVALID_PARAMETER;
  }

  Dpaa1EthDev = SNP_TO_DPAA1_DEV(Snp);
  FmanMemac = Dpaa1EthDev->FmanMemac;
  if (!FmanMemac) {
    ASSERT(FmanMemac != NULL);
    return EFI_INVALID_PARAMETER;
  }
  return EFI_SUCCESS;
}

/**
   SNP protocol Start() function

   @param Snp   A pointer to the EFI_SIMPLE_NETWORK_PROTOCOL instance.

   @retval EFI_SUCCESS, on success
   @retval error code, on failure
 **/
STATIC
EFI_STATUS
EFIAPI
Dpaa1SnpStart (
  IN  EFI_SIMPLE_NETWORK_PROTOCOL  *Snp
 )
{
  EFI_STATUS Status;
  EFI_SIMPLE_NETWORK_MODE *SnpMode;
  DPAA1_ETHERNET_DEVICE *Dpaa1EthDev;
  FMAN_MEMAC *FmanMemac;

  Status = Dpaa1SnpValidateParameters(Snp);
  if (EFI_ERROR(Status)) {
      DPAA1_ERROR_MSG("Failed to Validate Snp Parameters (error %u)\n", Status);
      return Status;
  }

  Dpaa1EthDev = SNP_TO_DPAA1_DEV(Snp);
  DPAA1_DEBUG_MSG("%a() called for DPAA1 Ethernet device 0x%p\n", __func__,
                  Dpaa1EthDev);

  SnpMode = Snp->Mode;
  FmanMemac = Dpaa1EthDev->FmanMemac;

  if (SnpMode->State == EfiSimpleNetworkStarted ||
      SnpMode->State == EfiSimpleNetworkInitialized) {
    DPAA1_ERROR_MSG("DPAA1 Ethernet device 0x%p (%a) already started\n",
                   Dpaa1EthDev, gFmanMemacStrings[FmanMemac->Id]);
    return EFI_ALREADY_STARTED;
  }

  ASSERT(SnpMode->State == EfiSimpleNetworkStopped);

  if (!Dpaa1EthDev->PhyInitialized) {
    /*
     * Initialize PHY:
     */
    Status = Dpaa1PhyInit(&FmanMemac->Phy);
    if (EFI_ERROR(Status)) {
      DPAA1_ERROR_MSG("Failed to initialize PHY for DPAA1 Ethernet device 0x%p (error %u)\n",
                      Dpaa1EthDev, Status);
      return Status;
    }

    Dpaa1EthDev->PhyInitialized = TRUE;
  }

  /*
   * Create DPAA1 network interface:
   */
  DPAA1_INFO_MSG("MAC address %a:%02x:%02x:%02x:%02x:%02x:%02x ...\n",
                 gFmanMemacStrings[FmanMemac->Id],
                 SnpMode->CurrentAddress.Addr[0],
                 SnpMode->CurrentAddress.Addr[1],
                 SnpMode->CurrentAddress.Addr[2],
                 SnpMode->CurrentAddress.Addr[3],
                 SnpMode->CurrentAddress.Addr[4],
                 SnpMode->CurrentAddress.Addr[5]);


  if (SnpMode->CurrentAddress.Addr[0] & 0x01) {
         DPAA1_ERROR_MSG("%s: MacAddress is multcast address\n", __func__);
         return EFI_INVALID_PARAMETER;
  }

  Dpaa1StopNetworkInterface(Dpaa1EthDev);

  SetMacAddr(Dpaa1EthDev->FmanEthDevice->Mac, SnpMode->CurrentAddress);

  Dpaa1StartNetworkInterface(Dpaa1EthDev);

  SnpMode->State = EfiSimpleNetworkStarted;

  return EFI_SUCCESS;
}


/**
   SNP protocol Stop() function

   @param Snp   A pointer to the EFI_SIMPLE_NETWORK_PROTOCOL instance.

   @retval EFI_SUCCESS, on success
   @retval error code, on failure
 **/
STATIC
EFI_STATUS
EFIAPI
Dpaa1SnpStop (
  IN        EFI_SIMPLE_NETWORK_PROTOCOL* Snp
  )
{
  EFI_STATUS Status;
  EFI_SIMPLE_NETWORK_MODE *SnpMode;
  DPAA1_ETHERNET_DEVICE *Dpaa1EthDev;
  FMAN_MEMAC *FmanMemac;

  Status = Dpaa1SnpValidateParameters(Snp);
  if (EFI_ERROR(Status)) {
      DPAA1_ERROR_MSG("Failed to Validate Snp Parameters (error %u)\n", Status);
      return Status;
  }

  Dpaa1EthDev = SNP_TO_DPAA1_DEV(Snp);
  DPAA1_DEBUG_MSG("%a() called for DPAA1 Ethernet device 0x%p\n", __func__,
                  Dpaa1EthDev);

  SnpMode = Snp->Mode;
  FmanMemac = Dpaa1EthDev->FmanMemac;

  if (SnpMode->State == EfiSimpleNetworkStopped) {
    DPAA1_ERROR_MSG("DPAA1 Ethernet device 0x%p (%a) not started\n",
                   Dpaa1EthDev, gFmanMemacStrings[FmanMemac->Id]);
    return EFI_NOT_STARTED;
  }

  if (SnpMode->State == EfiSimpleNetworkInitialized) {
    Dpaa1PhyShutdown(&FmanMemac->Phy);
  } else {
    ASSERT(SnpMode->State == EfiSimpleNetworkStarted);
    return EFI_ALREADY_STARTED;
  }

  /*
   * Destroy DPAA1 network interface:
   */
  DPAA1_INFO_MSG("Stopping DPAA1 Ethernet physical device for %a ...\n",
                 gFmanMemacStrings[FmanMemac->Id]);
  Dpaa1StopNetworkInterface(Dpaa1EthDev);

  //TODO check if needed to call here.
  Dpaa1PhyShutdown(&Dpaa1EthDev->FmanMemac->Phy);

  SnpMode->State = EfiSimpleNetworkStopped;
  return EFI_SUCCESS;
}


/**
   SNP protocol Initialize() function

   @param Snp           A pointer to the EFI_SIMPLE_NETWORK_PROTOCOL instance.
   @param RxBufferSize  (Optional)Rx buffer size
   @param TxBufferSize  (Optional)Tx buffer size

   @retval EFI_SUCCESS, on success
   @retval error code, on failure
 **/
STATIC
EFI_STATUS
EFIAPI
Dpaa1SnpInitialize (
  IN        EFI_SIMPLE_NETWORK_PROTOCOL* Snp,
  IN        UINTN                        RxBufferSize    OPTIONAL,
  IN        UINTN                        TxBufferSize    OPTIONAL
  )
{
  EFI_STATUS Status;
  EFI_SIMPLE_NETWORK_MODE *SnpMode;
  DPAA1_ETHERNET_DEVICE *Dpaa1EthDev;
  FMAN_MEMAC *FmanMemac;

  Status = Dpaa1SnpValidateParameters(Snp);
  if (EFI_ERROR(Status)) {
      DPAA1_ERROR_MSG("Failed to Validate Snp Parameters (error %u)\n", Status);
      return Status;
  }

  Dpaa1EthDev = SNP_TO_DPAA1_DEV(Snp);
  DPAA1_DEBUG_MSG("%a() called for DPAA1 Ethernet device 0x%p\n", __func__,
                  Dpaa1EthDev);

  SnpMode = Snp->Mode;
  FmanMemac = Dpaa1EthDev->FmanMemac;

  if (SnpMode->State == EfiSimpleNetworkInitialized) {
    DPAA1_WARN_MSG("DPAA1 Ethernet device 0x%p (%a) already initialized\n",
                   Dpaa1EthDev, gFmanMemacStrings[FmanMemac->Id]);
    return EFI_SUCCESS;
  }

  if (SnpMode->State == EfiSimpleNetworkStopped) {
    DPAA1_ERROR_MSG("DPAA1 Ethernet device 0x%p (%a) not started\n", Dpaa1EthDev,
                    gFmanMemacStrings[FmanMemac->Id]);
    return EFI_NOT_STARTED;
  }

  ASSERT(SnpMode->State == EfiSimpleNetworkStarted);

  /*
   * Start up PHY:
   */
  Status = Dpaa1PhyStartup(&FmanMemac->Phy);
  if (Status == EFI_TIMEOUT) {
    DPAA1_WARN_MSG("Link not ready for %a after Timeout. Check Ethernet cable\n",
                   gFmanMemacStrings[FmanMemac->Id]);
  } else if (EFI_ERROR(Status)) {
    DPAA1_ERROR_MSG("Failed to start PHY for DPAA1 Ethernet device 0x%p (%a) (error %u)\n",
                    Dpaa1EthDev, gFmanMemacStrings[FmanMemac->Id], Status);
    return Status;
  }

  /* set the MAC-PHY mode */
  SetInterface(Dpaa1EthDev->FmanEthDevice->Mac,
		Dpaa1EthDev->FmanMemac->Phy.PhyInterfaceType,
		FmanMemac->Phy.Speed);

  SnpMode->State = EfiSimpleNetworkInitialized;

  return EFI_SUCCESS;
}

/**
   SNP protocol Reset() function

   @param Snp           A pointer to the EFI_SIMPLE_NETWORK_PROTOCOL instance.
   @param Verification  Flag indicating that verification is to be done or not

   @retval EFI_SUCCESS, on success
   @retval error code, on failure
 **/
STATIC
EFI_STATUS
EFIAPI
Dpaa1SnpReset (
  IN        EFI_SIMPLE_NETWORK_PROTOCOL* Snp,
  IN        BOOLEAN Verification
  )
{
  EFI_SIMPLE_NETWORK_MODE *SnpMode;
  DPAA1_ETHERNET_DEVICE *Dpaa1EthDev;
  FMAN_MEMAC *FmanMemac;
  EFI_STATUS Status;

  Status = Dpaa1SnpValidateParameters(Snp);
  if (EFI_ERROR(Status)) {
      DPAA1_ERROR_MSG("Failed to Validate Snp Parameters (error %u)\n", Status);
      return Status;
  }

  Dpaa1EthDev = SNP_TO_DPAA1_DEV(Snp);
  DPAA1_DEBUG_MSG("%a() called for DPAA1 Ethernet device 0x%p\n", __func__,
                  Dpaa1EthDev);

  SnpMode = Snp->Mode;
  FmanMemac = Dpaa1EthDev->FmanMemac;

  if (SnpMode->State == EfiSimpleNetworkStarted) {
    DPAA1_ERROR_MSG("DPAA1 Ethernet device 0x%p (%a) not initialized\n",
                   Dpaa1EthDev, gFmanMemacStrings[FmanMemac->Id]);
    return EFI_DEVICE_ERROR;
  } else if (SnpMode->State == EfiSimpleNetworkStopped) {
    DPAA1_ERROR_MSG("DPAA1 Ethernet device 0x%p (%a) not started\n",
                   Dpaa1EthDev, gFmanMemacStrings[FmanMemac->Id]);
    return EFI_NOT_STARTED;
  }

  ASSERT(SnpMode->State == EfiSimpleNetworkInitialized);

  Dpaa1StopNetworkInterface(Dpaa1EthDev);
  Dpaa1PhyShutdown(&Dpaa1EthDev->FmanMemac->Phy);

  Dpaa1StartNetworkInterface(Dpaa1EthDev);

  Status = Dpaa1PhyInit(&FmanMemac->Phy);
    if (EFI_ERROR(Status)) {
      DPAA1_ERROR_MSG("Failed to initialize PHY for DPAA1 Ethernet device 0x%p (error %u)\n",
                      Dpaa1EthDev, Status);
    }

  return Status;
}


/**
   SNP protocol SnpShutdown() function

   @param Snp   A pointer to the EFI_SIMPLE_NETWORK_PROTOCOL instance.

   @retval EFI_SUCCESS, on success
   @retval error code, on failure
 **/
STATIC
EFI_STATUS
EFIAPI
Dpaa1SnpShutdown (
  IN        EFI_SIMPLE_NETWORK_PROTOCOL *Snp
  )
{
  EFI_STATUS Status;
  EFI_SIMPLE_NETWORK_MODE *SnpMode;
  DPAA1_ETHERNET_DEVICE *Dpaa1EthDev;
  FMAN_MEMAC *FmanMemac;

  Status = Dpaa1SnpValidateParameters(Snp);
  if (EFI_ERROR(Status)) {
      DPAA1_ERROR_MSG("Failed to Validate Snp Parameters (error %u)\n", Status);
      return Status;
  }

  Dpaa1EthDev = SNP_TO_DPAA1_DEV(Snp);
  DPAA1_DEBUG_MSG("%a() called for DPAA1 Ethernet device 0x%p\n", __func__,
                  Dpaa1EthDev);

  SnpMode = Snp->Mode;
  FmanMemac = Dpaa1EthDev->FmanMemac;

  if (SnpMode->State == EfiSimpleNetworkStarted) {
    DPAA1_ERROR_MSG("DPAA1 Ethernet device 0x%p (%a) not initialized\n",
                   Dpaa1EthDev, gFmanMemacStrings[FmanMemac->Id]);
    return EFI_DEVICE_ERROR;
  } else if (SnpMode->State == EfiSimpleNetworkStopped) {
    DPAA1_ERROR_MSG("DPAA1 Ethernet device 0x%p (%a) not started\n",
                   Dpaa1EthDev, gFmanMemacStrings[FmanMemac->Id]);
    return EFI_NOT_STARTED;
  }

  ASSERT(SnpMode->State == EfiSimpleNetworkInitialized);

  /*
   * Shutdown and reset DPAA1 network interface:
   */
  SnpMode->MCastFilterCount = 0;

  Dpaa1PhyShutdown(&FmanMemac->Phy);

  SnpMode->State = EfiSimpleNetworkStarted;

  return EFI_SUCCESS;
}


/**
  Enable and/or disable receive filters

  Please refer to the UEFI specification for the precedence rules among the
  Enable, Disable and ResetMCastFilter parameters.

  @param[in]  Snp               A pointer to the EFI_SIMPLE_NETWORK_PROTOCOL
                                instance.
  @param[in]  Enable            A bit mask of receive filters to enable.
  @param[in]  Disable           A bit mask of receive filters to disable.
  @param[in]  ResetMCastFilter  Set to TRUE to reset the contents of the multicast
                                receive filters on the network interface to
                                their default values.
  @param[in]  MCastFilterCnt    Number of multicast HW MAC addresses in the new
                                MCastFilter list. This value must be less than or
                                equal to the MCastFilterCnt field of
                                EFI_SIMPLE_NETWORK_MODE. This field is optional if
                                ResetMCastFilter is TRUE.
  @param[in]  MCastFilter       A pointer to a list of new multicast receive
                                filter HW MAC addresses. This list will replace
                                any existing multicast HW MAC address list. This
                                field is optional if ResetMCastFilter is TRUE.

  @retval  EFI_SUCCESS, on success
  @retval  error code, on failure

**/
STATIC
EFI_STATUS
EFIAPI
Dpaa1SnpReceiveFilters (
  IN  EFI_SIMPLE_NETWORK_PROTOCOL  *Snp,
  IN  UINT32                       Enable,
  IN  UINT32                       Disable,
  IN  BOOLEAN                      ResetMCastFilter,
  IN  UINTN                        MCastFilterCnt  OPTIONAL,
  IN  EFI_MAC_ADDRESS              *MCastFilter  OPTIONAL
  )
{
  EFI_STATUS Status;
  EFI_SIMPLE_NETWORK_MODE *SnpMode;
  DPAA1_ETHERNET_DEVICE *Dpaa1EthDev;
  FMAN_MEMAC *FmanMemac;

  Status = Dpaa1SnpValidateParameters(Snp);
  if (EFI_ERROR(Status)) {
      DPAA1_ERROR_MSG("Failed to Validate Snp Parameters (error %u)\n", Status);
      return Status;
  }

  Dpaa1EthDev = SNP_TO_DPAA1_DEV(Snp);
  DPAA1_DEBUG_MSG("%a() called for DPAA1 Ethernet device 0x%p\n", __func__,
                  Dpaa1EthDev);

  SnpMode = Snp->Mode;
  FmanMemac = Dpaa1EthDev->FmanMemac;

  // Check that driver was started and initialised
  if (SnpMode->State == EfiSimpleNetworkStarted) {
    DPAA1_ERROR_MSG("DPAA1 Ethernet device 0x%p (%a) not initialized\n",
                   Dpaa1EthDev, gFmanMemacStrings[FmanMemac->Id]);
    return EFI_DEVICE_ERROR;
  } else if (SnpMode->State == EfiSimpleNetworkStopped) {
    DPAA1_ERROR_MSG("DPAA1 Ethernet device 0x%p (%a) in stopped state\n",
                   Dpaa1EthDev, gFmanMemacStrings[FmanMemac->Id]);
    return EFI_NOT_STARTED;
  }

  /*
   * Validate parameters:
   */
  if (Enable & (~SnpMode->ReceiveFilterMask)) {
    DPAA1_ERROR_MSG(
      "Dpaa1SnpReceiveFilter 'Enable' mask has invalid bits:"
      " 0x%x (valid bits 0x%x)\n",
      Enable, SnpMode->ReceiveFilterMask);

    return EFI_INVALID_PARAMETER;
  }

  if (Disable & (~SnpMode->ReceiveFilterMask)) {
    DPAA1_ERROR_MSG(
      "Dpaa1SnpReceiveFilter 'Disable' mask has invalid bits:"
      " 0x%x (valid bits 0x%x)\n",
      Enable, SnpMode->ReceiveFilterMask);

    return EFI_INVALID_PARAMETER;
  }

  /*
   * As per the UEFI Specification, If the same bits are set in the Enable and
   * Disable parameters, then the bits in the Disable parameter takes precedence
   */

  if (Enable & Disable)
    Enable = Enable & ~Disable;

  if (!ResetMCastFilter)  {
    if (MCastFilterCnt > SnpMode->MaxMCastFilterCount) {
      DPAA1_ERROR_MSG(
          "Dpaa1SnpReceiveFilter 'MCastFilterCnt' (%u) is out of range\n",
          MCastFilterCnt);

      return EFI_INVALID_PARAMETER;
    }

    if (MCastFilterCnt != 0) {
      if (MCastFilter == NULL) {
        DPAA1_ERROR_MSG("Dpaa1SnpReceiveFilter 'MCastFilter' cannot be NULL\n");
        return EFI_INVALID_PARAMETER;
      }

      if (!(Enable & EFI_SIMPLE_NETWORK_RECEIVE_MULTICAST)) {
        DPAA1_ERROR_MSG(
            "Dpaa1SnpReceiveFilter 'Enable' mask (0x%x) does not have "
            "'Multicast' bit set\n",
            Enable);

        return EFI_INVALID_PARAMETER;
      }
    }
  }

  DPAA1_DEBUG_MSG("%a() called for 0x%p not implemented yet\n", __func__, Snp);

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
Dpaa1SnpStationAddress (
  IN  EFI_SIMPLE_NETWORK_PROTOCOL  *Snp,
  IN  BOOLEAN                      Reset,
  IN  EFI_MAC_ADDRESS              *New
)
{
  EFI_STATUS Status;
  EFI_SIMPLE_NETWORK_MODE *SnpMode;
  DPAA1_ETHERNET_DEVICE *Dpaa1EthDev;
  FMAN_MEMAC *FmanMemac;

  Status = Dpaa1SnpValidateParameters(Snp);
  if (EFI_ERROR(Status)) {
      DPAA1_ERROR_MSG("Failed to Validate Snp Parameters (error %u)\n", Status);
      return Status;
  }

  Dpaa1EthDev = SNP_TO_DPAA1_DEV(Snp);
  DPAA1_DEBUG_MSG("%a() called for DPAA1 Ethernet device 0x%p\n", __func__,
                  Dpaa1EthDev);

  SnpMode = Snp->Mode;
  FmanMemac = Dpaa1EthDev->FmanMemac;

  if (SnpMode->State == EfiSimpleNetworkStarted) {
    DPAA1_ERROR_MSG("DPAA1 Ethernet device 0x%p (%a) not initialized\n",
                   Dpaa1EthDev, gFmanMemacStrings[FmanMemac->Id]);
    return EFI_DEVICE_ERROR;
  } else if (SnpMode->State == EfiSimpleNetworkStopped) {
    DPAA1_ERROR_MSG("DPAA1 Ethernet device 0x%p (%a) in stopped state\n",
                   Dpaa1EthDev, gFmanMemacStrings[FmanMemac->Id]);
    return EFI_NOT_STARTED;
  }

  /*
   * TODO:  DPAA1-specific code here
   */
  DPAA1_DEBUG_MSG("%a() called for 0x%p not implemented yet\n", __func__, Snp);

  return EFI_UNSUPPORTED;
}


/**
   SNP protocol Statistics() function

   @param Snp   A pointer to the EFI_SIMPLE_NETWORK_PROTOCOL instance.

   @retval EFI_SUCCESS, on success
   @retval error code, on failure
 **/
STATIC
EFI_STATUS
EFIAPI
Dpaa1SnpStatistics (
  IN        EFI_SIMPLE_NETWORK_PROTOCOL* Snp,
  IN        BOOLEAN Reset,
  IN  OUT   UINTN *StatSize,
      OUT   EFI_NETWORK_STATISTICS *Statistics
  )
{
  EFI_STATUS Status;
  EFI_SIMPLE_NETWORK_MODE *SnpMode;
  DPAA1_ETHERNET_DEVICE *Dpaa1EthDev;
  FMAN_MEMAC *FmanMemac;

  Status = Dpaa1SnpValidateParameters(Snp);
  if (EFI_ERROR(Status)) {
      DPAA1_ERROR_MSG("Failed to Validate Snp Parameters (error %u)\n", Status);
      return Status;
  }

  Dpaa1EthDev = SNP_TO_DPAA1_DEV(Snp);
  DPAA1_DEBUG_MSG("%a() called for DPAA1 Ethernet device 0x%p\n", __func__,
                  Dpaa1EthDev);

  SnpMode = Snp->Mode;
  FmanMemac = Dpaa1EthDev->FmanMemac;

  if (SnpMode->State == EfiSimpleNetworkStarted) {
    DPAA1_ERROR_MSG("DPAA1 Ethernet device 0x%p (%a) not initialized\n",
                   Dpaa1EthDev, gFmanMemacStrings[FmanMemac->Id]);
    return EFI_DEVICE_ERROR;
  } else if (Snp->Mode->State == EfiSimpleNetworkStopped) {
    DPAA1_ERROR_MSG("DPAA1 Ethernet device 0x%p (%a) in stopped state\n",
                   Dpaa1EthDev, gFmanMemacStrings[FmanMemac->Id]);
    return EFI_NOT_STARTED;
  }

  /*
   * TODO:  DPAA1-specific code here
   */
  DPAA1_DEBUG_MSG("%a() called for 0x%p not implemented yet\n", __func__, Snp);

  return EFI_SUCCESS;
}


/**
   SNP protocol MCastIPtoMAC() function

   @param Snp   A pointer to the EFI_SIMPLE_NETWORK_PROTOCOL instance.

   @retval EFI_SUCCESS, on success
   @retval error code, on failure
 **/
STATIC
EFI_STATUS
EFIAPI
Dpaa1SnpMcastIptoMac (
  IN        EFI_SIMPLE_NETWORK_PROTOCOL* Snp,
  IN        BOOLEAN IsIpv6,
  IN        EFI_IP_ADDRESS *Ip,
      OUT   EFI_MAC_ADDRESS *McastMac
  )
{
  EFI_STATUS Status;
  EFI_SIMPLE_NETWORK_MODE *SnpMode;
  DPAA1_ETHERNET_DEVICE *Dpaa1EthDev;
  FMAN_MEMAC *FmanMemac;

  Status = Dpaa1SnpValidateParameters(Snp);
  if (EFI_ERROR(Status)) {
      DPAA1_ERROR_MSG("Failed to Validate Snp Parameters (error %u)\n", Status);
      return Status;
  }

  Dpaa1EthDev = SNP_TO_DPAA1_DEV(Snp);
  DPAA1_DEBUG_MSG("%a() called for DPAA1 Ethernet device 0x%p\n", __func__,
                  Dpaa1EthDev);

  SnpMode = Snp->Mode;
  FmanMemac = Dpaa1EthDev->FmanMemac;

  if (SnpMode->State == EfiSimpleNetworkStarted) {
    DPAA1_ERROR_MSG("DPAA1 Ethernet device 0x%p (%a) not initialized\n",
                   Dpaa1EthDev, gFmanMemacStrings[FmanMemac->Id]);
    return EFI_DEVICE_ERROR;
  } else if (SnpMode->State == EfiSimpleNetworkStopped) {
    DPAA1_ERROR_MSG("DPAA1 Ethernet device 0x%p (%a) in stopped state\n",
                   Dpaa1EthDev, gFmanMemacStrings[FmanMemac->Id]);
    return EFI_NOT_STARTED;
  }

  /*
   * TODO:  DPAA1-specific code here
   */
  DPAA1_DEBUG_MSG("%a() called for 0x%p not implemented yet\n", __func__, Snp);

  return EFI_SUCCESS;
}


/**
   SNP protocol NvData() function

   @param Snp   A pointer to the EFI_SIMPLE_NETWORK_PROTOCOL instance.

   @retval EFI_SUCCESS, on success
   @retval error code, on failure
 **/
STATIC
EFI_STATUS
EFIAPI
Dpaa1SnpNvData (
  IN        EFI_SIMPLE_NETWORK_PROTOCOL *Snp,
  IN        BOOLEAN read_write,
  IN        UINTN offset,
  IN        UINTN buff_size,
  IN  OUT   VOID *data
  )
{
  DPAA1_ETHERNET_DEVICE *Dpaa1EthDev;

  if (!Snp) {
    ASSERT(Snp != NULL);
    return EFI_INVALID_PARAMETER;
  }
  Dpaa1EthDev = SNP_TO_DPAA1_DEV(Snp);
  DPAA1_DEBUG_MSG("%a() called for DPAA1 Ethernet device 0x%p\n", __func__,
                  Dpaa1EthDev);

  DPAA1_WARN_MSG("SNP NvData not supported for DPAA1 Ethernet devices\n");
  return EFI_UNSUPPORTED;
}

/**
   SNP protocol GetStatus() function

   @param Snp   A pointer to the EFI_SIMPLE_NETWORK_PROTOCOL instance.

   @retval EFI_SUCCESS, on success
   @retval error code, on failure
 **/
STATIC
EFI_STATUS
EFIAPI
Dpaa1SnpGetStatus (
  IN   EFI_SIMPLE_NETWORK_PROTOCOL  *Snp,
  OUT  UINT32                       *IrqStat  OPTIONAL,
  OUT  VOID                         **TxBuff  OPTIONAL
  )
{
  EFI_STATUS Status;
  EFI_SIMPLE_NETWORK_MODE *SnpMode;
  DPAA1_ETHERNET_DEVICE *Dpaa1EthDev;
  FMAN_MEMAC *FmanMemac;

  Status = Dpaa1SnpValidateParameters(Snp);
  if (EFI_ERROR(Status)) {
      DPAA1_ERROR_MSG("Failed to Validate Snp Parameters (error %u)\n", Status);
      return Status;
  }

  Dpaa1EthDev = SNP_TO_DPAA1_DEV(Snp);
  DPAA1_DEBUG_MSG("%a() called for DPAA1 Ethernet device 0x%p\n", __func__,
                  Dpaa1EthDev);

  SnpMode = Snp->Mode;
  FmanMemac = Dpaa1EthDev->FmanMemac;

  if (SnpMode->State == EfiSimpleNetworkStarted) {
    return EFI_DEVICE_ERROR;
  } else if (SnpMode->State == EfiSimpleNetworkStopped) {
    return EFI_NOT_STARTED;
  }
  ASSERT(SnpMode->State == EfiSimpleNetworkInitialized);

  if (IrqStat != NULL) {
    *IrqStat = 0; /* TODO: Dow we need to set IRQ Tx/Rx status? */
  }

  if (TxBuff != NULL)
    GetTransmitStatus(Dpaa1EthDev->FmanEthDevice,TxBuff);

  /*
   * Check physical link status:
   */
  SnpMode->MediaPresent = Dpaa1PhyStatus(&FmanMemac->Phy);

  return EFI_SUCCESS;
}

/**
   SNP protocol Transmit() function

   @param Snp   A pointer to the EFI_SIMPLE_NETWORK_PROTOCOL instance.

   @retval EFI_SUCCESS, on success
   @retval error code, on failure
 **/
STATIC
EFI_STATUS
EFIAPI
Dpaa1SnpTransmit (
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
  DPAA1_ETHERNET_DEVICE *Dpaa1EthDev;
  FMAN_MEMAC *FmanMemac;

  Status = Dpaa1SnpValidateParameters(Snp);
  if (EFI_ERROR(Status)) {
      DPAA1_ERROR_MSG("Failed to Validate Snp Parameters (error %u)\n", Status);
      return Status;
  }

  Dpaa1EthDev = SNP_TO_DPAA1_DEV(Snp);
  DPAA1_DEBUG_MSG("%a() called for DPAA1 Ethernet device 0x%p\n", __func__,
                  Dpaa1EthDev);

  SnpMode = Snp->Mode;
  FmanMemac = Dpaa1EthDev->FmanMemac;

  ASSERT(HdrSize <= BuffSize);

  if (Data == NULL) {
    DPAA1_ERROR_MSG("%a() called with invalid Data parameter\n", __func__);
    return EFI_INVALID_PARAMETER;
  }

  if (SnpMode->State == EfiSimpleNetworkStarted) {
    DPAA1_ERROR_MSG("DPAA1 Ethernet device 0x%p (%a) not initialized\n",
                   Dpaa1EthDev, gFmanMemacStrings[FmanMemac->Id]);
    return EFI_DEVICE_ERROR;
  } else if (Snp->Mode->State == EfiSimpleNetworkStopped) {
    DPAA1_ERROR_MSG("DPAA1 Ethernet device 0x%p (%a) in stopped state\n",
                   Dpaa1EthDev, gFmanMemacStrings[FmanMemac->Id]);
    return EFI_NOT_STARTED;
  }

  if (HdrSize != 0) {
    if (HdrSize != SnpMode->MediaHeaderSize) {
      DPAA1_ERROR_MSG("%a() called with invalid HdrSize parameter: %lu\n",
                      __func__, HdrSize);
      return EFI_INVALID_PARAMETER;
    }

    if (DstAddr == NULL) {
      DPAA1_ERROR_MSG("%a() called with invalid DstAddr parameter\n", __func__);
      return EFI_INVALID_PARAMETER;
    }

    if (Protocol == NULL) {
      DPAA1_ERROR_MSG("%a() called with invalid Protocol parameter\n", __func__);
      return EFI_INVALID_PARAMETER;
    }
  }

  if (BuffSize < SnpMode->MediaHeaderSize ||
      BuffSize > MEMAC_MAXFRM) {
      DPAA1_ERROR_MSG("%a() called with invalid BuffSize parameter: %lu\n",
                      __func__, BuffSize);
      return EFI_BUFFER_TOO_SMALL;
  }

  Status = Dpaa1Transmit(Dpaa1EthDev,
				HdrSize,
				BuffSize,
				Data,
				SrcAddr,
				DstAddr,
				Protocol);

  return Status;
}

/**
   SNP protocol Receive() function

   @param Snp   A pointer to the EFI_SIMPLE_NETWORK_PROTOCOL instance.

   @retval EFI_SUCCESS, on success
   @retval error code, on failure
 **/
STATIC
EFI_STATUS
EFIAPI
Dpaa1SnpReceive (
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
  DPAA1_ETHERNET_DEVICE *Dpaa1EthDev;
  FMAN_MEMAC *FmanMemac;

  Status = Dpaa1SnpValidateParameters(Snp);
  if (EFI_ERROR(Status)) {
      DPAA1_ERROR_MSG("Failed to Validate Snp Parameters (error %u)\n", Status);
      return Status;
  }

  Dpaa1EthDev = SNP_TO_DPAA1_DEV(Snp);
  DPAA1_DEBUG_MSG("%a() called for DPAA1 Ethernet device 0x%p\n", __func__,
                  Dpaa1EthDev);

  SnpMode = Snp->Mode;
  FmanMemac = Dpaa1EthDev->FmanMemac;

  if (Data == NULL) {
    DPAA1_ERROR_MSG("%a() called with invalid Data parameter\n", __func__);
    return EFI_INVALID_PARAMETER;
  }

  if (BuffSize == NULL) {
    DPAA1_ERROR_MSG("%a() called with Invalid BuffSize parameter\n", __func__);
    return EFI_INVALID_PARAMETER;
  }

  if (SnpMode->State == EfiSimpleNetworkStarted) {
    DPAA1_ERROR_MSG("DPAA1 Ethernet device 0x%p (%a) not initialized\n",
                   Dpaa1EthDev, gFmanMemacStrings[FmanMemac->Id]);
    return EFI_DEVICE_ERROR;
  } else if (Snp->Mode->State == EfiSimpleNetworkStopped) {
    DPAA1_ERROR_MSG("DPAA1 Ethernet device 0x%p (%a) in stopped state\n",
                   Dpaa1EthDev, gFmanMemacStrings[FmanMemac->Id]);
    return EFI_NOT_STARTED;
  }

   if (*BuffSize < SnpMode->MediaHeaderSize) {
      DPAA1_ERROR_MSG("%a() called with invalid *BuffSize parameter: %lu\n",
                      __func__, *BuffSize);
      return EFI_BUFFER_TOO_SMALL;
  }

  if (HdrSize != NULL) {
    ASSERT(SnpMode->MediaHeaderSize == sizeof(ETHER_HEAD));
    *HdrSize = SnpMode->MediaHeaderSize;
  }

  Status = Dpaa1Receive(Dpaa1EthDev,
                           BuffSize,
                           Data,
                           SrcAddr,
                           DstAddr,
                           Protocol);

  return Status;
}

/**
 * Initializer template for a DPAA1 Ethernet device instance
 */
STATIC CONST DPAA1_ETHERNET_DEVICE gDpaa1EthernetDeviceInitTemplate = {
  .Signature = DPAA1_ETHERNET_DEVICE_SIGNATURE,
  .ControllerHandle = NULL,
  .Snp = {
    .Revision = EFI_SIMPLE_NETWORK_PROTOCOL_REVISION,
    .Start = Dpaa1SnpStart,
    .Stop = Dpaa1SnpStop,
    .Initialize = Dpaa1SnpInitialize,
    .Reset = Dpaa1SnpReset,
    .Shutdown = Dpaa1SnpShutdown,
    .ReceiveFilters = Dpaa1SnpReceiveFilters,
    .StationAddress = Dpaa1SnpStationAddress,
    .Statistics = Dpaa1SnpStatistics,
    .MCastIpToMac = Dpaa1SnpMcastIptoMac,
    .NvData = Dpaa1SnpNvData,
    .GetStatus = Dpaa1SnpGetStatus,
    .Transmit = Dpaa1SnpTransmit,
    .Receive = Dpaa1SnpReceive,
    .WaitForPacket = NULL,
    .Mode = NULL,
  },

  .SnpMode = {
    .State = EfiSimpleNetworkStopped,
    .HwAddressSize = NET_ETHER_ADDR_LEN,
    .MediaHeaderSize = sizeof(ETHER_HEAD),
    .MaxPacketSize = MEMAC_MAXFRM - sizeof(ETHER_HEAD) \
                     - NET_VLAN_TAG_LEN - 4/*Ethernet FCS(Frame Check Sequence)*/,
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
        .Length = { (UINT8)sizeof(MAC_ADDR_DEVICE_PATH),
                    (UINT8)(sizeof(MAC_ADDR_DEVICE_PATH) >> 8) },
      },

      .MacAddress = { .Addr = { 0 } },
      .IfType = NET_IFTYPE_ETHERNET,
    },

    .EndMarker = {
      .Type = END_DEVICE_PATH_TYPE,
      .SubType = END_ENTIRE_DEVICE_PATH_SUBTYPE,
      .Length = { (UINT8)sizeof(EFI_DEVICE_PATH_PROTOCOL), 0 }
    }
  },

  .FmanMemac = NULL ,
};

VOID
Dpaa1StopNetworkInterface (
  IN  DPAA1_ETHERNET_DEVICE *Dpaa1EthDev
  )
{
  ETH_DEVICE *FmanEthDevice;

  FmanEthDevice = (ETH_DEVICE *)Dpaa1EthDev->FmanEthDevice;

  /* Enable graceful stop for TX */
  TxGracefulStopEnable(FmanEthDevice);

  /* Disable BMI and Mac TX/RX ports */
  DisablePorts(FmanEthDevice);
}

VOID
Dpaa1StartNetworkInterface (
  IN  DPAA1_ETHERNET_DEVICE *Dpaa1EthDev
  )
{
  ETH_DEVICE *FmanEthDevice;

  FmanEthDevice = (ETH_DEVICE *)Dpaa1EthDev->FmanEthDevice;

  /* Enable BMI and Mac TX/RX ports */
  EnablePorts(FmanEthDevice);

  /* re-enable transmission of frame */
  TxGracefulStopDisable(FmanEthDevice);
}

/**
   Receives an Ethernet frame on a DPAA1 network interface

   @param Dpaa1EthDev	Pointer to DPAA1 network interface control block
   @param BuffSize          Pointer to frame size
   @param Data              Pointer to frame buffer
   @param SrcAddr           Pointer to source MAC address
   @param DstAddr           Pointer to destination MAC address
   @param Protocol          Pointer to carried protocol

   @retval EFI_SUCCESS, on success
   @retval error code, on failure
 **/
EFI_STATUS
Dpaa1Receive(
  IN  DPAA1_ETHERNET_DEVICE *Dpaa1EthDev,
  UINTN *BuffSize,
  VOID *Data,
  EFI_MAC_ADDRESS *SrcAddr,
  EFI_MAC_ADDRESS *DstAddr,
  UINT16 *Protocol
)
{
  EFI_STATUS Status;

  ETHER_HEAD *EthernetHeader = NULL;

  /*
   * Receive frame:
   */
  Status = ReceiveFrame(Dpaa1EthDev->FmanEthDevice,
                       BuffSize,
                       Data);

  if (!EFI_ERROR(Status)) {
    EthernetHeader = (ETHER_HEAD *)Data;
    if(DstAddr != NULL) {
      CopyMem(DstAddr, EthernetHeader->DstMac, NET_ETHER_ADDR_LEN);
    }
 
    if(SrcAddr != NULL) {
      CopyMem(SrcAddr, EthernetHeader->SrcMac, NET_ETHER_ADDR_LEN);
    }
 
    if (Protocol != NULL) {
      *Protocol = NTOHS(EthernetHeader->EtherType);
    }

    if (gDpaa1DebugFlags & DPAA1_DEBUG_TRACE_NET_PACKETS) {
      DPAA1_DEBUG_MSG("Rx: %x:%x:%x:%x:%x:%x|%x:%x:%x:%x:%x:%x|%x|%x|%x%x%x%x\n",
        EthernetHeader->SrcMac[0], EthernetHeader->SrcMac[1],
        EthernetHeader->SrcMac[2], EthernetHeader->SrcMac[3],
        EthernetHeader->SrcMac[4], EthernetHeader->SrcMac[5],
        EthernetHeader->DstMac[0], EthernetHeader->DstMac[1],
        EthernetHeader->DstMac[2], EthernetHeader->DstMac[3],
        EthernetHeader->DstMac[4], EthernetHeader->DstMac[5],
        NTOHS(EthernetHeader->EtherType), *BuffSize,
        ((UINT8 *)Data)[0],
        ((UINT8 *)Data)[1],
        ((UINT8 *)Data)[2],
        ((UINT8 *)Data)[3]);
    }
  }

  return Status;
}

/**
   Transmits an Ethernet frame on a DPAA1 network interface

   @param Dpaa1EthDev	   Pointer to DPAA1 network interface control block
   @param HdrSize              Header size
   @param BuffSize             Total frame size
   @param Data                 Pointer to frame buffer
   @param SrcAddr              Pointer to source MAC address
   @param DstAddr              Pointer to destination MAC address
   @param Protocol             Pointer to carried protocol

   @retval EFI_SUCCESS, on success
   @retval error code, on failure
 **/
EFI_STATUS
Dpaa1Transmit (
  IN  DPAA1_ETHERNET_DEVICE *Dpaa1EthDev,
  UINTN HdrSize,
  UINTN BuffSize,
  VOID *Data,
  EFI_MAC_ADDRESS *SrcAddr,
  EFI_MAC_ADDRESS *DstAddr,
  UINT16 *Protocol
  )
{
  EFI_STATUS Status;

  ASSERT(Data != NULL);
  ASSERT(BuffSize >= sizeof(ETHER_HEAD) && BuffSize <= MEMAC_MAXFRM);
  ASSERT(HdrSize <= BuffSize);

  if (HdrSize != 0) {
    /*
     * Populate Ethernet header:
     */
    ETHER_HEAD *EthernetHeader = (ETHER_HEAD *)Data;

    ASSERT(HdrSize == sizeof(ETHER_HEAD));
    ASSERT(SrcAddr != NULL);
    ASSERT(DstAddr != NULL);
    ASSERT(Protocol != NULL);

    CopyMem(EthernetHeader->DstMac, DstAddr, NET_ETHER_ADDR_LEN);
    CopyMem(EthernetHeader->SrcMac, SrcAddr, NET_ETHER_ADDR_LEN);
    EthernetHeader->EtherType = HTONS(*Protocol);
  }

  if (gDpaa1DebugFlags & DPAA1_DEBUG_TRACE_NET_PACKETS) {
    DPAA1_DEBUG_MSG("Tx: %x:%x:%x:%x:%x:%x|%x:%x:%x:%x:%x:%x|%x|%x|%x%x%x%x\n",
      SrcAddr->Addr[0], SrcAddr->Addr[1], SrcAddr->Addr[2],
      SrcAddr->Addr[3], SrcAddr->Addr[4], SrcAddr->Addr[5],
      DstAddr->Addr[0], DstAddr->Addr[1], DstAddr->Addr[2],
      DstAddr->Addr[3], DstAddr->Addr[4], DstAddr->Addr[5],
      *Protocol, BuffSize,
      ((UINT8 *)Data)[0],
      ((UINT8 *)Data)[1],
      ((UINT8 *)Data)[2],
      ((UINT8 *)Data)[3]);
  }

  Status = TransmitFrame(Dpaa1EthDev->FmanEthDevice, Data, BuffSize);

  return Status;
}

STATIC INT32 EthInit (
  IN  DPAA1_ETHERNET_DEVICE *EthDev
  )
{
  EFI_STATUS Status = EFI_SUCCESS;

  Status = FmRxPortParamInit(EthDev->FmanEthDevice);
  if (Status)
    return Status;

  Status = FmTxPortParamInit(EthDev->FmanEthDevice);
  if (Status)
    return Status;
  
  return Status;
}

STATIC INT32 EthDevStart (
  IN  DPAA1_ETHERNET_DEVICE *EthDev
  )
{
  EFI_STATUS Status = EFI_SUCCESS;
  ENET_MAC *Mac = EthDev->FmanEthDevice->Mac;

  /* Rx/TxBDs, Rx/TxQDs, Rx buff and parameter ram init */
  Status = EthInit(EthDev);
  if (Status != EFI_SUCCESS)
    return Status;

  /* setup the MAC controller */
  InitializeMac(Mac);

  /* For some reason we need to set SPEED_100 */
  if ((EthDev->FmanMemac->Phy.PhyInterfaceType == PHY_INTERFACE_SGMII) ||
      (EthDev->FmanMemac->Phy.PhyInterfaceType == PHY_INTERFACE_QSGMII))
    SetInterface(Mac, EthDev->FmanMemac->Phy.PhyInterfaceType, 100);

  /* init bmi rx and tx port, IM mode and disable */
  BmiRxPortInit(EthDev->FmanEthDevice->RxPort);
  BmiTxPortInit(EthDev->FmanEthDevice->TxPort);

  return Status;
}

STATIC
EFI_STATUS
EFIAPI
CreateDpaa1EthernetDevice(
  IN FMAN_MEMAC *Memac,
  OUT DPAA1_ETHERNET_DEVICE **Dpaa1EthDevPtr
  )
{
  EFI_STATUS Status;
  DPAA1_ETHERNET_DEVICE *Dpaa1EthDev;
  ETH_DEVICE *FmanEthDevice;
  UINT32 SocUniqueId = 0;

  Status = GetNVSocUniqueId (&SocUniqueId);
  if (Status != EFI_SUCCESS) {
    DPAA1_ERROR_MSG("Failed to get SocUniqueId \n");
    return Status;
  }

  Dpaa1EthDev = AllocateCopyPool(sizeof(DPAA1_ETHERNET_DEVICE),
                                 &gDpaa1EthernetDeviceInitTemplate);
  if (Dpaa1EthDev == NULL) {
    DPAA1_ERROR_MSG("Could not allocate DPAA1 Ethernet device\n");
    return EFI_OUT_OF_RESOURCES;
  }

  ASSERT(Dpaa1EthDev->Signature == DPAA1_ETHERNET_DEVICE_SIGNATURE);

  /* alloc the FMan ethernet private struct */
  FmanEthDevice = AllocateZeroPool(sizeof(ETH_DEVICE));
  if (FmanEthDevice == NULL) {
    DPAA1_ERROR_MSG("Could not allocate DPAA1 Ethernet device\n");
    return EFI_OUT_OF_RESOURCES;
  }

  Dpaa1EthDev->Snp.Mode = &Dpaa1EthDev->SnpMode;
  Dpaa1EthDev->FmanMemac = Memac;
  Dpaa1EthDev->FmanEthDevice = FmanEthDevice; 
  InitializeListHead(&Dpaa1EthDev->ListNode);

  *Dpaa1EthDevPtr = Dpaa1EthDev; 

  PopulateEthDev(FmanEthDevice, Memac->Id);

  /* init structure for global mac */
  Status = EthDevInitMac(FmanEthDevice);
  if (Status != EFI_SUCCESS)
    return Status;

  /* startup the FM Independent Mode */
  Status = EthDevStart(Dpaa1EthDev);
  if (Status != EFI_SUCCESS)
    return Status;

  if (PcdGetBool(PcdSgmiiPrtclInit)) {
    if (Dpaa1EthDev->FmanMemac->Phy.PhyInterfaceType == PHY_INTERFACE_SGMII ||
        Dpaa1EthDev->FmanMemac->Phy.PhyInterfaceType == PHY_INTERFACE_SGMII_2500) {
        DPAA1_PHY_MDIO_BUS MdioBus = {
        .Signature = DPAA1_PHY_MDIO_BUS_SIGNATURE,
        .IoRegs = (MEMAC_MDIO_BUS_REGS *)Dpaa1EthDev->FmanEthDevice->Mac->PhyRegs,
      };
  // On L1046A After initializing the MEMAC
  // And prior to initiating any SGMII traffic do SGMII protocol initialization

      DtsecInitPhy (&MdioBus, &Memac->Phy);
    }
  }

  Status = Dpaa1PhyInit(&Memac->Phy);
  if (EFI_ERROR(Status)) {
    DPAA1_ERROR_MSG("Failed to initialize PHY for DPAA1 Ethernet device 0x%p (error %u)\n",
                    Dpaa1EthDev, Status);
    goto ErrorExitFreeDpaa1EthDevice;
  } 
  Dpaa1EthDev->PhyInitialized = TRUE;

  /*
   * Set MAC address for the DPAA1 Ethernet device:
   */
//  if (EFI_ERROR(MacReadFromEeprom(Memac->Id, &Dpaa1EthDev->SnpMode.CurrentAddress))) {
    GenerateMacAddress(SocUniqueId, Memac->Id, &Dpaa1EthDev->SnpMode.CurrentAddress);
//  }

  /*
   * Copy MAC address to the UEFI device path for the DPAA1 Ethernet device:
   */
  CopyMem(&Dpaa1EthDev->DevicePath.MacAddrDevicePath.MacAddress,
          &Dpaa1EthDev->SnpMode.CurrentAddress,
          NET_ETHER_ADDR_LEN);

  /*
   * Install driver model protocols:
   */

  DPAA1_DEBUG_MSG("Installing UEFI protocol interfaces for DPAA1 Ethernet device 0x%p ...\n",
                  Dpaa1EthDev);

  ASSERT(Dpaa1EthDev->ControllerHandle == NULL);
  Status = gBS->InstallMultipleProtocolInterfaces (
                  &Dpaa1EthDev->ControllerHandle,
                  &gEfiSimpleNetworkProtocolGuid, &Dpaa1EthDev->Snp,
                  &gEfiDevicePathProtocolGuid, &Dpaa1EthDev->DevicePath,
                  NULL
                  );
  if (EFI_ERROR(Status)) {
    DPAA1_ERROR_MSG("Failed to install UEFI protocols for DPAA1 Ethernet device 0x%p (error %u)\n",
                    Dpaa1EthDev, Status);
    goto ErrorExitFreeDpaa1EthDevice;
  }

  return EFI_SUCCESS;

ErrorExitFreeDpaa1EthDevice:
  FreePool(Dpaa1EthDev);
  FreePool(FmanEthDevice);

  return Status; 
}

STATIC
VOID
EFIAPI
DestroyDpaa1EthernetDevice(
  IN DPAA1_ETHERNET_DEVICE *Dpaa1EthDev
  )
{
  EFI_STATUS Status;

  ASSERT(Dpaa1EthDev->Signature == DPAA1_ETHERNET_DEVICE_SIGNATURE);
  DPAA1_DEBUG_MSG("Destroying DPAA1 Ethernet device object 0x%p ...\n",
                  Dpaa1EthDev);

  if (Dpaa1EthDev->ControllerHandle != NULL) {
    Status = gBS->UninstallMultipleProtocolInterfaces (
                   Dpaa1EthDev->ControllerHandle,
                   &gEfiSimpleNetworkProtocolGuid, &Dpaa1EthDev->Snp,
                   &gEfiDevicePathProtocolGuid, &Dpaa1EthDev->DevicePath,
                   NULL
                   );

    if (EFI_ERROR(Status)) {
      DPAA1_ERROR_MSG("Failed to uninstall UEFI driver model protocols for DPAA1 device 0x%p (error %u)\n",
                      Dpaa1EthDev, Status);
    }
  }

  FreePool(Dpaa1EthDev->FmanEthDevice); 
  FreePool(Dpaa1EthDev); 
}

VOID
DestroyAllDpaa1NetworkInterfaces(VOID)
{
  LIST_ENTRY *ListNode;
  DPAA1_ETHERNET_DEVICE *Dpaa1EthDev;
  FMAN_MEMAC *FmanMemac;

  for (ListNode = GetFirstNode(&gDpaa1Driver.Dpaa1EthernetDevicesList);
       ListNode != &gDpaa1Driver.Dpaa1EthernetDevicesList;
       ListNode = GetNextNode(&gDpaa1Driver.Dpaa1EthernetDevicesList,
                              ListNode)) {
    Dpaa1EthDev = CR(ListNode, DPAA1_ETHERNET_DEVICE, ListNode,
                     DPAA1_ETHERNET_DEVICE_SIGNATURE);

    FmanMemac = Dpaa1EthDev->FmanMemac;
    ASSERT(FmanMemac != NULL);

    /*
     * Destroy DPAA1 network interface:
     */
    DPAA1_INFO_MSG("Disabling DPAA1 Ethernet physical device for %a ...\n",
                   gFmanMemacStrings[FmanMemac->Id]);
    Dpaa1StopNetworkInterface(Dpaa1EthDev);

    Dpaa1PhyShutdown(&Dpaa1EthDev->FmanMemac->Phy);
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
Dpaa1NotifyExitBootServices (
  EFI_EVENT Event,
  VOID      *Context
  )
{
  EFI_HANDLE ImageHandle = NULL;
  EFI_STATUS Status = EFI_SUCCESS;

  ASSERT(Event == gDpaa1Driver.ExitBootServicesEvent);
  ASSERT(gDpaa1Driver.ExitBootServicesEvent != NULL);

  DPAA1_DEBUG_MSG("************ %a() called (Event: 0x%x, Context: 0x%p)\n",
                  __func__, Event, Context);

  if (gDpaa1Driver.FmanStatus == EFI_SUCCESS) {
    DestroyAllDpaa1NetworkInterfaces();
    Status = Dpaa1EthernetUnload(ImageHandle);
    if (EFI_ERROR(Status))
      DPAA1_ERROR_MSG("Dpaa1EthernetUnload Failed \n");
  }
}


/**
   DPAA1 Ethernet Driver unload entry point

   @param ImageHandle   Driver image handle

   @retval EFI_SUCCESS, on success
   @retval error code, on failure
 */

EFI_STATUS
EFIAPI
Dpaa1EthernetUnload (
  IN EFI_HANDLE ImageHandle
  )
{
  EFI_STATUS Status;
  LIST_ENTRY *ListNode;
  LIST_ENTRY *NextNode;
  DPAA1_ETHERNET_DEVICE *Dpaa1EthDev;

  /*
   * Destroy all DPAA1 Ethernet devices:
   */
  for (ListNode = GetFirstNode(&gDpaa1Driver.Dpaa1EthernetDevicesList);
       ListNode != &gDpaa1Driver.Dpaa1EthernetDevicesList;
       ListNode = NextNode) {
    Dpaa1EthDev = CR(ListNode, DPAA1_ETHERNET_DEVICE, ListNode,
                     DPAA1_ETHERNET_DEVICE_SIGNATURE);

    NextNode = RemoveEntryList(&Dpaa1EthDev->ListNode);
    DestroyDpaa1EthernetDevice(Dpaa1EthDev);
  }

  ASSERT(IsListEmpty(&gDpaa1Driver.Dpaa1EthernetDevicesList));
  InitializeListHead(&gDpaa1Driver.MemacsList);

  Status = gBS->CloseEvent(gDpaa1Driver.ExitBootServicesEvent);
  if (EFI_ERROR(Status)) {
    DPAA1_ERROR_MSG("Failed to close UEFI event x0x%p (error %u)\n",
                    gDpaa1Driver.ExitBootServicesEvent, Status);
    return Status;
  }

  return EFI_SUCCESS;
}

VOID
InhibitPs (
  IN  VOID   *PAddr,
  IN  UINT32 MaxPortals,
  IN  UINT32 MaxArchPortals,
  IN  UINT32 PortalSize
  )
{
  UINT32     Value;
  UINT32     Index;;

  //
  // MaxArchPortals is the maximum based on memory size. This includes
  // the reserved memory in the SoC. MaxPortals is the number of physical
  // portals in the SoC
  //
  if (MaxPortals > MaxArchPortals) {
    DEBUG ((DEBUG_ERROR, "QBman portal config error\n"));
    MaxPortals = MaxArchPortals;
  }

  for (Index = 0; Index < MaxPortals; Index++) {
    BeMmioWrite32 ((UINTN)PAddr, -1);
    Value = BeMmioRead32 ((UINTN)PAddr);
    if (!Value) {
      DEBUG ((DEBUG_ERROR, "Stopped after %d portals\n", Index));
      break;
    }
    PAddr += PortalSize;
  }

  return;
}

VOID
SetQBManPortals (VOID)
{
  VOID *BmanPAddr;
  VOID *QmanPAddr;

  BmanPAddr = (VOID *)BMAN_CINH_BASE + BMAN_SWP_ISDR_REG;
  QmanPAddr = (VOID *)QMAN_CINH_BASE + QMAN_SWP_ISDR_REG;

  //
  // It will change default state of BMan
  // ISDR portals to all 1s
  //
  InhibitPs (BmanPAddr, BMAN_NUM_PORTALS, MAX_BMAN_PORTALS, BMAN_SP_CINH_SIZE);
  InhibitPs (QmanPAddr, QMAN_NUM_PORTALS, MAX_QMAN_PORTALS, QMAN_SP_CINH_SIZE);
}

/**
   DPAA1 Ethernet Driver initialization entry point

   @param ImageHandle   Driver image handle
   @param SystemTable   Pointer to the UEFI system table

   @retval EFI_SUCCESS, on success
   @retval error code, on failure
 */
EFI_STATUS
EFIAPI
Dpaa1EthernetInitialize (
  IN EFI_HANDLE         ImageHandle,
  IN EFI_SYSTEM_TABLE   *SystemTable
  )
{
  EFI_STATUS Status;
  LIST_ENTRY *ListNode;
  LIST_ENTRY *NextNode;
  DPAA1_ETHERNET_DEVICE *Dpaa1EthDev = NULL;
  BOOLEAN Dpaa1Enabled = PcdGetBool(PcdDpaa1Initialize);
  UINT64 Dpaa1UsedMemacsMask = PcdGet64(PcdDpaa1UsedMemacsMask);
  VOID              *Fdt;

  SetQBManPortals ();

  Status = EfiGetSystemConfigurationTable (&gFdtTableGuid, &Fdt);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "Did not find the Dtb Blob.\n"));
    return EFI_NOT_FOUND;
  }

  gDpaa1DebugFlags = PcdGet32(PcdDpaa1DebugFlags);
  DPAA1_DEBUG_MSG("Initialize %a() %a %a\n", __func__, __DATE__, __TIME__);

  if (!Dpaa1Enabled) {
    DPAA1_DEBUG_MSG("DPAA1 is disabled\n");
    return EFI_SUCCESS;
  }

  Status = Dpaa1PhyMdioBusesInit(gDpaa1MdioBuses, DPAA1_MDIO_BUSES_COUNT);
  if (EFI_ERROR(Status)) {
    return Status;
  } 

  /*
   * Discover enabled MEMACs in the RCW:
   */
  ASSERT(IsListEmpty(&gDpaa1Driver.MemacsList));
  SerDesProbeLanes(Dpaa1ParseSerDes, &gDpaa1Driver.MemacsList);
  /*
   * Since we can not deduce the RGMII interface presence from
   * SERDES protocol, we have to rely on Refrence Manual and
   * give appropriate DTSEC ids to RGMII interfaces.
  */
  Dpaa1DiscoverFmanMemac(FM1_DTSEC_3, PHY_INTERFACE_RGMII, &gDpaa1Driver.MemacsList);
  Dpaa1DiscoverFmanMemac(FM1_DTSEC_4, PHY_INTERFACE_RGMII, &gDpaa1Driver.MemacsList);
  ASSERT(!IsListEmpty(&gDpaa1Driver.MemacsList));
  /*
   * Load and boot DPAA1 FMAN firmware:
   */
  gDpaa1Driver.FmanStatus = DpaaFrameManagerInit();
  if (EFI_ERROR(gDpaa1Driver.FmanStatus)) {
    goto ErroExitCleanupMemacsList;
  }
  /*
   * Traverse list of discovered MEMACs and create corresponding DPAA1
   * Ethernet devices, for the MEMACs actually enabled by the user:
   */
  ASSERT(IsListEmpty(&gDpaa1Driver.Dpaa1EthernetDevicesList));
  for (ListNode = GetFirstNode(&gDpaa1Driver.MemacsList);
       ListNode != &gDpaa1Driver.MemacsList;
       ListNode = GetNextNode(&gDpaa1Driver.MemacsList, ListNode)) {

    FMAN_MEMAC*  Memac = CR(ListNode, FMAN_MEMAC, ListNode,
                            FMAN_MEMAC_SIGNATURE);
    DPAA1_DEBUG_MSG("Creating Eth device for :\n");
    DPAA1_DEBUG_MSG("Memac->Id = %a phy type = %d, address = 0x%x, \n",
	gFmanMemacStrings[Memac->Id], Memac->Phy.PhyInterfaceType, Memac->Phy.PhyAddress);
  
    if ((BIT(Memac->Id - 1) & Dpaa1UsedMemacsMask) == 0) {
      continue;
    }

    Status = CreateDpaa1EthernetDevice(Memac, &Dpaa1EthDev);
    if (EFI_ERROR(Status)) {
      continue;
    }

    InsertTailList(&gDpaa1Driver.Dpaa1EthernetDevicesList, &Dpaa1EthDev->ListNode); 
  }

  if (IsListEmpty(&gDpaa1Driver.Dpaa1EthernetDevicesList)) {
    DPAA1_ERROR_MSG("No usable DPAA1 devices\n");
    Status = EFI_DEVICE_ERROR;
    goto ErroExitCleanupMemacsList;
  }

  Status = FdtMemacFixup (Fdt);
  if (EFI_ERROR (Status)) {
    goto ErrorExitCleanupDpaa1EthDevices;
  }

  Status = gBS->CreateEvent (
              EVT_SIGNAL_EXIT_BOOT_SERVICES,
              TPL_CALLBACK,
              Dpaa1NotifyExitBootServices,
              NULL,
              &gDpaa1Driver.ExitBootServicesEvent
              );

  if (EFI_ERROR(Status)) {
    DPAA1_ERROR_MSG("Failed to create UEFI event (error %u)\n", Status);
    goto ErrorExitCleanupDpaa1EthDevices;
  }

  return EFI_SUCCESS;

ErrorExitCleanupDpaa1EthDevices:
  for (ListNode = GetFirstNode(&gDpaa1Driver.Dpaa1EthernetDevicesList);
       ListNode != &gDpaa1Driver.Dpaa1EthernetDevicesList;
       ListNode = NextNode) {
    Dpaa1EthDev = CR(ListNode, DPAA1_ETHERNET_DEVICE, ListNode,
                      DPAA1_ETHERNET_DEVICE_SIGNATURE);

    NextNode = RemoveEntryList(&Dpaa1EthDev->ListNode);
    DestroyDpaa1EthernetDevice(Dpaa1EthDev);
  }

  ASSERT(IsListEmpty(&gDpaa1Driver.Dpaa1EthernetDevicesList));

ErroExitCleanupMemacsList:
  InitializeListHead(&gDpaa1Driver.MemacsList);
  return Status; 
}

