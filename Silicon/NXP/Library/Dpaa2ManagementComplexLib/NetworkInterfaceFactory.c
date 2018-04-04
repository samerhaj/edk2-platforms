/** NetworkInterfaceFactory.c
  Services to create MC objecs that make an Ethernet network interface

  Copyright 2017 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution. The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include <Library/BaseLib.h>
#include <Library/BaseMemoryLib/MemLibInternals.h>
#include <Library/Dpaa2ManagementComplexLib.h>
#include <Library/DpaaDebugLib.h>
#include <Library/Dpaa2McIo.h>
#include <Library/Dpaa2McInterfaceLib/fsl_dpbp.h>
#include <Library/Dpaa2McInterfaceLib/fsl_dpio.h>
#include <Library/Dpaa2McInterfaceLib/fsl_dpni.h>
#include <Library/Dpaa2McInterfaceLib/fsl_dpmac.h>
#include <Library/Dpaa2McInterfaceLib/fsl_dprc.h>
#include <Library/Dpaa2McInterfaceLib/fsl_dpbp_cmd.h>
#include <Library/Dpaa2McInterfaceLib/fsl_dpio_cmd.h>
#include <Library/Dpaa2McInterfaceLib/fsl_dpni_cmd.h>
#include <Library/Dpaa2McInterfaceLib/fsl_dpmac_cmd.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/NetLib.h>
#include <Library/TimerLib.h>
#include <Library/UefiLib.h>

#include "ManagementComplex.h"
#include "QBman.h"

STATIC const EFI_LOCK gEfiLockInitializer = EFI_INITIALIZE_LOCK_VARIABLE(TPL_HIGH_LEVEL);

STATIC
EFI_STATUS
Dpaa2McCreateDprc (
  DPAA2_MC_IO *ParentDprcMcIo,
  UINT16 ParentDprcHandle,
  DPAA2_NETWORK_INTERFACE *Dpaa2NetInterface
  )
{
  EFI_STATUS Status;
  INT32 McFlibError;
  struct dprc_cfg DprcCfg;
  UINT64 DprcMcPortalOffset;
  INT32 DprcMcPortalId;

  ASSERT (ParentDprcHandle != 0);
  ASSERT (Dpaa2NetInterface->DprcHandle == 0);
  ZeroMem (&DprcCfg, sizeof (DprcCfg));
  DprcCfg.options = DPRC_CFG_OPT_TOPOLOGY_CHANGES_ALLOWED |
        DPRC_CFG_OPT_OBJ_CREATE_ALLOWED |
        DPRC_CFG_OPT_ALLOC_ALLOWED;
  DprcCfg.icid = DPRC_GET_ICID_FROM_POOL;
  DprcCfg.portal_id = DPRC_GET_PORTAL_ID_FROM_POOL;
  McFlibError = dprc_create_container (ParentDprcMcIo, MC_CMD_NO_FLAGS,
                    ParentDprcHandle,
                    &DprcCfg,
                    &Dpaa2NetInterface->DprcId,
                    &DprcMcPortalOffset);
  if (McFlibError != 0) {
    DPAA_ERROR_MSG ("Error calling dprc_create_container () for 0x%p\n",
                    Dpaa2NetInterface);
    return EFI_DEVICE_ERROR;
  }

  DprcMcPortalId = MC_PORTAL_OFFSET_TO_PORTAL_ID (DprcMcPortalOffset);
  Dpaa2NetInterface->DprcMcIo.McPortal =
    (volatile DPAA2_MC_COMMAND *)DPAA2_MC_PORTAL_ADDR (DprcMcPortalId);

  Dpaa2NetInterface->DprcMcIo.McSendCmdLock = gEfiLockInitializer;
  McFlibError = dprc_open (&Dpaa2NetInterface->DprcMcIo, MC_CMD_NO_FLAGS,
                          Dpaa2NetInterface->DprcId,
        &Dpaa2NetInterface->DprcHandle);
  if (McFlibError != 0) {
    DPAA_ERROR_MSG ("Error calling dprc_open () for 0x%p\n",
                    Dpaa2NetInterface);
    Status = EFI_DEVICE_ERROR;
    goto ErrorExitDestroyDprc;
  }

  ASSERT (Dpaa2NetInterface->DprcHandle != 0);
  DPAA_DEBUG_MSG ("Created dprc.%u (MC portal physical address 0x%llx) in the MC\n",
                  Dpaa2NetInterface->DprcId, Dpaa2NetInterface->DprcMcIo.McPortal);
  return EFI_SUCCESS;

ErrorExitDestroyDprc:
  (VOID)dprc_destroy_container (ParentDprcMcIo, MC_CMD_NO_FLAGS,
             ParentDprcHandle,
                               Dpaa2NetInterface->DprcId);
  return Status;
}


STATIC
VOID
Dpaa2McDestroyDprc (
  DPAA2_MC_IO *ParentDprcMcIo,
  UINT16 ParentDprcHandle,
  DPAA2_NETWORK_INTERFACE *Dpaa2NetInterface
  )
{
  INT32 McFlibError;

  ASSERT (Dpaa2NetInterface->DprcHandle != 0);
  McFlibError = dprc_close (&Dpaa2NetInterface->DprcMcIo, MC_CMD_NO_FLAGS,
                           Dpaa2NetInterface->DprcHandle);
  if (McFlibError != 0) {
    DPAA_ERROR_MSG ("Error calling dprc_close () for DPAA2 network interface 0x%p\n",
                    Dpaa2NetInterface);
  }

  Dpaa2NetInterface->DprcHandle = 0;
  McFlibError = dprc_destroy_container (ParentDprcMcIo, MC_CMD_NO_FLAGS,
                                       ParentDprcHandle,
                                       Dpaa2NetInterface->DprcId);
  if (McFlibError != 0) {
    DPAA_ERROR_MSG ("Error calling dprc_destroy_container () for DPAA2 network interface 0x%p\n",
                    Dpaa2NetInterface);
  }

  DPAA_DEBUG_MSG ("Destroyed dprc.%u\n", Dpaa2NetInterface->DprcId);
}


STATIC
EFI_STATUS
Dpaa2McCreateDpbp (
  DPAA2_MC_IO *DprcMcIo,
  DPAA2_NETWORK_INTERFACE *Dpaa2NetInterface
  )
{
  EFI_STATUS Status;
  INT32 McFlibError;
  struct dpbp_cfg DpbpCfg;
  struct dpbp_attr DpbpAttr;

  ASSERT (Dpaa2NetInterface->DpbpHandle == 0);

  /*
   * Create DPBP object:
   */
  DpbpCfg.options = 512; /* TODO: what is is magic number? */
  McFlibError = dpbp_create (DprcMcIo, MC_CMD_NO_FLAGS, &DpbpCfg,
          &Dpaa2NetInterface->DpbpHandle);
  if (McFlibError != 0) {
    DPAA_ERROR_MSG ("Error calling dpbp_create () for 0x%p\n",
                    Dpaa2NetInterface);
    return EFI_DEVICE_ERROR;
  }

  ZeroMem (&DpbpAttr, sizeof (DpbpAttr));
  McFlibError = dpbp_get_attributes (DprcMcIo, MC_CMD_NO_FLAGS,
            Dpaa2NetInterface->DpbpHandle,
            &DpbpAttr);
  if (McFlibError != 0) {
    DPAA_ERROR_MSG ("Error calling dpbp_get_attributes () for 0x%p\n",
                    Dpaa2NetInterface);
    Status = EFI_DEVICE_ERROR;
    goto ErrorExitDestroyDpbp;
  }

  if (DpbpAttr.version.major != DPBP_VER_MAJOR ||
      DpbpAttr.version.minor != DPBP_VER_MINOR) {
    DPAA_WARN_MSG (
      "WARNING: DPBP version mismatch (version supported: %u.%u, version found: %u.%u)\n",
      DPBP_VER_MAJOR, DPBP_VER_MINOR,
      DpbpAttr.version.major, DpbpAttr.version.minor);
  }

  Dpaa2NetInterface->DpbpId = DpbpAttr.id;
  Dpaa2NetInterface->DpbpHwBufferPoolId = DpbpAttr.bpid;
  DPAA_DEBUG_MSG ("Created dpbp.%u (HW buffer pool id %u) in the MC\n",
                  DpbpAttr.id, DpbpAttr.bpid);
  return EFI_SUCCESS;

ErrorExitDestroyDpbp:
  (VOID)dpbp_destroy (DprcMcIo, MC_CMD_NO_FLAGS, Dpaa2NetInterface->DpbpHandle);
  Dpaa2NetInterface->DpbpHandle = 0;
  return Status;
}


STATIC
VOID
Dpaa2McDestroyDpbp (
  DPAA2_MC_IO *DprcMcIo,
  DPAA2_NETWORK_INTERFACE *Dpaa2NetInterface
  )
{
  INT32 McFlibError;

  ASSERT (Dpaa2NetInterface->DpbpHandle != 0);
  McFlibError = dpbp_destroy (DprcMcIo, MC_CMD_NO_FLAGS,
                             Dpaa2NetInterface->DpbpHandle);
  if (McFlibError != 0) {
    DPAA_ERROR_MSG ("Error calling dpbp_destroy () for DPAA2 network interface 0x%p\n",
                    Dpaa2NetInterface);
  }

  Dpaa2NetInterface->DpbpHandle = 0;
  DPAA_DEBUG_MSG ("Destroyed dpbp.%u\n", Dpaa2NetInterface->DpbpId);
}


STATIC
EFI_STATUS
Dpaa2McCreateDpio (
  DPAA2_MC_IO *DprcMcIo,
  DPAA2_NETWORK_INTERFACE *Dpaa2NetInterface
  )
{
  EFI_STATUS Status;
  INT32 McFlibError;
  struct dpio_attr DpioAttr;
  struct dpio_cfg DpioCfg;

  ASSERT (Dpaa2NetInterface->DpioHandle == 0);
  DpioCfg.channel_mode = DPIO_LOCAL_CHANNEL;
  DpioCfg.num_priorities = 8;
  McFlibError = dpio_create (DprcMcIo, MC_CMD_NO_FLAGS, &DpioCfg,
                            &Dpaa2NetInterface->DpioHandle);
  if (McFlibError != 0) {
    DPAA_ERROR_MSG ("Error calling dpio_create () for 0x%p\n",
                    Dpaa2NetInterface);
    return EFI_DEVICE_ERROR;
  }

  ZeroMem (&DpioAttr, sizeof (DpioAttr));
  McFlibError = dpio_get_attributes (DprcMcIo, MC_CMD_NO_FLAGS,
            Dpaa2NetInterface->DpioHandle,
                                    &DpioAttr);
  if (McFlibError != 0) {
    DPAA_ERROR_MSG ("Error calling dpio_get_attributes () for 0x%p\n",
                    Dpaa2NetInterface);
    Status = EFI_DEVICE_ERROR;
    goto ErrorExitDestroyDpio;
  }

  if (DpioAttr.version.major != DPIO_VER_MAJOR ||
      DpioAttr.version.minor != DPIO_VER_MINOR) {
    DPAA_WARN_MSG (
      "WARNING: DPIO version mismatch (version supported: %u.%u, version found: %u.%u)\n",
      DPIO_VER_MAJOR, DPIO_VER_MINOR,
      DpioAttr.version.major, DpioAttr.version.minor);
  }

  Dpaa2NetInterface->DpioId = DpioAttr.id;

  McFlibError = dpio_enable (DprcMcIo, MC_CMD_NO_FLAGS, Dpaa2NetInterface->DpioHandle);
  if (McFlibError != 0) {
    DPAA_ERROR_MSG ("Error calling dpio_enable () for 0x%p\n",
                    Dpaa2NetInterface);
    Status = EFI_DEVICE_ERROR;
    goto ErrorExitDestroyDpio;
  }

  Status = QbmanSwPortalInit (&Dpaa2NetInterface->DpioQbmanPortal,
                             DpioAttr.qbman_portal_ce_offset,
                             DpioAttr.qbman_portal_ci_offset);
  if (EFI_ERROR (Status)) {
    goto ErrorExitDisableDpio;
  }

  DPAA_DEBUG_MSG ("Created dpio.%u in the MC ("
                  "ce_offset=0x%llx, ci_offset=0x%llx, portalid=%d, prios=%d)\n",
                  DpioAttr.id,
                  DpioAttr.qbman_portal_ce_offset,
                  DpioAttr.qbman_portal_ci_offset,
                  DpioAttr.qbman_portal_id,
                  DpioAttr.num_priorities);

  return EFI_SUCCESS;

ErrorExitDisableDpio:
  (VOID)dpio_disable (DprcMcIo, MC_CMD_NO_FLAGS, Dpaa2NetInterface->DpioHandle);

ErrorExitDestroyDpio:
  (VOID)dpio_destroy (DprcMcIo, MC_CMD_NO_FLAGS, Dpaa2NetInterface->DpioHandle);
  Dpaa2NetInterface->DpioHandle = 0;
  return Status;
}


STATIC
VOID
Dpaa2McDestroyDpio (
  DPAA2_MC_IO *DprcMcIo,
  DPAA2_NETWORK_INTERFACE *Dpaa2NetInterface
  )
{
  INT32 McFlibError;

  ASSERT (Dpaa2NetInterface->DpioHandle != 0);

  McFlibError = dpio_disable (DprcMcIo, MC_CMD_NO_FLAGS, Dpaa2NetInterface->DpioHandle);
  if (McFlibError != 0) {
      DPAA_ERROR_MSG ("Error calling dpio_disable () for 0x%p\n",
                      Dpaa2NetInterface);
  }

  McFlibError = dpio_destroy (DprcMcIo, MC_CMD_NO_FLAGS, Dpaa2NetInterface->DpioHandle);
  if (McFlibError != 0) {
      DPAA_ERROR_MSG ("Error calling dpio_destroy () for 0x%p\n",
                      Dpaa2NetInterface);
  }

  Dpaa2NetInterface->DpioHandle = 0;
  DPAA_DEBUG_MSG ("Destroyed dpio.%u\n", Dpaa2NetInterface->DpioId);
}


STATIC
EFI_STATUS
Dpaa2McCreateDpni (
  DPAA2_MC_IO *DprcMcIo,
  DPAA2_NETWORK_INTERFACE *Dpaa2NetInterface
  )
{
  EFI_STATUS Status;
  INT32 McFlibError;
  struct dpni_attr DpniAttr;
  struct dpni_cfg DpniCfg;
  struct dpni_extended_cfg DpniExtendedCfg;

  ASSERT (sizeof (DpniExtendedCfg) <=
         sizeof (Dpaa2NetInterface->DpniExtCfgDmaBuffer));

  ASSERT (Dpaa2NetInterface->DpniHandle == 0);

  ZeroMem (&DpniExtendedCfg, sizeof (DpniExtendedCfg));
  McFlibError = dpni_prepare_extended_cfg (&DpniExtendedCfg,
                                          (UINT8 *)Dpaa2NetInterface->DpniExtCfgDmaBuffer);
  if (McFlibError != 0) {
    DPAA_ERROR_MSG ("Error calling dpni_prepare_extended_cfg () for 0x%p\n",
                    Dpaa2NetInterface);
    return EFI_DEVICE_ERROR;
  }

  /*
   * Flush data cache for area pointed to by Dpaa2NetInterface->DpniExtCfgDmaBuffer:
   */
  CleanDcacheRange ((UINT64)Dpaa2NetInterface->DpniExtCfgDmaBuffer,
                   (UINT64)Dpaa2NetInterface->DpniExtCfgDmaBuffer +
                   sizeof (Dpaa2NetInterface->DpniExtCfgDmaBuffer));

  ZeroMem (&DpniCfg, sizeof (DpniCfg));
  DpniCfg.adv.options = DPNI_OPT_UNICAST_FILTER |
                        DPNI_OPT_MULTICAST_FILTER;

  DpniCfg.adv.ext_cfg_iova = (UINT64)Dpaa2NetInterface->DpniExtCfgDmaBuffer;
  McFlibError = dpni_create (DprcMcIo, MC_CMD_NO_FLAGS, &DpniCfg,
                            &Dpaa2NetInterface->DpniHandle);
  if (McFlibError != 0) {
    DPAA_ERROR_MSG ("Error calling dpni_create () for 0x%p\n",
                    Dpaa2NetInterface);
    return EFI_DEVICE_ERROR;
  }

  ZeroMem (&DpniAttr, sizeof (DpniAttr));
  DpniAttr.ext_cfg_iova = (UINT64)Dpaa2NetInterface->DpniExtCfgDmaBuffer;
  McFlibError = dpni_get_attributes (DprcMcIo, MC_CMD_NO_FLAGS,
            Dpaa2NetInterface->DpniHandle,
                                    &DpniAttr);
  if (McFlibError != 0) {
    DPAA_ERROR_MSG ("Error calling dpni_get_attributes () for 0x%p\n",
                    Dpaa2NetInterface);
    Status = EFI_DEVICE_ERROR;
    goto ErrorExitDestroyDpni;
  }

  if (gDpaaDebugFlags & DPAA_DEBUG_EXTRA_CHECKS) {
    struct dpni_extended_cfg ExtractedDpniExtendedCfg;

    ASSERT (DpniAttr.ext_cfg_iova ==
           (UINT64)Dpaa2NetInterface->DpniExtCfgDmaBuffer);

    McFlibError = dpni_extract_extended_cfg (&ExtractedDpniExtendedCfg,
                                            (UINT8 *)DpniAttr.ext_cfg_iova);

    ASSERT (CompareMem (&ExtractedDpniExtendedCfg,
                      &DpniExtendedCfg,
                      sizeof (ExtractedDpniExtendedCfg)) == 0);
  }

  if (DpniAttr.version.major != DPNI_VER_MAJOR ||
      DpniAttr.version.minor != DPNI_VER_MINOR) {
    DPAA_WARN_MSG (
      "WARNING: DPNI version mismatch (version supported: %u.%u, version found: %u.%u)\n",
      DPNI_VER_MAJOR, DPNI_VER_MINOR,
      DpniAttr.version.major, DpniAttr.version.minor);
  }

  Dpaa2NetInterface->DpniId = DpniAttr.id;
  DPAA_DEBUG_MSG ("Created dpni.%u in the MC\n", DpniAttr.id);
  return EFI_SUCCESS;

ErrorExitDestroyDpni:
  (VOID)dpni_destroy (DprcMcIo, MC_CMD_NO_FLAGS, Dpaa2NetInterface->DpniHandle);
  return Status;
}


STATIC
VOID
Dpaa2McDestroyDpni (
  DPAA2_MC_IO *DprcMcIo,
  DPAA2_NETWORK_INTERFACE *Dpaa2NetInterface
  )
{
  INT32 McFlibError;

  ASSERT (Dpaa2NetInterface->DpniHandle != 0);

  McFlibError = dpni_destroy (DprcMcIo, MC_CMD_NO_FLAGS, Dpaa2NetInterface->DpniHandle);
  if (McFlibError != 0) {
      DPAA_ERROR_MSG ("Error calling dpni_destroy () for 0x%p\n",
                      Dpaa2NetInterface);
  }

  Dpaa2NetInterface->DpniHandle = 0;
  DPAA_DEBUG_MSG ("Destroyed dpni.%u\n", Dpaa2NetInterface->DpniId);
}


STATIC
EFI_STATUS
Dpaa2McCreateDpmac (
  DPAA2_MC_IO *DprcMcIo,
  DPAA2_NETWORK_INTERFACE *Dpaa2NetInterface,
  WRIOP_DPMAC_ID DpmacId
  )
{
  EFI_STATUS Status;
  INT32 McFlibError;
  struct dpmac_cfg DpmacCfg;
  struct dpmac_attr DpmacAttr;

  ASSERT (Dpaa2NetInterface->DpmacHandle == 0);
  ZeroMem (&DpmacCfg, sizeof (DpmacCfg));
  DpmacCfg.mac_id = DpmacId;
  McFlibError = dpmac_create (DprcMcIo, MC_CMD_NO_FLAGS, &DpmacCfg,
                             &Dpaa2NetInterface->DpmacHandle);
  if (McFlibError != 0) {
    DPAA_ERROR_MSG ("Error calling dpmac_create () for 0x%p\n",
                    Dpaa2NetInterface);
    return EFI_DEVICE_ERROR;
  }

  ZeroMem (&DpmacAttr, sizeof (DpmacAttr));
  McFlibError = dpmac_get_attributes (DprcMcIo, MC_CMD_NO_FLAGS,
             Dpaa2NetInterface->DpmacHandle,
                                     &DpmacAttr);
  if (McFlibError != 0) {
    DPAA_ERROR_MSG ("Error calling dpmac_get_attributes () for 0x%p\n",
                    Dpaa2NetInterface);
    Status = EFI_DEVICE_ERROR;
    goto ErrorExitDestroyDpmac;
  }

  if (DpmacAttr.version.major != DPMAC_VER_MAJOR ||
      DpmacAttr.version.minor != DPMAC_VER_MINOR) {
    DPAA_WARN_MSG (
      "WARNING: DPMAC version mismatch (version supported: %u.%u, version found: %u.%u)\n",
      DPMAC_VER_MAJOR, DPMAC_VER_MINOR,
      DpmacAttr.version.major, DpmacAttr.version.minor);
  }

  ASSERT (DpmacAttr.id == DpmacId);
  Dpaa2NetInterface->DpmacId = DpmacId;
  DPAA_DEBUG_MSG ("Created dpmac.%u in the MC\n", DpmacAttr.id);
  return EFI_SUCCESS;

ErrorExitDestroyDpmac:
  (VOID)dpmac_destroy (DprcMcIo, MC_CMD_NO_FLAGS, Dpaa2NetInterface->DpmacHandle);
  return Status;
}


STATIC
VOID
Dpaa2McDestroyDpmac (
  DPAA2_MC_IO *DprcMcIo,
  DPAA2_NETWORK_INTERFACE *Dpaa2NetInterface
  )
{
  INT32 McFlibError;

  ASSERT (Dpaa2NetInterface->DpmacHandle != 0);

  McFlibError = dpmac_destroy (DprcMcIo, MC_CMD_NO_FLAGS, Dpaa2NetInterface->DpmacHandle);
  if (McFlibError != 0) {
      DPAA_ERROR_MSG ("Error calling dpmac_destroy () for 0x%p\n",
                      Dpaa2NetInterface);
  }

  Dpaa2NetInterface->DpmacHandle = 0;
  DPAA_DEBUG_MSG ("Destroyed dpmac.%u\n", Dpaa2NetInterface->DpmacId);
}


/**
   Creates a DPAA2 network interface in the MC

   @param Dpaa2NetInterface Pointer to DPAA2 network interface control block

   @retval EFI_SUCCESS, on success
   @retval error code, on failure
 **/
EFI_STATUS
Dpaa2McCreateNetworkInterface (
  DPAA2_NETWORK_INTERFACE *Dpaa2NetInterface
  )
{
  EFI_STATUS Status;
  DPAA2_MANAGEMENT_COMPLEX *Mc;

  Mc = &gManagementComplex;
  ASSERT (Dpaa2NetInterface->Signature == DPAA2_NETWORK_INTERFACE_SIGNATURE);
  ASSERT (!Dpaa2NetInterface->CreatedInMc);

  /*
   * Create child DPRC object:
   */
  Status = Dpaa2McCreateDprc (&Mc->RootDprcMcIo, Mc->RootDprcHandle, Dpaa2NetInterface);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  /*
   * Create DPBP object:
   */
  Status = Dpaa2McCreateDpbp (&Dpaa2NetInterface->DprcMcIo, Dpaa2NetInterface);
  if (EFI_ERROR (Status)) {
    goto ErrorExitDestroyDprc;
  }

  /*
   * Create DPIO object:
   */
  Status = Dpaa2McCreateDpio (&Dpaa2NetInterface->DprcMcIo, Dpaa2NetInterface);
  if (EFI_ERROR (Status)) {
    goto ErrorExitDestroyDpbp;
  }

  /*
   * Create DPNI object:
   */
  Status = Dpaa2McCreateDpni (&Dpaa2NetInterface->DprcMcIo, Dpaa2NetInterface);
  if (EFI_ERROR (Status)) {
    goto ErrorExitDestroyDpio;
  }

  Dpaa2NetInterface->CreatedInMc = TRUE;
  return EFI_SUCCESS;

ErrorExitDestroyDpio:
  Dpaa2McDestroyDpio (&Dpaa2NetInterface->DprcMcIo, Dpaa2NetInterface);

ErrorExitDestroyDpbp:
  Dpaa2McDestroyDpbp (&Dpaa2NetInterface->DprcMcIo, Dpaa2NetInterface);

ErrorExitDestroyDprc:
  Dpaa2McDestroyDprc (&Mc->RootDprcMcIo, Mc->RootDprcHandle, Dpaa2NetInterface);

  return Status;
}


/**
   Destroys a DPAA2 network interface in the MC

   @param Dpaa2NetInterface Pointer to DPAA2 network interface control block

 **/
VOID
Dpaa2McDestroyNetworkInterface (
  DPAA2_NETWORK_INTERFACE *Dpaa2NetInterface
  )
{
  DPAA2_MANAGEMENT_COMPLEX *Mc;

  Mc = &gManagementComplex;
  ASSERT (Dpaa2NetInterface->Signature == DPAA2_NETWORK_INTERFACE_SIGNATURE);
  ASSERT (Dpaa2NetInterface->CreatedInMc);
  ASSERT (!Dpaa2NetInterface->StartedInMc);

  Dpaa2NetInterface->CreatedInMc = FALSE;
  Dpaa2McDestroyDpni (&Dpaa2NetInterface->DprcMcIo, Dpaa2NetInterface);
  Dpaa2McDestroyDpio (&Dpaa2NetInterface->DprcMcIo, Dpaa2NetInterface);
  Dpaa2McDestroyDpbp (&Dpaa2NetInterface->DprcMcIo, Dpaa2NetInterface);
  Dpaa2McDestroyDprc (&Mc->RootDprcMcIo, Mc->RootDprcHandle, Dpaa2NetInterface);
}


/**
 * Bind a DPAA2 Ethernet interface's DPMAC to the DPAA2 Ethernet interface's DPNI
 */
STATIC
EFI_STATUS
Dpaa2McBindDpmacToDpni (
  DPAA2_MC_IO *DprcMcIo,
  UINT16 DprcHandle,
  DPAA2_NETWORK_INTERFACE *Dpaa2NetInterface
  )
{
  INT32 McFlibError;
  struct dprc_connection_cfg DprcConnectionCfg = {
    .committed_rate = 0,    /* best effort */
    .max_rate = 0
  };

  struct dprc_endpoint *DpmacEndpoint = &Dpaa2NetInterface->DpmacEndpoint;
  struct dprc_endpoint *DpniEndpoint = &Dpaa2NetInterface->DpniEndpoint;
  struct dprc_endpoint DebugEndpoint;
  int State = 0;

  ZeroMem (DpmacEndpoint, sizeof (struct dprc_endpoint));
  AsciiStrCpy (DpmacEndpoint->type, "dpmac");
  DpmacEndpoint->id = Dpaa2NetInterface->DpmacId;

  ZeroMem (DpniEndpoint, sizeof (struct dprc_endpoint));
  AsciiStrCpy (DpniEndpoint->type, "dpni");
  DpniEndpoint->id = Dpaa2NetInterface->DpniId;

  McFlibError = dprc_connect (DprcMcIo,
                             MC_CMD_NO_FLAGS,
           DprcHandle,
           DpmacEndpoint,
           DpniEndpoint,
           &DprcConnectionCfg);
  if (McFlibError != 0) {
      DPAA_ERROR_MSG ("Error calling dprc_connect () for 0x%p\n",
                      Dpaa2NetInterface);
      return EFI_DEVICE_ERROR;
  }

  if (gDpaaDebugFlags & DPAA_DEBUG_EXTRA_CHECKS) {
    ZeroMem (&DebugEndpoint, sizeof (struct dprc_endpoint));
    McFlibError = dprc_get_connection (DprcMcIo, MC_CMD_NO_FLAGS,
                                      DprcHandle, DpniEndpoint,
                                      &DebugEndpoint, &State);
    ASSERT (McFlibError == 0);
    ASSERT (AsciiStrCmp (DebugEndpoint.type, "dpmac") == 0);
    ASSERT (DebugEndpoint.id == DpmacEndpoint->id);

    ZeroMem (&DebugEndpoint, sizeof (struct dprc_endpoint));
    McFlibError = dprc_get_connection (DprcMcIo, MC_CMD_NO_FLAGS,
                                     DprcHandle, DpmacEndpoint,
                                     &DebugEndpoint, &State);
    ASSERT (McFlibError == 0);
    ASSERT (AsciiStrCmp (DebugEndpoint.type, "dpni") == 0);
    ASSERT (DebugEndpoint.id == DpniEndpoint->id);
  }

  return EFI_SUCCESS;
}


/**
 * Setup buffer Rx/Tx layouts for a DPNI
 */
STATIC
EFI_STATUS
Dpaa2McSetDpniRxTxBufferLayouts (
  DPAA2_MC_IO *DprcMcIo,
  DPAA2_NETWORK_INTERFACE *Dpaa2NetInterface
  )
{
  INT32 McFlibError;
  UINT16 DpniHandle;
  struct dpni_buffer_layout DpniBufLayout;

  DpniHandle = Dpaa2NetInterface->DpniHandle;
  ASSERT (DpniHandle != 0);

  Dpaa2NetInterface->FramePrivateDataSize = DPAA2_ETH_FRAME_SW_ANNOTATION_SIZE;

   /*
   * Configure Rx buffers layout:
   */
  ZeroMem (&DpniBufLayout, sizeof (DpniBufLayout));
  DpniBufLayout.options = DPNI_BUF_LAYOUT_OPT_PARSER_RESULT |
                          DPNI_BUF_LAYOUT_OPT_FRAME_STATUS |
                          DPNI_BUF_LAYOUT_OPT_PRIVATE_DATA_SIZE |
                          DPNI_BUF_LAYOUT_OPT_DATA_ALIGN;
  DpniBufLayout.pass_parser_result = TRUE;
  DpniBufLayout.pass_frame_status = TRUE;
  DpniBufLayout.private_data_size = Dpaa2NetInterface->FramePrivateDataSize;
  DpniBufLayout.data_align = DPAA2_ETH_FRAME_BUFFER_ALIGN;
  McFlibError = dpni_set_rx_buffer_layout (DprcMcIo, MC_CMD_NO_FLAGS,
            DpniHandle, &DpniBufLayout);
  if (McFlibError != 0) {
    DPAA_ERROR_MSG ("Error calling dpni_set_rx_buffer_layout () for 0x%p\n",
                    Dpaa2NetInterface);
    return EFI_DEVICE_ERROR;
  }

  /*
   * Configure Tx buffers layout:
   */
  DpniBufLayout.options &= ~(DPNI_BUF_LAYOUT_OPT_DATA_ALIGN |
           DPNI_BUF_LAYOUT_OPT_PARSER_RESULT);
  McFlibError = dpni_set_tx_buffer_layout (DprcMcIo, MC_CMD_NO_FLAGS,
            DpniHandle, &DpniBufLayout);
  if (McFlibError != 0) {
    DPAA_ERROR_MSG ("Error calling dpni_set_tx_buffer_layout () for 0x%p\n",
                    Dpaa2NetInterface);
    return EFI_DEVICE_ERROR;
  }

  /*
   * Configure Tx confirmations layout:
   */
  DpniBufLayout.options &= ~DPNI_BUF_LAYOUT_OPT_PRIVATE_DATA_SIZE;
  McFlibError = dpni_set_tx_conf_buffer_layout (DprcMcIo, MC_CMD_NO_FLAGS,
                                               DpniHandle, &DpniBufLayout);
  if (McFlibError != 0) {
    DPAA_ERROR_MSG ("Error calling dpni_set_tx_conf_buffer_layout () for 0x%p\n",
                    Dpaa2NetInterface);
    return EFI_DEVICE_ERROR;
  }

  /*
   * Retrieve the minimum required Tx data offset:
   */
  McFlibError = dpni_get_tx_data_offset (DprcMcIo, MC_CMD_NO_FLAGS,
                                        DpniHandle,
                &Dpaa2NetInterface->TxDataOffset);
  if (McFlibError != 0) {
    DPAA_ERROR_MSG ("Error calling dpni_get_tx_data_offset () for 0x%p\n",
                    Dpaa2NetInterface);
    return EFI_DEVICE_ERROR;
  }

  if (Dpaa2NetInterface->TxDataOffset % 64 != 0) {
    DPAA_WARN_MSG ("WARNING: Tx data offset is not a multiple of 64 for dpni.%u\n",
                   Dpaa2NetInterface->DpniId);
  }

  /*
   * Adjust Tx data offset to account for the software annotation space:
   */
  Dpaa2NetInterface->TxDataOffset += DPAA2_ETH_FRAME_SW_ANNOTATION_SIZE;
  return EFI_SUCCESS;
}


STATIC
UINT8
Dpaa2QbmanBufferPoolAdd7 (
  DPAA2_QBMAN_PORTAL *DpioQbmanPortal,
  UINT16 HwBufferPoolId
  )
{
  UINT64 Buffers[7];
  VOID *DataBuffer;
  UINT32 I;
  QBMAN_RELEASE_DESC ReleaseDesc;
  BOOLEAN ReleaseOk;

  for (I = 0; I < 7; I++) {
    DataBuffer = AllocatePages (EFI_SIZE_TO_PAGES (DPAA2_ETH_RX_FRAME_BUFFER_SIZE));
    if (DataBuffer == NULL) {
      DPAA_ERROR_MSG ("Could not allocate DPAA2 Ethernet frame buffer\n");
      goto ErrorAlloc;
    }

    ASSERT ((UINT64)DataBuffer % DPAA2_ETH_FRAME_BUFFER_ALIGN == 0);
    ZeroMem (DataBuffer, DPAA2_ETH_RX_FRAME_BUFFER_SIZE);
    CleanDcacheRange ((UINT64)DataBuffer,
                     (UINT64)DataBuffer + DPAA2_ETH_RX_FRAME_BUFFER_SIZE);

    Buffers[I] = (UINT64)DataBuffer;
  }

ReleaseBuffers:
  /*
   * In case the portal is busy, retry until successful.
   * This function is guaranteed to succeed in a reasonable amount
   * of time.
   */
  do {
    MicroSecondDelay (1000);
    QbmanReleaseDescClear (&ReleaseDesc);
    QbmanReleaseDescSetBpid (&ReleaseDesc, HwBufferPoolId);
    ReleaseOk = QbmanSwPortalReleaseBuffers (DpioQbmanPortal,
                                            &ReleaseDesc, Buffers, I);
  } while (!ReleaseOk);

  DPAA_DEBUG_MSG ("Added %u buffers to QBMAN buffer pool %u\n",
                  I, HwBufferPoolId);
  return I;

ErrorAlloc:
  if (I != 0) {
    goto ReleaseBuffers;
  }

  return 0;
}

/**
 * Seed the given buffer pool
 */
STATIC
EFI_STATUS
Dpaa2QbmanSeedBufferPool (
  DPAA2_QBMAN_PORTAL *DpioQbmanPortal,
  UINT16 HwBufferPoolId
  )
{
  UINT32 I;
  UINT8 BufCount;

  for (I = 0; I < DPAA2_ETH_NUM_FRAME_BUFFERS; I += 7) {
    BufCount = Dpaa2QbmanBufferPoolAdd7 (DpioQbmanPortal,
                                        HwBufferPoolId);
    if (BufCount < 7) {
      break;
    }
  }

  if (I == 0 && BufCount == 0) {
    return EFI_OUT_OF_RESOURCES;
  }

  return EFI_SUCCESS;
}


/**
 * Enable and seed data path buffer pool (DPBP)
 */
STATIC
EFI_STATUS
Dpaa2McInitializeDpbp (
  DPAA2_MC_IO *DprcMcIo,
  DPAA2_NETWORK_INTERFACE *Dpaa2NetInterface
  )
{
  EFI_STATUS Status;
  INT32 McFlibError;
  UINT16 DpbpHandle = Dpaa2NetInterface->DpbpHandle;

  ASSERT (DpbpHandle != 0);
  McFlibError = dpbp_enable (DprcMcIo, MC_CMD_NO_FLAGS, DpbpHandle);
  if (McFlibError != 0) {
    DPAA_ERROR_MSG ("Error calling dpbp_enable () for 0x%p\n",
                    Dpaa2NetInterface);
    return EFI_DEVICE_ERROR;
  }

  Status = Dpaa2QbmanSeedBufferPool (&Dpaa2NetInterface->DpioQbmanPortal,
                                    Dpaa2NetInterface->DpbpHwBufferPoolId);
  if (EFI_ERROR (Status)) {
    goto ErrorExitDisableDpbp;
  }

  return EFI_SUCCESS;

ErrorExitDisableDpbp:
  dpbp_disable (DprcMcIo, MC_CMD_NO_FLAGS, DpbpHandle);
  return Status;
}


STATIC
void
Dpaa2DrainQbmanBuffers (
  DPAA2_QBMAN_PORTAL *DpioQbmanPortal,
  UINT16 HwBufferPoolId,
  UINT32 NumBuffers
  )
{
  UINT64 Buffers[7];
  UINT32 TotalAcquiredBuffers;
  UINT32 NumAcquiredBuffers;
  UINT32 NumRemainingBuffers;
  UINT32 I, J;
  UINT32 NumDuplicateBuffers;
  VOID *DataBuffer;

  DPAA_DEBUG_MSG ("Removing %u buffers from QBMAN buffer pool %u ...\n",
                  NumBuffers, HwBufferPoolId);
  ASSERT (NumBuffers != 0 && NumBuffers <= 7);
  TotalAcquiredBuffers = 0;
  for (NumRemainingBuffers = NumBuffers;
       NumRemainingBuffers != 0;
       NumRemainingBuffers -= NumAcquiredBuffers) {
    NumAcquiredBuffers = QbmanSwPortalAcquireBuffers (DpioQbmanPortal,
                       HwBufferPoolId,
                       Buffers,
                                                     NumRemainingBuffers);
    if (NumAcquiredBuffers == 0) {
      DPAA_ERROR_MSG ("Could not acquire %u buffers from QBman buffer pool %u\n",
                      NumRemainingBuffers, HwBufferPoolId);
      return;
    }

    ASSERT (NumAcquiredBuffers <= NumRemainingBuffers);
    for (I = 0, NumDuplicateBuffers = 0; I < NumAcquiredBuffers; I++) {
    for (J = 0; J < I; J++) {
      if (Buffers[I] == Buffers[J]) {
        NumDuplicateBuffers++;
          break;
        }
    }
    if (J < I) {
      DPAA_WARN_MSG ("Duplicate buffer addresses received from QBman buffer pool %u\n",
        HwBufferPoolId);
      continue;
    }
      DataBuffer = (VOID *)(Buffers[I]);
      ASSERT ((UINT64)DataBuffer % DPAA2_ETH_FRAME_BUFFER_ALIGN == 0);
      FreePages (DataBuffer, EFI_SIZE_TO_PAGES (DPAA2_ETH_RX_FRAME_BUFFER_SIZE));
    }

    NumAcquiredBuffers -=  NumDuplicateBuffers;
    TotalAcquiredBuffers += NumAcquiredBuffers;
  }

  ASSERT (TotalAcquiredBuffers == NumBuffers);
}


STATIC
VOID
Dpaa2DrainQbmanBufferPool (
  DPAA2_QBMAN_PORTAL *DpioQbmanPortal,
  UINT16 HwBufferPoolId
  )
{
  UINT32 I;

  for (I = 0; I < DPAA2_ETH_NUM_FRAME_BUFFERS; I += 7) {
    Dpaa2DrainQbmanBuffers (DpioQbmanPortal, HwBufferPoolId, 7);
  }
}


/**
 * Disable and tear down data path buffer pool (DPBP)
 */
STATIC
VOID
Dpaa2McShutdownDpbp (
  DPAA2_MC_IO *DprcMcIo,
  DPAA2_NETWORK_INTERFACE *Dpaa2NetInterface
  )
{
  Dpaa2DrainQbmanBufferPool (&Dpaa2NetInterface->DpioQbmanPortal,
                            Dpaa2NetInterface->DpbpHwBufferPoolId);

  dpbp_disable (DprcMcIo, MC_CMD_NO_FLAGS, Dpaa2NetInterface->DpbpHandle);
  dpbp_reset (DprcMcIo, MC_CMD_NO_FLAGS, Dpaa2NetInterface->DpbpHandle);
}


STATIC
EFI_STATUS
Dpaa2McSetDpniBufferPools (
  DPAA2_MC_IO *DprcMcIo,
  DPAA2_NETWORK_INTERFACE *Dpaa2NetInterface
  )
{
  INT32 McFlibError;
  struct dpni_pools_cfg PoolsParams;
  struct dpni_tx_flow_cfg TxFlowCfg;
  struct dpni_tx_conf_cfg TxConfCfg;
  struct dpni_tx_conf_attr TxConfAttr;

  ZeroMem (&PoolsParams, sizeof (PoolsParams));
  PoolsParams.num_dpbp = 1;
  PoolsParams.pools[0].dpbp_id = (uint16_t)Dpaa2NetInterface->DpbpId;
  PoolsParams.pools[0].buffer_size = DPAA2_ETH_RX_FRAME_BUFFER_SIZE;
  McFlibError = dpni_set_pools (DprcMcIo, MC_CMD_NO_FLAGS,
             Dpaa2NetInterface->DpniHandle,
                               &PoolsParams);
  if (McFlibError != 0) {
    DPAA_ERROR_MSG ("Error calling dpni_set_pools () for 0x%p\n",
                    Dpaa2NetInterface);
    return EFI_DEVICE_ERROR;
  }

  Dpaa2NetInterface->TxFlowId = DPNI_NEW_FLOW_ID;
  ZeroMem (&TxFlowCfg, sizeof (TxFlowCfg));
  TxFlowCfg.options = DPNI_TX_FLOW_OPT_TX_CONF_ERROR;
  TxFlowCfg.use_common_tx_conf_queue = 0;
  McFlibError = dpni_set_tx_flow (DprcMcIo, MC_CMD_NO_FLAGS,
               Dpaa2NetInterface->DpniHandle,
                                 &Dpaa2NetInterface->TxFlowId,
               &TxFlowCfg);
  if (McFlibError != 0) {
    DPAA_ERROR_MSG ("Error calling dpni_set_tx_flow () for 0x%p\n",
                    Dpaa2NetInterface);
    return EFI_DEVICE_ERROR;
  }

  /*
   * Set Tx confirmations and error configuration:
   */
  ZeroMem (&TxConfCfg, sizeof (TxConfCfg));
  TxConfCfg.errors_only = FALSE;
  TxConfCfg.queue_cfg.options = DPNI_QUEUE_OPT_USER_CTX |
        DPNI_QUEUE_OPT_DEST;
  TxConfCfg.queue_cfg.user_ctx = (UINT64)Dpaa2NetInterface;
  TxConfCfg.queue_cfg.dest_cfg.dest_type = DPNI_DEST_NONE;
  McFlibError = dpni_set_tx_conf (DprcMcIo, MC_CMD_NO_FLAGS,
               Dpaa2NetInterface->DpniHandle,
               Dpaa2NetInterface->TxFlowId,
                                 &TxConfCfg);
  if (McFlibError != 0) {
    DPAA_ERROR_MSG ("Error calling dpni_set_tx_conf () for 0x%p\n",
                    Dpaa2NetInterface);
    return EFI_DEVICE_ERROR;
  }

  McFlibError = dpni_get_tx_conf (DprcMcIo, MC_CMD_NO_FLAGS,
                                 Dpaa2NetInterface->DpniHandle,
               Dpaa2NetInterface->TxFlowId,
               &TxConfAttr);
  if (McFlibError != 0) {
    DPAA_ERROR_MSG ("Error calling dpni_get_tx_conf () for 0x%p\n",
                    Dpaa2NetInterface);
    return EFI_DEVICE_ERROR;
  }

  Dpaa2NetInterface->TxConfirmationsQueueId = TxConfAttr.queue_attr.fqid;
  return EFI_SUCCESS;
}


STATIC
EFI_STATUS
Dpaa2McInitializeDpni (
  DPAA2_MC_IO *DprcMcIo,
  DPAA2_NETWORK_INTERFACE *Dpaa2NetInterface,
  EFI_MAC_ADDRESS *MacAddr
  )
{
  STATIC const EFI_MAC_ADDRESS BroadcastMacAddr = {
    .Addr =  { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff }
  };

  INT32 McFlibError;

  /*
   * Add unicast MAC address:
   */
  McFlibError = dpni_add_mac_addr (DprcMcIo, MC_CMD_NO_FLAGS,
          Dpaa2NetInterface->DpniHandle,
                                  MacAddr->Addr);
  if (McFlibError != 0) {
    DPAA_ERROR_MSG ("Error calling dpni_add_mac_addr () for 0x%p\n",
                    Dpaa2NetInterface);
    return EFI_DEVICE_ERROR;
  }

  /*
   * Explicitly add the broadcast address to the MAC filtering table;
   * the MC won't do that for us.
   */
  McFlibError = dpni_add_mac_addr (DprcMcIo, MC_CMD_NO_FLAGS,
          Dpaa2NetInterface->DpniHandle,
                                  BroadcastMacAddr.Addr);
  if (McFlibError != 0) {
    DPAA_ERROR_MSG ("Error calling dpni_add_mac_addr () for 0x%p\n",
                    Dpaa2NetInterface);
    return EFI_DEVICE_ERROR;
  }

  McFlibError = dpni_enable (DprcMcIo, MC_CMD_NO_FLAGS,
                            Dpaa2NetInterface->DpniHandle);
  if (McFlibError != 0) {
    DPAA_ERROR_MSG ("Error calling dpni_enable () for 0x%p\n",
                    Dpaa2NetInterface);
    return EFI_DEVICE_ERROR;
  }

  return EFI_SUCCESS;
}


STATIC
EFI_STATUS
Dpaa2McInitializeDpmac (
  DPAA2_MC_IO *DprcMcIo,
  DPAA2_NETWORK_INTERFACE *Dpaa2NetInterface,
  WRIOP_DPMAC *WriopDpmac
  )
{
  INT32 McFlibError;
  struct dpmac_link_state DpmacLinkState;

  ASSERT (Dpaa2NetInterface->DpmacId == WriopDpmac->Id);
  ASSERT (Dpaa2NetInterface->DpmacHandle != 0);

  ZeroMem (&DpmacLinkState, sizeof (DpmacLinkState));
  DpmacLinkState.rate = WriopDpmac->Phy.Speed;
  if (WriopDpmac->Phy.AutoNegotiation) {
    DpmacLinkState.options |= DPMAC_LINK_OPT_AUTONEG;
  } else {
    DpmacLinkState.options &= ~DPMAC_LINK_OPT_AUTONEG;
  }

  if (!WriopDpmac->Phy.FullDuplex) {
    DpmacLinkState.options |= DPMAC_LINK_OPT_HALF_DUPLEX;
  }

  DpmacLinkState.up = WriopDpmac->Phy.LinkUp;

  McFlibError = dpmac_set_link_state (DprcMcIo, MC_CMD_NO_FLAGS,
                                     Dpaa2NetInterface->DpmacHandle,
                                     &DpmacLinkState);
  if (McFlibError != 0) {
    DPAA_ERROR_MSG ("Error calling dpmac_set_link_state () for 0x%p\n",
                    Dpaa2NetInterface);
    return EFI_DEVICE_ERROR;
  }

  return EFI_SUCCESS;
}


/**
   Check the status of  the physical link of a DPAA2 network interface

   @param Dpaa2NetInterface Pointer to DPAA2 network interface control block

   @retval TRUE, link is up
   @retval FALSE, link is down
 **/
BOOLEAN
Dpaa2McCheckDpniLink (
  DPAA2_NETWORK_INTERFACE *Dpaa2NetInterface)
{
  INT32 McFlibError;
  struct dpni_link_state DpniLinkState;

  ASSERT (Dpaa2NetInterface->Signature == DPAA2_NETWORK_INTERFACE_SIGNATURE);
  ASSERT (Dpaa2NetInterface->StartedInMc);
  ASSERT (Dpaa2NetInterface->DpniHandle != 0);

  ZeroMem (&DpniLinkState, sizeof (DpniLinkState));
  McFlibError = dpni_get_link_state (&Dpaa2NetInterface->DprcMcIo,
                                    MC_CMD_NO_FLAGS,
            Dpaa2NetInterface->DpniHandle,
                                    &DpniLinkState);
  if (McFlibError != 0) {
    DPAA_ERROR_MSG ("Error calling dpni_get_link_state () for 0x%p\n",
                    Dpaa2NetInterface);
    return FALSE;
  }

  return DpniLinkState.up == 1;
}


STATIC
EFI_STATUS
Dpaa2McGetDpniQueues (
  DPAA2_MC_IO *DprcMcIo,
  DPAA2_NETWORK_INTERFACE *Dpaa2NetInterface
  )
{
  EFI_STATUS Status;
  INT32 McFlibError;
  struct dpni_queue_attr RxQueueAttr;

  ZeroMem (&RxQueueAttr, sizeof (RxQueueAttr));
  McFlibError = dpni_get_rx_flow (DprcMcIo, MC_CMD_NO_FLAGS,
               Dpaa2NetInterface->DpniHandle,
                                 0, 0, &RxQueueAttr);
  if (McFlibError != 0) {
    DPAA_ERROR_MSG ("Error calling dpni_get_rx_flow () for 0x%p\n",
                    Dpaa2NetInterface);
    Status = EFI_DEVICE_ERROR;
    goto ErrorExitDisableDpni;
  }

  Dpaa2NetInterface->RxDefaultFrameQueueId = RxQueueAttr.fqid;

  McFlibError = dpni_get_qdid (DprcMcIo, MC_CMD_NO_FLAGS,
            Dpaa2NetInterface->DpniHandle,
            &Dpaa2NetInterface->TxQueueDestinationId);
  if (McFlibError) {
    DPAA_ERROR_MSG ("Error calling dpni_get_qdid () for 0x%p\n",
                    Dpaa2NetInterface);
    Status = EFI_DEVICE_ERROR;
    goto ErrorExitDisableDpni;
  }

  return EFI_SUCCESS;

ErrorExitDisableDpni:
  (VOID)dpni_disable (DprcMcIo, MC_CMD_NO_FLAGS, Dpaa2NetInterface->DpniHandle);
  return Status;
}

STATIC
EFI_STATUS
Dpaa2McSetDpniMtu (
  DPAA2_MC_IO *DprcMcIo,
  DPAA2_NETWORK_INTERFACE *Dpaa2NetInterface,
  UINT32 *Mtu
  )
{
  INT32 McFlibError;
  McFlibError = dpni_get_max_frame_length (DprcMcIo, MC_CMD_NO_FLAGS,
                            Dpaa2NetInterface->DpniHandle,
                            (UINT16*)Mtu);
  *Mtu -= DPAA2_ETHERNET_HEADER_SIZE;
  if (McFlibError) {
    DPAA_ERROR_MSG ("Error calling dpni_get_mtu () for 0x%p\n",
                    Dpaa2NetInterface);
    return EFI_DEVICE_ERROR;
  }
  return EFI_SUCCESS;
}

/**
   Initialize a DPAA2 network interface in the MC

   @param Dpaa2NetInterface Pointer to DPAA2 network interface control block
   @param MacAddr           Pointer to MAC address
   @param WriopDpmac        Pointer to WRIOP DMAC object

   @retval EFI_SUCCESS, on success
   @retval error code, on failure
 **/
EFI_STATUS
Dpaa2McInitializeNetworkInterface (
  DPAA2_NETWORK_INTERFACE *Dpaa2NetInterface,
  EFI_MAC_ADDRESS *MacAddr,
  UINT32 *Mtu,
  WRIOP_DPMAC *WriopDpmac
  )
{
  EFI_STATUS Status;

  ASSERT (Dpaa2NetInterface->Signature == DPAA2_NETWORK_INTERFACE_SIGNATURE);
  ASSERT (Dpaa2NetInterface->CreatedInMc);
  ASSERT (!Dpaa2NetInterface->StartedInMc);
  ASSERT (Dpaa2NetInterface->DpniHandle != 0);
  ASSERT (Dpaa2NetInterface->DpbpHandle != 0);
  ASSERT (Dpaa2NetInterface->DprcHandle != 0);

  /*
   * Create DPMAC object:
   */
  Status = Dpaa2McCreateDpmac (&Dpaa2NetInterface->DprcMcIo, Dpaa2NetInterface,
                              WriopDpmac->Id);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  Status = Dpaa2McBindDpmacToDpni (&Dpaa2NetInterface->DprcMcIo,
                                  Dpaa2NetInterface->DprcHandle,
                                  Dpaa2NetInterface);
  if (EFI_ERROR (Status)) {
    goto ErrorExitDestroyDpmac;
  }

  Status = Dpaa2McSetDpniRxTxBufferLayouts (&Dpaa2NetInterface->DprcMcIo,
                                           Dpaa2NetInterface);
  if (EFI_ERROR (Status)) {
    goto ErrorExitDestroyDpmac;
  }

  Status = Dpaa2McInitializeDpbp (&Dpaa2NetInterface->DprcMcIo,
                                 Dpaa2NetInterface);
  if (EFI_ERROR (Status)) {
    goto ErrorExitDestroyDpmac;
  }

  Status = Dpaa2McSetDpniBufferPools (&Dpaa2NetInterface->DprcMcIo,
                                     Dpaa2NetInterface);
  if (EFI_ERROR (Status)) {
    goto ErrorExitDestroyDpmac;
  }

  Status = Dpaa2McInitializeDpni (&Dpaa2NetInterface->DprcMcIo,
                                 Dpaa2NetInterface, MacAddr);
  if (EFI_ERROR (Status)) {
    goto ErrorExitDestroyDpmac;
  }

  Status = Dpaa2McInitializeDpmac (&Dpaa2NetInterface->DprcMcIo,
                                  Dpaa2NetInterface, WriopDpmac);
  if (EFI_ERROR (Status)) {
    goto ErrorExitDestroyDpmac;
  }

  Status = Dpaa2McGetDpniQueues (&Dpaa2NetInterface->DprcMcIo, Dpaa2NetInterface);
  if (EFI_ERROR (Status)) {
    goto ErrorExitDestroyDpmac;
  }

  Status = Dpaa2McSetDpniMtu (&Dpaa2NetInterface->DprcMcIo, Dpaa2NetInterface, Mtu);
  if (EFI_ERROR (Status)) {
    goto ErrorExitDestroyDpmac;
  }

  Dpaa2NetInterface->StartedInMc = TRUE;

  if (gDpaaDebugFlags & DPAA_DEBUG_EXTRA_CHECKS) {
    if (!Dpaa2McCheckDpniLink (Dpaa2NetInterface)) {
      DPAA_DEBUG_MSG ("DPNI link is not up for 0x%p\n", Dpaa2NetInterface);
    }
  }

  return EFI_SUCCESS;

ErrorExitDestroyDpmac:
  Dpaa2McDestroyDpmac (&Dpaa2NetInterface->DprcMcIo, Dpaa2NetInterface);
  return Status;
}


STATIC
void
Dpaa2McGetDpniCounters (
  DPAA2_MC_IO *DprcMcIo,
  DPAA2_NETWORK_INTERFACE *Dpaa2NetInterface
  )
{
  INT32 McFlibError;
  UINT64 Value;

  ASSERT (Dpaa2NetInterface->DpniHandle != 0);
  McFlibError = dpni_get_counter (DprcMcIo, MC_CMD_NO_FLAGS,
                     Dpaa2NetInterface->DpniHandle,
                     DPNI_CNT_ING_FRAME,
                     &Value);
  if (McFlibError != 0) {
    DPAA_ERROR_MSG ("Error calling dpni_get_counter (DPNI_CNT_ING_FRAME) for 0x%p\n",
                    Dpaa2NetInterface);
    return;
  }

  DPAA_DEBUG_MSG ("DPNI_CNT_ING_FRAME=%llu\n", Value);

  McFlibError = dpni_get_counter (DprcMcIo, MC_CMD_NO_FLAGS,
                     Dpaa2NetInterface->DpniHandle,
                     DPNI_CNT_ING_BYTE,
                     &Value);
  if (McFlibError != 0) {
    DPAA_ERROR_MSG ("Error calling dpni_get_counter (DPNI_CNT_ING_BYTE) for 0x%p\n",
                    Dpaa2NetInterface);
    return;
  }

  DPAA_DEBUG_MSG ("DPNI_CNT_ING_BYTE=%llu\n", Value);

  McFlibError = dpni_get_counter (DprcMcIo, MC_CMD_NO_FLAGS,
                     Dpaa2NetInterface->DpniHandle,
                     DPNI_CNT_ING_FRAME_DROP,
                     &Value);
  if (McFlibError != 0) {
    DPAA_ERROR_MSG ("Error calling dpni_get_counter (DPNI_CNT_ING_FRAME_DROP) for 0x%p\n",
                    Dpaa2NetInterface);
    return;
  }

  DPAA_DEBUG_MSG ("DPNI_CNT_ING_FRAME_DROP=%llu\n", Value);

  McFlibError = dpni_get_counter (DprcMcIo, MC_CMD_NO_FLAGS,
                     Dpaa2NetInterface->DpniHandle,
                     DPNI_CNT_ING_FRAME_DISCARD,
                                 &Value);
  if (McFlibError != 0) {
    DPAA_ERROR_MSG ("Error calling dpni_get_counter (DPNI_CNT_ING_FRAME_DISCARD) for 0x%p\n",
                    Dpaa2NetInterface);
    return;
  }

  DPAA_DEBUG_MSG ("DPNI_CNT_ING_FRAME_DISCARD=%llu\n", Value);

  McFlibError = dpni_get_counter (DprcMcIo, MC_CMD_NO_FLAGS,
                                 Dpaa2NetInterface->DpniHandle,
                     DPNI_CNT_EGR_FRAME,
                                 &Value);
  if (McFlibError != 0) {
    DPAA_ERROR_MSG ("Error calling dpni_get_counter (DPNI_CNT_EGR_FRAME) for 0x%p\n",
                    Dpaa2NetInterface);
    return;
  }

  DPAA_DEBUG_MSG ("DPNI_CNT_EGR_FRAME=%llu\n", Value);

  McFlibError = dpni_get_counter (DprcMcIo, MC_CMD_NO_FLAGS,
                                 Dpaa2NetInterface->DpniHandle,
                     DPNI_CNT_EGR_BYTE,
                                 &Value);
  if (McFlibError != 0) {
    DPAA_ERROR_MSG ("Error calling dpni_get_counter (DPNI_CNT_EGR_BYTE) for 0x%p\n",
                    Dpaa2NetInterface);
    return;
  }

  DPAA_DEBUG_MSG ("DPNI_CNT_EGR_BYTE=%llu\n", Value);

  McFlibError = dpni_get_counter (DprcMcIo, MC_CMD_NO_FLAGS,
                                 Dpaa2NetInterface->DpniHandle,
                     DPNI_CNT_EGR_FRAME_DISCARD,
                                 &Value);
  if (McFlibError != 0) {
    DPAA_ERROR_MSG ("Error calling dpni_get_counter (DPNI_CNT_EGR_FRAME_DISCARD) for 0x%p\n",
                    Dpaa2NetInterface);
    return;
  }

  DPAA_DEBUG_MSG ("DPNI_CNT_EGR_FRAME_DISCARD=%llu\n", Value);
}


STATIC
VOID
Dpaa2McGetDpmacCounters (
  DPAA2_MC_IO *DprcMcIo,
  DPAA2_NETWORK_INTERFACE *Dpaa2NetInterface
  )
{
  INT32 McFlibError;
  UINT64 Value;

  ASSERT (Dpaa2NetInterface->DpmacHandle != 0);

  McFlibError = dpmac_get_counter (DprcMcIo, MC_CMD_NO_FLAGS,
                                  Dpaa2NetInterface->DpmacHandle,
                      DPMAC_CNT_ING_BYTE,
                      &Value);
  if (McFlibError != 0) {
    DPAA_ERROR_MSG ("Error calling dpmac_get_counter (DPMAC_CNT_ING_BYTE) for 0x%p\n",
                    Dpaa2NetInterface);
    return;
  }

  DPAA_DEBUG_MSG ("DPMAC_CNT_ING_BYTE=%llu\n", Value);

  McFlibError = dpmac_get_counter (DprcMcIo, MC_CMD_NO_FLAGS,
                                  Dpaa2NetInterface->DpmacHandle,
                      DPMAC_CNT_ING_FRAME_DISCARD,
                                  &Value);
  if (McFlibError != 0) {
    DPAA_ERROR_MSG ("Error calling dpmac_get_counter (DPMAC_CNT_ING_FRAME_DISCARD) for 0x%p\n",
                    Dpaa2NetInterface);
    return;
  }

  DPAA_DEBUG_MSG ("DPMAC_CNT_ING_FRAME_DISCARD=%llu\n", Value);

  McFlibError = dpmac_get_counter (DprcMcIo, MC_CMD_NO_FLAGS,
                                  Dpaa2NetInterface->DpmacHandle,
                      DPMAC_CNT_ING_ALIGN_ERR,
                      &Value);
  if (McFlibError != 0) {
    DPAA_ERROR_MSG ("Error calling dpmac_get_counter (DPMAC_CNT_ING_ALIGN_ERR) for 0x%p\n",
                    Dpaa2NetInterface);
    return;
  }

  DPAA_DEBUG_MSG ("DPMAC_CNT_ING_ALIGN_ERR=%llu\n", Value);

  McFlibError = dpmac_get_counter (DprcMcIo, MC_CMD_NO_FLAGS,
                                  Dpaa2NetInterface->DpmacHandle,
                      DPMAC_CNT_ING_ERR_FRAME,
                      &Value);
  if (McFlibError != 0) {
    DPAA_ERROR_MSG ("Error calling dpmac_get_counter (DPMAC_CNT_ING_ERR_FRAME) for 0x%p\n",
                    Dpaa2NetInterface);
    return;
  }

  DPAA_DEBUG_MSG ("DPMAC_CNT_ING_ERR_FRAME=%llu\n", Value);

  McFlibError = dpmac_get_counter (DprcMcIo, MC_CMD_NO_FLAGS,
                                  Dpaa2NetInterface->DpmacHandle,
                      DPMAC_CNT_EGR_BYTE,
                                  &Value);
  if (McFlibError != 0) {
    DPAA_ERROR_MSG ("Error calling dpmac_get_counter (DPMAC_CNT_EGR_BYTE) for 0x%p\n",
                    Dpaa2NetInterface);
    return;
  }

  DPAA_DEBUG_MSG ("DPMAC_CNT_EGR_BYTE=%llu\n", Value);

  McFlibError = dpmac_get_counter (DprcMcIo, MC_CMD_NO_FLAGS,
                                  Dpaa2NetInterface->DpmacHandle,
                      DPMAC_CNT_EGR_ERR_FRAME,
                                  &Value);
  if (McFlibError != 0) {
    DPAA_ERROR_MSG ("Error calling dpmac_get_counter (DPMAC_CNT_EGR_ERR_FRAME) for 0x%p\n",
                    Dpaa2NetInterface);
    return;
  }

  DPAA_DEBUG_MSG ("DPMAC_CNT_EGR_ERR_FRAME=%llu\n", Value);
}


/**
 * Unbind the DPAA2 Ethernet interface's DPMAC from the DPAA2 Ethernet interface's DPNI
 */
STATIC
VOID
Dpaa2McUnbindDpmacFromDpni (
  DPAA2_MC_IO *DprcMcIo,
  UINT16 DprcHandle,
  DPAA2_NETWORK_INTERFACE *Dpaa2NetInterface
  )
{
  INT32 McFlibError;

  McFlibError = dprc_disconnect (DprcMcIo, MC_CMD_NO_FLAGS,
              DprcHandle, &Dpaa2NetInterface->DpmacEndpoint);
  if (McFlibError != 0) {
    DPAA_ERROR_MSG ("Error calling dprc_disconnect () for 0x%p\n",
                    Dpaa2NetInterface);
  }
}


STATIC
VOID
Dpaa2McShutdownDpni (
  DPAA2_MC_IO *DprcMcIo,
  DPAA2_NETWORK_INTERFACE *Dpaa2NetInterface)
{
  INT32 McFlibError;

  McFlibError = dpni_clear_mac_filters (DprcMcIo, MC_CMD_NO_FLAGS,
                                       Dpaa2NetInterface->DpniHandle,
                                       1, 1);

  McFlibError = dpni_disable (DprcMcIo, MC_CMD_NO_FLAGS,
                             Dpaa2NetInterface->DpniHandle);
  if (McFlibError != 0) {
    DPAA_ERROR_MSG ("Error calling dpni_disable () for 0x%p\n",
                    Dpaa2NetInterface);
  }
}


/**
   Shutdown a DPAA2 network interface in the MC

   @param Dpaa2NetInterface Pointer to DPAA2 network interface control block
 **/
VOID
Dpaa2McShutdownNetworkInterface (
  DPAA2_NETWORK_INTERFACE *Dpaa2NetInterface
  )
{
  ASSERT (Dpaa2NetInterface->Signature == DPAA2_NETWORK_INTERFACE_SIGNATURE);
  ASSERT (Dpaa2NetInterface->CreatedInMc);
  ASSERT (Dpaa2NetInterface->StartedInMc);
  ASSERT (Dpaa2NetInterface->DpniHandle != 0);
  ASSERT (Dpaa2NetInterface->DpbpHandle != 0);
  ASSERT (Dpaa2NetInterface->DprcHandle != 0);
  ASSERT (Dpaa2NetInterface->DpmacHandle != 0);

  if (gDpaaDebugFlags & DPAA_DEBUG_DUMP_VALUES) {
    Dpaa2McGetDpniCounters (&Dpaa2NetInterface->DprcMcIo, Dpaa2NetInterface);
    Dpaa2McGetDpmacCounters (&Dpaa2NetInterface->DprcMcIo, Dpaa2NetInterface);
  }

  Dpaa2NetInterface->StartedInMc = FALSE;
  Dpaa2McUnbindDpmacFromDpni (&Dpaa2NetInterface->DprcMcIo,
                             Dpaa2NetInterface->DprcHandle,
                             Dpaa2NetInterface);

  /*
   * Stop Tx and Rx traffic
   */
  Dpaa2McShutdownDpni (&Dpaa2NetInterface->DprcMcIo, Dpaa2NetInterface);

  Dpaa2McShutdownDpbp (&Dpaa2NetInterface->DprcMcIo, Dpaa2NetInterface);

  Dpaa2McDestroyDpmac (&Dpaa2NetInterface->DprcMcIo, Dpaa2NetInterface);
}


/**
   Reset a DPAA2 network interface in the MC

   @param Dpaa2NetInterface Pointer to DPAA2 network interface control block
 **/
VOID
Dpaa2McResetNetworkInterface (
  DPAA2_NETWORK_INTERFACE *Dpaa2NetInterface
  )
{
  ASSERT (Dpaa2NetInterface->Signature == DPAA2_NETWORK_INTERFACE_SIGNATURE);
  ASSERT (Dpaa2NetInterface->CreatedInMc);
  ASSERT (Dpaa2NetInterface->DpniHandle != 0);
  ASSERT (Dpaa2NetInterface->DpbpHandle != 0);

  (VOID)dpni_reset (&Dpaa2NetInterface->DprcMcIo, MC_CMD_NO_FLAGS,
                   Dpaa2NetInterface->DpniHandle);
  (VOID)dpbp_reset (&Dpaa2NetInterface->DprcMcIo, MC_CMD_NO_FLAGS,
                   Dpaa2NetInterface->DpbpHandle);
}


/**
   Transmits an Ethernet frame on a DPAA2 network interface

   @param Dpaa2NetInterface    Pointer to DPAA2 network interface control block
   @param HdrSize              Header size
   @param BuffSize             Total frame size
   @param Data                 Pointer to frame buffer
   @param SrcAddr              Pointer to source MAC address
   @param DstAddr              Pointer to destination MAC address
   @param Protocol             Pointer to carried protocol
   @param QBmanTxBufferAddrOut Pointer to area where QBman buffer address is to be returned

   @retval EFI_SUCCESS, on success
   @retval error code, on failure
 **/
EFI_STATUS
Dpaa2McNetworkInterfaceTransmit (
  DPAA2_NETWORK_INTERFACE *Dpaa2NetInterface,
  WRIOP_DPMAC *WriopDpmac,
  UINTN HdrSize,
  UINTN BuffSize,
  VOID *Data,
  EFI_MAC_ADDRESS *SrcAddr,
  EFI_MAC_ADDRESS *DstAddr,
  UINT16 *Protocol,
  UINT64 *QBmanTxBufferAddrOut
  )
{
  /**
   * Timeout for successfully enqueuing a Tx frame to QBman (in microseconds)
   */
# define QBMAN_TX_BUFFER_ENQUEUE_TIMEOUT_US 2000

  EFI_STATUS Status;
  QBMAN_ENQUEUE_DESC QbmanEnqueueDesc;
  QBMAN_RELEASE_DESC QbmanReleaseDesc;
  QBMAN_FRAME_DESC QbmanFrameDesc;
  UINT64 FrameBufferAddr;
  UINT32 TimeoutCount;
  UINT32 NumAcquiredBuffers;
  UINT16 DataOffset;
  DPAA2_QBMAN_PORTAL *DpioQbmanPortal;
  UINT16 HwBufferPoolId;

  DataOffset = Dpaa2NetInterface->TxDataOffset;
  DpioQbmanPortal = &Dpaa2NetInterface->DpioQbmanPortal;
  HwBufferPoolId = Dpaa2NetInterface->DpbpHwBufferPoolId;

  ASSERT (Dpaa2NetInterface->Signature == DPAA2_NETWORK_INTERFACE_SIGNATURE);
  ASSERT (Dpaa2NetInterface->CreatedInMc);
  ASSERT (Dpaa2NetInterface->DpniHandle != 0);
  ASSERT (Dpaa2NetInterface->DpbpHandle != 0);
  ASSERT (Dpaa2NetInterface->DprcHandle != 0);
  ASSERT (Data != NULL);
  ASSERT (BuffSize >= sizeof (ETHER_HEAD) && BuffSize <= DPAA2_ETH_RX_FRAME_BUFFER_SIZE);
  ASSERT (HdrSize <= BuffSize);
  if (HdrSize != 0) {
    /*
     * Populate Ethernet header:
     */
    ETHER_HEAD *EthernetHeader = (ETHER_HEAD *)Data;

    ASSERT (HdrSize == sizeof (ETHER_HEAD));
    ASSERT (SrcAddr != NULL);
    ASSERT (DstAddr != NULL);
    ASSERT (Protocol != NULL);

    CopyMem (EthernetHeader->DstMac, DstAddr, NET_ETHER_ADDR_LEN);
    CopyMem (EthernetHeader->SrcMac, SrcAddr, NET_ETHER_ADDR_LEN);
    EthernetHeader->EtherType = HTONS (*Protocol);
  }

  NumAcquiredBuffers = QbmanSwPortalAcquireBuffers (DpioQbmanPortal,
                                                   HwBufferPoolId,
                                                   &FrameBufferAddr,
                                                   1);
  if (NumAcquiredBuffers == 0) {
    return EFI_NOT_READY;
  }

  /*
   * Copy UEFI data buffer to QBman buffer:
   */
  CopyMem ((VOID *)(FrameBufferAddr + DataOffset), Data, BuffSize);

  CleanDcacheRange ((UINT64)FrameBufferAddr,
                   (UINT64)FrameBufferAddr + DPAA2_ETH_RX_FRAME_BUFFER_SIZE);

  ZeroMem (&QbmanFrameDesc, sizeof (QbmanFrameDesc));
  QbmanFrameDesc.Simple.AddressHighWord = FrameBufferAddr >> 32;
  QbmanFrameDesc.Simple.AddressLowWord = (UINT32)FrameBufferAddr;
  QbmanFrameDesc.Simple.BpidOffset &= 0xF000FFFF;
  QbmanFrameDesc.Simple.BpidOffset |= (UINT32)DataOffset << 16;
  QbmanFrameDesc.Simple.BpidOffset &= 0xFFFF0000;
  QbmanFrameDesc.Simple.BpidOffset |= (UINT32)HwBufferPoolId;
  QbmanFrameDesc.Simple.Length = (UINT32)BuffSize;
  QbmanFrameDesc.Simple.Control = QBMAN_FD_CTRL_ASAL |
                                  QBMAN_FD_CTRL_PTA |
          QBMAN_FD_CTRL_PTV1;

  QbmanEnqueueDescClear (&QbmanEnqueueDesc);
  QbmanEnqueueDescSetNoOrp (&QbmanEnqueueDesc, FALSE);
  QbmanEnqueueDescSetQueueDest (&QbmanEnqueueDesc,
                         Dpaa2NetInterface->TxQueueDestinationId,
                         Dpaa2NetInterface->TxFlowId,
                               0);

  TimeoutCount = QBMAN_TX_BUFFER_ENQUEUE_TIMEOUT_US;
  for ( ; ; ) {
    BOOLEAN EnqueueOk = QbmanSwPortalEnqueue (DpioQbmanPortal,
                                             &QbmanEnqueueDesc,
                                             &QbmanFrameDesc);

    if (EnqueueOk) {
      break;
    }

    if (TimeoutCount == 0) {
      DPAA_ERROR_MSG ("Timeout enqueueing Tx buffer to QBman for interface 0x%p\n",
                      Dpaa2NetInterface);
      Status = EFI_NOT_READY;
      goto ErrorExit;
    }

    MicroSecondDelay (100);
    TimeoutCount -= 100;
  }

  if (gDpaaDebugFlags & DPAA_DEBUG_TRACE_NET_PACKETS) {
    DPAA_DEBUG_MSG ("Tx: %x:%x:%x:%x:%x:%x|%x:%x:%x:%x:%x:%x|%x|%x|%x%x%x%x\n",
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

  *QBmanTxBufferAddrOut = FrameBufferAddr;
  return EFI_SUCCESS;

ErrorExit:
  /*
   * Release Tx buffer into the QBMAN:
   */
  QbmanReleaseDescClear (&QbmanReleaseDesc);
  QbmanReleaseDescSetBpid (&QbmanReleaseDesc, HwBufferPoolId);
  TimeoutCount = QBMAN_BUFFER_RELEASE_TIMEOUT_US;
  for ( ; ; ) {
    BOOLEAN ReleaseOk =  QbmanSwPortalReleaseBuffers (DpioQbmanPortal,
                                                    &QbmanReleaseDesc,
                                                    &FrameBufferAddr,
                                                    1);
    if (ReleaseOk) {
      break;
    }

    if (TimeoutCount == 0) {
      DPAA_ERROR_MSG ("Timeout releasing Tx buffer to QBman for interface 0x%p\n",
                      Dpaa2NetInterface);
      break;
    }

    MicroSecondDelay (100);
    TimeoutCount -= 100;
  }

  return Status;
}


STATIC
VOID
Dppa2NetworkInterfaceReceiveFrame (
  DPAA2_NETWORK_INTERFACE *Dpaa2NetInterface,
  const QBMAN_FRAME_DESC *FrameDesc,
  UINTN *BuffSize,
  VOID *Data,
  EFI_MAC_ADDRESS *SrcAddr,
  EFI_MAC_ADDRESS *DstAddr,
  UINT16 *Protocol
  )
{
  UINT32 TimeoutCount;
  UINT64 FrameBufferAddr;
  UINT16 FrameDataOffset;
  UINT32 FrameDataLength;
  UINT32 FrameStatus;
  QBMAN_FRAME_ANNOTATION_STATUS *FrameAnnotationStatus;
  QBMAN_RELEASE_DESC QbmanReleaseDesc;
  ETHER_HEAD *EthernetHeader;
  DPAA2_QBMAN_PORTAL *DpioQbmanPortal;
  UINT32 DataSize;

  DataSize = 0;
  DpioQbmanPortal = &Dpaa2NetInterface->DpioQbmanPortal;
  FrameBufferAddr = ((UINT64)FrameDesc->Simple.AddressHighWord << 32) +
                    FrameDesc->Simple.AddressLowWord;

  FrameDataOffset = (FrameDesc->Simple.BpidOffset >> 16) & 0x0FFF;
  FrameDataLength = FrameDesc->Simple.Length;

  if (FrameDesc->Simple.FrameContext & QBMAN_FD_FRC_FASV) {
    /*
     * Check frame annotation status:
     */
    FrameAnnotationStatus = (QBMAN_FRAME_ANNOTATION_STATUS *)
      (FrameBufferAddr + Dpaa2NetInterface->FramePrivateDataSize);

    FrameStatus = FrameAnnotationStatus->Status;
    if (FrameStatus & QBMAN_ETH_RX_ERROR_MASK) {
      DPAA_ERROR_MSG ("Frame received with errors: 0x%08x\n",
          FrameStatus & QBMAN_ETH_RX_ERROR_MASK);
      goto CommonExit;
    }

    if (FrameStatus & QBMAN_ETH_RX_UNSUPPORTED_MASK) {
      DPAA_ERROR_MSG ("Frame received with unsupported features: 0x%08x\n",
          FrameStatus & QBMAN_ETH_RX_UNSUPPORTED_MASK);
      goto CommonExit;
    }
  }

  if (*BuffSize < FrameDataLength) {
    DPAA_ERROR_MSG ("Frame truncated (actual frame length %u, buffer size: %lu\n",
                    FrameDataLength, *BuffSize);

    DataSize = *BuffSize;
  } else {
    DataSize = FrameDataLength;
  }

  /*
   * TODO: Do we need to purge the data cache, so that we don't
   * read stale data?
   */
  CopyMem (Data, (VOID *)(FrameBufferAddr + FrameDataOffset), DataSize);

  ASSERT (DataSize >= sizeof (ETHER_HEAD));
  EthernetHeader = (ETHER_HEAD *)Data;
  if (DstAddr != NULL) {
    CopyMem (DstAddr, EthernetHeader->DstMac, NET_ETHER_ADDR_LEN);
  }

  if (SrcAddr != NULL) {
    CopyMem (SrcAddr, EthernetHeader->SrcMac, NET_ETHER_ADDR_LEN);
  }

  if (Protocol != NULL) {
    *Protocol = NTOHS (EthernetHeader->EtherType);
  }

CommonExit:
  *BuffSize = DataSize;
  CleanDcacheRange (FrameBufferAddr, FrameBufferAddr + DPAA2_ETH_RX_FRAME_BUFFER_SIZE);

  /*
   * Release Rx buffer into the QBMAN:
   */
  QbmanReleaseDescClear (&QbmanReleaseDesc);
  QbmanReleaseDescSetBpid (&QbmanReleaseDesc, Dpaa2NetInterface->DpbpHwBufferPoolId);
  TimeoutCount = QBMAN_BUFFER_RELEASE_TIMEOUT_US;
  for ( ; ; ) {
    BOOLEAN ReleaseOk =  QbmanSwPortalReleaseBuffers (DpioQbmanPortal,
                                                    &QbmanReleaseDesc,
                                                    &FrameBufferAddr,
                                                    1);
    if (ReleaseOk) {
      break;
    }

    if (TimeoutCount == 0) {
      DPAA_ERROR_MSG ("Timeout releasing Rx buffer to QBman for interface 0x%p\n",
                      Dpaa2NetInterface);
      break;
    }

    MicroSecondDelay (100);
    TimeoutCount -= 100;
  }
}


/**
   Receives an Ethernet frame on a DPAA2 network interface

   @param Dpaa2NetInterface Pointer to DPAA2 network interface control block
   @param BuffSize          Pointer to frame size
   @param Data              Pointer to frame buffer
   @param SrcAddr           Pointer to source MAC address
   @param DstAddr           Pointer to destination MAC address
   @param Protocol          Pointer to carried protocol

   @retval EFI_SUCCESS, on success
   @retval error code, on failure
 **/
EFI_STATUS
Dpaa2McNetworkInterfaceReceive (
  DPAA2_NETWORK_INTERFACE *Dpaa2NetInterface,
  WRIOP_DPMAC *WriopDpmac,
  UINTN *BuffSize,
  VOID *Data,
  EFI_MAC_ADDRESS *SrcAddr,
  EFI_MAC_ADDRESS *DstAddr,
  UINT16 *Protocol
  )
{
  UINT32 TimeoutCount;
  const QBMAN_DEQUEUE_ENTRY *DequeueEntry;
  const QBMAN_FRAME_DESC *FrameDesc;
  UINT32 DequeueEntryFlags;
  BOOLEAN PullOk;
  QBMAN_PULL_DESC PullDesc;
  DPAA2_QBMAN_PORTAL *DpioQbmanPortal;

  DpioQbmanPortal = &Dpaa2NetInterface->DpioQbmanPortal;
  QbmanPullDescClear (&PullDesc);
  QbmanPullDescSetNumframes (&PullDesc, 1);
  QbmanPullDescSetFrameDesc (&PullDesc,
                            Dpaa2NetInterface->RxDefaultFrameQueueId);
  PullOk = QbmanSwPortalPull (DpioQbmanPortal, &PullDesc);
  if (!PullOk) {
    return EFI_NOT_READY;
  }

  TimeoutCount = QBMAN_DEQUEUE_TIMEOUT_US;
  for ( ; ; ) {
    DequeueEntry = QbmanSwPortalGetNextDqrrEntry (DpioQbmanPortal);
    if (DequeueEntry != NULL) {
      break;
    }

    if (TimeoutCount == 0) {
      DPAA_ERROR_MSG ("Timeout dequeueing Rx frame from QBman for interface 0x%p\n",
                      Dpaa2NetInterface);
      return EFI_NOT_READY;
    }

    MicroSecondDelay (100);
    TimeoutCount -= 100;
  }

  DequeueEntryFlags = QbmanGetFlagsFromDequeueEntry (DequeueEntry);
  if ((DequeueEntryFlags & QBMAN_DEQUEUE_STAT_VALIDFRAME) == 0) {
    QbmanSwPortalConsumeDqrrEntry (DpioQbmanPortal, DequeueEntry);
    return EFI_DEVICE_ERROR;
  }

  /*
   * Receive frame:
   */
  FrameDesc = QbmanGetFrameDescFromDequeueEntry (DequeueEntry);
  Dppa2NetworkInterfaceReceiveFrame (Dpaa2NetInterface,
                                    FrameDesc,
                                    BuffSize,
                                    Data,
                                    SrcAddr,
                                    DstAddr,
                                    Protocol);
  QbmanSwPortalConsumeDqrrEntry (DpioQbmanPortal, DequeueEntry);

  if (gDpaaDebugFlags & DPAA_DEBUG_TRACE_NET_PACKETS) {
    DPAA_DEBUG_MSG ("Rx: %x:%x:%x:%x:%x:%x|%x:%x:%x:%x:%x:%x|%x|%x|%x%x%x%x\n",
      SrcAddr->Addr[0], SrcAddr->Addr[1], SrcAddr->Addr[2],
      SrcAddr->Addr[3], SrcAddr->Addr[4], SrcAddr->Addr[5],
      DstAddr->Addr[0], DstAddr->Addr[1], DstAddr->Addr[2],
      DstAddr->Addr[3], DstAddr->Addr[4], DstAddr->Addr[5],
      *Protocol, *BuffSize,
      ((UINT8 *)Data)[0],
      ((UINT8 *)Data)[1],
      ((UINT8 *)Data)[2],
      ((UINT8 *)Data)[3]);
  }

  return EFI_SUCCESS;
}


/**
   Adds a multicast address to the hardware Multicast filter table

   @param Dpaa2NetInterface Pointer to DPAA2 network interface control block
   @param SrcAddr           Pointer to multicast MAC address

   @retval EFI_SUCCESS, on success
   @retval error code, on failure
 **/
EFI_STATUS
Dpaa2McAddMulticastMacAddress (
  DPAA2_NETWORK_INTERFACE *Dpaa2NetInterface,
  EFI_MAC_ADDRESS *MulticastMacAddr
  )
{
  INT32 McFlibError;

  ASSERT (MulticastMacAddr->Addr[0] & MAC_MULTICAST_ADDRESS_BIT);

  McFlibError = dpni_add_mac_addr (&Dpaa2NetInterface->DprcMcIo,
                                   MC_CMD_NO_FLAGS,
                                   Dpaa2NetInterface->DpniHandle,
                                   MulticastMacAddr->Addr);
  if (McFlibError != 0) {
    DPAA_ERROR_MSG ("Error calling dpni_add_mac_addr () for 0x%p\n",
                    Dpaa2NetInterface);
    return EFI_DEVICE_ERROR;
  }

  DPAA_DEBUG_MSG ("Added multicast address %x:%x:%x:%x:%x:%x to dpni.%u\n",
                  MulticastMacAddr->Addr[0],
                  MulticastMacAddr->Addr[1],
                  MulticastMacAddr->Addr[2],
                  MulticastMacAddr->Addr[3],
                  MulticastMacAddr->Addr[4],
                  MulticastMacAddr->Addr[5],
                  Dpaa2NetInterface->DpniId);

  return EFI_SUCCESS;
}


/**
   Removes a multicast address from the hardware Multicast filter table

   @param Dpaa2NetInterface Pointer to DPAA2 network interface control block
   @param SrcAddr           Pointer to multicast MAC address

   @retval EFI_SUCCESS, on success
   @retval error code, on failure
 **/
EFI_STATUS
Dpaa2McRemoveMulticastMacAddress (
  DPAA2_NETWORK_INTERFACE *Dpaa2NetInterface,
  EFI_MAC_ADDRESS *MulticastMacAddr
  )
{
  INT32 McFlibError;

  ASSERT (MulticastMacAddr->Addr[0] & MAC_MULTICAST_ADDRESS_BIT);

  McFlibError = dpni_remove_mac_addr (&Dpaa2NetInterface->DprcMcIo,
                                     MC_CMD_NO_FLAGS,
                                     Dpaa2NetInterface->DpniHandle,
                                     MulticastMacAddr->Addr);
  if (McFlibError != 0) {
    DPAA_ERROR_MSG ("Error calling dpni_remove_mac_addr () for 0x%p\n",
                    Dpaa2NetInterface);
    return EFI_DEVICE_ERROR;
  }

  DPAA_DEBUG_MSG ("Removed multicast address %x:%x:%x:%x:%x:%x to dpni.%u\n",
                  MulticastMacAddr->Addr[0],
                  MulticastMacAddr->Addr[1],
                  MulticastMacAddr->Addr[2],
                  MulticastMacAddr->Addr[3],
                  MulticastMacAddr->Addr[4],
                  MulticastMacAddr->Addr[5],
                  Dpaa2NetInterface->DpniId);

  return EFI_SUCCESS;
}


/**
   Check if a Tx completion (confirmation) has been received. If so, it returns
   the address of the corresponding QBman Tx buffer.

   @param Dpaa2NetInterface    Pointer to DPAA2 network interface control block
   @param QBmanTxBufferAddrOut Pointer to area where QBman buffer address is to be returned

   @retval EFI_SUCCESS, on success
   @retval error code, on failure

 **/
EFI_STATUS
Dpaa2McNetworkInterfaceCheckTxCompletion (
  DPAA2_NETWORK_INTERFACE *Dpaa2NetInterface,
  UINT64 *QBmanTxBufferAddrOut
  )
{
  QBMAN_PULL_DESC PullDesc;
  QBMAN_RELEASE_DESC QbmanReleaseDesc;
  UINT64 FrameBufferAddr;
  UINT32 TimeoutCount;
  BOOLEAN PullOk;
  UINT32 DequeueEntryFlags;
  const QBMAN_DEQUEUE_ENTRY *DequeueEntry;
  const QBMAN_FRAME_DESC *FrameDesc;
  DPAA2_QBMAN_PORTAL *DpioQbmanPortal;

  DpioQbmanPortal = &Dpaa2NetInterface->DpioQbmanPortal;
  QbmanPullDescClear (&PullDesc);
  QbmanPullDescSetNumframes (&PullDesc, 1);
  QbmanPullDescSetFrameDesc (&PullDesc,
                            Dpaa2NetInterface->TxConfirmationsQueueId);
  PullOk = QbmanSwPortalPull (DpioQbmanPortal, &PullDesc);
  if (!PullOk) {
    return EFI_NOT_READY;
  }

  TimeoutCount = QBMAN_DEQUEUE_TIMEOUT_US;
  for ( ; ; ) {
    DequeueEntry = QbmanSwPortalGetNextDqrrEntry (DpioQbmanPortal);
    if (DequeueEntry != NULL) {
      break;
    }

    if (TimeoutCount == 0) {
      DPAA_ERROR_MSG ("Timeout dequeueing Tx confirmation from QBman for interface 0x%p\n",
                      Dpaa2NetInterface);
      return EFI_NOT_READY;
    }

    MicroSecondDelay (100);
    TimeoutCount -= 100;
  }

  DequeueEntryFlags = QbmanGetFlagsFromDequeueEntry (DequeueEntry);
  if ((DequeueEntryFlags & QBMAN_DEQUEUE_STAT_VALIDFRAME) == 0) {
    QbmanSwPortalConsumeDqrrEntry (DpioQbmanPortal, DequeueEntry);
    return EFI_DEVICE_ERROR;
  }

  FrameDesc = QbmanGetFrameDescFromDequeueEntry (DequeueEntry);
  FrameBufferAddr =
      ((UINT64)FrameDesc->Simple.AddressHighWord << 32) +
      FrameDesc->Simple.AddressLowWord;

  /*
   * Release Tx buffer into the QBMAN:
   */
  QbmanReleaseDescClear (&QbmanReleaseDesc);
  QbmanReleaseDescSetBpid (&QbmanReleaseDesc, Dpaa2NetInterface->DpbpHwBufferPoolId);
  TimeoutCount = QBMAN_BUFFER_RELEASE_TIMEOUT_US;
  for ( ; ; ) {
    BOOLEAN ReleaseOk =  QbmanSwPortalReleaseBuffers (DpioQbmanPortal,
                                                    &QbmanReleaseDesc,
                                                    &FrameBufferAddr,
                                                    1);
    if (ReleaseOk) {
      break;
    }

    if (TimeoutCount == 0) {
      DPAA_ERROR_MSG ("Timeout releasing Rx buffer to QBman for interface 0x%p\n",
                      Dpaa2NetInterface);
      break;
    }

    MicroSecondDelay (100);
    TimeoutCount -= 100;
  }

  QbmanSwPortalConsumeDqrrEntry (DpioQbmanPortal, DequeueEntry);
  *QBmanTxBufferAddrOut = FrameBufferAddr;
  return EFI_SUCCESS;
}
