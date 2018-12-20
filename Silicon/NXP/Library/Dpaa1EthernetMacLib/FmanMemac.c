/** FmanMemac.c
  DPAA1 FMAN MAC services implementation

  Copyright (c) 2016, Freescale Semiconductor, Inc. All rights Reserved.

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution. The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/
#include <libfdt.h>
#include <Library/Dpaa1DebugLib.h>
#include <Library/Dpaa1EthernetMacLib.h>
#include <Library/FrameManager.h>
#include <Library/IoLib.h>
#include <Library/ItbParse.h>
#include <Library/PcdLib.h>
#include <Library/Utils.h>

CONST CHAR8 *CONST gFmanMemacStrings[] = {
  [FM1_DTSEC_1] = "MEMAC1",
  [FM1_DTSEC_2] = "MEMAC2",
  [FM1_DTSEC_3] = "MEMAC3",
  [FM1_DTSEC_4] = "MEMAC4",
  [FM1_DTSEC_5] = "MEMAC5",
  [FM1_DTSEC_6] = "MEMAC6",
  [FM1_DTSEC_9] = "MEMAC9",
  [FM1_DTSEC_10] = "MEMAC10",
};

STATIC CONST CHAR8 *CONST gPhyInterfaceTypeStrings[] = {
  [PHY_INTERFACE_XFI] = "xgmii",
  [PHY_INTERFACE_SGMII] = "sgmii",
  [PHY_INTERFACE_SGMII_2500] = "Sgmii_2500",
  [PHY_INTERFACE_QSGMII] = "qsgmii",
  [PHY_INTERFACE_RGMII] = "rgmii-txid",
};

#define FMAN_MEMAC_INITIALIZER(_MemacId) \
        [_MemacId] = {                                                  \
          .Signature = FMAN_MEMAC_SIGNATURE,                            \
          .Id = _MemacId,                                               \
          .Enabled = FALSE,                                             \
          .Phy = {                                                      \
            .Signature = DPAA1_PHY_SIGNATURE,                           \
            .PhyInterfaceType = PHY_INTERFACE_NONE,                     \
          },                                                            \
        }

STATIC FMAN_MEMAC gFmanMemacs[] = {
  FMAN_MEMAC_INITIALIZER(FM1_DTSEC_1),
  FMAN_MEMAC_INITIALIZER(FM1_DTSEC_2),
  FMAN_MEMAC_INITIALIZER(FM1_DTSEC_3),
  FMAN_MEMAC_INITIALIZER(FM1_DTSEC_4),
  FMAN_MEMAC_INITIALIZER(FM1_DTSEC_5),
  FMAN_MEMAC_INITIALIZER(FM1_DTSEC_6),
  FMAN_MEMAC_INITIALIZER(FM1_DTSEC_9), 
  FMAN_MEMAC_INITIALIZER(FM1_DTSEC_10),
};

VOID
DumpMac (
  IN Memac *regs
 )
{
  DPAA1_DEBUG_MSG(" ======= DUMPING MAC REGISTERS ======= \n");
  DPAA1_DEBUG_MSG(" CommandConfig    : 0x%x \n", MmioReadBe32((UINTN)&regs->CommandConfig   ));
  DPAA1_DEBUG_MSG(" MacAddr_0        : 0x%x \n", MmioReadBe32((UINTN)&regs->MacAddr_0       ));
  DPAA1_DEBUG_MSG(" MacAddr_1        : 0x%x \n", MmioReadBe32((UINTN)&regs->MacAddr_1       ));
  DPAA1_DEBUG_MSG(" Maxfrm            : 0x%x \n", MmioReadBe32((UINTN)&regs->Maxfrm           ));
  DPAA1_DEBUG_MSG(" RxFifo		   : 0x%x \n", MmioReadBe32((UINTN)&regs->RxFifo		   ));
  DPAA1_DEBUG_MSG(" TxFifo		   : 0x%x \n", MmioReadBe32((UINTN)&regs->TxFifo		   ));
  DPAA1_DEBUG_MSG(" HashtableCtrl    : 0x%x \n", MmioReadBe32((UINTN)&regs->HashtableCtrl   ));
  DPAA1_DEBUG_MSG(" Ievent            : 0x%x \n", MmioReadBe32((UINTN)&regs->Ievent           ));
  DPAA1_DEBUG_MSG(" TxIpgLength     : 0x%x \n", MmioReadBe32((UINTN)&regs->TxIpgLength    ));
  DPAA1_DEBUG_MSG(" Imask             : 0x%x \n", MmioReadBe32((UINTN)&regs->Imask            ));
  DPAA1_DEBUG_MSG(" RxPauseStatus   : 0x%x \n", MmioReadBe32((UINTN)&regs->RxPauseStatus  ));
  DPAA1_DEBUG_MSG(" LpwakeTimer      : 0x%x \n", MmioReadBe32((UINTN)&regs->LpwakeTimer     ));
  DPAA1_DEBUG_MSG(" SleepTimer       : 0x%x \n", MmioReadBe32((UINTN)&regs->SleepTimer      ));
  DPAA1_DEBUG_MSG(" StatnConfig      : 0x%x \n", MmioReadBe32((UINTN)&regs->StatnConfig     ));
  DPAA1_DEBUG_MSG(" ClPauseQuanta[0]: 0x%x \n", MmioReadBe32((UINTN)&regs->ClPauseQuanta[0]));
  DPAA1_DEBUG_MSG(" ClPauseQuanta[1]: 0x%x \n", MmioReadBe32((UINTN)&regs->ClPauseQuanta[1]));
  DPAA1_DEBUG_MSG(" ClPauseQuanta[2]: 0x%x \n", MmioReadBe32((UINTN)&regs->ClPauseQuanta[2]));
  DPAA1_DEBUG_MSG(" ClPauseQuanta[3]: 0x%x \n", MmioReadBe32((UINTN)&regs->ClPauseQuanta[3]));
  DPAA1_DEBUG_MSG(" ClPauseThresh[0]: 0x%x \n", MmioReadBe32((UINTN)&regs->ClPauseThresh[0]));
  DPAA1_DEBUG_MSG(" ClPauseThresh[1]: 0x%x \n", MmioReadBe32((UINTN)&regs->ClPauseThresh[1]));
  DPAA1_DEBUG_MSG(" ClPauseThresh[2]: 0x%x \n", MmioReadBe32((UINTN)&regs->ClPauseThresh[2]));
  DPAA1_DEBUG_MSG(" ClPauseThresh[3]: 0x%x \n", MmioReadBe32((UINTN)&regs->ClPauseThresh[3]));

  DPAA1_DEBUG_MSG(" IfMode		   : 0x%x \n", MmioReadBe32((UINTN)&regs->IfMode           ));
  DPAA1_DEBUG_MSG(" IfStatus	   : 0x%x \n", MmioReadBe32((UINTN)&regs->IfStatus         ));
}

VOID
DumpMacStats (
  IN  ENET_MAC *Mac
  )
{
  Memac *Regs = Mac->Base;
  DPAA1_DEBUG_MSG(" ------------- DUMPING MAC STATISTICS -------------\n");
  DPAA1_DEBUG_MSG(" ------------- DUMPING RX STATISTICS -------------\n");

  DPAA1_DEBUG_MSG(" RxAlignErrL %d \n", 
	MmioReadBe32((UINTN)&Regs->RxAlignErrL));      	/* Rx alignment error lower */
  DPAA1_DEBUG_MSG(" RxAlignErrU %d \n", 
	MmioReadBe32((UINTN)&Regs->RxAlignErrU));      	/* Rx alignment error upper */
  DPAA1_DEBUG_MSG(" RxFrameL %d \n", 
	MmioReadBe32((UINTN)&Regs->RxFrameL));   		/* Rx frame counter lower */
  DPAA1_DEBUG_MSG(" RxFrameU %d \n", 
	MmioReadBe32((UINTN)&Regs->RxFrameU));   		/* Rx frame counter upper */
  DPAA1_DEBUG_MSG(" RxFrameCrcErrL %d \n", 
	MmioReadBe32((UINTN)&Regs->RxFrameCrcErrL)); 	/* Rx frame check sequence error lower */
  DPAA1_DEBUG_MSG(" RxFrameCrcErrU %d \n", 
	MmioReadBe32((UINTN)&Regs->RxFrameCrcErrU)); 	/* Rx frame check sequence error upper */
  DPAA1_DEBUG_MSG(" RxErrL %d \n", 
	MmioReadBe32((UINTN)&Regs->RxErrL));     /* Rx frame error lower */
  DPAA1_DEBUG_MSG(" RxErrU %d \n", 
	MmioReadBe32((UINTN)&Regs->RxErrU));     /* Rx frame error upper */
  DPAA1_DEBUG_MSG(" RxDropL %d \n", 
	MmioReadBe32((UINTN)&Regs->RxDropL));    /* Rx dropped packets lower */
  DPAA1_DEBUG_MSG(" RxDropU %d \n", 
	MmioReadBe32((UINTN)&Regs->RxDropU));    /* Rx dropped packets upper */
  DPAA1_DEBUG_MSG(" RxPktL %d \n", 
	MmioReadBe32((UINTN)&Regs->RxPktL));     /* Rx packets lower */
  DPAA1_DEBUG_MSG(" RxPktU %d \n", 
	MmioReadBe32((UINTN)&Regs->RxPktU));     /* Rx packets upper */
  DPAA1_DEBUG_MSG(" RxUndszL %d \n", 
	MmioReadBe32((UINTN)&Regs->RxUndszL));   /* Rx undersized packet lower */
  DPAA1_DEBUG_MSG(" RxUndszU %d \n", 
	MmioReadBe32((UINTN)&Regs->RxUndszU));   /* Rx undersized packet upper */
  DPAA1_DEBUG_MSG(" RxOverszL %d \n", 
	MmioReadBe32((UINTN)&Regs->RxOverszL));  /* Rx oversized packet lower */
  DPAA1_DEBUG_MSG(" RxOverszU %d \n", 
	MmioReadBe32((UINTN)&Regs->RxOverszU));  /* Rx oversized packet upper */
  DPAA1_DEBUG_MSG(" RxJabberL %d \n", 
	MmioReadBe32((UINTN)&Regs->RxJabberL));  /* Rx Jabber packet lower */
  DPAA1_DEBUG_MSG(" RxJabberU %d \n", 
	MmioReadBe32((UINTN)&Regs->RxJabberU));  /* Rx Jabber packet upper */

  DPAA1_DEBUG_MSG(" ------------- DUMPING TX STATISTICS -------------\n");
  DPAA1_DEBUG_MSG(" TxFrameL %d \n", 
	MmioReadBe32((UINTN)&Regs->TxFrameL));   /* Tx frame counter lower */
  DPAA1_DEBUG_MSG(" TxFrameU %d \n", 
	MmioReadBe32((UINTN)&Regs->TxFrameU));   /* Tx frame counter upper */
  DPAA1_DEBUG_MSG(" TxFrameCrcErrL %d \n", 
	MmioReadBe32((UINTN)&Regs->TxFrameCrcErrL)); /* Tx frame check sequence error lower */
  DPAA1_DEBUG_MSG(" TxFrameCrcErrU %d \n", 
	MmioReadBe32((UINTN)&Regs->TxFrameCrcErrU)); /* Tx frame check sequence error upper */
  DPAA1_DEBUG_MSG(" TxFrameErrL %d \n", 
	MmioReadBe32((UINTN)&Regs->TxFrameErrL));      /* Tx frame error lower */
  DPAA1_DEBUG_MSG(" TxFrameErrU %d \n", 
	MmioReadBe32((UINTN)&Regs->TxFrameErrU));      /* Tx frame error upper */
  DPAA1_DEBUG_MSG(" TxUndszL %d \n", 
	MmioReadBe32((UINTN)&Regs->TxUndszL));   /* Tx undersized packet lower */
  DPAA1_DEBUG_MSG(" TxUndszU %d \n", 
	MmioReadBe32((UINTN)&Regs->TxUndszU));   /* Tx undersized packet upper */
}

/**
   Initializes a given FMAN DPAA1 MAC. If the MEMAC is enabled,
   it is added to the specified list of enabled MEMACs

   @param[in] MeMacId	        DPAA1 MAC Id
   @param[in] LaneProtocol	        SerDes lane protocol for the PHY associated
                                    with the given DPAA1 MAC
   @param[in] PhyInterfaceType      Ethernet PHY interface type
   @param[in] MdioBus               Pointer to the MDIO bus that connects the DMPAC to a PHY
   @param[in] PhyAddress            PHY addRess on the MDIO bus
   @param[in] EnabledMemacList      Pointer to the head of the list of enabled MEMACs

   @retval None

 **/

VOID
FmanMemacInit(
  IN  FMAN_MEMAC_ID MemacId,
  IN  PHY_INTERFACE_TYPE PhyInterfaceType,
  IN  DPAA1_PHY_MDIO_BUS *MdioBus,
  IN  UINT8 PhyAddress,
  OUT LIST_ENTRY *EnabledMemacsList
)
{
  STATIC FMAN_MEMAC *Memac;

  /* Initialize with default values */
  Memac = &gFmanMemacs[MemacId];

  ASSERT(Memac->Id == MemacId);
  Memac->Enabled = IsMemacEnabled(MemacId);
  Memac->Phy.PhyInterfaceType = PhyInterfaceType;
  Memac->Phy.MdioBus = MdioBus;
  Memac->Phy.PhyAddress = PhyAddress;

  DPAA1_INFO_MSG(
    "Memac %a PHY type %a, %a\n",
    gFmanMemacStrings[MemacId],
    gPhyInterfaceTypeStrings[PhyInterfaceType],
    Memac->Enabled ? "Enabled" : "Disabled");

  if (Memac->Enabled) {
    InitializeListHead(&Memac->ListNode);
    InsertTailList(EnabledMemacsList, &Memac->ListNode);
  }
}

CONST CHAR8 *
PhyInterfaceTypeToString(PHY_INTERFACE_TYPE PhyInterfaceType)
{
  if (PhyInterfaceType < NUM_PHY_INTERFACE_TYPES) {
    return gPhyInterfaceTypeStrings[PhyInterfaceType];
  } else {
    return "Unknown";
  }
}

/**
  Fix the Memac state in Fdt (Flattened Device tree)
  Update the status field to "okay" or "disabled"
  Update the phy-interface-type field.

  @param[in]  Fdt   Fdt blob to fixup

  @retval EFI_SUCCESS       Successfully fixed the device tree
  @retval EFI_DEVICE_ERROR  Failed to fix the device tree
**/
EFI_STATUS
FdtMemacFixup (
  IN  VOID  *Fdt
)
{
  UINTN       Index;
  INTN        FdtStatus;
  EFI_STATUS  Status;
  INT32       NodeOffset;
  FMAN_CCSR   *FmanCcsr;
  UINT64      MemacOffset;
  UINT64      Regs;
  UINT32      Phandle;

  FmanCcsr = (FMAN_CCSR *)PcdGet64(PcdDpaa1FmanAddr);

  for (Index = 0; Index < ARRAY_SIZE (gFmanMemacs); Index++) {
    if (gFmanMemacs[Index].Id == INVALID_FMAN_MEMAC_ID) {
      continue;
    }
    MemacOffset = (UINT64)(&FmanCcsr->memac[gFmanMemacs[Index].Id - 1].FmanMemac) - (UINT64)FmanCcsr;

    // Get the node
    for (NodeOffset = fdt_node_offset_by_compatible (Fdt, -1, "fsl,fman-memac");
         NodeOffset != -FDT_ERR_NOTFOUND;
         NodeOffset = fdt_node_offset_by_compatible (Fdt, NodeOffset, "fsl,fman-memac")) {
      Status = FdtGetAddressSize (Fdt, NodeOffset, "reg", 0, &Regs, NULL);
      if (EFI_ERROR (Status)) {
        DEBUG ((DEBUG_ERROR, "Error: can't get regs base address(Status = %r) for Memac Id %d!\n",
                Status, gFmanMemacs[Index].Id));
        continue;
      }
      if (Regs == MemacOffset) {
        break;
      }
    }
    if (NodeOffset == -FDT_ERR_NOTFOUND) {
      DEBUG ((DEBUG_ERROR, "Error: can't get node for Memac Id %d!\n", gFmanMemacs[Index].Id));
      continue;
    }

    // Disable or enable the node
    if (gFmanMemacs[Index].Enabled) {
      FdtStatus = fdt_setprop_string (Fdt, NodeOffset, "status", "okay");
      if (FdtStatus) {
        DEBUG ((DEBUG_ERROR, "could not set property %a for node %a error %a", "status",
                fdt_strerror(FdtStatus), fdt_get_name (Fdt, NodeOffset, NULL)));
        return EFI_DEVICE_ERROR;
      }

      // Set the phy interface string
      if (gFmanMemacs[Index].Phy.PhyInterfaceType != PHY_INTERFACE_NONE) {
        FdtStatus = fdt_setprop_string (Fdt, NodeOffset, "phy-connection-type",
                                        PhyInterfaceTypeToString (gFmanMemacs[Index].Phy.PhyInterfaceType));
        if (FdtStatus) {
          DEBUG ((DEBUG_ERROR, "could not set property %a for node %a error %a", "status",
                  fdt_strerror(FdtStatus), fdt_get_name (Fdt, NodeOffset, NULL)));
          return EFI_DEVICE_ERROR;
        }
      }
    } else {
      /* disable the MAC node */
      FdtStatus = fdt_setprop_string (Fdt, NodeOffset, "status", "disabled");
      if (FdtStatus) {
        DEBUG ((DEBUG_ERROR, "could not set property %a for node %a error %a", "phy-connection-type",
                fdt_strerror(FdtStatus), fdt_get_name (Fdt, NodeOffset, NULL)));
        return EFI_DEVICE_ERROR;
      }
      /* disable the fsl,dpa-ethernet node that points to the MAC */
      Phandle = fdt_get_phandle (Fdt, NodeOffset);
      for (NodeOffset = fdt_node_offset_by_prop_value (Fdt, -1, "fsl,fman-mac", &Phandle, sizeof (Phandle));
           NodeOffset != -FDT_ERR_NOTFOUND;
           NodeOffset = fdt_node_offset_by_prop_value (Fdt, NodeOffset, "fsl,fman-mac", &Phandle, sizeof (Phandle))) {
        FdtStatus = fdt_setprop_string (Fdt, NodeOffset, "status", "disabled");
        if (FdtStatus) {
          DEBUG ((DEBUG_ERROR, "could not set property %a for node %a error %a", "status",
                  fdt_strerror(FdtStatus), fdt_get_name (Fdt, NodeOffset, NULL)));
          return EFI_DEVICE_ERROR;
        }
      }
    }
  }

  return EFI_SUCCESS;
}

VOID InitializeMac (
  IN  ENET_MAC *Mac
  )
{
  Memac *Regs = Mac->Base;

  /* mask all interrupt */
  MmioWriteBe32((UINTN)&Regs->Imask, IMASK_MASK_ALL);

  /* clear all events */
  MmioWriteBe32((UINTN)&Regs->Ievent, IEVENT_CLEAR_ALL);

  /* set the max receive length */
  MmioWriteBe32((UINTN)&Regs->Maxfrm, MEMAC_MAXFRM & MAXFRM_MASK);

  /* multicast frame reception for the hash entry disable */
  MmioWriteBe32((UINTN)&Regs->HashtableCtrl, 0);
}

VOID SetInterface (
  IN  ENET_MAC *Mac,
  IN  PHY_INTERFACE_TYPE Type,
  IN  INT32 Speed
  )
{
  Memac *Regs = Mac->Base;
  UINT32 Mode, Status;

  /* clear all bits relative with interface mode */
  Mode = MmioReadBe32((UINTN)&Regs->IfMode);
  Status = MmioReadBe32((UINTN)&Regs->IfStatus);

  switch (Type) {
  case PHY_INTERFACE_SGMII:
  case PHY_INTERFACE_QSGMII:
         Mode &= ~IF_MODE_MASK;
         Mode |= (IF_MODE_GMII);
         break;
  case PHY_INTERFACE_RGMII:
         Mode |= (IF_MODE_GMII | IF_MODE_RGMII);
         break;
  case PHY_INTERFACE_XFI:
         Mode &= ~IF_MODE_MASK;
         Mode |= IF_MODE_XFI;
         break;
  default:
         break;
  }
  /* Enable automatic speed selection for Non-XFI */
  if (Type != PHY_INTERFACE_XFI)
         Mode |= IF_MODE_EN_AUTO;
  if (Type == PHY_INTERFACE_RGMII) {
        Mode &= ~IF_MODE_EN_AUTO;
        Mode &= ~IF_MODE_SETSP_MASK;
        switch (Speed) {
        case 1000:
                Mode |= IF_MODE_SETSP_1000M;
                break;
        case 100:
                Mode |= IF_MODE_SETSP_100M;
                break;
        case 10:
                Mode |= IF_MODE_SETSP_10M;
        default:
                break;
        }
  }

  DPAA1_DEBUG_MSG(" IfMode = %x\n",Mode);
  DPAA1_DEBUG_MSG(" IfStatus = %x\n", Status);

  MmioWriteBe32((UINTN)&Regs->IfMode, Mode);
  Status = MmioReadBe32((UINTN)&Regs->IfStatus);

  return;
}

VOID
TxGracefulStopEnable (
  IN ETH_DEVICE *FmanEthDevice
  )
{
  FMAN_GLOBAL_PARAM *Pram;

  Pram = FmanEthDevice->TxPram;
  /* Enable graceful stop for TX */
  MmioSetBitsBe32((UINTN)&Pram->Mode, PARAM_MODE_GRACEFUL_STOP);
  MemoryFence();
}

VOID
TxGracefulStopDisable (
  IN ETH_DEVICE *FmanEthDevice
  )
{
  FMAN_GLOBAL_PARAM *Pram;

  Pram = FmanEthDevice->TxPram;
  /* re-enable transmission of frames */
  MmioClearBitsBe32((UINTN)&Pram->Mode, PARAM_MODE_GRACEFUL_STOP);
  MemoryFence();
}

VOID
DisableMac (
  ENET_MAC *Mac
)
{
  Memac *Regs = Mac->Base;

  MmioClearBitsBe32((UINTN)&Regs->CommandConfig, MEMAC_CMD_CFG_RXTX_EN);
  MmioSetBitsBe32((UINTN)&Regs->CommandConfig, MEMAC_CMD_CFG_SWR);
}

VOID
EnableMac (
  ENET_MAC *Mac
)
{
  Memac *Regs = Mac->Base;

  MmioSetBitsBe32((UINTN)&Regs->CommandConfig, MEMAC_CMD_CFG_SWR);
  MmioSetBitsBe32((UINTN)&Regs->StatnConfig, MEMAC_CMD_CFG_CLR_STATS);
  MmioSetBitsBe32((UINTN)&Regs->CommandConfig,
      MEMAC_CMD_CFG_RXTX_EN | MEMAC_CMD_CFG_PAD );
  MmioClearBitsBe32((UINTN)&Regs->CommandConfig, MEMAC_CMD_CFG_NO_LEN_CHK);
}

VOID
DisablePorts (
  IN  ETH_DEVICE *FmanEthDevice
  )
{
  UINT32 Timeout = 1000000;

  /* disable bmi Tx port */
  MmioClearBitsBe32((UINTN)&FmanEthDevice->TxPort->FmanBmTcfg, FMAN_BM_TCFG_EN);

  /* wait until the tx port is not busy */
  while ((MmioReadBe32((UINTN)&FmanEthDevice->TxPort->FmanBmTst) & FMAN_BM_TST_BSY)
		 && Timeout--)
              ;

  /* disable MAC rx/tx port */
  DisableMac(FmanEthDevice->Mac);

  /* disable bmi Rx port */
  Timeout = 1000000;

  MmioClearBitsBe32((UINTN)&FmanEthDevice->RxPort->FmanBmRcfg, FMAN_BM_RCFG_EN);

  /* wait until the rx port is not busy */
  while ((MmioReadBe32((UINTN)&FmanEthDevice->RxPort->FmanBmRst) & FMAN_BM_RST_BSY)
		 && Timeout--)
         ;
}

VOID
EnablePorts (
  IN  ETH_DEVICE *FmanEthDevice
  )
{
  /* enable bmi Rx port */
  MmioSetBitsBe32((UINTN)&FmanEthDevice->RxPort->FmanBmRcfg, FMAN_BM_RCFG_EN);
  /* enable MAC rx/tx port */
  EnableMac(FmanEthDevice->Mac);
  /* enable bmi Tx port */
  MmioSetBitsBe32((UINTN)&FmanEthDevice->TxPort->FmanBmTcfg, FMAN_BM_TCFG_EN);
}

VOID
SetMacAddr (
  IN  ENET_MAC *Mac,
  IN  EFI_MAC_ADDRESS MAddr
  )
{
  Memac *Regs = Mac->Base;
  UINT32 Addr0, Addr1;

  /*
   * if a MAC addRess is 0x12345678ABCD, perform a write to
   * MAC_ADDR0 of 0x78563412,
   * MAC_ADDR1 of 0x0000CDAB
   */
  Addr0 = (MAddr.Addr[3] << 24) | (MAddr.Addr[2] << 16) | \
                (MAddr.Addr[1] << 8)  | (MAddr.Addr[0]);
  MmioWriteBe32((UINTN)&Regs->MacAddr_0, Addr0);

  Addr1 = ((MAddr.Addr[5] << 8) | MAddr.Addr[4]) & 0x0000ffff;
  MmioWriteBe32((UINTN)&Regs->MacAddr_1, Addr1);
}
