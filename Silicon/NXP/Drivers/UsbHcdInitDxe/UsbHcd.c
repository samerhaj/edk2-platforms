/** @file

  Copyright 2017 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include <Bitops.h>
#include <Library/DebugLib.h>
#include <Library/IoLib.h>
#include <Library/NonDiscoverableDeviceRegistrationLib.h>

#include "UsbHcd.h"

STATIC
VOID
XhciSetBeatBurstLength (
  IN  UINTN  UsbReg
  )
{
  Dwc3       *Dwc3Reg;

  Dwc3Reg = (VOID *)(UsbReg + DWC3_REG_OFFSET);

  MmioAndThenOr32 ((UINTN)&Dwc3Reg->GSBusCfg0, ~USB3_ENABLE_BEAT_BURST_MASK,
                                              USB3_ENABLE_BEAT_BURST);
  MmioOr32 ((UINTN)&Dwc3Reg->GSBusCfg1, USB3_SET_BEAT_BURST_LIMIT);

  return;
}

STATIC
VOID
Dwc3SetFladj (
  IN  Dwc3   *Dwc3Reg,
  IN  UINT32 Val
  )
{
  MmioOr32 ((UINTN)&Dwc3Reg->GFLAdj, GFLADJ_30MHZ_REG_SEL |
                        GFLADJ_30MHZ(Val));
}

VOID
Dwc3SetMode (
  IN  Dwc3   *Dwc3Reg,
  IN  UINT32 Mode
  )
{
  MmioAndThenOr32 ((UINTN)&Dwc3Reg->GCtl,
               ~(DWC3_GCTL_PRTCAPDIR(DWC3_GCTL_PRTCAP_OTG)),
               DWC3_GCTL_PRTCAPDIR(Mode));
}

STATIC
VOID
Dwc3CoreSoftReset (
  IN  Dwc3   *Dwc3Reg
  )
{
  MmioOr32 ((UINTN)&Dwc3Reg->GCtl, DWC3_GCTL_CORESOFTRESET);
  MmioOr32 ((UINTN)&Dwc3Reg->GUsb3PipeCtl[0], DWC3_GUSB3PIPECTL_PHYSOFTRST);
  MmioOr32 ((UINTN)&Dwc3Reg->GUsb2PhyCfg, DWC3_GUSB2PHYCFG_PHYSOFTRST);
  MmioAnd32 ((UINTN)&Dwc3Reg->GUsb3PipeCtl[0], ~DWC3_GUSB3PIPECTL_PHYSOFTRST);
  MmioAnd32 ((UINTN)&Dwc3Reg->GUsb2PhyCfg, ~DWC3_GUSB2PHYCFG_PHYSOFTRST);
  MmioAnd32 ((UINTN)&Dwc3Reg->GCtl, ~DWC3_GCTL_CORESOFTRESET);

  return;
}

STATIC
EFI_STATUS
Dwc3CoreInit (
  IN  Dwc3   *Dwc3Reg
  )
{
  UINT32     Revision;
  UINT32     Reg;
  UINTN      Dwc3Hwparams1;

  Revision = MmioRead32 ((UINTN)&Dwc3Reg->GSnpsId);
  //
  // This should read as 0x5533, ascii of U3(DWC_usb3) followed by revision number
  //
  if ((Revision & DWC3_GSNPSID_MASK) != DWC3_SYNOPSIS_ID) {
    DEBUG ((DEBUG_ERROR,"This is not a DesignWare USB3 DRD Core.\n"));
    return EFI_NOT_FOUND;
  }

  Dwc3CoreSoftReset (Dwc3Reg);

  Reg = MmioRead32 ((UINTN)&Dwc3Reg->GCtl);
  Reg &= ~DWC3_GCTL_SCALEDOWN_MASK;
  Reg &= ~DWC3_GCTL_DISSCRAMBLE;

  Dwc3Hwparams1 = MmioRead32 ((UINTN)&Dwc3Reg->GHwParams1);

  if (DWC3_GHWPARAMS1_EN_PWROPT(Dwc3Hwparams1) == DWC3_GHWPARAMS1_EN_PWROPT_CLK) {
    Reg &= ~DWC3_GCTL_DSBLCLKGTNG;
  } else {
    DEBUG ((DEBUG_ERROR,"No power optimization available.\n"));
  }

  if ((Revision & DWC3_RELEASE_MASK) < DWC3_RELEASE_190a) {
    Reg |= DWC3_GCTL_U2RSTECN;
  }

  MmioWrite32 ((UINTN)&Dwc3Reg->GCtl, Reg);

  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
XhciCoreInit (
  IN  UINTN  UsbReg
  )
{
  EFI_STATUS Status;
  Dwc3       *Dwc3Reg;

  Dwc3Reg = (VOID *)(UsbReg + DWC3_REG_OFFSET);

  Status = Dwc3CoreInit (Dwc3Reg);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "Dwc3CoreInit Failed for controller 0x%x (0x%x) \n",
                  UsbReg, Status));
    return Status;
  }

  Dwc3SetMode (Dwc3Reg, DWC3_GCTL_PRTCAP_HOST);

  Dwc3SetFladj (Dwc3Reg, GFLADJ_30MHZ_DEFAULT);

  return Status;
}

STATIC
EFI_STATUS
EFIAPI
InitializeUsbController (
  IN  UINTN  UsbReg
  )
{
  EFI_STATUS Status;

  Status = XhciCoreInit (UsbReg);

  if (EFI_ERROR (Status)) {
    return Status;
  }

  //
  // Change beat burst and outstanding pipelined transfers requests
  //
  XhciSetBeatBurstLength (UsbReg);

  return Status;
}

/**
  The Entry Point of module. It follows the standard UEFI driver model.

  @param[in] ImageHandle   The firmware allocated handle for the EFI image.
  @param[in] SystemTable   A pointer to the EFI System Table.

  @retval EFI_SUCCESS      The entry point is executed successfully.
  @retval other            Some error occurs when executing this entry point.

**/
EFI_STATUS
EFIAPI
InitializeUsbHcd (
  IN EFI_HANDLE            ImageHandle,
  IN EFI_SYSTEM_TABLE      *SystemTable
  )
{
  EFI_STATUS               Status;
  UINT32                   NumUsbController;
  UINT32                   ControllerAddr;

  Status = EFI_SUCCESS;
  NumUsbController = PcdGet32 (PcdNumUsbController);

  while (NumUsbController) {
    NumUsbController--;
    ControllerAddr = PcdGet32 (PcdUsbBaseAddr) +
                     (NumUsbController * PcdGet32 (PcdUsbSize));

    Status = InitializeUsbController (ControllerAddr);
    if (EFI_ERROR (Status)) {
      DEBUG ((DEBUG_ERROR, "USB Controller initialization Failed for %d (0x%x)\n",
                            ControllerAddr, Status));
      continue;
    }

    Status = RegisterNonDiscoverableMmioDevice (
               NonDiscoverableDeviceTypeXhci,
               NonDiscoverableDeviceDmaTypeNonCoherent,
               NULL,
               NULL,
               1,
               ControllerAddr, PcdGet32 (PcdUsbSize)
             );

    if (EFI_ERROR (Status)) {
      DEBUG ((DEBUG_ERROR, "Failed to register USB device (0x%x) with error 0x%x \n",
                           ControllerAddr, Status));
    }
  }

  return Status;
}
