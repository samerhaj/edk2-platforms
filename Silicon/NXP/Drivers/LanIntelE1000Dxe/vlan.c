/**************************************************************************

Copyright (c) 2001-2010, Intel Corporation
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 3. Neither the name of the Intel Corporation nor the names of its
    contributors may be used to endorse or promote products derived from
    this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

***************************************************************************/

#include "e1000.h"

extern EFI_DRIVER_BINDING_PROTOCOL  mGigUndiDriverBinding;
extern EFI_GUID gEfiPro1000ComGuid;

EFI_GUID     gEfiVlanProtocolGuid = EFI_VLAN_GUID;

EFI_VLAN_PROTOCOL gGigUndiVlanData = {
  1,
  GigUndiSetVlanTag
};

EFI_STATUS
GigUndiSetVlanTag (
  IN      EFI_HANDLE              ControllerHandle,
  IN      BOOLEAN                 VlanEnable,
  IN      UINT16                  VlanId,
  IN      UINT16                  VlanPriority,
  IN      BOOLEAN                 VlanCfi
  )
/*++

Routine Description:
  Enables or disables 802.3Q VLAN tagging on the specified network interface.

Arguments:
  ControllerHandle - The handle to the network interface to configure VLAN tagging.
  VlanEnable       - Enable or disable 802.3Q ethernet header and VLAN tag insertion
  VlanTag          - Vlan tag to insert into each 802.3Q packet.

Returns:
  EFI_SUCCESS on success, appropriate EFI status code on failure

--*/
{
  EFI_STATUS                                Status;
  GIG_UNDI_PRIVATE_DATA                     *GigUndiPrivateData;
  EFI_PRO1000_COM_PROTOCOL                  *EfiPro1000Com;
  UINT32                                    Reg;

  if (VlanId > 0x0FFF) {
    DEBUGPRINT (VLAN, ("VlanId parameter out of range.\n"));
    return EFI_INVALID_PARAMETER;
  }

  if (VlanPriority > 7) {
    DEBUGPRINT (VLAN, ("VlanPriority parameter out of range.\n"));
    return EFI_INVALID_PARAMETER;
  }

  if (VlanCfi > 1) {
    DEBUGPRINT (VLAN, ("VlanCfi parameter out of range.\n"));
    return EFI_INVALID_PARAMETER;
  }

  //
  //  Open an instance for the Network Interface Identifier Protocol so we can check
  // if the child handle interface is actually valid.
  //
  DEBUGPRINT (VLAN, ("Open an instance for the Network Interface Identifier Protocol\n"));
  Status = gBS->OpenProtocol (
                  ControllerHandle,
                  &gEfiPro1000ComGuid,
                  (VOID **) &EfiPro1000Com,
                  mGigUndiDriverBinding.DriverBindingHandle,
                  NULL,
                  EFI_OPEN_PROTOCOL_GET_PROTOCOL
                  );

  if (EFI_ERROR (Status)) {
    DEBUGPRINT(CRITICAL, ("2. OpenProtocol returns %r\n", Status));
    DEBUGWAIT (CRITICAL);
    return Status;
  }

  GigUndiPrivateData = GIG_UNDI_PRIVATE_DATA_FROM_THIS (EfiPro1000Com->NIIProtocol_31);

  DEBUGPRINT (VLAN, ("Subsystem Vendor ID = %X\n", GigUndiPrivateData->NicInfo.hw.subsystem_vendor_id));
  DEBUGPRINT (VLAN, ("Vendor ID = %X\n", GigUndiPrivateData->NicInfo.hw.vendor_id));
  DEBUGPRINT (VLAN, ("Device ID = %X\n", GigUndiPrivateData->NicInfo.hw.device_id));

  Reg = E1000_READ_REG (&GigUndiPrivateData->NicInfo.hw, E1000_CTRL);
  if (VlanEnable) {
    GigUndiPrivateData->NicInfo.VlanEnable = TRUE;
    GigUndiPrivateData->NicInfo.VlanTag = VlanId | (VlanCfi << 12) | (VlanPriority << 13);
    DEBUGPRINT (VLAN, ("VlanTag = %X\n", GigUndiPrivateData->NicInfo.VlanTag));
    Reg |= E1000_CTRL_VME;
    DEBUGPRINT (VLAN, ("VME in CTRL register enabled\n"));
  } else {
    GigUndiPrivateData->NicInfo.VlanEnable = FALSE;
    GigUndiPrivateData->NicInfo.VlanTag = 0;
    Reg &= ~E1000_CTRL_VME;
    DEBUGPRINT (VLAN, ("VME in CTRL register disabled\n"));
  }
  E1000_WRITE_REG (&GigUndiPrivateData->NicInfo.hw, E1000_CTRL, Reg);
  DEBUGPRINT (VLAN, ("E1000_CTRL=%X\n", Reg));
  DEBUGPRINT (VLAN, ("Vlan setting complete.\n"));

  Reg = E1000_READ_REG (&GigUndiPrivateData->NicInfo.hw, E1000_VET);
  DEBUGPRINT (VLAN, ("E1000_VET=%X\n", Reg));

  return Status;
}


