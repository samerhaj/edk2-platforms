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


#ifndef _VLAN_H
#define _VLAN_H

#define EFI_VLAN_GUID \
  { \
    0xe1ad94a, 0xdcf4, 0x11db, \
    { \
      0x97, 0x5, 0x0, 0xe0, 0x81, 0x61, 0x16, 0x5f \
    } \
  }

typedef EFI_STATUS
(EFIAPI *EFI_SET_VLAN_TAG) (
        IN      EFI_HANDLE              ControllerHandle,
        IN      BOOLEAN                 VlanEnable,
        IN      UINT16                  VlanId,
        IN      UINT16                  VlanPriority,
        IN      BOOLEAN                 VlanCfi
        );

typedef struct _EFI_VLAN_PROTOCOL {
  UINT16             Version;
  EFI_SET_VLAN_TAG   SetVlanTag;
} EFI_VLAN_PROTOCOL;

extern EFI_GUID gEfiVlanProtocolGuid;

extern EFI_VLAN_PROTOCOL gGigUndiVlanData;

EFI_STATUS
GigUndiSetVlanTag (
  IN      EFI_HANDLE              ControllerHandle,
  IN      BOOLEAN                 VlanEnable,
  IN      UINT16                  VlanId,
  IN      UINT16                  VlanPriority,
  IN      BOOLEAN                 VlanCfi
  );

#endif
