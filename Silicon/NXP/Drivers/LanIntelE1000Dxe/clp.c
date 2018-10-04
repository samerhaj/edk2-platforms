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
#include "clp.h"


EFI_GUID     gEfiPlatformToDriverConfigurationProtocolGuid = EFI_PLATFORM_TO_DRIVER_CONFIGURATION_PROTOCOL_GUID;

VOID
EepromMacAddressSet(
  IN GIG_DRIVER_DATA *GigAdapter,
  IN UINT16 *MacAddress
);


INTN
FindKey(
  IN CHAR8 *Key,
  IN EFI_CONFIGURE_CLP_PARAMETER_BLK *ClpBlock
  )
/*++

Routine Description:
  Find key in CLP command line.

Arguments:
  Key - Key to find on CLP command line.
  ClpBlock - Pointer to the CLP structure.

Returns:
  If key found, returns the index of the last character in the key.
  If key not found, returns -1.

--*/
{
  UINTN i = 0;
  UINTN j = 0;
  CHAR8 *CharPtr = ClpBlock->CLPCommand;

  DEBUGPRINT(CLP, ("FindKey()\n"));
  DEBUGPRINT(CLP, ("CLPCommandLength=%d \n", ClpBlock->CLPCommandLength));

  for (; i < ClpBlock->CLPCommandLength; i++) {
    if (Key[j] == CharPtr[i]) {
      DEBUGPRINT(CLP, ("Match j=%d i=%d  ", j, i));
      DEBUGPRINT(CLP, ("Key[j]=%c CharPtr[i]=%c\n", Key[j], CharPtr[i]));
      j++;
    }
    else
      j = 0;

    if (Key[j] == '\0' && j > 0) {
      j=0;
      DEBUGPRINT(CLP, ("Found key: "));
      while (Key[j] != '\0') {
        DEBUGPRINT(CLP, ("%c", Key[j]));
        j++;
      }
      DEBUGPRINT(CLP, ("\n"));
      break;
    }
  }

  if (i >= ClpBlock->CLPCommandLength) {
    j=0;
    DEBUGPRINT(CLP, ("Did not find key!: "));
    while (Key[j] != '\0') {
      DEBUGPRINT(CLP, ("%c", Key[j]));
      j++;
    }
    DEBUGPRINT(CLP, ("\n"));
    return -1;
  }
  DEBUGPRINT(CLP, ("Returning index=%d\n", i));
  return i;
}


BOOLEAN
ProcessAddressKey(
  IN GIG_DRIVER_DATA *GigAdapter,
  EFI_CONFIGURE_CLP_PARAMETER_BLK *ClpBlock
  )
/*++

Routine Description:
  Processes alternate MAC address with the "Address" key present.

Arguments:
  GigAdapter - Pointer to adapter structure
  ClpBlock - Pointer to the CLP structure.

Returns:
  FALSE - MAC Address not set.
  TRUE - Successfully set MAC Address.

--*/
{
  INTN i = -1;
  UINTN j = 0;
  CHAR8 *CharPtr = ClpBlock->CLPCommand;
  UINT8 Uint8 = 0;
  UINT16 MacAddr[3] = {0};
  UINT8 *MacAddress = (UINT8*) MacAddr;  // Necessary to preserve 16-bit alignment on IA64 for call to e1000_read_nvm

  //
  // Process MAC address
  //
  DEBUGPRINT(CLP, ("ProcessAddressKey()\n"));
  DEBUGPRINT(CLP, ("CLPCommandLength=%d \n", ClpBlock->CLPCommandLength));

  DEBUGPRINT(CLP, ("Search for equals sign\n"));
  i = FindKey("=", ClpBlock);
  if (i == -1) {
    return FALSE;
  }

  DEBUGPRINT(CLP, ("FindKey returned index=%d\n", i));

  while (CharPtr[i] != '\0') {
    if (CharPtr[i] >= '0' && CharPtr[i] <= '9') {
      Uint8 = CharPtr[i] - '0';
      DEBUGPRINT(CLP, ("%X ", Uint8));
    }
    else
    if (CharPtr[i] >= 'A' && CharPtr[i] <= 'F') {
      Uint8 = CharPtr[i] - 'A' + 10;
      DEBUGPRINT(CLP, ("%X ", Uint8));
    }
    else
    if (CharPtr[i] >= 'a' && CharPtr[i] <= 'f') {
      Uint8 = CharPtr[i] - 'a' + 10;
      DEBUGPRINT(CLP, ("%X ", Uint8));
    }
    else {
      i++;
      continue;
    }

    //
    // Assign the MAC address to the final value.
    //
    if (j & 1) {
      MacAddress[j/2] |= Uint8;
    }
    else
    {
      MacAddress[j/2] |= Uint8 << 4;
    }
    j++;
    if (j >= 12) {
      break;
    }
    i++;
  }

  DEBUGPRINT(CLP, ("\n"));

  if (j < 6) {
    DEBUGPRINT(CLP, ("Incomplete MAC address\n"));
    return FALSE;
  } else {
    DEBUGPRINT(CLP, ("MAC=%X-%X-%X-%X-%X-%X\n", MacAddress[0], MacAddress[1], MacAddress[2],
      MacAddress[3], MacAddress[4], MacAddress[5]));
  }

  DEBUGPRINT(CLP, ("Calling EepromMacAddressSet\n"));
  EepromMacAddressSet(GigAdapter, MacAddr);
  return TRUE;
}


BOOLEAN
ProcessClp(
  IN GIG_DRIVER_DATA *GigAdapter,
  EFI_CONFIGURE_CLP_PARAMETER_BLK *ClpBlock
  )
/*++

Routine Description:
  Processes CLP command line parameters.

Arguments:
  GigAdapter - Pointer to adapter structure
  ClpBlock - Pointer to the CLP structure.

Returns:
  PXE_STATCODE

--*/
{
  DEBUGPRINT(CLP, ("ProcessClp()\n"));

  DEBUGPRINT(CLP, ("Search for key\n"));
  if (FindKey("Address", ClpBlock) != -1) {
    return ProcessAddressKey(GigAdapter, ClpBlock);
  }
  return FALSE;
}


VOID
EepromMacAddressSet(
  IN GIG_DRIVER_DATA *GigAdapter,
  IN UINT16 *MacAddress
)
/*++

Routine Description:
  Sets the override MAC address to the default value.

Arguments:
  GigAdapter - Pointer to adapter structure
  MacAddress - Value to set the MAC address to.

Returns:
  VOID

--*/
{
  UINT16               MacOffset;
  UINT16               OldMacAddress[3];
#if (DBG_LVL&CLP)
  UINT8 *MacAddr = (UINT8*) MacAddress;
#endif

  DEBUGPRINT(CLP, ("MAC=%X-%X-%X-%X-%X-%X\n", MacAddr[0], MacAddr[1], MacAddr[2],
      MacAddr[3], MacAddr[4], MacAddr[5]));

  //
  // Determine the location of the card.
  //
  GigAdapter->PciIo->GetLocation (
    GigAdapter->PciIo,
    &GigAdapter->Segment,
    &GigAdapter->Bus,
    &GigAdapter->Device,
    &GigAdapter->Function
    );

  //
  // Read the address where the override MAC address is stored.
  //
  e1000_read_nvm (&GigAdapter->hw, NVM_ALT_MAC_ADDR_PTR, 1, &MacOffset);

  if (MacOffset == 0xFFFF) {
    DEBUGPRINT(CLP, ("Invalid offset for alt MAC address\n"));
    return;
  }

  DEBUGPRINT(CLP, ("MAC addresses at offset %X\n", MacOffset));

  //
  // Adjust the MAC address offset if this is the second port (function 1), or the
  // third or fourth port (Barton Hills)
  //
  MacOffset = MacOffset + (UINT16) (3*GigAdapter->Function);

  //
  // Read the current stored MAC address to see if it needs to be changed
  //
  e1000_read_nvm (&GigAdapter->hw, MacOffset, 3, OldMacAddress);

  if ((MacAddress[0] != OldMacAddress[0]) || (MacAddress[1] != OldMacAddress[1]) ||
    (MacAddress[2] != OldMacAddress[2])) {
    DEBUGPRINT(CLP, ("Updating MAC addresses in EEPROM\n"));
    e1000_write_nvm(&GigAdapter->hw, MacOffset, 3, MacAddress);
  } else {
    DEBUGPRINT(CLP, ("No need to update MAC addresses in EEPROM\n"));
  }

}


VOID
EepromMacAddressDefault(
  IN GIG_DRIVER_DATA *GigAdapter
)/*++

Routine Description:
  Sets the override MAC address back to FF-FF-FF-FF-FF-FF to disable.

Arguments:
  GigAdapter - Pointer to adapter structure

Returns:
  PXE_STATCODE

--*/
{
  UINT16               MacOffset;
  UINT16               MacAddress[3];
  UINT16               MacDefault[3] = {0xFFFF, 0xFFFF, 0xFFFF};

  //
  // Determine the location of the card.
  //
  GigAdapter->PciIo->GetLocation (
    GigAdapter->PciIo,
    &GigAdapter->Segment,
    &GigAdapter->Bus,
    &GigAdapter->Device,
    &GigAdapter->Function
    );

  //
  // Read the address where the override MAC address is stored.
  //
  e1000_read_nvm (&GigAdapter->hw, NVM_ALT_MAC_ADDR_PTR, 1, &MacOffset);

  if (MacOffset == 0xFFFF) {
    DEBUGPRINT(CLP, ("Invalid offset for alt MAC address\n"));
    return;
  }

  DEBUGPRINT(CLP, ("MAC addresses at offset %X\n", MacOffset));

  //
  // Adjust the MAC address offset if this is the second port (function 1)
  //
  MacOffset = MacOffset + (UINT16) (3*GigAdapter->Function);

  //
  // Read the current stored MAC address to see if it needs to be changed
  //
  e1000_read_nvm (&GigAdapter->hw, MacOffset, 3, (UINT16*) MacAddress);

  if ((MacAddress[0] != 0xFFFF) || (MacAddress[1] != 0xFFFF) || (MacAddress[2] != 0xFFFF)) {
    DEBUGPRINT(CLP, ("Setting default MAC addresses in EEPROM\n"));
    e1000_write_nvm(&GigAdapter->hw, MacOffset, 3, MacDefault);
  } else {
    DEBUGPRINT(CLP, ("No need to update MAC addresses in EEPROM\n"));
  }

}


EFI_STATUS
ClpEntry(
  IN EFI_DRIVER_BINDING_PROTOCOL  *This,
  IN EFI_HANDLE                   Controller,
  IN GIG_DRIVER_DATA *GigAdapter
  )
/*++

Routine Description:
  Entry point for EFI CLP code.  Searches for CLP interface and attempts to get CLP parameters.

Arguments:
  This - Handle to driver binding protocol
  Controller - Controller handle associated with the CLP instance
  GigAdapter - Pointer to adapter structure

Returns:
  EFI_STATUS

--*/
{
  EFI_PLATFORM_TO_DRIVER_CONFIGURATION_PROTOCOL *ClpProt;
  EFI_STATUS                                    Status;
  UINTN                                         Instance = 0;
  EFI_GUID                                      *ParameterTypeGuid;
  UINTN                                         ParameterBlockSize;
  EFI_PLATFORM_CONFIGURATION_ACTION             Action;
  EFI_CONFIGURE_CLP_PARAMETER_BLK               *ClpBlock;
  UINTN                                         HandleCount;
  EFI_HANDLE                                    *HandleBuffer;
  BOOLEAN                                       AltMacSet = FALSE;
  UINTN                                         i;

  DEBUGPRINT(CLP, ("ClpEntry()\n"));

  Status = gBS->LocateHandleBuffer (
    ByProtocol,
    &gEfiPlatformToDriverConfigurationProtocolGuid,
    NULL,
    &HandleCount,
    &HandleBuffer
    );

  if (EFI_ERROR(Status)) {
    DEBUGPRINT(CLP, ("LocateHandleBuffer returns %r\n", Status));
    return Status;
  }

  if (HandleCount == 0) {
    DEBUGPRINT(CLP, ("CLP Interface not found, restoring default MAC address.\n"));
    return EFI_NOT_FOUND;
  }

  DEBUGPRINT(CLP, ("HandleCount=%d\n", HandleCount));

  Status = gBS->OpenProtocol (
             HandleBuffer[0],
             &gEfiPlatformToDriverConfigurationProtocolGuid,
             (VOID **) &ClpProt,
             This->DriverBindingHandle,
             Controller,
             EFI_OPEN_PROTOCOL_GET_PROTOCOL
             );

  if (EFI_ERROR(Status)) {
    DEBUGPRINT(CLP, ("Error finding CLP protocol: %r\n", Status));
    return Status;
  }
  DEBUGPRINT(CLP, ("HandleCount=%d\n", HandleCount));

  while (Instance < 10) {
    DEBUGPRINT(CLP, ("Calling CLP->Query\n"));
    Status = ClpProt->Query(
                        ClpProt,
                        Controller,
                        NULL,
                        &Instance,
                        &ParameterTypeGuid,
                        (VOID*) &ClpBlock,
                        &ParameterBlockSize
                        );

    DEBUGPRINT(CLP, ("CLP Query returns: %r\n", Status));
    if (Status == EFI_NOT_FOUND || EFI_ERROR(Status)) {
      break;
    }

    DEBUGPRINT(CLP, ("Instance=%d\n", Instance));
    DEBUGPRINT(CLP, ("ParameterBlockSize=%d\n", ParameterBlockSize));

    DEBUGPRINT(CLP, ("CLPCommandLength=%d \n", ClpBlock->CLPCommandLength));
    DEBUGPRINT(CLP, ("CLPCommand="));
    for (i = 0; i < ClpBlock->CLPCommandLength; i++) {
      DEBUGPRINT(CLP, ("%c", ClpBlock->CLPCommand[i]));
    }
    DEBUGPRINT(CLP, ("\n"));

    DEBUGPRINT(CLP, ("Calling ProcessCLP\n"));
    if (ProcessClp(GigAdapter, ClpBlock) == TRUE) {
      DEBUGPRINT(CLP, ("Set Alt MAC address\n"));
      AltMacSet = TRUE;
    }

    Action = EfiPlatformConfigurationActionNone;
    ClpBlock->CLPReturnString = "Success";
    ClpBlock->CLPReturnStringLength = 8;
    ClpBlock->CLPCmdStatus = 0;
    ClpBlock->CLPErrorValue = 2;
    ClpBlock->CLPMsgCode = 0;

    DEBUGPRINT(CLP, ("Calling CLP Response @ %X...", ClpProt->Response));
    Status = ClpProt->Response (
                        ClpProt,
                        Controller,
                        NULL,
                        &Instance,
                        ParameterTypeGuid,
                        (VOID*) ClpBlock,
                        sizeof(EFI_CONFIGURE_CLP_PARAMETER_BLK),
                        Action
                        );

    DEBUGPRINT(CLP, ("CLP Response returns: %r\n", Status));
    if (EFI_ERROR(Status)) {
      break;
    }
    Instance++;
  }

  if (AltMacSet == FALSE) {
    DEBUGPRINT(CLP, ("Setting CLP back to default.\n"));
    EepromMacAddressDefault(GigAdapter);
  }

  gBS->CloseProtocol (
        Controller,
        &gEfiPlatformToDriverConfigurationProtocolGuid,
        This->DriverBindingHandle,
        Controller
        );

  DEBUGWAIT(CLP);

  return Status;
}

