/*++
Copyright (c) 2004-2010, Intel Corporation
All rights reserved. This program and the accompanying materials
are licensed and made available under the terms and conditions of the BSD License
which accompanies this distribution.  The full text of the license may be found at
http://opensource.org/licenses/bsd-license.php

THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

Module Name:
  Hii.c

Abstract:

  This is an example of how a driver might export data to the HII protocol to be
  later utilized by the Setup Protocol

--*/

#include "e1000.h"
#include "hii.h"

EFI_GUID   mHiiFormGuid = E1000_HII_FORM_GUID;
EFI_GUID   mE1000DataGuid = E1000_HII_DATA_GUID;

#if 0
STRING_TOKEN (STR_TERMINATION_STRING);
#endif

extern EFI_DRIVER_BINDING_PROTOCOL mGigUndiDriverBinding;

CHAR16     VariableName[] = L"GigNVData";

UINT32     gGuidInstance = 0;

EFI_STATUS
GigUndiComponentNameGetDriverName (
  IN  EFI_COMPONENT_NAME_PROTOCOL  *This,
  IN  CHAR8                        *Language,
  OUT CHAR16                       **DriverName
  );


VOID
SetWakeOnLan (
  GIG_UNDI_PRIVATE_DATA     *GigUndiPrivateData,
  IN UINT8                  Enable
  );

VOID
GetWakeOnLanStatus (
  GIG_UNDI_PRIVATE_DATA     *GigUndiPrivateData
  );

VOID
GetWakeOnLanSupport (
  GIG_UNDI_PRIVATE_DATA     *GigUndiPrivateData
  );

BOOLEAN
GetFlashEnableStatus (
      GIG_UNDI_PRIVATE_DATA     *GigUndiPrivateData
);

EFI_STATUS
GetFlashEnableSupport (
      GIG_UNDI_PRIVATE_DATA     *GigUndiPrivateData
);

EFI_STATUS
SetFlashEnable (
      GIG_UNDI_PRIVATE_DATA     *GigUndiPrivateData,
      BOOLEAN					Enable
);



VOID
e1000_MemCopy (
    IN UINT8* Dest,
    IN UINT8* Source,
    IN UINT32 Count
    );

UINTN
GetLanSpeedStatus(
	GIG_UNDI_PRIVATE_DATA     *GigUndiPrivateData
)
{
  UINTN         Active;
  UINT16        SetupOffset;
  UINT16        ConfigOffset;
  UINT16        SetupWord;
  UINT16        CustomConfigWord;

  struct e1000_hw *hw = &GigUndiPrivateData->NicInfo.hw;

  //
  //   For media other than copper we do not support speed settings
  //

  if (GigUndiPrivateData->NicInfo.hw.phy.media_type != e1000_media_type_copper)
	  return LINK_SPEED_AUTO_NEG;

  // If the device is a dual port device then we need to use the EEPROM settings
  // for the second adapter port
  //
  switch(GigUndiPrivateData->NicInfo.Function) {
	case 0:
	  ConfigOffset = CONFIG_CUSTOM_WORD;
	  SetupOffset  = SETUP_OPTIONS_WORD;
	  break;
	case 1:
	  ConfigOffset = CONFIG_CUSTOM_WORD_LANB;
	  SetupOffset  = SETUP_OPTIONS_WORD_LANB;
	  break;
	case 2:
	  ConfigOffset = CONFIG_CUSTOM_WORD_LANC;
	  SetupOffset  = SETUP_OPTIONS_WORD_LANC;
	  break;
	case 3:
	  ConfigOffset = CONFIG_CUSTOM_WORD_LAND;
	  SetupOffset  = SETUP_OPTIONS_WORD_LAND;
	  break;
	default:
	  ConfigOffset = CONFIG_CUSTOM_WORD;
	  SetupOffset  = SETUP_OPTIONS_WORD;
  }

  e1000_read_nvm(hw, SetupOffset, 1, &SetupWord);

  //
  // Save the original setup word value so we can tell if the user changed it
  //
  GigUndiPrivateData->NicInfo.BackupSetupWord = SetupWord;

  //
  // If the boot agent EEPROM signature is not set properly then we will initialize
  // the words to default values and assume a default autonegotiation setting
  //
  e1000_read_nvm(hw, ConfigOffset, 1, &CustomConfigWord);


  if ((CustomConfigWord & SIG_MASK) != SIG) {
	CustomConfigWord = SIG;
	SetupWord = DISPLAY_SETUP_MESSAGE;
	Active = LINK_SPEED_AUTO_NEG;
  } else {
	//
	// The signature bits are set so get the speed duplex settings
	// Mask of the speed and duplex setting bits so that we can determine
	// what the settings are
	//
	switch (SetupWord & (FSP_MASK | FDP_FULL_DUPLEX_BIT))
	{
	case (FDP_FULL_DUPLEX_BIT | FSP_100MBS):
	  hw->mac.autoneg              = 0;
	  hw->mac.forced_speed_duplex  = ADVERTISE_100_FULL;
	  Active = LINK_SPEED_100FULL;
	  break;
	case (FDP_FULL_DUPLEX_BIT | FSP_10MBS):
	  hw->mac.autoneg              = 0;
	  hw->mac.forced_speed_duplex  = ADVERTISE_10_FULL;
	  Active = LINK_SPEED_10FULL;
	  break;
	case (FSP_100MBS):
	  hw->mac.autoneg              = 0;
	  hw->mac.forced_speed_duplex  = ADVERTISE_100_HALF;
	  Active = LINK_SPEED_100HALF;
	  break;
	case (FSP_10MBS):
	  hw->mac.autoneg              = 0;
	  hw->mac.forced_speed_duplex  = ADVERTISE_10_HALF;
	  Active = LINK_SPEED_10HALF;
	  break;
	default:
	  hw->mac.autoneg              = 1;
	  Active = LINK_SPEED_AUTO_NEG;
	}
  }

  GigUndiPrivateData->NicInfo.SetupWord = SetupWord;
  GigUndiPrivateData->NicInfo.CustomConfigWord = CustomConfigWord;

  return Active;
}

VOID
SetLanSpeed(
  GIG_UNDI_PRIVATE_DATA     *GigUndiPrivateData,
  UINT8	LanSpeed
)
{
  UINT16        SetupOffset;
  UINT16        ConfigOffset;
  UINT16        SetupWord;
  UINT16        CustomConfigWord;
  UINT8         ReceiveStarted;

  struct e1000_hw *hw = &GigUndiPrivateData->NicInfo.hw;

  //
  // Configure offsets depending on function number
  //
  switch(GigUndiPrivateData->NicInfo.Function) {
	case 0:
	  ConfigOffset = CONFIG_CUSTOM_WORD;
	  SetupOffset  = SETUP_OPTIONS_WORD;
	  break;
	case 1:
	  ConfigOffset = CONFIG_CUSTOM_WORD_LANB;
	  SetupOffset  = SETUP_OPTIONS_WORD_LANB;
	  break;
	case 2:
	  ConfigOffset = CONFIG_CUSTOM_WORD_LANC;
	  SetupOffset  = SETUP_OPTIONS_WORD_LANC;
	  break;
	case 3:
	  ConfigOffset = CONFIG_CUSTOM_WORD_LAND;
	  SetupOffset  = SETUP_OPTIONS_WORD_LAND;
	  break;
	default:
	  ConfigOffset = CONFIG_CUSTOM_WORD;
	  SetupOffset  = SETUP_OPTIONS_WORD;
  }

  SetupWord = GigUndiPrivateData->NicInfo.SetupWord;
  CustomConfigWord = GigUndiPrivateData->NicInfo.CustomConfigWord;

  switch (LanSpeed)
  {
  case LINK_SPEED_AUTO_NEG:
    //
    // Speed mask has already been cleared
    //
    SetupWord &= ~FSP_MASK;
    SetupWord |= (FSP_AUTONEG);
    hw->mac.autoneg  = 1;
    break;
  case LINK_SPEED_100FULL:
    SetupWord &= ~FSP_MASK;
    SetupWord |= (FDP_FULL_DUPLEX_BIT | FSP_100MBS);
    hw->mac.autoneg  = 0;
    hw->mac.forced_speed_duplex  = ADVERTISE_100_FULL;
    break;
  case LINK_SPEED_100HALF:
    SetupWord &= ~(FSP_MASK | FDP_FULL_DUPLEX_BIT);
    SetupWord |= FSP_100MBS;
    hw->mac.autoneg  = 0;
    hw->mac.forced_speed_duplex  = ADVERTISE_100_HALF;
    break;
  case LINK_SPEED_10FULL:
    SetupWord &= ~FSP_MASK;
    SetupWord |= (FDP_FULL_DUPLEX_BIT | FSP_10MBS);
    hw->mac.autoneg  = 0;
    hw->mac.forced_speed_duplex  = ADVERTISE_10_FULL;
    break;
  case LINK_SPEED_10HALF:
    SetupWord &= ~(FSP_MASK | FDP_FULL_DUPLEX_BIT);
    SetupWord |= FSP_10MBS;
    hw->mac.forced_speed_duplex  = ADVERTISE_10_HALF;
    hw->mac.autoneg  = 0;
    break;
  default:
    break;
  }

  //
  // Only write the EEPROM if the speed/duplex value has changed
  //
  if (SetupWord != GigUndiPrivateData->NicInfo.BackupSetupWord) {
	if (e1000_write_nvm (hw, ConfigOffset, 1, &CustomConfigWord) == E1000_SUCCESS) {
	  if (e1000_write_nvm (hw, SetupOffset, 1, &SetupWord) == E1000_SUCCESS) {
	    //
	    // Success
	    //
	    e1000_update_nvm_checksum (hw);
	    GigUndiPrivateData->NicInfo.BackupSetupWord = SetupWord;
	  }
	}
    //
    // After speed/duplex setting completes we need to perform a full reset of the adapter.
    // If the adapter was initialized on entry then force a full reset of the adapter.
    // Also reenable the receive unit if it was enabled before we started the PHY loopback test.
    //

	ReceiveStarted = GigUndiPrivateData->NicInfo.ReceiveStarted;

    e1000_reset_hw(&GigUndiPrivateData->NicInfo.hw);
    GigUndiPrivateData->NicInfo.HwInitialized = FALSE;
    if (GigUndiPrivateData->NicInfo.State == PXE_STATFLAGS_GET_STATE_INITIALIZED) {
      e1000_Inititialize (&GigUndiPrivateData->NicInfo);
	  DEBUGPRINT(HII, ("e1000_Inititialize complete\n"));
	  //
	  //  Restart the receive unit if it was running on entry
	  //
	  if (ReceiveStarted) {
	    DEBUGPRINT(HII, ("RESTARTING RU\n"));
	    e1000_ReceiveEnable(&GigUndiPrivateData->NicInfo);
	  }
    }
    DEBUGPRINT(HII, ("ADAPTER RESET COMPLETE\n"));
  }
}


EFI_STATUS
EFIAPI
ExtractConfig (
  IN  CONST EFI_HII_CONFIG_ACCESS_PROTOCOL   *This,
  IN  CONST EFI_STRING                       Request,
  OUT EFI_STRING                             *Progress,
  OUT EFI_STRING                             *Results
  )
/*++

  Routine Description:
    This function allows a caller to extract the current configuration for one
    or more named elements from the target driver.

  Arguments:
    This       - Points to the EFI_HII_CONFIG_ACCESS_PROTOCOL.
    Request    - A null-terminated Unicode string in <ConfigRequest> format.
    Progress   - On return, points to a character in the Request string.
                 Points to the string's null terminator if request was successful.
                 Points to the most recent '&' before the first failing name/value
                 pair (or the beginning of the string if the failure is in the
                 first name/value pair) if the request was not successful.
    Results    - A null-terminated Unicode string in <ConfigAltResp> format which
                 has all values filled in for the names in the Request string.
                 String to be allocated by the called function.

  Returns:
    EFI_SUCCESS           - The Results is filled with the requested values.
    EFI_OUT_OF_RESOURCES  - Not enough memory to store the results.
    EFI_INVALID_PARAMETER - Request is NULL, illegal syntax, or unknown name.
    EFI_NOT_FOUND         - Routing data doesn't match any storage in this driver.

--*/
{
  EFI_STATUS                       Status;
  UINTN                            BufferSize;
  GIG_UNDI_PRIVATE_DATA            *GigUndiPrivateData;

  DEBUGPRINT(HII, ("ExtractConfig\n"));
  GigUndiPrivateData = DRIVER_SAMPLE_PRIVATE_FROM_THIS (This);

  //
  // Get Buffer Storage data from EFI variable
  //
  BufferSize = sizeof (GIG_DRIVER_CONFIGURATION);
  Status = gRT->GetVariable (
                  VariableName,
                  &mE1000DataGuid,
                  NULL,
                  &BufferSize,
                  &GigUndiPrivateData->Configuration
                  );
  if (EFI_ERROR (Status)) {
    DEBUGPRINT(CRITICAL, ("GetVariable failed with %r\n", Status));
    DEBUGWAIT(CRITICAL);
    return Status;
  }

  // Get Wake on LAN settings

  GetWakeOnLanSupport(GigUndiPrivateData);
  GetWakeOnLanStatus(GigUndiPrivateData);

  DEBUGPRINT(HII, ("GetWakeOnLan %d\n", GigUndiPrivateData->Configuration.WolEnable));

  // Get Option ROM Enable status

  GigUndiPrivateData->Configuration.OptionRomEnable = GetFlashEnableStatus(GigUndiPrivateData);

  DEBUGPRINT(HII, ("GetFlashEnable %d\n", GigUndiPrivateData->Configuration.OptionRomEnable));

  //
  // Get link speed settings
  //
  GigUndiPrivateData->Configuration.LinkSpeed = GetLanSpeedStatus(GigUndiPrivateData);

  DEBUGPRINT(HII, ("GetLinkSpeed %d\n", GigUndiPrivateData->Configuration.LinkSpeed));

  //
  // Convert buffer data to <ConfigResp> by helper function BlockToConfig()
  //
  Status = GigUndiPrivateData->HiiConfigRouting->BlockToConfig (
                                GigUndiPrivateData->HiiConfigRouting,
                                Request,
                                (UINT8 *) &GigUndiPrivateData->Configuration,
                                sizeof(GIG_DRIVER_CONFIGURATION),
                                Results,
                                Progress
                                );
  if (EFI_ERROR (Status)) {
    DEBUGPRINT(CRITICAL, ("BlockToConfig failed with %r\n", Status));
    DEBUGWAIT(CRITICAL);
  }

  DEBUGWAIT(HII);
  return Status;
}

EFI_STATUS
EFIAPI
RouteConfig (
  IN  CONST EFI_HII_CONFIG_ACCESS_PROTOCOL   *This,
  IN  CONST EFI_STRING                       Configuration,
  OUT EFI_STRING                             *Progress
  )
/*++

  Routine Description:
    This function processes the results of changes in configuration.

  Arguments:
    This          - Points to the EFI_HII_CONFIG_ACCESS_PROTOCOL.
    Configuration - A null-terminated Unicode string in <ConfigResp> format.
    Progress      - A pointer to a string filled in with the offset of the most
                    recent '&' before the first failing name/value pair (or the
                    beginning of the string if the failure is in the first
                    name/value pair) or the terminating NULL if all was successful.

  Returns:
    EFI_SUCCESS           - The Results is processed successfully.
    EFI_INVALID_PARAMETER - Configuration is NULL.
    EFI_NOT_FOUND         - Routing data doesn't match any storage in this driver.

--*/
{
  EFI_STATUS                       Status;
  UINTN                            BufferSize;
  GIG_UNDI_PRIVATE_DATA            *GigUndiPrivateData;

  DEBUGPRINT(HII, ("RouteConfig\n"));
  GigUndiPrivateData = DRIVER_SAMPLE_PRIVATE_FROM_THIS (This);

  //
  // Get Buffer Storage data from EFI variable
  //
  BufferSize = sizeof (GIG_DRIVER_CONFIGURATION);
  Status = gRT->GetVariable (
                  VariableName,
                  &mE1000DataGuid,
                  NULL,
                  &BufferSize,
                  &GigUndiPrivateData->Configuration
                  );
  if (EFI_ERROR (Status)) {
    DEBUGPRINT(CRITICAL, ("GetVariable failed with %r\n", Status));
    DEBUGWAIT(CRITICAL);
    return Status;
  }

  //
  // Convert <ConfigResp> to buffer data by helper function ConfigToBlock()
  //
  BufferSize = sizeof (GIG_DRIVER_CONFIGURATION);
  Status = GigUndiPrivateData->HiiConfigRouting->ConfigToBlock (
                               GigUndiPrivateData->HiiConfigRouting,
                               Configuration,
                               (UINT8 *) &GigUndiPrivateData->Configuration,
                               &BufferSize,
                               Progress
                               );
  if (EFI_ERROR (Status)) {
    DEBUGPRINT(CRITICAL, ("ConfigToBlock returns %r\n", Status));
    DEBUGWAIT(CRITICAL);
    return Status;
  }
  DEBUGPRINT(HII, ("Set WakeOnLan %d\n", GigUndiPrivateData->Configuration.WolEnable));

  SetFlashEnable (
	GigUndiPrivateData,
	GigUndiPrivateData->Configuration.OptionRomEnable
    );

  SetWakeOnLan (
    GigUndiPrivateData,
    GigUndiPrivateData->Configuration.WolEnable
    );

  SetLanSpeed(
    GigUndiPrivateData,
    GigUndiPrivateData->Configuration.LinkSpeed
    );

  //
  // Store Buffer Storage back to EFI variable
  //
  Status = gRT->SetVariable(
                  VariableName,
                  &mE1000DataGuid,
                  EFI_VARIABLE_NON_VOLATILE | EFI_VARIABLE_BOOTSERVICE_ACCESS,
                  sizeof (GIG_DRIVER_CONFIGURATION),
                  &GigUndiPrivateData->Configuration
                  );
  if (EFI_ERROR (Status)) {
    DEBUGPRINT(CRITICAL, ("SetVariable failed with %r\n", Status));
    DEBUGWAIT(CRITICAL);
  }

  DEBUGWAIT(HII);
  return Status;
}

EFI_STATUS
GetFlashEnableInformation (
      GIG_UNDI_PRIVATE_DATA     *GigUndiPrivateData,
      UINT32                    *EepromWord,
      UINT16                    *EepromBitMask
    )
/*++

  Routine Description:
    This initializes eeprom addresss and bit mask for flash enable/disable operations.

  Arguments:

  Returns:
    EFI_SUCCESS         - The Results is processed successfully.
    EFI_UNSUPPORTED 	- flash enable/disable operations not supported on current port.

--*/
{
    UINT16        SetupWord;
    UINT32        Result;

    struct e1000_hw *hw = &GigUndiPrivateData->NicInfo.hw;

  //
  //  Check for LOM
  //
  Result = e1000_read_nvm(hw, COMPATIBILITY_WORD, 1, &SetupWord);

  if (Result != E1000_SUCCESS) return EFI_DEVICE_ERROR;

  if (SetupWord & COMPATABILITY_LOM_BIT) {
	DEBUGPRINT(HII, ("LOM - unsupported\n"));
	DEBUGWAIT(HII);
  	return EFI_UNSUPPORTED;
  }

  DEBUGPRINT(HII, ("\n\nGetFlashEnableInformation\n"));
  DEBUGPRINT(HII, ("Device ID: %x\n", GigUndiPrivateData->NicInfo.hw.device_id));
  DEBUGPRINT(HII, ("Function: %x\n", GigUndiPrivateData->NicInfo.Function));
  DEBUGWAIT(HII);

  switch (GigUndiPrivateData->NicInfo.hw.mac.type) {
#ifndef NO_82571_SUPPORT
      case e1000_82571:
      case e1000_82572:
          if (GigUndiPrivateData->NicInfo.Function == 0x1)
          {
              *EepromWord = E1000_INIT_CONTROL_WORD3_LANB;
              *EepromBitMask = E1000_FLASH_DISABLE_BIT;
          }
          else
          {
              *EepromWord = E1000_INIT_CONTROL_WORD3;
              *EepromBitMask = E1000_FLASH_DISABLE_BIT;
          }
      break;
      case e1000_82574:
          /* Read word containing NVM type bit out of NVM */
          Result = e1000_read_nvm(hw, E1000_FLASH_SIZE_WORD_HARTW, 1, &SetupWord);

          if (Result != E1000_SUCCESS) return EFI_DEVICE_ERROR;

          if ((GigUndiPrivateData->NicInfo.hw.device_id != E1000_DEV_ID_82574L &&
        		  GigUndiPrivateData->NicInfo.hw.device_id != E1000_DEV_ID_82574LA) ||
              ((SetupWord & E1000_NVM_TYPE_BIT_HARTW) != E1000_NVM_TYPE_BIT_HARTW))
          {
              return EFI_UNSUPPORTED;
          }
          *EepromWord = E1000_HARTW_FLASH_LAN_ADDRESS;
          *EepromBitMask = E1000_HARTW_EXP_ROM_DISABLE;
      break;
      case e1000_82583:
        break;
#endif
      case e1000_82573:
#ifndef NO_80003ES2LAN_SUPPORT
      case e1000_80003es2lan:
#endif
#ifndef NO_ICH8LAN_SUPPORT
      case e1000_ich8lan:
      case e1000_ich9lan:
#endif
      case e1000_pchlan:
    case e1000_pch2lan:
      return EFI_UNSUPPORTED;
    break;
#ifndef NO_82575_SUPPORT
      case e1000_82575:
      case e1000_82576:

    	  if (GigUndiPrivateData->NicInfo.hw.subsystem_vendor_id == 0x1734) // FSC
    	  {
    		  return EFI_UNSUPPORTED;
    	  }

          if (GigUndiPrivateData->NicInfo.Function == 0x01)
           {
               *EepromWord = E1000_INIT_CONTROL_WORD3_LANB;
           }
           else
           {
               *EepromWord = E1000_INIT_CONTROL_WORD3;
           }

          if ((GigUndiPrivateData->NicInfo.hw.revision_id == 0) &&
               (GigUndiPrivateData->NicInfo.hw.device_id != E1000_DEV_ID_82576))
           {
               *EepromBitMask = E1000_FLASH_DISABLE_BIT;
           }
           else
           {
               *EepromBitMask = E1000_FLASH_DISABLE_BIT_ZOAR;
           }

      break;
#endif /* NO_82575_SUPPORT */
      case e1000_82580:
          *EepromBitMask = E1000_FLASH_DISABLE_BIT_ZOAR;

           switch(GigUndiPrivateData->NicInfo.Function)
           {
           case 0x1:
               *EepromWord = (LAN1_BASE_ADDRESS_82580 + E1000_INIT_CONTROL_WORD3);
               break;

           case 0x2:
               *EepromWord = (LAN2_BASE_ADDRESS_82580 + E1000_INIT_CONTROL_WORD3);
               break;

           case 0x3:
               *EepromWord = (LAN3_BASE_ADDRESS_82580 + E1000_INIT_CONTROL_WORD3);
               break;

           default: //0x0
               *EepromWord = E1000_INIT_CONTROL_WORD3;
               break;
           }

      break;
      default:
    	  return EFI_UNSUPPORTED;
      break;
  }

  DEBUGPRINT(HII, ("EEPROM Word: %x\n", *EepromWord));
  DEBUGPRINT(HII, ("EepromBitMask: %x\n", *EepromBitMask));

  return EFI_SUCCESS;
}

EFI_STATUS
GetFlashEnableSupport (
      GIG_UNDI_PRIVATE_DATA     *GigUndiPrivateData
)
{
    UINT32                    	  EepromWord;
    UINT16                        EepromBitMask;

	return GetFlashEnableInformation(GigUndiPrivateData, &EepromWord, &EepromBitMask);
}

BOOLEAN
GetFlashEnableStatus (
      GIG_UNDI_PRIVATE_DATA     *GigUndiPrivateData
)
{
    UINT32                    	  EepromWord;
    UINT16                        EepromBitMask;

    UINT16						  ReadWord;

    struct e1000_hw *hw = &GigUndiPrivateData->NicInfo.hw;

    DEBUGPRINT(HII, ("GetFlashEnableStatus\n"));

    if (GetFlashEnableInformation(GigUndiPrivateData, &EepromWord, &EepromBitMask) != EFI_SUCCESS) {
      return FALSE;
    }

    e1000_read_nvm(hw, EepromWord, 1, &ReadWord);

    DEBUGPRINT(HII, ("EEPROM Word: %x\n", EepromWord));
    DEBUGPRINT(HII, ("EepromBitMask: %x\n", EepromBitMask));
    DEBUGPRINT(HII, ("ReadWord: %x\n", ReadWord));

    if (ReadWord & EepromBitMask){
    	return FALSE;  // Flash disabled
    } else {
    	return TRUE;   // Flash enabled
    }
}

EFI_STATUS
SetFlashEnable (
      GIG_UNDI_PRIVATE_DATA     *GigUndiPrivateData,
      BOOLEAN					Enable
)
{
	UINT32  EepromWord;
	UINT16  EepromBitMask;

	UINT16  ConfWord, ConfWordTmp;

	struct e1000_hw *hw = &GigUndiPrivateData->NicInfo.hw;

	GetFlashEnableInformation(GigUndiPrivateData, &EepromWord, &EepromBitMask);

	e1000_read_nvm(hw, EepromWord, 1, &ConfWord);

	if (Enable){
	  ConfWordTmp = ConfWord & ~EepromBitMask;
	} else {
	  ConfWordTmp = ConfWord | EepromBitMask;
	}

	if (ConfWordTmp != ConfWord) {
	  if (e1000_write_nvm (hw, EepromWord, 1, &ConfWordTmp) == E1000_SUCCESS)
		return EFI_SUCCESS;
	  else return EFI_DEVICE_ERROR;
	}
	else return EFI_SUCCESS;
}


EFI_STATUS
HiiOpenProtocol(
  GIG_UNDI_PRIVATE_DATA     *GigUndiPrivateData
  )
{
  EFI_STATUS                      Status;

  //
  // Initialize the Hii Database handle to NULL so we can check later
  // to see whether it was installed.
  //
  GigUndiPrivateData->HiiDatabase = NULL;
  GigUndiPrivateData->HiiString = NULL;
  GigUndiPrivateData->FormBrowser2 = NULL;
  GigUndiPrivateData->HiiConfigRouting = NULL;

  DEBUGPRINT(HII, ("Locate HII Protocol\n"));
  //
  // Locate Hii Database protocol
  //
  Status = gBS->LocateProtocol (
      &gEfiHiiDatabaseProtocolGuid,
      NULL,
      (VOID**)&GigUndiPrivateData->HiiDatabase
      );
  if (EFI_ERROR (Status)) {
    DEBUGPRINT(CRITICAL, ("Error finding HII protocol: %r\n", Status));
    DEBUGWAIT(CRITICAL);
    return Status;
  }

  //
  // Locate HiiString protocol
  //
  DEBUGPRINT(HII, ("Locate HII String Protocol\n"));
  Status = gBS->LocateProtocol (
      &gEfiHiiStringProtocolGuid,
      NULL,
      (VOID**)&GigUndiPrivateData->HiiString
      );
  if (EFI_ERROR (Status)) {
    DEBUGPRINT(CRITICAL, ("Error finding HII String protocol: %r\n", Status));
    DEBUGWAIT(CRITICAL);
    return Status;
  }

  //
  // Locate Formbrowser2 protocol
  //
  DEBUGPRINT(HII, ("Locate HII Form Browser Protocol\n"));
  Status = gBS->LocateProtocol (
      &gEfiFormBrowser2ProtocolGuid,
      NULL,
      (VOID**)&GigUndiPrivateData->FormBrowser2
      );
  if (EFI_ERROR (Status)) {
    DEBUGPRINT(CRITICAL, ("Error finding HII form browser protocol: %r\n", Status));
    DEBUGWAIT(CRITICAL);
    return Status;
  }

  //
  // Locate ConfigRouting protocol
  //
  DEBUGPRINT(HII, ("Locate HII ConfigRouting Protocol\n"));
  Status = gBS->LocateProtocol (
      &gEfiHiiConfigRoutingProtocolGuid,
      NULL,
      (VOID**)&GigUndiPrivateData->HiiConfigRouting
      );
  if (EFI_ERROR (Status)) {
    DEBUGPRINT(CRITICAL, ("Error finding HII ConfigRouting protocol: %r\n", Status));
    DEBUGWAIT(CRITICAL);
    return Status;
  }

  DEBUGWAIT(HII);
  return Status;
}

VOID
SetMacIdString (
  GIG_UNDI_PRIVATE_DATA     *GigUndiPrivateData,
  OUT CHAR16 *String
  )
{
  switch (GigUndiPrivateData->NicInfo.hw.mac.type) {
#ifndef NO_82571_SUPPORT
    case e1000_82571:
      UnicodeSPrint(String, 80, L"Intel 82571");
      break;
    case e1000_82572:
      UnicodeSPrint(String, 80, L"Intel 82572");
      break;
    case e1000_82573:
      UnicodeSPrint(String, 80, L"Intel 82573");
      break;
#ifndef NO_82574_SUPPORT
    case e1000_82574:
      UnicodeSPrint(String, 80, L"Intel 82574");
      break;
    case e1000_82583:
      UnicodeSPrint(String, 80, L"Intel 82583V");
      break;
#endif
#endif
#ifndef NO_80003ES2LAN_SUPPORT
    case e1000_80003es2lan:
      UnicodeSPrint(String, 80, L"Intel 80003ES2LAN");
      break;
#endif
#ifndef NO_ICH8LAN_SUPPORT
    case e1000_ich8lan:
      UnicodeSPrint(String, 80, L"Intel ICH8");
      break;
    case e1000_ich9lan:
      UnicodeSPrint(String, 80, L"Intel ICH9");
      break;
    case e1000_pchlan:
      UnicodeSPrint(String, 80, L"Intel PCH");
      break;
#endif
#ifndef NO_82575_SUPPORT
    case e1000_82575:
      UnicodeSPrint(String, 80, L"Intel 82575");
      break;
    case e1000_82576:
      UnicodeSPrint(String, 80, L"Intel 82576");
      break;
#endif /* NO_82575_SUPPORT */
    case e1000_82580:
      UnicodeSPrint(String, 80, L"Intel 82580");
      break;

    default:
      UnicodeSPrint(String, 80, L"unknown");
      break;
  }
}


VOID
SetWakeOnLan (
  GIG_UNDI_PRIVATE_DATA     *GigUndiPrivateData,
  IN UINT8                  Enable
  )
{
  UINT16 Reg;
  UINT16 Offset;

  DEBUGPRINT(HII, ("SetWakeOnLan\n"));
  if (GigUndiPrivateData->Configuration.WolEnable == WOL_NA) {
    DEBUGPRINT(HII, ("WOL not supported on this port\n"));
    return;
  }

  if (GigUndiPrivateData->NicInfo.Function == 0) {
    Offset = E1000_INIT_CONTROL_WORD3;
  } else {
    Offset = E1000_INIT_CONTROL_WORD3_LANB;
  }
  e1000_read_nvm(&GigUndiPrivateData->NicInfo.hw, Offset, 1, (UINT16*) &Reg);
  if (Enable == WOL_ENABLE) {
    Reg |= E1000_APM_ENABLE_BIT;
  } else {
    Reg &= ~E1000_APM_ENABLE_BIT;
  }
  e1000_write_nvm(&GigUndiPrivateData->NicInfo.hw, Offset, 1, (UINT16*) &Reg);

  Offset = E1000_INIT_CONTROL_WORD2;
  e1000_read_nvm(&GigUndiPrivateData->NicInfo.hw, Offset, 1, (UINT16*) &Reg);
  if (Enable == WOL_ENABLE) {
    Reg |= E1000_APM_PME_ENABLE_BIT;
  } else {
    Reg &= ~E1000_APM_PME_ENABLE_BIT;
  }
  e1000_write_nvm(&GigUndiPrivateData->NicInfo.hw, Offset, 1, (UINT16*) &Reg);

  e1000_update_nvm_checksum(&GigUndiPrivateData->NicInfo.hw);

  return;
}

VOID
GetWakeOnLanSupport (
  GIG_UNDI_PRIVATE_DATA     *GigUndiPrivateData
  )
{
  EFI_STATUS          Status;
  UINTN               NumHandles = 0;
  EFI_HANDLE          *HandleBuf;
  EFI_PCI_IO_PROTOCOL *PciIo;
  UINTN               Seg, Bus, Device, Function;
  UINT16              DeviceId;
  UINTN               i = 0;
  BOOLEAN             LomBit;

  UINT16              SetupWord;
  UINT32              Result;
  struct e1000_hw     *hw = &GigUndiPrivateData->NicInfo.hw;

  LomBit = FALSE; // Default LomBit to FALSE

  DEBUGPRINT(HII, ("GetWakeOnLanSupport\n"));
  DEBUGWAIT(HII);

  Status = EFI_UNSUPPORTED;

  //
  //  Check for LOM
  //
  Result = e1000_read_nvm(hw, COMPATIBILITY_WORD, 1, &SetupWord);

  if (Result == E1000_SUCCESS){
    if (SetupWord & COMPATABILITY_LOM_BIT){
      LomBit = TRUE;
    }
  }

  GigUndiPrivateData->Configuration.WolEnable = WOL_DISABLE;

  //
  // Check for WOL special cases.
  //
  switch (GigUndiPrivateData->NicInfo.hw.device_id) {
    case E1000_DEV_ID_82571EB_COPPER:
      if (GigUndiPrivateData->NicInfo.hw.subsystem_device_id != 0x135E) {
        break;
      }
    case E1000_DEV_ID_82576: // WOL supported on each port of 82576 LOM device
      DEBUGPRINT(HII, ("WOL supported on LOM device with ID: %X with subdevice ID: %X\n",
        GigUndiPrivateData->NicInfo.hw.device_id, GigUndiPrivateData->NicInfo.hw.subsystem_device_id));
      DEBUGWAIT(HII);
      if (LomBit == FALSE) break;
    case E1000_DEV_ID_82576_SERDES_QUAD:
      DEBUGPRINT(HII, ("WOL supported on this device ID: %X with subdevice ID: %X\n",
          GigUndiPrivateData->NicInfo.hw.device_id, GigUndiPrivateData->NicInfo.hw.subsystem_device_id));
      DEBUGWAIT(HII);
      return;
    case E1000_DEV_ID_82575GB_QUAD_COPPER:
      GigUndiPrivateData->Configuration.WolEnable = WOL_NA;
      DEBUGPRINT(HII, ("WOL not supported on this device ID: %X\n", GigUndiPrivateData->NicInfo.hw.device_id));
      DEBUGWAIT(HII);
      return;
  }

  if (GigUndiPrivateData->NicInfo.Function != 0) {
    GigUndiPrivateData->Configuration.WolEnable = WOL_NA;
    DEBUGPRINT(HII, ("WOL not supported on non-primary port!\n"));
    DEBUGWAIT(HII);
    return;
  }

  //
  // Check for quad-port device WOL support
  //
  if (GigUndiPrivateData->NicInfo.hw.device_id == E1000_DEV_ID_82571EB_QUAD_COPPER ||
    GigUndiPrivateData->NicInfo.hw.device_id == E1000_DEV_ID_82571EB_QUAD_COPPER_LP ||
    GigUndiPrivateData->NicInfo.hw.device_id == E1000_DEV_ID_82571EB_QUAD_FIBER ||
    GigUndiPrivateData->NicInfo.hw.device_id == E1000_DEV_ID_82576_QUAD_COPPER ||
    GigUndiPrivateData->NicInfo.hw.device_id == E1000_DEV_ID_82576_QUAD_COPPER_ET2 ||
    GigUndiPrivateData->NicInfo.hw.device_id == E1000_DEV_ID_82571PT_QUAD_COPPER
    ) {

    DEBUGPRINT(HII, ("Quad port card detected, device ID: %x\n", GigUndiPrivateData->NicInfo.hw.device_id));
    Status = gBS->LocateHandleBuffer(
      ByProtocol,
      &gEfiPciIoProtocolGuid,
      NULL,
      &NumHandles,
      &HandleBuf
      );

    if (EFI_ERROR(Status)) {
      DEBUGPRINT(CRITICAL, ("LocateHandleBuffer returns %r\n", Status));
      DEBUGWAIT(CRITICAL);
      return;
    }

    for (i = 0; i < NumHandles; i++) {
      Status = gBS->OpenProtocol (
                    HandleBuf[i],
                    &gEfiPciIoProtocolGuid,
                    (VOID **) &PciIo,
                    mGigUndiDriverBinding.DriverBindingHandle,
                    GigUndiPrivateData->ControllerHandle,
                    EFI_OPEN_PROTOCOL_GET_PROTOCOL
                    );

      if (EFI_ERROR(Status)) {
        DEBUGPRINT(CRITICAL, ("OpenProtocol returns status %r for handle[%d] %X\n", Status, i, HandleBuf[i]));
        continue;
      }

      PciIo->GetLocation (
        PciIo,
        &Seg,
        &Bus,
        &Device,
        &Function
        );

      //
      // Only PCI function 0 on the first device supports WOL. On PCIe
      // cards each device will be on its own secondary bus as device 0.
      // If we can read device 0 at the next lower bus number and it's
      // the same ID, then we are looking at the second device on the
      // card and WOL is not supported. Otherwise it must be the PCIe
      // switch and therefore this is the first device.
      //
      if (Bus == (GigUndiPrivateData->NicInfo.Bus - 1)) {
        PciIo->Pci.Read (
          PciIo,
          EfiPciIoWidthUint16,
          2,
          1,
          &DeviceId
          );
        DEBUGPRINT(HII, ("GetWakeOnLanSupport: Read device ID %X on Bus %d\n", DeviceId, Bus));
        if (DeviceId == GigUndiPrivateData->NicInfo.hw.device_id) {
          DEBUGPRINT(HII, ("WOL not supported on non-primary port!\n"));
          GigUndiPrivateData->Configuration.WolEnable = WOL_NA;
        }

        Status = gBS->CloseProtocol(
          HandleBuf[i],
          &gEfiPciIoProtocolGuid,
          mGigUndiDriverBinding.DriverBindingHandle,
          GigUndiPrivateData->ControllerHandle
          );

        if (EFI_ERROR(Status)) {
          DEBUGPRINT(CRITICAL, ("CloseProtocol returns %r\n", Status));
          DEBUGWAIT(CRITICAL);
          break;
        }

        if (GigUndiPrivateData->Configuration.WolEnable == WOL_NA) {
          break;
        }

      }
    }
  }

  if (GigUndiPrivateData->Configuration.WolEnable == WOL_NA) {
    DEBUGPRINT(HII, ("WOL not supported on this port!\n"));
  } else {
    DEBUGPRINT(HII, ("WOL supported on this port!\n"));
  }

  DEBUGWAIT(HII);
}

VOID
GetWakeOnLanStatus (
  GIG_UNDI_PRIVATE_DATA     *GigUndiPrivateData
  )
{
  UINT16 Reg;
  UINT16 Offset;

  DEBUGPRINT(HII, ("GetWakeOnLanStatus\n"));
  if (GigUndiPrivateData->Configuration.WolEnable == WOL_NA) {
    DEBUGPRINT(HII, ("WOL not supported on this port\n"));
    return;
  }

  if (GigUndiPrivateData->NicInfo.Function == 0) {
    Offset = E1000_INIT_CONTROL_WORD3;
    DEBUGPRINT(HII, ("Port A WoL status\n"));
  } else {
    Offset = E1000_INIT_CONTROL_WORD3_LANB;
    DEBUGPRINT(HII, ("Port B WoL status\n"));
  }
  e1000_read_nvm(&GigUndiPrivateData->NicInfo.hw, Offset, 1, (UINT16*) &Reg);
  if ((Reg & E1000_APM_ENABLE_BIT) == E1000_APM_ENABLE_BIT) {
    GigUndiPrivateData->Configuration.WolEnable = WOL_ENABLE;
    DEBUGPRINT(HII, ("WOL is enabled on this port\n"));
  } else {
    GigUndiPrivateData->Configuration.WolEnable = WOL_DISABLE;
    DEBUGPRINT(HII, ("WOL is disabled on this port\n"));
  }

}

EFI_STATUS
SetMenuStrings(
  GIG_UNDI_PRIVATE_DATA     *GigUndiPrivateData,
  CHAR8                     *Lang
  )
{
  EFI_STATUS                      Status;
  CHAR16                          *BrandString;
  CHAR16                          String[HII_STRING_LEN];
  CHAR16                          SubString[HII_STRING_LEN];
  CHAR8                           MacAddr[6];
  UINTN                           i, Size;
  CHAR8                           PBAString8[MAX_PBA_STR_LENGTH];
  CHAR16                          PBAString[MAX_PBA_STR_LENGTH];


  BrandString = GigUndiPrivateData->Brand;

  //
  // Branding strings are not localized, use the English branding
  // string.
  //
  UnicodeSPrint(SubString, 85, L"%s", BrandString);
  UnicodeSPrint(String, 0, L"%s - %02x:%02x:%02x:%02x:%02x:%02x",
      SubString,
      GigUndiPrivateData->NicInfo.hw.mac.addr[0],
      GigUndiPrivateData->NicInfo.hw.mac.addr[1],
      GigUndiPrivateData->NicInfo.hw.mac.addr[2],
      GigUndiPrivateData->NicInfo.hw.mac.addr[3],
      GigUndiPrivateData->NicInfo.hw.mac.addr[4],
      GigUndiPrivateData->NicInfo.hw.mac.addr[5]);

  Status = HiiSetString (
               GigUndiPrivateData->HiiHandle,
               STRING_TOKEN (STR_INV_FORM_SET_TITLE),
               String,
               NULL
               );
  if (EFI_ERROR (Status)) {
    DEBUGPRINT(CRITICAL, ("IfrLibSetString error: %r\n", Status));
    DEBUGWAIT(CRITICAL);
    return Status;
  }

  //
  // Set Factory Default MAC address
  //
  DEBUGPRINT(HII, ("Setting MAC address\n"));
  e1000_read_nvm(&GigUndiPrivateData->NicInfo.hw, 0, 3, (UINT16*) MacAddr);

  DEBUGPRINT(HII, ("Adjusting MAC address for PCI function %d\n", (CHAR8) GigUndiPrivateData->NicInfo.Function));
  MacAddr[5] ^= (CHAR8) GigUndiPrivateData->NicInfo.Function;

  Size = HII_STRING_LEN;
  Status = GigUndiPrivateData->HiiString->GetString (
                                   GigUndiPrivateData->HiiString,
                                   Lang,
                                   GigUndiPrivateData->HiiHandle,
                                   STRING_TOKEN (STR_MAC_ADDR_TEXT),
                                   SubString,
                                   &Size,
                                   NULL
                                  );

  if (EFI_ERROR (Status)) {
    DEBUGPRINT (CRITICAL, ("1:GetString error: %r\n", Status));
    DEBUGWAIT(CRITICAL);
    return Status;
  }

  UnicodeSPrint(String, 0, SubString, MacAddr[0], MacAddr[1], MacAddr[2], MacAddr[3], MacAddr[4], MacAddr[5]);

  Status = GigUndiPrivateData->HiiString->SetString (
                                 GigUndiPrivateData->HiiString,
                                 GigUndiPrivateData->HiiHandle,
                                 STRING_TOKEN (STR_MAC_ADDR_TEXT),
                                 Lang,
                                 String,
                                 NULL
                                );

  if (EFI_ERROR (Status)) {
    DEBUGPRINT (CRITICAL, ("1:SetString error: %r\n", Status));
    DEBUGWAIT(CRITICAL);
    return Status;
  }

  //
  // Set Alternate MAC address
  //
  Size = HII_STRING_LEN;
  Status = GigUndiPrivateData->HiiString->GetString (
                                   GigUndiPrivateData->HiiString,
                                   Lang,
                                   GigUndiPrivateData->HiiHandle,
                                   STRING_TOKEN (STR_ALT_MAC_ADDR_TEXT),
                                   SubString,
                                   &Size,
                                   NULL
                                  );

  if (EFI_ERROR (Status)) {
    DEBUGPRINT (CRITICAL, ("2:GetString error: %r, Size = %d\n", Status, Size));
    DEBUGWAIT(CRITICAL);
    return Status;
  }

  UnicodeSPrint(String, 0, SubString,
      GigUndiPrivateData->NicInfo.hw.mac.addr[0],
      GigUndiPrivateData->NicInfo.hw.mac.addr[1],
      GigUndiPrivateData->NicInfo.hw.mac.addr[2],
      GigUndiPrivateData->NicInfo.hw.mac.addr[3],
      GigUndiPrivateData->NicInfo.hw.mac.addr[4],
      GigUndiPrivateData->NicInfo.hw.mac.addr[5]);

  Status = GigUndiPrivateData->HiiString->SetString (
                                 GigUndiPrivateData->HiiString,
                                 GigUndiPrivateData->HiiHandle,
                                 STRING_TOKEN (STR_ALT_MAC_ADDR_TEXT),
                                 Lang,
                                 String,
                                 NULL
                                );

  if (EFI_ERROR (Status)) {
    DEBUGPRINT (CRITICAL, ("1:SetString error: %r\n", Status));
    DEBUGWAIT(CRITICAL);
    return Status;
  }

  //
  // Set PCI Bus/Device/Function
  //
  Size = HII_STRING_LEN;
  Status = GigUndiPrivateData->HiiString->GetString (
                                   GigUndiPrivateData->HiiString,
                                   Lang,
                                   GigUndiPrivateData->HiiHandle,
                                   STRING_TOKEN (STR_PCI_BUS_DEV_FUNC_TEXT),
                                   SubString,
                                   &Size,
                                   NULL
                                  );

  UnicodeSPrint(String, 0, SubString,
    GigUndiPrivateData->NicInfo.Bus,
    GigUndiPrivateData->NicInfo.Device,
    GigUndiPrivateData->NicInfo.Function
    );

  Status = GigUndiPrivateData->HiiString->SetString (
                                 GigUndiPrivateData->HiiString,
                                 GigUndiPrivateData->HiiHandle,
                                 STRING_TOKEN (STR_PCI_BUS_DEV_FUNC_TEXT),
                                 Lang,
                                 String,
                                 NULL
                                );

  if (EFI_ERROR (Status)) {
    DEBUGPRINT (CRITICAL, ("4:SetString error: %r\n", Status));
    DEBUGWAIT(CRITICAL);
    return Status;
  }


  //
  // Set UEFI Driver Branding String
  //
  GigUndiComponentNameGetDriverName ( NULL, "eng", &BrandString);

  Size = HII_STRING_LEN;
  Status = GigUndiPrivateData->HiiString->GetString (
                                   GigUndiPrivateData->HiiString,
                                   Lang,
                                   GigUndiPrivateData->HiiHandle,
                                   STRING_TOKEN (STR_EFI_DRIVER_VER_TEXT),
                                   SubString,
                                   &Size,
                                   NULL
                                  );

  UnicodeSPrint(String, 0, SubString, BrandString);

  Status = GigUndiPrivateData->HiiString->SetString (
                                 GigUndiPrivateData->HiiString,
                                 GigUndiPrivateData->HiiHandle,
                                 STRING_TOKEN (STR_EFI_DRIVER_VER_TEXT),
                                 Lang,
                                 String,
                                 NULL
                                );

  if (EFI_ERROR (Status)) {
    DEBUGPRINT (CRITICAL, ("4:IfrLibSetString error: %r\n", Status));
    DEBUGWAIT(CRITICAL);
    return Status;
  }

  //
  // Set MAC ID String, not localized, use English string.
  //
  SetMacIdString(GigUndiPrivateData, String);
  Status = HiiSetString (
               GigUndiPrivateData->HiiHandle,
               STRING_TOKEN (STR_CONTROLER_ID_TEXT),
               String,
               NULL
               );
  if (EFI_ERROR (Status)) {
    DEBUGPRINT(CRITICAL, ("6:IfrLibSetString error: %r\n", Status));
    DEBUGWAIT(CRITICAL);
    return Status;
  }

  //
  // Set PBA number
  //

  Size = HII_STRING_LEN;
  Status = GigUndiPrivateData->HiiString->GetString (
                                   GigUndiPrivateData->HiiString,
                                   Lang,
                                   GigUndiPrivateData->HiiHandle,
                                   STRING_TOKEN (STR_ADAPTER_PBA_TEXT),
                                   SubString,
                                   &Size,
                                   NULL
                                  );


  if (e1000_read_pba_string(
  		&GigUndiPrivateData->NicInfo.hw,
  		(UINT8*)PBAString8,
  		MAX_PBA_STR_LENGTH) == E1000_SUCCESS) {

    //
	// Convert CHAR8 to CHAR16 for use with UnicodeSPrint
	//
	i = 0;
	while((PBAString8[i] != '\0') && (i < MAX_PBA_STR_LENGTH)) {
    	PBAString[i] = (CHAR16)PBAString8[i];
    	i++;
    }
	PBAString[i] = '\0';

	UnicodeSPrint(String, 0, SubString, PBAString);
  }

  Status = GigUndiPrivateData->HiiString->SetString (
                                 GigUndiPrivateData->HiiString,
                                 GigUndiPrivateData->HiiHandle,
                                 STRING_TOKEN (STR_ADAPTER_PBA_TEXT),
                                 Lang,
                                 String,
                                 NULL
                                );
  if (EFI_ERROR (Status)) {
    DEBUGPRINT (CRITICAL, ("4:IfrLibSetString error: %r\n", Status));
    DEBUGWAIT(CRITICAL);
    return Status;
  }

  return Status;
}

EFI_STATUS
AddDynamicContents(
  GIG_UNDI_PRIVATE_DATA     *GigUndiPrivateData
)
/*++

  Routine Description:
    This function adds some OneOfOpcodes to port configuration form. These Opcodes are dynamically configured
    depending of current adapter and port capabilities.

  Arguments:
    GigUndiPrivateData  - Points to the driver instance private data.

  Returns:
    EFI_SUCCESS if opcodes were added correctly, otherwise EFI error code
    is returned.

--*/
{
#if 0 //TODO: Port Me
	  EFI_STATUS                      Status;
	  UINTN                           CapCnt;
	  IFR_OPTION                      *IfrOptionList;

	  UINT8                			  QuestionFlags;

	  //
	  // Initialize the container for dynamic opcodes.
	  //
	  StartOpCodeHandle = HiiAllocateOpCodeHandle ();
	  ASSERT (StartOpCodeHandle != NULL);

	  //
	  // Allocate space for creation of Option List
	  //
	  IfrOptionList = AllocatePool (5 * sizeof (IFR_OPTION));
	  if (IfrOptionList == NULL){
		  DEBUGPRINT(CRITICAL, ("Allocate memory for IfrOptionList, out of resource.\n"));
		  DEBUGWAIT(CRITICAL);
		  return EFI_OUT_OF_RESOURCES;
	  }

	  //
	  // Create Flash Enable/Disable options
	  //

	  if (GetFlashEnableSupport (GigUndiPrivateData) == EFI_SUCCESS) {
      IfrOptionList[0].Flags        = 0;
      IfrOptionList[0].StringToken  = STRING_TOKEN (STR_DISABLED_TEXT);
      IfrOptionList[0].Value.UINT8     = OROM_DISABLE;
      IfrOptionList[1].Flags        = EFI_IFR_OPTION_DEFAULT;
      IfrOptionList[1].StringToken  = STRING_TOKEN (STR_ENABLED_TEXT);
      IfrOptionList[1].Value.UINT8     = OROM_ENABLE;

      CapCnt = 2;

      QuestionFlags = EFI_IFR_FLAG_RESET_REQUIRED;
	  } else {
      IfrOptionList[0].Flags        = EFI_IFR_OPTION_DEFAULT;
      IfrOptionList[0].StringToken  = STRING_TOKEN (STR_NA_TEXT);
      IfrOptionList[0].Value.UINT8     = OROM_DISABLE;

      CapCnt = 1;

      QuestionFlags = EFI_IFR_FLAG_READ_ONLY;
	  }

	  //
	  //  Create OneOfOpcode
	  //
	  Status = HiiCreateOneOfOpCode (
	            StartOpCodeHandle,
	            0x8001,     // Question ID
	            0x1234,		// Variable ID
	            0,			// Offset for Enable OROM field
	            STRING_TOKEN (STR_OPTION_ROM_EN_PROMPT),      // Prompt Token
	            STRING_TOKEN (STR_OPTION_ROM_EN_HELP), 		// Help Token
	            QuestionFlags,
	            EFI_IFR_NUMERIC_SIZE_1,
	            IfrOptionList,
	            CapCnt,      	// Number of Options
	            NULL
	            );

	  //
	  // Create link speed options depending on link capabilities of adapter
	  //

	  CapCnt=0;

	  IfrOptionList[0].Flags        = EFI_IFR_OPTION_DEFAULT;
	  IfrOptionList[0].StringToken  = STRING_TOKEN (STR_AUTONEG_TEXT);
	  IfrOptionList[0].Value.UINT8     = LINK_SPEED_AUTO_NEG;
	  CapCnt++;

	  QuestionFlags = EFI_IFR_FLAG_READ_ONLY;

	  // Other options are only available for copper media type

	  if (GigUndiPrivateData->NicInfo.hw.phy.media_type == e1000_media_type_copper) {

	    IfrOptionList[1].Flags        = 0;
	    IfrOptionList[1].StringToken  = STRING_TOKEN (STR_10HALF_TEXT);
	    IfrOptionList[1].Value.UINT8     = LINK_SPEED_10HALF;
	    IfrOptionList[2].Flags        = 0;
	    IfrOptionList[2].StringToken  = STRING_TOKEN (STR_10FULL_TEXT);
	    IfrOptionList[2].Value.UINT8     = LINK_SPEED_10FULL;
	    IfrOptionList[3].Flags        = 0;
	    IfrOptionList[3].StringToken  = STRING_TOKEN (STR_100HALF_TEXT);
	    IfrOptionList[3].Value.UINT8     = LINK_SPEED_100HALF;
	    IfrOptionList[4].Flags        = 0;
	    IfrOptionList[4].StringToken  = STRING_TOKEN (STR_100FULL_TEXT);
	    IfrOptionList[4].Value.UINT8     = LINK_SPEED_100FULL;
	    CapCnt+=4;

		QuestionFlags = 0;

	  }

	  //
	  //  Create OneOfOpcode
	  //
	  Status = HiiCreateOneOfOpCode (
	            StartOpCodeHandle,
	            0x8001,     // Question ID
	            0x1234,		// Variable ID
	            1,			// Offset for LinkSpeed field
	            STRING_TOKEN (STR_LINK_SPEED_PROMPT),      // Prompt Token
	            STRING_TOKEN (STR_LINK_SPEED_HELP), 		// Help Token
	            QuestionFlags,
	            EFI_IFR_NUMERIC_SIZE_1,
	            IfrOptionList,
	            CapCnt,      	// Number of Options
	            NULL
	            );

	  if (EFI_ERROR(Status)) {
	    DEBUGPRINT(CRITICAL, ("CreateOneOfOpcode %r\n", Status));
	    DEBUGWAIT(CRITICAL);
	    return Status;
	  }

	  //
	  // Create WOL options depending on port capabilities
	  //

	  GetWakeOnLanSupport(GigUndiPrivateData);
	  GetWakeOnLanStatus(GigUndiPrivateData);

	  if (GigUndiPrivateData->Configuration.WolEnable == WOL_NA) {
	    IfrOptionList[0].Flags        = EFI_IFR_OPTION_DEFAULT;
	    IfrOptionList[0].StringToken  = STRING_TOKEN (STR_NA_TEXT);
	    IfrOptionList[0].Value.UINT8     = WOL_NA;

	    CapCnt = 1;

	    QuestionFlags = EFI_IFR_FLAG_READ_ONLY;

	  } else {
	    IfrOptionList[0].Flags        = 0;
	    IfrOptionList[0].StringToken  = STRING_TOKEN (STR_DISABLED_TEXT);
	    IfrOptionList[0].Value.UINT8     = WOL_DISABLE;
	    IfrOptionList[1].Flags        = EFI_IFR_OPTION_DEFAULT;
	    IfrOptionList[1].StringToken  = STRING_TOKEN (STR_ENABLED_TEXT);
	    IfrOptionList[1].Value.UINT8     = WOL_ENABLE;

	    CapCnt = 2;

	    QuestionFlags = 0;

	  }

	  //
	  //  Create OneOfOpcode
	  //
	  Status = HiiCreateOneOfOpCode (
	            StartOpCodeHandle,
	            0x8001,     // Question ID
	            0x1234,		// Variable ID
	            2,			// Offset for LegacyBoot field
	            STRING_TOKEN (STR_WOL_TEXT),      // Prompt Token
	            STRING_TOKEN (STR_WOL_HELP), 		// Help Token
	            QuestionFlags,
	            EFI_IFR_NUMERIC_SIZE_1,
	            IfrOptionList,
	            CapCnt,      	// Number of Options
	            NULL
	            );

	  if (EFI_ERROR(Status)) {
	    DEBUGPRINT(CRITICAL, ("CreateOneOfOpcode %r\n", Status));
	    DEBUGWAIT(CRITICAL);
	    return Status;
	  }


	  //
	  //  Update Form ID 0x1235 with newly created element
	  //

	  Status = HiiUpdateForm (
				 GigUndiPrivateData->HiiHandle,
				 &mHiiFormGuid,
				 FORM_2,		// Destination form ID
         TRUE,
         NULL
         );

	  if (EFI_ERROR(Status)) {
	    DEBUGPRINT(CRITICAL, ("IfrLibUpdateForm %r\n", Status));
	    DEBUGWAIT(CRITICAL);
	    return Status;
	  }

	  gBS->FreePool (IfrOptionList);

	  return Status;
#else
	return EFI_UNSUPPORTED;
#endif
}


EFI_STATUS
InventoryPackage(
  GIG_UNDI_PRIVATE_DATA     *GigUndiPrivateData
)
{
#if 0 //TODO: PortMe
  EFI_STATUS                      Status;
  EFI_HII_PACKAGE_LIST_HEADER     *PackageList = NULL;
  CHAR8                           Lang[HII_STRING_LEN];
  CHAR8                           SubLang[HII_STRING_LEN];
  UINTN                           Size;
  UINTN                           i, j;

  DEBUGPRINT(HII, ("InventoryPackage\n"));

  PackageList = PreparePackageList (
                  2,
                  &GigUndiPrivateData->HiiFormGuid,
                  GigUndiStrings,
                  inventoryBin
                  );
  if (PackageList == NULL) {
    DEBUGPRINT(CRITICAL, ("PreparePackageList, out of resource.\n"));
    DEBUGWAIT(CRITICAL);
    return EFI_OUT_OF_RESOURCES;
  }

  DEBUGPRINT(HII, ("Calling NewPackageList: %X %X\n", GigUndiPrivateData->HiiDatabase, PackageList));
  Status = GigUndiPrivateData->HiiDatabase->NewPackageList (
                          GigUndiPrivateData->HiiDatabase,
                          PackageList,
                          GigUndiPrivateData->ControllerHandle,
                          &GigUndiPrivateData->HiiHandle
                          );
  if (EFI_ERROR (Status)) {
    gBS->FreePool (PackageList);
    DEBUGPRINT(CRITICAL, ("NewPackageList error: %r\n", Status));
    DEBUGWAIT(CRITICAL);
    return Status;
  }

  Size = HII_STRING_LEN;
  Status = GigUndiPrivateData->HiiString->GetLanguages (
                                 GigUndiPrivateData->HiiString,
                                 GigUndiPrivateData->HiiHandle,
                                 Lang,
                                 &Size
                                );

  for (i = 0; i < Size; i++) {
    DEBUGPRINT(HII, ("%c", Lang[i]));
  }
  DEBUGPRINT (HII, ("Languages: %a, Size: %d\n", Lang, Size));
  DEBUGPRINT (HII, ("GetLanguages returns %r\n", Status));

  i = 0;
  while (i < Size) {
    j=0;
    do {
      if (Lang[i] == ';') {
        SubLang[j]='\0';
        i++;
        j++;
        break;
      }else{
        SubLang[j]=Lang[i];
        i++;
        j++;
      }

    } while (Lang[i] != '\0');

	//
    //Set strings for all languages except x-UEFI
    //
    if (SubLang[0] != 'x') Status = SetMenuStrings(GigUndiPrivateData, SubLang);

    if (EFI_ERROR(Status)) {
      DEBUGPRINT(CRITICAL, ("SetMenuStrings returns %r\n", Status));
      DEBUGWAIT(CRITICAL);
      break;
    }

    // This was the last supported language so we can leave the loop
    if (Lang[i] == '\0')
      break;
  }

  Status = AddDynamicContents(GigUndiPrivateData);

  if (EFI_ERROR(Status)) {
    DEBUGPRINT(CRITICAL, ("AddDynamicContents returns %r\n", Status));
    DEBUGWAIT(CRITICAL);
    return Status;
  }

  return Status;
#else
  return EFI_UNSUPPORTED;
#endif
}

EFI_STATUS
EFIAPI
DriverCallback (
  IN  CONST EFI_HII_CONFIG_ACCESS_PROTOCOL   *This,
  IN  EFI_BROWSER_ACTION                     Action,
  IN  EFI_QUESTION_ID                        QuestionId,
  IN  UINT8                                  Type,
  IN  EFI_IFR_TYPE_VALUE                     *Value,
  OUT EFI_BROWSER_ACTION_REQUEST             *ActionRequest
  )
/*++

  Routine Description:
    This function processes the results of changes in configuration.

  Arguments:
    This          - Points to the EFI_HII_CONFIG_ACCESS_PROTOCOL.
    Action        - Specifies the type of action taken by the browser.
    QuestionId    - A unique value which is sent to the original exporting driver
                    so that it can identify the type of data to expect.
    Type          - The type of value for the question.
    Value         - A pointer to the data being sent to the original exporting driver.
    ActionRequest - On return, points to the action requested by the callback function.

  Returns:
    EFI_SUCCESS          - The callback successfully handled the action.
    EFI_OUT_OF_RESOURCES - Not enough storage is available to hold the variable and its data.
    EFI_DEVICE_ERROR     - The variable could not be saved.
    EFI_UNSUPPORTED      - The specified Action is not supported by the callback.

--*/
{
  EFI_STATUS                      Status;

  DEBUGPRINT(HII, ("DriverCallback\n"));
  DEBUGWAIT(HII);

  if ((Value == NULL) || (ActionRequest == NULL)) {
    return EFI_INVALID_PARAMETER;
  }

  Status = EFI_SUCCESS;

  switch (QuestionId) {
  case 0x1236:
    DEBUGPRINT(HII, ("Save key press\n"));
    DEBUGWAIT(HII);
     *ActionRequest = EFI_BROWSER_ACTION_REQUEST_SUBMIT;
    break;

  case 0x1237:
    //
    // User press "Exit now", request Browser to exit
    //
    DEBUGPRINT(HII, ("Exit key press\n"));
    DEBUGWAIT(HII);
    *ActionRequest = EFI_BROWSER_ACTION_REQUEST_EXIT;
    break;

  default:
    break;
  }

  return Status;
}



EFI_STATUS
EFIAPI
HiiInit (
  GIG_UNDI_PRIVATE_DATA     *GigUndiPrivateData
  )
/*++

  Routine Description:
    Installs the HII user interface screen in the UEFI device manager.

  Arguments:
    GigUndiPrivateData  - Points to the driver instance private data.

  Returns:
    EFI_SUCCESS if HII interface installed correctly, otherwise EFI error code
    is returned.

--*/
{
  EFI_STATUS                      Status;
  EFI_SCREEN_DESCRIPTOR           Screen;
  UINTN                           BufferSize;
  BOOLEAN                         ExtractIfrDefault;
  UINT8                           MacAddr[6];

  DEBUGPRINT(HII, ("HiiInit\n"));

  //
  // Try to open the HII protocols first.  If they are not present in the system
  // get out.
  //
  Status = HiiOpenProtocol(GigUndiPrivateData);
  if (EFI_ERROR (Status)) {
    DEBUGPRINT(CRITICAL, ("HiiOpenProtocol returns: %x\n", Status));
    DEBUGWAIT(CRITICAL);
    return Status;
  }

  //
  // Initialize screen dimensions for SendForm().
  // Remove 3 characters from top and bottom
  //
  ZeroMem (&Screen, sizeof (EFI_SCREEN_DESCRIPTOR));
  gST->ConOut->QueryMode (gST->ConOut, gST->ConOut->Mode->Mode, &Screen.RightColumn, &Screen.BottomRow);

  Screen.TopRow     = 3;
  Screen.BottomRow  = Screen.BottomRow - 3;

  e1000_MemCopy ((UINT8 *) (UINTN) &GigUndiPrivateData->HiiFormGuid, (UINT8 *) (UINTN) &mHiiFormGuid, sizeof(EFI_GUID));
  //
  // Copy the factory default MAC address into the last 6 bytes of the GUID to ensure the GUID is unique.
  //
  e1000_read_nvm(&GigUndiPrivateData->NicInfo.hw, 0, 3, (UINT16*) MacAddr);
  MacAddr[5] ^= (CHAR8) GigUndiPrivateData->NicInfo.Function;

  GigUndiPrivateData->HiiFormGuid.Data4[0] = MacAddr[0];
  GigUndiPrivateData->HiiFormGuid.Data4[1] = MacAddr[1];
  GigUndiPrivateData->HiiFormGuid.Data4[2] = MacAddr[2];
  GigUndiPrivateData->HiiFormGuid.Data4[3] = MacAddr[3];
  GigUndiPrivateData->HiiFormGuid.Data4[4] = MacAddr[4];
  GigUndiPrivateData->HiiFormGuid.Data4[5] = MacAddr[5];


  GigUndiPrivateData->ConfigAccess.ExtractConfig = ExtractConfig;
  GigUndiPrivateData->ConfigAccess.RouteConfig = RouteConfig;
  GigUndiPrivateData->ConfigAccess.Callback = DriverCallback;

  Status = gBS->InstallProtocolInterface (
                  &GigUndiPrivateData->ControllerHandle,
                  &gEfiHiiConfigAccessProtocolGuid,
                  EFI_NATIVE_INTERFACE,
                  &GigUndiPrivateData->ConfigAccess
                  );
  if (EFI_ERROR (Status)) {
    DEBUGPRINT(CRITICAL, ("InstallProtocolInterface error: %r\n", Status));
    DEBUGWAIT(CRITICAL);
  }
  DEBUGPRINT(CRITICAL, ("InstallProtocolInterface NO error: %r\n\n\n", Status));

  ASSERT_EFI_ERROR (Status);

  Status = InventoryPackage(GigUndiPrivateData);
  if (EFI_ERROR (Status)) {
    DEBUGPRINT(CRITICAL, ("InventoryPackage returns: %r\n", Status));
    DEBUGWAIT(CRITICAL);
    return Status;
  }

  //
  // Initialize configuration data
  //
  DEBUGPRINT(HII, ("initialize configuration data\n"));
  ZeroMem (&GigUndiPrivateData->Configuration, sizeof (GIG_DRIVER_CONFIGURATION));

  //
  // Try to read NV config EFI variable first
  //
  ExtractIfrDefault = TRUE;
  BufferSize = sizeof (GIG_DRIVER_CONFIGURATION);
  Status = gRT->GetVariable (
                  VariableName,
                  &mE1000DataGuid,
                  NULL,
                  &BufferSize,
                  &GigUndiPrivateData->Configuration
                  );
  if (!EFI_ERROR (Status) && (BufferSize == sizeof (GIG_DRIVER_CONFIGURATION))) {
    DEBUGPRINT(HII, ("E1000 NV variable already created\n"));
    ExtractIfrDefault = FALSE;
  }

  if (ExtractIfrDefault) {
    //
    // EFI variable for NV config doesn't exit, we should build this variable
    // based on default values stored in IFR
    //
    DEBUGPRINT(HII, ("Creating E1000 variable.\n"));
    BufferSize = sizeof (GIG_DRIVER_CONFIGURATION);

    GigUndiPrivateData->Configuration.OptionRomEnable = GetFlashEnableStatus(GigUndiPrivateData);
    GigUndiPrivateData->Configuration.LinkSpeed = GetLanSpeedStatus(GigUndiPrivateData);

    GetWakeOnLanSupport (GigUndiPrivateData);
    GetWakeOnLanStatus (GigUndiPrivateData);

    gRT->SetVariable(
           VariableName,
           &mE1000DataGuid,
           EFI_VARIABLE_NON_VOLATILE | EFI_VARIABLE_BOOTSERVICE_ACCESS,
           sizeof (GIG_DRIVER_CONFIGURATION),
           &GigUndiPrivateData->Configuration
           );
  }


  DEBUGPRINT(HII, ("HiiInit is complete\n"));

  return EFI_SUCCESS;
}


EFI_STATUS
EFIAPI
HiiUnload (
  GIG_UNDI_PRIVATE_DATA     *GigUndiPrivateData
  )
/*++

  Routine Description:
    HII uninstalls the HII user interface screen in the UEFI device manager.

  Arguments:
    GigUndiPrivateData  - Points to the driver instance private data.

  Returns:
    EFI_SUCCESS if HII interface uninstalled correctly, otherwise EFI error code
    is returned.

--*/
{
  EFI_STATUS                      Status;

  if (GigUndiPrivateData->HiiDatabase == NULL ||
    GigUndiPrivateData->HiiString == NULL ||
    GigUndiPrivateData->FormBrowser2 == NULL ||
    GigUndiPrivateData->HiiConfigRouting == NULL
    ) {
     DEBUGPRINT(HII, ("HII Not initialized, returning.\n"));
     return EFI_SUCCESS;
  }

  DEBUGPRINT(HII, ("Calling RemovePackageList: %X\n", GigUndiPrivateData->HiiDatabase));
  Status = GigUndiPrivateData->HiiDatabase->RemovePackageList (
                          GigUndiPrivateData->HiiDatabase,
                          GigUndiPrivateData->HiiHandle
                          );
  if (EFI_ERROR (Status)) {
    DEBUGPRINT(CRITICAL, ("RemovePackageList error: %r\n", Status));
    DEBUGWAIT(CRITICAL);
    return Status;
  }


  Status = gBS->UninstallProtocolInterface (
                  GigUndiPrivateData->ControllerHandle,
                  &gEfiHiiConfigAccessProtocolGuid,
                  &GigUndiPrivateData->ConfigAccess
                  );
  if (EFI_ERROR (Status)) {
    DEBUGPRINT(CRITICAL, ("UninstallProtocolInterface error: %r\n", Status));
    DEBUGWAIT(CRITICAL);
  }

  return Status;
}

