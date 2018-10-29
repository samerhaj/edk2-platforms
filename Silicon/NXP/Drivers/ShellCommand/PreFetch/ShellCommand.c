/** ShellCommand.c

  Copyright 2017-2018 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the
  BSD License which accompanies this distribution.  The full text of the
  license may be found at http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include <Library/ArmSmcLib.h>
#include <Library/BdsLib.h>
#include "ShellCommand.h"

//
// Internal variables
//

// Protocol structure for prefetch command
STATIC CONST EFI_SHELL_DYNAMIC_COMMAND_PROTOCOL
                              mShellDynCmdProtocolPreFetch = {
                                      L"prefetch",                // Name of the command
                                      ShellDynCmdPreFetchHandler, // Handler
                                      ShellDynCmdPreFetchGetHelp  // GetHelp
};

// guid for indentification of installation
STATIC CONST EFI_GUID mPreFetchShellCommandHiiGuid = {
                                              0x12BE8CCF, 0xD9CE, 0x4EEA,
                         {0xAF, 0x22, 0xB1, 0x97, 0x31, 0xC1, 0xBF, 0xB3}
                         };

// Hii Handle
EFI_HANDLE mPreFetchShellCommandHiiHandle;

// Variable name to hold value of prefetch between reset
STATIC CHAR16  mPreFetchName[] =  L"PreFetchVal";

/**
  Display current prefetch settings

  @param[in] prefetch mask

  @return void
**/

VOID
DisplayPreFetchSettings (
  UINT16 Value
  )
{
    DEBUG ((DEBUG_ERROR, "Prefetch is %a on all Cores\n",
				(Value)? "Disabled" : "Enabled"));
}
/**
  Set and Get the global variable value.

  @param[in]   Get Value to get or Set, True for get, False for Set.

  @param[in]   *Value - Value to set or get

  @return      Same from gRT->Set/Get Variable call

**/
EFI_STATUS
PreFetchSetGetValue (
  BOOLEAN Get,
  UINT16 *Value
  )
{
  EFI_STATUS  Status;
  UINTN       Size;
  UINT32      Attributes;

  Size = sizeof(UINT16);
  Attributes  = EFI_VARIABLE_NON_VOLATILE | EFI_VARIABLE_BOOTSERVICE_ACCESS | EFI_VARIABLE_RUNTIME_ACCESS;
  DEBUG_MSG ("%a Value\n",Get ? "Getting": "Setting");
  if (Get) {
    Status = gRT->GetVariable (mPreFetchName,
                            &gNxpQoriqLsTokenSpaceGuid,
                            NULL, &Size, (VOID *)Value);
  } else {
    Status = gRT->SetVariable (mPreFetchName,
                        &gNxpQoriqLsTokenSpaceGuid,
                        Attributes,
                        Size,
                        Value);
  }
  DEBUG_MSG ("Value set/get error status %d and Value %d\n",Status, *Value);
  return Status;
}

/**
  Configure PPA for new prefetch value

  @param[in]  Prefetch value.

  @return  PPA return code.

**/
UINTN
PreFetchConfigurePPA (
  UINT16 PreFetchEn
  )
{
  ARM_SMC_ARGS ArmSmcArgs;
  DEBUG_MSG("Entering PreFetchConfigurePPA %d\n",PreFetchEn);
  /* Inform to PPA */
  ArmSmcArgs.Arg0 = PPA_PRE_FETCH_CMD;
  ArmSmcArgs.Arg1 = PreFetchEn;
  ArmCallSmc (&ArmSmcArgs);
  if (ArmSmcArgs.Arg0) {
    DEBUG_ERR ("Failed to set PreFetch settings Code: %d \n",
                                    ArmSmcArgs.Arg0);
  }
  DEBUG_MSG ("Exiting function PreFetchConfigrePPA\n");
  return ArmSmcArgs.Arg0;
}

/**
  Main entry point of the Shell Command driver.

  @param[in]  ImageHandle   The firmware allocated handle for the present driver
                            UEFI image.
  @param[in]  *SystemTable  A pointer to the EFI System table.

  @retval  EFI_SUCCESS           The driver was initialized.
  @retval  EFI_OUT_OF_RESOURCES  The "End of DXE" event could not be allocated
                                 or there was not enough memory in pool to
                                 install the Shell Dynamic Command protocol.
  @retval  EFI_LOAD_ERROR        Unable to add the HII package.

**/
EFI_STATUS
ShellCommandEntryPoint (
  IN EFI_HANDLE        ImageHandle,
  IN EFI_SYSTEM_TABLE  *SystemTable
  )
{
  EFI_STATUS  Status;
  EFI_HANDLE  Handle;
  UINT16      PreFetchEn;
  UINT32      Attributes;

  Status = EFI_NOT_FOUND;
  Handle = NULL;
  PreFetchEn = DEFAULT_PREFETCH_VALUE;
  Attributes = EFI_VARIABLE_NON_VOLATILE | EFI_VARIABLE_BOOTSERVICE_ACCESS | EFI_VARIABLE_RUNTIME_ACCESS;
  DEBUG_MSG("Entering prefetch ShellCommandEntryPoint\n");
  // Check value of prefetch setting from previous boot
  Status = PreFetchSetGetValue(TRUE, &PreFetchEn);
  // In case this is first time boot, create the prefetch global variable
  if (Status == EFI_NOT_FOUND) {
    DEBUG_MSG("First boot, enabled prefetch on all cores\n");
    Status = PreFetchSetGetValue(FALSE, &PreFetchEn);
  }
  if (Status == EFI_SUCCESS) {
    // Display current prefetch settings
    DisplayPreFetchSettings(PreFetchEn);
    // Inform prefetch settins to PPA, In case we need to disable on some cores
    if (PreFetchEn)
      Status = PreFetchConfigurePPA(PreFetchEn);

    // create Hii Package
    mPreFetchShellCommandHiiHandle = HiiAddPackages (
                                   &mPreFetchShellCommandHiiGuid,
                                   ImageHandle,
                                   ShellCommandStrings,
                                   NULL
                                   );
    // In case either PPA is rejecting call or Hii is not created
    // remove the global variable and exit
    if (mPreFetchShellCommandHiiHandle == NULL || EFI_ERROR(Status)) {
      // Convert PPA error into UEFI world
      // We are faking it in case of mPreFetchShellCommandHiiHandle is NULL
      Status = EFI_DEVICE_ERROR;
      goto Remove_Global;
    }

    // We install dynamic EFI command on separate handles as we cannot register
    // more than one protocol of the same protocol interface on the same handle.
    Status = gBS->InstallMultipleProtocolInterfaces (
                      &Handle,
                      &gEfiShellDynamicCommandProtocolGuid,
                      &mShellDynCmdProtocolPreFetch,
                      NULL
                      );
    if(EFI_ERROR (Status))
      goto Remove_Hii;
  }
  DEBUG_MSG("Exiting prefetch ShellCommandEntryPoint code %d\n", Status);
  return Status;

Remove_Hii:
  DEBUG_ERR("Removing Hii Package\n");
  HiiRemovePackages (mPreFetchShellCommandHiiHandle);

Remove_Global:
  DEBUG_ERR("Removing Global Variable\n");
  gRT->SetVariable(mPreFetchName, &gNxpQoriqLsTokenSpaceGuid, Attributes, 0, NULL);
  return Status;
}
