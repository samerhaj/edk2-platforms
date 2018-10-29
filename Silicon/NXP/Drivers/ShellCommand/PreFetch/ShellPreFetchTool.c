/** Prefetch.c
  Uefi Shell command "prefetch" implementation. 

  Copyright 2017-2018 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the
  BSD License which accompanies this distribution.  The full text of the
  license may be found at http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/
#include <Library/PcdLib.h>
#include "ShellCommand.h"

/**
 * List of command line options for this command
 * We have None, thereforefore empty
*/

STATIC CONST SHELL_PARAM_ITEM ParamList[] = {
  {NULL, TypeMax}
  };

/**
  Help Function that display how to use command line.

**/
VOID
PreFetchHelp (
  )
{
  DEBUG((EFI_D_ERROR, "\nCommand line arguments should be:\n"
  "prefetch value: to enable/disable prefetch settings\n"
  "\t\t value must be enable/disable\n"
  "\t\t e.g prefetch enable or prefetch disable\n"
  "prefetch: to display current prefetch settings\n"));
}

/**
  This is the shell command "prefetch" handler function. This function handles
  the command when it is invoked in the shell.

  @param[in]  This             The instance of the
                               EFI_SHELL_DYNAMIC_COMMAND_PROTOCOL.
  @param[in]  SystemTable      The pointer to the UEFI system table.
  @param[in]  ShellParameters  The parameters associated with the command.
  @param[in]  Shell            The instance of the shell protocol used in the
                               context of processing this command.

  @return  SHELL_SUCCESS            The operation was successful.
  @return  SHELL_ABORTED            Operation aborted due to internal error.
  @return  SHELL_OUT_OF_RESOURCES   A memory allocation failed.

**/
SHELL_STATUS
EFIAPI
ShellDynCmdPreFetchHandler (
  IN EFI_SHELL_DYNAMIC_COMMAND_PROTOCOL  *This,
  IN EFI_SYSTEM_TABLE                    *SystemTable,
  IN EFI_SHELL_PARAMETERS_PROTOCOL       *ShellParameters,
  IN EFI_SHELL_PROTOCOL                  *Shell
  )
{
  EFI_STATUS    Status;
  LIST_ENTRY    *ParamPackage = NULL;
  CONST CHAR16* Data;
  UINT16 Value = 0;

  DEBUG_MSG("Entering Prefetch Shell Handler\n");
  //
  // Install the Shell and Shell Parameters Protocols on the driver
  // image. This is necessary for the initialisation of the Shell
  // Library to succeed in the next step.
  //
  Status = gBS->InstallMultipleProtocolInterfaces (
                  &gImageHandle,
                  &gEfiShellProtocolGuid, Shell,
                  &gEfiShellParametersProtocolGuid, ShellParameters,
                  NULL
                  );
  if (EFI_ERROR (Status)) {
    DEBUG_ERR("Not able to install protocol\n");
    return SHELL_ABORTED;
  }

  //
  // Initialise the Shell Library as we are going to use it.
  // Assert that the return code is EFI_SUCCESS as it should.
  // To anticipate any change is the codes returned by
  // ShellInitialize(), leave in case of error.
  //
  Status = ShellInitialize ();
  if (EFI_ERROR (Status)) {
    ASSERT_EFI_ERROR (Status);
    DEBUG_ERR("Not able to intialize shell\n");
    goto exit;
  }

  // parse command line argument and store in ParamPackage
  Status = ShellCommandLineParse (ParamList, &ParamPackage, NULL, TRUE);
  if (Status != EFI_SUCCESS) {
    DEBUG_ERR("Invalid parameter\n");
    goto exit;
  }

  // check user needs to pass paramters sanity
  if (ShellCommandLineGetCount(ParamPackage) > 2) {
    DEBUG_ERR("Too many arguments\n");
    PreFetchHelp();
    Status = EFI_INVALID_PARAMETER;
    goto exit;
  }

  // Get the value of data passed on Shell
  Data = ShellCommandLineGetRawValue(ParamPackage, 1);

  // In case of prefetch value to be fetched, Data is NULL
  if (Data == NULL) {
    // Display current Prefetch settings
    PreFetchSetGetValue(TRUE, &Value);
    DisplayPreFetchSettings(Value);
    goto exit;
  }

  if(StrCmp(Data, L"enable") == 0) {
    DEBUG_MSG("Enabled prefetch\n");
    Value = 0;
  } else if (StrCmp(Data, L"disable") == 0) {
    DEBUG_MSG("Disable prefetch\n");
    Value = 0xFF;
  } else {
    PreFetchHelp();
    Status = EFI_INVALID_PARAMETER;
    goto exit;
  }
  // Update the user value to PPA
  Status = PreFetchConfigurePPA(Value);
  if (Status == EFI_SUCCESS) {
    // Set the new value for future reference
    Status = PreFetchSetGetValue(FALSE, &Value);
  }

exit:
  gBS->UninstallMultipleProtocolInterfaces (
         gImageHandle,
         &gEfiShellProtocolGuid, Shell,
         &gEfiShellParametersProtocolGuid, ShellParameters,
         NULL
         );
  return Status;
}

/**
  This is the shell command "newresettool" help handler function. This
  function returns the formatted help for the "resettool" command.
  The format matchs that in Appendix B of the revision 2.1 of the
  UEFI Shell Specification.

  @param[in]  This      The instance of the EFI_SHELL_DYNAMIC_COMMAND_PROTOCOL.
  @param[in]  Language  The pointer to the language string to use.

  @return  CHAR16*  Pool allocated help string, must be freed by caller.
**/
CHAR16*
EFIAPI
ShellDynCmdPreFetchGetHelp (
  IN EFI_SHELL_DYNAMIC_COMMAND_PROTOCOL  *This,
  IN CONST CHAR8                         *Language
  )
{
  return HiiGetString (
                mPreFetchShellCommandHiiHandle,
                STRING_TOKEN (STR_GET_HELP_PREFETCHTOOL),
                Language
                );
}
