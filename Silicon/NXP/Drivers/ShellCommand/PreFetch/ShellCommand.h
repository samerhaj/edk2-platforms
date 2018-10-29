/** ShellCommand.h

  Copyright (c) 2011 - 2014, ARM Ltd. All rights reserved.<BR>
  Copyright 2017 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of
  the BSD License which accompanies this distribution.  The full text
  of the license may be found at http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#ifndef __SHELL_PREFETCH_COMMAND__
#define __SHELL_PREFETCH_COMMAND__

#include <Library/DebugLib.h>
#include <Library/HiiLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/ShellLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiLib.h>
#include <Library/UefiRuntimeServicesTableLib.h>
#include <Protocol/EfiShell.h>
#include <Protocol/EfiShellDynamicCommand.h>
#include <Uefi.h>

// Set DEBUG to 1 to enable debug feature
//#define PRE_FETCH_DEBUG
#ifdef PRE_FETCH_DEBUG
#define DEBUG_MSG(_Fmt,...)   DEBUG((EFI_D_RELEASE, "PRE-FETCH DBG: " _Fmt, ##__VA_ARGS__));
#else
#define DEBUG_MSG(_Fmt,...)
#endif
#define DEBUG_ERR(_Fmt,...)    DEBUG((EFI_D_ERROR, "PRE-FETCH ERROR: " _Fmt, ##__VA_ARGS__));

// Default value of prefetch. By default prefetch is enabled on all cores
#define DEFAULT_PREFETCH_VALUE 0

// Prefetch command, to be send to PPA
#define PPA_PRE_FETCH_CMD 0xC200FF13

// Hii Handle
extern EFI_HANDLE mPreFetchShellCommandHiiHandle;

/**
  Display current prefetch settings

  @param[in] prefetch mask

  @return void
**/
VOID
DisplayPreFetchSettings (
  UINT16 Value
  );

/**
  Configure PPA for new prefetch value

  @param[in]  Prefetch value.

  @return  PPA return code.

**/
UINTN
PreFetchConfigurePPA (
  IN UINT16 PreFetch
  );

/**
  Set and Get the global variable value.

  @param[in]  Get Value to get or Set, True for get, False for Set.

  @param[in] *Value - Value to set or get

  @return  As from gRT->Set/Get Variable call

**/
EFI_STATUS
PreFetchSetGetValue (
  BOOLEAN Get,
  UINT16 *Value
  );

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
  @return  SHELL_INVALID_PARAMETER  The parameters of the command are not valid.
  @return  SHELL_NOT_FOUND          Failed to locate a protocol or a file.
  @return  SHELL_OUT_OF_RESOURCES   A memory allocation failed.

**/
SHELL_STATUS
EFIAPI
ShellDynCmdPreFetchHandler (
  IN EFI_SHELL_DYNAMIC_COMMAND_PROTOCOL  *This,
  IN EFI_SYSTEM_TABLE                    *SystemTable,
  IN EFI_SHELL_PARAMETERS_PROTOCOL       *ShellParameters,
  IN EFI_SHELL_PROTOCOL                  *Shell
  );

/**
  This is the shell command "prefetch" help handler function. This
  function returns the formatted help for the "prefetch" command.
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
  );

#endif
