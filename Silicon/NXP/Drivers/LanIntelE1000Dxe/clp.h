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

#ifndef _CLP_H_
#define _CLP_H_

#define EFI_PLATFORM_TO_DRIVER_CONFIGURATION_PROTOCOL_GUID \
  { 0x642cd590, 0x8059, 0x4c0a, { 0xa9, 0x58, 0xc5, 0xec, 0x7, 0xd2, 0x3c, 0x4b } }

typedef enum {
  EfiPlatformConfigurationActionNone               = 0,
  EfiPlatformConfigurationActionStopController     = 1,
  EfiPlatformConfigurationActionRestartController  = 2,
  EfiPlatformConfigurationActionRestartPlatform    = 3,
  EfiPlatformConfigurationActionNvramFailed        = 4,
  EfiPlatformConfigurationActionMaximum
} EFI_PLATFORM_CONFIGURATION_ACTION;

#define EFI_PLATFORM_TO_DRIVER_CONFIGURATION_CLP_GUID \
  {0x345ecc0e, 0xcb6, 0x4b75, 0xbb, 0x57, 0x1b, 0x12, 0x9c, 0x47, 0x33,0x3e)

typedef struct _EFI_CONFIGURE_CLP_PARAMETER_BLK {
  CHAR8   *CLPCommand;
  UINT32  CLPCommandLength;
  CHAR8   *CLPReturnString;
  UINT32  CLPReturnStringLength;
  UINT8   CLPCmdStatus;
  UINT8   CLPErrorValue;
  UINT16  CLPMsgCode;
} EFI_CONFIGURE_CLP_PARAMETER_BLK;

typedef struct _EFI_PLATFORM_TO_DRIVER_CONFIGURATION_PROTOCOL EFI_PLATFORM_TO_DRIVER_CONFIGURATION_PROTOCOL;

typedef
EFI_STATUS
(EFIAPI *EFI_PLATFORM_TO_DRIVER_CONFIGURATION_QUERY) (
  IN  EFI_PLATFORM_TO_DRIVER_CONFIGURATION_PROTOCOL  *This,
  IN  EFI_HANDLE                           ControllerHandle,
  IN  EFI_HANDLE                           ChildHandle  OPTIONAL,
  IN  UINTN                                *Instance,
  OUT EFI_GUID                             **ParameterTypeGuid,
  OUT VOID                                 **ParameterBlock,
  OUT UINTN                                *ParameterBlockSize
  );

typedef
EFI_STATUS
(EFIAPI *EFI_PLATFORM_TO_DRIVER_CONFIGURATION_RESPONSE) (
  IN  EFI_PLATFORM_TO_DRIVER_CONFIGURATION_PROTOCOL  *This,
  IN  EFI_HANDLE                           ControllerHandle,
  IN  EFI_HANDLE                           ChildHandle  OPTIONAL,
  IN  UINTN                                *Instance,
  IN  EFI_GUID                             *ParameterTypeGuid,
  IN  VOID                                 *ParameterBlock,
  IN  UINTN                                ParameterBlockSize,
  IN  EFI_PLATFORM_CONFIGURATION_ACTION    ConfigurationAction
  );

struct _EFI_PLATFORM_TO_DRIVER_CONFIGURATION_PROTOCOL {
  EFI_PLATFORM_TO_DRIVER_CONFIGURATION_QUERY      Query;
  EFI_PLATFORM_TO_DRIVER_CONFIGURATION_RESPONSE   Response;
};

EFI_STATUS
ClpEntry(
  IN EFI_DRIVER_BINDING_PROTOCOL  *This,
  IN EFI_HANDLE                   Controller,
  GIG_DRIVER_DATA                 *GigAdapter
  );
#endif
