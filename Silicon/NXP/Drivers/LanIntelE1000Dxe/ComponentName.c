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
#include "brand.h"

extern EFI_DRIVER_BINDING_PROTOCOL mGigUndiDriverBinding;

//
// Version and Branding Information
//

CHAR16 VersionNumber[36];

CHAR16* PrefixString = L"Intel(R) PRO/1000 ";
CHAR16* PlatformName = L" PCI-E  ";

static EFI_UNICODE_STRING_TABLE mGigUndiDriverNameTable[] = {
  { "eng", VersionNumber},
  { "en-US", VersionNumber},
  { NULL, NULL }
};

#define WILD_CARD           0x0000
//
// EFI Component Name Functions
//
EFI_STATUS
GigUndiComponentNameGetDriverName (
  IN  EFI_COMPONENT_NAME_PROTOCOL  *This,
  IN  CHAR8                        *Language,
  OUT CHAR16                       **DriverName
  );

EFI_STATUS
GigUndiComponentNameGetControllerName (
  IN  EFI_COMPONENT_NAME_PROTOCOL  *This,
  IN  EFI_HANDLE                   ControllerHandle,
  IN  EFI_HANDLE                   ChildHandle        OPTIONAL,
  IN  CHAR8                        *Language,
  OUT CHAR16                       **ControllerName
  );

//
// EFI Component Name Protocol
//

EFI_COMPONENT_NAME_PROTOCOL gGigUndiComponentName = {
  GigUndiComponentNameGetDriverName,
  GigUndiComponentNameGetControllerName,
  "eng"
};

EFI_COMPONENT_NAME2_PROTOCOL gGigUndiComponentName2 = {
  (EFI_COMPONENT_NAME2_GET_DRIVER_NAME) GigUndiComponentNameGetDriverName,
  (EFI_COMPONENT_NAME2_GET_CONTROLLER_NAME) GigUndiComponentNameGetControllerName,
  "en-US"
};

EFI_DRIVER_SUPPORTED_EFI_VERSION_PROTOCOL gGigUndiSupportedEFIVersion = {
  sizeof(EFI_DRIVER_SUPPORTED_EFI_VERSION_PROTOCOL),
  EFI_2_10_SYSTEM_TABLE_REVISION
};

VOID
GetUndiControllerName (
  GIG_UNDI_PRIVATE_DATA *GigUndiPrivateData
  )
/*++
  Routine Description:
    Searches through the branding string list for the best possible match for the controller
     associated with GigUndiPrivateData.

  Arguments:
    GigUndiPrivateData - Stores the PCI IDs and component name table for the device.

  Returns:
    VOID
--*/
{
  struct e1000_hw      *Nic;
  CHAR16               *Brand;
  UINTN                i;
  INTN                 VendorMatch;
  INTN                 DeviceIdMatch;
  INTN                 SubVendorMatch;
  INTN                 SubSystemMatch;

  i = 0;
  VendorMatch = -1;
  DeviceIdMatch = -1;
  SubVendorMatch = -1;
  SubSystemMatch = -1;
  Brand = NULL;

  Nic = &GigUndiPrivateData->NicInfo.hw;

  for (i = 0; branding_table[i].vendor_id != 0xFFFF; i++) {
    if (Nic->vendor_id == branding_table[i].vendor_id) {
      if (Nic->device_id == branding_table[i].device_id) {
        if (Nic->subsystem_vendor_id == branding_table[i].subvendor_id) {
          if (Nic->subsystem_device_id == branding_table[i].subsystem_id) {
            //
            //  Found the best possible match
            //
            SubSystemMatch = i;
            break;
          }
          else if (branding_table[i].subsystem_id == WILD_CARD) {
            SubVendorMatch = i;
          }
        }
        else if (branding_table[i].subvendor_id == WILD_CARD) {
          DeviceIdMatch = i;
        }
      }
      else if (branding_table[i].device_id == WILD_CARD) {
        //
        //  Worst match: Vendor ID only, everything else wildcard
        //
        VendorMatch = i;
      }
    }
  }

  //
  // Pick the best possible match
  //
  do {
    if (SubSystemMatch != -1) {
      Brand = branding_table[SubSystemMatch].brand_string;
      break;
    }
    if (SubVendorMatch != -1) {
      Brand = branding_table[SubVendorMatch].brand_string;
      break;
    }
    if (DeviceIdMatch != -1) {
      Brand = branding_table[DeviceIdMatch].brand_string;
      break;
    }
    if (VendorMatch != -1) {
      Brand = branding_table[VendorMatch].brand_string;
      break;
    }
  } while(0);

  if (Brand == NULL) {
    return;
  }

  GigUndiPrivateData->Brand = Brand;
    AddUnicodeString (
      "en-US",
      gGigUndiComponentName2.SupportedLanguages,
      &GigUndiPrivateData->ControllerNameTable,
      Brand
      );

    AddUnicodeString (
      "eng",
      gGigUndiComponentName.SupportedLanguages,
      &GigUndiPrivateData->ControllerNameTable,
      Brand
      );
}


EFI_STATUS
GigUndiComponentNameGetDriverName (
  IN  EFI_COMPONENT_NAME_PROTOCOL  *This,
  IN  CHAR8                        *Language,
  OUT CHAR16                       **DriverName
  )
/*++

  Routine Description:
    Retrieves a Unicode string that is the user readable name of the EFI Driver.

  Arguments:
    This       - A pointer to the EFI_COMPONENT_NAME_PROTOCOL instance.
    Language   - A pointer to a three character ISO 639-2 language identifier.
                 This is the language of the driver name that that the caller
                 is requesting, and it must match one of the languages specified
                 in SupportedLanguages.  The number of languages supported by a
                 driver is up to the driver writer.
    DriverName - A pointer to the Unicode string to return.  This Unicode string
                 is the name of the driver specified by This in the language
                 specified by Language.

  Returns:
    EFI_SUCCES            - The Unicode string for the Driver specified by This
                            and the language specified by Language was returned
                            in DriverName.
    EFI_INVALID_PARAMETER - Language is NULL.
    EFI_INVALID_PARAMETER - DriverName is NULL.
    EFI_UNSUPPORTED       - The driver specified by This does not support the
                            language specified by Language.

--*/
{
  EFI_STATUS Status;

  DEBUGPRINT(INIT, ("GigUndiComponentNameGetDriverName\n"));

  //
  // First lets put together the branding string
  //
  CopyMem(&VersionNumber[0], PrefixString, 18 * sizeof(CHAR16));

  VersionNumber[18] = '0' + MAJORVERSION;
  VersionNumber[19] = '.';
  VersionNumber[20] = '0' + MINORVERSION;
  VersionNumber[21] = '.';
  VersionNumber[22] = '0' + BUILDNUMBER/10;
  VersionNumber[23] = '0' + BUILDNUMBER%10;

  CopyMem(&VersionNumber[24], PlatformName, 7 * sizeof(CHAR16));
  VersionNumber[31] = '\0';

  if (This == (EFI_COMPONENT_NAME_PROTOCOL *)&gGigUndiComponentName2) {
    Language = "en-US";
    Status = LookupUnicodeString (
             Language,
             gGigUndiComponentName2.SupportedLanguages,
             mGigUndiDriverNameTable,
             DriverName
             );
    if (EFI_ERROR(Status)) {
      DEBUGPRINT(CRITICAL, ("LookupUnicodeString returns %r\n", Status));
    }
    return Status;
  }

  Language = "eng";
  Status = LookupUnicodeString (
             Language,
             gGigUndiComponentName.SupportedLanguages,
             mGigUndiDriverNameTable,
             DriverName
             );
  if (EFI_ERROR(Status)) {
    DEBUGPRINT(CRITICAL, ("LookupUnicodeString returns %r\n", Status));
  }

  return Status;
}

EFI_STATUS
GigUndiComponentNameGetControllerName (
  IN  EFI_COMPONENT_NAME_PROTOCOL  *This,
  IN  EFI_HANDLE                   ControllerHandle,
  IN  EFI_HANDLE                   ChildHandle        OPTIONAL,
  IN  CHAR8                        *Language,
  OUT CHAR16                       **ControllerName
  )
/*++

  Routine Description:
    Retrieves a Unicode string that is the user readable name of the controller
    that is being managed by an EFI Driver.

  Arguments:
    This             - A pointer to the EFI_COMPONENT_NAME_PROTOCOL instance.
    ControllerHandle - The handle of a controller that the driver specified by
                       This is managing.  This handle specifies the controller
                       whose name is to be returned.
    ChildHandle      - The handle of the child controller to retrieve the name
                       of.  This is an optional parameter that may be NULL.  It
                       will be NULL for device drivers.  It will also be NULL
                       for a bus drivers that wish to retrieve the name of the
                       bus controller.  It will not be NULL for a bus driver
                       that wishes to retrieve the name of a child controller.
    Language         - A pointer to a three character ISO 639-2 language
                       identifier.  This is the language of the controller name
                       that that the caller is requesting, and it must match one
                       of the languages specified in SupportedLanguages.  The
                       number of languages supported by a driver is up to the
                       driver writer.
    ControllerName   - A pointer to the Unicode string to return.  This Unicode
                       string is the name of the controller specified by
                       ControllerHandle and ChildHandle in the language specified
                       by Language from the point of view of the driver specified
                       by This.

  Returns:
    EFI_SUCCESS           - The Unicode string for the user readable name in the
                            language specified by Language for the driver
                            specified by This was returned in DriverName.
    EFI_INVALID_PARAMETER - ControllerHandle is not a valid EFI_HANDLE.
    EFI_INVALID_PARAMETER - ChildHandle is not NULL and it is not a valid EFI_HANDLE.
    EFI_INVALID_PARAMETER - Language is NULL.
    EFI_INVALID_PARAMETER - ControllerName is NULL.
    EFI_UNSUPPORTED       - The driver specified by This is not currently managing
                            the controller specified by ControllerHandle and
                            ChildHandle.
    EFI_UNSUPPORTED       - The driver specified by This does not support the
                            language specified by Language.

--*/
{
  EFI_NETWORK_INTERFACE_IDENTIFIER_PROTOCOL *NIIProtocol;
  EFI_DEVICE_PATH_PROTOCOL                  *UndiDevicePath;
  GIG_UNDI_PRIVATE_DATA                     *GigUndiPrivateData;
  EFI_STATUS                                Status;

  //
  // Make sure this driver is currently managing ControllerHandle
  //
  Status = gBS->OpenProtocol (
             ControllerHandle,
             &gEfiDevicePathProtocolGuid,
             (VOID **)&UndiDevicePath,
             mGigUndiDriverBinding.DriverBindingHandle,
             ControllerHandle,
             EFI_OPEN_PROTOCOL_BY_DRIVER
             );

  if (Status != EFI_ALREADY_STARTED) {
    gBS->CloseProtocol (
           ControllerHandle,
           &gEfiDevicePathProtocolGuid,
           mGigUndiDriverBinding.DriverBindingHandle,
           ControllerHandle
           );
    return EFI_UNSUPPORTED;
  }

  //
  // Make sure this driver is currently managing ControllerHandle
  //
  if (ChildHandle == NULL) {
    return EFI_UNSUPPORTED;
  }
  else {
    Status = gBS->OpenProtocol (
               ChildHandle,
               &gEfiNetworkInterfaceIdentifierProtocolGuid_31,
               (VOID **)&NIIProtocol,
               mGigUndiDriverBinding.DriverBindingHandle,
               ChildHandle,
               EFI_OPEN_PROTOCOL_GET_PROTOCOL
               );

    if (EFI_ERROR (Status)) {
      return Status;
    }

    GigUndiPrivateData = GIG_UNDI_PRIVATE_DATA_FROM_THIS (NIIProtocol);

    if (This == (EFI_COMPONENT_NAME_PROTOCOL *)&gGigUndiComponentName2) {
   	  Language = "en-US";
      Status = LookupUnicodeString (
               Language,
               gGigUndiComponentName2.SupportedLanguages,
               GigUndiPrivateData->ControllerNameTable,
               ControllerName
               );
      if (EFI_ERROR(Status)) {
        DEBUGPRINT(CRITICAL, ("LookupUnicodeString returns %r\n", Status));
      }
      return Status;
    }

    Language = "eng";
    Status = LookupUnicodeString (
             Language,
             gGigUndiComponentName.SupportedLanguages,
             GigUndiPrivateData->ControllerNameTable,
             ControllerName
             );
    if (EFI_ERROR(Status)) {
      DEBUGPRINT(CRITICAL, ("LookupUnicodeString returns %r\n", Status));
    }
    return Status;
  }
}
