/** @file
  UEFI Component Name(2) protocol implementation.

  Copyright (c) 2011, Intel Corporation
  All rights reserved. This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include "Ax88179.h"

/**
  EFI Component Name Protocol declaration
**/
GLOBAL_REMOVE_IF_UNREFERENCED EFI_COMPONENT_NAME_PROTOCOL  gComponentName = {
  GetDriverName,
  GetControllerName,
  "eng"
};

/**
  EFI Component Name 2 Protocol declaration
**/
GLOBAL_REMOVE_IF_UNREFERENCED EFI_COMPONENT_NAME2_PROTOCOL gComponentName2 = {
  (EFI_COMPONENT_NAME2_GET_DRIVER_NAME) GetDriverName,
  (EFI_COMPONENT_NAME2_GET_CONTROLLER_NAME) GetControllerName,
  "en"
};


/**
  Driver name table declaration
**/
GLOBAL_REMOVE_IF_UNREFERENCED EFI_UNICODE_STRING_TABLE
mDriverNameTable[] = {
  {"eng;en", L"ASIX AX88179 Ethernet Driver 2.9.0"},
  {NULL,  NULL}
};

/**
  Controller name table declaration
**/
GLOBAL_REMOVE_IF_UNREFERENCED EFI_UNICODE_STRING_TABLE
mControllerNameTable[] = {
  {"eng;en", L"AX88179 USB3.0 Gigabit Ethernet Controller"},
  {NULL,  NULL}
};

/**
  Retrieves a Unicode string that is the user readable name of the driver.

  This function retrieves the user readable name of a driver in the form of a
  Unicode string. If the driver specified by This has a user readable name in
  the language specified by Language, then a pointer to the driver name is
  returned in DriverName, and EFI_SUCCESS is returned. If the driver specified
  by This does not support the language specified by Language,
  then EFI_UNSUPPORTED is returned.

  @param [in] pThis             A pointer to the EFI_COMPONENT_NAME2_PROTOCOL or
                                EFI_COMPONENT_NAME_PROTOCOL instance.
  @param [in] pLanguage         A pointer to a Null-terminated ASCII string
                                array indicating the language. This is the
                                language of the driver name that the caller is
                                requesting, and it must match one of the
                                languages specified in SupportedLanguages. The
                                number of languages supported by a driver is up
                                to the driver writer. Language is specified
                                in RFC 3066 or ISO 639-2 language code format.
  @param [out] ppDriverName     A pointer to the Unicode string to return.
                                This Unicode string is the name of the
                                driver specified by This in the language
                                specified by Language.

  @retval EFI_SUCCESS           The Unicode string for the Driver specified by
                                This and the language specified by Language was
                                returned in DriverName.
  @retval EFI_INVALID_PARAMETER Language is NULL.
  @retval EFI_INVALID_PARAMETER DriverName is NULL.
  @retval EFI_UNSUPPORTED       The driver specified by This does not support
                                the language specified by Language.

**/
EFI_STATUS
EFIAPI
GetDriverName (
  IN  EFI_COMPONENT_NAME_PROTOCOL * pThis,
  IN  CHAR8 * pLanguage,
  OUT CHAR16 ** ppDriverName
  )
{
  EFI_STATUS Status;


  Status = LookupUnicodeString2 (
             pLanguage,
             pThis->SupportedLanguages,
             mDriverNameTable,
             ppDriverName,
             (BOOLEAN)(pThis == &gComponentName)
             );

  return Status;
}

/**
  Retrieves a Unicode string that is the user readable name of the controller
  that is being managed by a driver.

  This function retrieves the user readable name of the controller specified by
  ControllerHandle and ChildHandle in the form of a Unicode string. If the
  driver specified by This has a user readable name in the language specified by
  Language, then a pointer to the controller name is returned in ControllerName,
  and EFI_SUCCESS is returned.  If the driver specified by This is not currently
  managing the controller specified by ControllerHandle and ChildHandle,
  then EFI_UNSUPPORTED is returned.  If the driver specified by This does not
  support the language specified by Language, then EFI_UNSUPPORTED is returned.

  @param [in] pThis             A pointer to the EFI_COMPONENT_NAME2_PROTOCOL or
                                EFI_COMPONENT_NAME_PROTOCOL instance.
  @param [in] ControllerHandle  The handle of a controller that the driver
                                specified by This is managing.  This handle
                                specifies the controller whose name is to be
                                returned.
  @param [in] ChildHandle       The handle of the child controller to retrieve
                                the name of.  This is an optional parameter that
                                may be NULL.  It will be NULL for device
                                drivers.  It will also be NULL for a bus drivers
                                that wish to retrieve the name of the bus
                                controller.  It will not be NULL for a bus
                                driver that wishes to retrieve the name of a
                                child controller.
  @param [in] pLanguage         A pointer to a Null-terminated ASCII string
                                array indicating the language.  This is the
                                language of the driver name that the caller is
                                requesting, and it must match one of the
                                languages specified in SupportedLanguages. The
                                number of languages supported by a driver is up
                                to the driver writer. Language is specified in
                                RFC 3066 or ISO 639-2 language code format.
  @param [out] ppControllerName A pointer to the Unicode string to return.
                                This Unicode string is the name of the
                                controller specified by ControllerHandle and
                                ChildHandle in the language specified by
                                Language from the point of view of the driver
                                specified by This.

  @retval EFI_SUCCESS           The Unicode string for the user readable name in
                                the language specified by Language for the
                                driver specified by This was returned in
                                DriverName.
  @retval EFI_INVALID_PARAMETER ControllerHandle is not a valid EFI_HANDLE.
  @retval EFI_INVALID_PARAMETER ChildHandle is not NULL and it is not a valid
                                EFI_HANDLE.
  @retval EFI_INVALID_PARAMETER Language is NULL.
  @retval EFI_INVALID_PARAMETER ControllerName is NULL.
  @retval EFI_UNSUPPORTED       The driver specified by This is not currently
                                managing the controller specified by
                                ControllerHandle and ChildHandle.
  @retval EFI_UNSUPPORTED       The driver specified by This does not support
                                the language specified by Language.

**/



#if PASS_SCT
EFI_STATUS
EFIAPI
GetControllerName (
  IN  EFI_COMPONENT_NAME_PROTOCOL * pThis,
  IN  EFI_HANDLE ControllerHandle,
  IN OPTIONAL EFI_HANDLE ChildHandle,
  IN  CHAR8 * pLanguage,
  OUT CHAR16 ** ppControllerName
  )
{

  EFI_STATUS                  Status;
  EFI_USB_IO_PROTOCOL         *UsbIoProtocol;

  //
  // This is a device driver, so ChildHandle must be NULL.
  //
  if (ChildHandle != NULL) {
    return EFI_UNSUPPORTED;
  }

  //
  // Check Controller's handle
  //
  Status = gBS->OpenProtocol (
                  ControllerHandle,
                  &gEfiUsbIoProtocolGuid,
                  (VOID **) &UsbIoProtocol,
                  gDriverBinding.DriverBindingHandle,
                  ControllerHandle,
                  EFI_OPEN_PROTOCOL_BY_DRIVER
                  );
  if (!EFI_ERROR (Status)) {
    gBS->CloseProtocol (
           ControllerHandle,
           &gEfiUsbIoProtocolGuid,
           gDriverBinding.DriverBindingHandle,
           ControllerHandle
           );
    return EFI_UNSUPPORTED;
  }

  if (Status != EFI_ALREADY_STARTED) {
    return EFI_UNSUPPORTED;
  }

  Status =  LookupUnicodeString2 (
           pLanguage,
            pThis->SupportedLanguages,
           mControllerNameTable,
           ppControllerName,
           (BOOLEAN)(pThis == &gComponentName)
           );
         
  return  Status;
}
#else

EFI_STATUS
EFIAPI
GetControllerName (
  IN  EFI_COMPONENT_NAME_PROTOCOL * pThis,
  IN  EFI_HANDLE ControllerHandle,
  IN OPTIONAL EFI_HANDLE ChildHandle,
  IN  CHAR8 * pLanguage,
  OUT CHAR16 ** ppControllerName
  )
{
  EFI_STATUS Status;
  EFI_USB_IO_PROTOCOL         *UsbIoProtocol;
  EFI_USB_DEVICE_DESCRIPTOR Device;
  
  //
  //  Connect to the USB stack
  //
  Status = gBS->OpenProtocol (
                  ControllerHandle,
                  &gEfiUsbIoProtocolGuid,
                  (VOID **) &UsbIoProtocol,
                  gDriverBinding.DriverBindingHandle,         
                  ControllerHandle,
                  EFI_OPEN_PROTOCOL_BY_DRIVER
                  );
                  
  
  if (!EFI_ERROR ( Status )) {
      Status = UsbIoProtocol->UsbGetDeviceDescriptor ( UsbIoProtocol, &Device );
      if (EFI_ERROR(Status)) {
      	Status = EFI_UNSUPPORTED;
      } else {
          //
          //  Validate the adapter
          //
          if (( VENDOR_ID == Device.IdVendor )
            && ( PRODUCT_ID == Device.IdProduct )) {
            *ppControllerName = L"AX88179 USB3.0 Gigabit Ethernet Controller";
            Status = EFI_SUCCESS;
          } else if (( VENDOR_ID == Device.IdVendor )
            && ( PRODUCT_ID_178A == Device.IdProduct )) {
            *ppControllerName = L"AX88178A USB2.0 Gigabit Ethernet Controller";
            Status = EFI_SUCCESS;
          } else {
            Status = EFI_UNSUPPORTED;
          } 
            
      }
      
      gBS->CloseProtocol (
               ControllerHandle,
               &gEfiUsbIoProtocolGuid,
               gDriverBinding.DriverBindingHandle,
               ControllerHandle
               );
  }
  //
  // Return the operation status
  //
  if (Status != EFI_SUCCESS) {	  
		*ppControllerName = L"AX88179_178A Gigabit Ethernet Controller";
        Status = EFI_SUCCESS;
  }
  return Status;
}

#endif
