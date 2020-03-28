/** @file
  Implement the driver binding protocol for Asix AX88772 Ethernet driver.
                     
  Copyright (c) 2011, Intel Corporation
  All rights reserved. This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include "Ax88772.h"
#include <Library/MemoryAllocationLib.h>
/**
  Verify the controller type

  @param [in] pThis                Protocol instance pointer.
  @param [in] Controller           Handle of device to test.
  @param [in] pRemainingDevicePath Not used.

  @retval EFI_SUCCESS          This driver supports this device.
  @retval other                This driver does not support this device.

**/
EFI_STATUS
EFIAPI
DriverSupported (
  IN EFI_DRIVER_BINDING_PROTOCOL * pThis,
  IN EFI_HANDLE Controller,
  IN EFI_DEVICE_PATH_PROTOCOL * pRemainingDevicePath
  )
{
  EFI_USB_DEVICE_DESCRIPTOR Device;
  EFI_USB_IO_PROTOCOL * pUsbIo;
  EFI_STATUS Status;

  //
  //  Connect to the USB stack
  //
  Status = gBS->OpenProtocol (
                  Controller,
                  &gEfiUsbIoProtocolGuid,
                  (VOID **) &pUsbIo,
                  pThis->DriverBindingHandle,         
                  Controller,
                  EFI_OPEN_PROTOCOL_BY_DRIVER
                  );
  if (!EFI_ERROR ( Status )) {
    //
    //  Get the interface descriptor to check the USB class and find a transport
    //  protocol handler.
    //
    Status = pUsbIo->UsbGetDeviceDescriptor ( pUsbIo, &Device );
    if (EFI_ERROR ( Status )) {
    	Status = EFI_UNSUPPORTED;
    } else {
      //
      //  Validate the adapter
      //      
      if ( VENDOR_ID == Device.IdVendor ) {
        if (PRODUCT_AX88772B_ID != Device.IdProduct) {
        } else if (PRODUCT_AX88772B_ASUS_ID == Device.IdProduct) {
        } else if (PRODUCT_AX88772A_ID == Device.IdProduct) {
        } else if (PRODUCT_ID == Device.IdProduct) {
        } else {
          Status = EFI_UNSUPPORTED;
        }
      } else if ( VENDOR_AX88772B_LENOVO_ID == Device.IdVendor ) {
        if (PRODUCT_AX88772B_LENOVO_ID != Device.IdProduct) {
          Status = EFI_UNSUPPORTED;
        }
      } else {
        Status = EFI_UNSUPPORTED;
      }   
    }
   
    //
    //  Done with the USB stack
    //
    gBS->CloseProtocol (
           Controller,
           &gEfiUsbIoProtocolGuid,
           pThis->DriverBindingHandle,
           Controller
           );
  }

  //
  //  Return the device supported status
  //

  return Status;
}


/**
  Start this driver on Controller by opening UsbIo and DevicePath protocols.
  Initialize PXE structures, create a copy of the Controller Device Path with the
  NIC's MAC address appended to it, install the NetworkInterfaceIdentifier protocol
  on the newly created Device Path.

  @param [in] pThis                Protocol instance pointer.
  @param [in] Controller           Handle of device to work with.
  @param [in] pRemainingDevicePath Not used, always produce all possible children.

  @retval EFI_SUCCESS          This driver is added to Controller.
  @retval other                This driver does not support this device.

**/
EFI_STATUS
EFIAPI
DriverStart (
  IN EFI_DRIVER_BINDING_PROTOCOL * pThis,
  IN EFI_HANDLE Controller,
  IN EFI_DEVICE_PATH_PROTOCOL * pRemainingDevicePath
  )
{
  EFI_STATUS Status;
  NIC_DEVICE *pNicDevice;
  UINTN LengthInBytes;
  EFI_DEVICE_PATH_PROTOCOL *ParentDevicePath = NULL;
  MAC_ADDR_DEVICE_PATH MacDeviceNode;
  EFI_USB_DEVICE_DESCRIPTOR Device;

  //
  //  Allocate the device structure
  //
  LengthInBytes = sizeof ( *pNicDevice );
  Status = gBS->AllocatePool (
                  EfiRuntimeServicesData,
                  LengthInBytes,
                  (VOID **) &pNicDevice
                  );

  if (EFI_ERROR (Status)) {
    goto ERR;
  }
	
  //
  //  Set the structure signature
  //
  ZeroMem ( pNicDevice, LengthInBytes );
  pNicDevice->Signature = DEV_SIGNATURE;

  Status = gBS->OpenProtocol (
                    Controller,
                    &gEfiUsbIoProtocolGuid,
                    (VOID **) &pNicDevice->pUsbIo,
                    pThis->DriverBindingHandle,
                    Controller,
                    EFI_OPEN_PROTOCOL_BY_DRIVER
                    );

  if (EFI_ERROR (Status)) {
    goto ERR;
  }

  pNicDevice->Flag772A = FALSE;
  pNicDevice->pUsbIo->UsbGetDeviceDescriptor ( pNicDevice->pUsbIo, &Device );
  if ((PRODUCT_AX88772A_ID == Device.IdProduct) ||
      (PRODUCT_ID == Device.IdProduct))
    pNicDevice->Flag772A = TRUE;
    //
    //  Initialize the simple network protocol
    //
    Status = SN_Setup ( pNicDevice );

    if (EFI_ERROR(Status)){
      gBS->CloseProtocol (
                Controller,
                &gEfiUsbIoProtocolGuid,
                pThis->DriverBindingHandle,
                Controller
                );
    goto ERR;
  }

  //
  // Set Device Path
  //
    			
  Status = gBS->OpenProtocol (
                    Controller,
                    &gEfiDevicePathProtocolGuid,
                    (VOID **) &ParentDevicePath,
                    pThis->DriverBindingHandle,
                    Controller,
                    EFI_OPEN_PROTOCOL_BY_DRIVER
                    );
  if (EFI_ERROR(Status)) { 
    gBS->CloseProtocol (
              Controller,
              &gEfiUsbIoProtocolGuid,
              pThis->DriverBindingHandle,
              Controller
              );
    goto ERR;
  }

  ZeroMem (&MacDeviceNode, sizeof (MAC_ADDR_DEVICE_PATH));
  MacDeviceNode.Header.Type = MESSAGING_DEVICE_PATH;
  MacDeviceNode.Header.SubType = MSG_MAC_ADDR_DP;

  SetDevicePathNodeLength (&MacDeviceNode.Header, sizeof (MAC_ADDR_DEVICE_PATH));
      			
  CopyMem (&MacDeviceNode.MacAddress,
           &pNicDevice->SimpleNetworkData.CurrentAddress,
           PXE_HWADDR_LEN_ETHER);
      								
  MacDeviceNode.IfType = pNicDevice->SimpleNetworkData.IfType;

  pNicDevice->MyDevPath = AppendDevicePathNode (
                                          ParentDevicePath,
                                          (EFI_DEVICE_PATH_PROTOCOL *) &MacDeviceNode
                                          );

  pNicDevice->Controller = NULL;

  //
  //  Install both the simple network and device path protocols.
  //
  Status = gBS->InstallMultipleProtocolInterfaces (
                          &pNicDevice->Controller,
                          &gEfiCallerIdGuid,
                          pNicDevice,
                          &gEfiSimpleNetworkProtocolGuid,            
                          &pNicDevice->SimpleNetwork,
                          &gEfiDevicePathProtocolGuid,
                          pNicDevice->MyDevPath,
                          NULL
                          );

  if (EFI_ERROR(Status)){
    gBS->CloseProtocol (
              Controller,
              &gEfiDevicePathProtocolGuid,
              pThis->DriverBindingHandle,
              Controller);
    gBS->CloseProtocol (
              Controller,
              &gEfiUsbIoProtocolGuid,
              pThis->DriverBindingHandle,
              Controller
              );
    goto ERR;
  }

  //
  // Open For Child Device
  //
  Status = gBS->OpenProtocol (                                                                         
                  Controller,
                  &gEfiUsbIoProtocolGuid,
                  (VOID **) &pNicDevice->pUsbIo,
                  pThis->DriverBindingHandle,
                  pNicDevice->Controller,
                  EFI_OPEN_PROTOCOL_BY_CHILD_CONTROLLER
                  );

  if (EFI_ERROR(Status)){
    gBS->UninstallMultipleProtocolInterfaces (
                          &pNicDevice->Controller,
                          &gEfiCallerIdGuid,
                          pNicDevice,
                          &gEfiSimpleNetworkProtocolGuid,            
                          &pNicDevice->SimpleNetwork,
                          &gEfiDevicePathProtocolGuid,
                          pNicDevice->MyDevPath,
                          NULL
                          );
    gBS->CloseProtocol (
              Controller,
              &gEfiDevicePathProtocolGuid,
              pThis->DriverBindingHandle,
              Controller);
    gBS->CloseProtocol (
              Controller,
              &gEfiUsbIoProtocolGuid,
              pThis->DriverBindingHandle,
              Controller
              );
    goto ERR;
  }

  return Status;
ERR:
  if ( NULL != pNicDevice->pBulkInbuf)
    gBS->FreePool (pNicDevice->pBulkInbuf);    
  if ( NULL != pNicDevice->pTxTest)
    gBS->FreePool (pNicDevice->pTxTest);
  if ( NULL != pNicDevice->MyDevPath)
    gBS->FreePool (pNicDevice->MyDevPath);
  if (NULL != pNicDevice)
    gBS->FreePool ( pNicDevice );
  return Status;
}

/**
  Stop this driver on Controller by removing NetworkInterfaceIdentifier protocol and
  closing the DevicePath and PciIo protocols on Controller.

  @param [in] pThis                Protocol instance pointer.
  @param [in] Controller           Handle of device to stop driver on.
  @param [in] NumberOfChildren     How many children need to be stopped.
  @param [in] pChildHandleBuffer   Not used.

  @retval EFI_SUCCESS          This driver is removed Controller.
  @retval EFI_DEVICE_ERROR     The device could not be stopped due to a device error.
  @retval other                This driver was not removed from this device.

**/
EFI_STATUS
EFIAPI
DriverStop (
  IN  EFI_DRIVER_BINDING_PROTOCOL * pThis,
  IN  EFI_HANDLE Controller,
  IN  UINTN NumberOfChildren,
  IN  EFI_HANDLE * ChildHandleBuffer
  )
{
  BOOLEAN AllChildrenStopped;
  UINTN Index;
  EFI_SIMPLE_NETWORK_PROTOCOL *SimpleNetwork;
  EFI_STATUS Status = EFI_SUCCESS;
  NIC_DEVICE *pNicDevice;

  //
  // Complete all outstanding transactions to Controller.
  // Don't allow any new transaction to Controller to be started.
  //
  if (NumberOfChildren == 0) {
    Status = gBS->OpenProtocol (
                      Controller,
                      &gEfiSimpleNetworkProtocolGuid,
                      (VOID **) &SimpleNetwork,
                      pThis->DriverBindingHandle,
                      Controller,
                      EFI_OPEN_PROTOCOL_GET_PROTOCOL
                      );
				                
    if (EFI_ERROR(Status)) {
      //
      // This is a 2nd type handle(multi-lun root), it needs to close devicepath
      // and usbio protocol.
      //
      gBS->CloseProtocol (
                Controller,
                &gEfiDevicePathProtocolGuid,
                pThis->DriverBindingHandle,
                Controller
                );
      gBS->CloseProtocol (
                Controller,
                &gEfiUsbIoProtocolGuid,
                pThis->DriverBindingHandle,
                            Controller
                );
      return EFI_SUCCESS;
    }
      
    pNicDevice = DEV_FROM_SIMPLE_NETWORK ( SimpleNetwork );
      
    Status = gBS->UninstallMultipleProtocolInterfaces (
				                  Controller,				                  
				                  &gEfiCallerIdGuid,
				                  pNicDevice,
				                  &gEfiSimpleNetworkProtocolGuid,            
				                  &pNicDevice->SimpleNetwork,
				                  &gEfiDevicePathProtocolGuid,
				                  pNicDevice->MyDevPath,
				                  NULL
				                  );
                          
    if (EFI_ERROR (Status)) {
      return Status;
    }
    //
    // Close the bus driver
    //
    Status = gBS->CloseProtocol (
                       Controller,
                       &gEfiDevicePathProtocolGuid,
                       pThis->DriverBindingHandle,
                       Controller
                       );

    if (EFI_ERROR(Status)){ }

    Status = gBS->CloseProtocol (
                       Controller,
                       &gEfiUsbIoProtocolGuid,
                       pThis->DriverBindingHandle,
                       Controller
                       );

    if (EFI_ERROR(Status)){ }

    return EFI_SUCCESS;
  }
    
  AllChildrenStopped = TRUE;

  for (Index = 0; Index < NumberOfChildren; Index++) {
    Status = gBS->OpenProtocol (
                      ChildHandleBuffer[Index],
                      &gEfiSimpleNetworkProtocolGuid,
                      (VOID **) &SimpleNetwork,
                      pThis->DriverBindingHandle,
                      Controller,
                      EFI_OPEN_PROTOCOL_GET_PROTOCOL
                      );
				                
    if (EFI_ERROR (Status)) {
      AllChildrenStopped = FALSE;
      continue;
    } 
        
    pNicDevice = DEV_FROM_SIMPLE_NETWORK ( SimpleNetwork );
        
    gBS->CloseProtocol (
              Controller,
              &gEfiUsbIoProtocolGuid,
              pThis->DriverBindingHandle,
              ChildHandleBuffer[Index]
              ); 
				                    
    Status = gBS->UninstallMultipleProtocolInterfaces (
				                  ChildHandleBuffer[Index],				                  
				                  &gEfiCallerIdGuid,
				                  pNicDevice,
				                  &gEfiSimpleNetworkProtocolGuid,            
				                  &pNicDevice->SimpleNetwork,
				                  &gEfiDevicePathProtocolGuid,
				                  pNicDevice->MyDevPath,
				                  NULL
				                  );
                          
    if (EFI_ERROR (Status)) {
      Status = gBS->OpenProtocol (                                                                         
                  Controller,
                  &gEfiUsbIoProtocolGuid,
                  (VOID **) &pNicDevice->pUsbIo,
                  pThis->DriverBindingHandle,
                  ChildHandleBuffer[Index],
                  EFI_OPEN_PROTOCOL_BY_CHILD_CONTROLLER
                  );
    } else {
      if ( NULL != pNicDevice->pBulkInbuf)
        gBS->FreePool (pNicDevice->pBulkInbuf);
        if ( NULL != pNicDevice->pTxTest)
          gBS->FreePool (pNicDevice->pTxTest);
        if ( NULL != pNicDevice->MyDevPath)
          gBS->FreePool (pNicDevice->MyDevPath);
        if ( NULL != pNicDevice)
          gBS->FreePool (pNicDevice);
    }
  }
        
  if (!AllChildrenStopped) {
    return EFI_DEVICE_ERROR;
  }

  return EFI_SUCCESS;
}


/**
  Driver binding protocol declaration
**/
EFI_DRIVER_BINDING_PROTOCOL  gDriverBinding = {
  DriverSupported,
  DriverStart,
  DriverStop,
  0xa,
  NULL,
  NULL
};


/**
  Ax88772 driver unload routine.

  @param [in] ImageHandle       Handle for the image.

  @retval EFI_SUCCESS           Image may be unloaded

**/
EFI_STATUS
EFIAPI
DriverUnload (
  IN EFI_HANDLE ImageHandle
  )
{
  UINTN BufferSize;
  UINTN Index;
  UINTN Max;
  EFI_HANDLE * pHandle;
  EFI_STATUS Status;


  //
  //  Determine which devices are using this driver
  //
  BufferSize = 0;
  pHandle = NULL;
  Status = gBS->LocateHandle (
                  ByProtocol,
                  &gEfiCallerIdGuid,
                  NULL,
                  &BufferSize,
                  NULL );
  if ( EFI_BUFFER_TOO_SMALL == Status ) {
    for ( ; ; ) {
      //
      //  One or more block IO devices are present
      //
      Status = gBS->AllocatePool (
                      EfiRuntimeServicesData,
                      BufferSize,
                      (VOID **) &pHandle
                      );
      if ( EFI_ERROR ( Status )) {

        break;
      }

      //
      //  Locate the block IO devices
      //
      Status = gBS->LocateHandle (
                      ByProtocol,
                      &gEfiCallerIdGuid,
                      NULL,
                      &BufferSize,
                      pHandle );
      if ( EFI_ERROR ( Status )) {
        //
        //  Error getting handles
        //
        break;
      }
      
      //
      //  Remove any use of the driver
      //
      Max = BufferSize / sizeof ( pHandle[ 0 ]);
      for ( Index = 0; Max > Index; Index++ ) {
        Status = DriverStop ( &gDriverBinding,
                              pHandle[ Index ],
                              0,
                              NULL );
        if ( EFI_ERROR ( Status )) {
          break;
        }
      }
      break;
    }
  } else {
    if ( EFI_NOT_FOUND == Status ) {
      //
      //  No devices were found
      //
      Status = EFI_SUCCESS;
    }
  }

  //
  //  Free the handle array          
  //
  if ( NULL != pHandle ) {
    gBS->FreePool ( pHandle );
  }

  //
  //  Remove the protocols installed by the EntryPoint routine.
  //
  if ( !EFI_ERROR ( Status )) {
    gBS->UninstallMultipleProtocolInterfaces (
            ImageHandle,
            &gEfiDriverBindingProtocolGuid,
            &gDriverBinding,                              
            &gEfiComponentNameProtocolGuid,
            &gComponentName,
            &gEfiComponentName2ProtocolGuid,
            &gComponentName2,
            NULL
            );
  }

  //
  //  Return the unload status
  //
  return Status;
}


/**
Ax88772 driver entry point.

@param [in] ImageHandle       Handle for the image.
@param [in] pSystemTable      Address of the system table.

@retval EFI_SUCCESS           Image successfully loaded.

**/
EFI_STATUS
EFIAPI
EntryPoint (
  IN EFI_HANDLE ImageHandle,
  IN EFI_SYSTEM_TABLE * pSystemTable
  )
{
  EFI_LOADED_IMAGE_PROTOCOL * pLoadedImage;
  EFI_STATUS    Status;

  //
  //  Enable unload support
  //
  Status = gBS->HandleProtocol (
                  gImageHandle,
                  &gEfiLoadedImageProtocolGuid,
                  (VOID **)&pLoadedImage
                  );
  if (!EFI_ERROR (Status)) {
    pLoadedImage->Unload = DriverUnload;
  }

  //
  //  Add the driver to the list of drivers
  //
  Status = EfiLibInstallDriverBindingComponentName2 (
             ImageHandle,
             pSystemTable,
             &gDriverBinding,
             ImageHandle,
             &gComponentName,
             &gComponentName2
             );
  if ( !EFI_ERROR ( Status )) { }
  return Status;
}
