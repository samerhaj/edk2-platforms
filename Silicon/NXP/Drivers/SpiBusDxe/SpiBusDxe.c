/** @file
  This file implements SPI IO Protocol which enables the user to manipulate a single
  SPI device independent of the host controller and SPI design.

  Based on MdeModulePkg/Bus/I2c/I2cDxe/I2cBus.c

  Copyright (c) 2013 - 2015, Intel Corporation. All rights reserved.<BR>
  Copyright 2018 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

 @par Specification Reference:
  - UEFI 2.7 errata A, Chapter 8, Runtime Services
  - UEFI 2.7 errata A, Chapter 10, Device Path Protocol
  - UEFI 2.7 errata A, Chapter 11, UEFI Driver Model
  - PI 1.6, Volume 5, Chapter 18 SPI Protocol Stack
**/
#include <Library/BaseMemoryLib.h>
#include <Library/DebugLib.h>
#include <Library/DevicePathLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiLib.h>
#include <Library/UefiRuntimeLib.h>

#include <Protocol/DriverBinding.h>

#include "SpiBusDxe.h"

//
// Global Variables
//
extern EFI_DRIVER_BINDING_PROTOCOL    gSpiBusDriverBinding;

//
//  EFI_DRIVER_BINDING_PROTOCOL instance
//
EFI_DRIVER_BINDING_PROTOCOL gSpiBusDriverBinding = {
  .Supported = SpiBusDriverSupported,
  .Start = SpiBusDriverStart,
  .Stop = SpiBusDriverStop,
  .Version = 0x10,
  .ImageHandle = NULL,
  .DriverBindingHandle = NULL
};

//
// Template for SPI Bus Context
//
SPI_BUS_CONTEXT gEfiSpiBusContextTemplate = {
  .Signature = SPI_BUS_SIGNATURE,
  .SpiHost = NULL,
  .SpiBus = NULL,
  .Link = {
    .ForwardLink = NULL,
    .BackLink = NULL
  }
};

//
// Template for SPI Device Context
//
SPI_DEVICE_CONTEXT gEfiSpiDeviceContextTemplate = {
  .Signature = SPI_DEVICE_SIGNATURE,
  .Handle = NULL,
  .SpiIo = {
    .SpiPeripheral = NULL,
    .OriginalSpiPeripheral = NULL,
    .FrameSizeSupportMask = 0,
    .MaximumTransferBytes = 1,
    .Attributes = 0,
    .LegacySpiProtocol = NULL,
    .Transaction = SpiBusTransaction,
    .UpdateSpiPeripheral = SpiBusUpdateSpiPeripheral
  },
  .SpiBusContext = NULL,
  .Link = {
    .ForwardLink = NULL,
    .BackLink = NULL
  }
};

STATIC EFI_EVENT     mSpiBusVirtualAddrChangeEvent;

// Link list of SPI Buses that are runtime
STATIC LIST_ENTRY    mSpiBusList = INITIALIZE_LIST_HEAD_VARIABLE (mSpiBusList);

// Link list of SPI Devices that are runtime
STATIC LIST_ENTRY    mSpiDeviceList = INITIALIZE_LIST_HEAD_VARIABLE (mSpiDeviceList);

/**
  Check if the Spi Host controller's device path exists in Spi Bus configuration

  @param[in]  SpiBusConfig      A pointer to the EFI_SPI_CONFIGURATION_PROTOCOL instance.
  @param[in]  ParentDevicePath  A pointer to the Spi Host controller device path.
  @param[Out] SpiBusIndex       Index of Spi Bus corresponding to Spi Host controller which controls that SPI bus.
  @param[Out] SpiBusRuntime     Indicates weather Spi Bus is to be configured for runtime access.

  @retval EFI_SUCCESS             Spi Bus Found.
  @retval EFI_NOT_FOUND           No Spi Bus Found.
  @retval EFI_INVALID_PARAMETER   Input Pointers are NULL.

**/
STATIC
EFI_STATUS
SearchSpiHostController (
  IN  EFI_SPI_CONFIGURATION_PROTOCOL  *SpiBusConfig,
  IN  EFI_DEVICE_PATH_PROTOCOL        *ParentDevicePath,
  OUT UINT32                          *SpiBusIndex,
  OUT BOOLEAN                         *SpiBusRuntime
  )
{
  CONST EFI_SPI_BUS         *SpiBus;
  UINT32                    Index;
  EFI_STATUS                Status;

  if (SpiBusConfig == NULL || ParentDevicePath == NULL || SpiBusIndex == NULL || SpiBusRuntime == NULL) {
    return EFI_INVALID_PARAMETER;
  }

  if (SpiBusConfig->Buslist == NULL || SpiBusConfig->BusCount == 0) {
    return EFI_NOT_FOUND;
  }

  for (Index = 0, Status = EFI_NOT_FOUND; Index < SpiBusConfig->BusCount; Index++) {
    SpiBus = SpiBusConfig->Buslist[Index];
    if (SpiBus == NULL || SpiBus->ControllerPath == NULL) {
      continue;
    }
    if (CompareMem (ParentDevicePath, SpiBus->ControllerPath, GetDevicePathSize (SpiBus->ControllerPath)) == 0) {
      *SpiBusIndex = Index;
      Status = EFI_SUCCESS;
      if (SpiBus->RuntimePeripherallist != NULL) {
        *SpiBusRuntime = TRUE;
      }
      break;
    }
  }

  return Status;
}

/**
  Tests to see if this driver supports a given controller. If a child device is provided,
  it further tests to see if this driver supports creating a handle for the specified child device.

  This function checks to see if the driver specified by This supports the device specified by
  ControllerHandle. Drivers will typically use the device path attached to
  ControllerHandle and/or the services from the bus I/O abstraction attached to
  ControllerHandle to determine if the driver supports ControllerHandle. This function
  may be called many times during platform initialization. In order to reduce boot times, the tests
  performed by this function must be very small, and take as little time as possible to execute. This
  function must not change the state of any hardware devices, and this function must be aware that the
  device specified by ControllerHandle may already be managed by the same driver or a
  different driver. This function must match its calls to AllocatePages() with FreePages(),
  AllocatePool() with FreePool(), and OpenProtocol() with CloseProtocol().
  Since ControllerHandle may have been previously started by the same driver, if a protocol is
  already in the opened state, then it must not be closed with CloseProtocol(). This is required
  to guarantee the state of ControllerHandle is not modified by this function.

  @param[in]  This                 A pointer to the EFI_DRIVER_BINDING_PROTOCOL instance.
  @param[in]  ControllerHandle     The handle of the controller to test. This handle
                                   must support a protocol interface that supplies
                                   an I/O abstraction to the driver.
  @param[in]  RemainingDevicePath  A pointer to the remaining portion of a device path.  This
                                   parameter is ignored by device drivers, and is optional for bus
                                   drivers. For bus drivers, if this parameter is not NULL, then
                                   the bus driver must determine if the bus controller specified
                                   by ControllerHandle and the child controller specified
                                   by RemainingDevicePath are both supported by this
                                   bus driver.

  @retval EFI_SUCCESS              The device specified by ControllerHandle and
                                   RemainingDevicePath is supported by the driver specified by This.
  @retval EFI_ALREADY_STARTED      The device specified by ControllerHandle and
                                   RemainingDevicePath is already being managed by the driver
                                   specified by This.
  @retval EFI_ACCESS_DENIED        The device specified by ControllerHandle and
                                   RemainingDevicePath is already being managed by a different
                                   driver or an application that requires exclusive access.
                                   Currently not implemented.
  @retval EFI_UNSUPPORTED          The device specified by ControllerHandle and
                                   RemainingDevicePath is not supported by the driver specified by This.
**/
EFI_STATUS
EFIAPI
SpiBusDriverSupported (
  IN EFI_DRIVER_BINDING_PROTOCOL  *This,
  IN EFI_HANDLE                   Controller,
  IN EFI_DEVICE_PATH_PROTOCOL     *RemainingDevicePath
  )
{
  EFI_STATUS                          Status;
  EFI_SPI_HC_PROTOCOL                 *SpiHostController;
  EFI_DEVICE_PATH_PROTOCOL            *ParentDevicePath;
  EFI_DEVICE_PATH_PROTOCOL            *DevPathNode;
  EFI_HANDLE                          *Handle;
  UINTN                               HandleCount;

  //
  // Determine if the SPI Host controller Protocol is available
  //
  Status = gBS->OpenProtocol (
                  Controller,
                  &gEfiSpiHcProtocolGuid,
                  (VOID **) &SpiHostController,
                  This->DriverBindingHandle,
                  Controller,
                  EFI_OPEN_PROTOCOL_BY_DRIVER
                  );
  if (EFI_ERROR (Status)) {
    return Status;
  }

  if (!EFI_ERROR (Status)) {
    gBS->CloseProtocol (
          Controller,
          &gEfiSpiHcProtocolGuid,
          This->DriverBindingHandle,
          Controller
          );
  }

  //
  // Determine if the Device Path protocol is available
  //
  Status = gBS->OpenProtocol (
                  Controller,
                  &gEfiDevicePathProtocolGuid,
                  (VOID **) &ParentDevicePath,
                  This->DriverBindingHandle,
                  Controller,
                  EFI_OPEN_PROTOCOL_BY_DRIVER
                  );

  if (EFI_ERROR (Status)) {
    return Status;
  }

  if (!EFI_ERROR (Status)) {
    gBS->CloseProtocol (
          Controller,
          &gEfiDevicePathProtocolGuid,
          This->DriverBindingHandle,
          Controller
          );
  }

  if ((RemainingDevicePath != NULL) && !IsDevicePathEnd (RemainingDevicePath)) {
    //
    // Check if the first node of RemainingDevicePath is a hardware vendor device path
    //
    if ((DevicePathType (RemainingDevicePath) != HARDWARE_DEVICE_PATH) ||
        (DevicePathSubType (RemainingDevicePath) != HW_VENDOR_DP)) {
      return EFI_UNSUPPORTED;
    }
    //
    // Check if the second node of RemainingDevicePath is a controller node
    //
    DevPathNode = NextDevicePathNode (RemainingDevicePath);
    if (!IsDevicePathEnd (DevPathNode)) {
      if ((DevicePathType (DevPathNode) != HARDWARE_DEVICE_PATH) ||
          (DevicePathSubType (DevPathNode) != HW_CONTROLLER_DP)) {
        return EFI_UNSUPPORTED;
      }
    }
  }

  //
  // Determine if the SPI Configuration Protocol is available
  //
  Status = gBS->LocateHandleBuffer (
                  ByProtocol,
                  &gEfiSpiConfigurationProtocolGuid,
                  NULL,
                  &HandleCount,
                  &Handle
                  );
  if (EFI_ERROR (Status)) {
    return Status;
  }

  if (!EFI_ERROR (Status)) {
    FreePool (Handle);
    // Only one SpiConfiguration protocol is allowed
    if (HandleCount != 1) {
      Status = EFI_UNSUPPORTED;
    }
  }

  return Status;
}

/**
  Starts a device controller or a bus controller.

  The Start() function is designed to be invoked from the EFI boot service ConnectController().
  As a result, much of the error checking on the parameters to Start() has been moved into this
  common boot service. It is legal to call Start() from other locations,
  but the following calling restrictions must be followed or the system behavior will not be deterministic.
  1. ControllerHandle must be a valid EFI_HANDLE.
  2. If RemainingDevicePath is not NULL, then it must be a pointer to a naturally aligned
     EFI_DEVICE_PATH_PROTOCOL.
  3. Prior to calling Start(), the Supported() function for the driver specified by This must
     have been called with the same calling parameters, and Supported() must have returned EFI_SUCCESS.

  @param[in]  This                 A pointer to the EFI_DRIVER_BINDING_PROTOCOL instance.
  @param[in]  ControllerHandle     The handle of the controller to start. This handle
                                   must support a protocol interface that supplies
                                   an I/O abstraction to the driver.
  @param[in]  RemainingDevicePath  A pointer to the remaining portion of a device path.  This
                                   parameter is ignored by device drivers, and is optional for bus
                                   drivers. For a bus driver, if this parameter is NULL, then handles
                                   for all the children of Controller are created by this driver.
                                   If this parameter is not NULL and the first Device Path Node is
                                   not the End of Device Path Node, then only the handle for the
                                   child device specified by the first Device Path Node of
                                   RemainingDevicePath is created by this driver.
                                   If the first Device Path Node of RemainingDevicePath is
                                   the End of Device Path Node, no child handle is created by this
                                   driver.

  @retval EFI_SUCCESS              The device was started.
  @retval EFI_DEVICE_ERROR         The device could not be started due to a device error.Currently not implemented.
  @retval EFI_OUT_OF_RESOURCES     The request could not be completed due to a lack of resources.
  @retval Others                   The driver failded to start the device.

**/
EFI_STATUS
EFIAPI
SpiBusDriverStart (
  IN EFI_DRIVER_BINDING_PROTOCOL  *This,
  IN EFI_HANDLE                   Controller,
  IN EFI_DEVICE_PATH_PROTOCOL     *RemainingDevicePath
  )
{
  SPI_BUS_CONTEXT                     *SpiBusContext;
  EFI_STATUS                          Status;
  EFI_SPI_HC_PROTOCOL                 *SpiHostController;
  EFI_LEGACY_SPI_CONTROLLER_PROTOCOL  *LegacySpiHostController;
  EFI_DEVICE_PATH_PROTOCOL            *ParentDevicePath;
  EFI_HANDLE                          *Handle;
  UINTN                               HandleCount;
  EFI_SPI_CONFIGURATION_PROTOCOL      *SpiBusConfig;
  UINT32                              SpiBusIndex;
  BOOLEAN                             SpiBusRuntime;

  SpiBusContext          = NULL;
  ParentDevicePath       = NULL;
  SpiHostController      = NULL;
  SpiBusConfig           = NULL;
  SpiBusIndex            = 0;
  SpiBusRuntime          = FALSE;
  LegacySpiHostController = NULL;

  //
  //  Determine if the SPI controller is available
  //
  Status = gBS->OpenProtocol (
                  Controller,
                  &gEfiSpiHcProtocolGuid,
                  (VOID**)&SpiHostController,
                  This->DriverBindingHandle,
                  Controller,
                  EFI_OPEN_PROTOCOL_BY_DRIVER
                  );
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "SpiBus: open SPI host Controller error, Status = %r\n", Status));
    return Status;
  }

  //
  //  Get the Device Path protocol
  //
  Status = gBS->OpenProtocol (
                   Controller,
                   &gEfiDevicePathProtocolGuid,
                   (VOID **) &ParentDevicePath,
                   This->DriverBindingHandle,
                   Controller,
                   EFI_OPEN_PROTOCOL_BY_DRIVER
                   );
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "SpiBus: open device path error, Status = %r\n", Status));
    goto ErrorExit;
  }

  if ((RemainingDevicePath != NULL) && IsDevicePathEnd (RemainingDevicePath)) {
    //
    // If RemainingDevicePath is the End of Device Path Node,
    // don't create any child device and return EFI_SUCESS
    //
    return EFI_SUCCESS;
  }

  //
  //  Get the Spi Bus Configuration protocol
  //
  Status = gBS->LocateHandleBuffer (
                  ByProtocol,
                  &gEfiSpiConfigurationProtocolGuid,
                  NULL,
                  &HandleCount,
                  &Handle
                  );
  if (EFI_ERROR (Status) || (HandleCount != 1)) {
    DEBUG ((
      DEBUG_ERROR,
      "SpiBus: SPI Configuration Protocol error, Status = %r, HandleCount =%lu\n",
      Status,
      HandleCount
      ));
    goto ErrorExit;
  }

  //
  // Open Spi Configuration Protocol
  //
  Status = gBS->HandleProtocol (
                  Handle[0],
                  &gEfiSpiConfigurationProtocolGuid,
                  (VOID **)&SpiBusConfig
                  );
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "SpiBus: SPI Configuration Protocol error, Status = %r\n", Status));
    goto ErrorExit;
  }

  Status = SearchSpiHostController (SpiBusConfig, ParentDevicePath, &SpiBusIndex, &SpiBusRuntime);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "SpiBus: Could not find Spi Host Controller Status = %r\n", Status));
    goto ErrorExit;
  }

  //
  // Allocate the buffer for SPI_BUS_CONTEXT.
  //
  if (SpiBusRuntime == FALSE) {
    SpiBusContext = AllocateCopyPool (sizeof (SPI_BUS_CONTEXT), &gEfiSpiBusContextTemplate);
  } else {
    SpiBusContext = AllocateRuntimeCopyPool (sizeof (SPI_BUS_CONTEXT), &gEfiSpiBusContextTemplate);
  }
  if (SpiBusContext == NULL) {
    DEBUG ((DEBUG_ERROR, "SpiBus: there is no enough memory to allocate.\n"));
    Status = EFI_OUT_OF_RESOURCES;
    goto ErrorExit;
  }

  /*
     +----------------+
  .->| SPI_BUS_CONTEXT|
  |  +----------------+
  |
  |  +----------------------------+
  |  | SPI_DEVICE_CONTEXT         |
  `--|                            |
     |                            |
     | SPI IO Protocol Structure  | <----- SPI IO Protocol
     |                            |
     +----------------------------+

  */
  SpiBusContext->SpiHost  = SpiHostController;
  SpiBusContext->SpiBus   = SpiBusConfig->Buslist[SpiBusIndex];
  SpiBusContext->DriverBindingHandle = This->DriverBindingHandle;

  Status = gBS->InstallMultipleProtocolInterfaces (
                  &Controller,
                  &gEfiCallerIdGuid, SpiBusContext,
                  NULL
                  );
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "SpiBus: install private protocol error, Status = %r.\n", Status));
    goto ErrorExit;
  }

  if (SpiBusRuntime == TRUE) {
    InsertTailList (&mSpiBusList, &SpiBusContext->Link);
  }

  //
  //  Determine if the SPI controller has installed Legacy Spi controller protocol
  //
  Status = gBS->OpenProtocol (
                  Controller,
                  &gEfiLegacySpiControllerProtocolGuid,
                  (VOID**)&LegacySpiHostController,
                  This->DriverBindingHandle,
                  Controller,
                  EFI_OPEN_PROTOCOL_BY_DRIVER
                  );
  if (EFI_ERROR (Status) && Status != EFI_UNSUPPORTED) {
    DEBUG ((DEBUG_ERROR, "SpiBus: open SPI host Controller error, Status = %r\n", Status));
    goto ErrorExit;
  }

  //
  // Start the driver
  //
  Status = RegisterSpiDevice (SpiBusContext, Controller, LegacySpiHostController, FALSE);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "SpiBus: Failed to register SPI peripherals on SPI bus Status = %r\n", Status));
    goto ErrorExit;
  }

  if (SpiBusRuntime == TRUE) {
    Status = RegisterSpiDevice (SpiBusContext, Controller, LegacySpiHostController, TRUE);
    if (EFI_ERROR (Status)) {
      DEBUG ((DEBUG_ERROR, "SpiBus: Failed to register Runtime SPI peripherals on SPI bus Status = %r\n", Status));
      goto ErrorExit;
    }
  }

ErrorExit:
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "SpiBus: Start() function failed, Status = %r\n", Status));
    if (ParentDevicePath != NULL) {
      gBS->CloseProtocol (
            Controller,
            &gEfiDevicePathProtocolGuid,
            This->DriverBindingHandle,
            Controller
            );
    }

    if (SpiHostController != NULL) {
      gBS->CloseProtocol (
            Controller,
            &gEfiSpiHcProtocolGuid,
            This->DriverBindingHandle,
            Controller
            );
    }

    if (Handle != NULL) {
      FreePool (Handle);
    }

    if (SpiBusContext != NULL) {
      Status = gBS->UninstallMultipleProtocolInterfaces (
                      &Controller,
                      gEfiCallerIdGuid, SpiBusContext,
                      NULL
                      );
      if (SpiBusRuntime == TRUE) {
        RemoveEntryList (&SpiBusContext->Link);
      }
      FreePool (SpiBusContext);
    }

    if (LegacySpiHostController != NULL) {
      gBS->CloseProtocol (
            Controller,
            &gEfiLegacySpiControllerProtocolGuid,
            This->DriverBindingHandle,
            Controller
            );
    }
  }

  //
  //  Return the operation status.
  //
  return Status;
}


/**
  Stops a device controller or a bus controller.

  The Stop() function is designed to be invoked from the EFI boot service DisconnectController().
  As a result, much of the error checking on the parameters to Stop() has been moved
  into this common boot service. It is legal to call Stop() from other locations,
  but the following calling restrictions must be followed or the system behavior will not be deterministic.
  1. ControllerHandle must be a valid EFI_HANDLE that was used on a previous call to this
     same driver's Start() function.
  2. The first NumberOfChildren handles of ChildHandleBuffer must all be a valid
     EFI_HANDLE. In addition, all of these handles must have been created in this driver's
     Start() function, and the Start() function must have called OpenProtocol() on
     ControllerHandle with an Attribute of EFI_OPEN_PROTOCOL_BY_CHILD_CONTROLLER.

  @param[in]  This              A pointer to the EFI_DRIVER_BINDING_PROTOCOL instance.
  @param[in]  ControllerHandle  A handle to the device being stopped. The handle must
                                support a bus specific I/O protocol for the driver
                                to use to stop the device.
  @param[in]  NumberOfChildren  The number of child device handles in ChildHandleBuffer.
  @param[in]  ChildHandleBuffer An array of child handles to be freed. May be NULL
                                if NumberOfChildren is 0.

  @retval EFI_SUCCESS           The device was stopped.
  @retval EFI_DEVICE_ERROR      The device could not be stopped due to a device error.

**/
EFI_STATUS
EFIAPI
SpiBusDriverStop (
  IN  EFI_DRIVER_BINDING_PROTOCOL  *This,
  IN  EFI_HANDLE                   Controller,
  IN  UINTN                        NumberOfChildren,
  IN  EFI_HANDLE                   *ChildHandleBuffer
  )
{
  SPI_BUS_CONTEXT             *SpiBusContext;
  BOOLEAN                     SpiBusRuntime;
  EFI_STATUS                  Status;
  BOOLEAN                     AllChildrenStopped;
  UINTN                       Index;

  if (NumberOfChildren == 0) {
    gBS->CloseProtocol (
          Controller,
          &gEfiDevicePathProtocolGuid,
          This->DriverBindingHandle,
          Controller
          );

    gBS->CloseProtocol (
          Controller,
          &gEfiSpiHcProtocolGuid,
          This->DriverBindingHandle,
          Controller
          );

    gBS->CloseProtocol (
          Controller,
          &gEfiLegacySpiControllerProtocolGuid,
          This->DriverBindingHandle,
          Controller
          );
    //
    // Get our context back
    //
    Status = gBS->OpenProtocol (
                    Controller,
                    &gEfiCallerIdGuid,
                    (VOID **) &SpiBusContext,
                    This->DriverBindingHandle,
                    Controller,
                    EFI_OPEN_PROTOCOL_GET_PROTOCOL
                    );
    if (!EFI_ERROR (Status)) {
      SpiBusRuntime = SpiBusContext->SpiBus->RuntimePeripherallist ? TRUE : FALSE;

      gBS->UninstallMultipleProtocolInterfaces (
            Controller,
            &gEfiCallerIdGuid, SpiBusContext,
            NULL
            );
      //
      // No more child now, free bus context data.
      //
      if (SpiBusRuntime) {
        RemoveEntryList (&SpiBusContext->Link);
      }
      FreePool (SpiBusContext);
    }
    return Status;
  }

  AllChildrenStopped = TRUE;

  for (Index = 0; Index < NumberOfChildren; Index++) {

    Status = UnRegisterSpiDevice (This, Controller, ChildHandleBuffer[Index]);
    if (EFI_ERROR (Status)) {
      AllChildrenStopped = FALSE;
    }
  }

  if (!AllChildrenStopped) {
    return EFI_DEVICE_ERROR;
  }
  return EFI_SUCCESS;
}

/**
  Enumerate the SPI bus

  This routine walks the platform specific data describing the
  SPI bus to create the SPI devices where driver GUIDs were
  specified.

  @param[in] SpiBusContext            Address of an SPI_BUS_CONTEXT structure
  @param[in] Controller               Handle to the controller
  @param[in] LegacySpiHostController  A pointer to the LEGACY_SPI_CONTROLLER_PROTOCOL
                                      interface installed on controller.
  @param[in] RegisterRuntime          Weather to register runtime SPI devices or non runtime
                                      Spi devices.

  @retval EFI_SUCCESS       The bus is successfully configured

**/
EFI_STATUS
RegisterSpiDevice (
  IN SPI_BUS_CONTEXT                     *SpiBusContext,
  IN EFI_HANDLE                          Controller,
  IN EFI_LEGACY_SPI_CONTROLLER_PROTOCOL  *LegacySpiHostController,
  IN BOOLEAN                             RegisterRuntime
  )
{
  CONST EFI_SPI_PERIPHERAL      *SpiPeripheral;
  SPI_DEVICE_CONTEXT            *SpiDeviceContext;
  UINT32                        SpiAttributes;
  UINTN                         StartBit;
  UINTN                         EndBit;
  EFI_STATUS                    Status;

  Status                 = EFI_SUCCESS;
  SpiDeviceContext       = NULL;
  if (RegisterRuntime == FALSE) {
    SpiPeripheral = SpiBusContext->SpiBus->Peripherallist;
  } else {
    SpiPeripheral = SpiBusContext->SpiBus->RuntimePeripherallist;
  }

  //
  //  Walk the list of SPI devices on this bus
  //
  for ( ; SpiPeripheral != NULL; SpiPeripheral = SpiPeripheral->NextSpiPeripheral) {
    //
    //  Determine if the device info is valid
    //
    if ((SpiPeripheral->SpiPeripheralDriverGuid == NULL)
      || (SpiPeripheral->SpiPart == NULL)
      || (SpiPeripheral->SpiBus != SpiBusContext->SpiBus)
      || (SpiPeripheral->ChipSelectParameter == NULL))
    {
      DEBUG ((DEBUG_ERROR, "Invalid EFI_SPI_PERIPHERAL reported by Spi Configuration protocol.\n"));
      continue;
    }

    //
    // Build the device context for current SPI device.
    //
    if (RegisterRuntime == FALSE) {
      SpiDeviceContext = AllocateCopyPool (sizeof (SPI_DEVICE_CONTEXT), &gEfiSpiDeviceContextTemplate);
    } else {
      SpiDeviceContext = AllocateRuntimeCopyPool (sizeof (SPI_DEVICE_CONTEXT), &gEfiSpiDeviceContextTemplate);
    }

    if (SpiDeviceContext == NULL) {
      DEBUG ((DEBUG_ERROR, "Failed to allocate memory for SPI device context.\n"));
      continue;
    }

    //
    //  Initialize the specific device context
    //
    SpiDeviceContext->SpiBusContext = SpiBusContext;
    SpiDeviceContext->SpiIo.SpiPeripheral = SpiPeripheral;
    SpiDeviceContext->SpiIo.OriginalSpiPeripheral = SpiPeripheral;
    SpiDeviceContext->SpiIo.FrameSizeSupportMask = SpiBusContext->SpiHost->FrameSizeSupportMask;
    SpiDeviceContext->SpiIo.MaximumTransferBytes = SpiBusContext->SpiHost->MaximumTransferBytes;
    SpiAttributes = SpiBusContext->SpiHost->Attributes;
    // SPI Io attributes are least attributes supported by both SPI peripheral and SPI Host controller
    StartBit = __builtin_ctz (SPI_HALF_DUPLEX);
    EndBit = __builtin_ctz (SPI_SUPPORTS_READ_ONLY_OPERATIONS);
    SpiAttributes = BitFieldOr32 (
                      SpiAttributes,
                      StartBit,
                      EndBit,
                      BitFieldRead32 (SpiPeripheral->Attributes, StartBit, EndBit)
                      );

    StartBit = __builtin_ctz (SPI_SUPPORTS_DTR_OPERATIONS);
    EndBit = __builtin_ctz (SPI_SUPPORTS_8_BIT_DATA_BUS_WIDTH);
    SpiAttributes = BitFieldAnd32 (
                      SpiAttributes,
                      StartBit,
                      EndBit,
                      BitFieldRead32 (SpiPeripheral->Attributes, StartBit, EndBit)
                      );

    SpiDeviceContext->SpiIo.Attributes = SpiAttributes;
    SpiDeviceContext->SpiIo.LegacySpiProtocol = LegacySpiHostController;

    //
    //  Install the protocol
    //
    Status = gBS->InstallMultipleProtocolInterfaces (
                    &SpiDeviceContext->Handle,
                    SpiPeripheral->SpiPeripheralDriverGuid, &SpiDeviceContext->SpiIo,
                    &gEfiCallerIdGuid, SpiDeviceContext,
                    NULL
                    );
    if (EFI_ERROR (Status)) {
      //
      // Free resources for this SPI device
      //
      ReleaseSpiDeviceContext (SpiDeviceContext);
      continue;
    }

    //
    // Create the child handle
    //
    Status = gBS->OpenProtocol (
                    Controller,
                    &gEfiSpiHcProtocolGuid,
                    (VOID **) &SpiBusContext->SpiHost,
                    SpiBusContext->DriverBindingHandle,
                    SpiDeviceContext->Handle,
                    EFI_OPEN_PROTOCOL_BY_CHILD_CONTROLLER
                    );
    if (EFI_ERROR (Status)) {
      Status = gBS->UninstallMultipleProtocolInterfaces (
                      SpiDeviceContext->Handle,
                      SpiPeripheral->SpiPeripheralDriverGuid, &SpiDeviceContext->SpiIo,
                      &gEfiCallerIdGuid, SpiDeviceContext,
                      NULL
                      );
      //
      // Free resources for this SPI device
      //
      ReleaseSpiDeviceContext (SpiDeviceContext);
      continue;
    }

    //
    // Child has been created successfully
    //
    if (RegisterRuntime == TRUE) {
      InsertTailList (&mSpiDeviceList, &SpiDeviceContext->Link);
    }
  }

  return Status;
}

/**
  Initiate a SPI transaction between the host and a SPI peripheral.

  This routine must be called at or below TPL_NOTIFY.
  This routine works with the SPI bus layer to pass the SPI transactions to the
  SPI controller for execution on the SPI bus.

  @param[in]  This              Pointer to an EFI_SPI_IO_PROTOCOL structure.
  @param[in]  RequestPacket     Pointer to an EFI_SPI_REQUEST_PACKET
                                structure describing the SPI transactions.

  @param[in]  ClockHz           Specify the ClockHz value as zero (0) to use
                                the maximum clock frequency supported by the
                                SPI controller and part. Specify a non-zero
                                value only when a specific SPI transaction
                                requires a reduced clock rate.

  @retval EFI_SUCCESS             The transaction completed successfully.
  @retval EFI_ALREADY_STARTED     The controller is busy with another transaction.
  @retval EFI_BAD_BUFFER_SIZE     The Length value in SPI Transaction is wrong.
  @retval EFI_DEVICE_ERROR        There was an SPI error during the transaction.
  @retval EFI_INVALID_PARAMETER   The parameters specified in RequestPacket are not
                                  Valid. or the RequestPacket is NULL.
  @retval EFI_NOT_READY           Support for the chip select is not properly
                                  initialized
  @retval EFI_INVALID_PARAMETER   The ChipSeLect value or its contents are
                                  invalid
  @retval EFI_NO_RESPONSE         The SPI device is not responding to the slave
                                  address.  EFI_DEVICE_ERROR will be returned if
                                  the controller cannot distinguish when the NACK
                                  occurred.
  @retval EFI_UNSUPPORTED         The controller does not support the requested
                                  transaction. or The SPI controller was not able to support
                                  the frequency requested by ClockHz
**/
EFI_STATUS
SpiBusTransaction (
  IN  CONST EFI_SPI_IO_PROTOCOL  *This,
  IN  EFI_SPI_REQUEST_PACKET     *RequestPacket,
  IN  UINT32                     ClockHz OPTIONAL
  )
{
  EFI_STATUS                      Status;
  SPI_DEVICE_CONTEXT              *SpiDeviceContext;
  SPI_BUS_CONTEXT                 *SpiBusContext;
  CONST EFI_SPI_HC_PROTOCOL       *SpiHostController;
  CONST EFI_SPI_BUS               *SpiBus;
  CONST EFI_SPI_PERIPHERAL        *SpiPeripheral;
  UINT32                          RequestedClockHz;
  BOOLEAN                         ChipSelectPolarity;

  if (This == NULL || RequestPacket == NULL) {
    return EFI_INVALID_PARAMETER;
  }

  // Backup requested clock frequency
  RequestedClockHz = ClockHz;

  SpiPeripheral = This->SpiPeripheral;
  ASSERT (SpiPeripheral != NULL || SpiPeripheral->SpiPart != NULL);

  SpiDeviceContext = SPI_DEVICE_CONTEXT_FROM_PROTOCOL (This);
  SpiBusContext = SpiDeviceContext->SpiBusContext;
  ASSERT (SpiBusContext != NULL);

  SpiHostController = SpiBusContext->SpiHost;
  SpiBus = SpiBusContext->SpiBus;
  ASSERT (SpiHostController != NULL || SpiBus != NULL);

  // The SPI transaction consists of:
  // 1. Adjusting the clock speed, polarity and phase for a SPI peripheral
  if (RequestedClockHz == 0) {
    if (SpiPeripheral->MaxClockHz != 0) {
      ClockHz = MIN (SpiPeripheral->MaxClockHz, SpiPeripheral->SpiPart->MaxClockHz);
    } else {
      ClockHz = SpiPeripheral->SpiPart->MaxClockHz;
    }
    RequestedClockHz = ClockHz;
  } else {
    if (SpiPeripheral->MaxClockHz != 0) {
      if (RequestedClockHz > MIN (SpiPeripheral->MaxClockHz, SpiPeripheral->SpiPart->MaxClockHz)) {
        return EFI_UNSUPPORTED;
      }
    } else {
      if (RequestedClockHz > SpiPeripheral->SpiPart->MaxClockHz) {
        return EFI_UNSUPPORTED;
      }
    }
  }

  if (SpiBus->Clock != NULL) {
    Status = SpiBus->Clock (SpiPeripheral, &ClockHz);
  } else {
    Status = SpiHostController->Clock (SpiHostController, SpiPeripheral, &ClockHz);
  }
  if (EFI_ERROR (Status)) {
    return Status;
  } else if (ClockHz > RequestedClockHz) {
    return EFI_UNSUPPORTED;
  }

  // 2. Use the chip select to enable the SPI peripheral, signaling the transaction start to the chip
  ChipSelectPolarity = SpiPeripheral->SpiPart->ChipSelectPolarity;

  if (SpiPeripheral->ChipSelect != NULL) {
    Status = SpiPeripheral->ChipSelect (SpiPeripheral, ChipSelectPolarity);
  } else {
    Status = SpiHostController->ChipSelect (SpiHostController, SpiPeripheral, ChipSelectPolarity);
  }
  if (EFI_ERROR (Status)) {
    return Status;
  }

  // 3. Transfer the data in one or both directions simultaneously
  Status = SpiHostController->Transaction (SpiHostController, RequestPacket);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  // 4. Remove the chip select from the SPI peripheral signaling the transaction end to the chip
  if (SpiPeripheral->ChipSelect != NULL) {
    Status = SpiPeripheral->ChipSelect (SpiPeripheral, !ChipSelectPolarity);
  } else {
    Status = SpiHostController->ChipSelect (SpiHostController, SpiPeripheral, !ChipSelectPolarity);
  }
  if (EFI_ERROR (Status)) {
    return Status;
  }

  // 5. Optionally, shutdown the SPI controller's internal clock to reduce power
  ClockHz = 0;
  if (SpiBus->Clock != NULL) {
    Status = SpiBus->Clock (SpiPeripheral, &ClockHz);
  } else {
    Status = SpiHostController->Clock (SpiHostController, SpiPeripheral, &ClockHz);
  }

  // Since its optional for a controller to turn off the clock of spi peripherals
  // its not an error if any controller doesn't support this.
  if (Status != EFI_UNSUPPORTED) {
    return Status;
  }

  return EFI_SUCCESS;
}

/**
  Update the SPI peripheral associated with this SPI 10 instance.

  Support socketed SPI parts by allowing the SPI peripheral driver to replace
  the SPI peripheral after the connection is made. An example use is socketed
  SPI NOR flash parts, where the size and parameters change depending upon
  device is in the socket.

  @param[in] This           Pointer to an EFI_SPI_IO_PROTOCOL structure.
  @param[in] SpiPeripheral  Pointer to an EFI_SPI_PERIPHERAL structure.

  @retval EFI_SUCCESS            The SPI peripheral was updated successfully
  @retval EFI_INVALID_PARAMETER  The SpiPeripheral value is NULL,
                                 or the SpiPeripheral->SpiBus is NULL,
                                 or the SpiPeripheral->SpiBus pointing at
                                 wrong bus,
                                 or the SpiPeripheral->SpiPart is NULL

**/
EFI_STATUS
SpiBusUpdateSpiPeripheral (
  IN CONST EFI_SPI_IO_PROTOCOL  *This,
  IN CONST EFI_SPI_PERIPHERAL   *SpiPeripheral
  )
{
  SPI_DEVICE_CONTEXT          *SpiDeviceContext;
  BOOLEAN                     ReinstallProtocol;
  CONST EFI_SPI_PERIPHERAL    *ExistingSpiPeripheral;
  EFI_STATUS                  Status;
  CONST EFI_SPI_HC_PROTOCOL   *SpiHostController;
  SPI_BUS_CONTEXT             *SpiBusContext;

  if ( (This == NULL)
    || (SpiPeripheral == NULL)
    || (SpiPeripheral->SpiBus == NULL)
    || (SpiPeripheral->SpiPart == NULL)
    || (SpiPeripheral->ChipSelectParameter == NULL)
    || (SpiPeripheral->SpiPeripheralDriverGuid == NULL))
  {
    return EFI_INVALID_PARAMETER;
  }

  ExistingSpiPeripheral = This->SpiPeripheral;
  ASSERT (ExistingSpiPeripheral != NULL || ExistingSpiPeripheral->SpiBus != NULL);

  if ( (SpiPeripheral->SpiBus != ExistingSpiPeripheral->SpiBus)
    || (SpiPeripheral->ChipSelectParameter != ExistingSpiPeripheral->ChipSelectParameter))
  {
    return EFI_INVALID_PARAMETER;
  }

  SpiDeviceContext = SPI_DEVICE_CONTEXT_FROM_PROTOCOL (This);
  SpiBusContext = SpiDeviceContext->SpiBusContext;
  ASSERT (SpiBusContext != NULL);

  SpiHostController = SpiBusContext->SpiHost;
  Status = SpiHostController->UpdateSpiPeripheral (SpiHostController, SpiPeripheral);
  if (EFI_ERROR (Status)) {
    Status = EFI_INVALID_PARAMETER;
    return Status;
  }

  ReinstallProtocol = FALSE;
  if (!(CompareGuid (SpiPeripheral->SpiPeripheralDriverGuid, ExistingSpiPeripheral->SpiPeripheralDriverGuid))) {
    ReinstallProtocol = TRUE;
  }

  if (ReinstallProtocol) {
    Status = gBS->UninstallProtocolInterface (
                    SpiDeviceContext->Handle,
                    (EFI_GUID *)ExistingSpiPeripheral->SpiPeripheralDriverGuid,
                    &SpiDeviceContext->SpiIo
                    );
    if (EFI_ERROR (Status)) {
      return Status;
    }
  }

  SpiDeviceContext->SpiIo.SpiPeripheral = SpiPeripheral;

  if (SpiDeviceContext->SpiIo.OriginalSpiPeripheral == NULL) {
    SpiDeviceContext->SpiIo.OriginalSpiPeripheral = ExistingSpiPeripheral;
  } else if (SpiDeviceContext->SpiIo.OriginalSpiPeripheral != ExistingSpiPeripheral) {
    if (SpiPeripheral->SpiPart != ExistingSpiPeripheral->SpiPart) {
      FreePool ((VOID *)(ExistingSpiPeripheral->SpiPart));
    }
    if (SpiPeripheral->ConfigurationData != ExistingSpiPeripheral->ConfigurationData) {
      FreePool ((VOID *)(ExistingSpiPeripheral->ConfigurationData));
    }
    FreePool ((VOID *)ExistingSpiPeripheral);
  }

  if (ReinstallProtocol) {
    Status = gBS->InstallProtocolInterface (
                    &SpiDeviceContext->Handle,
                    (EFI_GUID *)SpiPeripheral->SpiPeripheralDriverGuid,
                    EFI_NATIVE_INTERFACE,
                    &SpiDeviceContext->SpiIo
                    );
    if (EFI_ERROR (Status)) {
      return Status;
    }
  }

  return EFI_SUCCESS;
}

/**
  Release all the resources allocated for the SPI device.

  This function releases all the resources allocated for the SPI device.

  @param  SpiDeviceContext         The SPI child device involved for the operation.

**/
VOID
ReleaseSpiDeviceContext (
  IN SPI_DEVICE_CONTEXT          *SpiDeviceContext
  )
{
  if (SpiDeviceContext == NULL) {
    return;
  }

  FreePool (SpiDeviceContext);
}

/**
  Unregister an SPI device.

  This function removes the protocols installed on the controller handle and
  frees the resources allocated for the SPI device.

  @param  This                  The pointer to EFI_DRIVER_BINDING_PROTOCOL instance.
  @param  Controller            The controller handle of the SPI device.
  @param  Handle                The child handle.

  @retval EFI_SUCCESS           The SPI device is successfully unregistered.
  @return Others                Some error occurs when unregistering the SPI device.

**/
EFI_STATUS
UnRegisterSpiDevice (
  IN  EFI_DRIVER_BINDING_PROTOCOL    *This,
  IN  EFI_HANDLE                     Controller,
  IN  EFI_HANDLE                     Handle
  )
{
  EFI_STATUS                  Status;
  SPI_DEVICE_CONTEXT          *SpiDeviceContext;
  CONST EFI_SPI_PERIPHERAL    *SpiPeripheral;
  EFI_SPI_HC_PROTOCOL         *SpiHost;

  SpiDeviceContext = NULL;

  Status = gBS->OpenProtocol (
                  Handle,
                  &gEfiCallerIdGuid,
                  (VOID **) &SpiDeviceContext,
                  This->DriverBindingHandle,
                  Controller,
                  EFI_OPEN_PROTOCOL_GET_PROTOCOL
                  );
  if (EFI_ERROR (Status)) {
    return Status;
  }

  SpiPeripheral = SpiDeviceContext->SpiIo.SpiPeripheral;

  //
  // Close the child handle
  //
  gBS->CloseProtocol (
         Controller,
         &gEfiSpiHcProtocolGuid,
         This->DriverBindingHandle,
         Handle
         );

  //
  // The SPI Bus driver installs the SPI Io and Local Protocol in the DriverBindingStart().
  // Here should uninstall them.
  //
  Status = gBS->UninstallMultipleProtocolInterfaces (
                  Handle,
                  SpiPeripheral->SpiPeripheralDriverGuid, &SpiDeviceContext->SpiIo,
                  &gEfiCallerIdGuid, SpiDeviceContext,
                  NULL
                  );

  if (EFI_ERROR (Status)) {
    //
    // Keep parent and child relationship
    //
    gBS->OpenProtocol (
          Controller,
          &gEfiSpiHcProtocolGuid,
          (VOID **) &SpiHost,
          This->DriverBindingHandle,
          Handle,
          EFI_OPEN_PROTOCOL_BY_CHILD_CONTROLLER
          );
    return Status;
  }

  //
  // Free resources for this SPI device
  //
  if (SpiDeviceContext->Link.ForwardLink != NULL || SpiDeviceContext->Link.BackLink != NULL) {
    RemoveEntryList (&SpiDeviceContext->Link);
  }
  ReleaseSpiDeviceContext (SpiDeviceContext);

  return EFI_SUCCESS;
}

/**
  Fixup internal data so that EFI can be call in virtual mode.
  Call the passed in Child Notify event and convert any pointers in
  lib to virtual mode.

  @param[in]    Event   The Event that is being processed
  @param[in]    Context Event Context
**/
VOID
EFIAPI
SpiBusVirtualNotifyEvent (
  IN EFI_EVENT        Event,
  IN VOID             *Context
  )
{
  LIST_ENTRY          *Link;
  SPI_DEVICE_CONTEXT  *SpiDeviceContext;
  SPI_BUS_CONTEXT     *SpiBusContext;

  for (Link = mSpiBusList.ForwardLink;  Link != &mSpiBusList; Link = Link->ForwardLink) {
    SpiBusContext = CR (Link, SPI_BUS_CONTEXT, Link, SPI_BUS_SIGNATURE);

    EfiConvertPointer (0x0, (VOID **)&SpiBusContext->SpiHost);
    EfiConvertPointer (0x0, (VOID **)&SpiBusContext->SpiBus);
  }

  for (Link = mSpiDeviceList.ForwardLink;  Link != &mSpiDeviceList; Link = Link->ForwardLink) {
    SpiDeviceContext = CR (Link, SPI_DEVICE_CONTEXT, Link, SPI_DEVICE_SIGNATURE);

    EfiConvertPointer (0x0, (VOID **)&SpiDeviceContext->SpiBusContext);

    EfiConvertPointer (0x0, (VOID **)&SpiDeviceContext->SpiIo.SpiPeripheral);
    EfiConvertPointer (0x0, (VOID **)&SpiDeviceContext->SpiIo.OriginalSpiPeripheral);
    EfiConvertPointer (0x0, (VOID **)&SpiDeviceContext->SpiIo.LegacySpiProtocol);
    EfiConvertPointer (0x0, (VOID **)&SpiDeviceContext->SpiIo.Transaction);
    EfiConvertPointer (0x0, (VOID **)&SpiDeviceContext->SpiIo.UpdateSpiPeripheral);
  }

  return;
}

/**
  The user entry point for the SPI bus module. The user code starts with
  this function.

  @param[in] ImageHandle    The firmware allocated handle for the EFI image.
  @param[in] SystemTable    A pointer to the EFI System Table.

  @retval EFI_SUCCESS       The entry point is executed successfully.
  @retval other             Some error occurs when executing this entry point.

**/
EFI_STATUS
EFIAPI
InitializeSpiBus(
  IN EFI_HANDLE           ImageHandle,
  IN EFI_SYSTEM_TABLE     *SystemTable
  )
{
  EFI_STATUS              Status;

  //
  // Install driver model protocol(s).
  //
  Status = EfiLibInstallDriverBinding (
             ImageHandle,
             SystemTable,
             &gSpiBusDriverBinding,
             NULL
             );
  ASSERT_EFI_ERROR (Status);

  //
  // Register for the virtual address change event
  //
  Status = gBS->CreateEventEx (
                  EVT_NOTIFY_SIGNAL,
                  TPL_NOTIFY,
                  SpiBusVirtualNotifyEvent,
                  NULL,
                  &gEfiEventVirtualAddressChangeGuid,
                  &mSpiBusVirtualAddrChangeEvent
                  );
  ASSERT_EFI_ERROR (Status);

  return Status;
}

/**
  This is the unload handle for SPI bus module.

  Disconnect the driver specified by ImageHandle from all the devices in the handle database.
  Uninstall all the protocols installed in the driver entry point.

  @param[in] ImageHandle           The drivers' driver image.

  @retval    EFI_SUCCESS           The image is unloaded.
  @retval    Others                Failed to unload the image.

**/
EFI_STATUS
EFIAPI
SpiBusUnload (
  IN EFI_HANDLE             ImageHandle
  )
{
  EFI_STATUS                        Status;
  EFI_HANDLE                        *DeviceHandleBuffer;
  UINTN                             DeviceHandleCount;
  UINTN                             Index;

  //
  // Get the list of all SPI Controller handles in the handle database.
  // If there is an error getting the list, then the unload
  // operation fails.
  //
  Status = gBS->LocateHandleBuffer (
                  ByProtocol,
                  &gEfiSpiHcProtocolGuid,
                  NULL,
                  &DeviceHandleCount,
                  &DeviceHandleBuffer
                  );

  if (!EFI_ERROR (Status)) {
    //
    // Disconnect the driver specified by Driver BindingHandle from all
    // the devices in the handle database.
    //
    for (Index = 0; Index < DeviceHandleCount; Index++) {
      Status = gBS->DisconnectController (
                      DeviceHandleBuffer[Index],
                      gSpiBusDriverBinding.DriverBindingHandle,
                      NULL
                      );
      if (EFI_ERROR (Status)) {
        goto Done;
      }
    }
  }

  //
  // Uninstall all the protocols installed in the driver entry point
  //
  Status = gBS->UninstallMultipleProtocolInterfaces (
                  gSpiBusDriverBinding.DriverBindingHandle,
                  &gEfiDriverBindingProtocolGuid,
                  &gSpiBusDriverBinding,
                  NULL
                  );
  ASSERT_EFI_ERROR (Status);

  Status = EFI_SUCCESS;

Done:
  //
  // Free the buffer containing the list of handles from the handle database
  //
  if (DeviceHandleBuffer != NULL) {
    gBS->FreePool (DeviceHandleBuffer);
  }

  return Status;
}
