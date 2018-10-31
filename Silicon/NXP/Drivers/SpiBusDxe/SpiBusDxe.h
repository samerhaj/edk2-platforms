/** @file
  Private data structures for the SPI DXE driver.

  This file defines common data structures, macro definitions and some module
  internal function header files.

  Based on MdeModulePkg/Bus/I2c/I2cDxe/I2cDxe.h

  Copyright (c) 2013, Intel Corporation. All rights reserved.<BR>
  Copyright 2018 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#ifndef __SPI_BUS_DXE_H__
#define __SPI_BUS_DXE_H__

#include <Pi/PiSpi.h>
#include <Protocol/SpiHc.h>
#include <Protocol/SpiIo.h>
#include <Protocol/SpiConfiguration.h>

#define SPI_BUS_SIGNATURE             SIGNATURE_32 ('S', 'P', 'I', 'B')
#define SPI_DEVICE_SIGNATURE          SIGNATURE_32 ('S', 'P', 'I', 'D')

///
/// SPI bus context. This data structure provides connection point
/// between SPI IO protocol (which tells the SPI bus) and SPI host
/// controller protocol.
///
typedef struct {
  ///
  /// Structure identification
  ///
  UINT32                       Signature;

  ///
  /// Spi Host controller protocol
  ///
  CONST EFI_SPI_HC_PROTOCOL    *SpiHost;

  ///
  /// Spi BUS data structure
  ///
  CONST EFI_SPI_BUS            *SpiBus;

  EFI_HANDLE                   DriverBindingHandle;

  ///
  /// Virtual Address change event
  ///
  EFI_EVENT                    SpiBusVirtualAddressEvent;
} SPI_BUS_CONTEXT;

///
/// SPI device context
///
typedef struct {
  ///
  /// Structure identification
  ///
  UINT32                        Signature;

  ///
  /// Spi device handle
  ///
  EFI_HANDLE                    Handle;

  ///
  /// Upper level API to support the SPI device I/O
  ///
  EFI_SPI_IO_PROTOCOL           SpiIo;

  ///
  /// Context for the common I/O support including the
  /// lower level API to the host controller.
  ///
  SPI_BUS_CONTEXT               *SpiBusContext;

  ///
  /// Virtual Address change event
  ///
  EFI_EVENT                     SpiDeviceVirtualAddressEvent;
} SPI_DEVICE_CONTEXT;

#define SPI_DEVICE_CONTEXT_FROM_PROTOCOL(a) CR (a, SPI_DEVICE_CONTEXT, SpiIo, SPI_DEVICE_SIGNATURE)

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
  );

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
  );

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
  );

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
  );

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
  );

/**
  Retrieves a Unicode string that is the user readable name of the driver.

  This function retrieves the user readable name of a driver in the form of a
  Unicode string. If the driver specified by This has a user readable name in
  the language specified by Language, then a pointer to the driver name is
  returned in DriverName, and EFI_SUCCESS is returned. If the driver specified
  by This does not support the language specified by Language,
  then EFI_UNSUPPORTED is returned.

  @param  This[in]              A pointer to the EFI_COMPONENT_NAME2_PROTOCOL or
                                EFI_COMPONENT_NAME_PROTOCOL instance.

  @param  Language[in]          A pointer to a Null-terminated ASCII string
                                array indicating the language. This is the
                                language of the driver name that the caller is
                                requesting, and it must match one of the
                                languages specified in SupportedLanguages. The
                                number of languages supported by a driver is up
                                to the driver writer. Language is specified
                                in RFC 4646 or ISO 639-2 language code format.

  @param  DriverName[out]       A pointer to the Unicode string to return.
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
SpiBusComponentNameGetDriverName (
  IN  EFI_COMPONENT_NAME2_PROTOCOL *This,
  IN  CHAR8                        *Language,
  OUT CHAR16                       **DriverName
  );

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

  @param  This[in]              A pointer to the EFI_COMPONENT_NAME2_PROTOCOL or
                                EFI_COMPONENT_NAME_PROTOCOL instance.

  @param  ControllerHandle[in]  The handle of a controller that the driver
                                specified by This is managing.  This handle
                                specifies the controller whose name is to be
                                returned.

  @param  ChildHandle[in]       The handle of the child controller to retrieve
                                the name of.  This is an optional parameter that
                                may be NULL.  It will be NULL for device
                                drivers.  It will also be NULL for a bus drivers
                                that wish to retrieve the name of the bus
                                controller.  It will not be NULL for a bus
                                driver that wishes to retrieve the name of a
                                child controller.

  @param  Language[in]          A pointer to a Null-terminated ASCII string
                                array indicating the language.  This is the
                                language of the driver name that the caller is
                                requesting, and it must match one of the
                                languages specified in SupportedLanguages. The
                                number of languages supported by a driver is up
                                to the driver writer. Language is specified in
                                RFC 4646 or ISO 639-2 language code format.

  @param  ControllerName[out]   A pointer to the Unicode string to return.
                                This Unicode string is the name of the
                                controller specified by ControllerHandle and
                                ChildHandle in the language specified by
                                Language from the point of view of the driver
                                specified by This.

  @retval EFI_SUCCESS           The Unicode string for the user readable name in
                                the language specified by Language for the
                                driver specified by This was returned in
                                DriverName.

  @retval EFI_INVALID_PARAMETER ControllerHandle is NULL.

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
EFI_STATUS
EFIAPI
SpiBusComponentNameGetControllerName (
  IN  EFI_COMPONENT_NAME2_PROTOCOL                    *This,
  IN  EFI_HANDLE                                      ControllerHandle,
  IN  EFI_HANDLE                                      ChildHandle        OPTIONAL,
  IN  CHAR8                                           *Language,
  OUT CHAR16                                          **ControllerName
  );

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
  );

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
  );

/**
  Release all the resources allocated for the SPI device.

  This function releases all the resources allocated for the SPI device.

  @param  SpiDeviceContext         The SPI child device involved for the operation.

**/
VOID
ReleaseSpiDeviceContext (
  IN SPI_DEVICE_CONTEXT          *SpiDeviceContext
  );

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
  );

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
                                 or the SpiP eripheral - >SpiBus pointing at
                                 wrong bus,
                                 or the SpiP eripheral - >SpiPart is NULL

**/
EFI_STATUS
SpiBusUpdateSpiPeripheral (
  IN CONST EFI_SPI_IO_PROTOCOL  *This,
  IN CONST EFI_SPI_PERIPHERAL   *SpiPeripheral
  );

#endif  //  __SPI_BUS_DXE_H__
