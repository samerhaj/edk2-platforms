/** @file
  This file defines the SPI I/O Protocol.

  Copyright (c) 2017, Intel Corporation. All rights reserved.<BR>
  Copyright 2017-2018 NXP

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD
  License which accompanies this distribution. The full text of the license may
  be found at http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

  @par Revision Reference:
    This Protocol was introduced in revised UEFI PI Specification 1.6.

**/

#ifndef __SPI_IO_PROTOCOL_H__
#define __SPI_IO_PROTOCOL_H__

#include <Pi/PiSpi.h>
#include <Protocol/LegacySpiController.h>
#include <Protocol/SpiConfiguration.h>

typedef struct _EFI_SPI_IO_PROTOCOL EFI_SPI_IO_PROTOCOL;

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
typedef
EFI_STATUS
(EFIAPI *EFI_SPI_IO_PROTOCOL_TRANSACTION) (
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
typedef EFI_STATUS
(EFIAPI *EFI_SPI_IO_PROTOCOL_UPDATE_SPI_PERIPHERAL) (
  IN CONST EFI_SPI_IO_PROTOCOL  *This,
  IN CONST EFI_SPI_PERIPHERAL   *SpiPeripheral
  );

///
/// Support managed SPI data transactions between the SPI controller and a SPI
/// chip.
///
struct _EFI_SPI_IO_PROTOCOL {
  ///
  /// Address of an EFI_SPI_PERIPHERAL data structure associated with this
  /// protocol instance.
  ///
  CONST EFI_SPI_PERIPHERAL                  *SpiPeripheral;

  ///
  /// Address of the original EFI_SPI_PERIPHERAL data structure associated with
  /// this protocol instance.
  ///
  CONST EFI_SPI_PERIPHERAL                  *OriginalSpiPeripheral;

  ///
  /// Mask of frame sizes which the SPI 10 layer supports. Frame size of N-bits
  /// is supported when bit N-1 is set. The host controller must support a
  /// frame size of 8-bits. Frame sizes of 16, 24 and 32-bits are converted to
  /// 8-bit frame sizes by the SPI bus layer if the frame size is not supported
  /// by the SPI host controller.
  ///
  UINT32                                    FrameSizeSupportMask;

  ///
  /// Maximum transfer size in bytes: 1 - Oxffffffff
  ///
  UINT32                                    MaximumTransferBytes;

  ///
  /// Transaction attributes: One or more from:
  /// * SPI_HALF_DUPLEX
  ///   - The SPI host or peripheral supports only half duplex transactions
  /// * SPI_SUPPORTS_WRITE_ONLY_OPERATIONS
  ///   - The SPI host or peripheral supports write only transactions.
  /// * SPI_SUPPORTS_READ_ONLY_OPERATIONS
  ///   - The SPI host or peripheral support read only transactions.
  /// * SPI_SUPPORTS_DTR_OPERATIONS
  ///   - The SPI SPI host and peripheral supports Double Transfer Rate (DTR).
  ///     DTR : Transfer may be input or output on both the
  ///     rising and falling edges of the clock.
  /// * SPI_SUPPORTS_2_B1T_DATA_BUS_W1DTH
  ///   - The SPI host and peripheral supports a 2-bit data bus
  /// * SPI_SUPPORTS_4_BIT_DATA_BUS_W1DTH
  ///   - The SPI host and peripheral supports a 4-bit data bus
  /// * SPI_SUPPORTS_8_BIT_DATA_BUS_W1DTH
  ///   - The SPI host and peripheral supports a 8-bit data bus
  /// * SPI_TRANSFER_SIZE_INCLUDES_OPCODE
  ///   - Transfer size includes the opcode byte
  /// * SPI_TRANSFER_SIZE_INCLUDES_ADDRESS
  ///   - Transfer size includes the 4 address bytes
  UINT32                                    Attributes;

  ///
  /// Pointer to legacy SPI controller protocol
  ///
  CONST EFI_LEGACY_SPI_CONTROLLER_PROTOCOL  *LegacySpiProtocol;

  ///
  /// Initiate a SPI transaction between the host and a SPI peripheral.
  ///
  EFI_SPI_IO_PROTOCOL_TRANSACTION           Transaction;

  ///
  /// Update the SPI peripheral associated with this SPI 10 instance.
  ///
  EFI_SPI_IO_PROTOCOL_UPDATE_SPI_PERIPHERAL UpdateSpiPeripheral;
};

#endif // __SPI_IO_PROTOCOL_H__
