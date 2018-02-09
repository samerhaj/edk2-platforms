/** @file
  This file defines the SPI Host Controller Protocol.

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

#ifndef __SPI_HC_PROTOCOL_H__
#define __SPI_HC_PROTOCOL_H__

#include <Pi/PiSpi.h>
#include <Protocol/SpiConfiguration.h>

///
/// Global ID for the SPI Host Controller Protocol
///
#define EFI_SPI_HOST_GUID  \
  { 0xc74e5db2, 0xfa96, 0x4ae2,   \
    { 0xb3, 0x99, 0x15, 0x97, 0x7f, 0xe3, 0x0, 0x2d }}

///
/// EDK2-style name
///
#define EFI_SPI_HC_PROTOCOL_GUID  EFI_SPI_HOST_GUID

typedef struct _EFI_SPI_HC_PROTOCOL EFI_SPI_HC_PROTOCOL;

/**
  Assert or deassert the SPI chip select.

  This routine is called at TPL_NOTIFY.
  Update the value of the chip select line for a SPI peripheral. The SPI bus
  layer calls this routine either in the board layer or in the SPI controller
  to manipulate the chip select pin at the start and end of a SPI transaction.

  @param[in] This           Pointer to an EFI_SPI_HC_PROTOCOL structure.
  @param[in] SpiPeripheral  The address of an EFI_SPI_PERIPHERAL data structure
                            describing the SPI peripheral whose chip select pin
                            is to be manipulated. The routine may access the
                            ChipSelectParameter field to gain sufficient
                            context to complete the operation.
  @param[in] PinValue       The value to be applied to the chip select line of
                            the SPI peripheral.

  @retval EFI_SUCCESS            The chip select was set as requested
  @retval EFI_NOT_READY          Support for the chip select is not properly
                                 initialized
  @retval EFI_INVALID_PARAMETER  The ChipSeLect value or its contents are
                                 invalid

**/
typedef EFI_STATUS
(EFIAPI *EFI_SPI_HC_PROTOCOL_CHIP_SELECT) (
  IN CONST EFI_SPI_HC_PROTOCOL  *This,
  IN CONST EFI_SPI_PERIPHERAL   *SpiPeripheral,
  IN BOOLEAN                    PinValue
  );

/**
  Set up the clock generator to produce the correct clock frequency, phase and
  polarity for a SPI chip.

  This routine is called at TPL_NOTIFY.
  This routine updates the clock generator to generate the correct frequency
  and polarity for the SPI clock.

  @param[in] This           Pointer to an EFI_SPI_HC_PROTOCOL structure.
  @param[in] SpiPeripheral  Pointer to a EFI_SPI_PERIPHERAL data structure from
                            which the routine can access the ClockParameter,
                            ClockPhase and ClockPolarity fields. The routine
                            also has access to the names for the SPI bus and
                            chip which can be used during debugging.
  @param[in] ClockHz        Pointer to the requested clock frequency. The SPI
                            host controller will choose a supported clock
                            frequency which is less then or equal to this
                            value. Specify zero to turn the clock generator
                            off. The actual clock frequency supported by the
                            SPI host controller will be returned.

  @retval EFI_SUCCESS      The clock was set up successfully
  @retval EFI_UNSUPPORTED  The SPI controller was not able to support the
                           frequency requested by ClockHz

**/
typedef EFI_STATUS
(EFIAPI *EFI_SPI_HC_PROTOCOL_CLOCK) (
  IN CONST EFI_SPI_HC_PROTOCOL  *This,
  IN CONST EFI_SPI_PERIPHERAL   *SpiPeripheral,
  IN UINT32                     *ClockHz
  );

/**
  Perform the SPI transaction on the SPI peripheral using the SPI host
  controller.

  This routine is called at TPL_NOTIFY.
  This routine synchronously returns EFI_SUCCESS indicating that the
  asynchronous SPI transaction was started. The routine then waits for
  completion of the SPI transaction prior to returning the final transaction
  status.

  @param[in] This            Pointer to an EFI_SPI_HC_PROTOCOL structure.
  @param[in] RequestPacket   Pointer to a EFI_SPI_REQUEST_PACKET containing
                             the description of the SPI transactions to perform.

  @retval EFI_SUCCESS             The transaction completed successfully.
  @retval EFI_ALREADY_STARTED     The controller is busy with another transaction.
  @retval EFI_BAD_BUFFER_SIZE     The Length value in SPI Transaction is wrong.
  @retval EFI_DEVICE_ERROR        There was an SPI error during the transaction.
  @retval EFI_INVALID_PARAMETER   The parameters specified in RequestPacket are not
                                  Valid. or the RequestPacket is NULL.
  @retval EFI_NO_RESPONSE         The SPI device is not responding to the slave
                                  address.  EFI_DEVICE_ERROR will be returned if
                                  the controller cannot distinguish when the NACK
                                  occurred.
  @retval EFI_UNSUPPORTED         The controller does not support the requested
                                  transaction.
**/
typedef EFI_STATUS
(EFIAPI *EFI_SPI_HC_PROTOCOL_TRANSACTION) (
  IN CONST EFI_SPI_HC_PROTOCOL  *This,
  IN EFI_SPI_REQUEST_PACKET     *RequestPacket
  );

///
/// Support a SPI data transaction between the SPI controller and a SPI chip.
///
struct _EFI_SPI_HC_PROTOCOL {
  ///
  /// Host control attributes, may have zero or more of the following set:
  /// * SPI_HALF_DUPLEX - The SPI Host Controller supports only half duplex
  ///   transactions
  /// * SPI_SUPPORTS_WRITE_ONLY_OPERATIONS - The SPI Host Controller supports
  ///   write only transactions.
  /// * SPI_SUPPORTS_READ_ONLY_OPERATIONS - The SPI Host Controller supports
  ///   read only transactions.
  /// * SPI_SUPPORTS_DTR_OPERATIONS - The SPI Host Controller supports
  ///   Double Transfer Rate (DTR).
  ///   DTR : Transfer may be input or output on both the
  ///   rising and falling edges of the clock.
  /// * SPI_SUPPORTS_2_B1T_DATA_BUS_W1DTH - The SPI Host Controller is wired to
  ///   support a 2-bit data bus
  /// * SPI_SUPPORTS_4_B1T_DATA_BUS_W1DTH - The SPI Host Controller is wired to
  ///   support a 4-bit data bus
  /// * SPI_SUPPORTS_8_B1T_DATA_BUS_W1DTH - The SPI Host Controller is wired to
  ///   support a 8-bit data bus
  /// The SPI host controller must support a 1 - bit bus
  /// width.
  /// * HC_TX_FRAME_IN_MOST_SIGNIFICANT_BITS
  ///   - The SPI host controller requires the transmit frame to be in most
  ///     significant bits instead of least significant bits.The host driver
  ///     will adjust the frames if necessary.
  /// * HC_RX_FRAME_IN_MOST_SIGNIFICANT_BITS
  ///   - The SPI host controller places the receive frame to be in most
  ///     significant bits instead of least significant bits.The host driver
  ///     will adjust the frames to be in the least significant bits if
  ///     necessary.
  /// * SPI_TRANSFER_SIZE_INCLUDES_OPCODE
  ///   - Transfer size includes the opcode byte
  /// * SPI_TRANSFER_SIZE_INCLUDES_ADDRESS
  ///   - Transfer size includes the 4 address bytes
  UINT32                          Attributes;

  ///
  /// Mask of frame sizes which the SPI host controller supports. Frame size of
  /// N-bits is supported when bit N-1 is set. The host controller must support
  /// a frame size of 8-bits.
  ///
  UINT32                          FrameSizeSupportMask;

  ///
  /// Maximum transfer size in bytes: 1 - Oxffffffff
  ///
  UINT32                          MaximumTransferBytes;

  ///
  /// Assert or deassert the SPI chip select.
  ///
  EFI_SPI_HC_PROTOCOL_CHIP_SELECT           ChipSelect;

  ///
  /// Set up the clock generator to produce the correct clock frequency, phase
  /// and polarity for a SPI chip.
  ///
  EFI_SPI_HC_PROTOCOL_CLOCK                 Clock;

  ///
  /// Perform the SPI transaction on the SPI peripheral using the SPI host
  /// controller.
  ///
  EFI_SPI_HC_PROTOCOL_TRANSACTION           Transaction;
};

extern EFI_GUID gEfiSpiHcProtocolGuid;

#endif // __SPI_HC_PROTOCOL_H__
